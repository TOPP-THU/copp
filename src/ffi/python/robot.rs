//! Python wrappers for robot and constraint-buffer construction.
//!
//! The Rust [`Robot`](crate::robot::Robot) type is generic over a robot model.
//! Python exposes one owning [`PyRobot`] class backed by [`PyRobotModel`], plus
//! a lightweight [`PyConstraints`] proxy. A [`PyConstraints`] value can either
//! be returned from `robot.constraints` or independently constructed around a
//! hidden point-mass robot when only raw TOPP constraints are needed.

use crate::copp::constraints::ModePopConstraints;
use crate::diag::{CoppError, RobotDynamicsError};
use crate::ffi::python::array::{array_like_to_vec_f64, array_like_to_vec2_f64};
use crate::ffi::python::error::to_py_err;
use crate::ffi::python::path::{PyMatrixLayout, PyPath};
use crate::robot::{Robot as RustRobot, RobotBasic, RobotTorque};
use nalgebra::DMatrix;
use numpy::{PyArray1, PyReadonlyArray1};
use pyo3::Borrowed;
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyAnyMethods;
use std::sync::{Arc, Mutex, MutexGuard};

/// Shared robot state used by [`PyRobot`] and [`PyConstraints`] proxies.
pub(crate) type SharedRobot = Arc<Mutex<RustRobot<PyRobotModel>>>;

/// Register robot-related classes on the native [`PyModule`].
pub(crate) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyRobot>()?;
    m.add_class::<PyConstraints>()?;
    Ok(())
}

/// Internal parser for `MatrixLayout | str | None` robot arguments.
#[derive(Clone, Copy)]
struct MatrixLayoutArg(
    /// Parsed matrix layout for Python robot inputs.
    PyMatrixLayout,
);

impl FromPyObject<'_, '_> for MatrixLayoutArg {
    /// [`PyErr`] returned when parsing fails.
    type Error = PyErr;

    /// Parse enum values, accepted strings, or `None` into a matrix layout.
    fn extract(obj: Borrowed<'_, '_, PyAny>) -> Result<Self, Self::Error> {
        if obj.is_none() {
            return Ok(Self(PyMatrixLayout::SampleMajor));
        }

        if let Ok(layout) = obj.extract::<PyMatrixLayout>() {
            return Ok(Self(layout));
        }

        if let Ok(text) = obj.extract::<&str>() {
            return match normalize_token(text).as_str() {
                "sample_major" | "samplemajor" => Ok(Self(PyMatrixLayout::SampleMajor)),
                "dim_major" | "dimmajor" => Ok(Self(PyMatrixLayout::DimMajor)),
                _ => Err(PyValueError::new_err(
                    "`layout` must be MatrixLayout.SAMPLE_MAJOR, MatrixLayout.DIM_MAJOR, \"sample_major\", or \"dim_major\"",
                )),
            };
        }

        Err(PyValueError::new_err(
            "`layout` must be a MatrixLayout value or a string",
        ))
    }
}

/// Python-owned robot wrapper backed by [`RustRobot`].
#[pyclass(name = "Robot", module = "copp_py._native")]
pub(crate) struct PyRobot {
    /// Shared robot model and constraint storage.
    inner: SharedRobot,
}

#[pymethods]
impl PyRobot {
    /// Construct a robot wrapper.
    ///
    /// When ``inverse_dynamics`` is ``None``, torque evaluation uses point-mass
    /// dynamics, i.e. ``tau = ddq``. Passing a callable or object with an
    /// ``inverse_dynamics(q, dq, ddq)`` method enables Python inverse dynamics.
    #[new]
    #[pyo3(
        signature = (dim, *, capacity = None, inverse_dynamics = None),
        text_signature = "(dim, *, capacity=None, inverse_dynamics=None)"
    )]
    fn new(
        py: Python<'_>,
        dim: usize,
        capacity: Option<usize>,
        inverse_dynamics: Option<Py<PyAny>>,
    ) -> PyResult<Self> {
        Self::from_parts(py, dim, capacity, inverse_dynamics)
    }

    /// Construct an explicit point-mass robot, where ``tau = ddq``.
    #[staticmethod]
    #[pyo3(
        signature = (dim, *, capacity = None),
        text_signature = "(dim, *, capacity=None)"
    )]
    fn point_mass(py: Python<'_>, dim: usize, capacity: Option<usize>) -> PyResult<Self> {
        Self::from_parts(py, dim, capacity, None)
    }

    /// Return the robot dimension.
    #[getter]
    fn dim(&self) -> PyResult<usize> {
        Ok(lock_robot(&self.inner)?.dim())
    }

    /// Return the number of station samples currently stored.
    #[getter]
    fn len(&self) -> PyResult<usize> {
        Ok(lock_robot(&self.inner)?.constraints.len())
    }

    /// Return the allocated station-buffer capacity.
    #[getter]
    fn capacity(&self) -> PyResult<usize> {
        Ok(lock_robot(&self.inner)?.constraints.capacity())
    }

    /// Return whether the constraint buffer contains no stations.
    #[getter]
    fn is_empty(&self) -> PyResult<bool> {
        Ok(lock_robot(&self.inner)?.constraints.is_empty())
    }

    /// Return the active global station-id range ``(idx_s_start, idx_s_end)``.
    #[getter]
    fn idx_s_range(&self) -> PyResult<(usize, usize)> {
        let robot = lock_robot(&self.inner)?;
        Ok((
            robot.constraints.idx_s_start(),
            robot.constraints.idx_s_end(),
        ))
    }

    /// Return whether a Python inverse-dynamics callback is installed.
    #[getter]
    fn has_inverse_dynamics(&self) -> PyResult<bool> {
        Ok(lock_robot(&self.inner)?.model().has_inverse_dynamics())
    }

    /// Return a lightweight proxy for raw constraint-buffer operations.
    #[getter]
    fn constraints(&self) -> PyConstraints {
        PyConstraints {
            inner: Arc::clone(&self.inner),
        }
    }

    /// Return ``len(robot)`` as the current number of station samples.
    fn __len__(&self) -> PyResult<usize> {
        self.len()
    }

    /// Return a compact representation for interactive Python sessions.
    fn __repr__(&self) -> PyResult<String> {
        let robot = lock_robot(&self.inner)?;
        Ok(format!(
            "Robot(dim={}, len={}, capacity={}, has_inverse_dynamics={})",
            robot.dim(),
            robot.constraints.len(),
            robot.constraints.capacity(),
            robot.model().has_inverse_dynamics()
        ))
    }

    /// Replace the Python inverse-dynamics callback.
    ///
    /// The callback may be a plain callable ``f(q, dq, ddq)`` or an object with
    /// a callable ``inverse_dynamics(q, dq, ddq)`` method. It must return a
    /// vector convertible to contiguous ``float64`` with shape ``(dim,)``.
    fn set_inverse_dynamics(&self, py: Python<'_>, inverse_dynamics: Py<PyAny>) -> PyResult<()> {
        validate_inverse_dynamics(py, &inverse_dynamics)?;
        lock_robot(&self.inner)?
            .model_mut()
            .set_inverse_dynamics(Some(inverse_dynamics));
        Ok(())
    }

    /// Clear the Python inverse-dynamics callback and restore point dynamics.
    fn clear_inverse_dynamics(&self) -> PyResult<()> {
        lock_robot(&self.inner)?
            .model_mut()
            .set_inverse_dynamics(None);
        Ok(())
    }

    /// Append strictly increasing station samples to the robot.
    fn append_s(&self, s: &Bound<'_, PyAny>) -> PyResult<()> {
        let s = array_like_to_vec_f64("s", s)?;
        let mut robot = lock_robot(&self.inner)?;
        robot
            .with_s(&s)
            .map(|_| ())
            .map_err(|error| to_py_err(error.into()))
    }

    /// Store discrete path derivatives over an existing station interval.
    #[pyo3(
        signature = (q, dq, ddq, idx_s, *, dddq = None, layout = MatrixLayoutArg(PyMatrixLayout::SampleMajor)),
        text_signature = "(q, dq, ddq, idx_s, *, dddq=None, layout='sample_major')"
    )]
    fn set_q(
        &self,
        q: &Bound<'_, PyAny>,
        dq: &Bound<'_, PyAny>,
        ddq: &Bound<'_, PyAny>,
        idx_s: usize,
        dddq: Option<&Bound<'_, PyAny>>,
        layout: MatrixLayoutArg,
    ) -> PyResult<()> {
        let dim = lock_robot(&self.inner)?.dim();
        let q = matrix_to_dmatrix("q", q, layout.0, Some(dim))?;
        let dq = matrix_to_dmatrix("dq", dq, layout.0, Some(dim))?;
        let ddq = matrix_to_dmatrix("ddq", ddq, layout.0, Some(dim))?;
        let dddq = dddq
            .map(|array| matrix_to_dmatrix("dddq", array, layout.0, Some(dim)))
            .transpose()?;

        let dddq_view = dddq.as_ref().map(|matrix| matrix.as_view());
        let mut robot = lock_robot(&self.inner)?;
        robot
            .with_q(
                &q.as_view(),
                &dq.as_view(),
                &ddq.as_view(),
                dddq_view.as_ref(),
                idx_s,
            )
            .map(|_| ())
            .map_err(|error| to_py_err(error.into()))
    }

    /// Sample a path and store derivatives up to second order.
    fn set_q_from_path_2nd(
        &self,
        path: PyRef<'_, PyPath>,
        idx_s_from: usize,
        idx_s_to: usize,
    ) -> PyResult<()> {
        let s = {
            let robot = lock_robot(&self.inner)?;
            robot
                .constraints
                .s_vec(idx_s_from, idx_s_to)
                .map_err(|error| to_py_err(error.into()))?
        };
        let derivs = path.evaluate_up_to_2nd_rust(&s)?;
        let mut robot = lock_robot(&self.inner)?;
        robot
            .with_q(
                &derivs.q.as_view(),
                &derivs.dq.as_ref().expect("dq must exist").as_view(),
                &derivs.ddq.as_ref().expect("ddq must exist").as_view(),
                None,
                idx_s_from,
            )
            .map(|_| ())
            .map_err(|error| to_py_err(error.into()))
    }

    /// Sample a path and store derivatives up to third order.
    fn set_q_from_path_3rd(
        &self,
        path: PyRef<'_, PyPath>,
        idx_s_from: usize,
        idx_s_to: usize,
    ) -> PyResult<()> {
        let s = {
            let robot = lock_robot(&self.inner)?;
            robot
                .constraints
                .s_vec(idx_s_from, idx_s_to)
                .map_err(|error| to_py_err(error.into()))?
        };
        let derivs = path.evaluate_up_to_3rd_rust(&s)?;
        let dddq_view = derivs.dddq.as_ref().map(|matrix| matrix.as_view());
        let mut robot = lock_robot(&self.inner)?;
        robot
            .with_q(
                &derivs.q.as_view(),
                &derivs.dq.as_ref().expect("dq must exist").as_view(),
                &derivs.ddq.as_ref().expect("ddq must exist").as_view(),
                dddq_view.as_ref(),
                idx_s_from,
            )
            .map(|_| ())
            .map_err(|error| to_py_err(error.into()))
    }

    /// Add axial velocity limits over an existing station interval.
    #[pyo3(
        signature = (upper, lower, *, start_idx_s, length = None, layout = MatrixLayoutArg(PyMatrixLayout::SampleMajor)),
        text_signature = "(upper, lower, *, start_idx_s, length=None, layout='sample_major')"
    )]
    fn add_velocity_limits(
        &self,
        upper: &Bound<'_, PyAny>,
        lower: &Bound<'_, PyAny>,
        start_idx_s: usize,
        length: Option<usize>,
        layout: MatrixLayoutArg,
    ) -> PyResult<()> {
        self.add_limits(
            upper,
            lower,
            start_idx_s,
            length,
            layout.0,
            AxialLimitKind::Velocity,
        )
    }

    /// Add axial acceleration limits over an existing station interval.
    #[pyo3(
        signature = (upper, lower, *, start_idx_s, length = None, layout = MatrixLayoutArg(PyMatrixLayout::SampleMajor)),
        text_signature = "(upper, lower, *, start_idx_s, length=None, layout='sample_major')"
    )]
    fn add_acceleration_limits(
        &self,
        upper: &Bound<'_, PyAny>,
        lower: &Bound<'_, PyAny>,
        start_idx_s: usize,
        length: Option<usize>,
        layout: MatrixLayoutArg,
    ) -> PyResult<()> {
        self.add_limits(
            upper,
            lower,
            start_idx_s,
            length,
            layout.0,
            AxialLimitKind::Acceleration,
        )
    }

    /// Add axial jerk limits over an existing station interval.
    #[pyo3(
        signature = (upper, lower, *, start_idx_s, length = None, layout = MatrixLayoutArg(PyMatrixLayout::SampleMajor)),
        text_signature = "(upper, lower, *, start_idx_s, length=None, layout='sample_major')"
    )]
    fn add_jerk_limits(
        &self,
        upper: &Bound<'_, PyAny>,
        lower: &Bound<'_, PyAny>,
        start_idx_s: usize,
        length: Option<usize>,
        layout: MatrixLayoutArg,
    ) -> PyResult<()> {
        self.add_limits(
            upper,
            lower,
            start_idx_s,
            length,
            layout.0,
            AxialLimitKind::Jerk,
        )
    }

    /// Add axial torque limits using point or Python inverse dynamics.
    #[pyo3(
        signature = (upper, lower, *, start_idx_s, length = None, layout = MatrixLayoutArg(PyMatrixLayout::SampleMajor)),
        text_signature = "(upper, lower, *, start_idx_s, length=None, layout='sample_major')"
    )]
    fn add_torque_limits(
        &self,
        upper: &Bound<'_, PyAny>,
        lower: &Bound<'_, PyAny>,
        start_idx_s: usize,
        length: Option<usize>,
        layout: MatrixLayoutArg,
    ) -> PyResult<()> {
        self.add_limits(
            upper,
            lower,
            start_idx_s,
            length,
            layout.0,
            AxialLimitKind::Torque,
        )
    }
}

impl PyRobot {
    /// Clone the shared robot handle for solver descriptors.
    pub(crate) fn shared_robot(&self) -> SharedRobot {
        Arc::clone(&self.inner)
    }

    /// Construct a [`PyRobot`] from validated constructor parts.
    fn from_parts(
        py: Python<'_>,
        dim: usize,
        capacity: Option<usize>,
        inverse_dynamics: Option<Py<PyAny>>,
    ) -> PyResult<Self> {
        if dim == 0 {
            return Err(PyValueError::new_err("`dim` must be a positive integer"));
        }
        if capacity.is_some_and(|capacity| capacity == 0) {
            return Err(PyValueError::new_err(
                "`capacity` must be a positive integer or None",
            ));
        }
        if let Some(callback) = inverse_dynamics.as_ref() {
            validate_inverse_dynamics(py, callback)?;
        }

        let model = PyRobotModel::new(dim, inverse_dynamics);
        let robot = match capacity {
            Some(capacity) => RustRobot::with_capacity(model, capacity),
            None => RustRobot::new(model),
        };

        Ok(Self {
            inner: Arc::new(Mutex::new(robot)),
        })
    }

    /// Add one family of axial limits.
    fn add_limits(
        &self,
        upper: &Bound<'_, PyAny>,
        lower: &Bound<'_, PyAny>,
        start_idx_s: usize,
        length: Option<usize>,
        layout: PyMatrixLayout,
        kind: AxialLimitKind,
    ) -> PyResult<()> {
        let mut robot = lock_robot(&self.inner)?;
        let default_length = inferred_station_length(&robot, start_idx_s);
        let upper = limit_to_dmatrix("upper", upper, robot.dim(), default_length, length, layout)?;
        let lower = limit_to_dmatrix("lower", lower, robot.dim(), default_length, length, layout)?;

        match kind {
            AxialLimitKind::Velocity => robot
                .with_axial_velocity(&upper.as_view(), &lower.as_view(), start_idx_s)
                .map(|_| ())
                .map_err(|error| to_py_err(error.into())),
            AxialLimitKind::Acceleration => robot
                .with_axial_acceleration(&upper.as_view(), &lower.as_view(), start_idx_s)
                .map(|_| ())
                .map_err(|error| to_py_err(error.into())),
            AxialLimitKind::Jerk => robot
                .with_axial_jerk(&upper.as_view(), &lower.as_view(), start_idx_s)
                .map(|_| ())
                .map_err(|error| to_py_err(error.into())),
            AxialLimitKind::Torque => {
                robot.model().clear_pending_error();
                let result = robot
                    .with_axial_torque(&upper.as_view(), &lower.as_view(), start_idx_s)
                    .map(|_| ());
                map_robot_copp_result(&robot, result)
            }
        }
    }
}

/// Raw constraint-buffer proxy returned by [`PyRobot::constraints`].
#[pyclass(name = "Constraints", module = "copp_py._native")]
pub(crate) struct PyConstraints {
    /// Shared robot state whose constraint buffer is mutated by this proxy.
    inner: SharedRobot,
}

impl PyConstraints {
    /// Clone the shared robot handle for solver descriptors.
    pub(crate) fn shared_robot(&self) -> SharedRobot {
        Arc::clone(&self.inner)
    }
}

#[pymethods]
impl PyConstraints {
    /// Construct an independent raw constraint buffer.
    ///
    /// This constructor is intended for TOPP-only workflows that do not need a
    /// full Python [`PyRobot`] object. Internally it owns a hidden point-mass
    /// robot so the same storage and validation path is shared with
    /// `robot.constraints`.
    #[new]
    #[pyo3(
        signature = (dim, *, capacity = None),
        text_signature = "(dim, *, capacity=None)"
    )]
    fn new(py: Python<'_>, dim: usize, capacity: Option<usize>) -> PyResult<Self> {
        let robot = PyRobot::from_parts(py, dim, capacity, None)?;
        Ok(Self { inner: robot.inner })
    }

    /// Return the robot/path dimension.
    #[getter]
    fn dim(&self) -> PyResult<usize> {
        Ok(lock_robot(&self.inner)?.dim())
    }

    /// Return the number of station samples currently stored.
    #[getter]
    fn len(&self) -> PyResult<usize> {
        Ok(lock_robot(&self.inner)?.constraints.len())
    }

    /// Return the allocated station-buffer capacity.
    #[getter]
    fn capacity(&self) -> PyResult<usize> {
        Ok(lock_robot(&self.inner)?.constraints.capacity())
    }

    /// Return whether the constraint buffer contains no stations.
    #[getter]
    fn is_empty(&self) -> PyResult<bool> {
        Ok(lock_robot(&self.inner)?.constraints.is_empty())
    }

    /// Return the active global station-id range ``(idx_s_start, idx_s_end)``.
    #[getter]
    fn idx_s_range(&self) -> PyResult<(usize, usize)> {
        let robot = lock_robot(&self.inner)?;
        Ok((
            robot.constraints.idx_s_start(),
            robot.constraints.idx_s_end(),
        ))
    }

    /// Return a compact proxy representation.
    fn __repr__(&self) -> PyResult<String> {
        let robot = lock_robot(&self.inner)?;
        Ok(format!(
            "Constraints(dim={}, len={}, capacity={})",
            robot.dim(),
            robot.constraints.len(),
            robot.constraints.capacity()
        ))
    }

    /// Append strictly increasing station samples to the constraint buffer.
    fn append_s(&self, s: &Bound<'_, PyAny>) -> PyResult<()> {
        let s = array_like_to_vec_f64("s", s)?;
        lock_robot(&self.inner)?
            .constraints
            .with_s(&s)
            .map(|_| ())
            .map_err(|error| to_py_err(error.into()))
    }

    /// Export station samples over a half-open station-id interval.
    #[pyo3(
        signature = (idx_s_from = None, idx_s_to = None),
        text_signature = "(idx_s_from=None, idx_s_to=None)"
    )]
    fn s_values<'py>(
        &self,
        py: Python<'py>,
        idx_s_from: Option<usize>,
        idx_s_to: Option<usize>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let robot = lock_robot(&self.inner)?;
        let (idx_s_from, idx_s_to) = default_interval(&robot, idx_s_from, idx_s_to);
        let values = robot
            .constraints
            .s_vec(idx_s_from, idx_s_to)
            .map_err(|error| to_py_err(error.into()))?;
        Ok(PyArray1::from_vec(py, values))
    }

    /// Export first-order upper bounds over a half-open station-id interval.
    #[pyo3(
        signature = (idx_s_from = None, idx_s_to = None),
        text_signature = "(idx_s_from=None, idx_s_to=None)"
    )]
    fn amax_values<'py>(
        &self,
        py: Python<'py>,
        idx_s_from: Option<usize>,
        idx_s_to: Option<usize>,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let robot = lock_robot(&self.inner)?;
        let (idx_s_from, idx_s_to) = default_interval(&robot, idx_s_from, idx_s_to);
        let values = robot
            .constraints
            .amax_vec(idx_s_from, idx_s_to)
            .map_err(|error| to_py_err(error.into()))?;
        Ok(PyArray1::from_vec(py, values))
    }

    /// Overwrite first-order bounds over a station interval.
    fn amax_substitute(&self, amax: &Bound<'_, PyAny>, idx_s: usize) -> PyResult<()> {
        let amax = array_like_to_vec_f64("amax", amax)?;
        lock_robot(&self.inner)?
            .constraints
            .amax_substitute(&amax, idx_s)
            .map_err(|error| to_py_err(error.into()))
    }

    /// Clear all stored constraints and station data.
    #[pyo3(signature = (*, keep_idx_s = false), text_signature = "(*, keep_idx_s=False)")]
    fn clear(&self, keep_idx_s: bool) -> PyResult<()> {
        lock_robot(&self.inner)?.constraints.clear(keep_idx_s);
        Ok(())
    }

    /// Remove ``n_cols`` station samples from the front.
    fn pop_front_n(&self, n_cols: usize) -> PyResult<()> {
        lock_robot(&self.inner)?
            .constraints
            .pop_front(ModePopConstraints::PopNCols(n_cols));
        Ok(())
    }

    /// Remove ``n_cols`` station samples from the back.
    fn pop_back_n(&self, n_cols: usize) -> PyResult<()> {
        lock_robot(&self.inner)?
            .constraints
            .pop_back(ModePopConstraints::PopNCols(n_cols));
        Ok(())
    }

    /// Remove front samples until the kept window starts at ``idx_s_cut``.
    fn pop_front_until(&self, idx_s_cut: usize) -> PyResult<()> {
        lock_robot(&self.inner)?
            .constraints
            .pop_front(ModePopConstraints::CutAtIdxS(idx_s_cut));
        Ok(())
    }

    /// Remove back samples until the kept window ends before ``idx_s_cut``.
    fn pop_back_until(&self, idx_s_cut: usize) -> PyResult<()> {
        lock_robot(&self.inner)?
            .constraints
            .pop_back(ModePopConstraints::CutAtIdxS(idx_s_cut));
        Ok(())
    }

    /// Add or tighten first-order raw constraints.
    #[pyo3(
        signature = (amax, idx_s, *, layout = MatrixLayoutArg(PyMatrixLayout::SampleMajor)),
        text_signature = "(amax, idx_s, *, layout='sample_major')"
    )]
    fn add_constraint_1st(
        &self,
        amax: &Bound<'_, PyAny>,
        idx_s: usize,
        layout: MatrixLayoutArg,
    ) -> PyResult<()> {
        let amax = matrix_to_dmatrix("amax", amax, layout.0, None)?;
        lock_robot(&self.inner)?
            .constraints
            .with_constraint_1order(&amax.as_view(), idx_s)
            .map(|_| ())
            .map_err(|error| to_py_err(error.into()))
    }

    /// Add second-order raw constraints.
    #[pyo3(
        signature = (acc_a, acc_b, acc_max, idx_s, *, is_negative = false, layout = MatrixLayoutArg(PyMatrixLayout::SampleMajor)),
        text_signature = "(acc_a, acc_b, acc_max, idx_s, *, is_negative=False, layout='sample_major')"
    )]
    fn add_constraint_2nd(
        &self,
        acc_a: &Bound<'_, PyAny>,
        acc_b: &Bound<'_, PyAny>,
        acc_max: &Bound<'_, PyAny>,
        idx_s: usize,
        is_negative: bool,
        layout: MatrixLayoutArg,
    ) -> PyResult<()> {
        let acc_a = matrix_to_dmatrix("acc_a", acc_a, layout.0, None)?;
        let acc_b = matrix_to_dmatrix("acc_b", acc_b, layout.0, None)?;
        let acc_max = matrix_to_dmatrix("acc_max", acc_max, layout.0, None)?;
        lock_robot(&self.inner)?
            .constraints
            .with_constraint_2order(
                &acc_a.as_view(),
                &acc_b.as_view(),
                &acc_max.as_view(),
                idx_s,
                is_negative,
            )
            .map(|_| ())
            .map_err(|error| to_py_err(error.into()))
    }

    /// Add third-order raw constraints.
    #[pyo3(
        signature = (jerk_a, jerk_b, jerk_c, jerk_d, jerk_max, idx_s, *, is_negative = false, layout = MatrixLayoutArg(PyMatrixLayout::SampleMajor)),
        text_signature = "(jerk_a, jerk_b, jerk_c, jerk_d, jerk_max, idx_s, *, is_negative=False, layout='sample_major')"
    )]
    fn add_constraint_3rd(
        &self,
        jerk_a: &Bound<'_, PyAny>,
        jerk_b: &Bound<'_, PyAny>,
        jerk_c: &Bound<'_, PyAny>,
        jerk_d: &Bound<'_, PyAny>,
        jerk_max: &Bound<'_, PyAny>,
        idx_s: usize,
        is_negative: bool,
        layout: MatrixLayoutArg,
    ) -> PyResult<()> {
        let jerk_a = matrix_to_dmatrix("jerk_a", jerk_a, layout.0, None)?;
        let jerk_b = matrix_to_dmatrix("jerk_b", jerk_b, layout.0, None)?;
        let jerk_c = matrix_to_dmatrix("jerk_c", jerk_c, layout.0, None)?;
        let jerk_d = matrix_to_dmatrix("jerk_d", jerk_d, layout.0, None)?;
        let jerk_max = matrix_to_dmatrix("jerk_max", jerk_max, layout.0, None)?;
        lock_robot(&self.inner)?
            .constraints
            .with_constraint_3order(
                &jerk_a.as_view(),
                &jerk_b.as_view(),
                &jerk_c.as_view(),
                &jerk_d.as_view(),
                &jerk_max.as_view(),
                idx_s,
                is_negative,
            )
            .map(|_| ())
            .map_err(|error| to_py_err(error.into()))
    }
}

/// Python-capable robot model implementing [`RobotBasic`] and [`RobotTorque`].
pub(crate) struct PyRobotModel {
    /// Robot/path dimension.
    dim: usize,
    /// Optional Python inverse-dynamics callable or protocol object.
    inverse_dynamics: Option<Py<PyAny>>,
    /// Pending Python exception raised inside [`RobotTorque::inverse_dynamics`].
    pending_error: Mutex<Option<PyErr>>,
}

impl PyRobotModel {
    /// Construct a model with optional Python inverse dynamics.
    fn new(dim: usize, inverse_dynamics: Option<Py<PyAny>>) -> Self {
        Self {
            dim,
            inverse_dynamics,
            pending_error: Mutex::new(None),
        }
    }

    /// Return whether a Python dynamics callback is installed.
    fn has_inverse_dynamics(&self) -> bool {
        self.inverse_dynamics.is_some()
    }

    /// Replace the Python inverse-dynamics callback.
    fn set_inverse_dynamics(&mut self, inverse_dynamics: Option<Py<PyAny>>) {
        self.inverse_dynamics = inverse_dynamics;
    }

    /// Store a Python exception and return a Rust dynamics error sentinel.
    fn store_error(&self, error: PyErr) -> RobotDynamicsError {
        *self
            .pending_error
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner()) = Some(error);
        RobotDynamicsError::new("python inverse_dynamics callback failed")
    }

    /// Clear any stale Python callback exception.
    fn clear_pending_error(&self) {
        self.pending_error
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .take();
    }

    /// Take the pending Python callback exception, if present.
    fn take_pending_error(&self) -> Option<PyErr> {
        self.pending_error
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .take()
    }
}

impl RobotBasic for PyRobotModel {
    /// Return robot/path dimension.
    fn dim(&self) -> usize {
        self.dim
    }
}

impl RobotTorque for PyRobotModel {
    /// Evaluate point dynamics or delegate to a Python inverse-dynamics callback.
    fn inverse_dynamics(
        &self,
        q: &[f64],
        dq: &[f64],
        ddq: &[f64],
        tau: &mut [f64],
    ) -> Result<(), RobotDynamicsError> {
        let Some(callback) = &self.inverse_dynamics else {
            tau.copy_from_slice(ddq);
            return Ok(());
        };

        Python::attach(|py| -> PyResult<()> {
            let q = PyArray1::from_slice(py, q);
            let dq = PyArray1::from_slice(py, dq);
            let ddq = PyArray1::from_slice(py, ddq);
            let callback = callback.bind(py);
            let result = if callback.is_callable() {
                callback.call1((&q, &dq, &ddq))?
            } else {
                callback.call_method1("inverse_dynamics", (&q, &dq, &ddq))?
            };

            let numpy = py.import("numpy")?;
            let dtype = numpy.getattr("float64")?;
            let array_obj = numpy
                .getattr("ascontiguousarray")?
                .call1((&result, dtype))?;
            let array = array_obj.extract::<PyReadonlyArray1<'_, f64>>()?;
            let values = array.as_slice().map_err(|_| {
                PyValueError::new_err(
                    "`inverse_dynamics` must return a contiguous one-dimensional float64 array",
                )
            })?;
            if values.len() != self.dim {
                return Err(PyValueError::new_err(format!(
                    "`inverse_dynamics` returned {} values; expected {}",
                    values.len(),
                    self.dim
                )));
            }
            tau.copy_from_slice(values);
            Ok(())
        })
        .map_err(|error| self.store_error(error))
    }
}

/// Axial limit family selected by Python convenience methods.
enum AxialLimitKind {
    /// Velocity limits.
    Velocity,
    /// Acceleration limits.
    Acceleration,
    /// Jerk limits.
    Jerk,
    /// Torque limits.
    Torque,
}

/// Validate an inverse-dynamics callback or protocol object.
fn validate_inverse_dynamics(py: Python<'_>, callback: &Py<PyAny>) -> PyResult<()> {
    let callback = callback.bind(py);
    if callback.is_callable() {
        return Ok(());
    }
    if callback.hasattr("inverse_dynamics")? && callback.getattr("inverse_dynamics")?.is_callable()
    {
        return Ok(());
    }
    Err(PyValueError::new_err(
        "`inverse_dynamics` must be a callable or define an `inverse_dynamics(q, dq, ddq)` method",
    ))
}

/// Lock shared robot state with a Python-facing poisoning error.
fn lock_robot(shared: &SharedRobot) -> PyResult<MutexGuard<'_, RustRobot<PyRobotModel>>> {
    shared
        .lock()
        .map_err(|_| PyRuntimeError::new_err("internal robot state lock was poisoned"))
}

/// Return whether a shared robot has a Python inverse-dynamics callback.
pub(crate) fn shared_has_inverse_dynamics(shared: &SharedRobot) -> PyResult<bool> {
    Ok(lock_robot(shared)?.model().has_inverse_dynamics())
}

/// Borrow a shared Rust robot while holding its lock.
///
/// Solver wrappers call this directly from a detached Python thread when the
/// robot has no Python callback installed. If a callback is installed, callers
/// keep the GIL and this function preserves any pending Python exception.
pub(crate) fn with_shared_robot_result<R>(
    shared: &SharedRobot,
    f: impl FnOnce(&RustRobot<PyRobotModel>) -> Result<R, CoppError>,
) -> PyResult<R> {
    let robot = lock_robot(shared)?;
    robot.model().clear_pending_error();
    let result = f(&robot);
    map_robot_copp_result(&robot, result)
}

/// Mutably borrow a shared Rust robot while holding its lock.
pub(crate) fn with_shared_robot_mut_result<R>(
    shared: &SharedRobot,
    f: impl FnOnce(&mut RustRobot<PyRobotModel>) -> Result<R, CoppError>,
) -> PyResult<R> {
    let mut robot = lock_robot(shared)?;
    robot.model().clear_pending_error();
    let result = f(&mut robot);
    map_robot_copp_result(&robot, result)
}

/// Borrow shared raw constraints while holding their robot lock.
pub(crate) fn with_shared_constraints_result<R>(
    shared: &SharedRobot,
    f: impl FnOnce(&crate::copp::constraints::Constraints) -> PyResult<R>,
) -> PyResult<R> {
    let robot = lock_robot(shared)?;
    f(&robot.constraints)
}

/// Mutably borrow shared raw constraints while holding their robot lock.
pub(crate) fn with_shared_constraints_mut_result<R>(
    shared: &SharedRobot,
    f: impl FnOnce(&mut crate::copp::constraints::Constraints) -> PyResult<R>,
) -> PyResult<R> {
    let mut robot = lock_robot(shared)?;
    f(&mut robot.constraints)
}

/// Prefer a pending Python dynamics exception over a Rust sentinel error.
fn map_robot_copp_result<T>(
    robot: &RustRobot<PyRobotModel>,
    result: Result<T, CoppError>,
) -> PyResult<T> {
    result.map_err(|error| {
        if let Some(py_err) = robot.model().take_pending_error() {
            return py_err;
        }
        to_py_err(error)
    })
}

/// Return the default station length for a broadcast limit.
fn inferred_station_length(robot: &RustRobot<PyRobotModel>, start_idx_s: usize) -> usize {
    robot.constraints.idx_s_end().saturating_sub(start_idx_s)
}

/// Return interval defaults from the current constraint window.
fn default_interval(
    robot: &RustRobot<PyRobotModel>,
    idx_s_from: Option<usize>,
    idx_s_to: Option<usize>,
) -> (usize, usize) {
    (
        idx_s_from.unwrap_or_else(|| robot.constraints.idx_s_start()),
        idx_s_to.unwrap_or_else(|| robot.constraints.idx_s_end()),
    )
}

/// Convert a Python limit input into a Rust `(dim, n_samples)` matrix.
fn limit_to_dmatrix(
    name: &str,
    value: &Bound<'_, PyAny>,
    dim: usize,
    default_length: usize,
    length: Option<usize>,
    layout: PyMatrixLayout,
) -> PyResult<DMatrix<f64>> {
    if let Ok(values) = array_like_to_vec_f64(name, value) {
        if values.len() != dim {
            return Err(PyValueError::new_err(format!(
                "`{name}` must have length {dim}; got {}",
                values.len()
            )));
        }
        let ncols = length.unwrap_or(default_length);
        return Ok(DMatrix::from_fn(dim, ncols, |row, _| values[row]));
    }

    if let Ok(matrix) = matrix_to_dmatrix(name, value, layout, Some(dim)) {
        if let Some(length) = length
            && matrix.ncols() != length
        {
            return Err(PyValueError::new_err(format!(
                "`length` is {length}, but `{name}` contains {} station samples",
                matrix.ncols()
            )));
        }
        return Ok(matrix);
    }

    Err(PyValueError::new_err(format!(
        "`{name}` must be convertible to a one- or two-dimensional float64 array"
    )))
}

/// Convert a Python ArrayLike matrix to a Rust `(rows, n_samples)` [`DMatrix`].
fn matrix_to_dmatrix(
    name: &str,
    value: &Bound<'_, PyAny>,
    layout: PyMatrixLayout,
    expected_rows: Option<usize>,
) -> PyResult<DMatrix<f64>> {
    let (rows, cols, data) = array_like_to_vec2_f64(name, value)?;

    let matrix = match layout {
        PyMatrixLayout::SampleMajor => {
            let n_samples = rows;
            let out_rows = cols;
            let mut matrix = DMatrix::<f64>::zeros(out_rows, n_samples);
            for sample in 0..n_samples {
                for row in 0..out_rows {
                    matrix[(row, sample)] = data[sample * out_rows + row];
                }
            }
            matrix
        }
        PyMatrixLayout::DimMajor => DMatrix::from_row_slice(rows, cols, &data),
    };

    if let Some(expected_rows) = expected_rows
        && matrix.nrows() != expected_rows
    {
        return Err(PyValueError::new_err(format!(
            "`{name}` has {} rows after layout conversion; expected {expected_rows}",
            matrix.nrows()
        )));
    }

    Ok(matrix)
}

/// Normalize user-facing option strings for lenient matching.
fn normalize_token(text: &str) -> String {
    text.trim().to_ascii_lowercase().replace('-', "_")
}
