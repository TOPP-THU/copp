//! Python object wrappers for path construction and path evaluation.
//!
//! The Rust path API uses [`DMatrix`] in `(dim, n_samples)` layout. This module
//! exposes a Python-friendly facade with explicit [`PyMatrixLayout`] conversion,
//! [`PySplineConfig`] ownership, and [`PyPathDerivatives`] result objects.

use crate::diag::PathError;
use crate::ffi::python::array::{array_like_to_vec_f64, array_like_to_vec2_f64};
use crate::ffi::python::error::to_py_err;
use crate::path::{
    OutOfRangeMode as RustOutOfRangeMode, Parametrization as RustParametrization, Path as RustPath,
    PathEvaluator2nd, PathEvaluator3rd, SplineConfig as RustSplineConfig,
};
use nalgebra::DMatrix;
use numpy::{PyArray1, PyArray2, PyArrayMethods};
use pyo3::Borrowed;
use pyo3::exceptions::{PyAttributeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyAnyMethods, PyTuple, PyTupleMethods};
use std::sync::{Arc, Mutex};

/// Register path-related classes on the native [`PyModule`].
pub(crate) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyOutOfRangeMode>()?;
    m.add_class::<PyParametrization>()?;
    m.add_class::<PyMatrixLayout>()?;
    m.add_class::<PySplineConfig>()?;
    m.add_class::<PyPathDerivatives>()?;
    m.add_class::<PyPath>()?;
    Ok(())
}

/// Python enum mirroring [`RustOutOfRangeMode`].
#[pyclass(
    name = "OutOfRangeMode",
    module = "copp_py._native",
    eq,
    eq_int,
    rename_all = "SCREAMING_SNAKE_CASE",
    from_py_object
)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum PyOutOfRangeMode {
    /// Reject out-of-range path parameters with a core [`crate::diag::PathError`].
    Error,
    /// Clamp out-of-range path parameters into the configured path range.
    Clamp,
}

impl PyOutOfRangeMode {
    /// Convert the Python wrapper enum into [`RustOutOfRangeMode`].
    fn to_rust(self) -> RustOutOfRangeMode {
        match self {
            Self::Error => RustOutOfRangeMode::Error,
            Self::Clamp => RustOutOfRangeMode::Clamp,
        }
    }
}

/// Python enum mirroring [`RustParametrization`].
#[pyclass(
    name = "Parametrization",
    module = "copp_py._native",
    eq,
    eq_int,
    rename_all = "SCREAMING_SNAKE_CASE",
    from_py_object
)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum PyParametrization {
    /// Uniformly distribute waypoints over the configured path range.
    Uniform,
}

impl PyParametrization {
    /// Convert the Python wrapper enum into [`RustParametrization`].
    fn to_rust(self) -> RustParametrization {
        match self {
            Self::Uniform => RustParametrization::Uniform,
        }
    }
}

/// Matrix layout contract used by Python path inputs and outputs.
#[pyclass(
    name = "MatrixLayout",
    module = "copp_py._native",
    eq,
    eq_int,
    rename_all = "SCREAMING_SNAKE_CASE",
    from_py_object
)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum PyMatrixLayout {
    /// Python-style layout: rows are samples/waypoints, columns are dimensions.
    SampleMajor,
    /// Dimension-major layout: rows are dimensions, columns are samples/waypoints.
    DimMajor,
}

/// Internal parser for `OutOfRangeMode | str | None` Python arguments.
#[derive(Clone, Copy)]
struct OutOfRangeArg(
    /// Parsed out-of-range mode used by [`pyo3`] method arguments.
    PyOutOfRangeMode,
);

impl FromPyObject<'_, '_> for OutOfRangeArg {
    /// [`PyErr`] returned when parsing fails.
    type Error = PyErr;

    /// Parse enum values, accepted strings, or `None` into an out-of-range mode.
    fn extract(obj: Borrowed<'_, '_, PyAny>) -> Result<Self, Self::Error> {
        if obj.is_none() {
            return Ok(Self(PyOutOfRangeMode::Error));
        }

        if let Ok(mode) = obj.extract::<PyOutOfRangeMode>() {
            return Ok(Self(mode));
        }

        if let Ok(text) = obj.extract::<&str>() {
            return match normalize_token(text).as_str() {
                "error" => Ok(Self(PyOutOfRangeMode::Error)),
                "clamp" => Ok(Self(PyOutOfRangeMode::Clamp)),
                _ => Err(PyValueError::new_err(
                    "`out_of_range` must be OutOfRangeMode.ERROR, OutOfRangeMode.CLAMP, \"error\", or \"clamp\"",
                )),
            };
        }

        Err(PyValueError::new_err(
            "`out_of_range` must be an OutOfRangeMode value or a string",
        ))
    }
}

/// Internal parser for `Parametrization | str | None` Python arguments.
#[derive(Clone, Copy)]
struct ParametrizationArg(
    /// Parsed waypoint parametrization used by [`pyo3`] method arguments.
    PyParametrization,
);

impl FromPyObject<'_, '_> for ParametrizationArg {
    /// [`PyErr`] returned when parsing fails.
    type Error = PyErr;

    /// Parse enum values, accepted strings, or `None` into a parametrization.
    fn extract(obj: Borrowed<'_, '_, PyAny>) -> Result<Self, Self::Error> {
        if obj.is_none() {
            return Ok(Self(PyParametrization::Uniform));
        }

        if let Ok(parametrization) = obj.extract::<PyParametrization>() {
            return Ok(Self(parametrization));
        }

        if let Ok(text) = obj.extract::<&str>() {
            return match normalize_token(text).as_str() {
                "uniform" => Ok(Self(PyParametrization::Uniform)),
                _ => Err(PyValueError::new_err(
                    "`parametrization` must be Parametrization.UNIFORM or \"uniform\"",
                )),
            };
        }

        Err(PyValueError::new_err(
            "`parametrization` must be a Parametrization value or a string",
        ))
    }
}

/// Internal parser for `MatrixLayout | str | None` Python arguments.
#[derive(Clone, Copy)]
struct MatrixLayoutArg(
    /// Parsed matrix layout used by [`pyo3`] method arguments.
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

/// Python-owned waypoint spline configuration backed by [`RustSplineConfig`].
#[pyclass(name = "SplineConfig", module = "copp_py._native")]
pub(crate) struct PySplineConfig {
    /// Odd spline order, at least 3 when passed to the Rust core.
    order: usize,
    /// Waypoint parameter assignment policy.
    parametrization: PyParametrization,
    /// Lower endpoint of the path parameter range.
    s_min: f64,
    /// Upper endpoint of the path parameter range.
    s_max: f64,
    /// Runtime behavior for path evaluation outside `[s_min, s_max]`.
    out_of_range: PyOutOfRangeMode,
    /// Optional endpoint derivative [`DMatrix`] at `s_min`, shape `(dim, m)`.
    start_state: Option<DMatrix<f64>>,
    /// Optional endpoint derivative [`DMatrix`] at `s_max`, shape `(dim, m)`.
    end_state: Option<DMatrix<f64>>,
}

#[pymethods]
impl PySplineConfig {
    /// Configuration for waypoint-spline path construction.
    ///
    /// Parameters
    /// ----------
    /// order : int, default=5
    ///     Odd spline order, at least 3. Common values are 3, 5, and 7.
    /// s_min : float, default=0.0
    ///     Lower endpoint of the path parameter range.
    /// s_max : float, default=1.0
    ///     Upper endpoint of the path parameter range. Must be greater than
    ///     ``s_min`` when building a path.
    /// out_of_range : OutOfRangeMode | str, default=OutOfRangeMode.ERROR
    ///     Behavior when evaluating outside ``[s_min, s_max]``. Strings
    ///     ``"error"`` and ``"clamp"`` are accepted for convenience.
    /// parametrization : Parametrization | str, default=Parametrization.UNIFORM
    ///     Waypoint parameter assignment policy. Only uniform spacing is
    ///     currently supported.
    /// start_state, end_state : ArrayLike | None
    ///     Optional boundary derivative matrices convertible to float64 with shape
    ///     ``(dim, (order - 1) // 2)``. ``None`` means zero endpoint
    ///     derivatives.
    #[new]
    #[pyo3(
        signature = (*, order = 5, s_min = 0.0, s_max = 1.0, out_of_range = OutOfRangeArg(PyOutOfRangeMode::Error), parametrization = ParametrizationArg(PyParametrization::Uniform), start_state = None, end_state = None),
        text_signature = "(*, order=5, s_min=0.0, s_max=1.0, out_of_range='error', parametrization='uniform', start_state=None, end_state=None)"
    )]
    fn new<'py>(
        order: usize,
        s_min: f64,
        s_max: f64,
        out_of_range: OutOfRangeArg,
        parametrization: ParametrizationArg,
        start_state: Option<&Bound<'py, PyAny>>,
        end_state: Option<&Bound<'py, PyAny>>,
    ) -> PyResult<Self> {
        Ok(Self {
            order,
            parametrization: parametrization.0,
            s_min,
            s_max,
            out_of_range: out_of_range.0,
            start_state: optional_boundary_matrix("start_state", start_state)?,
            end_state: optional_boundary_matrix("end_state", end_state)?,
        })
    }

    /// Return the spline order stored in this Python config object.
    #[getter]
    fn order(&self) -> usize {
        self.order
    }

    /// Set the spline order without validating core-level spline constraints.
    ///
    /// The Rust core validates oddness and minimum order when a [`RustPath`] is
    /// constructed, keeping setter behavior lightweight and Pythonic.
    #[setter]
    fn set_order(&mut self, order: usize) {
        self.order = order;
    }

    /// Return the lower endpoint of the path parameter range.
    #[getter]
    fn s_min(&self) -> f64 {
        self.s_min
    }

    /// Set the lower endpoint of the path parameter range.
    #[setter]
    fn set_s_min(&mut self, s_min: f64) {
        self.s_min = s_min;
    }

    /// Return the upper endpoint of the path parameter range.
    #[getter]
    fn s_max(&self) -> f64 {
        self.s_max
    }

    /// Set the upper endpoint of the path parameter range.
    #[setter]
    fn set_s_max(&mut self, s_max: f64) {
        self.s_max = s_max;
    }

    /// Return the out-of-range evaluation policy.
    #[getter]
    fn out_of_range(&self) -> PyOutOfRangeMode {
        self.out_of_range
    }

    /// Set the out-of-range evaluation policy from an enum value or string.
    #[setter]
    fn set_out_of_range(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.out_of_range = parse_out_of_range(value)?;
        Ok(())
    }

    /// Return the waypoint parametrization policy.
    #[getter]
    fn parametrization(&self) -> PyParametrization {
        self.parametrization
    }

    /// Set the waypoint parametrization policy from an enum value or string.
    #[setter]
    fn set_parametrization(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.parametrization = parse_parametrization(value)?;
        Ok(())
    }

    /// Return a copy of the start boundary derivative matrix, if present.
    #[getter]
    fn start_state<'py>(&self, py: Python<'py>) -> PyResult<Option<Bound<'py, PyArray2<f64>>>> {
        optional_boundary_matrix_to_pyarray2(py, self.start_state.as_ref())
    }

    /// Set or clear the start boundary derivative matrix.
    #[setter]
    fn set_start_state(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.start_state = parse_optional_boundary_value("start_state", value)?;
        Ok(())
    }

    /// Return a copy of the end boundary derivative matrix, if present.
    #[getter]
    fn end_state<'py>(&self, py: Python<'py>) -> PyResult<Option<Bound<'py, PyArray2<f64>>>> {
        optional_boundary_matrix_to_pyarray2(py, self.end_state.as_ref())
    }

    /// Set or clear the end boundary derivative matrix.
    #[setter]
    fn set_end_state(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.end_state = parse_optional_boundary_value("end_state", value)?;
        Ok(())
    }

    /// Return a compact representation for interactive Python sessions.
    fn __repr__(&self) -> String {
        format!(
            "SplineConfig(order={}, s_min={}, s_max={}, out_of_range={:?}, parametrization={:?})",
            self.order, self.s_min, self.s_max, self.out_of_range, self.parametrization
        )
    }
}

impl PySplineConfig {
    /// Convert the owned Python configuration into [`RustSplineConfig`].
    fn to_rust(&self) -> RustSplineConfig {
        RustSplineConfig {
            order: self.order,
            parametrization: self.parametrization.to_rust(),
            s_min: self.s_min,
            s_max: self.s_max,
            out_of_range_mode: self.out_of_range.to_rust(),
            start_state: self.start_state.clone(),
            end_state: self.end_state.clone(),
        }
    }
}

/// Python-visible path evaluation result container.
#[pyclass(name = "PathDerivatives", module = "copp_py._native")]
pub(crate) struct PyPathDerivatives {
    /// Position samples as a [`PyArray2`] in the path's selected output layout.
    #[pyo3(get)]
    q: Py<PyArray2<f64>>,
    /// First derivative samples, or `None` when not requested.
    #[pyo3(get)]
    dq: Option<Py<PyArray2<f64>>>,
    /// Second derivative samples, or `None` when not requested.
    #[pyo3(get)]
    ddq: Option<Py<PyArray2<f64>>>,
    /// Third derivative samples, or `None` when not requested.
    #[pyo3(get)]
    dddq: Option<Py<PyArray2<f64>>>,
}

#[pymethods]
impl PyPathDerivatives {
    /// Return a compact representation without printing array contents.
    fn __repr__(&self) -> &'static str {
        "PathDerivatives(q=<numpy.ndarray>, dq=..., ddq=..., dddq=...)"
    }
}

/// Python-owned wrapper around the Rust [`RustPath`] object.
#[pyclass(name = "Path", module = "copp_py._native")]
pub(crate) struct PyPath {
    /// The core [`RustPath`] object that owns spline/evaluator data.
    inner: RustPath,
    /// Preferred Python matrix layout for inputs and derivative outputs.
    layout: PyMatrixLayout,
    /// Shared Python callback state for evaluator-backed paths.
    callback_state: Option<Arc<PyCallbackState>>,
}

#[pymethods]
impl PyPath {
    /// Build a waypoint spline path.
    ///
    /// Parameters
    /// ----------
    /// waypoints : ArrayLike
    ///     Waypoint matrix convertible to float64. With the default
    ///     ``MatrixLayout.SAMPLE_MAJOR``, shape is ``(n_points, dim)`` and
    ///     each row is one waypoint. With ``MatrixLayout.DIM_MAJOR``, shape is
    ///     ``(dim, n_points)`` and each column is one waypoint.
    /// config : SplineConfig | None, default=None
    ///     Spline construction options. When omitted, keyword arguments build
    ///     an equivalent temporary ``SplineConfig``.
    /// order : int, default=5
    ///     Odd spline order used only when ``config`` is omitted.
    /// s_min, s_max : float, default=0.0, 1.0
    ///     Path-parameter range used only when ``config`` is omitted.
    /// out_of_range : OutOfRangeMode | str, default=OutOfRangeMode.ERROR
    ///     Out-of-range policy used only when ``config`` is omitted.
    /// parametrization : Parametrization | str, default=Parametrization.UNIFORM
    ///     Waypoint parameter assignment used only when ``config`` is omitted.
    /// start_state, end_state : ArrayLike | None
    ///     Boundary derivative matrices used only when ``config`` is omitted.
    /// layout : MatrixLayout | str, default=MatrixLayout.SAMPLE_MAJOR
    ///     Matrix layout for both input waypoints and returned derivative
    ///     arrays. Strings ``"sample_major"`` and ``"dim_major"`` are
    ///     accepted for convenience.
    ///
    /// Returns
    /// -------
    /// Path
    ///     A Python-owned wrapper around the Rust path object.
    ///
    /// Raises
    /// ------
    /// ValueError
    ///     If array layout, dtype, wrapper-level options, or mixed
    ///     ``config``/keyword options are invalid.
    /// CoppError
    ///     If the Rust COPP core rejects the path data or spline options.
    #[staticmethod]
    #[pyo3(
        signature = (waypoints, config = None, *, order = 5, s_min = 0.0, s_max = 1.0, out_of_range = OutOfRangeArg(PyOutOfRangeMode::Error), parametrization = ParametrizationArg(PyParametrization::Uniform), start_state = None, end_state = None, layout = MatrixLayoutArg(PyMatrixLayout::SampleMajor)),
        text_signature = "(waypoints, config=None, *, order=5, s_min=0.0, s_max=1.0, out_of_range='error', parametrization='uniform', start_state=None, end_state=None, layout='sample_major')"
    )]
    fn from_waypoints<'py>(
        waypoints: &Bound<'py, PyAny>,
        config: Option<PyRef<'py, PySplineConfig>>,
        order: usize,
        s_min: f64,
        s_max: f64,
        out_of_range: OutOfRangeArg,
        parametrization: ParametrizationArg,
        start_state: Option<&Bound<'py, PyAny>>,
        end_state: Option<&Bound<'py, PyAny>>,
        layout: MatrixLayoutArg,
    ) -> PyResult<Self> {
        let layout = layout.0;
        let waypoints = waypoints_to_dmatrix("waypoints", waypoints, layout)?;
        let direct_config = PySplineConfig {
            order,
            parametrization: parametrization.0,
            s_min,
            s_max,
            out_of_range: out_of_range.0,
            start_state: optional_boundary_matrix("start_state", start_state)?,
            end_state: optional_boundary_matrix("end_state", end_state)?,
        };
        let config = match config {
            Some(config) => {
                ensure_default_direct_config(&direct_config)?;
                config.to_rust()
            }
            None => direct_config.to_rust(),
        };
        let inner = RustPath::from_waypoints(&waypoints, config)
            .map_err(|error| to_py_err(error.into()))?;
        Ok(Self {
            inner,
            layout,
            callback_state: None,
        })
    }

    /// Build a path from a Python evaluator object with derivatives up to second order.
    ///
    /// Parameters
    /// ----------
    /// evaluator : object
    ///     Python object implementing the evaluator protocol. It must provide
    ///     ``dim`` as an integer attribute or zero-argument method, and
    ///     ``evaluate_up_to_2nd(s) -> (q, dq, ddq)``. ``evaluate_q(s) -> q``
    ///     is optional and is used when available.
    /// s_min, s_max : float
    ///     Valid path-parameter range. Evaluator-backed paths currently match
    ///     the Rust API and reject out-of-range queries.
    /// layout : MatrixLayout | str, default=MatrixLayout.SAMPLE_MAJOR
    ///     Matrix layout expected from callback return arrays and used for
    ///     returned derivative arrays.
    ///
    /// Raises
    /// ------
    /// ValueError
    ///     If the evaluator protocol or callback return arrays are invalid.
    /// CoppError
    ///     If the Rust COPP core rejects the path range or evaluator dimension.
    /// Exception
    ///     Any exception raised by the Python evaluator callback is propagated
    ///     unchanged.
    #[staticmethod]
    #[pyo3(
        signature = (evaluator, s_min, s_max, *, layout = MatrixLayoutArg(PyMatrixLayout::SampleMajor)),
        text_signature = "(evaluator, s_min, s_max, *, layout='sample_major')"
    )]
    fn from_evaluator_2nd(
        py: Python<'_>,
        evaluator: Py<PyAny>,
        s_min: f64,
        s_max: f64,
        layout: MatrixLayoutArg,
    ) -> PyResult<Self> {
        let (dim, has_evaluate_q) = {
            let evaluator_bound = evaluator.bind(py);
            let dim = evaluator_dim(evaluator_bound)?;
            ensure_evaluator_method(evaluator_bound, "evaluate_up_to_2nd", "from_evaluator_2nd")?;
            (
                dim,
                has_callable_evaluator_method(evaluator_bound, "evaluate_q")?,
            )
        };
        let state = Arc::new(PyCallbackState::new(
            evaluator,
            dim,
            layout.0,
            has_evaluate_q,
            true,
        ));
        let inner = RustPath::from_evaluator_2nd(
            PyCallbackPathEvaluator2nd {
                state: Arc::clone(&state),
            },
            s_min,
            s_max,
        )
        .map_err(|error| to_py_err(error.into()))?;

        Ok(Self {
            inner,
            layout: layout.0,
            callback_state: Some(state),
        })
    }

    /// Build a path from a Python evaluator object with derivatives up to third order.
    ///
    /// Parameters
    /// ----------
    /// evaluator : object
    ///     Python object implementing the evaluator protocol. It must provide
    ///     ``dim`` as an integer attribute or zero-argument method, and
    ///     ``evaluate_up_to_3rd(s) -> (q, dq, ddq, dddq)``. ``evaluate_q`` and
    ///     ``evaluate_up_to_2nd`` are optional. If ``evaluate_up_to_2nd`` is
    ///     omitted, second-order evaluation calls ``evaluate_up_to_3rd`` and
    ///     discards ``dddq``; this is supported for convenience but not
    ///     recommended for performance-sensitive code.
    /// s_min, s_max : float
    ///     Valid path-parameter range. Evaluator-backed paths currently match
    ///     the Rust API and reject out-of-range queries.
    /// layout : MatrixLayout | str, default=MatrixLayout.SAMPLE_MAJOR
    ///     Matrix layout expected from callback return arrays and used for
    ///     returned derivative arrays.
    ///
    /// Raises
    /// ------
    /// ValueError
    ///     If the evaluator protocol or callback return arrays are invalid.
    /// CoppError
    ///     If the Rust COPP core rejects the path range or evaluator dimension.
    /// Exception
    ///     Any exception raised by the Python evaluator callback is propagated
    ///     unchanged.
    #[staticmethod]
    #[pyo3(
        signature = (evaluator, s_min, s_max, *, layout = MatrixLayoutArg(PyMatrixLayout::SampleMajor)),
        text_signature = "(evaluator, s_min, s_max, *, layout='sample_major')"
    )]
    fn from_evaluator_3rd(
        py: Python<'_>,
        evaluator: Py<PyAny>,
        s_min: f64,
        s_max: f64,
        layout: MatrixLayoutArg,
    ) -> PyResult<Self> {
        let (dim, has_evaluate_q, has_evaluate_up_to_2nd) = {
            let evaluator_bound = evaluator.bind(py);
            let dim = evaluator_dim(evaluator_bound)?;
            ensure_evaluator_method(evaluator_bound, "evaluate_up_to_3rd", "from_evaluator_3rd")?;
            (
                dim,
                has_callable_evaluator_method(evaluator_bound, "evaluate_q")?,
                has_callable_evaluator_method(evaluator_bound, "evaluate_up_to_2nd")?,
            )
        };
        let state = Arc::new(PyCallbackState::new(
            evaluator,
            dim,
            layout.0,
            has_evaluate_q,
            has_evaluate_up_to_2nd,
        ));
        let inner = RustPath::from_evaluator_3rd(
            PyCallbackPathEvaluator3rd {
                state: Arc::clone(&state),
            },
            s_min,
            s_max,
        )
        .map_err(|error| to_py_err(error.into()))?;

        Ok(Self {
            inner,
            layout: layout.0,
            callback_state: Some(state),
        })
    }

    /// Compatibility alias for ``Path.from_evaluator_3rd``.
    ///
    /// New Python code should prefer ``from_evaluator_2nd`` or
    /// ``from_evaluator_3rd`` so the supported derivative order is explicit.
    #[staticmethod]
    #[pyo3(
        signature = (evaluator, s_min, s_max, *, layout = MatrixLayoutArg(PyMatrixLayout::SampleMajor)),
        text_signature = "(evaluator, s_min, s_max, *, layout='sample_major')"
    )]
    fn from_evaluator(
        py: Python<'_>,
        evaluator: Py<PyAny>,
        s_min: f64,
        s_max: f64,
        layout: MatrixLayoutArg,
    ) -> PyResult<Self> {
        Self::from_evaluator_3rd(py, evaluator, s_min, s_max, layout)
    }

    /// Build a path from a scalar-parametric Python callable using JAX.
    ///
    /// This convenience constructor lives in the Python facade layer. It lazily
    /// imports `copp_py._parametric`, builds a JAX-backed Python evaluator
    /// object, and delegates to [`Self::from_evaluator_3rd`]. The Rust core
    /// still receives the same [`PathEvaluator3rd`] protocol as explicit
    /// evaluator-backed paths.
    ///
    /// Parameters
    /// ----------
    /// q_fn : callable
    ///     Scalar-parametric function `q_fn(s)` returning one path vector.
    ///     The function should use `jax.numpy` operations so JAX can trace it.
    /// s_min, s_max : float
    ///     Valid path-parameter range. Parametric paths currently match the
    ///     Rust evaluator API and reject out-of-range queries.
    /// layout : MatrixLayout | str, default=MatrixLayout.SAMPLE_MAJOR
    ///     Matrix layout used for returned derivative arrays.
    /// jit : bool, default=True
    ///     JIT-compile the JAX batched evaluator.
    /// require_x64 : bool, default=True
    ///     Require JAX 64-bit mode before constructing the path.
    ///
    /// Raises
    /// ------
    /// ImportError
    ///     If the selected optional dependency is not installed.
    /// ValueError
    ///     If the callback output or JAX configuration is invalid.
    #[staticmethod]
    #[pyo3(
        signature = (q_fn, s_min, s_max, *, layout = MatrixLayoutArg(PyMatrixLayout::SampleMajor), jit = true, require_x64 = true),
        text_signature = "(q_fn, s_min, s_max, *, layout='sample_major', jit=True, require_x64=True)"
    )]
    fn from_jax(
        py: Python<'_>,
        q_fn: Py<PyAny>,
        s_min: f64,
        s_max: f64,
        layout: MatrixLayoutArg,
        jit: bool,
        require_x64: bool,
    ) -> PyResult<Py<PyAny>> {
        let helper = py.import("copp_py._parametric")?.getattr("_from_jax")?;
        Ok(helper
            .call1((q_fn, s_min, s_max, layout_token(layout.0), jit, require_x64))?
            .unbind())
    }

    /// Build a path from a scalar-parametric Python callable using Autograd.
    ///
    /// This is the lightweight Python AD alternative to [`Self::from_jax`].
    /// The Autograd dependency is imported lazily only when this constructor is
    /// called.
    ///
    /// Parameters
    /// ----------
    /// q_fn : callable
    ///     Scalar-parametric function `q_fn(s)` returning one path vector.
    ///     The function should use `autograd.numpy` operations so Autograd can
    ///     trace it.
    /// s_min, s_max : float
    ///     Valid path-parameter range. Autograd-backed paths currently match
    ///     the Rust evaluator API and reject out-of-range queries.
    /// layout : MatrixLayout | str, default=MatrixLayout.SAMPLE_MAJOR
    ///     Matrix layout used for returned derivative arrays.
    ///
    /// Raises
    /// ------
    /// ImportError
    ///     If Autograd is not installed.
    /// ValueError
    ///     If the callback output is invalid.
    #[staticmethod]
    #[pyo3(
        signature = (q_fn, s_min, s_max, *, layout = MatrixLayoutArg(PyMatrixLayout::SampleMajor)),
        text_signature = "(q_fn, s_min, s_max, *, layout='sample_major')"
    )]
    fn from_autograd(
        py: Python<'_>,
        q_fn: Py<PyAny>,
        s_min: f64,
        s_max: f64,
        layout: MatrixLayoutArg,
    ) -> PyResult<Py<PyAny>> {
        let helper = py
            .import("copp_py._parametric")?
            .getattr("_from_autograd")?;
        Ok(helper
            .call1((q_fn, s_min, s_max, layout_token(layout.0)))?
            .unbind())
    }

    /// Build a path from a scalar-parameter CasADi expression.
    ///
    /// The expression is converted to a Python evaluator by
    /// `copp_py._parametric` and then delegated to [`Self::from_evaluator_3rd`].
    /// This keeps CasADi optional and outside the Rust dependency graph.
    ///
    /// Parameters
    /// ----------
    /// q_expr : casadi.SX | casadi.MX | sequence
    ///     Vector expression for the path position.
    /// symbol : casadi.SX | casadi.MX
    ///     Scalar path parameter used to differentiate `q_expr`.
    /// s_min, s_max : float, default=0.0, 1.0
    ///     Valid path-parameter range.
    /// layout : MatrixLayout | str, default=MatrixLayout.SAMPLE_MAJOR
    ///     Matrix layout used for returned derivative arrays.
    #[staticmethod]
    #[pyo3(
        signature = (q_expr, *, symbol, s_min = 0.0, s_max = 1.0, layout = MatrixLayoutArg(PyMatrixLayout::SampleMajor)),
        text_signature = "(q_expr, *, symbol, s_min=0.0, s_max=1.0, layout='sample_major')"
    )]
    fn from_casadi(
        py: Python<'_>,
        q_expr: Py<PyAny>,
        symbol: Py<PyAny>,
        s_min: f64,
        s_max: f64,
        layout: MatrixLayoutArg,
    ) -> PyResult<Py<PyAny>> {
        let helper = py.import("copp_py._parametric")?.getattr("_from_casadi")?;
        Ok(helper
            .call1((q_expr, symbol, s_min, s_max, layout_token(layout.0)))?
            .unbind())
    }

    /// Build a path from scalar-parameter SymPy expressions.
    ///
    /// The SymPy expressions are differentiated in Python and wrapped as a
    /// [`PathEvaluator3rd`] object before entering the Rust core.
    ///
    /// Parameters
    /// ----------
    /// q_exprs : sympy.Expr | sequence | sympy.Matrix
    ///     Position expression or vector of position expressions.
    /// symbol : sympy.Symbol
    ///     Scalar path parameter used to differentiate `q_exprs`.
    /// s_min, s_max : float, default=0.0, 1.0
    ///     Valid path-parameter range.
    /// layout : MatrixLayout | str, default=MatrixLayout.SAMPLE_MAJOR
    ///     Matrix layout used for returned derivative arrays.
    #[staticmethod]
    #[pyo3(
        signature = (q_exprs, *, symbol, s_min = 0.0, s_max = 1.0, layout = MatrixLayoutArg(PyMatrixLayout::SampleMajor)),
        text_signature = "(q_exprs, *, symbol, s_min=0.0, s_max=1.0, layout='sample_major')"
    )]
    fn from_sympy(
        py: Python<'_>,
        q_exprs: Py<PyAny>,
        symbol: Py<PyAny>,
        s_min: f64,
        s_max: f64,
        layout: MatrixLayoutArg,
    ) -> PyResult<Py<PyAny>> {
        let helper = py.import("copp_py._parametric")?.getattr("_from_sympy")?;
        Ok(helper
            .call1((q_exprs, symbol, s_min, s_max, layout_token(layout.0)))?
            .unbind())
    }

    /// Return the number of path dimensions.
    #[getter]
    fn dim(&self) -> usize {
        self.inner.dim()
    }

    /// Return the inclusive valid path parameter range.
    #[getter]
    fn s_range(&self) -> (f64, f64) {
        self.inner.s_range()
    }

    /// Evaluate position ``q`` only at the query path parameters.
    ///
    /// Returns a ``PathDerivatives`` object whose ``q`` field is populated and
    /// whose derivative fields are ``None``.
    fn evaluate_q<'py>(
        &self,
        py: Python<'py>,
        s: &Bound<'py, PyAny>,
    ) -> PyResult<PyPathDerivatives> {
        let s = array_like_to_vec_f64("s", s)?;
        self.clear_callback_error();
        let out = self.map_path_error(self.inner.evaluate_q(&s))?;
        path_derivatives_to_python(py, out, self.layout)
    }

    /// Evaluate position, first derivative, and second derivative.
    ///
    /// Returns a ``PathDerivatives`` object with ``q``, ``dq``, and ``ddq``
    /// populated. ``dddq`` is ``None``.
    fn evaluate_up_to_2nd<'py>(
        &self,
        py: Python<'py>,
        s: &Bound<'py, PyAny>,
    ) -> PyResult<PyPathDerivatives> {
        let s = array_like_to_vec_f64("s", s)?;
        self.clear_callback_error();
        let out = self.map_path_error(self.inner.evaluate_up_to_2nd(&s))?;
        path_derivatives_to_python(py, out, self.layout)
    }

    /// Evaluate position and derivatives up to third order.
    ///
    /// Returns a ``PathDerivatives`` object with ``q``, ``dq``, ``ddq``, and
    /// ``dddq`` populated.
    fn evaluate_up_to_3rd<'py>(
        &self,
        py: Python<'py>,
        s: &Bound<'py, PyAny>,
    ) -> PyResult<PyPathDerivatives> {
        let s = array_like_to_vec_f64("s", s)?;
        self.clear_callback_error();
        let out = self.map_path_error(self.inner.evaluate_up_to_3rd(&s))?;
        path_derivatives_to_python(py, out, self.layout)
    }
}

impl PyPath {
    /// Evaluate up to second order for sibling Python FFI modules.
    ///
    /// This preserves pending Python callback exceptions from evaluator-backed
    /// paths while returning the Rust-owned derivative matrices needed by
    /// robot/constraint wrappers.
    pub(crate) fn evaluate_up_to_2nd_rust(
        &self,
        s: &[f64],
    ) -> PyResult<crate::path::PathDerivatives> {
        self.clear_callback_error();
        self.map_path_error(self.inner.evaluate_up_to_2nd(s))
    }

    /// Evaluate up to third order for sibling Python FFI modules.
    ///
    /// See [`Self::evaluate_up_to_2nd_rust`] for the callback-error handling
    /// contract.
    pub(crate) fn evaluate_up_to_3rd_rust(
        &self,
        s: &[f64],
    ) -> PyResult<crate::path::PathDerivatives> {
        self.clear_callback_error();
        self.map_path_error(self.inner.evaluate_up_to_3rd(s))
    }

    /// Clear stale Python callback errors before a new Rust core evaluation.
    fn clear_callback_error(&self) {
        if let Some(state) = &self.callback_state {
            state.clear_pending_error();
        }
    }

    /// Prefer a stored Python callback exception over the sentinel [`PathError`].
    fn map_path_error<T>(&self, result: Result<T, PathError>) -> PyResult<T> {
        result.map_err(|error| {
            if let Some(state) = &self.callback_state
                && let Some(py_err) = state.take_pending_error()
            {
                return py_err;
            }

            to_py_err(error.into())
        })
    }
}

/// Shared state for Python object evaluators stored inside [`RustPath`].
struct PyCallbackState {
    /// Python evaluator object implementing the path callback protocol.
    evaluator: Py<PyAny>,
    /// Stable evaluator dimension captured at construction time.
    dim: usize,
    /// Matrix layout expected from Python callback return arrays.
    layout: PyMatrixLayout,
    /// Whether the object provides an optimized `evaluate_q` method.
    has_evaluate_q: bool,
    /// Whether the object provides a second-order callback.
    has_evaluate_up_to_2nd: bool,
    /// Pending Python exception raised from inside a trait callback.
    pending_error: Mutex<Option<PyErr>>,
}

impl PyCallbackState {
    /// Construct callback state after the Python protocol has been validated.
    fn new(
        evaluator: Py<PyAny>,
        dim: usize,
        layout: PyMatrixLayout,
        has_evaluate_q: bool,
        has_evaluate_up_to_2nd: bool,
    ) -> Self {
        Self {
            evaluator,
            dim,
            layout,
            has_evaluate_q,
            has_evaluate_up_to_2nd,
            pending_error: Mutex::new(None),
        }
    }

    /// Store a Python exception and return the sentinel [`PathError`] for the Rust trait.
    fn store_error(&self, error: PyErr) -> PathError {
        *self
            .pending_error
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner()) = Some(error);
        PathError::DimensionMismatch
    }

    /// Remove any pending Python exception from an earlier callback.
    fn clear_pending_error(&self) {
        self.pending_error
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .take();
    }

    /// Take the Python exception raised by the most recent callback, if any.
    fn take_pending_error(&self) -> Option<PyErr> {
        self.pending_error
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .take()
    }

    /// Call a Python method returning only `q` and copy it into the Rust buffer.
    fn call_evaluate_q(
        &self,
        method_name: &str,
        s: &[f64],
        q: &mut [f64],
    ) -> Result<(), PathError> {
        Python::attach(|py| -> PyResult<()> {
            let s_array = PyArray1::from_slice(py, s);
            let result = self
                .evaluator
                .bind(py)
                .call_method1(method_name, (&s_array,))?;
            let q_array = callback_array(method_name, "q", &result)?;
            copy_callback_matrix_to_buffer(method_name, "q", q_array, self, s.len(), q)
        })
        .map_err(|error| self.store_error(error))
    }

    /// Call a Python method returning `q`, `dq`, and `ddq`.
    fn call_evaluate_up_to_2nd(
        &self,
        method_name: &str,
        s: &[f64],
        q: &mut [f64],
        dq: &mut [f64],
        ddq: &mut [f64],
    ) -> Result<(), PathError> {
        Python::attach(|py| -> PyResult<()> {
            let s_array = PyArray1::from_slice(py, s);
            let result = self
                .evaluator
                .bind(py)
                .call_method1(method_name, (&s_array,))?;
            let tuple = callback_tuple(method_name, &result, 3)?;

            let q_item = tuple.get_item(0)?;
            let q_array = callback_array(method_name, "q", &q_item)?;
            copy_callback_matrix_to_buffer(method_name, "q", q_array, self, s.len(), q)?;

            let dq_item = tuple.get_item(1)?;
            let dq_array = callback_array(method_name, "dq", &dq_item)?;
            copy_callback_matrix_to_buffer(method_name, "dq", dq_array, self, s.len(), dq)?;

            let ddq_item = tuple.get_item(2)?;
            let ddq_array = callback_array(method_name, "ddq", &ddq_item)?;
            copy_callback_matrix_to_buffer(method_name, "ddq", ddq_array, self, s.len(), ddq)
        })
        .map_err(|error| self.store_error(error))
    }

    /// Call a Python method returning `q`, `dq`, `ddq`, and `dddq`.
    fn call_evaluate_up_to_3rd(
        &self,
        method_name: &str,
        s: &[f64],
        q: &mut [f64],
        dq: &mut [f64],
        ddq: &mut [f64],
        dddq: &mut [f64],
    ) -> Result<(), PathError> {
        Python::attach(|py| -> PyResult<()> {
            let s_array = PyArray1::from_slice(py, s);
            let result = self
                .evaluator
                .bind(py)
                .call_method1(method_name, (&s_array,))?;
            let tuple = callback_tuple(method_name, &result, 4)?;

            let q_item = tuple.get_item(0)?;
            let q_array = callback_array(method_name, "q", &q_item)?;
            copy_callback_matrix_to_buffer(method_name, "q", q_array, self, s.len(), q)?;

            let dq_item = tuple.get_item(1)?;
            let dq_array = callback_array(method_name, "dq", &dq_item)?;
            copy_callback_matrix_to_buffer(method_name, "dq", dq_array, self, s.len(), dq)?;

            let ddq_item = tuple.get_item(2)?;
            let ddq_array = callback_array(method_name, "ddq", &ddq_item)?;
            copy_callback_matrix_to_buffer(method_name, "ddq", ddq_array, self, s.len(), ddq)?;

            let dddq_item = tuple.get_item(3)?;
            let dddq_array = callback_array(method_name, "dddq", &dddq_item)?;
            copy_callback_matrix_to_buffer(method_name, "dddq", dddq_array, self, s.len(), dddq)
        })
        .map_err(|error| self.store_error(error))
    }
}

/// Python evaluator that supports explicit derivatives up to second order.
struct PyCallbackPathEvaluator2nd {
    /// Shared callback state and pending-error slot.
    state: Arc<PyCallbackState>,
}

impl PyCallbackPathEvaluator2nd {
    /// Check that Rust core output buffers match the stored evaluator dimension.
    fn check_output_len(&self, s: &[f64], buffers: &[&[f64]]) -> Result<(), PathError> {
        let expected = self.state.dim * s.len();
        if buffers.iter().any(|buffer| buffer.len() != expected) {
            return Err(PathError::DimensionMismatch);
        }
        Ok(())
    }
}

impl PathEvaluator2nd for PyCallbackPathEvaluator2nd {
    /// Return the evaluator dimension captured during Python construction.
    fn dim(&self) -> usize {
        self.state.dim
    }

    /// Evaluate `q`, using the optional Python shortcut when present.
    fn evaluate_q(&self, s: &[f64], q: &mut [f64]) -> Result<(), PathError> {
        self.check_output_len(s, &[q])?;
        if self.state.has_evaluate_q {
            self.state.call_evaluate_q("evaluate_q", s, q)
        } else {
            let mut dq = vec![0.0; q.len()];
            let mut ddq = vec![0.0; q.len()];
            self.evaluate_up_to_2nd(s, q, &mut dq, &mut ddq)
        }
    }

    /// Evaluate `q`, `dq`, and `ddq` through the required Python callback.
    fn evaluate_up_to_2nd(
        &self,
        s: &[f64],
        q: &mut [f64],
        dq: &mut [f64],
        ddq: &mut [f64],
    ) -> Result<(), PathError> {
        self.check_output_len(s, &[q, dq, ddq])?;
        self.state
            .call_evaluate_up_to_2nd("evaluate_up_to_2nd", s, q, dq, ddq)
    }
}

/// Python evaluator that supports explicit derivatives up to third order.
struct PyCallbackPathEvaluator3rd {
    /// Shared callback state and pending-error slot.
    state: Arc<PyCallbackState>,
}

impl PyCallbackPathEvaluator3rd {
    /// Check that Rust core output buffers match the stored evaluator dimension.
    fn check_output_len(&self, s: &[f64], buffers: &[&[f64]]) -> Result<(), PathError> {
        let expected = self.state.dim * s.len();
        if buffers.iter().any(|buffer| buffer.len() != expected) {
            return Err(PathError::DimensionMismatch);
        }
        Ok(())
    }
}

impl PathEvaluator2nd for PyCallbackPathEvaluator3rd {
    /// Return the evaluator dimension captured during Python construction.
    fn dim(&self) -> usize {
        self.state.dim
    }

    /// Evaluate `q`, falling back from `evaluate_q` to higher-order callbacks.
    fn evaluate_q(&self, s: &[f64], q: &mut [f64]) -> Result<(), PathError> {
        self.check_output_len(s, &[q])?;
        if self.state.has_evaluate_q {
            self.state.call_evaluate_q("evaluate_q", s, q)
        } else if self.state.has_evaluate_up_to_2nd {
            let mut dq = vec![0.0; q.len()];
            let mut ddq = vec![0.0; q.len()];
            self.evaluate_up_to_2nd(s, q, &mut dq, &mut ddq)
        } else {
            let mut dq = vec![0.0; q.len()];
            let mut ddq = vec![0.0; q.len()];
            let mut dddq = vec![0.0; q.len()];
            self.evaluate_up_to_3rd(s, q, &mut dq, &mut ddq, &mut dddq)
        }
    }

    /// Evaluate up to second order, optionally falling back to the third-order callback.
    fn evaluate_up_to_2nd(
        &self,
        s: &[f64],
        q: &mut [f64],
        dq: &mut [f64],
        ddq: &mut [f64],
    ) -> Result<(), PathError> {
        self.check_output_len(s, &[q, dq, ddq])?;
        if self.state.has_evaluate_up_to_2nd {
            self.state
                .call_evaluate_up_to_2nd("evaluate_up_to_2nd", s, q, dq, ddq)
        } else {
            let mut dddq = vec![0.0; q.len()];
            self.evaluate_up_to_3rd(s, q, dq, ddq, &mut dddq)
        }
    }
}

impl PathEvaluator3rd for PyCallbackPathEvaluator3rd {
    /// Evaluate all derivative orders through the required Python callback.
    fn evaluate_up_to_3rd(
        &self,
        s: &[f64],
        q: &mut [f64],
        dq: &mut [f64],
        ddq: &mut [f64],
        dddq: &mut [f64],
    ) -> Result<(), PathError> {
        self.check_output_len(s, &[q, dq, ddq, dddq])?;
        self.state
            .call_evaluate_up_to_3rd("evaluate_up_to_3rd", s, q, dq, ddq, dddq)
    }
}

/// Extract an evaluator dimension from a `dim` attribute or zero-argument method.
fn evaluator_dim(evaluator: &Bound<'_, PyAny>) -> PyResult<usize> {
    let dim_obj = evaluator.getattr("dim").map_err(|error| {
        if error.is_instance_of::<PyAttributeError>(evaluator.py()) {
            PyValueError::new_err(
                "path evaluator must define `dim` as a positive integer attribute or method",
            )
        } else {
            error
        }
    })?;
    let dim = if dim_obj.is_callable() {
        dim_obj.call0()?.extract::<usize>()?
    } else {
        dim_obj.extract::<usize>()?
    };

    if dim == 0 {
        return Err(PyValueError::new_err(
            "path evaluator `dim` must be a positive integer",
        ));
    }

    Ok(dim)
}

/// Ensure that a Python evaluator provides a required method.
fn ensure_evaluator_method(
    evaluator: &Bound<'_, PyAny>,
    method_name: &str,
    constructor_name: &str,
) -> PyResult<()> {
    if has_callable_evaluator_method(evaluator, method_name)? {
        Ok(())
    } else {
        Err(PyValueError::new_err(format!(
            "`Path.{constructor_name}` evaluator must define `{method_name}(s)`"
        )))
    }
}

/// Check whether an evaluator attribute exists and is callable.
fn has_callable_evaluator_method(
    evaluator: &Bound<'_, PyAny>,
    method_name: &str,
) -> PyResult<bool> {
    if !evaluator.hasattr(method_name)? {
        return Ok(false);
    }

    Ok(evaluator.getattr(method_name)?.is_callable())
}

/// Cast a callback return object to a fixed-length [`PyTuple`].
fn callback_tuple<'py>(
    method_name: &str,
    result: &'py Bound<'py, PyAny>,
    expected_len: usize,
) -> PyResult<&'py Bound<'py, PyTuple>> {
    let tuple = result.cast::<PyTuple>().map_err(|_| {
        PyValueError::new_err(format!(
            "`{method_name}` must return a tuple of {expected_len} array-like values"
        ))
    })?;

    if tuple.len() != expected_len {
        return Err(PyValueError::new_err(format!(
            "`{method_name}` must return a tuple of {expected_len} array-like values; got {}",
            tuple.len()
        )));
    }

    Ok(tuple)
}

/// Extract one callback output field as a copied two-dimensional `float64` array.
fn callback_array(
    method_name: &str,
    field_name: &str,
    value: &Bound<'_, PyAny>,
) -> PyResult<(usize, usize, Vec<f64>)> {
    array_like_to_vec2_f64(
        &format!("`{method_name}` return field `{field_name}`"),
        value,
    )
}

/// Copy one Python callback matrix into a Rust column-major evaluator buffer.
fn copy_callback_matrix_to_buffer(
    method_name: &str,
    field_name: &str,
    array: (usize, usize, Vec<f64>),
    state: &PyCallbackState,
    n_samples: usize,
    output: &mut [f64],
) -> PyResult<()> {
    let (expected_rows, expected_cols) =
        callback_expected_shape(state.layout, state.dim, n_samples);
    let (rows, cols, data) = array;
    if (rows, cols) != (expected_rows, expected_cols) {
        return Err(PyValueError::new_err(format!(
            "`{method_name}` returned `{field_name}` with shape ({rows}, {cols}); expected shape ({expected_rows}, {expected_cols})"
        )));
    }

    match state.layout {
        PyMatrixLayout::SampleMajor => {
            for sample in 0..n_samples {
                for dim in 0..state.dim {
                    output[dim + sample * state.dim] = data[sample * state.dim + dim];
                }
            }
        }
        PyMatrixLayout::DimMajor => {
            for dim in 0..state.dim {
                for sample in 0..n_samples {
                    output[dim + sample * state.dim] = data[dim * n_samples + sample];
                }
            }
        }
    }

    Ok(())
}

/// Return the Python callback matrix shape for a given layout.
fn callback_expected_shape(layout: PyMatrixLayout, dim: usize, n_samples: usize) -> (usize, usize) {
    match layout {
        PyMatrixLayout::SampleMajor => (n_samples, dim),
        PyMatrixLayout::DimMajor => (dim, n_samples),
    }
}

/// Return the canonical Python token for a [`PyMatrixLayout`] value.
fn layout_token(layout: PyMatrixLayout) -> &'static str {
    match layout {
        PyMatrixLayout::SampleMajor => "sample_major",
        PyMatrixLayout::DimMajor => "dim_major",
    }
}

/// Parse a Python value accepted by the `out_of_range` property setter.
fn parse_out_of_range(value: &Bound<'_, PyAny>) -> PyResult<PyOutOfRangeMode> {
    if value.is_none() {
        return Ok(PyOutOfRangeMode::Error);
    }

    if let Ok(mode) = value.extract::<PyOutOfRangeMode>() {
        return Ok(mode);
    }

    if let Ok(text) = value.extract::<&str>() {
        return match normalize_token(text).as_str() {
            "error" => Ok(PyOutOfRangeMode::Error),
            "clamp" => Ok(PyOutOfRangeMode::Clamp),
            _ => Err(PyValueError::new_err(
                "`out_of_range` must be OutOfRangeMode.ERROR, OutOfRangeMode.CLAMP, \"error\", or \"clamp\"",
            )),
        };
    }

    Err(PyValueError::new_err(
        "`out_of_range` must be an OutOfRangeMode value or a string",
    ))
}

/// Parse a Python value accepted by the `parametrization` property setter.
fn parse_parametrization(value: &Bound<'_, PyAny>) -> PyResult<PyParametrization> {
    if value.is_none() {
        return Ok(PyParametrization::Uniform);
    }

    if let Ok(parametrization) = value.extract::<PyParametrization>() {
        return Ok(parametrization);
    }

    if let Ok(text) = value.extract::<&str>() {
        return match normalize_token(text).as_str() {
            "uniform" => Ok(PyParametrization::Uniform),
            _ => Err(PyValueError::new_err(
                "`parametrization` must be Parametrization.UNIFORM or \"uniform\"",
            )),
        };
    }

    Err(PyValueError::new_err(
        "`parametrization` must be a Parametrization value or a string",
    ))
}

/// Normalize user-facing option strings for lenient matching.
///
/// The parser accepts case differences and hyphen/underscore spelling
/// differences while still keeping the documented spelling canonical.
fn normalize_token(text: &str) -> String {
    text.trim().to_ascii_lowercase().replace('-', "_")
}

/// Reject ambiguous calls that pass both `config` and direct spline keywords.
fn ensure_default_direct_config(config: &PySplineConfig) -> PyResult<()> {
    let has_non_default = config.order != 5
        || config.s_min != 0.0
        || config.s_max != 1.0
        || config.out_of_range != PyOutOfRangeMode::Error
        || config.parametrization != PyParametrization::Uniform
        || config.start_state.is_some()
        || config.end_state.is_some();

    if has_non_default {
        return Err(PyValueError::new_err(
            "pass either a `SplineConfig` object or direct spline keyword options, not both",
        ));
    }

    Ok(())
}

/// Convert an optional Python boundary-state array into an optional [`DMatrix`].
fn optional_boundary_matrix(
    name: &str,
    array: Option<&Bound<'_, PyAny>>,
) -> PyResult<Option<DMatrix<f64>>> {
    array
        .map(|array| c_contiguous_matrix_to_dmatrix(name, array))
        .transpose()
}

/// Parse a boundary-state property assignment from Python.
///
/// `None` clears the stored matrix; otherwise the value must be convertible to
/// a two-dimensional `float64` array.
fn parse_optional_boundary_value(
    name: &str,
    value: &Bound<'_, PyAny>,
) -> PyResult<Option<DMatrix<f64>>> {
    if value.is_none() {
        return Ok(None);
    }

    optional_boundary_matrix(name, Some(value))
}

/// Convert an ArrayLike two-dimensional value into [`DMatrix`].
///
/// The shape is preserved exactly. This is appropriate for boundary derivative
/// matrices, whose Python and Rust layouts are both documented as `(dim, m)`.
fn c_contiguous_matrix_to_dmatrix(name: &str, value: &Bound<'_, PyAny>) -> PyResult<DMatrix<f64>> {
    let (rows, cols, data) = array_like_to_vec2_f64(name, value)?;
    Ok(DMatrix::from_row_slice(rows, cols, &data))
}

/// Convert Python waypoint arrays into the Rust core `(dim, n_points)` layout.
///
/// With `SampleMajor`, the input is interpreted as `(n_points, dim)` and
/// transposed into a [`DMatrix`]. With `DimMajor`, the input shape is already the
/// Rust-facing `(dim, n_points)` contract.
fn waypoints_to_dmatrix(
    name: &str,
    value: &Bound<'_, PyAny>,
    layout: PyMatrixLayout,
) -> PyResult<DMatrix<f64>> {
    let (rows, cols, data) = array_like_to_vec2_f64(name, value)?;

    match layout {
        PyMatrixLayout::SampleMajor => {
            let n_points = rows;
            let dim = cols;
            let mut matrix = DMatrix::<f64>::zeros(dim, n_points);
            for point in 0..n_points {
                for d in 0..dim {
                    matrix[(d, point)] = data[point * dim + d];
                }
            }
            Ok(matrix)
        }
        PyMatrixLayout::DimMajor => Ok(DMatrix::from_row_slice(rows, cols, &data)),
    }
}

/// Convert Rust [`crate::path::PathDerivatives`] into [`PyPathDerivatives`].
fn path_derivatives_to_python(
    py: Python<'_>,
    out: crate::path::PathDerivatives,
    layout: PyMatrixLayout,
) -> PyResult<PyPathDerivatives> {
    let q = matrix_to_pyarray2(py, &out.q, layout)?.unbind();
    let dq = optional_matrix_to_pyarray2(py, out.dq.as_ref(), layout)?.map(Bound::unbind);
    let ddq = optional_matrix_to_pyarray2(py, out.ddq.as_ref(), layout)?.map(Bound::unbind);
    let dddq = optional_matrix_to_pyarray2(py, out.dddq.as_ref(), layout)?.map(Bound::unbind);

    Ok(PyPathDerivatives { q, dq, ddq, dddq })
}

/// Convert an optional boundary [`DMatrix`] to a Python [`PyArray2`].
///
/// Boundary matrices are always exposed in `(dim, m)` layout
/// because they are configuration data rather than sampled trajectory output.
fn optional_boundary_matrix_to_pyarray2<'py>(
    py: Python<'py>,
    matrix: Option<&DMatrix<f64>>,
) -> PyResult<Option<Bound<'py, PyArray2<f64>>>> {
    matrix
        .map(|matrix| matrix_to_pyarray2(py, matrix, PyMatrixLayout::DimMajor))
        .transpose()
}

/// Convert an optional derivative [`DMatrix`] to a Python [`PyArray2`].
fn optional_matrix_to_pyarray2<'py>(
    py: Python<'py>,
    matrix: Option<&DMatrix<f64>>,
    layout: PyMatrixLayout,
) -> PyResult<Option<Bound<'py, PyArray2<f64>>>> {
    matrix
        .map(|matrix| matrix_to_pyarray2(py, matrix, layout))
        .transpose()
}

/// Convert a Rust [`DMatrix`] into a C-contiguous [`PyArray2`].
///
/// Rust matrices arrive as `(dim, n_samples)`. `SampleMajor` produces
/// `(n_samples, dim)` for Python users; `DimMajor` preserves `(dim, n_samples)`.
fn matrix_to_pyarray2<'py>(
    py: Python<'py>,
    matrix: &DMatrix<f64>,
    layout: PyMatrixLayout,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let rows = matrix.nrows();
    let cols = matrix.ncols();
    let (out_rows, out_cols) = match layout {
        PyMatrixLayout::SampleMajor => (cols, rows),
        PyMatrixLayout::DimMajor => (rows, cols),
    };
    let array = PyArray2::<f64>::zeros(py, [out_rows, out_cols], false);

    {
        let mut writable = array.readwrite();
        let data = writable.as_slice_mut().map_err(|_| {
            PyValueError::new_err("failed to create a contiguous NumPy output array")
        })?;

        match layout {
            PyMatrixLayout::SampleMajor => {
                for sample in 0..cols {
                    for dim in 0..rows {
                        data[sample * out_cols + dim] = matrix[(dim, sample)];
                    }
                }
            }
            PyMatrixLayout::DimMajor => {
                for dim in 0..rows {
                    for sample in 0..cols {
                        data[dim * out_cols + sample] = matrix[(dim, sample)];
                    }
                }
            }
        }
    }

    Ok(array)
}
