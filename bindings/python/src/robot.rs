use crate::callback_error::PyCallbackErrorSlot;
use crate::convert::{dmatrix_to_ndarray2, ndarray1_to_vec, ndarray2_to_dmatrix};
use crate::errors::{constraint_err_to_py, copp_err_to_py};
use crate::path::PyPath;
use copp::constraints::ModePopConstraints;
use copp::diag::RobotDynamicsError;
use copp::robot::{Robot, RobotBasic, RobotTorque};
use nalgebra::DMatrix;
use numpy::{PyArray1, PyArray2, PyReadonlyArray1, PyReadonlyArray2, PyUntypedArrayMethods};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

type PyArray2Bound<'py> = Bound<'py, PyArray2<f64>>;
type PyAccConstraints<'py> = (PyArray2Bound<'py>, PyArray2Bound<'py>, PyArray2Bound<'py>);
type PyJerkConstraints<'py> = (
    PyArray2Bound<'py>,
    PyArray2Bound<'py>,
    PyArray2Bound<'py>,
    PyArray2Bound<'py>,
    PyArray2Bound<'py>,
);
type PyJerkLinearConstraints<'py> = (
    PyArray2Bound<'py>,
    PyArray2Bound<'py>,
    PyArray2Bound<'py>,
    PyArray2Bound<'py>,
);

pub struct PyRobotModel {
    dim: usize,
    inv_dyn_fn: Option<PyObject>,
    callback_error: PyCallbackErrorSlot,
}

impl RobotBasic for PyRobotModel {
    fn dim(&self) -> usize {
        self.dim
    }
}

impl RobotTorque for PyRobotModel {
    fn inverse_dynamics(
        &self,
        q: &[f64],
        dq: &[f64],
        ddq: &[f64],
        tau: &mut [f64],
    ) -> Result<(), RobotDynamicsError> {
        if self.callback_error.has_error() {
            return Err(RobotDynamicsError::new(
                "inverse_dynamics skipped after a previous Python callback error",
            ));
        }
        if let Some(ref py_fn) = self.inv_dyn_fn {
            let result = Python::with_gil(|py| -> PyResult<Vec<f64>> {
                let result = py_fn.call1(py, (q.to_vec(), dq.to_vec(), ddq.to_vec()))?;
                result.extract(py)
            });
            match result {
                Ok(tau_out) if tau_out.len() == tau.len() => tau.copy_from_slice(&tau_out),
                Ok(tau_out) => {
                    let message = format!(
                        "inverse_dynamics must return {} values, got {}",
                        tau.len(),
                        tau_out.len()
                    );
                    self.callback_error
                        .set_once(PyValueError::new_err(message.clone()));
                    return Err(RobotDynamicsError::new(message));
                }
                Err(err) => {
                    let message = err.to_string();
                    self.callback_error.set_once(err);
                    return Err(RobotDynamicsError::new(message));
                }
            }
        } else {
            tau.copy_from_slice(ddq);
        }
        Ok(())
    }
}

#[pyclass(name = "Robot")]
pub struct PyRobot {
    pub inner: Robot<PyRobotModel>,
    callback_error: PyCallbackErrorSlot,
}

#[pymethods]
impl PyRobot {
    #[new]
    #[pyo3(signature = (dim, inverse_dynamics=None, capacity=0))]
    fn new(dim: usize, inverse_dynamics: Option<PyObject>, capacity: usize) -> Self {
        let callback_error = PyCallbackErrorSlot::new();
        let model = PyRobotModel {
            dim,
            inv_dyn_fn: inverse_dynamics,
            callback_error: callback_error.clone(),
        };
        let robot = if capacity > 0 {
            Robot::with_capacity(model, capacity)
        } else {
            Robot::new(model)
        };
        Self {
            inner: robot,
            callback_error,
        }
    }

    fn with_s(&mut self, s: PyReadonlyArray1<'_, f64>) -> PyResult<()> {
        let sv = ndarray1_to_vec(s);
        self.inner
            .with_s(&sv)
            .map(|_| ())
            .map_err(constraint_err_to_py)
    }

    /// Replace the inverse-dynamics callback on an already-constructed robot.
    ///
    /// Mirrors the C ABI `copp_robot_set_inverse_dynamics`.
    fn set_inverse_dynamics(&mut self, inverse_dynamics: PyObject) {
        self.callback_error.take();
        self.inner.model_mut().inv_dyn_fn = Some(inverse_dynamics);
    }

    /// Clear the inverse-dynamics callback, restoring the default behavior
    /// (`tau = ddq`). Mirrors the C ABI `copp_robot_clear_inverse_dynamics`.
    fn clear_inverse_dynamics(&mut self) {
        self.callback_error.take();
        self.inner.model_mut().inv_dyn_fn = None;
    }

    #[pyo3(signature = (q, dq, ddq, dddq=None, idx_s=0))]
    fn with_q(
        &mut self,
        q: PyReadonlyArray2<'_, f64>,
        dq: PyReadonlyArray2<'_, f64>,
        ddq: PyReadonlyArray2<'_, f64>,
        dddq: Option<PyReadonlyArray2<'_, f64>>,
        idx_s: usize,
    ) -> PyResult<()> {
        let qm = ndarray2_to_dmatrix(q);
        let dqm = ndarray2_to_dmatrix(dq);
        let ddqm = ndarray2_to_dmatrix(ddq);
        let dddqm = dddq.map(ndarray2_to_dmatrix);
        self.inner
            .with_q(
                &qm.as_view(),
                &dqm.as_view(),
                &ddqm.as_view(),
                dddqm.as_ref().map(|m| m.as_view()).as_ref(),
                idx_s,
            )
            .map(|_| ())
            .map_err(constraint_err_to_py)
    }

    #[pyo3(signature = (vel_max, vel_min, idx_s=0))]
    fn with_axial_velocity(
        &mut self,
        vel_max: &Bound<'_, PyAny>,
        vel_min: &Bound<'_, PyAny>,
        idx_s: usize,
    ) -> PyResult<()> {
        let n = remaining_station_count(&self.inner, idx_s)?;
        let vmax = upper_bound_to_matrix(vel_max, self.inner.dim(), n, "vel_max")?;
        let vmin = upper_bound_to_matrix(vel_min, self.inner.dim(), n, "vel_min")?;
        self.inner
            .with_axial_velocity(&vmax.as_view(), &vmin.as_view(), idx_s)
            .map(|_| ())
            .map_err(constraint_err_to_py)
    }

    #[pyo3(signature = (acc_max, acc_min, idx_s=0))]
    fn with_axial_acceleration(
        &mut self,
        acc_max: &Bound<'_, PyAny>,
        acc_min: &Bound<'_, PyAny>,
        idx_s: usize,
    ) -> PyResult<()> {
        let n = remaining_station_count(&self.inner, idx_s)?;
        let amax = upper_bound_to_matrix(acc_max, self.inner.dim(), n, "acc_max")?;
        let amin = upper_bound_to_matrix(acc_min, self.inner.dim(), n, "acc_min")?;
        self.inner
            .with_axial_acceleration(&amax.as_view(), &amin.as_view(), idx_s)
            .map(|_| ())
            .map_err(constraint_err_to_py)
    }

    #[pyo3(signature = (jerk_max, jerk_min, idx_s=0))]
    fn with_axial_jerk(
        &mut self,
        jerk_max: &Bound<'_, PyAny>,
        jerk_min: &Bound<'_, PyAny>,
        idx_s: usize,
    ) -> PyResult<()> {
        let n = remaining_station_count(&self.inner, idx_s)?;
        let jmax = upper_bound_to_matrix(jerk_max, self.inner.dim(), n, "jerk_max")?;
        let jmin = upper_bound_to_matrix(jerk_min, self.inner.dim(), n, "jerk_min")?;
        self.inner
            .with_axial_jerk(&jmax.as_view(), &jmin.as_view(), idx_s)
            .map(|_| ())
            .map_err(constraint_err_to_py)
    }

    #[getter]
    fn dim(&self) -> usize {
        self.inner.dim()
    }

    #[pyo3(signature = (path, idx_s_from, idx_s_to))]
    fn with_q_from_path_2nd(
        &mut self,
        path: &PyPath,
        idx_s_from: usize,
        idx_s_to: usize,
    ) -> PyResult<()> {
        self.inner
            .with_q_from_path_2nd(path.inner(), idx_s_from, idx_s_to)
            .map(|_| ())
            .map_err(copp_err_to_py)
    }

    #[pyo3(signature = (path, idx_s_from, idx_s_to))]
    fn with_q_from_path_3rd(
        &mut self,
        path: &PyPath,
        idx_s_from: usize,
        idx_s_to: usize,
    ) -> PyResult<()> {
        self.inner
            .with_q_from_path_3rd(path.inner(), idx_s_from, idx_s_to)
            .map(|_| ())
            .map_err(copp_err_to_py)
    }

    #[pyo3(signature = (torque_max, torque_min, idx_s=0))]
    fn with_axial_torque(
        &mut self,
        torque_max: &Bound<'_, PyAny>,
        torque_min: &Bound<'_, PyAny>,
        idx_s: usize,
    ) -> PyResult<()> {
        let n = remaining_station_count(&self.inner, idx_s)?;
        let tmax = upper_bound_to_matrix(torque_max, self.inner.dim(), n, "torque_max")?;
        let tmin = upper_bound_to_matrix(torque_min, self.inner.dim(), n, "torque_min")?;
        let result = self
            .inner
            .with_axial_torque(&tmax.as_view(), &tmin.as_view(), idx_s)
            .map(|_| ())
            .map_err(copp_err_to_py);
        if let Some(err) = self.take_callback_error() {
            return Err(err);
        }
        result
    }

    #[getter]
    fn len(&self) -> usize {
        self.inner.constraints.len()
    }

    #[getter]
    fn is_empty(&self) -> bool {
        self.inner.constraints.is_empty()
    }

    #[getter]
    fn idx_s_start(&self) -> usize {
        self.inner.constraints.idx_s_start()
    }

    #[getter]
    fn idx_s_end(&self) -> usize {
        self.inner.constraints.idx_s_end()
    }

    #[getter]
    fn amax_rows(&self) -> usize {
        self.inner.constraints.amax_rows()
    }

    #[getter]
    fn acc_rows(&self) -> usize {
        self.inner.constraints.acc_rows()
    }

    #[getter]
    fn jerk_rows(&self) -> usize {
        self.inner.constraints.jerk_rows()
    }

    fn get_s(&self, idx_s: usize) -> PyResult<f64> {
        self.inner
            .constraints
            .get_s(idx_s)
            .map_err(constraint_err_to_py)
    }

    fn get_amax(&self, idx_s: usize) -> PyResult<f64> {
        self.inner
            .constraints
            .get_amax(idx_s)
            .map_err(constraint_err_to_py)
    }

    fn s_vec<'py>(
        &self,
        py: Python<'py>,
        idx_s_from: usize,
        idx_s_to: usize,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let v = self
            .inner
            .constraints
            .s_vec(idx_s_from, idx_s_to)
            .map_err(constraint_err_to_py)?;
        Ok(PyArray1::from_slice(py, &v))
    }

    fn amax_vec<'py>(
        &self,
        py: Python<'py>,
        idx_s_from: usize,
        idx_s_to: usize,
    ) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let v = self
            .inner
            .constraints
            .amax_vec(idx_s_from, idx_s_to)
            .map_err(constraint_err_to_py)?;
        Ok(PyArray1::from_slice(py, &v))
    }

    #[pyo3(signature = (amax, idx_s=0))]
    fn with_constraint_1order(&mut self, amax: &Bound<'_, PyAny>, idx_s: usize) -> PyResult<()> {
        if let Ok(arr2) = amax.extract::<PyReadonlyArray2<'_, f64>>() {
            let mat = ndarray2_to_dmatrix(arr2);
            return self
                .inner
                .constraints
                .with_constraint_1order(&mat.as_view(), idx_s)
                .map(|_| ())
                .map_err(constraint_err_to_py);
        }
        let arr1 = amax.extract::<PyReadonlyArray1<'_, f64>>()?;
        let values = ndarray1_to_vec(arr1);
        self.inner
            .constraints
            .with_constraint_1order(&values, idx_s)
            .map(|_| ())
            .map_err(constraint_err_to_py)
    }

    #[pyo3(signature = (acc_a, acc_b, acc_max, idx_s=0, is_negative=false))]
    fn with_constraint_2order(
        &mut self,
        acc_a: PyReadonlyArray2<'_, f64>,
        acc_b: PyReadonlyArray2<'_, f64>,
        acc_max: PyReadonlyArray2<'_, f64>,
        idx_s: usize,
        is_negative: bool,
    ) -> PyResult<()> {
        let acc_a = ndarray2_to_dmatrix(acc_a);
        let acc_b = ndarray2_to_dmatrix(acc_b);
        let acc_max = ndarray2_to_dmatrix(acc_max);
        self.inner
            .constraints
            .with_constraint_2order(
                &acc_a.as_view(),
                &acc_b.as_view(),
                &acc_max.as_view(),
                idx_s,
                is_negative,
            )
            .map(|_| ())
            .map_err(constraint_err_to_py)
    }

    #[pyo3(signature = (jerk_a, jerk_b, jerk_c, jerk_d, jerk_max, idx_s=0, is_negative=false))]
    #[allow(clippy::too_many_arguments)]
    fn with_constraint_3order(
        &mut self,
        jerk_a: PyReadonlyArray2<'_, f64>,
        jerk_b: PyReadonlyArray2<'_, f64>,
        jerk_c: PyReadonlyArray2<'_, f64>,
        jerk_d: PyReadonlyArray2<'_, f64>,
        jerk_max: PyReadonlyArray2<'_, f64>,
        idx_s: usize,
        is_negative: bool,
    ) -> PyResult<()> {
        let jerk_a = ndarray2_to_dmatrix(jerk_a);
        let jerk_b = ndarray2_to_dmatrix(jerk_b);
        let jerk_c = ndarray2_to_dmatrix(jerk_c);
        let jerk_d = ndarray2_to_dmatrix(jerk_d);
        let jerk_max = ndarray2_to_dmatrix(jerk_max);
        self.inner
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
            .map_err(constraint_err_to_py)
    }

    fn get_acc_constraints<'py>(
        &self,
        py: Python<'py>,
        idx_s: usize,
    ) -> PyResult<PyAccConstraints<'py>> {
        let (a, b, max) = self
            .inner
            .constraints
            .get_acc_constraints(idx_s)
            .map_err(constraint_err_to_py)?;
        Ok((
            dmatrix_to_ndarray2(py, &a.into_owned()),
            dmatrix_to_ndarray2(py, &b.into_owned()),
            dmatrix_to_ndarray2(py, &max.into_owned()),
        ))
    }

    fn get_jerk_constraints<'py>(
        &self,
        py: Python<'py>,
        idx_s: usize,
    ) -> PyResult<PyJerkConstraints<'py>> {
        let (a, b, c, d, max) = self
            .inner
            .constraints
            .get_jerk_constraints(idx_s)
            .map_err(constraint_err_to_py)?;
        Ok((
            dmatrix_to_ndarray2(py, &a.into_owned()),
            dmatrix_to_ndarray2(py, &b.into_owned()),
            dmatrix_to_ndarray2(py, &c.into_owned()),
            dmatrix_to_ndarray2(py, &d.into_owned()),
            dmatrix_to_ndarray2(py, &max.into_owned()),
        ))
    }

    fn get_jerk_linear_constraints<'py>(
        &self,
        py: Python<'py>,
        idx_s: usize,
    ) -> PyResult<PyJerkLinearConstraints<'py>> {
        let (a, b, c, max) = self
            .inner
            .constraints
            .get_jerk_linear_constraints(idx_s)
            .map_err(constraint_err_to_py)?;
        Ok((
            dmatrix_to_ndarray2(py, &a.into_owned()),
            dmatrix_to_ndarray2(py, &b.into_owned()),
            dmatrix_to_ndarray2(py, &c.into_owned()),
            dmatrix_to_ndarray2(py, &max.into_owned()),
        ))
    }

    fn pop_front_n(&mut self, n_cols: usize) {
        self.inner
            .constraints
            .pop_front(ModePopConstraints::PopNCols(n_cols));
    }

    fn pop_back_n(&mut self, n_cols: usize) {
        self.inner
            .constraints
            .pop_back(ModePopConstraints::PopNCols(n_cols));
    }

    fn pop_front_to(&mut self, idx_s_cut: usize) {
        self.inner
            .constraints
            .pop_front(ModePopConstraints::CutAtIdxS(idx_s_cut));
    }

    fn pop_back_to(&mut self, idx_s_cut: usize) {
        self.inner
            .constraints
            .pop_back(ModePopConstraints::CutAtIdxS(idx_s_cut));
    }

    #[pyo3(signature = (keep_idx_s=true))]
    fn clear(&mut self, keep_idx_s: bool) {
        self.inner.constraints.clear(keep_idx_s);
    }

    fn expand_capacity(&mut self, new_capacity: usize) {
        self.inner.constraints.expand_capacity(new_capacity);
    }

    fn amax_substitute(
        &mut self,
        amax_new: PyReadonlyArray1<'_, f64>,
        idx_from: usize,
    ) -> PyResult<()> {
        let v = ndarray1_to_vec(amax_new);
        self.inner
            .constraints
            .amax_substitute(&v, idx_from)
            .map_err(constraint_err_to_py)
    }
}

impl PyRobot {
    pub(crate) fn take_callback_error(&self) -> Option<PyErr> {
        self.callback_error.take()
    }
}

fn remaining_station_count(robot: &Robot<PyRobotModel>, idx_s: usize) -> PyResult<usize> {
    let end = robot.constraints.idx_s_end();
    if idx_s > end {
        return Err(PyValueError::new_err(format!(
            "idx_s={idx_s} is outside current station range [{}, {})",
            robot.constraints.idx_s_start(),
            end
        )));
    }
    Ok(end - idx_s)
}

fn upper_bound_to_matrix(
    obj: &Bound<'_, PyAny>,
    dim: usize,
    broadcast_cols: usize,
    name: &str,
) -> PyResult<DMatrix<f64>> {
    if let Ok(arr2) = obj.extract::<PyReadonlyArray2<'_, f64>>() {
        let shape = arr2.shape();
        if shape[0] != dim {
            return Err(PyValueError::new_err(format!(
                "{name} must have {dim} rows, got {}",
                shape[0]
            )));
        }
        return Ok(ndarray2_to_dmatrix(arr2));
    }

    let arr1 = obj.extract::<PyReadonlyArray1<'_, f64>>()?;
    let values = ndarray1_to_vec(arr1);
    if values.len() != dim {
        return Err(PyValueError::new_err(format!(
            "{name} as a 1-D broadcast vector must have length {dim}, got {}. Use a 2-D array with shape ({dim}, n) for station-dependent limits.",
            values.len()
        )));
    }
    Ok(DMatrix::from_fn(dim, broadcast_cols, |i, _| values[i]))
}
