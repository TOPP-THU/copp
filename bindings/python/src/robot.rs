use crate::callback_error::PyCallbackErrorSlot;
use crate::convert::{ndarray1_to_vec, ndarray2_to_dmatrix};
use crate::errors::constraint_err_to_py;
use copp::robot::{Robot, RobotBasic, RobotTorque};
use numpy::{PyReadonlyArray1, PyReadonlyArray2};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

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
    fn inverse_dynamics(&self, q: &[f64], dq: &[f64], ddq: &[f64], tau: &mut [f64]) {
        if self.callback_error.has_error() {
            return;
        }
        if let Some(ref py_fn) = self.inv_dyn_fn {
            let result = Python::with_gil(|py| -> PyResult<Vec<f64>> {
                let result = py_fn.call1(py, (q.to_vec(), dq.to_vec(), ddq.to_vec()))?;
                result.extract(py)
            });
            match result {
                Ok(tau_out) if tau_out.len() == tau.len() => tau.copy_from_slice(&tau_out),
                Ok(tau_out) => self.callback_error.set_once(PyValueError::new_err(format!(
                    "inverse_dynamics must return {} values, got {}",
                    tau.len(),
                    tau_out.len()
                ))),
                Err(err) => self.callback_error.set_once(err),
            }
        } else {
            tau.copy_from_slice(ddq);
        }
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
        self.inner.with_s(&sv).map_err(constraint_err_to_py)
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
            .map_err(constraint_err_to_py)
    }

    #[pyo3(signature = (vel_max, vel_min, idx_s=0))]
    fn with_axial_velocity(
        &mut self,
        vel_max: PyReadonlyArray1<'_, f64>,
        vel_min: PyReadonlyArray1<'_, f64>,
        idx_s: usize,
    ) -> PyResult<()> {
        let vmax = ndarray1_to_vec(vel_max);
        let vmin = ndarray1_to_vec(vel_min);
        let n = self.inner.constraints.len();
        self.inner
            .with_axial_velocity((&vmax[..], n), (&vmin[..], n), idx_s)
            .map_err(constraint_err_to_py)
    }

    #[pyo3(signature = (acc_max, acc_min, idx_s=0))]
    fn with_axial_acceleration(
        &mut self,
        acc_max: PyReadonlyArray1<'_, f64>,
        acc_min: PyReadonlyArray1<'_, f64>,
        idx_s: usize,
    ) -> PyResult<()> {
        let amax = ndarray1_to_vec(acc_max);
        let amin = ndarray1_to_vec(acc_min);
        let n = self.inner.constraints.len();
        self.inner
            .with_axial_acceleration((&amax[..], n), (&amin[..], n), idx_s)
            .map_err(constraint_err_to_py)
    }

    #[pyo3(signature = (jerk_max, jerk_min, idx_s=0))]
    fn with_axial_jerk(
        &mut self,
        jerk_max: PyReadonlyArray1<'_, f64>,
        jerk_min: PyReadonlyArray1<'_, f64>,
        idx_s: usize,
    ) -> PyResult<()> {
        let jmax = ndarray1_to_vec(jerk_max);
        let jmin = ndarray1_to_vec(jerk_min);
        let n = self.inner.constraints.len();
        self.inner
            .with_axial_jerk((&jmax[..], n), (&jmin[..], n), idx_s)
            .map_err(constraint_err_to_py)
    }

    #[getter]
    fn dim(&self) -> usize {
        self.inner.dim()
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
