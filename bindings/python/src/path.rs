use crate::callback_error::PyCallbackErrorSlot;
use crate::convert::{dmatrix_to_ndarray2, ndarray1_to_vec, ndarray2_to_dmatrix};
use crate::errors::path_err_to_py;
use copp::path::{
    Jet3, OutOfRangeMode, Path, PathDerivatives, SplineConfig, cos as jet_cos, exp as jet_exp,
    ln as jet_ln, powi as jet_powi, sin as jet_sin, sqrt as jet_sqrt,
};
use numpy::{PyArray2, PyReadonlyArray1, PyReadonlyArray2};
use pyo3::prelude::*;
use std::sync::Arc;

#[pyclass(name = "Jet3")]
#[derive(Clone)]
pub struct PyJet3 {
    pub inner: Jet3,
}

#[pymethods]
impl PyJet3 {
    #[new]
    fn new(v: f64) -> Self {
        Self {
            inner: Jet3::constant(v),
        }
    }

    #[staticmethod]
    fn seed(v: f64) -> Self {
        Self {
            inner: Jet3::seed(v),
        }
    }

    #[staticmethod]
    fn constant(v: f64) -> Self {
        Self {
            inner: Jet3::constant(v),
        }
    }

    #[getter]
    fn v(&self) -> f64 {
        self.inner.v
    }
    #[getter]
    fn d1(&self) -> f64 {
        self.inner.d1
    }
    #[getter]
    fn d2(&self) -> f64 {
        self.inner.d2
    }
    #[getter]
    fn d3(&self) -> f64 {
        self.inner.d3
    }

    fn __repr__(&self) -> String {
        format!(
            "Jet3(v={}, d1={}, d2={}, d3={})",
            self.inner.v, self.inner.d1, self.inner.d2, self.inner.d3
        )
    }

    fn __add__(&self, other: &Bound<'_, PyAny>) -> PyResult<Self> {
        if let Ok(r) = other.extract::<PyJet3>() {
            Ok(Self {
                inner: self.inner + r.inner,
            })
        } else if let Ok(r) = other.extract::<f64>() {
            Ok(Self {
                inner: self.inner + Jet3::constant(r),
            })
        } else {
            Err(pyo3::exceptions::PyTypeError::new_err("unsupported type"))
        }
    }
    fn __radd__(&self, other: &Bound<'_, PyAny>) -> PyResult<Self> {
        self.__add__(other)
    }

    fn __sub__(&self, other: &Bound<'_, PyAny>) -> PyResult<Self> {
        if let Ok(r) = other.extract::<PyJet3>() {
            Ok(Self {
                inner: self.inner - r.inner,
            })
        } else if let Ok(r) = other.extract::<f64>() {
            Ok(Self {
                inner: self.inner - Jet3::constant(r),
            })
        } else {
            Err(pyo3::exceptions::PyTypeError::new_err("unsupported type"))
        }
    }

    fn __rsub__(&self, other: &Bound<'_, PyAny>) -> PyResult<Self> {
        if let Ok(l) = other.extract::<f64>() {
            Ok(Self {
                inner: Jet3::constant(l) - self.inner,
            })
        } else {
            Err(pyo3::exceptions::PyTypeError::new_err("unsupported type"))
        }
    }

    fn __mul__(&self, other: &Bound<'_, PyAny>) -> PyResult<Self> {
        if let Ok(r) = other.extract::<PyJet3>() {
            Ok(Self {
                inner: self.inner * r.inner,
            })
        } else if let Ok(r) = other.extract::<f64>() {
            Ok(Self {
                inner: self.inner * Jet3::constant(r),
            })
        } else {
            Err(pyo3::exceptions::PyTypeError::new_err("unsupported type"))
        }
    }
    fn __rmul__(&self, other: &Bound<'_, PyAny>) -> PyResult<Self> {
        self.__mul__(other)
    }

    fn __truediv__(&self, other: &Bound<'_, PyAny>) -> PyResult<Self> {
        if let Ok(r) = other.extract::<PyJet3>() {
            Ok(Self {
                inner: self.inner / r.inner,
            })
        } else if let Ok(r) = other.extract::<f64>() {
            Ok(Self {
                inner: self.inner / Jet3::constant(r),
            })
        } else {
            Err(pyo3::exceptions::PyTypeError::new_err("unsupported type"))
        }
    }

    fn __neg__(&self) -> Self {
        Self {
            inner: Jet3::constant(0.0) - self.inner,
        }
    }

    fn __pow__(&self, n: i32, _modulo: Option<i32>) -> Self {
        Self {
            inner: self.inner.powi(n),
        }
    }
}

#[pyfunction]
pub fn sin(x: PyJet3) -> PyJet3 {
    PyJet3 {
        inner: jet_sin(x.inner),
    }
}
#[pyfunction]
pub fn cos(x: PyJet3) -> PyJet3 {
    PyJet3 {
        inner: jet_cos(x.inner),
    }
}
#[pyfunction]
pub fn exp(x: PyJet3) -> PyJet3 {
    PyJet3 {
        inner: jet_exp(x.inner),
    }
}
#[pyfunction]
pub fn ln(x: PyJet3) -> PyJet3 {
    PyJet3 {
        inner: jet_ln(x.inner),
    }
}
#[pyfunction]
pub fn sqrt(x: PyJet3) -> PyJet3 {
    PyJet3 {
        inner: jet_sqrt(x.inner),
    }
}
#[pyfunction]
pub fn powi(x: PyJet3, n: i32) -> PyJet3 {
    PyJet3 {
        inner: jet_powi(x.inner, n),
    }
}

// ─── SplineConfig ───

#[pyclass(name = "SplineConfig")]
pub struct PySplineConfig {
    pub inner: SplineConfig,
}

#[pymethods]
impl PySplineConfig {
    #[new]
    #[pyo3(signature = (order=5, s_min=0.0, s_max=1.0, clamp=false))]
    fn new(order: usize, s_min: f64, s_max: f64, clamp: bool) -> Self {
        let cfg = SplineConfig {
            order,
            s_min,
            s_max,
            out_of_range_mode: if clamp {
                OutOfRangeMode::Clamp
            } else {
                OutOfRangeMode::Error
            },
            ..SplineConfig::default()
        };
        Self { inner: cfg }
    }
}

// ─── PathDerivatives ───

#[pyclass(name = "PathDerivatives")]
pub struct PyPathDerivatives {
    pub inner: PathDerivatives,
}

#[pymethods]
impl PyPathDerivatives {
    #[getter]
    fn q<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f64>> {
        dmatrix_to_ndarray2(py, &self.inner.q)
    }
    #[getter]
    fn dq<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        self.inner.dq.as_ref().map(|m| dmatrix_to_ndarray2(py, m))
    }
    #[getter]
    fn ddq<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        self.inner.ddq.as_ref().map(|m| dmatrix_to_ndarray2(py, m))
    }
    #[getter]
    fn dddq<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        self.inner.dddq.as_ref().map(|m| dmatrix_to_ndarray2(py, m))
    }
}

// ─── Path ───

#[pyclass(name = "Path")]
pub struct PyPath {
    inner: Path,
    callback_error: Option<PyCallbackErrorSlot>,
}

#[pymethods]
impl PyPath {
    #[staticmethod]
    #[pyo3(signature = (waypoints, config=None))]
    fn from_waypoints(
        waypoints: PyReadonlyArray2<'_, f64>,
        config: Option<&PySplineConfig>,
    ) -> PyResult<Self> {
        let mat = ndarray2_to_dmatrix(waypoints);
        let cfg = match config {
            Some(c) => SplineConfig {
                order: c.inner.order,
                s_min: c.inner.s_min,
                s_max: c.inner.s_max,
                out_of_range_mode: c.inner.out_of_range_mode,
                ..SplineConfig::default()
            },
            None => SplineConfig::default(),
        };
        let path = Path::from_waypoints(&mat, cfg).map_err(path_err_to_py)?;
        Ok(Self {
            inner: path,
            callback_error: None,
        })
    }

    #[staticmethod]
    fn from_parametric(_py: Python<'_>, q_fn: PyObject, s_min: f64, s_max: f64) -> PyResult<Self> {
        let q_fn = Arc::new(q_fn);
        let callback_error = PyCallbackErrorSlot::new();
        let closure_error = callback_error.clone();
        let closure = move |s: Jet3| -> Vec<Jet3> {
            if closure_error.has_error() {
                return Vec::new();
            }
            Python::with_gil(|py| {
                let py_s = PyJet3 { inner: s };
                let result = match q_fn.call1(py, (py_s,)) {
                    Ok(result) => result,
                    Err(err) => {
                        closure_error.set_once(err);
                        return Vec::new();
                    }
                };
                match result.extract::<Vec<PyJet3>>(py) {
                    Ok(list) => list.into_iter().map(|j| j.inner).collect(),
                    Err(err) => {
                        closure_error.set_once(err);
                        Vec::new()
                    }
                }
            })
        };
        let path = Path::from_parametric(closure, s_min, s_max);
        if let Some(err) = callback_error.take() {
            return Err(err);
        }
        Ok(Self {
            inner: path.map_err(path_err_to_py)?,
            callback_error: Some(callback_error),
        })
    }

    fn evaluate_q<'py>(
        &self,
        py: Python<'py>,
        s: PyReadonlyArray1<'py, f64>,
    ) -> PyResult<PyPathDerivatives> {
        let sv = ndarray1_to_vec(s);
        let result = py.allow_threads(|| self.inner.evaluate_q(&sv));
        if let Some(err) = self.callback_error.as_ref().and_then(|slot| slot.take()) {
            return Err(err);
        }
        Ok(PyPathDerivatives {
            inner: result.map_err(path_err_to_py)?,
        })
    }

    fn evaluate_up_to_2nd<'py>(
        &self,
        py: Python<'py>,
        s: PyReadonlyArray1<'py, f64>,
    ) -> PyResult<PyPathDerivatives> {
        let sv = ndarray1_to_vec(s);
        let result = py.allow_threads(|| self.inner.evaluate_up_to_2nd(&sv));
        if let Some(err) = self.callback_error.as_ref().and_then(|slot| slot.take()) {
            return Err(err);
        }
        Ok(PyPathDerivatives {
            inner: result.map_err(path_err_to_py)?,
        })
    }

    fn evaluate_up_to_3rd<'py>(
        &self,
        py: Python<'py>,
        s: PyReadonlyArray1<'py, f64>,
    ) -> PyResult<PyPathDerivatives> {
        let sv = ndarray1_to_vec(s);
        let result = py.allow_threads(|| self.inner.evaluate_up_to_3rd(&sv));
        if let Some(err) = self.callback_error.as_ref().and_then(|slot| slot.take()) {
            return Err(err);
        }
        Ok(PyPathDerivatives {
            inner: result.map_err(path_err_to_py)?,
        })
    }

    #[getter]
    fn dim(&self) -> usize {
        self.inner.dim()
    }

    #[getter]
    fn s_range(&self) -> (f64, f64) {
        self.inner.s_range()
    }
}
