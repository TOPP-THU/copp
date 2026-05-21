use crate::errors::copp_err_to_py;
use copp::diag::Verbosity;
use copp::solver::copp2_socp::{ClarabelOptions, ClarabelOptionsBuilder};
use copp::solver::reach_set2::{ReachSet2Options, ReachSet2OptionsBuilder};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

fn str_to_verbosity(s: &str) -> PyResult<Verbosity> {
    match s.to_lowercase().as_str() {
        "silent" => Ok(Verbosity::Silent),
        "summary" => Ok(Verbosity::Summary),
        "debug" => Ok(Verbosity::Debug),
        "trace" => Ok(Verbosity::Trace),
        _ => Err(PyValueError::new_err(format!(
            "invalid verbosity '{s}'. expected one of: silent, summary, debug, trace"
        ))),
    }
}

#[pyclass(name = "ReachSet2Options")]
pub struct PyReachSet2Options {
    pub inner: ReachSet2Options,
}

#[pymethods]
impl PyReachSet2Options {
    #[new]
    #[pyo3(signature = (a_cmp_abs_tol=None, a_cmp_rel_tol=None, lp_feas_tol=None, verbosity="silent"))]
    fn new(
        a_cmp_abs_tol: Option<f64>,
        a_cmp_rel_tol: Option<f64>,
        lp_feas_tol: Option<f64>,
        verbosity: &str,
    ) -> PyResult<Self> {
        let mut b = ReachSet2OptionsBuilder::new();
        if let Some(v) = a_cmp_abs_tol {
            b = b.a_cmp_abs_tol(v);
        }
        if let Some(v) = a_cmp_rel_tol {
            b = b.a_cmp_rel_tol(v);
        }
        if let Some(v) = lp_feas_tol {
            b = b.lp_feas_tol(v);
        }
        b = b.verbosity(str_to_verbosity(verbosity)?);
        Ok(Self {
            inner: b.build().map_err(copp_err_to_py)?,
        })
    }
}

#[pyclass(name = "ClarabelOptions")]
pub struct PyClarabelOptions {
    pub inner: ClarabelOptions,
}

#[pymethods]
impl PyClarabelOptions {
    #[new]
    #[pyo3(signature = (
        verbosity="silent",
        allow_almost_solved=false,
        allow_max_iterations=false,
        allow_max_time=false,
        allow_insufficient_progress=false
    ))]
    fn new(
        verbosity: &str,
        allow_almost_solved: bool,
        allow_max_iterations: bool,
        allow_max_time: bool,
        allow_insufficient_progress: bool,
    ) -> PyResult<Self> {
        let opts = ClarabelOptionsBuilder::new()
            .verbosity(str_to_verbosity(verbosity)?)
            .allow_almost_solved(allow_almost_solved)
            .allow_max_iterations(allow_max_iterations)
            .allow_max_time(allow_max_time)
            .allow_insufficient_progress(allow_insufficient_progress)
            .build()
            .map_err(copp_err_to_py)?;
        Ok(Self { inner: opts })
    }
}
