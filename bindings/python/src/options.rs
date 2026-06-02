use crate::enums::{DirectSolveMethod, Verbosity};
use crate::errors::copp_err_to_py;
use clarabel::solver::DefaultSettings;
use copp::diag::{
    VerbosityOutput, set_verbosity_log_file as rust_set_verbosity_log_file,
    set_verbosity_output as rust_set_verbosity_output, verbosity_output as rust_verbosity_output,
};
use copp::solver::copp2_socp::{ClarabelOptions, ClarabelOptionsBuilder};
use copp::solver::reach_set2::{ReachSet2Options, ReachSet2OptionsBuilder};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

#[pyclass(name = "ReachSet2Options")]
pub struct PyReachSet2Options {
    pub inner: ReachSet2Options,
}

#[pymethods]
impl PyReachSet2Options {
    #[new]
    #[pyo3(signature = (a_cmp_abs_tol=None, a_cmp_rel_tol=None, lp_feas_tol=None, verbosity=Verbosity::Silent))]
    fn new(
        a_cmp_abs_tol: Option<f64>,
        a_cmp_rel_tol: Option<f64>,
        lp_feas_tol: Option<f64>,
        verbosity: Verbosity,
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
        b = b.verbosity(verbosity.into());
        Ok(Self {
            inner: b.build().map_err(copp_err_to_py)?,
        })
    }

    #[getter]
    fn lp_feas_tol(&self) -> f64 {
        self.inner.lp_feas_tol()
    }

    #[getter]
    fn a_cmp_abs_tol(&self) -> f64 {
        self.inner.a_cmp_abs_tol()
    }

    #[getter]
    fn a_cmp_rel_tol(&self) -> f64 {
        self.inner.a_cmp_rel_tol()
    }

    #[getter]
    fn verbosity(&self) -> Verbosity {
        self.inner.verbosity().into()
    }
}

#[pyclass(name = "ClarabelSettings")]
#[derive(Clone)]
pub struct PyClarabelSettings {
    #[pyo3(get, set)]
    pub max_iter: u32,
    #[pyo3(get, set)]
    pub time_limit: f64,
    #[pyo3(get, set)]
    pub verbose: bool,
    #[pyo3(get, set)]
    pub max_step_fraction: f64,
    #[pyo3(get, set)]
    pub tol_gap_abs: f64,
    #[pyo3(get, set)]
    pub tol_gap_rel: f64,
    #[pyo3(get, set)]
    pub tol_feas: f64,
    #[pyo3(get, set)]
    pub tol_infeas_abs: f64,
    #[pyo3(get, set)]
    pub tol_infeas_rel: f64,
    #[pyo3(get, set)]
    pub tol_ktratio: f64,
    #[pyo3(get, set)]
    pub reduced_tol_gap_abs: f64,
    #[pyo3(get, set)]
    pub reduced_tol_gap_rel: f64,
    #[pyo3(get, set)]
    pub reduced_tol_feas: f64,
    #[pyo3(get, set)]
    pub reduced_tol_infeas_abs: f64,
    #[pyo3(get, set)]
    pub reduced_tol_infeas_rel: f64,
    #[pyo3(get, set)]
    pub reduced_tol_ktratio: f64,
    #[pyo3(get, set)]
    pub equilibrate_enable: bool,
    #[pyo3(get, set)]
    pub equilibrate_max_iter: u32,
    #[pyo3(get, set)]
    pub equilibrate_min_scaling: f64,
    #[pyo3(get, set)]
    pub equilibrate_max_scaling: f64,
    #[pyo3(get, set)]
    pub linesearch_backtrack_step: f64,
    #[pyo3(get, set)]
    pub min_switch_step_length: f64,
    #[pyo3(get, set)]
    pub min_terminate_step_length: f64,
    #[pyo3(get, set)]
    pub max_threads: u32,
    #[pyo3(get, set)]
    pub direct_kkt_solver: bool,
    #[pyo3(get, set)]
    pub direct_solve_method: DirectSolveMethod,
    #[pyo3(get, set)]
    pub static_regularization_enable: bool,
    #[pyo3(get, set)]
    pub static_regularization_constant: f64,
    #[pyo3(get, set)]
    pub static_regularization_proportional: f64,
    #[pyo3(get, set)]
    pub dynamic_regularization_enable: bool,
    #[pyo3(get, set)]
    pub dynamic_regularization_eps: f64,
    #[pyo3(get, set)]
    pub dynamic_regularization_delta: f64,
    #[pyo3(get, set)]
    pub iterative_refinement_enable: bool,
    #[pyo3(get, set)]
    pub iterative_refinement_reltol: f64,
    #[pyo3(get, set)]
    pub iterative_refinement_abstol: f64,
    #[pyo3(get, set)]
    pub iterative_refinement_max_iter: u32,
    #[pyo3(get, set)]
    pub iterative_refinement_stop_ratio: f64,
    #[pyo3(get, set)]
    pub presolve_enable: bool,
    #[pyo3(get, set)]
    pub input_sparse_dropzeros: bool,
}

#[pymethods]
impl PyClarabelSettings {
    #[new]
    fn new() -> Self {
        let settings = DefaultSettings::<f64> {
            verbose: cfg!(debug_assertions),
            equilibrate_max_iter: 20,
            ..DefaultSettings::default()
        };
        Self::from_settings(&settings)
    }

    #[staticmethod]
    fn default() -> Self {
        Self::new()
    }
}

impl PyClarabelSettings {
    pub(crate) fn from_settings(settings: &DefaultSettings<f64>) -> Self {
        Self {
            max_iter: settings.max_iter,
            time_limit: settings.time_limit,
            verbose: settings.verbose,
            max_step_fraction: settings.max_step_fraction,
            tol_gap_abs: settings.tol_gap_abs,
            tol_gap_rel: settings.tol_gap_rel,
            tol_feas: settings.tol_feas,
            tol_infeas_abs: settings.tol_infeas_abs,
            tol_infeas_rel: settings.tol_infeas_rel,
            tol_ktratio: settings.tol_ktratio,
            reduced_tol_gap_abs: settings.reduced_tol_gap_abs,
            reduced_tol_gap_rel: settings.reduced_tol_gap_rel,
            reduced_tol_feas: settings.reduced_tol_feas,
            reduced_tol_infeas_abs: settings.reduced_tol_infeas_abs,
            reduced_tol_infeas_rel: settings.reduced_tol_infeas_rel,
            reduced_tol_ktratio: settings.reduced_tol_ktratio,
            equilibrate_enable: settings.equilibrate_enable,
            equilibrate_max_iter: settings.equilibrate_max_iter,
            equilibrate_min_scaling: settings.equilibrate_min_scaling,
            equilibrate_max_scaling: settings.equilibrate_max_scaling,
            linesearch_backtrack_step: settings.linesearch_backtrack_step,
            min_switch_step_length: settings.min_switch_step_length,
            min_terminate_step_length: settings.min_terminate_step_length,
            max_threads: settings.max_threads,
            direct_kkt_solver: settings.direct_kkt_solver,
            direct_solve_method: DirectSolveMethod::from_clarabel_name(
                &settings.direct_solve_method,
            ),
            static_regularization_enable: settings.static_regularization_enable,
            static_regularization_constant: settings.static_regularization_constant,
            static_regularization_proportional: settings.static_regularization_proportional,
            dynamic_regularization_enable: settings.dynamic_regularization_enable,
            dynamic_regularization_eps: settings.dynamic_regularization_eps,
            dynamic_regularization_delta: settings.dynamic_regularization_delta,
            iterative_refinement_enable: settings.iterative_refinement_enable,
            iterative_refinement_reltol: settings.iterative_refinement_reltol,
            iterative_refinement_abstol: settings.iterative_refinement_abstol,
            iterative_refinement_max_iter: settings.iterative_refinement_max_iter,
            iterative_refinement_stop_ratio: settings.iterative_refinement_stop_ratio,
            presolve_enable: settings.presolve_enable,
            input_sparse_dropzeros: settings.input_sparse_dropzeros,
        }
    }

    pub(crate) fn to_settings(&self) -> DefaultSettings<f64> {
        DefaultSettings::<f64> {
            max_iter: self.max_iter,
            time_limit: self.time_limit,
            verbose: self.verbose,
            max_step_fraction: self.max_step_fraction,
            tol_gap_abs: self.tol_gap_abs,
            tol_gap_rel: self.tol_gap_rel,
            tol_feas: self.tol_feas,
            tol_infeas_abs: self.tol_infeas_abs,
            tol_infeas_rel: self.tol_infeas_rel,
            tol_ktratio: self.tol_ktratio,
            reduced_tol_gap_abs: self.reduced_tol_gap_abs,
            reduced_tol_gap_rel: self.reduced_tol_gap_rel,
            reduced_tol_feas: self.reduced_tol_feas,
            reduced_tol_infeas_abs: self.reduced_tol_infeas_abs,
            reduced_tol_infeas_rel: self.reduced_tol_infeas_rel,
            reduced_tol_ktratio: self.reduced_tol_ktratio,
            equilibrate_enable: self.equilibrate_enable,
            equilibrate_max_iter: self.equilibrate_max_iter,
            equilibrate_min_scaling: self.equilibrate_min_scaling,
            equilibrate_max_scaling: self.equilibrate_max_scaling,
            linesearch_backtrack_step: self.linesearch_backtrack_step,
            min_switch_step_length: self.min_switch_step_length,
            min_terminate_step_length: self.min_terminate_step_length,
            max_threads: self.max_threads,
            direct_kkt_solver: self.direct_kkt_solver,
            direct_solve_method: self.direct_solve_method.as_clarabel_name().to_string(),
            static_regularization_enable: self.static_regularization_enable,
            static_regularization_constant: self.static_regularization_constant,
            static_regularization_proportional: self.static_regularization_proportional,
            dynamic_regularization_enable: self.dynamic_regularization_enable,
            dynamic_regularization_eps: self.dynamic_regularization_eps,
            dynamic_regularization_delta: self.dynamic_regularization_delta,
            iterative_refinement_enable: self.iterative_refinement_enable,
            iterative_refinement_reltol: self.iterative_refinement_reltol,
            iterative_refinement_abstol: self.iterative_refinement_abstol,
            iterative_refinement_max_iter: self.iterative_refinement_max_iter,
            iterative_refinement_stop_ratio: self.iterative_refinement_stop_ratio,
            presolve_enable: self.presolve_enable,
            input_sparse_dropzeros: self.input_sparse_dropzeros,
        }
    }
}

#[pyclass(name = "ClarabelOptions")]
pub struct PyClarabelOptions {
    pub inner: ClarabelOptions,
    verbosity: Verbosity,
    allow_almost_solved: bool,
    allow_max_iterations: bool,
    allow_max_time: bool,
    allow_insufficient_progress: bool,
    allow_callback_terminated: bool,
}

#[pymethods]
impl PyClarabelOptions {
    #[new]
    #[pyo3(signature = (
        verbosity=Verbosity::Silent,
        allow_almost_solved=true,
        allow_max_iterations=false,
        allow_max_time=false,
        allow_insufficient_progress=false,
        allow_callback_terminated=false,
        settings=None
    ))]
    fn new(
        verbosity: Verbosity,
        allow_almost_solved: bool,
        allow_max_iterations: bool,
        allow_max_time: bool,
        allow_insufficient_progress: bool,
        allow_callback_terminated: bool,
        settings: Option<&PyClarabelSettings>,
    ) -> PyResult<Self> {
        let builder = match settings {
            Some(settings) => ClarabelOptionsBuilder::with_clarabel_setting(settings.to_settings()),
            None => ClarabelOptionsBuilder::new(),
        };
        let opts = builder
            .verbosity(verbosity.into())
            .allow_almost_solved(allow_almost_solved)
            .allow_max_iterations(allow_max_iterations)
            .allow_max_time(allow_max_time)
            .allow_insufficient_progress(allow_insufficient_progress)
            .allow_callback_terminated(allow_callback_terminated)
            .build()
            .map_err(copp_err_to_py)?;
        Ok(Self {
            inner: opts,
            verbosity,
            allow_almost_solved,
            allow_max_iterations,
            allow_max_time,
            allow_insufficient_progress,
            allow_callback_terminated,
        })
    }

    #[getter]
    fn verbosity(&self) -> Verbosity {
        self.verbosity
    }

    #[getter]
    fn settings(&self) -> PyClarabelSettings {
        PyClarabelSettings::from_settings(self.inner.clarabel_settings())
    }

    #[getter]
    fn allow_almost_solved(&self) -> bool {
        self.allow_almost_solved
    }

    #[getter]
    fn allow_max_iterations(&self) -> bool {
        self.allow_max_iterations
    }

    #[getter]
    fn allow_max_time(&self) -> bool {
        self.allow_max_time
    }

    #[getter]
    fn allow_insufficient_progress(&self) -> bool {
        self.allow_insufficient_progress
    }

    #[getter]
    fn allow_callback_terminated(&self) -> bool {
        self.allow_callback_terminated
    }
}

#[pyfunction]
#[pyo3(signature = (output, path=None))]
pub fn set_verbosity_output(output: &str, path: Option<String>) -> PyResult<()> {
    let output = match output.to_lowercase().as_str() {
        "println" | "print" | "stdout" => VerbosityOutput::Println,
        "log" => VerbosityOutput::Log,
        "file" => VerbosityOutput::File(
            path.ok_or_else(|| PyValueError::new_err("path is required when output='file'"))?,
        ),
        other => {
            return Err(PyValueError::new_err(format!(
                "invalid verbosity output '{other}'. expected one of: println, log, file"
            )));
        }
    };
    rust_set_verbosity_output(output).map_err(copp_err_to_py)
}

#[pyfunction]
pub fn set_verbosity_log_file(path: String) -> PyResult<()> {
    rust_set_verbosity_log_file(path).map_err(copp_err_to_py)
}

#[pyfunction]
pub fn verbosity_output() -> (String, Option<String>) {
    match rust_verbosity_output() {
        VerbosityOutput::Println => ("println".to_string(), None),
        VerbosityOutput::Log => ("log".to_string(), None),
        VerbosityOutput::File(path) => ("file".to_string(), Some(path)),
    }
}
