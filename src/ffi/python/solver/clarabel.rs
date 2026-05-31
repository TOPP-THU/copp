//! Python wrappers for Clarabel-backed solver options and diagnostics.

use super::common::{PyVerbosity, VerbosityArg, normalize_token, parse_verbosity};
use crate::ffi::python::error::to_py_err;
use crate::ffi::python::interpolation::PyProfile3rd;
use crate::solver::copp2_socp::{ClarabelOptions as RustClarabelOptions, ClarabelOptionsBuilder};
use clarabel::solver::{DefaultSettings, DefaultSolution, LinearSolverInfo, SolverStatus};
use numpy::PyArray1;
use pyo3::Borrowed;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyAnyMethods;

/// Register Clarabel-related classes on the native [`PyModule`].
pub(super) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyClarabelDirectSolveMethod>()?;
    m.add_class::<PyClarabelSolverStatus>()?;
    m.add_class::<PyClarabelLinearSolverInfo>()?;
    m.add_class::<PyClarabelSettings>()?;
    m.add_class::<PyClarabelOptions>()?;
    m.add_class::<PyCopp3ClarabelResult>()?;
    Ok(())
}

/// Direct linear-solver method forwarded to Clarabel.
#[pyclass(
    name = "ClarabelDirectSolveMethod",
    module = "copp_py._native",
    eq,
    eq_int,
    rename_all = "SCREAMING_SNAKE_CASE",
    from_py_object
)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum PyClarabelDirectSolveMethod {
    /// Let Clarabel choose the direct solver.
    Auto,
    /// Use Clarabel's QDLDL backend.
    Qdldl,
    /// Use the optional FAER sparse backend when available.
    Faer,
    /// Use the optional MKL Pardiso backend when available.
    Mkl,
    /// Use the optional Panua Pardiso backend when available.
    Panua,
}

impl PyClarabelDirectSolveMethod {
    /// Convert a Clarabel solver-method string into a Python enum value.
    fn from_clarabel_name(name: &str) -> Self {
        match name {
            "qdldl" => Self::Qdldl,
            "faer" => Self::Faer,
            "mkl" => Self::Mkl,
            "panua" => Self::Panua,
            _ => Self::Auto,
        }
    }

    /// Convert the Python enum value into Clarabel's method string.
    fn as_clarabel_name(self) -> &'static str {
        match self {
            Self::Auto => "auto",
            Self::Qdldl => "qdldl",
            Self::Faer => "faer",
            Self::Mkl => "mkl",
            Self::Panua => "panua",
        }
    }
}

/// Internal parser for `ClarabelDirectSolveMethod | str | None` arguments.
#[derive(Clone, Copy)]
struct DirectSolveMethodArg(
    /// Parsed direct linear-solver method.
    PyClarabelDirectSolveMethod,
);

impl FromPyObject<'_, '_> for DirectSolveMethodArg {
    /// [`PyErr`] returned when parsing fails.
    type Error = PyErr;

    /// Parse enum values, accepted strings, or `None`.
    fn extract(obj: Borrowed<'_, '_, PyAny>) -> Result<Self, Self::Error> {
        if obj.is_none() {
            return Ok(Self(PyClarabelDirectSolveMethod::Auto));
        }

        if let Ok(method) = obj.extract::<PyClarabelDirectSolveMethod>() {
            return Ok(Self(method));
        }

        if let Ok(text) = obj.extract::<&str>() {
            return match normalize_token(text).as_str() {
                "auto" => Ok(Self(PyClarabelDirectSolveMethod::Auto)),
                "qdldl" => Ok(Self(PyClarabelDirectSolveMethod::Qdldl)),
                "faer" => Ok(Self(PyClarabelDirectSolveMethod::Faer)),
                "mkl" => Ok(Self(PyClarabelDirectSolveMethod::Mkl)),
                "panua" => Ok(Self(PyClarabelDirectSolveMethod::Panua)),
                _ => Err(PyValueError::new_err(
                    "`direct_solve_method` must be ClarabelDirectSolveMethod.AUTO, ClarabelDirectSolveMethod.QDLDL, ClarabelDirectSolveMethod.FAER, ClarabelDirectSolveMethod.MKL, ClarabelDirectSolveMethod.PANUA, or one of \"auto\", \"qdldl\", \"faer\", \"mkl\", \"panua\"",
                )),
            };
        }

        Err(PyValueError::new_err(
            "`direct_solve_method` must be a ClarabelDirectSolveMethod value or a string",
        ))
    }
}

/// Clarabel solver status returned by SOCP expert APIs.
#[pyclass(
    name = "ClarabelSolverStatus",
    module = "copp_py._native",
    eq,
    eq_int,
    rename_all = "SCREAMING_SNAKE_CASE",
    skip_from_py_object
)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum PyClarabelSolverStatus {
    /// Solver has not run.
    Unsolved,
    /// Solver terminated with a solution.
    Solved,
    /// Problem is primal infeasible.
    PrimalInfeasible,
    /// Problem is dual infeasible.
    DualInfeasible,
    /// Solver terminated with a reduced-accuracy solution.
    AlmostSolved,
    /// Problem is primal infeasible with reduced accuracy.
    AlmostPrimalInfeasible,
    /// Problem is dual infeasible with reduced accuracy.
    AlmostDualInfeasible,
    /// Iteration limit reached before an accepted solution or certificate.
    MaxIterations,
    /// Time limit reached before an accepted solution or certificate.
    MaxTime,
    /// Solver terminated with a numerical error.
    NumericalError,
    /// Solver terminated due to lack of progress.
    InsufficientProgress,
    /// Solver terminated by callback.
    CallbackTerminated,
}

impl PyClarabelSolverStatus {
    /// Convert from Clarabel's solver status enum.
    pub(super) fn from_rust(status: SolverStatus) -> Self {
        match status {
            SolverStatus::Unsolved => Self::Unsolved,
            SolverStatus::Solved => Self::Solved,
            SolverStatus::PrimalInfeasible => Self::PrimalInfeasible,
            SolverStatus::DualInfeasible => Self::DualInfeasible,
            SolverStatus::AlmostSolved => Self::AlmostSolved,
            SolverStatus::AlmostPrimalInfeasible => Self::AlmostPrimalInfeasible,
            SolverStatus::AlmostDualInfeasible => Self::AlmostDualInfeasible,
            SolverStatus::MaxIterations => Self::MaxIterations,
            SolverStatus::MaxTime => Self::MaxTime,
            SolverStatus::NumericalError => Self::NumericalError,
            SolverStatus::InsufficientProgress => Self::InsufficientProgress,
            SolverStatus::CallbackTerminated => Self::CallbackTerminated,
        }
    }
}

/// Python-owned linear-solver metadata reported by Clarabel.
#[pyclass(
    name = "ClarabelLinearSolverInfo",
    module = "copp_py._native",
    skip_from_py_object
)]
#[derive(Clone, Debug)]
pub(crate) struct PyClarabelLinearSolverInfo {
    /// Linear solver method name mapped to COPP's stable enum.
    method: PyClarabelDirectSolveMethod,
    /// Number of threads used by the linear solver.
    threads: usize,
    /// Whether the linear solver used a direct factorization method.
    direct: bool,
    /// Number of nonzeros in the linear system.
    nnz_a: usize,
    /// Number of nonzeros in the factored system.
    nnz_l: usize,
}

#[pymethods]
impl PyClarabelLinearSolverInfo {
    /// Return the direct linear-solver method.
    #[getter]
    fn method(&self) -> PyClarabelDirectSolveMethod {
        self.method
    }

    /// Return the number of solver threads used.
    #[getter]
    fn threads(&self) -> usize {
        self.threads
    }

    /// Return whether a direct factorization method was used.
    #[getter]
    fn direct(&self) -> bool {
        self.direct
    }

    /// Return the number of nonzeros in the linear system.
    #[getter]
    fn nnz_a(&self) -> usize {
        self.nnz_a
    }

    /// Return the number of nonzeros in the factored system.
    #[getter]
    fn nnz_l(&self) -> usize {
        self.nnz_l
    }

    /// Return a compact representation for interactive Python sessions.
    fn __repr__(&self) -> String {
        format!(
            "ClarabelLinearSolverInfo(method={:?}, threads={}, direct={}, nnz_a={}, nnz_l={})",
            self.method, self.threads, self.direct, self.nnz_a, self.nnz_l
        )
    }
}

impl From<LinearSolverInfo> for PyClarabelLinearSolverInfo {
    /// Convert Clarabel linear-solver metadata into a Python-owned object.
    fn from(value: LinearSolverInfo) -> Self {
        Self {
            method: PyClarabelDirectSolveMethod::from_clarabel_name(value.name.as_str()),
            threads: value.threads,
            direct: value.direct,
            nnz_a: value.nnzA,
            nnz_l: value.nnzL,
        }
    }
}

/// Python-owned raw Clarabel numerical settings.
///
/// This mirrors the Clarabel [`DefaultSettings<f64>`] fields exposed by the
/// C ABI. Defaults follow COPP's Rust Clarabel policy: Clarabel defaults with
/// `equilibrate_max_iter=20`, and `verbose=false` for the default
/// crate-level verbosity.
#[pyclass(
    name = "ClarabelSettings",
    module = "copp_py._native",
    skip_from_py_object
)]
#[derive(Clone, Debug)]
pub(crate) struct PyClarabelSettings {
    /// Maximum number of interior-point iterations.
    #[pyo3(get, set)]
    max_iter: u32,
    /// Maximum solve time in seconds.
    #[pyo3(get, set)]
    time_limit: f64,
    /// Enable Clarabel's internal solver log.
    #[pyo3(get, set)]
    verbose: bool,
    /// Maximum interior-point step length.
    #[pyo3(get, set)]
    max_step_fraction: f64,
    /// Absolute duality-gap tolerance.
    #[pyo3(get, set)]
    tol_gap_abs: f64,
    /// Relative duality-gap tolerance.
    #[pyo3(get, set)]
    tol_gap_rel: f64,
    /// Primal/dual feasibility tolerance.
    #[pyo3(get, set)]
    tol_feas: f64,
    /// Absolute infeasibility tolerance.
    #[pyo3(get, set)]
    tol_infeas_abs: f64,
    /// Relative infeasibility tolerance.
    #[pyo3(get, set)]
    tol_infeas_rel: f64,
    /// Kappa/tau tolerance.
    #[pyo3(get, set)]
    tol_ktratio: f64,
    /// Reduced absolute duality-gap tolerance for almost-solved status.
    #[pyo3(get, set)]
    reduced_tol_gap_abs: f64,
    /// Reduced relative duality-gap tolerance for almost-solved status.
    #[pyo3(get, set)]
    reduced_tol_gap_rel: f64,
    /// Reduced feasibility tolerance for almost-solved status.
    #[pyo3(get, set)]
    reduced_tol_feas: f64,
    /// Reduced absolute infeasibility tolerance for almost-solved status.
    #[pyo3(get, set)]
    reduced_tol_infeas_abs: f64,
    /// Reduced relative infeasibility tolerance for almost-solved status.
    #[pyo3(get, set)]
    reduced_tol_infeas_rel: f64,
    /// Reduced kappa/tau tolerance for almost-solved status.
    #[pyo3(get, set)]
    reduced_tol_ktratio: f64,
    /// Enable data equilibration before solving.
    #[pyo3(get, set)]
    equilibrate_enable: bool,
    /// Maximum number of equilibration iterations.
    #[pyo3(get, set)]
    equilibrate_max_iter: u32,
    /// Minimum equilibration scaling.
    #[pyo3(get, set)]
    equilibrate_min_scaling: f64,
    /// Maximum equilibration scaling.
    #[pyo3(get, set)]
    equilibrate_max_scaling: f64,
    /// Backtracking factor used by the line search.
    #[pyo3(get, set)]
    linesearch_backtrack_step: f64,
    /// Minimum step length for asymmetric-cone switching.
    #[pyo3(get, set)]
    min_switch_step_length: f64,
    /// Minimum step length for termination checks.
    #[pyo3(get, set)]
    min_terminate_step_length: f64,
    /// Maximum solver threads. `0` lets Clarabel choose.
    #[pyo3(get, set)]
    max_threads: u32,
    /// Whether to use a direct KKT solver. Clarabel currently requires `true`.
    #[pyo3(get, set)]
    direct_kkt_solver: bool,
    /// Direct linear solver method.
    direct_solve_method: PyClarabelDirectSolveMethod,
    /// Enable static KKT regularization.
    #[pyo3(get, set)]
    static_regularization_enable: bool,
    /// Static KKT regularization constant.
    #[pyo3(get, set)]
    static_regularization_constant: f64,
    /// Static KKT regularization proportional term.
    #[pyo3(get, set)]
    static_regularization_proportional: f64,
    /// Enable dynamic KKT regularization.
    #[pyo3(get, set)]
    dynamic_regularization_enable: bool,
    /// Dynamic regularization threshold.
    #[pyo3(get, set)]
    dynamic_regularization_eps: f64,
    /// Dynamic regularization shift.
    #[pyo3(get, set)]
    dynamic_regularization_delta: f64,
    /// Enable iterative refinement for direct solves.
    #[pyo3(get, set)]
    iterative_refinement_enable: bool,
    /// Iterative-refinement relative tolerance.
    #[pyo3(get, set)]
    iterative_refinement_reltol: f64,
    /// Iterative-refinement absolute tolerance.
    #[pyo3(get, set)]
    iterative_refinement_abstol: f64,
    /// Iterative-refinement maximum iterations.
    #[pyo3(get, set)]
    iterative_refinement_max_iter: u32,
    /// Iterative-refinement stalling ratio.
    #[pyo3(get, set)]
    iterative_refinement_stop_ratio: f64,
    /// Enable Clarabel presolve.
    #[pyo3(get, set)]
    presolve_enable: bool,
    /// Drop structural zeros from sparse inputs before solving.
    #[pyo3(get, set)]
    input_sparse_dropzeros: bool,
}

#[pymethods]
impl PyClarabelSettings {
    /// Construct raw Clarabel numerical settings.
    ///
    /// Defaults mirror `copp.ClarabelOptions().clarabel_settings`. Most users
    /// should tune only a small subset such as `max_iter`, `time_limit`, and
    /// tolerances.
    #[new]
    #[pyo3(
        signature = (
            *,
            max_iter = 200,
            time_limit = f64::INFINITY,
            verbose = false,
            max_step_fraction = 0.99,
            tol_gap_abs = 1.0e-8,
            tol_gap_rel = 1.0e-8,
            tol_feas = 1.0e-8,
            tol_infeas_abs = 1.0e-8,
            tol_infeas_rel = 1.0e-8,
            tol_ktratio = 1.0e-6,
            reduced_tol_gap_abs = 5.0e-5,
            reduced_tol_gap_rel = 5.0e-5,
            reduced_tol_feas = 1.0e-4,
            reduced_tol_infeas_abs = 5.0e-12,
            reduced_tol_infeas_rel = 5.0e-5,
            reduced_tol_ktratio = 1.0e-4,
            equilibrate_enable = true,
            equilibrate_max_iter = 20,
            equilibrate_min_scaling = 1.0e-4,
            equilibrate_max_scaling = 1.0e4,
            linesearch_backtrack_step = 0.8,
            min_switch_step_length = 0.1,
            min_terminate_step_length = 1.0e-4,
            max_threads = 0,
            direct_kkt_solver = true,
            direct_solve_method = DirectSolveMethodArg(PyClarabelDirectSolveMethod::Auto),
            static_regularization_enable = true,
            static_regularization_constant = 1.0e-8,
            static_regularization_proportional = f64::EPSILON * f64::EPSILON,
            dynamic_regularization_enable = true,
            dynamic_regularization_eps = 1.0e-13,
            dynamic_regularization_delta = 2.0e-7,
            iterative_refinement_enable = true,
            iterative_refinement_reltol = 1.0e-13,
            iterative_refinement_abstol = 1.0e-12,
            iterative_refinement_max_iter = 10,
            iterative_refinement_stop_ratio = 5.0,
            presolve_enable = true,
            input_sparse_dropzeros = false
        ),
        text_signature = "(*, max_iter=200, time_limit=1e999, verbose=False, max_step_fraction=0.99, tol_gap_abs=1e-08, tol_gap_rel=1e-08, tol_feas=1e-08, tol_infeas_abs=1e-08, tol_infeas_rel=1e-08, tol_ktratio=1e-06, reduced_tol_gap_abs=5e-05, reduced_tol_gap_rel=5e-05, reduced_tol_feas=0.0001, reduced_tol_infeas_abs=5e-12, reduced_tol_infeas_rel=5e-05, reduced_tol_ktratio=0.0001, equilibrate_enable=True, equilibrate_max_iter=20, equilibrate_min_scaling=0.0001, equilibrate_max_scaling=10000.0, linesearch_backtrack_step=0.8, min_switch_step_length=0.1, min_terminate_step_length=0.0001, max_threads=0, direct_kkt_solver=True, direct_solve_method='auto', static_regularization_enable=True, static_regularization_constant=1e-08, static_regularization_proportional=4.930380657631324e-32, dynamic_regularization_enable=True, dynamic_regularization_eps=1e-13, dynamic_regularization_delta=2e-07, iterative_refinement_enable=True, iterative_refinement_reltol=1e-13, iterative_refinement_abstol=1e-12, iterative_refinement_max_iter=10, iterative_refinement_stop_ratio=5.0, presolve_enable=True, input_sparse_dropzeros=False)"
    )]
    #[allow(clippy::too_many_arguments)]
    fn new(
        max_iter: u32,
        time_limit: f64,
        verbose: bool,
        max_step_fraction: f64,
        tol_gap_abs: f64,
        tol_gap_rel: f64,
        tol_feas: f64,
        tol_infeas_abs: f64,
        tol_infeas_rel: f64,
        tol_ktratio: f64,
        reduced_tol_gap_abs: f64,
        reduced_tol_gap_rel: f64,
        reduced_tol_feas: f64,
        reduced_tol_infeas_abs: f64,
        reduced_tol_infeas_rel: f64,
        reduced_tol_ktratio: f64,
        equilibrate_enable: bool,
        equilibrate_max_iter: u32,
        equilibrate_min_scaling: f64,
        equilibrate_max_scaling: f64,
        linesearch_backtrack_step: f64,
        min_switch_step_length: f64,
        min_terminate_step_length: f64,
        max_threads: u32,
        direct_kkt_solver: bool,
        direct_solve_method: DirectSolveMethodArg,
        static_regularization_enable: bool,
        static_regularization_constant: f64,
        static_regularization_proportional: f64,
        dynamic_regularization_enable: bool,
        dynamic_regularization_eps: f64,
        dynamic_regularization_delta: f64,
        iterative_refinement_enable: bool,
        iterative_refinement_reltol: f64,
        iterative_refinement_abstol: f64,
        iterative_refinement_max_iter: u32,
        iterative_refinement_stop_ratio: f64,
        presolve_enable: bool,
        input_sparse_dropzeros: bool,
    ) -> Self {
        Self {
            max_iter,
            time_limit,
            verbose,
            max_step_fraction,
            tol_gap_abs,
            tol_gap_rel,
            tol_feas,
            tol_infeas_abs,
            tol_infeas_rel,
            tol_ktratio,
            reduced_tol_gap_abs,
            reduced_tol_gap_rel,
            reduced_tol_feas,
            reduced_tol_infeas_abs,
            reduced_tol_infeas_rel,
            reduced_tol_ktratio,
            equilibrate_enable,
            equilibrate_max_iter,
            equilibrate_min_scaling,
            equilibrate_max_scaling,
            linesearch_backtrack_step,
            min_switch_step_length,
            min_terminate_step_length,
            max_threads,
            direct_kkt_solver,
            direct_solve_method: direct_solve_method.0,
            static_regularization_enable,
            static_regularization_constant,
            static_regularization_proportional,
            dynamic_regularization_enable,
            dynamic_regularization_eps,
            dynamic_regularization_delta,
            iterative_refinement_enable,
            iterative_refinement_reltol,
            iterative_refinement_abstol,
            iterative_refinement_max_iter,
            iterative_refinement_stop_ratio,
            presolve_enable,
            input_sparse_dropzeros,
        }
    }

    /// Return the direct linear-solver method.
    #[getter]
    fn direct_solve_method(&self) -> PyClarabelDirectSolveMethod {
        self.direct_solve_method
    }

    /// Set the direct linear-solver method from an enum value or string.
    #[setter]
    fn set_direct_solve_method(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.direct_solve_method = DirectSolveMethodArg::extract(value.as_borrowed())?.0;
        Ok(())
    }

    /// Return a compact representation for interactive Python sessions.
    fn __repr__(&self) -> String {
        format!(
            "ClarabelSettings(max_iter={}, time_limit={}, tol_gap_rel={}, tol_feas={}, direct_solve_method={:?})",
            self.max_iter,
            self.time_limit,
            self.tol_gap_rel,
            self.tol_feas,
            self.direct_solve_method
        )
    }
}

impl PyClarabelSettings {
    /// Build Python settings from a Rust Clarabel settings value.
    fn from_rust(settings: &DefaultSettings<f64>) -> Self {
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
            direct_solve_method: PyClarabelDirectSolveMethod::from_clarabel_name(
                settings.direct_solve_method.as_str(),
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

    /// Return COPP's finalized default Clarabel settings.
    fn default_policy() -> PyResult<Self> {
        let options = ClarabelOptionsBuilder::new().build().map_err(to_py_err)?;
        Ok(Self::from_rust(options.clarabel_settings()))
    }

    /// Convert Python settings into Clarabel's Rust settings struct.
    fn to_rust(&self) -> DefaultSettings<f64> {
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
            direct_solve_method: self.direct_solve_method.as_clarabel_name().to_owned(),
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

/// Python-owned shared options for Clarabel-backed SOCP solvers.
///
/// Rust defaults from [`ClarabelOptionsBuilder::new`]:
/// `verbosity=Verbosity.Silent`, `allow_almost_solved=true`,
/// `allow_max_iterations=false`, `allow_max_time=false`,
/// `allow_callback_terminated=false`, and
/// `allow_insufficient_progress=false`.
///
/// Low-level Clarabel numerical settings are stored as a
/// [`PyClarabelSettings`] value and default to COPP's finalized Rust settings.
#[pyclass(
    name = "ClarabelOptions",
    module = "copp_py._native",
    skip_from_py_object
)]
pub(crate) struct PyClarabelOptions {
    /// Solver diagnostic verbosity.
    verbosity: PyVerbosity,
    /// Accept Clarabel `AlmostSolved` as usable.
    allow_almost_solved: bool,
    /// Accept Clarabel `MaxIterations` as usable.
    allow_max_iterations: bool,
    /// Accept Clarabel `MaxTime` as usable.
    allow_max_time: bool,
    /// Accept Clarabel `CallbackTerminated` as usable.
    allow_callback_terminated: bool,
    /// Accept Clarabel `InsufficientProgress` as usable.
    allow_insufficient_progress: bool,
    /// Raw Clarabel numerical settings.
    clarabel_settings: PyClarabelSettings,
}

#[pymethods]
impl PyClarabelOptions {
    /// Construct shared Clarabel-backed solver options.
    ///
    /// Defaults mirror Rust: `verbosity=Verbosity.Silent`,
    /// `allow_almost_solved=true`, and all other `allow_*` switches disabled.
    #[new]
    #[pyo3(
        signature = (
            *,
            verbosity = VerbosityArg(PyVerbosity::Silent),
            allow_almost_solved = true,
            allow_max_iterations = false,
            allow_max_time = false,
            allow_callback_terminated = false,
            allow_insufficient_progress = false,
            clarabel_settings = None
        ),
        text_signature = "(*, verbosity='silent', allow_almost_solved=True, allow_max_iterations=False, allow_max_time=False, allow_callback_terminated=False, allow_insufficient_progress=False, clarabel_settings=None)"
    )]
    fn new(
        verbosity: VerbosityArg,
        allow_almost_solved: bool,
        allow_max_iterations: bool,
        allow_max_time: bool,
        allow_callback_terminated: bool,
        allow_insufficient_progress: bool,
        clarabel_settings: Option<PyRef<'_, PyClarabelSettings>>,
    ) -> PyResult<Self> {
        let options = Self {
            verbosity: verbosity.0,
            allow_almost_solved,
            allow_max_iterations,
            allow_max_time,
            allow_callback_terminated,
            allow_insufficient_progress,
            clarabel_settings: match clarabel_settings {
                Some(settings) => (*settings).clone(),
                None => PyClarabelSettings::default_policy()?,
            },
        };
        options.to_rust()?;
        Ok(options)
    }

    /// Return the diagnostic verbosity level.
    #[getter]
    fn verbosity(&self) -> PyVerbosity {
        self.verbosity
    }

    /// Set diagnostic verbosity from an enum value or string.
    #[setter]
    fn set_verbosity(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        let mut candidate = self.clone();
        candidate.verbosity = parse_verbosity(value)?;
        candidate.to_rust()?;
        self.verbosity = candidate.verbosity;
        Ok(())
    }

    /// Return whether `AlmostSolved` is accepted as usable.
    #[getter]
    fn allow_almost_solved(&self) -> bool {
        self.allow_almost_solved
    }

    /// Set whether `AlmostSolved` is accepted as usable.
    #[setter]
    fn set_allow_almost_solved(&mut self, value: bool) -> PyResult<()> {
        let mut candidate = self.clone();
        candidate.allow_almost_solved = value;
        candidate.to_rust()?;
        self.allow_almost_solved = value;
        Ok(())
    }

    /// Return whether `MaxIterations` is accepted as usable.
    #[getter]
    fn allow_max_iterations(&self) -> bool {
        self.allow_max_iterations
    }

    /// Set whether `MaxIterations` is accepted as usable.
    #[setter]
    fn set_allow_max_iterations(&mut self, value: bool) -> PyResult<()> {
        let mut candidate = self.clone();
        candidate.allow_max_iterations = value;
        candidate.to_rust()?;
        self.allow_max_iterations = value;
        Ok(())
    }

    /// Return whether `MaxTime` is accepted as usable.
    #[getter]
    fn allow_max_time(&self) -> bool {
        self.allow_max_time
    }

    /// Set whether `MaxTime` is accepted as usable.
    #[setter]
    fn set_allow_max_time(&mut self, value: bool) -> PyResult<()> {
        let mut candidate = self.clone();
        candidate.allow_max_time = value;
        candidate.to_rust()?;
        self.allow_max_time = value;
        Ok(())
    }

    /// Return whether `CallbackTerminated` is accepted as usable.
    #[getter]
    fn allow_callback_terminated(&self) -> bool {
        self.allow_callback_terminated
    }

    /// Set whether `CallbackTerminated` is accepted as usable.
    #[setter]
    fn set_allow_callback_terminated(&mut self, value: bool) -> PyResult<()> {
        let mut candidate = self.clone();
        candidate.allow_callback_terminated = value;
        candidate.to_rust()?;
        self.allow_callback_terminated = value;
        Ok(())
    }

    /// Return whether `InsufficientProgress` is accepted as usable.
    #[getter]
    fn allow_insufficient_progress(&self) -> bool {
        self.allow_insufficient_progress
    }

    /// Set whether `InsufficientProgress` is accepted as usable.
    #[setter]
    fn set_allow_insufficient_progress(&mut self, value: bool) -> PyResult<()> {
        let mut candidate = self.clone();
        candidate.allow_insufficient_progress = value;
        candidate.to_rust()?;
        self.allow_insufficient_progress = value;
        Ok(())
    }

    /// Return a copy of the raw Clarabel numerical settings.
    ///
    /// Assign a modified settings object back to `options.clarabel_settings`
    /// or pass it to the constructor to update solver behavior.
    #[getter]
    fn clarabel_settings(&self) -> PyClarabelSettings {
        self.clarabel_settings.clone()
    }

    /// Replace the raw Clarabel numerical settings.
    #[setter]
    fn set_clarabel_settings(&mut self, value: PyRef<'_, PyClarabelSettings>) -> PyResult<()> {
        let mut candidate = self.clone();
        candidate.clarabel_settings = (*value).clone();
        candidate.to_rust()?;
        self.clarabel_settings = candidate.clarabel_settings;
        Ok(())
    }

    /// Return a compact representation for interactive Python sessions.
    fn __repr__(&self) -> String {
        format!(
            "ClarabelOptions(verbosity={:?}, allow_almost_solved={}, allow_max_iterations={}, allow_max_time={}, allow_callback_terminated={}, allow_insufficient_progress={}, clarabel_settings={:?})",
            self.verbosity,
            self.allow_almost_solved,
            self.allow_max_iterations,
            self.allow_max_time,
            self.allow_callback_terminated,
            self.allow_insufficient_progress,
            self.clarabel_settings
        )
    }
}

impl Clone for PyClarabelOptions {
    /// Clone the plain option values.
    fn clone(&self) -> Self {
        Self {
            verbosity: self.verbosity,
            allow_almost_solved: self.allow_almost_solved,
            allow_max_iterations: self.allow_max_iterations,
            allow_max_time: self.allow_max_time,
            allow_callback_terminated: self.allow_callback_terminated,
            allow_insufficient_progress: self.allow_insufficient_progress,
            clarabel_settings: self.clarabel_settings.clone(),
        }
    }
}

impl PyClarabelOptions {
    /// Convert Python options into validated Rust Clarabel options.
    fn to_rust(&self) -> PyResult<RustClarabelOptions> {
        ClarabelOptionsBuilder::with_clarabel_setting(self.clarabel_settings.to_rust())
            .verbosity(self.verbosity.to_rust())
            .allow_almost_solved(self.allow_almost_solved)
            .allow_max_iterations(self.allow_max_iterations)
            .allow_max_time(self.allow_max_time)
            .allow_callback_terminated(self.allow_callback_terminated)
            .allow_insufficient_progress(self.allow_insufficient_progress)
            .build()
            .map_err(to_py_err)
    }
}

/// Python-owned expert result for third-order Clarabel solvers.
///
/// This mirrors the C ABI `Copp3SocpResult` but uses a Python-facing name that
/// describes the shared Clarabel result shape used by TOPP3-LP, TOPP3-SOCP,
/// and COPP3-SOCP expert APIs.
#[pyclass(name = "Copp3ClarabelResult", module = "copp_py._native")]
pub(crate) struct PyCopp3ClarabelResult {
    /// Accepted profile, when available.
    profile: Option<PyProfile3rd>,
    /// Raw Clarabel primal solution vector.
    x: Vec<f64>,
    /// Raw Clarabel dual solution vector.
    z: Vec<f64>,
    /// Raw Clarabel primal-cone slack vector.
    s: Vec<f64>,
    /// Final Clarabel solver status.
    solver_status: PyClarabelSolverStatus,
    /// Primal objective value reported by Clarabel.
    obj_val: f64,
    /// Dual objective value reported by Clarabel.
    obj_val_dual: f64,
    /// Clarabel solve time in seconds.
    solve_time: f64,
    /// Number of interior-point iterations.
    iterations: u32,
    /// Primal residual reported by Clarabel.
    r_prim: f64,
    /// Dual residual reported by Clarabel.
    r_dual: f64,
    /// Linear-solver metadata reported by Clarabel.
    linsolver: PyClarabelLinearSolverInfo,
    /// Weighted COPP objective value, when available for COPP3 solvers.
    objective_value: Option<f64>,
    /// Per-objective unweighted values, when available for COPP3 solvers.
    objective_terms: Option<Vec<f64>>,
}

#[pymethods]
impl PyCopp3ClarabelResult {
    /// Return the accepted profile, or `None` when unavailable.
    #[getter]
    fn profile(&self) -> Option<PyProfile3rd> {
        self.profile.clone()
    }

    /// Return the raw Clarabel primal solution vector.
    #[getter]
    fn x<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        PyArray1::from_vec(py, self.x.clone())
    }

    /// Return the raw Clarabel dual solution vector.
    #[getter]
    fn z<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        PyArray1::from_vec(py, self.z.clone())
    }

    /// Return the raw Clarabel primal-cone slack vector.
    #[getter]
    fn s<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        PyArray1::from_vec(py, self.s.clone())
    }

    /// Return the final Clarabel solver status.
    #[getter]
    fn solver_status(&self) -> PyClarabelSolverStatus {
        self.solver_status
    }

    /// Return the primal objective value reported by Clarabel.
    #[getter]
    fn obj_val(&self) -> f64 {
        self.obj_val
    }

    /// Return the dual objective value reported by Clarabel.
    #[getter]
    fn obj_val_dual(&self) -> f64 {
        self.obj_val_dual
    }

    /// Return Clarabel solve time in seconds.
    #[getter]
    fn solve_time(&self) -> f64 {
        self.solve_time
    }

    /// Return the number of interior-point iterations.
    #[getter]
    fn iterations(&self) -> u32 {
        self.iterations
    }

    /// Return the primal residual reported by Clarabel.
    #[getter]
    fn r_prim(&self) -> f64 {
        self.r_prim
    }

    /// Return the dual residual reported by Clarabel.
    #[getter]
    fn r_dual(&self) -> f64 {
        self.r_dual
    }

    /// Return linear-solver metadata reported by Clarabel.
    #[getter]
    fn linsolver(&self) -> PyClarabelLinearSolverInfo {
        self.linsolver.clone()
    }

    /// Return weighted COPP objective value, if available.
    #[getter]
    fn objective_value(&self) -> Option<f64> {
        self.objective_value
    }

    /// Return per-objective unweighted values, if available.
    #[getter]
    fn objective_terms<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.objective_terms
            .as_ref()
            .map(|terms| PyArray1::from_vec(py, terms.clone()))
    }

    /// Return a compact representation for interactive Python sessions.
    fn __repr__(&self) -> String {
        format!(
            "Copp3ClarabelResult(profile_available={}, solver_status={:?}, iterations={}, objective_value={:?})",
            self.profile.is_some(),
            self.solver_status,
            self.iterations,
            self.objective_value
        )
    }
}

impl PyCopp3ClarabelResult {
    /// Convert Rust expert outputs into a Python-owned third-order result.
    pub(super) fn from_solution(
        profile: Option<crate::copp::copp3::Topp3Profile>,
        solution: DefaultSolution<f64>,
        linsolver: LinearSolverInfo,
        objective_breakdown: Option<(f64, Vec<f64>)>,
    ) -> Self {
        let DefaultSolution {
            x,
            z,
            s,
            status,
            obj_val,
            obj_val_dual,
            solve_time,
            iterations,
            r_prim,
            r_dual,
        } = solution;
        let (objective_value, objective_terms) = match objective_breakdown {
            Some((value, terms)) => (Some(value), Some(terms)),
            None => (None, None),
        };

        Self {
            profile: profile.map(PyProfile3rd::from_rust),
            x,
            z,
            s,
            solver_status: PyClarabelSolverStatus::from_rust(status),
            obj_val,
            obj_val_dual,
            solve_time,
            iterations,
            r_prim,
            r_dual,
            linsolver: linsolver.into(),
            objective_value,
            objective_terms,
        }
    }
}

/// Convert an optional Python Clarabel options object into validated Rust options.
pub(super) fn clarabel_options_or_default(
    options: Option<PyRef<'_, PyClarabelOptions>>,
) -> PyResult<RustClarabelOptions> {
    match options {
        Some(options) => options.to_rust(),
        None => ClarabelOptionsBuilder::new().build().map_err(to_py_err),
    }
}
