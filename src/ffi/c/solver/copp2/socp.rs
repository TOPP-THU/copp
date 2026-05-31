//! C ABI wrappers for COPP2-SOCP solvers.

use crate::copp::copp2::opt2::copp2_socp::{
    copp2_socp_expert_with_info as rust_copp2_socp_expert, objective_value_copp2_opt,
};
use crate::copp::{ClarabelOptions, clarabel_to_copp3_solution};
use crate::ffi::c::core::CoppVerbosity;
use crate::ffi::c::core::status::{clear_last_error, panic_to_status};
use crate::ffi::c::formulation::{Copp2Problem, objective_slice};
use crate::ffi::c::{CoppProfile3rd, CoppRobot, CoppSliceF64, CoppStatus, CoppVecF64};
use crate::solver::copp2_socp::{
    ClarabelOptionsBuilder, Copp2ProblemBuilder, copp2_socp as rust_copp2_socp,
};
use clarabel::solver::{DefaultSettings, DefaultSolution, LinearSolverInfo, SolverStatus};
use std::panic::{AssertUnwindSafe, catch_unwind};

/// Direct linear solver method forwarded to Clarabel.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CoppClarabelDirectSolveMethod {
    /// Let Clarabel choose the direct solver.
    Auto = 0,
    /// Use Clarabel's QDLDL backend.
    Qdldl = 1,
    /// Use the optional FAER sparse backend when compiled into Clarabel.
    Faer = 2,
    /// Use the optional MKL Pardiso backend when compiled into Clarabel.
    Mkl = 3,
    /// Use the optional Panua Pardiso backend when compiled into Clarabel.
    Panua = 4,
}

/// Clarabel solver status returned by SOCP expert results.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CoppClarabelSolverStatus {
    /// Solver has not run.
    Unsolved = 0,
    /// Solver terminated with a solution.
    Solved = 1,
    /// Problem is primal infeasible.
    PrimalInfeasible = 2,
    /// Problem is dual infeasible.
    DualInfeasible = 3,
    /// Solver terminated with a reduced-accuracy solution.
    AlmostSolved = 4,
    /// Problem is primal infeasible with reduced accuracy.
    AlmostPrimalInfeasible = 5,
    /// Problem is dual infeasible with reduced accuracy.
    AlmostDualInfeasible = 6,
    /// Iteration limit reached before an accepted solution or certificate.
    MaxIterations = 7,
    /// Time limit reached before an accepted solution or certificate.
    MaxTime = 8,
    /// Solver terminated with a numerical error.
    NumericalError = 9,
    /// Solver terminated due to lack of progress.
    InsufficientProgress = 10,
    /// Solver terminated by callback.
    CallbackTerminated = 11,
}

impl From<SolverStatus> for CoppClarabelSolverStatus {
    fn from(status: SolverStatus) -> Self {
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

impl CoppClarabelDirectSolveMethod {
    fn from_settings(settings: &DefaultSettings<f64>) -> Self {
        Self::from_clarabel_name(settings.direct_solve_method.as_str())
    }

    fn from_clarabel_name(name: &str) -> Self {
        match name {
            "qdldl" => Self::Qdldl,
            "faer" => Self::Faer,
            "mkl" => Self::Mkl,
            "panua" => Self::Panua,
            _ => Self::Auto,
        }
    }

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

/// Linear solver metadata reported by Clarabel.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CoppClarabelLinearSolverInfo {
    /// Linear solver method name mapped to COPP's stable enum.
    pub method: CoppClarabelDirectSolveMethod,
    /// Number of threads used by the linear solver.
    pub threads: usize,
    /// Whether the linear solver used a direct factorization method.
    pub direct: bool,
    /// Number of nonzeros in the linear system.
    pub nnz_a: usize,
    /// Number of nonzeros in the factored system.
    pub nnz_l: usize,
}

impl CoppClarabelLinearSolverInfo {
    pub(crate) const fn empty() -> Self {
        Self {
            method: CoppClarabelDirectSolveMethod::Auto,
            threads: 0,
            direct: false,
            nnz_a: 0,
            nnz_l: 0,
        }
    }
}

impl From<LinearSolverInfo> for CoppClarabelLinearSolverInfo {
    fn from(info: LinearSolverInfo) -> Self {
        Self {
            method: CoppClarabelDirectSolveMethod::from_clarabel_name(info.name.as_str()),
            threads: info.threads,
            direct: info.direct,
            nnz_a: info.nnzA,
            nnz_l: info.nnzL,
        }
    }
}

/// Compact COPP2-SOCP expert result returned to C.
///
/// `has_a` is true only when the Clarabel status is accepted by
/// `CoppClarabelOptions`. When true, `a` owns a COPP-allocated vector and the
/// whole result must be released with `copp2_socp_result_free`. When false,
/// `a` is empty and diagnostics still describe the Clarabel run.
#[repr(C)]
#[derive(Debug)]
pub struct Copp2SocpResult {
    /// Whether `a` contains an accepted `a(s) = (ds/dt)^2` profile.
    pub has_a: bool,
    /// Accepted `a` profile, or an empty vector when `has_a == false`.
    pub a: CoppVecF64,
    /// Raw Clarabel primal solution vector.
    pub x: CoppVecF64,
    /// Raw Clarabel dual solution vector.
    pub z: CoppVecF64,
    /// Raw Clarabel primal-cone slack vector.
    pub s: CoppVecF64,
    /// Final Clarabel solver status.
    pub solver_status: CoppClarabelSolverStatus,
    /// Primal objective value reported by Clarabel.
    pub obj_val: f64,
    /// Dual objective value reported by Clarabel.
    pub obj_val_dual: f64,
    /// Clarabel solve time in seconds.
    pub solve_time: f64,
    /// Number of interior-point iterations.
    pub iterations: u32,
    /// Primal residual reported by Clarabel.
    pub r_prim: f64,
    /// Dual residual reported by Clarabel.
    pub r_dual: f64,
    /// Linear solver metadata reported by Clarabel.
    pub linsolver: CoppClarabelLinearSolverInfo,
    /// Weighted COPP objective value computed from accepted `a`; NaN otherwise.
    pub objective_value: f64,
    /// Per-objective unweighted values computed from accepted `a`.
    pub objective_terms: CoppVecF64,
}

impl Copp2SocpResult {
    fn empty() -> Self {
        Self {
            has_a: false,
            a: CoppVecF64::empty(),
            x: CoppVecF64::empty(),
            z: CoppVecF64::empty(),
            s: CoppVecF64::empty(),
            solver_status: CoppClarabelSolverStatus::Unsolved,
            obj_val: f64::NAN,
            obj_val_dual: f64::NAN,
            solve_time: 0.0,
            iterations: 0,
            r_prim: f64::NAN,
            r_dual: f64::NAN,
            linsolver: CoppClarabelLinearSolverInfo::empty(),
            objective_value: f64::NAN,
            objective_terms: CoppVecF64::empty(),
        }
    }

    fn from_solution(
        a_profile: Option<Vec<f64>>,
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
        let (objective_value, objective_terms) =
            objective_breakdown.unwrap_or((f64::NAN, Vec::new()));

        Self {
            has_a: a_profile.is_some(),
            a: a_profile.map_or_else(CoppVecF64::empty, CoppVecF64::from_vec),
            x: CoppVecF64::from_vec(x),
            z: CoppVecF64::from_vec(z),
            s: CoppVecF64::from_vec(s),
            solver_status: status.into(),
            obj_val,
            obj_val_dual,
            solve_time,
            iterations,
            r_prim,
            r_dual,
            linsolver: linsolver.into(),
            objective_value,
            objective_terms: CoppVecF64::from_vec(objective_terms),
        }
    }

    fn free(self) {
        self.a.free();
        self.x.free();
        self.z.free();
        self.s.free();
        self.objective_terms.free();
    }
}

/// Advanced raw Clarabel solver settings.
///
/// This mirrors the `clarabel::solver::DefaultSettings<f64>` fields available
/// in the current build. Optional Clarabel backends, such as FAER or Pardiso,
/// are selected through `direct_solve_method`; if the linked Clarabel was not
/// compiled with that backend, the solve returns a solver/options error.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CoppClarabelSettings {
    /// Maximum number of interior-point iterations.
    pub max_iter: u32,
    /// Maximum solve time in seconds.
    pub time_limit: f64,
    /// Enable Clarabel's internal solver log.
    pub verbose: bool,
    /// Maximum interior-point step length.
    pub max_step_fraction: f64,
    /// Absolute duality-gap tolerance.
    pub tol_gap_abs: f64,
    /// Relative duality-gap tolerance.
    pub tol_gap_rel: f64,
    /// Primal/dual feasibility tolerance.
    pub tol_feas: f64,
    /// Absolute infeasibility tolerance.
    pub tol_infeas_abs: f64,
    /// Relative infeasibility tolerance.
    pub tol_infeas_rel: f64,
    /// Kappa/tau tolerance.
    pub tol_ktratio: f64,
    /// Reduced absolute duality-gap tolerance for almost-solved status.
    pub reduced_tol_gap_abs: f64,
    /// Reduced relative duality-gap tolerance for almost-solved status.
    pub reduced_tol_gap_rel: f64,
    /// Reduced feasibility tolerance for almost-solved status.
    pub reduced_tol_feas: f64,
    /// Reduced absolute infeasibility tolerance for almost-solved status.
    pub reduced_tol_infeas_abs: f64,
    /// Reduced relative infeasibility tolerance for almost-solved status.
    pub reduced_tol_infeas_rel: f64,
    /// Reduced kappa/tau tolerance for almost-solved status.
    pub reduced_tol_ktratio: f64,
    /// Enable data equilibration before solving.
    pub equilibrate_enable: bool,
    /// Maximum number of equilibration iterations.
    pub equilibrate_max_iter: u32,
    /// Minimum equilibration scaling.
    pub equilibrate_min_scaling: f64,
    /// Maximum equilibration scaling.
    pub equilibrate_max_scaling: f64,
    /// Backtracking factor used by the line search.
    pub linesearch_backtrack_step: f64,
    /// Minimum step length for asymmetric-cone switching.
    pub min_switch_step_length: f64,
    /// Minimum step length for termination checks.
    pub min_terminate_step_length: f64,
    /// Maximum solver threads. `0` lets Clarabel choose.
    pub max_threads: u32,
    /// Whether to use a direct KKT solver. Clarabel currently requires `true`.
    pub direct_kkt_solver: bool,
    /// Direct linear solver method.
    pub direct_solve_method: CoppClarabelDirectSolveMethod,
    /// Enable static KKT regularization.
    pub static_regularization_enable: bool,
    /// Static KKT regularization constant.
    pub static_regularization_constant: f64,
    /// Static KKT regularization proportional term.
    pub static_regularization_proportional: f64,
    /// Enable dynamic KKT regularization.
    pub dynamic_regularization_enable: bool,
    /// Dynamic regularization threshold.
    pub dynamic_regularization_eps: f64,
    /// Dynamic regularization shift.
    pub dynamic_regularization_delta: f64,
    /// Enable iterative refinement for direct solves.
    pub iterative_refinement_enable: bool,
    /// Iterative-refinement relative tolerance.
    pub iterative_refinement_reltol: f64,
    /// Iterative-refinement absolute tolerance.
    pub iterative_refinement_abstol: f64,
    /// Iterative-refinement maximum iterations.
    pub iterative_refinement_max_iter: u32,
    /// Iterative-refinement stalling ratio.
    pub iterative_refinement_stop_ratio: f64,
    /// Enable Clarabel presolve.
    pub presolve_enable: bool,
    /// Drop structural zeros from sparse inputs before solving.
    pub input_sparse_dropzeros: bool,
}

impl CoppClarabelSettings {
    fn from_settings(settings: &DefaultSettings<f64>) -> Self {
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
            direct_solve_method: CoppClarabelDirectSolveMethod::from_settings(settings),
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

    fn build(self) -> DefaultSettings<f64> {
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
            // Clarabel stores this setting as an owned String, so the short
            // conversion from our C enum name is required by its current API.
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

/// Shared Clarabel options for COPP SOCP solvers.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CoppClarabelOptions {
    /// Crate-level verbosity for COPP assembly diagnostics.
    pub verbosity: CoppVerbosity,
    /// Accept Clarabel status `AlmostSolved` as usable.
    pub allow_almost_solved: bool,
    /// Accept Clarabel status `MaxIterations` as usable.
    pub allow_max_iterations: bool,
    /// Accept Clarabel status `MaxTime` as usable.
    pub allow_max_time: bool,
    /// Accept Clarabel status `CallbackTerminated` as usable.
    pub allow_callback_terminated: bool,
    /// Accept Clarabel status `InsufficientProgress` as usable.
    pub allow_insufficient_progress: bool,
    /// Advanced raw Clarabel solver settings.
    pub clarabel_settings: CoppClarabelSettings,
}

impl CoppClarabelOptions {
    pub(crate) fn default_options() -> Result<Self, CoppStatus> {
        let options = ClarabelOptionsBuilder::new()
            .allow_almost_solved(true)
            .build()
            .map_err(|error| CoppStatus::from(&error))?;

        Ok(Self {
            verbosity: options.verbosity().into(),
            allow_almost_solved: true,
            allow_max_iterations: false,
            allow_max_time: false,
            allow_callback_terminated: false,
            allow_insufficient_progress: false,
            clarabel_settings: CoppClarabelSettings::from_settings(options.clarabel_settings()),
        })
    }

    pub(crate) fn build(self) -> ClarabelOptions {
        ClarabelOptions {
            verbosity: self.verbosity.into(),
            clarabel_settings: self.clarabel_settings.build(),
            allow_almost_solved: self.allow_almost_solved,
            allow_max_iterations: self.allow_max_iterations,
            allow_max_time: self.allow_max_time,
            allow_callback_terminated: self.allow_callback_terminated,
            allow_insufficient_progress: self.allow_insufficient_progress,
        }
    }
}

/// Write default shared Clarabel options into `out_options`.
///
/// Defaults use COPP's default Clarabel settings and accept `Solved` and
/// `AlmostSolved` statuses.
///
/// # Safety
/// `out_options` must be valid for one `CoppClarabelOptions` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_clarabel_default_options(
    out_options: *mut CoppClarabelOptions,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if out_options.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        let options = CoppClarabelOptions::default_options()?;
        // SAFETY: `out_options` was checked for null above and is expected to
        // be valid for one write by the C ABI contract.
        unsafe {
            out_options.write(options);
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Extract a second-order COPP profile from a Clarabel primal solution.
///
/// Use this helper with an expert SOCP result when the raw primal `x` vector
/// should be converted back into the accepted `a(s) = dot{s}^2` profile. The
/// output has length `s_len`; missing tail values are filled with zero and all
/// entries are clamped to be non-negative.
///
/// # Example
/// The example below converts an expert raw primal vector into a second-order
/// `a(s)` profile.
///
/// ```c
/// struct CoppVecF64 a = {0};
/// check(copp_clarabel_solution_to_profile_2nd(
///     n,
///     (struct CoppSliceF64){expert.x.data, expert.x.len},
///     &a));
/// copp_vec_f64_free(a);
/// ```
///
/// # Safety
/// `x.data` must be valid for `x.len` reads when `x.len` is non-zero.
/// `out_a` must be valid for one `CoppVecF64` write and must later be
/// released with `copp_vec_f64_free`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_clarabel_solution_to_profile_2nd(
    s_len: usize,
    x: CoppSliceF64,
    out_a: *mut CoppVecF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    let out_a = match CoppVecF64::out_ptr(out_a) {
        Ok(out_a) => out_a,
        Err(status) => return status,
    };
    // SAFETY: `out_a` was checked for null above and is expected to be valid
    // for one write by the C ABI contract.
    unsafe {
        CoppVecF64::write_empty_to(out_a);
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: The C ABI contract requires non-empty input slices to point
        // to valid contiguous `double` arrays for the duration of this call.
        let x = unsafe { x.as_slice()? };
        let mut a = vec![0.0; s_len];
        for (dst, src) in a.iter_mut().zip(x.iter()) {
            *dst = src.max(0.0);
        }
        // SAFETY: Same checked output location as above.
        unsafe {
            CoppVecF64::write_vec_to(out_a, a);
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Extract a third-order COPP profile from a Clarabel primal solution.
///
/// Use this helper with a TOPP3/COPP3 expert SOCP result when the raw primal
/// `x` vector should be converted back into the accepted
/// `(a, b, num_stationary)` profile. The station vector `s` defines the profile
/// length and interval spacing.
///
/// # Example
/// The example below converts an expert raw primal vector into a third-order
/// `(a, b)` profile.
///
/// ```c
/// struct CoppProfile3rd profile = {{0}, {0}, 0, 0};
/// check(copp_clarabel_solution_to_profile_3rd(
///     (struct CoppSliceF64){s, n},
///     (struct CoppSliceF64){expert.x.data, expert.x.len},
///     expert.profile.num_stationary_start,
///     expert.profile.num_stationary_end,
///     &profile));
/// copp_profile_3rd_free(profile);
/// ```
///
/// # Safety
/// `s.data` and `x.data` must be valid for their declared lengths when those
/// lengths are non-zero. `out_profile` must be valid for one `CoppProfile3rd`
/// write and must later be released with `copp_profile_3rd_free`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_clarabel_solution_to_profile_3rd(
    s: CoppSliceF64,
    x: CoppSliceF64,
    num_stationary_start: usize,
    num_stationary_end: usize,
    out_profile: *mut CoppProfile3rd,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if out_profile.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }
    // SAFETY: `out_profile` was checked for null above and is expected to be
    // valid for one write by the C ABI contract.
    unsafe {
        out_profile.write(CoppProfile3rd::empty());
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: The C ABI contract requires non-empty input slices to point
        // to valid contiguous `double` arrays for the duration of this call.
        let s = unsafe { s.as_slice()? };
        // SAFETY: Same input-slice contract as above.
        let x = unsafe { x.as_slice()? };
        if s.len() < 2 {
            return Err(CoppStatus::InvalidShape);
        }
        if num_stationary_start + num_stationary_end >= s.len() {
            return Err(CoppStatus::InvalidArgument);
        }
        let min_x_len = s.len().checked_mul(2).ok_or(CoppStatus::InvalidShape)?;
        if x.len() < min_x_len {
            return Err(CoppStatus::InvalidShape);
        }

        let profile = clarabel_to_copp3_solution(x, s, (num_stationary_start, num_stationary_end));
        // SAFETY: Same checked output location as above.
        unsafe {
            out_profile.write(CoppProfile3rd::from_parts(
                profile.a,
                profile.b,
                profile.num_stationary,
            ));
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Solve a COPP2-SOCP problem.
///
/// This C ABI builds an internal COPP2 problem from the borrowed `Copp2Problem`,
/// forwards `options` to the Clarabel backend, and returns only the optimized
/// `a(s) = (ds/dt)^2` profile.
///
/// Supported objective kinds are `Time`, `Linear`, `ThermalEnergy`, and
/// `TotalVariationTorque`.  The torque objectives use the robot's stored
/// inverse-dynamics callback when provided, otherwise point dynamics
/// (`tau = ddq`) is used.
///
/// On success, `out_a` owns a COPP-allocated vector over the closed station
/// interval and must be released with `copp_vec_f64_free`.
///
/// # Example
/// The example below solves COPP2-SOCP with Clarabel options and releases the
/// optimized `a(s)` profile.
///
/// ```c
/// struct CoppClarabelOptions options;
/// struct Copp2Problem problem = {
///     robot, 0, n - 1, 0.0, 0.0, objectives, num_objectives,
/// };
/// struct CoppVecF64 a = {0};
///
/// check(copp_clarabel_default_options(&options));
/// options.allow_almost_solved = true;
/// check(copp2_socp(problem, options, &a));
/// copp_vec_f64_free(a);
/// ```
///
/// # Safety
/// `problem.robot` must be a non-null handle returned by `copp_robot_create`
/// and must remain valid for the duration of this call. `problem.objectives`
/// must be valid for `problem.num_objectives` reads when
/// `problem.num_objectives` is non-zero. If the robot has a stored
/// inverse-dynamics callback, it must remain valid for this call. The robot
/// must not be freed or mutated concurrently during the solve. `out_a` must
/// be valid for one `CoppVecF64` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp2_socp(
    problem: Copp2Problem,
    options: CoppClarabelOptions,
    out_a: *mut CoppVecF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    let out_a = match CoppVecF64::out_ptr(out_a) {
        Ok(out_a) => out_a,
        Err(status) => return status,
    };
    // SAFETY: `out_a` was checked for null above; caller must provide a valid
    // output location as part of the C ABI contract.
    unsafe {
        CoppVecF64::write_empty_to(out_a);
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: The C ABI contract requires `problem.robot` to be a live
        // handle created by this module for the duration of this call.
        let inner = unsafe { CoppRobot::inner(problem.robot) }.ok_or(CoppStatus::NullPointer)?;
        let robot = &inner.robot;
        // SAFETY: Objective descriptors are borrowed only during this call.
        let objectives = unsafe { objective_slice(problem.objectives, problem.num_objectives)? };
        let objectives = objectives
            .iter()
            .map(|objective| {
                // SAFETY: Objective descriptors are borrowed only during this call.
                unsafe { objective.as_rust_objective(true) }
            })
            .collect::<Result<Vec<_>, _>>()?;
        let options = options.build();
        let problem = Copp2ProblemBuilder::new(
            robot,
            (problem.idx_s_start, problem.idx_s_final),
            (problem.a_start, problem.a_final),
            &objectives,
        )
        .build()
        .map_err(|error| CoppStatus::from(&error))?;
        let a = rust_copp2_socp(&problem, &options).map_err(|error| CoppStatus::from(&error))?;

        // SAFETY: Same checked output location as above.
        unsafe {
            CoppVecF64::write_vec_to(out_a, a);
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Solve a COPP2-SOCP problem and always return Clarabel diagnostics.
///
/// This expert entry follows the advanced-diagnostics convention: true
/// runtime failures still return a non-OK `CoppStatus`, but non-accepted
/// Clarabel statuses are reported inside `out_result` with `has_a == false`
/// and the function returns `COPP_STATUS_OK`.
///
/// # Example
/// The example below distinguishes C ABI failures from non-accepted Clarabel
/// solves and then frees all expert buffers.
///
/// ```c
/// struct Copp2SocpResult result = {0};
/// enum CoppStatus status = copp2_socp_expert(problem, options, &result);
/// if (status != COPP_STATUS_OK) {
///     fprintf(stderr, "COPP failed: %s\n", copp_last_error_message());
/// } else if (!result.has_a) {
///     fprintf(stderr,
///             "Clarabel status=%d iterations=%u r_prim=%.3e r_dual=%.3e\n",
///             (int)result.solver_status,
///             result.iterations,
///             result.r_prim,
///             result.r_dual);
/// } else {
///     printf("accepted profile length: %zu\n", result.a.len);
/// }
/// copp2_socp_result_free(result);
/// ```
///
/// # Safety
/// Same safety requirements as `copp2_socp`. `out_result` must be valid for
/// one `Copp2SocpResult` write and must later be released with
/// `copp2_socp_result_free`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp2_socp_expert(
    problem: Copp2Problem,
    options: CoppClarabelOptions,
    out_result: *mut Copp2SocpResult,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if out_result.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }
    // SAFETY: `out_result` was checked for null above and is expected to be
    // valid for one write by the C ABI contract.
    unsafe {
        out_result.write(Copp2SocpResult::empty());
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: The C ABI contract requires `problem.robot` to be a live
        // handle created by this module for the duration of this call.
        let inner = unsafe { CoppRobot::inner(problem.robot) }.ok_or(CoppStatus::NullPointer)?;
        let robot = &inner.robot;
        // SAFETY: Objective descriptors are borrowed only during this call.
        let objectives = unsafe { objective_slice(problem.objectives, problem.num_objectives)? };
        let objectives = objectives
            .iter()
            .map(|objective| {
                // SAFETY: Objective descriptors are borrowed only during this call.
                unsafe { objective.as_rust_objective(true) }
            })
            .collect::<Result<Vec<_>, _>>()?;
        let options = options.build();
        let problem = Copp2ProblemBuilder::new(
            robot,
            (problem.idx_s_start, problem.idx_s_final),
            (problem.a_start, problem.a_final),
            &objectives,
        )
        .build()
        .map_err(|error| CoppStatus::from(&error))?;
        let expert =
            rust_copp2_socp_expert(&problem, &options).map_err(|error| CoppStatus::from(&error))?;
        let objective_breakdown = expert
            .result
            .as_ref()
            .map(|a| objective_value_copp2_opt(robot, problem.idx_s_interval.0, &objectives, a));
        let result = Copp2SocpResult::from_solution(
            expert.result,
            expert.solution,
            expert.linsolver,
            objective_breakdown,
        );
        // SAFETY: Same checked output location as above.
        unsafe {
            out_result.write(result);
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Release memory owned by a `Copp2SocpResult`.
///
/// Passing an already-freed result is invalid. Prefer freeing the full result
/// with this function instead of freeing `result.a` manually.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp2_socp_result_free(result: Copp2SocpResult) {
    result.free();
    clear_last_error();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::diag::Verbosity;
    use std::mem::MaybeUninit;

    fn custom_settings(method: &str) -> DefaultSettings<f64> {
        DefaultSettings::<f64> {
            max_iter: 77,
            time_limit: 12.5,
            verbose: true,
            max_step_fraction: 0.91,
            tol_gap_abs: 1.0e-7,
            tol_gap_rel: 2.0e-7,
            tol_feas: 3.0e-7,
            tol_infeas_abs: 4.0e-7,
            tol_infeas_rel: 5.0e-7,
            tol_ktratio: 6.0e-7,
            reduced_tol_gap_abs: 7.0e-5,
            reduced_tol_gap_rel: 8.0e-5,
            reduced_tol_feas: 9.0e-5,
            reduced_tol_infeas_abs: 1.0e-4,
            reduced_tol_infeas_rel: 1.1e-4,
            reduced_tol_ktratio: 1.2e-4,
            equilibrate_enable: false,
            equilibrate_max_iter: 13,
            equilibrate_min_scaling: 1.0e-3,
            equilibrate_max_scaling: 1.0e3,
            linesearch_backtrack_step: 0.73,
            min_switch_step_length: 1.0e-2,
            min_terminate_step_length: 2.0e-2,
            max_threads: 4,
            direct_kkt_solver: true,
            direct_solve_method: method.to_owned(),
            static_regularization_enable: false,
            static_regularization_constant: 1.0e-9,
            static_regularization_proportional: 2.0e-9,
            dynamic_regularization_enable: false,
            dynamic_regularization_eps: 3.0e-9,
            dynamic_regularization_delta: 4.0e-9,
            iterative_refinement_enable: false,
            iterative_refinement_reltol: 5.0e-9,
            iterative_refinement_abstol: 6.0e-9,
            iterative_refinement_max_iter: 9,
            iterative_refinement_stop_ratio: 7.0,
            presolve_enable: false,
            input_sparse_dropzeros: true,
        }
    }

    fn assert_settings_eq(actual: &DefaultSettings<f64>, expected: &DefaultSettings<f64>) {
        assert_eq!(actual.max_iter, expected.max_iter);
        assert_eq!(actual.time_limit, expected.time_limit);
        assert_eq!(actual.verbose, expected.verbose);
        assert_eq!(actual.max_step_fraction, expected.max_step_fraction);
        assert_eq!(actual.tol_gap_abs, expected.tol_gap_abs);
        assert_eq!(actual.tol_gap_rel, expected.tol_gap_rel);
        assert_eq!(actual.tol_feas, expected.tol_feas);
        assert_eq!(actual.tol_infeas_abs, expected.tol_infeas_abs);
        assert_eq!(actual.tol_infeas_rel, expected.tol_infeas_rel);
        assert_eq!(actual.tol_ktratio, expected.tol_ktratio);
        assert_eq!(actual.reduced_tol_gap_abs, expected.reduced_tol_gap_abs);
        assert_eq!(actual.reduced_tol_gap_rel, expected.reduced_tol_gap_rel);
        assert_eq!(actual.reduced_tol_feas, expected.reduced_tol_feas);
        assert_eq!(
            actual.reduced_tol_infeas_abs,
            expected.reduced_tol_infeas_abs
        );
        assert_eq!(
            actual.reduced_tol_infeas_rel,
            expected.reduced_tol_infeas_rel
        );
        assert_eq!(actual.reduced_tol_ktratio, expected.reduced_tol_ktratio);
        assert_eq!(actual.equilibrate_enable, expected.equilibrate_enable);
        assert_eq!(actual.equilibrate_max_iter, expected.equilibrate_max_iter);
        assert_eq!(
            actual.equilibrate_min_scaling,
            expected.equilibrate_min_scaling
        );
        assert_eq!(
            actual.equilibrate_max_scaling,
            expected.equilibrate_max_scaling
        );
        assert_eq!(
            actual.linesearch_backtrack_step,
            expected.linesearch_backtrack_step
        );
        assert_eq!(
            actual.min_switch_step_length,
            expected.min_switch_step_length
        );
        assert_eq!(
            actual.min_terminate_step_length,
            expected.min_terminate_step_length
        );
        assert_eq!(actual.max_threads, expected.max_threads);
        assert_eq!(actual.direct_kkt_solver, expected.direct_kkt_solver);
        assert_eq!(actual.direct_solve_method, expected.direct_solve_method);
        assert_eq!(
            actual.static_regularization_enable,
            expected.static_regularization_enable
        );
        assert_eq!(
            actual.static_regularization_constant,
            expected.static_regularization_constant
        );
        assert_eq!(
            actual.static_regularization_proportional,
            expected.static_regularization_proportional
        );
        assert_eq!(
            actual.dynamic_regularization_enable,
            expected.dynamic_regularization_enable
        );
        assert_eq!(
            actual.dynamic_regularization_eps,
            expected.dynamic_regularization_eps
        );
        assert_eq!(
            actual.dynamic_regularization_delta,
            expected.dynamic_regularization_delta
        );
        assert_eq!(
            actual.iterative_refinement_enable,
            expected.iterative_refinement_enable
        );
        assert_eq!(
            actual.iterative_refinement_reltol,
            expected.iterative_refinement_reltol
        );
        assert_eq!(
            actual.iterative_refinement_abstol,
            expected.iterative_refinement_abstol
        );
        assert_eq!(
            actual.iterative_refinement_max_iter,
            expected.iterative_refinement_max_iter
        );
        assert_eq!(
            actual.iterative_refinement_stop_ratio,
            expected.iterative_refinement_stop_ratio
        );
        assert_eq!(actual.presolve_enable, expected.presolve_enable);
        assert_eq!(
            actual.input_sparse_dropzeros,
            expected.input_sparse_dropzeros
        );
    }

    fn assert_c_settings_eq(actual: CoppClarabelSettings, expected: &DefaultSettings<f64>) {
        let CoppClarabelSettings {
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
            direct_solve_method,
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
        } = actual;

        assert_eq!(max_iter, expected.max_iter);
        assert_eq!(time_limit, expected.time_limit);
        assert_eq!(verbose, expected.verbose);
        assert_eq!(max_step_fraction, expected.max_step_fraction);
        assert_eq!(tol_gap_abs, expected.tol_gap_abs);
        assert_eq!(tol_gap_rel, expected.tol_gap_rel);
        assert_eq!(tol_feas, expected.tol_feas);
        assert_eq!(tol_infeas_abs, expected.tol_infeas_abs);
        assert_eq!(tol_infeas_rel, expected.tol_infeas_rel);
        assert_eq!(tol_ktratio, expected.tol_ktratio);
        assert_eq!(reduced_tol_gap_abs, expected.reduced_tol_gap_abs);
        assert_eq!(reduced_tol_gap_rel, expected.reduced_tol_gap_rel);
        assert_eq!(reduced_tol_feas, expected.reduced_tol_feas);
        assert_eq!(reduced_tol_infeas_abs, expected.reduced_tol_infeas_abs);
        assert_eq!(reduced_tol_infeas_rel, expected.reduced_tol_infeas_rel);
        assert_eq!(reduced_tol_ktratio, expected.reduced_tol_ktratio);
        assert_eq!(equilibrate_enable, expected.equilibrate_enable);
        assert_eq!(equilibrate_max_iter, expected.equilibrate_max_iter);
        assert_eq!(equilibrate_min_scaling, expected.equilibrate_min_scaling);
        assert_eq!(equilibrate_max_scaling, expected.equilibrate_max_scaling);
        assert_eq!(
            linesearch_backtrack_step,
            expected.linesearch_backtrack_step
        );
        assert_eq!(min_switch_step_length, expected.min_switch_step_length);
        assert_eq!(
            min_terminate_step_length,
            expected.min_terminate_step_length
        );
        assert_eq!(max_threads, expected.max_threads);
        assert_eq!(direct_kkt_solver, expected.direct_kkt_solver);
        assert_eq!(
            direct_solve_method,
            CoppClarabelDirectSolveMethod::from_settings(expected)
        );
        assert_eq!(
            static_regularization_enable,
            expected.static_regularization_enable
        );
        assert_eq!(
            static_regularization_constant,
            expected.static_regularization_constant
        );
        assert_eq!(
            static_regularization_proportional,
            expected.static_regularization_proportional
        );
        assert_eq!(
            dynamic_regularization_enable,
            expected.dynamic_regularization_enable
        );
        assert_eq!(
            dynamic_regularization_eps,
            expected.dynamic_regularization_eps
        );
        assert_eq!(
            dynamic_regularization_delta,
            expected.dynamic_regularization_delta
        );
        assert_eq!(
            iterative_refinement_enable,
            expected.iterative_refinement_enable
        );
        assert_eq!(
            iterative_refinement_reltol,
            expected.iterative_refinement_reltol
        );
        assert_eq!(
            iterative_refinement_abstol,
            expected.iterative_refinement_abstol
        );
        assert_eq!(
            iterative_refinement_max_iter,
            expected.iterative_refinement_max_iter
        );
        assert_eq!(
            iterative_refinement_stop_ratio,
            expected.iterative_refinement_stop_ratio
        );
        assert_eq!(presolve_enable, expected.presolve_enable);
        assert_eq!(input_sparse_dropzeros, expected.input_sparse_dropzeros);
    }

    fn assert_c_options_eq(actual: CoppClarabelOptions, expected: &ClarabelOptions) {
        let CoppClarabelOptions {
            verbosity,
            allow_almost_solved,
            allow_max_iterations,
            allow_max_time,
            allow_callback_terminated,
            allow_insufficient_progress,
            clarabel_settings,
        } = actual;

        assert_eq!(verbosity, expected.verbosity.into());
        assert_eq!(allow_almost_solved, expected.allow_almost_solved);
        assert_eq!(allow_max_iterations, expected.allow_max_iterations);
        assert_eq!(allow_max_time, expected.allow_max_time);
        assert_eq!(
            allow_callback_terminated,
            expected.allow_callback_terminated
        );
        assert_eq!(
            allow_insufficient_progress,
            expected.allow_insufficient_progress
        );
        assert_c_settings_eq(clarabel_settings, &expected.clarabel_settings);
    }

    #[test]
    fn clarabel_direct_solve_method_names_roundtrip() {
        for (name, method) in [
            ("auto", CoppClarabelDirectSolveMethod::Auto),
            ("qdldl", CoppClarabelDirectSolveMethod::Qdldl),
            ("faer", CoppClarabelDirectSolveMethod::Faer),
            ("mkl", CoppClarabelDirectSolveMethod::Mkl),
            ("panua", CoppClarabelDirectSolveMethod::Panua),
        ] {
            assert_eq!(
                CoppClarabelDirectSolveMethod::from_clarabel_name(name),
                method
            );
            assert_eq!(method.as_clarabel_name(), name);
        }
        assert_eq!(
            CoppClarabelDirectSolveMethod::from_clarabel_name("unknown"),
            CoppClarabelDirectSolveMethod::Auto
        );
    }

    #[test]
    fn clarabel_settings_map_all_fields_both_directions() {
        let settings = custom_settings("faer");
        let c_settings = CoppClarabelSettings::from_settings(&settings);

        assert_c_settings_eq(c_settings, &settings);
        assert_settings_eq(&c_settings.build(), &settings);
    }

    #[test]
    fn clarabel_default_options_match_rust_policy() {
        let expected = ClarabelOptionsBuilder::new().build().unwrap();
        assert!(expected.is_allow(SolverStatus::Solved));
        assert!(expected.is_allow(SolverStatus::AlmostSolved));
        assert!(!expected.is_allow(SolverStatus::MaxIterations));
        assert!(!expected.is_allow(SolverStatus::MaxTime));
        assert!(!expected.is_allow(SolverStatus::CallbackTerminated));
        assert!(!expected.is_allow(SolverStatus::InsufficientProgress));

        let mut out = MaybeUninit::<CoppClarabelOptions>::uninit();
        let status = unsafe { copp_clarabel_default_options(out.as_mut_ptr()) };
        assert_eq!(status, CoppStatus::Ok);
        let actual = unsafe { out.assume_init() };

        assert_c_options_eq(actual, &expected);
    }

    #[test]
    fn clarabel_options_build_maps_all_fields() {
        let actual = CoppClarabelOptions {
            verbosity: CoppVerbosity::Debug,
            allow_almost_solved: false,
            allow_max_iterations: true,
            allow_max_time: true,
            allow_callback_terminated: false,
            allow_insufficient_progress: true,
            clarabel_settings: CoppClarabelSettings::from_settings(&custom_settings("mkl")),
        }
        .build();

        let ClarabelOptions {
            verbosity,
            clarabel_settings,
            allow_almost_solved,
            allow_max_iterations,
            allow_max_time,
            allow_callback_terminated,
            allow_insufficient_progress,
        } = actual;

        assert!(verbosity == Verbosity::Debug);
        assert!(!allow_almost_solved);
        assert!(allow_max_iterations);
        assert!(allow_max_time);
        assert!(!allow_callback_terminated);
        assert!(allow_insufficient_progress);
        assert_settings_eq(&clarabel_settings, &custom_settings("mkl"));
    }
}
