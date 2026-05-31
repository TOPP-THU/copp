//! C ABI wrappers for COPP3-SOCP solvers.

use std::panic::{AssertUnwindSafe, catch_unwind};

use clarabel::solver::{DefaultSolution, LinearSolverInfo};

use crate::copp::copp3::Topp3Profile;
use crate::copp::copp3::opt3::copp3_socp::{
    copp3_socp as rust_copp3_socp, copp3_socp_expert_with_info as rust_copp3_socp_expert,
    objective_value_copp3_opt,
};

use crate::ffi::c::core::status::{clear_last_error, panic_to_status};
use crate::ffi::c::{
    Copp3Problem, CoppClarabelLinearSolverInfo, CoppClarabelOptions, CoppClarabelSolverStatus,
    CoppProfile3rd, CoppStatus, CoppVecF64,
};

/// Compact COPP3-SOCP expert result returned to C.
///
/// `has_profile` is true only when the Clarabel status is accepted by
/// `CoppClarabelOptions`. When true, `profile` owns COPP-allocated `a` and `b`
/// vectors and the whole result must be released with
/// `copp3_socp_result_free`. When false, `profile` is empty and diagnostics
/// still describe the Clarabel run.
#[repr(C)]
#[derive(Debug)]
pub struct Copp3SocpResult {
    /// Whether `profile` contains an accepted third-order profile.
    pub has_profile: bool,
    /// Accepted third-order profile, or an empty profile when
    /// `has_profile == false`.
    pub profile: CoppProfile3rd,
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
    /// Weighted COPP objective value computed from accepted profile; NaN
    /// otherwise.
    pub objective_value: f64,
    /// Per-objective unweighted values computed from accepted profile.
    pub objective_terms: CoppVecF64,
}

impl Copp3SocpResult {
    pub(crate) fn empty() -> Self {
        Self {
            has_profile: false,
            profile: CoppProfile3rd::empty(),
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

    pub(crate) fn from_solution(
        profile: Option<Topp3Profile>,
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
            has_profile: profile.is_some(),
            profile: profile.map_or_else(CoppProfile3rd::empty, |profile| {
                CoppProfile3rd::from_parts(profile.a, profile.b, profile.num_stationary)
            }),
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
        self.profile.free();
        self.x.free();
        self.z.free();
        self.s.free();
        self.objective_terms.free();
    }
}

/// Solve a COPP3-SOCP problem.
///
/// This C ABI builds an internal COPP3 problem from the borrowed `Copp3Problem`,
/// forwards `options` to the Clarabel backend, and returns only the accepted
/// third-order profile.
///
/// Supported objective kinds are `Time`, `Linear`, `ThermalEnergy`, and
/// `TotalVariationTorque`. Torque objectives use the robot's stored
/// inverse-dynamics callback when provided, otherwise point dynamics
/// (`tau = ddq`) is used.
///
/// On success, `out_profile` owns COPP-allocated `a` and `b` vectors and must
/// be released with `copp_profile_3rd_free`.
///
/// # Example
/// The example below solves COPP3-SOCP with Clarabel options and releases the
/// accepted third-order profile.
///
/// ```c
/// struct CoppClarabelOptions options;
/// struct Copp3Problem problem = {
///     robot, 0, (struct CoppSliceF64){a_seed.data, a_seed.len},
///     0.0, 0.0, 0.0, 0.0, 1, 1, 1e-10,
///     objectives, num_objectives,
/// };
/// struct CoppProfile3rd profile = {{0}, {0}, 0, 0};
///
/// check(copp_clarabel_default_options(&options));
/// check(copp3_socp(problem, options, &profile));
/// copp_profile_3rd_free(profile);
/// ```
///
/// # Safety
/// `problem.robot` must be a non-null mutable handle returned by
/// `copp_robot_create` and must remain valid for the duration of this call.
/// `problem.objectives` must be valid for `problem.num_objectives` reads when
/// non-zero. If the robot has a stored inverse-dynamics callback, it must
/// remain valid for this call. The solver builds the internal problem by
/// linearizing third-order constraints, which mutates the robot's internal
/// constraint cache. `out_profile` must be valid for one `CoppProfile3rd`
/// write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp3_socp(
    problem: Copp3Problem,
    options: CoppClarabelOptions,
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
        let options = options.build();
        // SAFETY: The C ABI contract requires the borrowed robot, slices, and
        // objectives inside `problem` to remain valid during this call.
        let profile = unsafe {
            problem.with_rust_problem(true, |problem| {
                rust_copp3_socp(&problem, &options).map_err(|error| CoppStatus::from(&error))
            })?
        };

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

/// Solve a COPP3-SOCP problem and always return Clarabel diagnostics.
///
/// This expert entry follows the advanced-diagnostics convention: true
/// runtime failures still return a non-OK `CoppStatus`, but non-accepted
/// Clarabel statuses are reported inside `out_result` with
/// `has_profile == false` and the function returns `COPP_STATUS_OK`.
///
/// Use the same advanced-diagnostics pattern as \ref copp2_socp_expert. The
/// result type is `Copp3SocpResult`, and `has_profile` replaces `has_a`.
///
/// # Safety
/// Same safety requirements as `copp3_socp`. `out_result` must be valid for
/// one `Copp3SocpResult` write and must later be released with
/// `copp3_socp_result_free`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp3_socp_expert(
    problem: Copp3Problem,
    options: CoppClarabelOptions,
    out_result: *mut Copp3SocpResult,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if out_result.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }
    // SAFETY: `out_result` was checked for null above and is expected to be
    // valid for one write by the C ABI contract.
    unsafe {
        out_result.write(Copp3SocpResult::empty());
    }

    match catch_unwind(AssertUnwindSafe(|| {
        let options = options.build();
        // SAFETY: The C ABI contract requires the borrowed robot, slices, and
        // objectives inside `problem` to remain valid during this call.
        let result = unsafe {
            problem.with_rust_problem(true, |problem| {
                let expert = rust_copp3_socp_expert(&problem, &options)
                    .map_err(|error| CoppStatus::from(&error))?;
                let objective_breakdown = expert
                    .result
                    .as_ref()
                    .map(|profile| objective_value_copp3_opt(&problem, profile.as_parts()));
                Ok(Copp3SocpResult::from_solution(
                    expert.result,
                    expert.solution,
                    expert.linsolver,
                    objective_breakdown,
                ))
            })?
        };

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

/// Release memory owned by a `Copp3SocpResult`.
///
/// Passing an already-freed result is invalid. Prefer freeing the full result
/// with this function instead of freeing `result.profile` manually.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp3_socp_result_free(result: Copp3SocpResult) {
    result.free();
    clear_last_error();
}
