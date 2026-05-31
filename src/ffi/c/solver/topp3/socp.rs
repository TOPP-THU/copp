//! C ABI wrappers for TOPP3-SOCP solvers.

use crate::copp::copp3::opt3::topp3_socp::{
    topp3_socp as rust_topp3_socp, topp3_socp_expert_with_info as rust_topp3_socp_expert,
};
use crate::ffi::c::core::status::panic_to_status;
use crate::ffi::c::solver::copp3::socp::Copp3SocpResult;
use crate::ffi::c::{CoppClarabelOptions, CoppProfile3rd, CoppStatus, Topp3Problem};
use std::panic::{AssertUnwindSafe, catch_unwind};

/// Solve a TOPP3-SOCP problem.
///
/// This C ABI builds an internal TOPP3 problem from the borrowed `Topp3Problem`,
/// forwards `options` to the Clarabel backend, and returns only the accepted
/// third-order profile.
///
/// On success, `out_profile` owns COPP-allocated `a` and `b` vectors and must
/// be released with `copp_profile_3rd_free`.
///
/// # Example
/// The example below solves TOPP3-SOCP and releases the accepted third-order
/// profile.
///
/// ```c
/// struct CoppClarabelOptions options;
/// struct CoppProfile3rd profile = {{0}, {0}, 0, 0};
///
/// check(copp_clarabel_default_options(&options));
/// check(topp3_socp(problem, options, &profile));
/// copp_profile_3rd_free(profile);
/// ```
///
/// # Safety
/// `problem.robot` must be a non-null mutable handle returned by
/// `copp_robot_create` and must remain valid for the duration of this call.
/// The solver builds the internal problem by linearizing third-order constraints,
/// which mutates the robot's internal constraint cache. `out_profile` must be
/// valid for one `CoppProfile3rd` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn topp3_socp(
    problem: Topp3Problem,
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
        // SAFETY: The C ABI contract requires the borrowed robot and
        // linearization slice inside `problem` to be valid for this call.
        let problem = unsafe { problem.build_rust()? };
        let profile =
            rust_topp3_socp(&problem, &options).map_err(|error| CoppStatus::from(&error))?;

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

/// Solve a TOPP3-SOCP problem and always return Clarabel diagnostics.
///
/// This expert entry follows the advanced-diagnostics convention: true
/// runtime failures still return a non-OK `CoppStatus`, but non-accepted
/// Clarabel statuses are reported inside `out_result` with
/// `has_profile == false` and the function returns `COPP_STATUS_OK`.
///
/// TOPP3-SOCP has no user-provided COPP objective list, so
/// `out_result.objective_value` is NaN and `out_result.objective_terms` is an
/// empty vector. Raw Clarabel objective values and solver diagnostics are still
/// populated.
///
/// Use the same advanced-diagnostics pattern as \ref copp2_socp_expert. The
/// result type is `Copp3SocpResult`, and `has_profile` replaces `has_a`.
///
/// # Safety
/// Same safety requirements as `topp3_socp`. `out_result` must be valid for
/// one `Copp3SocpResult` write and must later be released with
/// `copp3_socp_result_free`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn topp3_socp_expert(
    problem: Topp3Problem,
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
        // SAFETY: The C ABI contract requires the borrowed robot and
        // linearization slice inside `problem` to be valid for this call.
        let problem = unsafe { problem.build_rust()? };
        let expert =
            rust_topp3_socp_expert(&problem, &options).map_err(|error| CoppStatus::from(&error))?;
        let result =
            Copp3SocpResult::from_solution(expert.result, expert.solution, expert.linsolver, None);

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
