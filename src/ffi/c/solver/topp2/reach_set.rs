//! C ABI wrappers for TOPP2 reachable-set construction.

use crate::ffi::c::core::status::{clear_last_error, panic_to_status};
use crate::ffi::c::formulation::Topp2Problem;
use crate::ffi::c::solver::topp2::ra::Topp2RaOptions;
use crate::ffi::c::{CoppRobot, CoppStatus, CoppVecF64};
use crate::solver::{
    reach_set2::{
        ReachSet2, reach_set2_backward as rust_reach_set2_backward,
        reach_set2_bidirectional as rust_reach_set2_bidirectional,
    },
    topp2_ra::Topp2ProblemBuilder,
};
use std::{
    panic::{AssertUnwindSafe, catch_unwind},
    ptr::NonNull,
};

/// Reachable intervals returned by TOPP2 reachable-set construction.
///
/// For each station, the feasible interval is `a_min[k] <= a[k] <= a_max[k]`
/// where `a = (ds/dt)^2`.  The field order is `a_max` first, then `a_min`.
/// Both vectors are library-owned and must
/// be released together with `copp_reach_set2_result_free`.
#[repr(C)]
#[derive(Debug)]
pub struct CoppReachSet2Result {
    /// Upper reachable bound for each station.
    pub a_max: CoppVecF64,
    /// Lower reachable bound for each station.
    pub a_min: CoppVecF64,
}

impl CoppReachSet2Result {
    const fn empty() -> Self {
        Self {
            a_max: CoppVecF64::empty(),
            a_min: CoppVecF64::empty(),
        }
    }

    fn from_reach_set(reach_set: ReachSet2) -> Self {
        Self {
            a_max: CoppVecF64::from_vec(reach_set.a_max),
            a_min: CoppVecF64::from_vec(reach_set.a_min),
        }
    }

    fn out_ptr(out: *mut Self) -> Result<NonNull<Self>, CoppStatus> {
        NonNull::new(out).ok_or(CoppStatus::NullPointer)
    }

    /// # Safety
    /// `out` must be valid for one `CoppReachSet2Result` write.
    unsafe fn write_empty_to(out: NonNull<Self>) {
        // SAFETY: The caller guarantees that `out` is valid for one write.
        unsafe {
            out.as_ptr().write(Self::empty());
        }
    }

    /// # Safety
    /// `out` must be valid for one `CoppReachSet2Result` write.
    unsafe fn write_reach_set_to(out: NonNull<Self>, reach_set: ReachSet2) {
        // SAFETY: The caller guarantees that `out` is valid for one write.
        unsafe {
            out.as_ptr().write(Self::from_reach_set(reach_set));
        }
    }

    fn free(self) {
        self.a_max.free();
        self.a_min.free();
    }
}

/// Compute backward-only TOPP2 reachable intervals.
///
/// This performs the same backward pass as `reach_set2_backward`: the
/// terminal boundary `a_final` is enforced, while the start boundary is only
/// checked for basic feasibility and is not imposed by a forward clipping pass.
///
/// On success, `out_result` owns two vectors with length
/// `idx_s_final - idx_s_start + 1`, ordered as `a_max` then `a_min`.  Release
/// them with `copp_reach_set2_result_free`.
///
/// # Example
/// The example below computes backward-only reachable intervals for a TOPP2
/// problem.
///
/// ```c
/// struct Topp2RaOptions options;
/// struct Topp2Problem problem = {robot, 0, n - 1, 0.0, 0.0};
/// struct CoppReachSet2Result reach = {0};
///
/// check(topp2_ra_default_options(&options));
/// check(copp_reach_set2_backward(problem, options, &reach));
/// copp_reach_set2_result_free(reach);
/// ```
///
/// # Safety
/// `problem.robot` must be a non-null handle returned by `copp_robot_create`
/// and must remain valid for the duration of this call.  The robot must not be
/// freed or mutated concurrently.  `out_result` must be valid for one
/// `CoppReachSet2Result` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_reach_set2_backward(
    problem: Topp2Problem,
    options: Topp2RaOptions,
    out_result: *mut CoppReachSet2Result,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    solve_reach_set2(problem, options, out_result, rust_reach_set2_backward)
}

/// Compute bidirectional TOPP2 reachable intervals.
///
/// This performs a backward pass followed by a forward clipping pass, enforcing
/// both boundary values `a_start` and `a_final`.
///
/// On success, `out_result` owns two vectors with length
/// `idx_s_final - idx_s_start + 1`, ordered as `a_max` then `a_min`.  Release
/// them with `copp_reach_set2_result_free`.
///
/// # Example
/// The example below computes bidirectional reachable intervals while
/// enforcing both endpoint boundary values.
///
/// ```c
/// struct Topp2RaOptions options;
/// struct CoppReachSet2Result reach = {0};
///
/// check(topp2_ra_default_options(&options));
/// check(copp_reach_set2_bidirectional(problem, options, &reach));
/// copp_reach_set2_result_free(reach);
/// ```
///
/// # Safety
/// `problem.robot` must be a non-null handle returned by `copp_robot_create`
/// and must remain valid for the duration of this call.  The robot must not be
/// freed or mutated concurrently.  `out_result` must be valid for one
/// `CoppReachSet2Result` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_reach_set2_bidirectional(
    problem: Topp2Problem,
    options: Topp2RaOptions,
    out_result: *mut CoppReachSet2Result,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    solve_reach_set2(problem, options, out_result, rust_reach_set2_bidirectional)
}

fn solve_reach_set2(
    problem: Topp2Problem,
    options: Topp2RaOptions,
    out_result: *mut CoppReachSet2Result,
    solve: fn(
        &crate::solver::topp2_ra::Topp2Problem<'_>,
        &crate::solver::topp2_ra::ReachSet2Options,
    ) -> Result<ReachSet2, crate::diag::CoppError>,
) -> CoppStatus {
    let out_result = match CoppReachSet2Result::out_ptr(out_result) {
        Ok(out_result) => out_result,
        Err(status) => return status,
    };
    // SAFETY: `out_result` was checked for null above; caller must provide a
    // valid output location as part of the C ABI contract.
    unsafe {
        CoppReachSet2Result::write_empty_to(out_result);
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: The C ABI contract requires `problem.robot` to be a live
        // handle created by this module for the duration of this call.
        let robot = unsafe { CoppRobot::robot(problem.robot) }.ok_or(CoppStatus::NullPointer)?;
        let problem = Topp2ProblemBuilder::new(
            robot,
            (problem.idx_s_start, problem.idx_s_final),
            (problem.a_start, problem.a_final),
        )
        .build()
        .map_err(|error| CoppStatus::from(&error))?;
        let options = options.build()?;
        let reach_set = solve(&problem, &options).map_err(|error| CoppStatus::from(&error))?;

        // SAFETY: Same checked output location as above.
        unsafe {
            CoppReachSet2Result::write_reach_set_to(out_result, reach_set);
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Release a reachable-set result returned by COPP.
///
/// # Safety
/// `result` must either be empty/null or have been returned by a successful
/// `copp_reach_set2_backward` / `copp_reach_set2_bidirectional` call.  Passing
/// arbitrary pointers or modified capacity fields is invalid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_reach_set2_result_free(result: CoppReachSet2Result) {
    result.free();
    clear_last_error();
}
