//! C ABI wrappers for TOPP2-RA solvers.

use crate::ffi::c::core::CoppVerbosity;
use crate::ffi::c::core::status::panic_to_status;
use crate::ffi::c::formulation::Topp2Problem;
use crate::ffi::c::{CoppRobot, CoppStatus, CoppVecF64};
use crate::solver::topp2_ra::{
    ReachSet2OptionsBuilder, Topp2ProblemBuilder, topp2_ra as rust_topp2_ra,
};
use std::panic::{AssertUnwindSafe, catch_unwind};

/// Options for TOPP2-RA reachability analysis.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Topp2RaOptions {
    /// Feasibility tolerance used by LP subproblems.
    pub lp_feas_tol: f64,
    /// Absolute tolerance for comparing interval bounds.
    pub a_cmp_abs_tol: f64,
    /// Relative tolerance for comparing interval bounds.
    pub a_cmp_rel_tol: f64,
    /// Verbosity level for diagnostics.
    pub verbosity: CoppVerbosity,
}

impl Topp2RaOptions {
    fn default_options() -> Self {
        let builder = ReachSet2OptionsBuilder::new();
        Self {
            lp_feas_tol: builder.lp_feas_tol,
            a_cmp_abs_tol: builder.a_cmp_abs_tol,
            a_cmp_rel_tol: builder.a_cmp_rel_tol,
            verbosity: builder.verbosity.into(),
        }
    }

    pub(crate) fn build(self) -> Result<crate::solver::topp2_ra::ReachSet2Options, CoppStatus> {
        ReachSet2OptionsBuilder::new()
            .lp_feas_tol(self.lp_feas_tol)
            .a_cmp_abs_tol(self.a_cmp_abs_tol)
            .a_cmp_rel_tol(self.a_cmp_rel_tol)
            .verbosity(self.verbosity.into())
            .build()
            .map_err(|error| CoppStatus::from(&error))
    }
}

/// Write default TOPP2-RA options into `out_options`.
///
/// # Safety
/// `out_options` must be valid for one `Topp2RaOptions` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn topp2_ra_default_options(out_options: *mut Topp2RaOptions) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if out_options.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: `out_options` was checked for null above and is expected to
        // be valid for one write by the C ABI contract.
        unsafe {
            out_options.write(Topp2RaOptions::default_options());
        }
        CoppStatus::Ok
    })) {
        Ok(status) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Solve a TOPP2-RA problem.
///
/// On success, `out_a` owns a COPP-allocated vector containing the solved
/// `a(s) = (ds/dt)^2` profile over the closed station interval and must be
/// released with `copp_vec_f64_free`.
///
/// # Safety
/// `problem.robot` must be a non-null handle returned by `copp_robot_create`
/// and must remain valid for the duration of this call.  The robot must not be
/// freed or mutated concurrently during the solve.  `out_a` must be valid for
/// one `CoppVecF64` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn topp2_ra(
    problem: Topp2Problem,
    options: Topp2RaOptions,
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
        let robot = unsafe { CoppRobot::robot(problem.robot) }.ok_or(CoppStatus::NullPointer)?;
        let problem = Topp2ProblemBuilder::new(
            robot,
            (problem.idx_s_start, problem.idx_s_final),
            (problem.a_start, problem.a_final),
        )
        .build()
        .map_err(|error| CoppStatus::from(&error))?;
        let options = options.build()?;
        let a = rust_topp2_ra(&problem, &options).map_err(|error| CoppStatus::from(&error))?;

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::diag::Verbosity;
    use crate::solver::topp2_ra::ReachSet2Options;
    use std::mem::MaybeUninit;

    fn assert_options_eq(actual: Topp2RaOptions, expected: &ReachSet2Options) {
        let Topp2RaOptions {
            lp_feas_tol,
            a_cmp_abs_tol,
            a_cmp_rel_tol,
            verbosity,
        } = actual;
        let ReachSet2Options {
            lp_feas_tol: expected_lp_feas_tol,
            a_cmp_abs_tol: expected_a_cmp_abs_tol,
            a_cmp_rel_tol: expected_a_cmp_rel_tol,
            verbosity: expected_verbosity,
        } = expected;

        assert_eq!(lp_feas_tol, *expected_lp_feas_tol);
        assert_eq!(a_cmp_abs_tol, *expected_a_cmp_abs_tol);
        assert_eq!(a_cmp_rel_tol, *expected_a_cmp_rel_tol);
        assert_eq!(verbosity, (*expected_verbosity).into());
    }

    #[test]
    fn default_options_match_rust_builder() {
        let mut out = MaybeUninit::<Topp2RaOptions>::uninit();
        let status = unsafe { topp2_ra_default_options(out.as_mut_ptr()) };
        assert_eq!(status, CoppStatus::Ok);
        let actual = unsafe { out.assume_init() };

        let expected = ReachSet2OptionsBuilder::new().build().unwrap();
        assert_options_eq(actual, &expected);
    }

    #[test]
    fn build_maps_all_fields() {
        let actual = Topp2RaOptions {
            lp_feas_tol: 2.0e-7,
            a_cmp_abs_tol: 3.0e-8,
            a_cmp_rel_tol: 4.0e-8,
            verbosity: CoppVerbosity::Trace,
        }
        .build()
        .unwrap();

        let ReachSet2Options {
            lp_feas_tol,
            a_cmp_abs_tol,
            a_cmp_rel_tol,
            verbosity,
        } = actual;

        assert_eq!(lp_feas_tol, 2.0e-7);
        assert_eq!(a_cmp_abs_tol, 3.0e-8);
        assert_eq!(a_cmp_rel_tol, 4.0e-8);
        assert!(verbosity == Verbosity::Trace);
    }
}
