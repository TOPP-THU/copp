//! C ABI wrappers for interpolation utilities.

use crate::copp::InterpolationMode;
use crate::copp::copp2::stable::basic::{a_to_b_topp2, s_to_t_topp2, t_to_s_topp2};
use crate::copp::copp3::stable::basic::{s_to_t_topp3, t_to_s_topp3};
use crate::copp::copp3::stable::topp3_lp::force_positive_a;
use crate::diag::CoppError;
use crate::ffi::c::core::status::panic_to_status;
use crate::ffi::c::{CoppSliceF64, CoppSliceMutF64, CoppStatus, CoppVecF64};
use std::panic::{AssertUnwindSafe, catch_unwind};

/// Compute TOPP2 segment profile `b` from node profile `a`.
///
/// On success, `out_b` owns a COPP-allocated vector and must be released with
/// `copp_vec_f64_free`.
///
/// # Safety
/// `s.data` and `a.data` must be valid for `len` reads when their lengths are
/// non-zero. `out_b` must be valid for one `CoppVecF64` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_a_to_b_2nd(
    s: CoppSliceF64,
    a: CoppSliceF64,
    out_b: *mut CoppVecF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    let out_b = match CoppVecF64::out_ptr(out_b) {
        Ok(out_b) => out_b,
        Err(status) => return status,
    };
    // SAFETY: `out_b` was checked for null above; caller must provide a valid
    // output location as part of the C ABI contract.
    unsafe {
        CoppVecF64::write_empty_to(out_b);
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: The C ABI contract requires non-empty input slices to point
        // to valid contiguous `double` arrays for the duration of this call.
        let s = unsafe { s.as_slice()? };
        // SAFETY: Same input-slice contract as above.
        let a = unsafe { a.as_slice()? };
        let b = a_to_b_topp2(s, a).map_err(interpolation_status)?;
        // SAFETY: Same checked output location as above.
        unsafe {
            CoppVecF64::write_vec_to(out_b, b);
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Compute cumulative TOPP2 time profile `t(s)` from node profile `a(s)`.
///
/// On success, `*out_t_final` receives the final time and `out_t_s` owns a
/// COPP-allocated vector that must be released with `copp_vec_f64_free`.
///
/// # Safety
/// `s.data` and `a.data` must be valid for `len` reads when their lengths are
/// non-zero. `out_t_final` must be valid for one `double` write and `out_t_s`
/// must be valid for one `CoppVecF64` write. C callers can pass the address of
/// ordinary stack variables; heap allocation is not required for these outputs.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_s_to_t_2nd(
    s: CoppSliceF64,
    a: CoppSliceF64,
    t0: f64,
    out_t_final: *mut f64,
    out_t_s: *mut CoppVecF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if out_t_final.is_null() || out_t_s.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }
    let out_t_s = match CoppVecF64::out_ptr(out_t_s) {
        Ok(out_t_s) => out_t_s,
        Err(status) => return status,
    };
    // SAFETY: `out_t_s` was checked for null above; caller must provide a valid
    // output location as part of the C ABI contract.
    unsafe {
        CoppVecF64::write_empty_to(out_t_s);
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: The C ABI contract requires non-empty input slices to point
        // to valid contiguous `double` arrays for the duration of this call.
        let s = unsafe { s.as_slice()? };
        // SAFETY: Same input-slice contract as above.
        let a = unsafe { a.as_slice()? };
        let (t_final, t_s) = s_to_t_topp2(s, a, t0).map_err(interpolation_status)?;

        // SAFETY: `out_t_final` is checked for null above and is expected to be
        // valid for writes by the C ABI contract.
        unsafe {
            out_t_final.write(t_final);
        }
        // SAFETY: Same checked output location as above.
        unsafe {
            CoppVecF64::write_vec_to(out_t_s, t_s);
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Interpolate `s(t)` from TOPP2 profiles using a uniform time grid.
///
/// On success, `out_s_t` owns a COPP-allocated vector and must be released with
/// `copp_vec_f64_free`.
///
/// # Safety
/// `s.data`, `a.data`, and `t_s.data` must be valid for `len` reads when their
/// lengths are non-zero. `out_s_t` must be valid for one `CoppVecF64` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_t_to_s_uniform_2nd(
    s: CoppSliceF64,
    a: CoppSliceF64,
    t_s: CoppSliceF64,
    t0: f64,
    dt: f64,
    include_final: bool,
    out_s_t: *mut CoppVecF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    topp2_t_to_s_common(s, a, t_s, out_s_t, |s, a, t_s| {
        t_to_s_topp2(
            s,
            a,
            t_s,
            InterpolationMode::UniformTimeGrid(t0, dt, include_final),
        )
        .map_err(interpolation_status)
    })
}

/// Interpolate `s(t)` from TOPP2 profiles using caller-provided time samples.
///
/// On success, `out_s_t` owns a COPP-allocated vector and must be released with
/// `copp_vec_f64_free`.
///
/// # Safety
/// `s.data`, `a.data`, `t_s.data`, and `t_sample.data` must be valid for `len`
/// reads when their lengths are non-zero. `out_s_t` must be valid for one
/// `CoppVecF64` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_t_to_s_non_uniform_2nd(
    s: CoppSliceF64,
    a: CoppSliceF64,
    t_s: CoppSliceF64,
    t_sample: CoppSliceF64,
    out_s_t: *mut CoppVecF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    topp2_t_to_s_common(s, a, t_s, out_s_t, |s, a, t_s| {
        // SAFETY: The C ABI contract requires non-empty input slices to point
        // to valid contiguous `double` arrays for the duration of this call.
        let t_sample = unsafe { t_sample.as_slice()? };
        t_to_s_topp2(s, a, t_s, InterpolationMode::NonUniformTimeGrid(t_sample))
            .map_err(interpolation_status)
    })
}

/// Compute cumulative TOPP3/COPP3 time profile `t(s)` from node profiles
/// `a(s) = dot{s}^2` and `b(s) = ddot{s}`.
///
/// C callers pass profile parts explicitly instead of constructing a
/// `Topp3ProfileRef`: `a` and `b` are node-based arrays with length
/// `s.len`, and `num_stationary_*` are the effective stationary interval
/// counts at the start and end.
///
/// On success, `*out_t_final` receives the final time and `out_t_s` owns a
/// COPP-allocated vector that must be released with `copp_vec_f64_free`.
///
/// # Safety
/// `s.data`, `a.data`, and `b.data` must be valid for reads when their lengths
/// are non-zero. `out_t_final` must be valid for one `double` write and
/// `out_t_s` must be valid for one `CoppVecF64` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_s_to_t_3rd(
    s: CoppSliceF64,
    a: CoppSliceF64,
    b: CoppSliceF64,
    num_stationary_start: usize,
    num_stationary_end: usize,
    t0: f64,
    out_t_final: *mut f64,
    out_t_s: *mut CoppVecF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if out_t_final.is_null() || out_t_s.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }
    let out_t_s = match CoppVecF64::out_ptr(out_t_s) {
        Ok(out_t_s) => out_t_s,
        Err(status) => return status,
    };
    // SAFETY: `out_t_s` was checked for null above; caller must provide a valid
    // output location as part of the C ABI contract.
    unsafe {
        CoppVecF64::write_empty_to(out_t_s);
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: The C ABI contract requires non-empty input slices to point
        // to valid contiguous `double` arrays for the duration of this call.
        let s = unsafe { s.as_slice()? };
        // SAFETY: Same input-slice contract as above.
        let a = unsafe { a.as_slice()? };
        // SAFETY: Same input-slice contract as above.
        let b = unsafe { b.as_slice()? };
        let (t_final, t_s) =
            s_to_t_topp3(s, (a, b, (num_stationary_start, num_stationary_end)), t0)
                .map_err(interpolation_status)?;

        // SAFETY: `out_t_final` is checked for null above and is expected to
        // be valid for writes by the C ABI contract.
        unsafe {
            out_t_final.write(t_final);
        }
        // SAFETY: Same checked output location as above.
        unsafe {
            CoppVecF64::write_vec_to(out_t_s, t_s);
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Interpolate `s(t)` from TOPP3/COPP3 profiles using a uniform time grid.
///
/// C callers pass profile parts explicitly instead of constructing a
/// `Topp3ProfileRef`. On success, `out_s_t` owns a COPP-allocated vector and
/// must be released with `copp_vec_f64_free`.
///
/// # Safety
/// `s.data`, `a.data`, `b.data`, and `t_s.data` must be valid for reads when
/// their lengths are non-zero. `out_s_t` must be valid for one `CoppVecF64`
/// write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_t_to_s_uniform_3rd(
    s: CoppSliceF64,
    a: CoppSliceF64,
    b: CoppSliceF64,
    num_stationary_start: usize,
    num_stationary_end: usize,
    t_s: CoppSliceF64,
    t0: f64,
    dt: f64,
    include_final: bool,
    out_s_t: *mut CoppVecF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    topp3_t_to_s_common(
        s,
        a,
        b,
        (num_stationary_start, num_stationary_end),
        t_s,
        out_s_t,
        |s, a, b, num_stationary, t_s| {
            t_to_s_topp3(
                s,
                (a, b, num_stationary),
                t_s,
                InterpolationMode::UniformTimeGrid(t0, dt, include_final),
            )
            .map_err(interpolation_status)
        },
    )
}

/// Interpolate `s(t)` from TOPP3/COPP3 profiles using caller-provided time
/// samples.
///
/// C callers pass profile parts explicitly instead of constructing a
/// `Topp3ProfileRef`. On success, `out_s_t` owns a COPP-allocated vector and
/// must be released with `copp_vec_f64_free`.
///
/// # Safety
/// `s.data`, `a.data`, `b.data`, `t_s.data`, and `t_sample.data` must be valid
/// for reads when their lengths are non-zero. `out_s_t` must be valid for one
/// `CoppVecF64` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_t_to_s_non_uniform_3rd(
    s: CoppSliceF64,
    a: CoppSliceF64,
    b: CoppSliceF64,
    num_stationary_start: usize,
    num_stationary_end: usize,
    t_s: CoppSliceF64,
    t_sample: CoppSliceF64,
    out_s_t: *mut CoppVecF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    topp3_t_to_s_common(
        s,
        a,
        b,
        (num_stationary_start, num_stationary_end),
        t_s,
        out_s_t,
        |s, a, b, num_stationary, t_s| {
            // SAFETY: The C ABI contract requires non-empty input slices to
            // point to valid contiguous `double` arrays for the duration of
            // this call.
            let t_sample = unsafe { t_sample.as_slice()? };
            t_to_s_topp3(
                s,
                (a, b, num_stationary),
                t_s,
                InterpolationMode::NonUniformTimeGrid(t_sample),
            )
            .map_err(interpolation_status)
        },
    )
}

/// Adjust mutable TOPP3/COPP3 `a` and `b` node profiles in place so
/// interpolated `a(s)` stays strictly positive per interval.
///
/// C callers pass profile parts explicitly instead of constructing a
/// `Topp3ProfileMut`. `a` and `b` must be mutable node-based arrays with length
/// `s.len`. On success, `*out_succeed` receives the operation result flag; the
/// `a.data` and `b.data` buffers may have been modified.
///
/// # Safety
/// `s.data` must be valid for reads when non-empty. `a.data` and `b.data` must
/// be valid for unique mutable access when non-empty and must not alias each
/// other. `out_succeed` must be valid for one `bool` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_force_positive_a_3rd(
    s: CoppSliceF64,
    a: CoppSliceMutF64,
    b: CoppSliceMutF64,
    num_stationary_start: usize,
    num_stationary_end: usize,
    a_min: f64,
    out_succeed: *mut bool,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if out_succeed.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }
    // SAFETY: `out_succeed` was checked for null above and is expected to be
    // valid for one write by the C ABI contract.
    unsafe {
        out_succeed.write(false);
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: The C ABI contract requires non-empty input slices to point
        // to valid contiguous `double` arrays for the duration of this call.
        let s = unsafe { s.as_slice()? };
        // SAFETY: The C ABI contract requires non-empty mutable slices to be
        // valid and uniquely borrowed for the duration of this call.
        let a = unsafe { a.as_mut_slice()? };
        // SAFETY: Same mutable-slice contract as above.
        let b = unsafe { b.as_mut_slice()? };
        let succeed =
            force_positive_a((a, b, (num_stationary_start, num_stationary_end)), s, a_min)
                .map_err(interpolation_status)?;

        // SAFETY: Same checked output location as above.
        unsafe {
            out_succeed.write(succeed);
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Shared implementation for TOPP2 inverse interpolation C wrappers.
///
/// This helper centralizes output initialization, panic fencing, common input
/// slice conversion, error mapping, and COPP-allocated vector transfer.
fn topp2_t_to_s_common(
    s: CoppSliceF64,
    a: CoppSliceF64,
    t_s: CoppSliceF64,
    out_s_t: *mut CoppVecF64,
    interpolate: impl FnOnce(&[f64], &[f64], &[f64]) -> Result<Vec<f64>, CoppStatus>,
) -> CoppStatus {
    let out_s_t = match CoppVecF64::out_ptr(out_s_t) {
        Ok(out_s_t) => out_s_t,
        Err(status) => return status,
    };
    // SAFETY: `out_s_t` was checked for null above; caller must provide a valid
    // output location as part of the C ABI contract.
    unsafe {
        CoppVecF64::write_empty_to(out_s_t);
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: The C ABI contract requires non-empty input slices to point
        // to valid contiguous `double` arrays for the duration of this call.
        let s = unsafe { s.as_slice()? };
        // SAFETY: Same input-slice contract as above.
        let a = unsafe { a.as_slice()? };
        // SAFETY: Same input-slice contract as above.
        let t_s = unsafe { t_s.as_slice()? };

        let s_t = interpolate(s, a, t_s)?;
        // SAFETY: Same checked output location as above.
        unsafe {
            CoppVecF64::write_vec_to(out_s_t, s_t);
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Shared implementation for TOPP3 inverse interpolation C wrappers.
///
/// This helper centralizes output initialization, panic fencing, common input
/// slice conversion, error mapping, and COPP-allocated vector transfer.
fn topp3_t_to_s_common(
    s: CoppSliceF64,
    a: CoppSliceF64,
    b: CoppSliceF64,
    num_stationary: (usize, usize),
    t_s: CoppSliceF64,
    out_s_t: *mut CoppVecF64,
    interpolate: impl FnOnce(
        &[f64],
        &[f64],
        &[f64],
        (usize, usize),
        &[f64],
    ) -> Result<Vec<f64>, CoppStatus>,
) -> CoppStatus {
    let out_s_t = match CoppVecF64::out_ptr(out_s_t) {
        Ok(out_s_t) => out_s_t,
        Err(status) => return status,
    };
    // SAFETY: `out_s_t` was checked for null above; caller must provide a valid
    // output location as part of the C ABI contract.
    unsafe {
        CoppVecF64::write_empty_to(out_s_t);
    }

    let (num_stationary_start, num_stationary_end) = num_stationary;

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: The C ABI contract requires non-empty input slices to point
        // to valid contiguous `double` arrays for the duration of this call.
        let s = unsafe { s.as_slice()? };
        // SAFETY: Same input-slice contract as above.
        let a = unsafe { a.as_slice()? };
        // SAFETY: Same input-slice contract as above.
        let b = unsafe { b.as_slice()? };
        // SAFETY: Same input-slice contract as above.
        let t_s = unsafe { t_s.as_slice()? };

        let s_t = interpolate(s, a, b, (num_stationary_start, num_stationary_end), t_s)?;
        // SAFETY: Same checked output location as above.
        unsafe {
            CoppVecF64::write_vec_to(out_s_t, s_t);
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Map interpolation-layer errors to C ABI status codes.
///
/// Interpolation input checks report `CoppError::InvalidInput`, but they are not
/// solver failures. The C ABI therefore exposes them as invalid C arguments.
fn interpolation_status(error: CoppError) -> CoppStatus {
    match error {
        CoppError::InvalidInput(_, _) => CoppStatus::InvalidArgument,
        error => CoppStatus::from(&error),
    }
}
