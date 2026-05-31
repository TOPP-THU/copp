//! C ABI wrappers for robot/constraint storage.

use crate::copp::constraints::ModePopConstraints;
use crate::diag::{CoppError, RobotDynamicsError};
use crate::ffi::c::core::status::{
    clear_last_error, current_last_error_message_lossy, panic_to_status,
};
use crate::ffi::c::{CoppMatrixF64, CoppMatrixViewF64, CoppSliceF64, CoppStatus, CoppVecF64};
use crate::robot::{Robot, RobotBasic, RobotTorque};
use std::{
    ffi::c_void,
    panic::{AssertUnwindSafe, catch_unwind},
    ptr,
};

/// Opaque C handle for a library-owned robot and constraint buffer.
///
/// Create with `copp_robot_create` and release exactly once with
/// `copp_robot_free`.  C callers must not inspect or allocate this type
/// directly.
pub struct CoppRobot;

pub(crate) struct CoppRobotInner {
    pub(crate) robot: Robot<CoppTorqueForC>,
}

/// C callback for inverse dynamics.
///
/// The callback receives `dim`-element vectors `q`, `dq`, `ddq`, and must write
/// `dim` torque/force values into `tau` before returning [`CoppStatus::Ok`].
/// The `tau` buffer may contain old values on entry, so callbacks should
/// overwrite every entry directly. On failure, return a non-OK status; outer
/// COPP APIs report it as a robot dynamics error.
pub type CoppInverseDynamicsFn = Option<
    unsafe extern "C" fn(
        user_data: *mut c_void,
        dim: usize,
        q: *const f64,
        dq: *const f64,
        ddq: *const f64,
        tau: *mut f64,
    ) -> CoppStatus,
>;

#[derive(Clone, Copy, Debug)]
pub(crate) struct CoppInverseDynamicsData {
    inverse_dynamics: unsafe extern "C" fn(
        user_data: *mut c_void,
        dim: usize,
        q: *const f64,
        dq: *const f64,
        ddq: *const f64,
        tau: *mut f64,
    ) -> CoppStatus,
    user_data: *mut c_void,
}

impl CoppInverseDynamicsData {
    #[inline(always)]
    fn new(
        inverse_dynamics: unsafe extern "C" fn(
            user_data: *mut c_void,
            dim: usize,
            q: *const f64,
            dq: *const f64,
            ddq: *const f64,
            tau: *mut f64,
        ) -> CoppStatus,
        user_data: *mut c_void,
    ) -> Self {
        Self {
            inverse_dynamics,
            user_data,
        }
    }

    fn call(
        self,
        dim: usize,
        q: &[f64],
        dq: &[f64],
        ddq: &[f64],
        tau: &mut [f64],
    ) -> Result<(), RobotDynamicsError> {
        // SAFETY: The C ABI contract requires the callback to remain valid for
        // the current call and to write exactly `dim` output entries.
        let status = unsafe {
            (self.inverse_dynamics)(
                self.user_data,
                dim,
                q.as_ptr(),
                dq.as_ptr(),
                ddq.as_ptr(),
                tau.as_mut_ptr(),
            )
        };
        if status != CoppStatus::Ok {
            let detail = current_last_error_message_lossy()
                .filter(|message| !message.is_empty())
                .unwrap_or_else(|| status.message().to_string_lossy().into_owned());
            return Err(RobotDynamicsError::new(format!(
                "foreign inverse-dynamics callback returned {:?}: {}",
                status, detail
            )));
        }
        Ok(())
    }
}

/// C-ABI robot torque model.
///
/// A null callback represents the same point dynamics used by `usize` in the
/// built-in convention (`tau = ddq`).  A non-null callback delegates inverse dynamics to C
/// for every torque evaluation.
#[derive(Clone, Copy, Debug)]
pub(crate) struct CoppTorqueForC {
    dim: usize,
    inverse_dynamics: Option<CoppInverseDynamicsData>,
}

impl CoppTorqueForC {
    #[inline(always)]
    fn new(dim: usize) -> Self {
        Self {
            dim,
            inverse_dynamics: None,
        }
    }

    #[inline(always)]
    fn set_inverse_dynamics(&mut self, inverse_dynamics: Option<CoppInverseDynamicsData>) {
        self.inverse_dynamics = inverse_dynamics;
    }
}

impl RobotBasic for CoppTorqueForC {
    #[inline(always)]
    fn dim(&self) -> usize {
        self.dim
    }
}

impl RobotTorque for CoppTorqueForC {
    #[inline(always)]
    fn inverse_dynamics(
        &self,
        q: &[f64],
        dq: &[f64],
        ddq: &[f64],
        tau: &mut [f64],
    ) -> Result<(), RobotDynamicsError> {
        if q.len() != self.dim
            || dq.len() != self.dim
            || ddq.len() != self.dim
            || tau.len() != self.dim
        {
            return Err(RobotDynamicsError::new(format!(
                "inverse_dynamics received inconsistent dimensions: expected {}, got q={}, dq={}, ddq={}, tau={}",
                self.dim,
                q.len(),
                dq.len(),
                ddq.len(),
                tau.len()
            )));
        }

        if let Some(inverse_dynamics) = self.inverse_dynamics {
            inverse_dynamics.call(self.dim, q, dq, ddq, tau)?;
        } else {
            tau.copy_from_slice(ddq);
        }
        Ok(())
    }
}

impl CoppRobot {
    /// Borrow the wrapped robot inner state.
    pub(crate) unsafe fn inner<'a>(robot: *const Self) -> Option<&'a CoppRobotInner> {
        if robot.is_null() {
            return None;
        }

        // SAFETY: Non-null `CoppRobot` handles returned by this module are
        // pointers to `CoppRobotInner` cast to the public opaque handle type.
        Some(unsafe { &*(robot.cast::<CoppRobotInner>()) })
    }

    /// Mutably borrow the wrapped robot inner state.
    pub(crate) unsafe fn inner_mut<'a>(robot: *mut Self) -> Option<&'a mut CoppRobotInner> {
        if robot.is_null() {
            return None;
        }

        // SAFETY: Non-null `CoppRobot` handles returned by this module are
        // pointers to `CoppRobotInner` cast to the public opaque handle type.
        Some(unsafe { &mut *(robot.cast::<CoppRobotInner>()) })
    }

    /// Borrow the wrapped robot.
    pub(crate) unsafe fn robot<'a>(robot: *const Self) -> Option<&'a Robot<CoppTorqueForC>> {
        // SAFETY: Same handle contract as this function.
        Some(&unsafe { Self::inner(robot) }?.robot)
    }

    /// Mutably borrow the wrapped robot.
    pub(crate) unsafe fn robot_mut<'a>(robot: *mut Self) -> Option<&'a mut Robot<CoppTorqueForC>> {
        // SAFETY: Same handle contract as this function.
        Some(&mut unsafe { Self::inner_mut(robot) }?.robot)
    }
}

/// Create a library-owned robot handle for C callers.
///
/// On success, `*out_robot` receives a non-null handle that must be released
/// with `copp_robot_free`.
///
/// # Safety
/// `out_robot` must be valid for one `CoppRobot*` write. C callers can pass the
/// address of an ordinary pointer variable; heap allocation is not required for
/// the output pointer itself.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_create(
    dim: usize,
    capacity: usize,
    out_robot: *mut *mut CoppRobot,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if out_robot.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    // SAFETY: `out_robot` was checked for null above and is expected to be
    // valid for one pointer write by the C ABI contract.
    unsafe {
        out_robot.write(ptr::null_mut());
    }

    match catch_unwind(AssertUnwindSafe(|| {
        let robot = Box::new(CoppRobotInner {
            robot: Robot::with_capacity(CoppTorqueForC::new(dim), capacity),
        });
        // SAFETY: Same checked output location as above.
        unsafe {
            out_robot.write(Box::into_raw(robot).cast::<CoppRobot>());
        }
        CoppStatus::Ok
    })) {
        Ok(status) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Store an inverse-dynamics callback in a robot handle.
///
/// Passing a null `inverse_dynamics` pointer clears the callback and restores
/// default point dynamics (`tau = ddq`). When non-null, the callback and
/// `inverse_dynamics_user_data` must remain valid until replaced, cleared, or
/// the robot is freed.
///
/// # Example
/// The example below installs point dynamics explicitly as the robot's
/// inverse-dynamics callback.
///
/// ```c
/// static enum CoppStatus point_dynamics(
///     void *user_data,
///     size_t dim,
///     const double *q,
///     const double *dq,
///     const double *ddq,
///     double *tau)
/// {
///     for (size_t i = 0; i < dim; ++i) {
///         tau[i] = ddq[i];
///     }
///     return COPP_STATUS_OK;
/// }
///
/// check(copp_robot_set_inverse_dynamics(robot, point_dynamics, NULL));
/// ```
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// `inverse_dynamics`, when non-null, must not unwind and must write a
/// `dim`-element `tau` vector when returning `COPP_STATUS_OK`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_set_inverse_dynamics(
    robot: *mut CoppRobot,
    inverse_dynamics: CoppInverseDynamicsFn,
    inverse_dynamics_user_data: *mut c_void,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if robot.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: `robot` was checked for null above; the C ABI contract
        // requires it to be a live handle created by this module.
        let inner = unsafe { CoppRobot::inner_mut(robot) }.ok_or(CoppStatus::NullPointer)?;
        inner
            .robot
            .model_mut()
            .set_inverse_dynamics(inverse_dynamics.map(|inverse_dynamics| {
                CoppInverseDynamicsData::new(inverse_dynamics, inverse_dynamics_user_data)
            }));
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Clear the inverse-dynamics callback stored in a robot handle.
///
/// After this call, torque-related operations use default point dynamics
/// (`tau = ddq`).
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_clear_inverse_dynamics(robot: *mut CoppRobot) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if robot.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: `robot` was checked for null above; the C ABI contract
        // requires it to be a live handle created by this module.
        let inner = unsafe { CoppRobot::inner_mut(robot) }.ok_or(CoppStatus::NullPointer)?;
        inner.robot.model_mut().set_inverse_dynamics(None);
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Append strictly increasing station samples to a robot.
///
/// Empty input is accepted as a no-op.  Non-empty input must be strictly
/// increasing and must start after the current last stored station.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// `s.data` must be valid for `s.len` reads when `s.len` is non-zero.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_append_s(robot: *mut CoppRobot, s: CoppSliceF64) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if robot.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: `robot` was checked for null above; the C ABI contract
        // requires it to be a live handle created by this module.
        let robot = unsafe { CoppRobot::robot_mut(robot) }.ok_or(CoppStatus::NullPointer)?;
        // SAFETY: The C ABI contract requires non-empty input slices to point
        // to valid contiguous `double` arrays for the duration of this call.
        let s = unsafe { s.as_slice()? };

        robot.with_s(s).map_err(|error| CoppStatus::from(&error))?;
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Overwrite first-order bounds from a profile.
///
/// `amax` must contain one value per station to update.  The values replace
/// the robot's stored first-order bound over
/// `[idx_s, idx_s + amax.len)`.  This is useful before TOPP3 solvers when a
/// TOPP2 profile is used as a tighter linearization/reachability seed.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// `amax.data` must be valid for `amax.len` reads when `amax.len` is non-zero.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_amax_substitute(
    robot: *mut CoppRobot,
    amax: CoppSliceF64,
    idx_s: usize,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if robot.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: `robot` was checked for null above; the C ABI contract
        // requires it to be a live handle created by this module.
        let robot = unsafe { CoppRobot::robot_mut(robot) }.ok_or(CoppStatus::NullPointer)?;
        // SAFETY: The C ABI contract requires non-empty input slices to point
        // to valid contiguous `double` arrays for the duration of this call.
        let amax = unsafe { amax.as_slice()? };

        robot
            .constraints
            .amax_substitute(amax, idx_s)
            .map_err(|error| CoppStatus::from(&error))?;
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Return the robot/path dimension stored in a robot handle.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// `out_dim` must be valid for one `size_t` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_dim(
    robot: *const CoppRobot,
    out_dim: *mut usize,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if robot.is_null() || out_dim.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: `robot` and `out_dim` were checked for null above; the C ABI
        // contract requires `robot` to be a live handle from this module.
        let robot = unsafe { CoppRobot::robot(robot) }.ok_or(CoppStatus::NullPointer)?;
        unsafe {
            out_dim.write(robot.dim());
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Return the current number of logical station samples stored in a robot.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// `out_len` must be valid for one `size_t` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_len(
    robot: *const CoppRobot,
    out_len: *mut usize,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if robot.is_null() || out_len.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: Same checked pointer contract as this function.
        let robot = unsafe { CoppRobot::robot(robot) }.ok_or(CoppStatus::NullPointer)?;
        unsafe {
            out_len.write(robot.constraints.len());
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Return the allocated station capacity of the robot constraint buffer.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// `out_capacity` must be valid for one `size_t` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_capacity(
    robot: *const CoppRobot,
    out_capacity: *mut usize,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if robot.is_null() || out_capacity.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: Same checked pointer contract as this function.
        let robot = unsafe { CoppRobot::robot(robot) }.ok_or(CoppStatus::NullPointer)?;
        unsafe {
            out_capacity.write(robot.constraints.capacity());
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Report whether the robot constraint buffer currently has no stations.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// `out_is_empty` must be valid for one `bool` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_is_empty(
    robot: *const CoppRobot,
    out_is_empty: *mut bool,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if robot.is_null() || out_is_empty.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: Same checked pointer contract as this function.
        let robot = unsafe { CoppRobot::robot(robot) }.ok_or(CoppStatus::NullPointer)?;
        unsafe {
            out_is_empty.write(robot.constraints.is_empty());
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Return the first global station index currently stored in the robot.
///
/// The active station window is `[idx_s_start, idx_s_end)`.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// `out_idx_s_start` must be valid for one `size_t` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_idx_s_start(
    robot: *const CoppRobot,
    out_idx_s_start: *mut usize,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if robot.is_null() || out_idx_s_start.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: Same checked pointer contract as this function.
        let robot = unsafe { CoppRobot::robot(robot) }.ok_or(CoppStatus::NullPointer)?;
        unsafe {
            out_idx_s_start.write(robot.constraints.idx_s_start());
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Return the exclusive global station end index currently stored in the robot.
///
/// The active station window is `[idx_s_start, idx_s_end)`.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// `out_idx_s_end` must be valid for one `size_t` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_idx_s_end(
    robot: *const CoppRobot,
    out_idx_s_end: *mut usize,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if robot.is_null() || out_idx_s_end.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: Same checked pointer contract as this function.
        let robot = unsafe { CoppRobot::robot(robot) }.ok_or(CoppStatus::NullPointer)?;
        unsafe {
            out_idx_s_end.write(robot.constraints.idx_s_end());
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Return the allocated row counts for first-, second-, and third-order constraints.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// Every output pointer must be valid for one `size_t` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_constraint_rows(
    robot: *const CoppRobot,
    out_amax_rows: *mut usize,
    out_acc_rows: *mut usize,
    out_jerk_rows: *mut usize,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if robot.is_null()
        || out_amax_rows.is_null()
        || out_acc_rows.is_null()
        || out_jerk_rows.is_null()
    {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: Same checked pointer contract as this function.
        let robot = unsafe { CoppRobot::robot(robot) }.ok_or(CoppStatus::NullPointer)?;
        unsafe {
            out_amax_rows.write(robot.constraints.amax_rows());
            out_acc_rows.write(robot.constraints.acc_rows());
            out_jerk_rows.write(robot.constraints.jerk_rows());
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Read one station value `s[idx_s]` from the robot constraint buffer.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// `out_s` must be valid for one `double` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_get_s(
    robot: *const CoppRobot,
    idx_s: usize,
    out_s: *mut f64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if robot.is_null() || out_s.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: Same checked pointer contract as this function.
        let robot = unsafe { CoppRobot::robot(robot) }.ok_or(CoppStatus::NullPointer)?;
        let value = robot
            .constraints
            .get_s(idx_s)
            .map_err(|error| CoppStatus::from(&error))?;
        unsafe {
            out_s.write(value);
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Read one first-order upper bound `amax[idx_s]` from the robot.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// `out_amax` must be valid for one `double` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_get_amax(
    robot: *const CoppRobot,
    idx_s: usize,
    out_amax: *mut f64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if robot.is_null() || out_amax.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: Same checked pointer contract as this function.
        let robot = unsafe { CoppRobot::robot(robot) }.ok_or(CoppStatus::NullPointer)?;
        let value = robot
            .constraints
            .get_amax(idx_s)
            .map_err(|error| CoppStatus::from(&error))?;
        unsafe {
            out_amax.write(value);
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Export station values over `[idx_s_from, idx_s_to)`.
///
/// On success, `out_s` owns a COPP-allocated vector and must be released with
/// `copp_vec_f64_free`.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// `out_s` must be valid for one `CoppVecF64` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_s_vec(
    robot: *const CoppRobot,
    idx_s_from: usize,
    idx_s_to: usize,
    out_s: *mut CoppVecF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    export_vec_common(robot, idx_s_from, idx_s_to, out_s, RobotVecKind::Station)
}

/// Export first-order upper bounds over `[idx_s_from, idx_s_to)`.
///
/// On success, `out_amax` owns a COPP-allocated vector and must be released
/// with `copp_vec_f64_free`.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// `out_amax` must be valid for one `CoppVecF64` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_amax_vec(
    robot: *const CoppRobot,
    idx_s_from: usize,
    idx_s_to: usize,
    out_amax: *mut CoppVecF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    export_vec_common(robot, idx_s_from, idx_s_to, out_amax, RobotVecKind::Amax)
}

enum RobotVecKind {
    Station,
    Amax,
}

fn export_vec_common(
    robot: *const CoppRobot,
    idx_s_from: usize,
    idx_s_to: usize,
    out_vec: *mut CoppVecF64,
    kind: RobotVecKind,
) -> CoppStatus {
    if robot.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }
    let out_vec = match CoppVecF64::out_ptr(out_vec) {
        Ok(out_vec) => out_vec,
        Err(status) => return status,
    };
    // SAFETY: `out_vec` was checked for null above and is expected to be valid
    // for one write by the C ABI contract.
    unsafe {
        CoppVecF64::write_empty_to(out_vec);
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: Same checked pointer contract as this helper.
        let robot = unsafe { CoppRobot::robot(robot) }.ok_or(CoppStatus::NullPointer)?;
        let values = match kind {
            RobotVecKind::Station => robot
                .constraints
                .s_vec(idx_s_from, idx_s_to)
                .map_err(|error| CoppStatus::from(&error))?,
            RobotVecKind::Amax => robot
                .constraints
                .amax_vec(idx_s_from, idx_s_to)
                .map_err(|error| CoppStatus::from(&error))?,
        };
        // SAFETY: Same checked output location as above.
        unsafe {
            CoppVecF64::write_vec_to(out_vec, values);
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Export second-order constraint rows at one station.
///
/// On success, the three output matrices are `valid_rows x 1` column-major
/// matrices and must each be released with `copp_matrix_f64_free`.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// Every output pointer must be valid for one `CoppMatrixF64` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_acc_constraints_at(
    robot: *const CoppRobot,
    idx_s: usize,
    out_acc_a: *mut CoppMatrixF64,
    out_acc_b: *mut CoppMatrixF64,
    out_acc_max: *mut CoppMatrixF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if robot.is_null() || out_acc_a.is_null() || out_acc_b.is_null() || out_acc_max.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }
    let out_acc_a = match CoppMatrixF64::out_ptr(out_acc_a) {
        Ok(out) => out,
        Err(status) => return status,
    };
    let out_acc_b = match CoppMatrixF64::out_ptr(out_acc_b) {
        Ok(out) => out,
        Err(status) => return status,
    };
    let out_acc_max = match CoppMatrixF64::out_ptr(out_acc_max) {
        Ok(out) => out,
        Err(status) => return status,
    };
    // SAFETY: Output pointers were checked for null above.
    unsafe {
        CoppMatrixF64::write_empty_to(out_acc_a);
        CoppMatrixF64::write_empty_to(out_acc_b);
        CoppMatrixF64::write_empty_to(out_acc_max);
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: Same checked pointer contract as this function.
        let robot = unsafe { CoppRobot::robot(robot) }.ok_or(CoppStatus::NullPointer)?;
        let (acc_a, acc_b, acc_max) = robot
            .constraints
            .get_acc_constraints(idx_s)
            .map_err(|error| CoppStatus::from(&error))?;
        let acc_a = CoppMatrixF64::from_matrix(acc_a.into_owned())?;
        let acc_b = CoppMatrixF64::from_matrix(acc_b.into_owned())?;
        let acc_max = CoppMatrixF64::from_matrix(acc_max.into_owned())?;
        // SAFETY: Same checked output locations as above.
        unsafe {
            out_acc_a.as_ptr().write(acc_a);
            out_acc_b.as_ptr().write(acc_b);
            out_acc_max.as_ptr().write(acc_max);
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Export nonlinear third-order constraint rows at one station.
///
/// On success, the five output matrices are `valid_rows x 1` column-major
/// matrices and must each be released with `copp_matrix_f64_free`.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// Every output pointer must be valid for one `CoppMatrixF64` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_jerk_constraints_at(
    robot: *const CoppRobot,
    idx_s: usize,
    out_jerk_a: *mut CoppMatrixF64,
    out_jerk_b: *mut CoppMatrixF64,
    out_jerk_c: *mut CoppMatrixF64,
    out_jerk_d: *mut CoppMatrixF64,
    out_jerk_max: *mut CoppMatrixF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if robot.is_null()
        || out_jerk_a.is_null()
        || out_jerk_b.is_null()
        || out_jerk_c.is_null()
        || out_jerk_d.is_null()
        || out_jerk_max.is_null()
    {
        return CoppStatus::NullPointer.into_ffi_status();
    }
    let out_jerk_a = match CoppMatrixF64::out_ptr(out_jerk_a) {
        Ok(out) => out,
        Err(status) => return status,
    };
    let out_jerk_b = match CoppMatrixF64::out_ptr(out_jerk_b) {
        Ok(out) => out,
        Err(status) => return status,
    };
    let out_jerk_c = match CoppMatrixF64::out_ptr(out_jerk_c) {
        Ok(out) => out,
        Err(status) => return status,
    };
    let out_jerk_d = match CoppMatrixF64::out_ptr(out_jerk_d) {
        Ok(out) => out,
        Err(status) => return status,
    };
    let out_jerk_max = match CoppMatrixF64::out_ptr(out_jerk_max) {
        Ok(out) => out,
        Err(status) => return status,
    };
    // SAFETY: Output pointers were checked for null above.
    unsafe {
        CoppMatrixF64::write_empty_to(out_jerk_a);
        CoppMatrixF64::write_empty_to(out_jerk_b);
        CoppMatrixF64::write_empty_to(out_jerk_c);
        CoppMatrixF64::write_empty_to(out_jerk_d);
        CoppMatrixF64::write_empty_to(out_jerk_max);
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: Same checked pointer contract as this function.
        let robot = unsafe { CoppRobot::robot(robot) }.ok_or(CoppStatus::NullPointer)?;
        let (jerk_a, jerk_b, jerk_c, jerk_d, jerk_max) = robot
            .constraints
            .get_jerk_constraints(idx_s)
            .map_err(|error| CoppStatus::from(&error))?;
        let jerk_a = CoppMatrixF64::from_matrix(jerk_a.into_owned())?;
        let jerk_b = CoppMatrixF64::from_matrix(jerk_b.into_owned())?;
        let jerk_c = CoppMatrixF64::from_matrix(jerk_c.into_owned())?;
        let jerk_d = CoppMatrixF64::from_matrix(jerk_d.into_owned())?;
        let jerk_max = CoppMatrixF64::from_matrix(jerk_max.into_owned())?;
        // SAFETY: Same checked output locations as above.
        unsafe {
            out_jerk_a.as_ptr().write(jerk_a);
            out_jerk_b.as_ptr().write(jerk_b);
            out_jerk_c.as_ptr().write(jerk_c);
            out_jerk_d.as_ptr().write(jerk_d);
            out_jerk_max.as_ptr().write(jerk_max);
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Export linearized third-order constraint rows at one station.
///
/// The robot must already have been used to build a TOPP3/COPP3 problem with
/// a linearization interval covering `idx_s`. On success, the four output
/// matrices are `valid_rows x 1` column-major matrices and must each be
/// released with `copp_matrix_f64_free`.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// Every output pointer must be valid for one `CoppMatrixF64` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_jerk_linear_constraints_at(
    robot: *const CoppRobot,
    idx_s: usize,
    out_jerk_a_linear: *mut CoppMatrixF64,
    out_jerk_b: *mut CoppMatrixF64,
    out_jerk_c: *mut CoppMatrixF64,
    out_jerk_max_linear: *mut CoppMatrixF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if robot.is_null()
        || out_jerk_a_linear.is_null()
        || out_jerk_b.is_null()
        || out_jerk_c.is_null()
        || out_jerk_max_linear.is_null()
    {
        return CoppStatus::NullPointer.into_ffi_status();
    }
    let out_jerk_a_linear = match CoppMatrixF64::out_ptr(out_jerk_a_linear) {
        Ok(out) => out,
        Err(status) => return status,
    };
    let out_jerk_b = match CoppMatrixF64::out_ptr(out_jerk_b) {
        Ok(out) => out,
        Err(status) => return status,
    };
    let out_jerk_c = match CoppMatrixF64::out_ptr(out_jerk_c) {
        Ok(out) => out,
        Err(status) => return status,
    };
    let out_jerk_max_linear = match CoppMatrixF64::out_ptr(out_jerk_max_linear) {
        Ok(out) => out,
        Err(status) => return status,
    };
    // SAFETY: Output pointers were checked for null above.
    unsafe {
        CoppMatrixF64::write_empty_to(out_jerk_a_linear);
        CoppMatrixF64::write_empty_to(out_jerk_b);
        CoppMatrixF64::write_empty_to(out_jerk_c);
        CoppMatrixF64::write_empty_to(out_jerk_max_linear);
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: Same checked pointer contract as this function.
        let robot = unsafe { CoppRobot::robot(robot) }.ok_or(CoppStatus::NullPointer)?;
        let (jerk_a_linear, jerk_b, jerk_c, jerk_max_linear) = robot
            .constraints
            .get_jerk_linear_constraints(idx_s)
            .map_err(|error| CoppStatus::from(&error))?;
        let jerk_a_linear = CoppMatrixF64::from_matrix(jerk_a_linear.into_owned())?;
        let jerk_b = CoppMatrixF64::from_matrix(jerk_b.into_owned())?;
        let jerk_c = CoppMatrixF64::from_matrix(jerk_c.into_owned())?;
        let jerk_max_linear = CoppMatrixF64::from_matrix(jerk_max_linear.into_owned())?;
        // SAFETY: Same checked output locations as above.
        unsafe {
            out_jerk_a_linear.as_ptr().write(jerk_a_linear);
            out_jerk_b.as_ptr().write(jerk_b);
            out_jerk_c.as_ptr().write(jerk_c);
            out_jerk_max_linear.as_ptr().write(jerk_max_linear);
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Clear all logical constraints stored in the robot.
///
/// When `keep_idx_s` is true, the current global station origin is preserved;
/// otherwise the origin is reset to zero.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_clear_constraints(
    robot: *mut CoppRobot,
    keep_idx_s: bool,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if robot.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: Same checked pointer contract as this function.
        let robot = unsafe { CoppRobot::robot_mut(robot) }.ok_or(CoppStatus::NullPointer)?;
        robot.constraints.clear(keep_idx_s);
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Remove `n_cols` logical stations from the front of the robot buffer.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_pop_front_n(
    robot: *mut CoppRobot,
    n_cols: usize,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    pop_constraints_common(robot, true, ModePopConstraints::PopNCols(n_cols))
}

/// Remove `n_cols` logical stations from the back of the robot buffer.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_pop_back_n(robot: *mut CoppRobot, n_cols: usize) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    pop_constraints_common(robot, false, ModePopConstraints::PopNCols(n_cols))
}

/// Remove front stations so the kept window starts at `idx_s_cut` or later.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_pop_front_until(
    robot: *mut CoppRobot,
    idx_s_cut: usize,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    pop_constraints_common(robot, true, ModePopConstraints::CutAtIdxS(idx_s_cut))
}

/// Remove back stations so the kept window ends before `idx_s_cut`.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_pop_back_until(
    robot: *mut CoppRobot,
    idx_s_cut: usize,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    pop_constraints_common(robot, false, ModePopConstraints::CutAtIdxS(idx_s_cut))
}

fn pop_constraints_common(
    robot: *mut CoppRobot,
    front: bool,
    mode: ModePopConstraints,
) -> CoppStatus {
    if robot.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: Same checked pointer contract as this helper.
        let robot = unsafe { CoppRobot::robot_mut(robot) }.ok_or(CoppStatus::NullPointer)?;
        if front {
            robot.constraints.pop_front(mode);
        } else {
            robot.constraints.pop_back(mode);
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Write second-order path derivatives into an existing robot station interval.
///
/// The matrices must have shape `dim x N`, where `dim` is the robot dimension
/// and `N` is the number of station samples to update. Contiguous column-major
/// input is the fast path; other valid `CoppMatrixViewF64` layouts are copied
/// once into column-major temporary storage.
///
/// This function clears any existing third-derivative data over the same
/// interval.  Use `copp_robot_set_q_3rd` when third-order derivative data is
/// needed.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.  Non-empty
/// matrices must point to valid `double` arrays for their declared layouts.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_set_q_2nd(
    robot: *mut CoppRobot,
    idx_s: usize,
    q: CoppMatrixViewF64,
    dq: CoppMatrixViewF64,
    ddq: CoppMatrixViewF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    set_q_common(robot, idx_s, q, dq, ddq, None)
}

/// Write third-order path derivatives into an existing robot station interval.
///
/// The matrices must have shape `dim x N`, where `dim` is the robot dimension
/// and `N` is the number of station samples to update. Contiguous column-major
/// input is the fast path; other valid `CoppMatrixViewF64` layouts are copied
/// once into column-major temporary storage.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.  Non-empty
/// matrices must point to valid `double` arrays for their declared layouts.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_set_q_3rd(
    robot: *mut CoppRobot,
    idx_s: usize,
    q: CoppMatrixViewF64,
    dq: CoppMatrixViewF64,
    ddq: CoppMatrixViewF64,
    dddq: CoppMatrixViewF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    set_q_common(robot, idx_s, q, dq, ddq, Some(dddq))
}

/// Deprecated compatibility alias for `copp_robot_set_q_2nd`.
///
/// New code should call `copp_robot_set_q_2nd` so all robot-mutating functions
/// share the same `copp_robot_*` prefix.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_set_q_2nd(
    robot: *mut CoppRobot,
    idx_s: usize,
    q: CoppMatrixViewF64,
    dq: CoppMatrixViewF64,
    ddq: CoppMatrixViewF64,
) -> CoppStatus {
    unsafe { copp_robot_set_q_2nd(robot, idx_s, q, dq, ddq) }
}

/// Deprecated compatibility alias for `copp_robot_set_q_3rd`.
///
/// New code should call `copp_robot_set_q_3rd` so all robot-mutating functions
/// share the same `copp_robot_*` prefix.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_set_q_3rd(
    robot: *mut CoppRobot,
    idx_s: usize,
    q: CoppMatrixViewF64,
    dq: CoppMatrixViewF64,
    ddq: CoppMatrixViewF64,
    dddq: CoppMatrixViewF64,
) -> CoppStatus {
    unsafe { copp_robot_set_q_3rd(robot, idx_s, q, dq, ddq, dddq) }
}

fn set_q_common(
    robot: *mut CoppRobot,
    idx_s: usize,
    q: CoppMatrixViewF64,
    dq: CoppMatrixViewF64,
    ddq: CoppMatrixViewF64,
    dddq: Option<CoppMatrixViewF64>,
) -> CoppStatus {
    if robot.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: `robot` was checked for null above; the C ABI contract
        // requires it to be a live handle created by this module.
        let robot = unsafe { CoppRobot::robot_mut(robot) }.ok_or(CoppStatus::NullPointer)?;
        // SAFETY: The C ABI contract requires non-empty matrix views to point
        // to valid `double` arrays for their declared layouts.
        let q = unsafe { q.as_input_matrix()? };
        let dq = unsafe { dq.as_input_matrix()? };
        let ddq = unsafe { ddq.as_input_matrix()? };
        let dddq = match dddq {
            Some(dddq) => Some(unsafe { dddq.as_input_matrix()? }),
            None => None,
        };
        let q = q.as_view();
        let dq = dq.as_view();
        let ddq = ddq.as_view();
        let dddq = dddq.as_ref().map(|dddq| dddq.as_view());

        robot
            .with_q(&q, &dq, &ddq, dddq.as_ref(), idx_s)
            .map_err(|error| CoppStatus::from(&error))?;

        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Add raw first-order constraints over an existing robot station interval.
///
/// `amax` must be a matrix view with shape `R x N`.  For each station column,
/// COPP reduces all `R` rows to their minimum and tightens the stored
/// first-order bound:
/// `a <= min_r amax[r, station]`.
/// Contiguous column-major input is the fast path; other valid layouts are
/// copied once into column-major temporary storage.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// Non-empty matrices must point to valid `double` arrays for their declared
/// layouts.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_add_raw_constraint_1st(
    robot: *mut CoppRobot,
    idx_s: usize,
    amax: CoppMatrixViewF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if robot.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: `robot` was checked for null above; the C ABI contract
        // requires it to be a live handle created by this module.
        let robot = unsafe { CoppRobot::robot_mut(robot) }.ok_or(CoppStatus::NullPointer)?;
        // SAFETY: The C ABI contract requires non-empty matrix views to point
        // to valid `double` arrays for their declared layouts.
        let amax = unsafe { amax.as_input_matrix()? };
        let amax = amax.as_view();

        robot
            .constraints
            .with_constraint_1order(&amax, idx_s)
            .map_err(|error| CoppStatus::from(&error))?;

        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Add raw second-order inequality rows over a station interval.
///
/// All matrices must have the same shape `R x N`.  Each row
/// stores one inequality:
/// `acc_a * a + acc_b * b <= acc_max`.
/// Lower bounds should be provided by negating the row on the C side.
/// Contiguous column-major input is the fast path; other valid layouts are
/// copied once into column-major temporary storage.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// Non-empty matrices must point to valid `double` arrays for their declared
/// layouts.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_add_raw_constraint_2nd(
    robot: *mut CoppRobot,
    idx_s: usize,
    acc_a: CoppMatrixViewF64,
    acc_b: CoppMatrixViewF64,
    acc_max: CoppMatrixViewF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if robot.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: `robot` was checked for null above; the C ABI contract
        // requires it to be a live handle created by this module.
        let robot = unsafe { CoppRobot::robot_mut(robot) }.ok_or(CoppStatus::NullPointer)?;
        // SAFETY: The C ABI contract requires non-empty matrix views to point
        // to valid `double` arrays for their declared layouts.
        let acc_a = unsafe { acc_a.as_input_matrix()? };
        let acc_b = unsafe { acc_b.as_input_matrix()? };
        let acc_max = unsafe { acc_max.as_input_matrix()? };
        let acc_a = acc_a.as_view();
        let acc_b = acc_b.as_view();
        let acc_max = acc_max.as_view();

        robot
            .constraints
            .with_constraint_2order(&acc_a, &acc_b, &acc_max, idx_s, false)
            .map_err(|error| CoppStatus::from(&error))?;

        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Add raw third-order nonlinear inequality rows over a station interval.
///
/// All matrices must have the same shape `R x N`.  Each row
/// stores one inequality:
/// `sqrt(a) * (jerk_a*a + jerk_b*b + jerk_c*c + jerk_d) <= jerk_max`.
/// Lower bounds should be provided by negating the row on the C side.
/// Contiguous column-major input is the fast path; other valid layouts are
/// copied once into column-major temporary storage.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// Non-empty matrices must point to valid `double` arrays for their declared
/// layouts.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_add_raw_constraint_3rd(
    robot: *mut CoppRobot,
    idx_s: usize,
    jerk_a: CoppMatrixViewF64,
    jerk_b: CoppMatrixViewF64,
    jerk_c: CoppMatrixViewF64,
    jerk_d: CoppMatrixViewF64,
    jerk_max: CoppMatrixViewF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if robot.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: `robot` was checked for null above; the C ABI contract
        // requires it to be a live handle created by this module.
        let robot = unsafe { CoppRobot::robot_mut(robot) }.ok_or(CoppStatus::NullPointer)?;
        // SAFETY: The C ABI contract requires non-empty matrix views to point
        // to valid `double` arrays for their declared layouts.
        let jerk_a = unsafe { jerk_a.as_input_matrix()? };
        let jerk_b = unsafe { jerk_b.as_input_matrix()? };
        let jerk_c = unsafe { jerk_c.as_input_matrix()? };
        let jerk_d = unsafe { jerk_d.as_input_matrix()? };
        let jerk_max = unsafe { jerk_max.as_input_matrix()? };
        let jerk_a = jerk_a.as_view();
        let jerk_b = jerk_b.as_view();
        let jerk_c = jerk_c.as_view();
        let jerk_d = jerk_d.as_view();
        let jerk_max = jerk_max.as_view();

        robot
            .constraints
            .with_constraint_3order(&jerk_a, &jerk_b, &jerk_c, &jerk_d, &jerk_max, idx_s, false)
            .map_err(|error| CoppStatus::from(&error))?;

        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Add broadcast axial velocity limits over an existing robot station interval.
///
/// `velocity_max` and `velocity_min` must both have length equal to the robot
/// dimension.  The same per-axis limits are applied to every station in
/// `[start_idx_s, start_idx_s + len)`.  `len == 0` is accepted as a no-op.
///
/// # Example
/// The example below applies the same per-axis velocity box to every station
/// in an interval.
///
/// ```c
/// double vmax[] = {1.0, 1.0, 1.0};
/// double vmin[] = {-1.0, -1.0, -1.0};
/// check(copp_add_axial_velocity_limits(
///     robot,
///     0,
///     n,
///     (struct CoppSliceF64){vmax, 3},
///     (struct CoppSliceF64){vmin, 3}));
/// ```
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// Non-empty slices must point to valid contiguous `double` values.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_add_axial_velocity_limits(
    robot: *mut CoppRobot,
    start_idx_s: usize,
    len: usize,
    velocity_max: CoppSliceF64,
    velocity_min: CoppSliceF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    add_axial_limits_common(
        AxialLimitKind::Velocity,
        robot,
        start_idx_s,
        len,
        velocity_max,
        velocity_min,
    )
}

/// Add station-varying axial velocity limits over an existing robot interval.
///
/// `velocity_max` and `velocity_min` must be matrix views with shape `dim x N`.
/// Column `j` is applied to station `start_idx_s + j`.  `N == 0` is accepted
/// as a no-op when both matrices have shape `dim x 0`. Contiguous column-major
/// input is the fast path; other valid layouts are copied once into
/// column-major temporary storage.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// Non-empty matrices must point to valid `double` arrays for their declared
/// layouts.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_add_axial_velocity_limits_matrix(
    robot: *mut CoppRobot,
    start_idx_s: usize,
    velocity_max: CoppMatrixViewF64,
    velocity_min: CoppMatrixViewF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    add_axial_limits_matrix_common(
        AxialLimitKind::Velocity,
        robot,
        start_idx_s,
        velocity_max,
        velocity_min,
    )
}

/// Add broadcast axial acceleration limits over an existing robot station interval.
///
/// `acceleration_max` and `acceleration_min` must both have length equal to the
/// robot dimension.  The same per-axis limits are applied to every station in
/// `[start_idx_s, start_idx_s + len)`.  `len == 0` is accepted as a no-op.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// Non-empty slices must point to valid contiguous `double` values.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_add_axial_acceleration_limits(
    robot: *mut CoppRobot,
    start_idx_s: usize,
    len: usize,
    acceleration_max: CoppSliceF64,
    acceleration_min: CoppSliceF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    add_axial_limits_common(
        AxialLimitKind::Acceleration,
        robot,
        start_idx_s,
        len,
        acceleration_max,
        acceleration_min,
    )
}

/// Add station-varying axial acceleration limits over an existing robot interval.
///
/// `acceleration_max` and `acceleration_min` must be matrix views with shape
/// `dim x N`.  Column `j` is applied to station `start_idx_s + j`. `N == 0`
/// is accepted as a no-op when both matrices have shape `dim x 0`. Contiguous
/// column-major input is the fast path; other valid layouts are copied once
/// into column-major temporary storage.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// Non-empty matrices must point to valid `double` arrays for their declared
/// layouts.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_add_axial_acceleration_limits_matrix(
    robot: *mut CoppRobot,
    start_idx_s: usize,
    acceleration_max: CoppMatrixViewF64,
    acceleration_min: CoppMatrixViewF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    add_axial_limits_matrix_common(
        AxialLimitKind::Acceleration,
        robot,
        start_idx_s,
        acceleration_max,
        acceleration_min,
    )
}

/// Add broadcast axial jerk limits over an existing robot station interval.
///
/// `jerk_max` and `jerk_min` must both have length equal to the robot
/// dimension.  The same per-axis limits are applied to every station in
/// `[start_idx_s, start_idx_s + len)`.  `len == 0` is accepted as a no-op.
///
/// The robot must already contain third-order path derivative data over the
/// same interval, for example via `copp_robot_set_q_3rd` or
/// `copp_robot_sample_path_3rd`.
///
/// # Example
/// The example below applies broadcast jerk limits after third-order path
/// derivative data has been stored.
///
/// ```c
/// double jmax[] = {5.0, 5.0, 5.0};
/// double jmin[] = {-5.0, -5.0, -5.0};
/// check(copp_add_axial_jerk_limits(
///     robot,
///     0,
///     n,
///     (struct CoppSliceF64){jmax, 3},
///     (struct CoppSliceF64){jmin, 3}));
/// ```
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// Non-empty slices must point to valid contiguous `double` values.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_add_axial_jerk_limits(
    robot: *mut CoppRobot,
    start_idx_s: usize,
    len: usize,
    jerk_max: CoppSliceF64,
    jerk_min: CoppSliceF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    add_axial_limits_common(
        AxialLimitKind::Jerk,
        robot,
        start_idx_s,
        len,
        jerk_max,
        jerk_min,
    )
}

/// Add station-varying axial jerk limits over an existing robot interval.
///
/// `jerk_max` and `jerk_min` must be matrix views with shape `dim x N`. Column
/// `j` is applied to station `start_idx_s + j`.  `N == 0` is accepted as a
/// no-op when both matrices have shape `dim x 0`. Contiguous column-major input
/// is the fast path; other valid layouts are copied once into column-major
/// temporary storage.
///
/// The robot must already contain third-order path derivative data over the
/// same interval, for example via `copp_robot_set_q_3rd` or
/// `copp_robot_sample_path_3rd`.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// Non-empty matrices must point to valid `double` arrays for their declared
/// layouts.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_add_axial_jerk_limits_matrix(
    robot: *mut CoppRobot,
    start_idx_s: usize,
    jerk_max: CoppMatrixViewF64,
    jerk_min: CoppMatrixViewF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    add_axial_limits_matrix_common(AxialLimitKind::Jerk, robot, start_idx_s, jerk_max, jerk_min)
}

/// Add broadcast axial torque limits over an existing robot station interval.
///
/// `torque_max` and `torque_min` must both have length equal to the robot
/// dimension.  The same per-axis limits are applied to every station in
/// `[start_idx_s, start_idx_s + len)`.  `len == 0` is accepted as a no-op.
///
/// The robot must already contain second-order path derivative data over the
/// same interval, for example via `copp_robot_set_q_2nd` or
/// `copp_robot_sample_path_2nd`.
///
/// If the robot has no stored inverse-dynamics callback, point dynamics
/// (`tau = ddq`) is used.
///
/// # Example
/// The example below applies broadcast torque limits over an interval that
/// already has second-order path derivative data.
///
/// ```c
/// double tau_max[] = {40.0, 40.0, 40.0};
/// double tau_min[] = {-40.0, -40.0, -40.0};
/// check(copp_add_axial_torque_limits(
///     robot,
///     0,
///     n,
///     (struct CoppSliceF64){tau_max, 3},
///     (struct CoppSliceF64){tau_min, 3}));
/// ```
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// Non-empty slices must point to valid contiguous `double` values.
/// If a callback was set with `copp_robot_set_inverse_dynamics`, it must remain
/// valid for this call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_add_axial_torque_limits(
    robot: *mut CoppRobot,
    start_idx_s: usize,
    len: usize,
    torque_max: CoppSliceF64,
    torque_min: CoppSliceF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    add_axial_limits_common(
        AxialLimitKind::Torque,
        robot,
        start_idx_s,
        len,
        torque_max,
        torque_min,
    )
}

/// Add station-varying axial torque limits over an existing robot interval.
///
/// `torque_max` and `torque_min` must be matrix views with shape `dim x N`.
/// Column `j` is applied to station `start_idx_s + j`.  `N == 0` is accepted
/// as a no-op when both matrices have shape `dim x 0`. Contiguous column-major
/// input is the fast path; other valid layouts are copied once into
/// column-major temporary storage.
///
/// The robot must already contain second-order path derivative data over the
/// same interval, for example via `copp_robot_set_q_2nd` or
/// `copp_robot_sample_path_2nd`.
///
/// If the robot has no stored inverse-dynamics callback, point dynamics
/// (`tau = ddq`) is used.
///
/// # Safety
/// `robot` must be a non-null handle returned by `copp_robot_create`.
/// Non-empty matrices must point to valid `double` arrays for their declared
/// layouts. If a callback was set with `copp_robot_set_inverse_dynamics`, it
/// must remain valid for this call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_add_axial_torque_limits_matrix(
    robot: *mut CoppRobot,
    start_idx_s: usize,
    torque_max: CoppMatrixViewF64,
    torque_min: CoppMatrixViewF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    add_axial_limits_matrix_common(
        AxialLimitKind::Torque,
        robot,
        start_idx_s,
        torque_max,
        torque_min,
    )
}

enum AxialLimitKind {
    Velocity,
    Acceleration,
    Jerk,
    Torque,
}

fn add_axial_limits_common(
    kind: AxialLimitKind,
    robot: *mut CoppRobot,
    start_idx_s: usize,
    len: usize,
    upper: CoppSliceF64,
    lower: CoppSliceF64,
) -> CoppStatus {
    if robot.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: `robot` was checked for null above; the C ABI contract
        // requires it to be a live handle created by this module.
        let inner = unsafe { CoppRobot::inner_mut(robot) }.ok_or(CoppStatus::NullPointer)?;
        // SAFETY: The C ABI contract requires non-empty input slices to point
        // to valid contiguous `double` arrays for the duration of this call.
        let upper = unsafe { upper.as_slice()? };
        // SAFETY: Same input-slice contract as above.
        let lower = unsafe { lower.as_slice()? };

        if len == 0 {
            if upper.len() == inner.robot.dim() && lower.len() == inner.robot.dim() {
                return Ok(CoppStatus::Ok);
            }
            return Err(CoppStatus::ConstraintNoMatchDimensions);
        }

        match kind {
            AxialLimitKind::Velocity => inner
                .robot
                .with_axial_velocity((upper, len), (lower, len), start_idx_s)
                .map(|_| ())
                .map_err(CoppError::from),
            AxialLimitKind::Acceleration => inner
                .robot
                .with_axial_acceleration((upper, len), (lower, len), start_idx_s)
                .map(|_| ())
                .map_err(CoppError::from),
            AxialLimitKind::Jerk => inner
                .robot
                .with_axial_jerk((upper, len), (lower, len), start_idx_s)
                .map(|_| ())
                .map_err(CoppError::from),
            AxialLimitKind::Torque => inner
                .robot
                .with_axial_torque((upper, len), (lower, len), start_idx_s)
                .map(|_| ()),
        }
        .map_err(|error| CoppStatus::from(&error))?;

        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

fn add_axial_limits_matrix_common(
    kind: AxialLimitKind,
    robot: *mut CoppRobot,
    start_idx_s: usize,
    upper: CoppMatrixViewF64,
    lower: CoppMatrixViewF64,
) -> CoppStatus {
    if robot.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: `robot` was checked for null above; the C ABI contract
        // requires it to be a live handle created by this module.
        let inner = unsafe { CoppRobot::inner_mut(robot) }.ok_or(CoppStatus::NullPointer)?;
        // SAFETY: The C ABI contract requires non-empty matrix views to point
        // to valid `double` arrays for their declared layouts.
        let upper = unsafe { upper.as_input_matrix()? };
        let lower = unsafe { lower.as_input_matrix()? };
        let upper = upper.as_view();
        let lower = lower.as_view();

        match kind {
            AxialLimitKind::Velocity => inner
                .robot
                .with_axial_velocity(&upper, &lower, start_idx_s)
                .map(|_| ())
                .map_err(CoppError::from),
            AxialLimitKind::Acceleration => inner
                .robot
                .with_axial_acceleration(&upper, &lower, start_idx_s)
                .map(|_| ())
                .map_err(CoppError::from),
            AxialLimitKind::Jerk => inner
                .robot
                .with_axial_jerk(&upper, &lower, start_idx_s)
                .map(|_| ())
                .map_err(CoppError::from),
            AxialLimitKind::Torque => inner
                .robot
                .with_axial_torque(&upper, &lower, start_idx_s)
                .map(|_| ()),
        }
        .map_err(|error| CoppStatus::from(&error))?;

        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Release a robot handle created by `copp_robot_create`.
///
/// Passing null is allowed and has no effect.
///
/// # Safety
/// `robot` must either be null or a handle returned by `copp_robot_create` that
/// has not already been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_free(robot: *mut CoppRobot) {
    if robot.is_null() {
        clear_last_error();
        return;
    }

    // SAFETY: The C ABI contract requires `robot` to come from
    // `Box::into_raw` in `copp_robot_create` and to be freed at most once.
    unsafe {
        drop(Box::from_raw(robot.cast::<CoppRobotInner>()));
    }
    clear_last_error();
}
