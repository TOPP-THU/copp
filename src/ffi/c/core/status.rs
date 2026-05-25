//! Status-code mapping for the C ABI.

use crate::diag::{ConstraintError, CoppError, PathError};
use std::{
    any::Any,
    cell::RefCell,
    ffi::{CStr, c_char},
    slice,
};

const LAST_ERROR_MAX_BYTES: usize = 64 * 1024;
const EMPTY_C_MESSAGE: &[u8] = b"\0";

#[derive(Clone, Debug)]
struct LastError {
    code: CoppStatus,
    message: Vec<u8>,
}

impl LastError {
    fn empty() -> Self {
        Self {
            code: CoppStatus::Ok,
            message: EMPTY_C_MESSAGE.to_vec(),
        }
    }
}

thread_local! {
    static LAST_ERROR: RefCell<LastError> = RefCell::new(LastError::empty());
}

fn truncate_utf8_to_cap(message: &mut String) {
    if message.len() <= LAST_ERROR_MAX_BYTES {
        return;
    }

    let mut end = LAST_ERROR_MAX_BYTES;
    while end > 0 && !message.is_char_boundary(end) {
        end -= 1;
    }
    message.truncate(end);
}

fn normalize_message(bytes: &[u8]) -> Vec<u8> {
    let bytes = if bytes.len() > LAST_ERROR_MAX_BYTES {
        &bytes[..LAST_ERROR_MAX_BYTES]
    } else {
        bytes
    };
    let mut message = String::from_utf8_lossy(bytes).into_owned();
    message = message.replace('\0', "\u{FFFD}");
    truncate_utf8_to_cap(&mut message);

    let mut bytes = message.into_bytes();
    bytes.push(0);
    bytes
}

pub(crate) fn clear_last_error() {
    LAST_ERROR.with(|last_error| {
        *last_error.borrow_mut() = LastError::empty();
    });
}

pub(crate) fn set_last_error_bytes(status: CoppStatus, message: &[u8]) {
    let status = if status == CoppStatus::Ok {
        CoppStatus::SolverOther
    } else {
        status
    };
    let message = normalize_message(message);
    LAST_ERROR.with(|last_error| {
        *last_error.borrow_mut() = LastError {
            code: status,
            message,
        };
    });
}

pub(crate) fn set_last_error_message(status: CoppStatus, message: impl AsRef<str>) {
    set_last_error_bytes(status, message.as_ref().as_bytes());
}

pub(crate) fn panic_to_status(payload: Box<dyn Any + Send>) -> CoppStatus {
    match payload.downcast::<CoppStatus>() {
        Ok(status) => {
            let status = if *status == CoppStatus::Ok {
                CoppStatus::Panic
            } else {
                *status
            };
            set_last_error_bytes(status, status.message().to_bytes());
            status
        }
        Err(payload) => match payload.downcast::<String>() {
            Ok(message) => {
                set_last_error_message(CoppStatus::Panic, *message);
                CoppStatus::Panic
            }
            Err(payload) => match payload.downcast::<&'static str>() {
                Ok(message) => {
                    set_last_error_message(CoppStatus::Panic, *message);
                    CoppStatus::Panic
                }
                Err(_) => {
                    set_last_error_bytes(CoppStatus::Panic, CoppStatus::Panic.message().to_bytes());
                    CoppStatus::Panic
                }
            },
        },
    }
}

fn ensure_last_error(status: CoppStatus) {
    if status == CoppStatus::Ok {
        clear_last_error();
        return;
    }

    LAST_ERROR.with(|last_error| {
        let mut last_error = last_error.borrow_mut();
        if last_error.code == CoppStatus::Ok {
            *last_error = LastError {
                code: status,
                message: normalize_message(status.message().to_bytes()),
            };
        }
    });
}

pub(crate) fn current_last_error_message_lossy() -> Option<String> {
    LAST_ERROR.with(|last_error| {
        let last_error = last_error.borrow();
        if last_error.code == CoppStatus::Ok {
            return None;
        }
        let bytes = last_error
            .message
            .strip_suffix(&[0])
            .unwrap_or(&last_error.message);
        Some(String::from_utf8_lossy(bytes).into_owned())
    })
}

/// C ABI status code returned by COPP FFI functions.
///
/// The numeric ranges are grouped by subsystem so future bindings can preserve
/// source compatibility while still reporting precise failure classes.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CoppStatus {
    /// Operation completed successfully.
    Ok = 0,

    /// A required pointer argument was null.
    NullPointer = 1,
    /// A slice length or vector length was invalid.
    InvalidLength = 2,
    /// A matrix/vector shape was invalid.
    InvalidShape = 3,
    /// An unsupported enum value or option was provided through the C ABI.
    InvalidArgument = 4,
    /// COPP caught an internal panic at the C ABI boundary.
    Panic = 5,
    /// A memory allocation failed or an output buffer could not be created.
    AllocationFailed = 6,

    /// Filesystem or OS I/O failure.
    IoError = 100,

    /// Constraint input `s` is not strictly increasing.
    ConstraintNonIncreasingS = 200,
    /// Constraint input dimensions do not match the expected shape.
    ConstraintNoMatchDimensions = 201,
    /// Constraint input ordering does not match the expected contract.
    ConstraintNoMatchOrder = 202,
    /// Signed upper/lower bounds do not satisfy strict feasibility rules.
    ConstraintInvalidSignedBounds = 203,
    /// Requested station interval is outside the stored constraint range.
    ConstraintOutOfSBounds = 204,
    /// Profile `a` violates positivity requirements.
    ConstraintNonPositiveA = 205,
    /// Linearization floor is not strictly positive.
    ConstraintNonPositiveLinearizationFloor = 206,
    /// Required path derivative data is missing.
    ConstraintNoGivenQInfo = 207,
    /// Linearized jerk constraints are unavailable for the requested station.
    ConstraintLinearJerkNotAvailable = 208,
    /// Required dynamic-model data is unavailable.
    ConstraintNoDynamic = 209,
    /// Reference profile is infeasible under current constraints.
    ConstraintInfeasibleReference = 210,
    /// Requested constraint interval is empty.
    ConstraintEmptyInterval = 211,

    /// Path dimension is invalid.
    PathInvalidDimension = 300,
    /// Path parameter range is invalid.
    PathInvalidRange = 301,
    /// Spline order is invalid.
    PathInvalidOrder = 302,
    /// Path matrix/tensor dimensions are incompatible.
    PathDimensionMismatch = 303,
    /// Not enough waypoints were supplied to build a path.
    PathNotEnoughWaypoints = 304,
    /// Query parameter is outside the configured path range.
    PathOutOfRangeS = 305,
    /// Requested spline boundary condition is unsupported.
    PathUnsupportedBoundary = 306,
    /// Internal spline/path linear system is singular.
    PathSingularSystem = 307,
    /// Requested path derivative order is not supported by this path.
    PathUnsupportedDerivativeOrder = 308,

    /// Solver reported infeasibility.
    SolverInfeasible = 400,
    /// Solver reported unboundedness.
    SolverUnbounded = 401,
    /// Solver rejected the input model or data.
    SolverInvalidInput = 402,
    /// Solver rejected the provided options.
    SolverInvalidOptions = 403,
    /// Clarabel returned an internal solver error.
    ClarabelSolverError = 404,
    /// Clarabel terminated with a non-success status.
    ClarabelSolverStatus = 405,
    /// Backend-specific or uncategorized solver/runtime error.
    SolverOther = 499,

    /// User-provided robot inverse dynamics failed.
    RobotDynamicsError = 500,
}

impl CoppStatus {
    /// Short static status message suitable for C callers.
    #[inline]
    pub fn message(self) -> &'static CStr {
        match self {
            Self::Ok => c"ok",
            Self::NullPointer => c"null pointer",
            Self::InvalidLength => c"invalid length",
            Self::InvalidShape => c"invalid shape",
            Self::InvalidArgument => c"invalid argument",
            Self::Panic => c"panic across C ABI boundary",
            Self::AllocationFailed => c"allocation failed",
            Self::IoError => c"I/O error",
            Self::ConstraintNonIncreasingS => c"constraint error: non-increasing s",
            Self::ConstraintNoMatchDimensions => c"constraint error: dimensions do not match",
            Self::ConstraintNoMatchOrder => c"constraint error: order does not match",
            Self::ConstraintInvalidSignedBounds => c"constraint error: invalid signed bounds",
            Self::ConstraintOutOfSBounds => c"constraint error: station interval out of bounds",
            Self::ConstraintNonPositiveA => c"constraint error: non-positive a",
            Self::ConstraintNonPositiveLinearizationFloor => {
                c"constraint error: non-positive linearization floor"
            }
            Self::ConstraintNoGivenQInfo => c"constraint error: missing path derivative data",
            Self::ConstraintLinearJerkNotAvailable => {
                c"constraint error: linearized jerk unavailable"
            }
            Self::ConstraintNoDynamic => c"constraint error: dynamic model unavailable",
            Self::ConstraintInfeasibleReference => c"constraint error: infeasible reference",
            Self::ConstraintEmptyInterval => c"constraint error: empty interval",
            Self::PathInvalidDimension => c"path error: invalid dimension",
            Self::PathInvalidRange => c"path error: invalid range",
            Self::PathInvalidOrder => c"path error: invalid spline order",
            Self::PathDimensionMismatch => c"path error: dimension mismatch",
            Self::PathNotEnoughWaypoints => c"path error: not enough waypoints",
            Self::PathOutOfRangeS => c"path error: s out of range",
            Self::PathUnsupportedBoundary => c"path error: unsupported boundary",
            Self::PathSingularSystem => c"path error: singular system",
            Self::PathUnsupportedDerivativeOrder => c"path error: unsupported derivative order",
            Self::SolverInfeasible => c"solver error: infeasible",
            Self::SolverUnbounded => c"solver error: unbounded",
            Self::SolverInvalidInput => c"solver error: invalid input",
            Self::SolverInvalidOptions => c"solver error: invalid options",
            Self::ClarabelSolverError => c"solver error: Clarabel internal error",
            Self::ClarabelSolverStatus => c"solver error: Clarabel status failure",
            Self::SolverOther => c"solver error: other",
            Self::RobotDynamicsError => c"robot dynamics error",
        }
    }

    /// Raw pointer to the short static status message.
    #[inline]
    pub fn message_ptr(self) -> *const c_char {
        self.message().as_ptr()
    }

    /// Complete a C ABI call by updating the thread-local last-error slot.
    #[inline]
    pub(crate) fn into_ffi_status(self) -> Self {
        ensure_last_error(self);
        self
    }
}

impl From<&CoppError> for CoppStatus {
    fn from(error: &CoppError) -> Self {
        let status = match error {
            CoppError::IoError(_) => Self::IoError,
            CoppError::ConstraintError(error) => Self::from(error),
            CoppError::PathError(error) => Self::from(error),
            CoppError::RobotDynamicsError(_) => Self::RobotDynamicsError,
            CoppError::Infeasible(_, _) => Self::SolverInfeasible,
            CoppError::Unbounded(_, _) => Self::SolverUnbounded,
            CoppError::InvalidInput(_, _) => Self::SolverInvalidInput,
            CoppError::InvalidOptions(_, _) => Self::SolverInvalidOptions,
            CoppError::ClarabelSolverError(_, _) => Self::ClarabelSolverError,
            CoppError::ClarabelSolverStatus(_, _) => Self::ClarabelSolverStatus,
            CoppError::Other(_, _) => Self::SolverOther,
        };
        set_last_error_message(status, error.to_string());
        status
    }
}

impl From<&ConstraintError> for CoppStatus {
    fn from(error: &ConstraintError) -> Self {
        let status = match error {
            ConstraintError::NonIncreasingS { .. } => Self::ConstraintNonIncreasingS,
            ConstraintError::NoMatchDimensions => Self::ConstraintNoMatchDimensions,
            ConstraintError::NoMatchOrder => Self::ConstraintNoMatchOrder,
            ConstraintError::InvalidSignedBounds { .. } => Self::ConstraintInvalidSignedBounds,
            ConstraintError::OutOfSBounds { .. } => Self::ConstraintOutOfSBounds,
            ConstraintError::NonPositiveA => Self::ConstraintNonPositiveA,
            ConstraintError::NonPositiveLinearizationFloor => {
                Self::ConstraintNonPositiveLinearizationFloor
            }
            ConstraintError::NoGivenQInfo => Self::ConstraintNoGivenQInfo,
            ConstraintError::LinearJerkNotAvailable { .. } => {
                Self::ConstraintLinearJerkNotAvailable
            }
            ConstraintError::NoDynamic => Self::ConstraintNoDynamic,
            ConstraintError::InfeasibleReference => Self::ConstraintInfeasibleReference,
            ConstraintError::EmptyInterval { .. } => Self::ConstraintEmptyInterval,
        };
        set_last_error_message(status, error.to_string());
        status
    }
}

impl From<&PathError> for CoppStatus {
    fn from(error: &PathError) -> Self {
        let status = match error {
            PathError::InvalidDimension { .. } => Self::PathInvalidDimension,
            PathError::InvalidRange { .. } => Self::PathInvalidRange,
            PathError::InvalidOrder { .. } => Self::PathInvalidOrder,
            PathError::DimensionMismatch => Self::PathDimensionMismatch,
            PathError::UnsupportedDerivativeOrder { .. } => Self::PathUnsupportedDerivativeOrder,
            PathError::NotEnoughWaypoints { .. } => Self::PathNotEnoughWaypoints,
            PathError::OutOfRangeS { .. } => Self::PathOutOfRangeS,
            PathError::UnsupportedBoundary { .. } => Self::PathUnsupportedBoundary,
            PathError::SingularSystem => Self::PathSingularSystem,
        };
        set_last_error_message(status, error.to_string());
        status
    }
}

/// Return the status code associated with the current thread's last error.
///
/// A successful COPP C ABI call clears this slot back to `COPP_STATUS_OK`.
#[unsafe(no_mangle)]
pub extern "C" fn copp_last_error_code() -> CoppStatus {
    LAST_ERROR.with(|last_error| last_error.borrow().code)
}

/// Return the current thread's last detailed error message as UTF-8.
///
/// The returned pointer is owned by COPP and remains valid until the next COPP
/// C ABI call on the same thread updates or clears the last-error slot.
/// When no error is stored, this returns an empty string.
#[unsafe(no_mangle)]
pub extern "C" fn copp_last_error_message() -> *const c_char {
    LAST_ERROR.with(|last_error| last_error.borrow().message.as_ptr().cast())
}

/// Return the length in bytes of the current thread's last UTF-8 error message.
///
/// The terminating null byte is not included in the returned length.
#[unsafe(no_mangle)]
pub extern "C" fn copp_last_error_message_len() -> usize {
    LAST_ERROR.with(|last_error| last_error.borrow().message.len().saturating_sub(1))
}

/// Copy the current thread's last UTF-8 error message into a caller buffer.
///
/// `out_len`, when non-null, receives the full message length in bytes,
/// excluding the terminating null byte, even when `capacity` is too small.
/// If `buffer` is non-null and `capacity > 0`, COPP always writes a
/// null-terminated, possibly truncated UTF-8 prefix.
///
/// # Safety
/// `buffer` must be valid for `capacity` writable bytes when `capacity > 0`.
/// `out_len`, when non-null, must be valid for one `size_t` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_last_error_message_copy(
    buffer: *mut c_char,
    capacity: usize,
    out_len: *mut usize,
) -> CoppStatus {
    LAST_ERROR.with(|last_error| {
        let last_error = last_error.borrow();
        let message = last_error
            .message
            .strip_suffix(&[0])
            .unwrap_or(&last_error.message);

        if !out_len.is_null() {
            // SAFETY: The C ABI contract requires a non-null `out_len` to be
            // valid for one write.
            unsafe {
                out_len.write(message.len());
            }
        }

        if capacity == 0 {
            return CoppStatus::Ok;
        }
        if buffer.is_null() {
            return CoppStatus::NullPointer.into_ffi_status();
        }

        let mut copy_len = message.len().min(capacity - 1);
        while copy_len > 0 && std::str::from_utf8(&message[..copy_len]).is_err() {
            copy_len -= 1;
        }

        // SAFETY: `buffer` was checked for null and the C ABI contract
        // requires it to be valid for `capacity` writable bytes.
        unsafe {
            ptr_copy_nonoverlapping(message.as_ptr(), buffer.cast::<u8>(), copy_len);
            buffer.cast::<u8>().add(copy_len).write(0);
        }
        CoppStatus::Ok
    })
}

/// Clear the current thread's last detailed error message.
#[unsafe(no_mangle)]
pub extern "C" fn copp_clear_last_error() {
    clear_last_error();
}

/// Set the current thread's last detailed error message from a C string.
///
/// The message is interpreted as UTF-8 with lossy replacement for invalid byte
/// sequences. Interior null bytes cannot appear in this C-string variant; use
/// `copp_set_last_error_message_n` when the message length is known.
///
/// # Safety
/// `message` must point to a null-terminated byte string when non-null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_set_last_error_message(
    status: CoppStatus,
    message: *const c_char,
) -> CoppStatus {
    if message.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }
    // SAFETY: The C ABI contract requires `message` to be a valid
    // null-terminated byte string.
    let bytes = unsafe { CStr::from_ptr(message).to_bytes() };
    set_last_error_bytes(status, bytes);
    CoppStatus::Ok
}

/// Set the current thread's last detailed error message from a byte slice.
///
/// The message is interpreted as UTF-8 with lossy replacement for invalid byte
/// sequences, interior null bytes are replaced, and the stored message is
/// capped at 64 KiB.
///
/// # Safety
/// `message` must be valid for `len` reads when `len > 0`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_set_last_error_message_n(
    status: CoppStatus,
    message: *const c_char,
    len: usize,
) -> CoppStatus {
    if len > 0 && message.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }
    let bytes = if len == 0 {
        &[]
    } else {
        // SAFETY: The C ABI contract requires `message` to be valid for
        // `len` readable bytes.
        unsafe { slice::from_raw_parts(message.cast::<u8>(), len) }
    };
    set_last_error_bytes(status, bytes);
    CoppStatus::Ok
}

unsafe fn ptr_copy_nonoverlapping(src: *const u8, dst: *mut u8, len: usize) {
    // SAFETY: This helper only forwards the caller's checked pointer/length
    // contract to the standard library primitive.
    unsafe {
        std::ptr::copy_nonoverlapping(src, dst, len);
    }
}
