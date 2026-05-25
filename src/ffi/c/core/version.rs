//! Exported C ABI functions.

use crate::ffi::c::CoppStatus;
use std::ffi::c_char;

const COPP_VERSION: &[u8] = concat!(env!("CARGO_PKG_VERSION"), "\0").as_bytes();

/// Return the COPP library version.
///
/// The returned pointer is a static null-terminated string owned by the library.
#[unsafe(no_mangle)]
pub extern "C" fn copp_version() -> *const c_char {
    COPP_VERSION.as_ptr().cast()
}

/// Return a short static message for a COPP status code.
///
/// The returned pointer is a static null-terminated string owned by the library.
#[unsafe(no_mangle)]
pub extern "C" fn copp_status_message(status: CoppStatus) -> *const c_char {
    status.message_ptr()
}
