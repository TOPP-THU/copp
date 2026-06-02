//! Foreign-function interfaces for COPP.
//!
//! Language-specific ABI surfaces live in feature-gated submodules.

#[cfg(feature = "c")]
pub mod c;

#[cfg(feature = "cpp")]
pub mod cpp;

#[cfg(feature = "python")]
pub(crate) mod python;

#[cfg(feature = "c")]
pub use c::*;
