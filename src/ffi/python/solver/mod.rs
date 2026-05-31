//! Python wrappers for solver problem descriptors, options, and calls.
//!
//! The public Python API remains flat on `copp_py._native`, while this Rust
//! module is split by solver family to keep bindings maintainable.

use pyo3::prelude::*;

mod clarabel;
mod common;
mod copp2;
mod copp3;
mod topp2;
mod topp3;

/// Register solver-related classes and functions on the native [`PyModule`].
pub(crate) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    common::register(m)?;
    clarabel::register(m)?;
    topp2::register(m)?;
    topp3::register(m)?;
    copp2::register(m)?;
    copp3::register(m)?;
    Ok(())
}
