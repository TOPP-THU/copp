//! Python bindings for COPP.
//!
//! This module is compiled only with the `python` feature.

use pyo3::prelude::*;

mod array;
mod error;
mod interpolation;
mod objective;
mod path;
mod robot;
mod solver;

/// Runtime module documentation exposed as `copp_py._native.__doc__`.
///
/// The public pure-Python facade re-exports this module, but setting the native
/// module doc keeps `help(copp_py._native)` and direct debugging sessions
/// useful.
const MODULE_DOC: &str = r#"Python bindings for copp-py.

The public package is intended to be imported as:

    import copp_py as copp

This native extension contains the low-level PyO3 bindings for COPP path
construction, robot and constraint modeling, interpolation utilities,
objective terms, Clarabel-backed optimization helpers, and the TOPP/COPP
solver families exposed by the Python package. Most users should import
`copp_py` and use public Python namespaces such as `copp.path`,
`copp.robot`, `copp.constraints`, `copp.objective`, `copp.interpolation`,
`copp.clarabel`, and `copp.solver.*` instead of importing this module
directly.

Numerical inputs accept NumPy-compatible ArrayLike values and are copied into
contiguous float64 buffers at the wrapper boundary. Python-level format errors,
such as invalid array dimensionality or mutually exclusive keyword arguments,
are reported as ValueError. Errors returned by the Rust COPP core are reported
as copp.CoppError and typed subclasses such as copp.PathError and
copp.ConstraintError.
"#;

/// Return the copp-py package version.
///
/// Returns
/// -------
/// str
///     The version compiled into the native extension.
#[pyfunction]
fn version() -> &'static str {
    env!("CARGO_PKG_VERSION")
}

/// Initialize the compiled Python extension [`PyModule`].
///
/// All Python-facing classes and functions are registered here. The function is
/// called by CPython when importing `copp_py._native`.
#[pymodule]
#[pyo3(name = "_native")]
pub fn native(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__doc__", MODULE_DOC)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    error::register(m)?;
    m.add_function(wrap_pyfunction!(version, m)?)?;
    interpolation::register(m)?;
    path::register(m)?;
    objective::register(m)?;
    robot::register(m)?;
    solver::register(m)?;
    Ok(())
}
