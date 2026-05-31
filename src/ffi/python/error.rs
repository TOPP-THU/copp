//! Error conversion utilities for the Python binding layer.
//!
//! The Python API keeps wrapper argument errors as standard Python exceptions
//! such as `ValueError`, while errors returned by the Rust COPP core become
//! [`CoppError`].

use crate::diag::CoppError as RustCoppError;
use pyo3::prelude::*;

// Python-visible exception class for Rust core failures. The macro carries the
// runtime Python docstring in its string argument below.
pyo3::create_exception!(
    copp_py,
    CoppError,
    pyo3::exceptions::PyException,
    "Base exception for errors reported by the Rust COPP core.\n\n\
Python wrapper argument errors, such as invalid dtype or mutually exclusive \
keyword arguments, use standard Python exceptions like ValueError. This \
exception is reserved for errors returned by the underlying COPP library."
);

pyo3::create_exception!(
    copp_py,
    ConstraintError,
    CoppError,
    "Exception for errors reported by Rust constraint storage and query APIs.\n\n\
This is a subclass of CoppError."
);

pyo3::create_exception!(
    copp_py,
    PathError,
    CoppError,
    "Exception for errors reported by Rust path construction and evaluation APIs.\n\n\
This is a subclass of CoppError."
);

/// Convert the crate-wide Rust error type into the Python [`CoppError`] class.
///
/// The current binding preserves the Rust display message as the Python
/// exception text. If a structured Python error hierarchy is added later, this
/// is the central place to branch by [`RustCoppError`] variant.
pub(crate) fn to_py_err(error: RustCoppError) -> PyErr {
    let message = error.to_string();
    match &error {
        RustCoppError::ConstraintError(_) => ConstraintError::new_err(message),
        RustCoppError::PathError(_) => PathError::new_err(message),
        _ => CoppError::new_err(message),
    }
}

/// Register Python exception classes on the native [`PyModule`].
pub(crate) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("CoppError", m.py().get_type::<CoppError>())?;
    m.add("ConstraintError", m.py().get_type::<ConstraintError>())?;
    m.add("PathError", m.py().get_type::<PathError>())?;
    Ok(())
}
