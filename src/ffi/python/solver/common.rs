//! Shared helpers for Python solver wrappers.

use crate::diag::Verbosity as RustVerbosity;
use pyo3::Borrowed;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyAnyMethods;

/// Register shared solver enums on the native [`PyModule`].
pub(super) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyVerbosity>()?;
    Ok(())
}

/// Verbosity level used by Rust solver diagnostics.
#[pyclass(
    name = "Verbosity",
    module = "copp_py._native",
    eq,
    eq_int,
    rename_all = "SCREAMING_SNAKE_CASE",
    from_py_object
)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum PyVerbosity {
    /// Disable solver diagnostic logging.
    Silent,
    /// Emit start/end and summary-level solver messages.
    Summary,
    /// Emit detailed solver messages.
    Debug,
    /// Emit very detailed per-step solver messages.
    Trace,
}

impl PyVerbosity {
    /// Convert from the Python enum to the Rust diagnostics enum.
    pub(super) fn to_rust(self) -> RustVerbosity {
        match self {
            Self::Silent => RustVerbosity::Silent,
            Self::Summary => RustVerbosity::Summary,
            Self::Debug => RustVerbosity::Debug,
            Self::Trace => RustVerbosity::Trace,
        }
    }

    /// Convert from the Rust diagnostics enum to the Python enum.
    pub(super) fn from_rust(value: RustVerbosity) -> Self {
        match value {
            RustVerbosity::Silent => Self::Silent,
            RustVerbosity::Summary => Self::Summary,
            RustVerbosity::Debug => Self::Debug,
            RustVerbosity::Trace => Self::Trace,
        }
    }
}

/// Internal parser for `Verbosity | str | None` solver-option arguments.
#[derive(Clone, Copy)]
pub(super) struct VerbosityArg(
    /// Parsed verbosity level.
    pub(super) PyVerbosity,
);

impl FromPyObject<'_, '_> for VerbosityArg {
    /// [`PyErr`] returned when parsing fails.
    type Error = PyErr;

    /// Parse enum values, accepted strings, or `None`.
    fn extract(obj: Borrowed<'_, '_, PyAny>) -> Result<Self, Self::Error> {
        if obj.is_none() {
            return Ok(Self(PyVerbosity::from_rust(RustVerbosity::default())));
        }

        if let Ok(verbosity) = obj.extract::<PyVerbosity>() {
            return Ok(Self(verbosity));
        }

        if let Ok(text) = obj.extract::<&str>() {
            return match normalize_token(text).as_str() {
                "silent" => Ok(Self(PyVerbosity::Silent)),
                "summary" => Ok(Self(PyVerbosity::Summary)),
                "debug" => Ok(Self(PyVerbosity::Debug)),
                "trace" => Ok(Self(PyVerbosity::Trace)),
                _ => Err(PyValueError::new_err(
                    "`verbosity` must be Verbosity.SILENT, Verbosity.SUMMARY, Verbosity.DEBUG, Verbosity.TRACE, or one of \"silent\", \"summary\", \"debug\", \"trace\"",
                )),
            };
        }

        Err(PyValueError::new_err(
            "`verbosity` must be a Verbosity value or a string",
        ))
    }
}

/// Parse a Python value into [`PyVerbosity`].
pub(super) fn parse_verbosity(value: &Bound<'_, PyAny>) -> PyResult<PyVerbosity> {
    VerbosityArg::extract(value.as_borrowed()).map(|verbosity| verbosity.0)
}

/// Normalize user-facing option strings for lenient matching.
pub(super) fn normalize_token(text: &str) -> String {
    text.trim().to_ascii_lowercase().replace('-', "_")
}
