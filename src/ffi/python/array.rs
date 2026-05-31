//! NumPy array validation helpers shared by Python bindings.
//!
//! These functions keep wrapper-level errors consistent before data reaches the
//! Rust COPP core. They intentionally report Python argument-format issues as
//! standard Python exceptions instead of [`crate::ffi::python::error::CoppError`].

use numpy::{PyReadonlyArray1, PyReadonlyArray2};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyAnyMethods;

/// Convert any Python array-like object into a contiguous one-dimensional
/// `float64` vector.
///
/// The conversion uses `numpy.ascontiguousarray(value, dtype=numpy.float64)`.
/// This makes public Python entry points consistently accept lists, tuples,
/// NumPy arrays with compatible dtypes, and other NumPy-compatible array-like
/// values while keeping Rust-facing data owned and contiguous.
pub(crate) fn array_like_to_vec_f64(name: &str, value: &Bound<'_, PyAny>) -> PyResult<Vec<f64>> {
    let array_obj = as_contiguous_float64(name, value)?;
    let array = array_obj
        .extract::<PyReadonlyArray1<'_, f64>>()
        .map_err(|_| {
            PyValueError::new_err(format!(
                "`{name}` must be convertible to a one-dimensional float64 array"
            ))
        })?;
    let values = array.as_slice().map_err(|_| {
        PyValueError::new_err(format!(
            "`{name}` must be convertible to a contiguous one-dimensional float64 array"
        ))
    })?;
    Ok(values.to_vec())
}

/// Convert any Python array-like object into row-major two-dimensional data.
///
/// The returned tuple is `(rows, cols, data)` where `data` is C-contiguous
/// row-major `float64` values.
pub(crate) fn array_like_to_vec2_f64(
    name: &str,
    value: &Bound<'_, PyAny>,
) -> PyResult<(usize, usize, Vec<f64>)> {
    let array_obj = as_contiguous_float64(name, value)?;
    let array = array_obj
        .extract::<PyReadonlyArray2<'_, f64>>()
        .map_err(|_| {
            PyValueError::new_err(format!(
                "`{name}` must be convertible to a two-dimensional float64 array"
            ))
        })?;
    let view = array.as_array();
    if !view.is_standard_layout() {
        return Err(PyValueError::new_err(format!(
            "`{name}` must be convertible to a C-contiguous two-dimensional float64 array"
        )));
    }
    let (rows, cols) = view.dim();
    let data = array.as_slice().map_err(|_| {
        PyValueError::new_err(format!(
            "`{name}` must be convertible to a C-contiguous two-dimensional float64 array"
        ))
    })?;
    Ok((rows, cols, data.to_vec()))
}

/// Call `numpy.ascontiguousarray(value, dtype=numpy.float64)`.
fn as_contiguous_float64<'py>(
    name: &str,
    value: &Bound<'py, PyAny>,
) -> PyResult<Bound<'py, PyAny>> {
    let py = value.py();
    let numpy = py.import("numpy").map_err(|error| {
        PyValueError::new_err(format!(
            "`{name}` requires NumPy for ArrayLike conversion: {error}"
        ))
    })?;
    let dtype = numpy.getattr("float64")?;
    numpy.getattr("ascontiguousarray")?.call1((value, dtype))
}
