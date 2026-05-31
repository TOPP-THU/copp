//! Python wrappers for TOPP/COPP interpolation helpers.
//!
//! This module is intentionally thin: it copies Python ArrayLike inputs into
//! owned contiguous `float64` buffers at the Python boundary, delegates
//! numerical checks to the Rust core, and converts owned Rust vectors back into
//! [`PyArray1`].

use crate::InterpolationMode;
use crate::copp::copp3::Topp3ProfileRef;
use crate::ffi::python::array::array_like_to_vec_f64;
use crate::ffi::python::error::to_py_err;
use numpy::{IntoPyArray, PyArray1};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

/// Register interpolation functions on the native [`PyModule`].
pub(crate) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyProfile3rd>()?;
    m.add_function(wrap_pyfunction!(a_to_b_topp2, m)?)?;
    m.add_function(wrap_pyfunction!(s_to_t_topp2, m)?)?;
    m.add_function(wrap_pyfunction!(t_to_s_topp2_uniform, m)?)?;
    m.add_function(wrap_pyfunction!(t_to_s_topp2_samples, m)?)?;
    m.add_function(wrap_pyfunction!(t_to_s_topp2, m)?)?;
    m.add_function(wrap_pyfunction!(s_to_t_topp3, m)?)?;
    m.add_function(wrap_pyfunction!(t_to_s_topp3_uniform, m)?)?;
    m.add_function(wrap_pyfunction!(t_to_s_topp3_samples, m)?)?;
    m.add_function(wrap_pyfunction!(t_to_s_topp3, m)?)?;
    Ok(())
}

/// Python-owned third-order TOPP/COPP profile.
///
/// The object stores node profiles `a = (ds/dt)^2` and `b = dds/dt`, plus
/// stationary boundary interval counts. It owns copies of the input arrays so
/// later mutation of user-provided NumPy arrays cannot silently change solver
/// inputs.
#[pyclass(name = "Profile3rd", module = "copp_py._native", skip_from_py_object)]
#[derive(Clone)]
pub(crate) struct PyProfile3rd {
    /// Node samples of `a = (ds/dt)^2`.
    a: Vec<f64>,
    /// Node samples of `b = dds/dt`.
    b: Vec<f64>,
    /// Stationary boundary interval counts as `(start, end)`.
    num_stationary: (usize, usize),
}

#[pymethods]
impl PyProfile3rd {
    /// Construct a third-order TOPP/COPP profile from node samples.
    ///
    /// Parameters
    /// ----------
    /// a : ArrayLike
    ///     One-dimensional node profile convertible to float64 for
    ///     ``a = (ds/dt)^2``.
    /// b : ArrayLike
    ///     One-dimensional node profile convertible to float64 for
    ///     ``b = dds/dt``.
    ///     Must have the same length as ``a``.
    /// num_stationary : tuple[int, int], default=(0, 0)
    ///     Stationary boundary interval counts ``(start, end)``.
    ///
    /// Raises
    /// ------
    /// ValueError
    ///     If ``a`` or ``b`` cannot be converted to a one-dimensional float64
    ///     array, or if their lengths differ.
    #[new]
    #[pyo3(
        signature = (a, b, num_stationary = (0, 0)),
        text_signature = "(a, b, num_stationary=(0, 0))"
    )]
    fn new(
        a: &Bound<'_, PyAny>,
        b: &Bound<'_, PyAny>,
        num_stationary: (usize, usize),
    ) -> PyResult<Self> {
        let a = array_like_to_vec_f64("a", a)?;
        let b = array_like_to_vec_f64("b", b)?;
        if a.len() != b.len() {
            return Err(PyValueError::new_err(format!(
                "`a` and `b` must have the same length, got {} and {}",
                a.len(),
                b.len()
            )));
        }

        Ok(Self {
            a,
            b,
            num_stationary,
        })
    }

    /// Return a copy of the node profile `a = (ds/dt)^2`.
    #[getter]
    fn a<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        PyArray1::from_vec(py, self.a.clone())
    }

    /// Return a copy of the node profile `b = dds/dt`.
    #[getter]
    fn b<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        PyArray1::from_vec(py, self.b.clone())
    }

    /// Return stationary boundary interval counts as `(start, end)`.
    #[getter]
    fn num_stationary(&self) -> (usize, usize) {
        self.num_stationary
    }

    /// Return the number of stationary intervals at the start boundary.
    #[getter]
    fn num_stationary_start(&self) -> usize {
        self.num_stationary.0
    }

    /// Return the number of stationary intervals at the end boundary.
    #[getter]
    fn num_stationary_end(&self) -> usize {
        self.num_stationary.1
    }

    /// Return the number of station nodes in the profile.
    #[getter]
    fn len(&self) -> usize {
        self.a.len()
    }

    /// Return whether this profile contains at least one station node.
    fn __bool__(&self) -> bool {
        !self.a.is_empty()
    }

    /// Return `len(self)`.
    fn __len__(&self) -> usize {
        self.a.len()
    }

    /// Return a compact representation for interactive Python sessions.
    fn __repr__(&self) -> String {
        format!(
            "Profile3rd(len={}, num_stationary={:?})",
            self.a.len(),
            self.num_stationary
        )
    }
}

impl PyProfile3rd {
    /// Borrow this Python-owned profile as Rust third-order profile parts.
    pub(crate) fn as_parts(&self) -> Topp3ProfileRef<'_> {
        (&self.a, &self.b, self.num_stationary)
    }

    /// Convert an owned Rust [`crate::copp::copp3::Topp3Profile`] into a Python profile.
    pub(crate) fn from_rust(profile: crate::copp::copp3::Topp3Profile) -> Self {
        let (a, b, num_stationary) = profile.into_parts();
        Self {
            a,
            b,
            num_stationary,
        }
    }
}

/// Compute the segment acceleration-like profile ``b`` from a TOPP2/COPP2
/// node profile ``a``.
///
/// For each segment ``[s[k], s[k + 1]]``, this returns
/// ``b[k] = 0.5 * (a[k + 1] - a[k]) / (s[k + 1] - s[k])``. In the TOPP2/COPP2
/// convention, ``a`` stores ``ds/dt`` squared at station nodes, while ``b``
/// stores ``dds/dt`` on path segments.
///
/// Parameters
/// ----------
/// s : ArrayLike
///     One-dimensional station grid convertible to float64. Must be strictly
///     increasing and contain at least two entries.
/// a : ArrayLike
///     One-dimensional node profile convertible to float64 with
///     ``len(a) == len(s)``. Values must be finite.
///
/// Returns
/// -------
/// numpy.ndarray[numpy.float64]
///     Segment profile with length ``len(s) - 1``.
///
/// Raises
/// ------
/// ValueError
///     If an input cannot be converted to a one-dimensional float64 array.
/// CoppError
///     If the COPP core rejects the grid or profile, for example because
///     lengths differ, ``s`` is not strictly increasing, or values are not
///     finite.
///
/// Examples
/// --------
/// >>> import numpy as np
/// >>> import copp_py as copp
/// >>> s = np.array([0.0, 0.5, 1.0], dtype=np.float64)
/// >>> a = np.array([1.0, 1.0, 1.0], dtype=np.float64)
/// >>> copp.a_to_b_topp2(s, a)
/// array([0., 0.])
#[pyfunction]
fn a_to_b_topp2<'py>(
    py: Python<'py>,
    s: &Bound<'py, PyAny>,
    a: &Bound<'py, PyAny>,
) -> PyResult<Bound<'py, PyArray1<f64>>> {
    let s = array_like_to_vec_f64("s", s)?;
    let a = array_like_to_vec_f64("a", a)?;
    let b = crate::solver::topp2_ra::a_to_b_topp2(&s, &a).map_err(to_py_err)?;
    Ok(b.into_pyarray(py))
}

/// Integrate a TOPP2/COPP2 node profile ``a(s)`` into cumulative arrival times
/// ``t(s)``.
///
/// The returned array ``t_s`` satisfies ``t_s[i] == t(s[i])`` and starts from
/// ``t_s[0] == t0``. The scalar ``t_final`` is equal to ``t_s[-1]``.
///
/// Parameters
/// ----------
/// s : ArrayLike
///     One-dimensional station grid convertible to float64. Must be strictly
///     increasing and contain at least two entries.
/// a : ArrayLike
///     One-dimensional node profile convertible to float64 with
///     ``len(a) == len(s)``. Values must be finite and non-negative. Each interval must have a
///     positive finite speed denominator.
/// t0 : float, default=0.0
///     Initial time assigned to ``s[0]``.
///
/// Returns
/// -------
/// tuple[float, numpy.ndarray[numpy.float64]]
///     ``(t_final, t_s)`` where ``t_s`` has the same length as ``s``.
///
/// Raises
/// ------
/// ValueError
///     If an input cannot be converted to a one-dimensional float64 array.
/// CoppError
///     If the COPP core rejects the grid, profile, or time initial condition.
///
/// Notes
/// -----
/// This routine uses the same deterministic trapezoid-like TOPP2 timing model
/// as the Rust API. It does not resample the trajectory; it only computes the
/// arrival time at the provided station nodes.
#[pyfunction]
#[pyo3(signature = (s, a, t0 = 0.0))]
fn s_to_t_topp2<'py>(
    py: Python<'py>,
    s: &Bound<'py, PyAny>,
    a: &Bound<'py, PyAny>,
    t0: f64,
) -> PyResult<(f64, Bound<'py, PyArray1<f64>>)> {
    let s = array_like_to_vec_f64("s", s)?;
    let a = array_like_to_vec_f64("a", a)?;
    let (t_final, t_s) = crate::solver::topp2_ra::s_to_t_topp2(&s, &a, t0).map_err(to_py_err)?;
    Ok((t_final, t_s.into_pyarray(py)))
}

/// Sample the inverse TOPP2/COPP2 mapping ``s(t)`` on a uniform time grid.
///
/// This explicit entry point is preferred over the compatibility wrapper
/// [`t_to_s_topp2`] when the query grid is generated from a time step.
#[pyfunction]
#[pyo3(signature = (s, a, t_s, dt, *, t0 = 0.0, include_final = true))]
fn t_to_s_topp2_uniform<'py>(
    py: Python<'py>,
    s: &Bound<'py, PyAny>,
    a: &Bound<'py, PyAny>,
    t_s: &Bound<'py, PyAny>,
    dt: f64,
    t0: f64,
    include_final: bool,
) -> PyResult<Bound<'py, PyArray1<f64>>> {
    let s = array_like_to_vec_f64("s", s)?;
    let a = array_like_to_vec_f64("a", a)?;
    let t_s = array_like_to_vec_f64("t_s", t_s)?;
    let s_t = t_to_s_topp2_uniform_impl(&s, &a, &t_s, t0, dt, include_final)?;
    Ok(s_t.into_pyarray(py))
}

/// Sample the inverse TOPP2/COPP2 mapping ``s(t)`` at explicit query times.
///
/// This explicit entry point is preferred over the compatibility wrapper
/// [`t_to_s_topp2`] when the query grid is supplied by the caller.
#[pyfunction]
#[pyo3(signature = (s, a, t_s, t_sample))]
fn t_to_s_topp2_samples<'py>(
    py: Python<'py>,
    s: &Bound<'py, PyAny>,
    a: &Bound<'py, PyAny>,
    t_s: &Bound<'py, PyAny>,
    t_sample: &Bound<'py, PyAny>,
) -> PyResult<Bound<'py, PyArray1<f64>>> {
    let s = array_like_to_vec_f64("s", s)?;
    let a = array_like_to_vec_f64("a", a)?;
    let t_s = array_like_to_vec_f64("t_s", t_s)?;
    let t_sample = array_like_to_vec_f64("t_sample", t_sample)?;
    let s_t = t_to_s_topp2_samples_impl(&s, &a, &t_s, &t_sample)?;
    Ok(s_t.into_pyarray(py))
}

/// Compatibility wrapper for sampling inverse TOPP2/COPP2 mappings.
///
/// New code should prefer [`t_to_s_topp2_uniform`] or
/// [`t_to_s_topp2_samples`], which avoid mutually exclusive keyword modes.
///
/// Exactly one sampling mode must be selected:
///
/// - pass ``dt`` for a uniform time grid starting at ``t0``;
/// - pass ``t_sample`` for an explicit strictly increasing nonuniform time
///   grid.
///
/// Out-of-range query times are returned as ``NaN`` by the COPP core.
///
/// Parameters
/// ----------
/// s : ArrayLike
///     One-dimensional station grid convertible to float64. Values must be
///     strictly increasing.
/// a : ArrayLike
///     One-dimensional TOPP2/COPP2 node profile convertible to float64. Must
///     have the same length as ``s``.
/// t_s : ArrayLike
///     One-dimensional arrival-time profile convertible to float64, usually
///     returned by ``s_to_t_topp2``. Must have the same length as ``s`` and be
///     strictly increasing.
/// t0 : float, default=0.0
///     Start time for uniform-grid sampling. Ignored when ``t_sample`` is
///     provided.
/// dt : float | None, default=None
///     Positive uniform time step. Mutually exclusive with ``t_sample``.
/// include_final : bool, default=True
///     When using ``dt``, append ``s[-1]`` if the generated uniform grid does
///     not already include the final station.
/// t_sample : ArrayLike | None, default=None
///     Explicit nonuniform query times. Must be one-dimensional, finite,
///     non-empty, and strictly increasing. Mutually exclusive with ``dt``.
///
/// Returns
/// -------
/// numpy.ndarray[numpy.float64]
///     Sampled station values ``s(t)``.
///
/// Raises
/// ------
/// ValueError
///     If inputs cannot be converted to one-dimensional float64 arrays, if
///     neither sampling mode is selected, or if both ``dt`` and ``t_sample``
///     are provided.
/// CoppError
///     If the COPP core rejects the grid, profile, arrival times, or sampling
///     parameters.
///
/// Examples
/// --------
/// >>> import numpy as np
/// >>> import copp_py as copp
/// >>> s = np.array([0.0, 0.5, 1.0], dtype=np.float64)
/// >>> a = np.array([1.0, 1.0, 1.0], dtype=np.float64)
/// >>> _, t_s = copp.s_to_t_topp2(s, a)
/// >>> copp.t_to_s_topp2_uniform(s, a, t_s, 0.25)
/// array([0.  , 0.25, 0.5 , 0.75, 1.  ])
#[pyfunction]
#[pyo3(signature = (s, a, t_s, *, t0 = 0.0, dt = None, include_final = true, t_sample = None))]
fn t_to_s_topp2<'py>(
    py: Python<'py>,
    s: &Bound<'py, PyAny>,
    a: &Bound<'py, PyAny>,
    t_s: &Bound<'py, PyAny>,
    t0: f64,
    dt: Option<f64>,
    include_final: bool,
    t_sample: Option<&Bound<'py, PyAny>>,
) -> PyResult<Bound<'py, PyArray1<f64>>> {
    let s = array_like_to_vec_f64("s", s)?;
    let a = array_like_to_vec_f64("a", a)?;
    let t_s = array_like_to_vec_f64("t_s", t_s)?;

    let s_t = match (dt, t_sample) {
        (Some(dt), None) => t_to_s_topp2_uniform_impl(&s, &a, &t_s, t0, dt, include_final)?,
        (None, Some(t_sample)) => {
            let t_sample = array_like_to_vec_f64("t_sample", t_sample)?;
            t_to_s_topp2_samples_impl(&s, &a, &t_s, &t_sample)?
        }
        (Some(_), Some(_)) => {
            return Err(PyValueError::new_err(
                "pass either `dt` for a uniform time grid or `t_sample` for a nonuniform grid, not both",
            ));
        }
        (None, None) => {
            return Err(PyValueError::new_err(
                "`dt` is required unless `t_sample` is provided",
            ));
        }
    };

    Ok(s_t.into_pyarray(py))
}

/// Run TOPP2 inverse interpolation on a generated uniform time grid.
fn t_to_s_topp2_uniform_impl(
    s: &[f64],
    a: &[f64],
    t_s: &[f64],
    t0: f64,
    dt: f64,
    include_final: bool,
) -> PyResult<Vec<f64>> {
    crate::solver::topp2_ra::t_to_s_topp2(
        s,
        a,
        t_s,
        InterpolationMode::UniformTimeGrid(t0, dt, include_final),
    )
    .map_err(to_py_err)
}

/// Run TOPP2 inverse interpolation at explicit query times.
fn t_to_s_topp2_samples_impl(
    s: &[f64],
    a: &[f64],
    t_s: &[f64],
    t_sample: &[f64],
) -> PyResult<Vec<f64>> {
    crate::solver::topp2_ra::t_to_s_topp2(
        s,
        a,
        t_s,
        InterpolationMode::NonUniformTimeGrid(t_sample),
    )
    .map_err(to_py_err)
}

/// Integrate a TOPP3/COPP3 profile into cumulative arrival times ``t(s)``.
///
/// The input profile is node-based: both ``profile.a`` and ``profile.b`` must
/// have the same length as ``s``. The returned array ``t_s`` satisfies
/// ``t_s[i] == t(s[i])`` and starts from ``t_s[0] == t0``. The scalar
/// ``t_final`` is equal to ``t_s[-1]``.
///
/// Parameters
/// ----------
/// s : ArrayLike
///     One-dimensional station grid convertible to float64. Values must be
///     strictly increasing.
/// profile : Profile3rd
///     Third-order profile containing node samples of ``a = (ds/dt)^2`` and
///     ``b = dds/dt`` plus stationary boundary counts.
/// t0 : float, default=0.0
///     Initial time assigned to ``s[0]``.
///
/// Returns
/// -------
/// tuple[float, numpy.ndarray[numpy.float64]]
///     ``(t_final, t_s)`` where ``t_s`` has the same length as ``s``.
///
/// Raises
/// ------
/// ValueError
///     If ``s`` cannot be converted to a one-dimensional float64 array.
/// CoppError
///     If the COPP core rejects the grid, profile, stationary counts, or time
///     initial condition.
#[pyfunction]
#[pyo3(signature = (s, profile, t0 = 0.0))]
fn s_to_t_topp3<'py>(
    py: Python<'py>,
    s: &Bound<'py, PyAny>,
    profile: PyRef<'py, PyProfile3rd>,
    t0: f64,
) -> PyResult<(f64, Bound<'py, PyArray1<f64>>)> {
    let s = array_like_to_vec_f64("s", s)?;
    let (t_final, t_s) =
        crate::solver::topp3_lp::s_to_t_topp3(&s, profile.as_parts(), t0).map_err(to_py_err)?;
    Ok((t_final, t_s.into_pyarray(py)))
}

/// Sample the inverse TOPP3/COPP3 mapping ``s(t)`` on a uniform time grid.
///
/// This explicit entry point is preferred over the compatibility wrapper
/// [`t_to_s_topp3`] when the query grid is generated from a time step.
#[pyfunction]
#[pyo3(signature = (s, profile, t_s, dt, *, t0 = 0.0, include_final = true))]
fn t_to_s_topp3_uniform<'py>(
    py: Python<'py>,
    s: &Bound<'py, PyAny>,
    profile: PyRef<'py, PyProfile3rd>,
    t_s: &Bound<'py, PyAny>,
    dt: f64,
    t0: f64,
    include_final: bool,
) -> PyResult<Bound<'py, PyArray1<f64>>> {
    let s = array_like_to_vec_f64("s", s)?;
    let t_s = array_like_to_vec_f64("t_s", t_s)?;
    let s_t = t_to_s_topp3_uniform_impl(&s, profile.as_parts(), &t_s, t0, dt, include_final)?;
    Ok(s_t.into_pyarray(py))
}

/// Sample the inverse TOPP3/COPP3 mapping ``s(t)`` at explicit query times.
///
/// This explicit entry point is preferred over the compatibility wrapper
/// [`t_to_s_topp3`] when the query grid is supplied by the caller.
#[pyfunction]
#[pyo3(signature = (s, profile, t_s, t_sample))]
fn t_to_s_topp3_samples<'py>(
    py: Python<'py>,
    s: &Bound<'py, PyAny>,
    profile: PyRef<'py, PyProfile3rd>,
    t_s: &Bound<'py, PyAny>,
    t_sample: &Bound<'py, PyAny>,
) -> PyResult<Bound<'py, PyArray1<f64>>> {
    let s = array_like_to_vec_f64("s", s)?;
    let t_s = array_like_to_vec_f64("t_s", t_s)?;
    let t_sample = array_like_to_vec_f64("t_sample", t_sample)?;
    let s_t = t_to_s_topp3_samples_impl(&s, profile.as_parts(), &t_s, &t_sample)?;
    Ok(s_t.into_pyarray(py))
}

/// Compatibility wrapper for sampling inverse TOPP3/COPP3 mappings.
///
/// New code should prefer [`t_to_s_topp3_uniform`] or
/// [`t_to_s_topp3_samples`], which avoid mutually exclusive keyword modes.
///
/// Exactly one sampling mode must be selected:
///
/// - pass ``dt`` for a uniform time grid starting at ``t0``;
/// - pass ``t_sample`` for an explicit strictly increasing nonuniform time
///   grid.
///
/// Out-of-range query times are returned as ``NaN`` by the COPP core.
///
/// Parameters
/// ----------
/// s : ArrayLike
///     One-dimensional station grid convertible to float64. Values must be
///     strictly increasing.
/// profile : Profile3rd
///     Third-order profile used to integrate and invert the time mapping.
/// t_s : ArrayLike
///     One-dimensional arrival-time profile convertible to float64, usually
///     returned by ``s_to_t_topp3``. Must have the same length as ``s`` and be
///     strictly increasing.
/// t0 : float, default=0.0
///     Start time for uniform-grid sampling. Ignored when ``t_sample`` is
///     provided.
/// dt : float | None, default=None
///     Positive uniform time step. Mutually exclusive with ``t_sample``.
/// include_final : bool, default=True
///     When using ``dt``, append ``s[-1]`` if the generated uniform grid does
///     not already include the final station.
/// t_sample : ArrayLike | None, default=None
///     Explicit nonuniform query times. Must be one-dimensional, finite,
///     non-empty, and strictly increasing. Mutually exclusive with ``dt``.
///
/// Returns
/// -------
/// numpy.ndarray[numpy.float64]
///     Sampled station values ``s(t)``.
///
/// Raises
/// ------
/// ValueError
///     If inputs cannot be converted to one-dimensional float64 arrays, if
///     neither sampling mode is selected, or if both ``dt`` and ``t_sample``
///     are provided.
/// CoppError
///     If the COPP core rejects the grid, profile, arrival times, or sampling
///     parameters.
#[pyfunction]
#[pyo3(signature = (s, profile, t_s, *, t0 = 0.0, dt = None, include_final = true, t_sample = None))]
fn t_to_s_topp3<'py>(
    py: Python<'py>,
    s: &Bound<'py, PyAny>,
    profile: PyRef<'py, PyProfile3rd>,
    t_s: &Bound<'py, PyAny>,
    t0: f64,
    dt: Option<f64>,
    include_final: bool,
    t_sample: Option<&Bound<'py, PyAny>>,
) -> PyResult<Bound<'py, PyArray1<f64>>> {
    let s = array_like_to_vec_f64("s", s)?;
    let t_s = array_like_to_vec_f64("t_s", t_s)?;

    let s_t = match (dt, t_sample) {
        (Some(dt), None) => {
            t_to_s_topp3_uniform_impl(&s, profile.as_parts(), &t_s, t0, dt, include_final)?
        }
        (None, Some(t_sample)) => {
            let t_sample = array_like_to_vec_f64("t_sample", t_sample)?;
            t_to_s_topp3_samples_impl(&s, profile.as_parts(), &t_s, &t_sample)?
        }
        (Some(_), Some(_)) => {
            return Err(PyValueError::new_err(
                "pass either `dt` for a uniform time grid or `t_sample` for a nonuniform grid, not both",
            ));
        }
        (None, None) => {
            return Err(PyValueError::new_err(
                "`dt` is required unless `t_sample` is provided",
            ));
        }
    };

    Ok(s_t.into_pyarray(py))
}

/// Run TOPP3 inverse interpolation on a generated uniform time grid.
fn t_to_s_topp3_uniform_impl(
    s: &[f64],
    profile: Topp3ProfileRef<'_>,
    t_s: &[f64],
    t0: f64,
    dt: f64,
    include_final: bool,
) -> PyResult<Vec<f64>> {
    crate::solver::topp3_lp::t_to_s_topp3(
        s,
        profile,
        t_s,
        InterpolationMode::UniformTimeGrid(t0, dt, include_final),
    )
    .map_err(to_py_err)
}

/// Run TOPP3 inverse interpolation at explicit query times.
fn t_to_s_topp3_samples_impl(
    s: &[f64],
    profile: Topp3ProfileRef<'_>,
    t_s: &[f64],
    t_sample: &[f64],
) -> PyResult<Vec<f64>> {
    crate::solver::topp3_lp::t_to_s_topp3(
        s,
        profile,
        t_s,
        InterpolationMode::NonUniformTimeGrid(t_sample),
    )
    .map_err(to_py_err)
}
