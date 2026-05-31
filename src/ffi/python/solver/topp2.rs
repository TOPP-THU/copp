//! Python wrappers for TOPP2 problem descriptors and reachability solvers.

use super::common::{PyVerbosity, VerbosityArg, parse_verbosity};
use crate::ffi::python::error::to_py_err;
use crate::ffi::python::robot::{PyConstraints, SharedRobot, with_shared_constraints_result};
use crate::solver::reach_set2::{
    ReachSet2 as RustReachSet2, ReachSet2Options as RustReachSet2Options, ReachSet2OptionsBuilder,
    reach_set2_backward as rust_reach_set2_backward,
    reach_set2_bidirectional as rust_reach_set2_bidirectional,
};
use crate::solver::topp2_ra::{Topp2ProblemBuilder, topp2_ra as rust_topp2_ra};
use numpy::PyArray1;
use pyo3::prelude::*;

/// Register TOPP2 problem, option, result, and solver APIs.
pub(super) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyReachSet2Options>()?;
    m.add_class::<PyReachSet2>()?;
    m.add_class::<PyTopp2Problem>()?;
    m.add_function(wrap_pyfunction!(reach_set2_backward, m)?)?;
    m.add_function(wrap_pyfunction!(reach_set2_bidirectional, m)?)?;
    m.add_function(wrap_pyfunction!(topp2_ra, m)?)?;
    Ok(())
}

/// Python-owned options for TOPP2 reachable-set based solvers.
///
/// Rust defaults from [`ReachSet2OptionsBuilder::new`]:
/// `lp_feas_tol=1e-8`, `a_cmp_abs_tol=1e-8`, `a_cmp_rel_tol=1e-8`, and
/// `verbosity=Verbosity.Silent`.
#[pyclass(
    name = "ReachSet2Options",
    module = "copp_py._native",
    skip_from_py_object
)]
pub(crate) struct PyReachSet2Options {
    /// Feasibility tolerance used by LP subproblems.
    lp_feas_tol: f64,
    /// Absolute tolerance for comparing interval bounds.
    a_cmp_abs_tol: f64,
    /// Relative tolerance for comparing interval bounds.
    a_cmp_rel_tol: f64,
    /// Solver diagnostic verbosity.
    verbosity: PyVerbosity,
}

#[pymethods]
impl PyReachSet2Options {
    /// Construct TOPP2 reachable-set solver options.
    ///
    /// Defaults mirror Rust: `lp_feas_tol=1e-8`,
    /// `a_cmp_abs_tol=1e-8`, `a_cmp_rel_tol=1e-8`, and
    /// `verbosity=Verbosity.Silent`.
    #[new]
    #[pyo3(
        signature = (*, lp_feas_tol = 1.0e-8, a_cmp_abs_tol = 1.0e-8, a_cmp_rel_tol = 1.0e-8, verbosity = VerbosityArg(PyVerbosity::Silent)),
        text_signature = "(*, lp_feas_tol=1e-08, a_cmp_abs_tol=1e-08, a_cmp_rel_tol=1e-08, verbosity='silent')"
    )]
    fn new(
        lp_feas_tol: f64,
        a_cmp_abs_tol: f64,
        a_cmp_rel_tol: f64,
        verbosity: VerbosityArg,
    ) -> PyResult<Self> {
        let options = Self {
            lp_feas_tol,
            a_cmp_abs_tol,
            a_cmp_rel_tol,
            verbosity: verbosity.0,
        };
        options.to_rust()?;
        Ok(options)
    }

    /// Return the LP feasibility tolerance.
    #[getter]
    fn lp_feas_tol(&self) -> f64 {
        self.lp_feas_tol
    }

    /// Set the LP feasibility tolerance.
    #[setter]
    fn set_lp_feas_tol(&mut self, value: f64) -> PyResult<()> {
        let mut candidate = self.clone();
        candidate.lp_feas_tol = value;
        candidate.to_rust()?;
        self.lp_feas_tol = value;
        Ok(())
    }

    /// Return the absolute comparison tolerance for `a` bounds.
    #[getter]
    fn a_cmp_abs_tol(&self) -> f64 {
        self.a_cmp_abs_tol
    }

    /// Set the absolute comparison tolerance for `a` bounds.
    #[setter]
    fn set_a_cmp_abs_tol(&mut self, value: f64) -> PyResult<()> {
        let mut candidate = self.clone();
        candidate.a_cmp_abs_tol = value;
        candidate.to_rust()?;
        self.a_cmp_abs_tol = value;
        Ok(())
    }

    /// Return the relative comparison tolerance for `a` bounds.
    #[getter]
    fn a_cmp_rel_tol(&self) -> f64 {
        self.a_cmp_rel_tol
    }

    /// Set the relative comparison tolerance for `a` bounds.
    #[setter]
    fn set_a_cmp_rel_tol(&mut self, value: f64) -> PyResult<()> {
        let mut candidate = self.clone();
        candidate.a_cmp_rel_tol = value;
        candidate.to_rust()?;
        self.a_cmp_rel_tol = value;
        Ok(())
    }

    /// Return the diagnostic verbosity level.
    #[getter]
    fn verbosity(&self) -> PyVerbosity {
        self.verbosity
    }

    /// Set diagnostic verbosity from an enum value or string.
    #[setter]
    fn set_verbosity(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.verbosity = parse_verbosity(value)?;
        Ok(())
    }

    /// Return a compact representation for interactive Python sessions.
    fn __repr__(&self) -> String {
        format!(
            "ReachSet2Options(lp_feas_tol={}, a_cmp_abs_tol={}, a_cmp_rel_tol={}, verbosity={:?})",
            self.lp_feas_tol, self.a_cmp_abs_tol, self.a_cmp_rel_tol, self.verbosity
        )
    }
}

impl Clone for PyReachSet2Options {
    /// Clone the plain option values.
    fn clone(&self) -> Self {
        Self {
            lp_feas_tol: self.lp_feas_tol,
            a_cmp_abs_tol: self.a_cmp_abs_tol,
            a_cmp_rel_tol: self.a_cmp_rel_tol,
            verbosity: self.verbosity,
        }
    }
}

impl PyReachSet2Options {
    /// Convert Python options into validated Rust options.
    fn to_rust(&self) -> PyResult<RustReachSet2Options> {
        ReachSet2OptionsBuilder::new()
            .lp_feas_tol(self.lp_feas_tol)
            .a_cmp_abs_tol(self.a_cmp_abs_tol)
            .a_cmp_rel_tol(self.a_cmp_rel_tol)
            .verbosity(self.verbosity.to_rust())
            .build()
            .map_err(to_py_err)
    }
}

/// Python-owned TOPP2/COPP2 reachable intervals.
#[pyclass(name = "ReachSet2", module = "copp_py._native")]
pub(crate) struct PyReachSet2 {
    /// Upper reachable bound `a_max[k]` at each station.
    a_max: Vec<f64>,
    /// Lower reachable bound `a_min[k]` at each station.
    a_min: Vec<f64>,
}

#[pymethods]
impl PyReachSet2 {
    /// Return the upper reachable bound for each station.
    ///
    /// A fresh NumPy array is returned on each access. Mutating it does not
    /// modify this result object or any Rust-side solver state.
    #[getter]
    fn a_max<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        PyArray1::from_vec(py, self.a_max.clone())
    }

    /// Return the lower reachable bound for each station.
    ///
    /// A fresh NumPy array is returned on each access. Mutating it does not
    /// modify this result object or any Rust-side solver state.
    #[getter]
    fn a_min<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        PyArray1::from_vec(py, self.a_min.clone())
    }

    /// Return the number of stations covered by this reachable set.
    #[getter]
    fn len(&self) -> usize {
        self.a_max.len()
    }

    /// Return whether this reachable set contains at least one station.
    fn __bool__(&self) -> bool {
        !self.a_max.is_empty()
    }

    /// Return `len(self)`.
    fn __len__(&self) -> usize {
        self.a_max.len()
    }

    /// Return a compact representation for interactive Python sessions.
    fn __repr__(&self) -> String {
        format!("ReachSet2(len={})", self.a_max.len())
    }
}

impl From<RustReachSet2> for PyReachSet2 {
    /// Convert a Rust reachable set into a Python-owned result object.
    fn from(value: RustReachSet2) -> Self {
        Self {
            a_max: value.a_max,
            a_min: value.a_min,
        }
    }
}

/// Python descriptor for a TOPP2 problem.
#[pyclass(name = "Topp2Problem", module = "copp_py._native")]
pub(crate) struct PyTopp2Problem {
    /// Constraint proxy kept alive for borrowed Rust problem construction.
    constraints: Py<PyConstraints>,
    /// Shared constraint storage used by detached solver calls.
    shared: SharedRobot,
    /// Closed station-index interval `(idx_s_start, idx_s_final)`.
    idx_s_interval: (usize, usize),
    /// Endpoint state tuple `(a_start, a_final)`.
    a_boundary: (f64, f64),
}

#[pymethods]
impl PyTopp2Problem {
    /// Construct a TOPP2 problem descriptor from a constraints proxy.
    ///
    /// The descriptor stores a reference to the Python constraints proxy and
    /// rebuilds a validated Rust [`crate::solver::topp2_ra::Topp2Problem`] for
    /// each solver call. It does not copy constraint data.
    #[new]
    #[pyo3(
        signature = (constraints, idx_s_interval, a_boundary = (0.0, 0.0)),
        text_signature = "(constraints, idx_s_interval, a_boundary=(0.0, 0.0))"
    )]
    fn new(
        py: Python<'_>,
        constraints: Py<PyConstraints>,
        idx_s_interval: (usize, usize),
        a_boundary: (f64, f64),
    ) -> PyResult<Self> {
        let shared = constraints.bind(py).borrow().shared_robot();
        let problem = Self {
            constraints,
            shared,
            idx_s_interval,
            a_boundary,
        };
        problem.validate(py)?;
        Ok(problem)
    }

    /// Return the constraint proxy referenced by this problem descriptor.
    #[getter]
    fn constraints(&self, py: Python<'_>) -> Py<PyConstraints> {
        self.constraints.clone_ref(py)
    }

    /// Return the closed station-index interval.
    #[getter]
    fn idx_s_interval(&self) -> (usize, usize) {
        self.idx_s_interval
    }

    /// Return endpoint values `(a_start, a_final)`.
    #[getter]
    fn a_boundary(&self) -> (f64, f64) {
        self.a_boundary
    }

    /// Return the number of station samples in the closed interval.
    #[getter]
    fn s_len(&self) -> usize {
        self.idx_s_interval.1.saturating_sub(self.idx_s_interval.0) + 1
    }

    /// Validate the descriptor against the currently referenced constraints.
    fn validate(&self, py: Python<'_>) -> PyResult<()> {
        self.with_rust_problem(py, |_| Ok(()))
    }

    /// Return a compact representation for interactive Python sessions.
    fn __repr__(&self) -> String {
        format!(
            "Topp2Problem(idx_s_interval=({}, {}), a_boundary=({}, {}))",
            self.idx_s_interval.0, self.idx_s_interval.1, self.a_boundary.0, self.a_boundary.1
        )
    }
}

impl PyTopp2Problem {
    /// Build a validated Rust TOPP2 problem and pass it to `f`.
    fn with_rust_problem<R>(
        &self,
        py: Python<'_>,
        f: impl FnOnce(&crate::solver::topp2_ra::Topp2Problem<'_>) -> Result<R, crate::diag::CoppError>,
    ) -> PyResult<R> {
        let _ = py;
        Self::with_shared_problem(&self.shared, self.idx_s_interval, self.a_boundary, f)
    }

    /// Build a TOPP2 problem from shared constraints and pass it to `f`.
    fn with_shared_problem<R>(
        shared: &SharedRobot,
        idx_s_interval: (usize, usize),
        a_boundary: (f64, f64),
        f: impl FnOnce(&crate::solver::topp2_ra::Topp2Problem<'_>) -> Result<R, crate::diag::CoppError>,
    ) -> PyResult<R> {
        with_shared_constraints_result(shared, |constraints| {
            let problem =
                Topp2ProblemBuilder::with_constraint(constraints, idx_s_interval, a_boundary)
                    .build()
                    .map_err(to_py_err)?;
            f(&problem).map_err(to_py_err)
        })
    }

    /// Compute a backward-only reachable set with validated Rust options.
    fn compute_reach_set2_backward(
        &self,
        py: Python<'_>,
        options: &RustReachSet2Options,
    ) -> PyResult<PyReachSet2> {
        let shared = self.shared.clone();
        let idx_s_interval = self.idx_s_interval;
        let a_boundary = self.a_boundary;
        py.detach(move || {
            Self::with_shared_problem(&shared, idx_s_interval, a_boundary, |problem| {
                rust_reach_set2_backward(problem, options)
            })
            .map(PyReachSet2::from)
        })
    }

    /// Compute a bidirectional reachable set with validated Rust options.
    fn compute_reach_set2_bidirectional(
        &self,
        py: Python<'_>,
        options: &RustReachSet2Options,
    ) -> PyResult<PyReachSet2> {
        let shared = self.shared.clone();
        let idx_s_interval = self.idx_s_interval;
        let a_boundary = self.a_boundary;
        py.detach(move || {
            Self::with_shared_problem(&shared, idx_s_interval, a_boundary, |problem| {
                rust_reach_set2_bidirectional(problem, options)
            })
            .map(PyReachSet2::from)
        })
    }

    /// Solve this TOPP2 problem with validated Rust options.
    fn solve_topp2_ra(&self, py: Python<'_>, options: &RustReachSet2Options) -> PyResult<Vec<f64>> {
        let shared = self.shared.clone();
        let idx_s_interval = self.idx_s_interval;
        let a_boundary = self.a_boundary;
        py.detach(move || {
            Self::with_shared_problem(&shared, idx_s_interval, a_boundary, |problem| {
                rust_topp2_ra(problem, options)
            })
        })
    }
}

/// Compute backward-only TOPP2 reachable intervals.
///
/// `options=None` uses Rust [`ReachSet2OptionsBuilder::new`] defaults.
#[pyfunction]
#[pyo3(signature = (problem, options = None), text_signature = "(problem, options=None)")]
fn reach_set2_backward<'py>(
    py: Python<'py>,
    problem: PyRef<'py, PyTopp2Problem>,
    options: Option<PyRef<'py, PyReachSet2Options>>,
) -> PyResult<PyReachSet2> {
    let options = reach_set2_options_or_default(options)?;
    problem.compute_reach_set2_backward(py, &options)
}

/// Compute bidirectional TOPP2 reachable intervals.
///
/// `options=None` uses Rust [`ReachSet2OptionsBuilder::new`] defaults.
#[pyfunction]
#[pyo3(signature = (problem, options = None), text_signature = "(problem, options=None)")]
fn reach_set2_bidirectional<'py>(
    py: Python<'py>,
    problem: PyRef<'py, PyTopp2Problem>,
    options: Option<PyRef<'py, PyReachSet2Options>>,
) -> PyResult<PyReachSet2> {
    let options = reach_set2_options_or_default(options)?;
    problem.compute_reach_set2_bidirectional(py, &options)
}

/// Solve TOPP2 with reachability analysis.
#[pyfunction]
#[pyo3(signature = (problem, options = None), text_signature = "(problem, options=None)")]
fn topp2_ra<'py>(
    py: Python<'py>,
    problem: PyRef<'py, PyTopp2Problem>,
    options: Option<PyRef<'py, PyReachSet2Options>>,
) -> PyResult<Bound<'py, PyArray1<f64>>> {
    let options = reach_set2_options_or_default(options)?;
    let a = problem.solve_topp2_ra(py, &options)?;
    Ok(PyArray1::from_vec(py, a))
}

/// Convert an optional Python options object into validated Rust options.
fn reach_set2_options_or_default(
    options: Option<PyRef<'_, PyReachSet2Options>>,
) -> PyResult<RustReachSet2Options> {
    match options {
        Some(options) => options.to_rust(),
        None => ReachSet2OptionsBuilder::new().build().map_err(to_py_err),
    }
}
