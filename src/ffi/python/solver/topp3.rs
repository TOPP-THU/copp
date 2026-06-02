//! Python wrappers for TOPP3 problem descriptors and solvers.

use super::clarabel::{PyClarabelOptions, PyCopp3ClarabelResult, clarabel_options_or_default};
use crate::ffi::python::array::array_like_to_vec_f64;
use crate::ffi::python::error::to_py_err;
use crate::ffi::python::interpolation::PyProfile3rd;
use crate::ffi::python::robot::{PyConstraints, SharedRobot, with_shared_constraints_mut_result};
use crate::solver::copp2_socp::ClarabelOptions as RustClarabelOptions;
use crate::solver::topp3_lp::{
    Topp3ProblemBuilder as Topp3ProblemBuilder3, topp3_lp as rust_topp3_lp,
    topp3_lp_expert_with_info as rust_topp3_lp_expert,
};
use crate::solver::topp3_socp::{
    topp3_socp as rust_topp3_socp, topp3_socp_expert_with_info as rust_topp3_socp_expert,
};
use numpy::PyArray1;
use pyo3::Borrowed;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

/// Register TOPP3 problem and solver APIs.
pub(super) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyTopp3Problem>()?;
    m.add_function(wrap_pyfunction!(topp3_lp, m)?)?;
    m.add_function(wrap_pyfunction!(topp3_lp_expert, m)?)?;
    m.add_function(wrap_pyfunction!(topp3_socp, m)?)?;
    m.add_function(wrap_pyfunction!(topp3_socp_expert, m)?)?;
    Ok(())
}

/// Internal parser for `usize | tuple[usize, usize]` stationary-bound settings.
#[derive(Clone, Copy)]
pub(super) struct NumStationaryMaxArg(
    /// Parsed `(start, end)` stationary upper bounds.
    pub(super) (usize, usize),
);

impl FromPyObject<'_, '_> for NumStationaryMaxArg {
    /// [`PyErr`] returned when parsing fails.
    type Error = PyErr;

    /// Parse a symmetric integer shorthand or an explicit pair.
    fn extract(obj: Borrowed<'_, '_, PyAny>) -> Result<Self, Self::Error> {
        if let Ok(value) = obj.extract::<usize>() {
            return Ok(Self((value, value)));
        }

        if let Ok(pair) = obj.extract::<(usize, usize)>() {
            return Ok(Self(pair));
        }

        Err(PyValueError::new_err(
            "`num_stationary_max` must be an integer or a pair of integers",
        ))
    }
}

/// Python descriptor for a TOPP3 problem.
///
/// Construction copies the linearization reference profile and immediately
/// calls the Rust third-order builder, which linearizes cached jerk
/// constraints inside the referenced constraint buffer.
#[pyclass(name = "Topp3Problem", module = "copp_py._native")]
pub(crate) struct PyTopp3Problem {
    /// Constraint proxy kept alive for borrowed Rust problem construction.
    constraints: Py<PyConstraints>,
    /// Shared constraint storage used by detached solver calls.
    shared: SharedRobot,
    /// Reference profile used to linearize third-order constraints.
    a_linearization: Vec<f64>,
    /// Start station index of the optimization interval.
    idx_s_start: usize,
    /// Boundary values `(a_start, a_final)`.
    a_boundary: (f64, f64),
    /// Boundary values `(b_start, b_final)`.
    b_boundary: (f64, f64),
    /// User-input upper bound of stationary intervals at `(start, end)`.
    num_stationary_max: (usize, usize),
    /// Denominator floor for stable third-order linearization.
    a_linearization_floor: f64,
}

#[pymethods]
impl PyTopp3Problem {
    /// Construct and immediately linearize a TOPP3 problem descriptor.
    ///
    /// The descriptor stores a reference to a Python constraints proxy and an
    /// owned copy of `a_linearization`. The constructor calls
    /// `build_with_linearization()` once, so it may update cached third-order
    /// linearization rows inside `constraints`.
    #[new]
    #[pyo3(
        signature = (constraints, a_linearization, *, idx_s_start = 0, a_boundary = (0.0, 0.0), b_boundary = (0.0, 0.0), num_stationary_max = NumStationaryMaxArg((1, 1)), a_linearization_floor = 1.0e-10),
        text_signature = "(constraints, a_linearization, *, idx_s_start=0, a_boundary=(0.0, 0.0), b_boundary=(0.0, 0.0), num_stationary_max=1, a_linearization_floor=1e-10)"
    )]
    fn new(
        py: Python<'_>,
        constraints: Py<PyConstraints>,
        a_linearization: &Bound<'_, PyAny>,
        idx_s_start: usize,
        a_boundary: (f64, f64),
        b_boundary: (f64, f64),
        num_stationary_max: NumStationaryMaxArg,
        a_linearization_floor: f64,
    ) -> PyResult<Self> {
        let a_linearization = array_like_to_vec_f64("a_linearization", a_linearization)?;
        let shared = constraints.bind(py).borrow().shared_robot();
        let problem = Self {
            constraints,
            shared,
            a_linearization,
            idx_s_start,
            a_boundary,
            b_boundary,
            num_stationary_max: num_stationary_max.0,
            a_linearization_floor,
        };
        problem.validate(py)?;
        Ok(problem)
    }

    /// Return the constraint proxy referenced by this problem descriptor.
    #[getter]
    fn constraints(&self, py: Python<'_>) -> Py<PyConstraints> {
        self.constraints.clone_ref(py)
    }

    /// Return a copy of the linearization reference profile.
    #[getter]
    fn a_linearization<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        PyArray1::from_vec(py, self.a_linearization.clone())
    }

    /// Return the start station index of the optimization interval.
    #[getter]
    fn idx_s_start(&self) -> usize {
        self.idx_s_start
    }

    /// Return the final station index implied by `a_linearization`.
    #[getter]
    fn idx_s_final(&self) -> usize {
        self.idx_s_start
            .saturating_add(self.a_linearization.len().saturating_sub(1))
    }

    /// Return boundary values `(a_start, a_final)`.
    #[getter]
    fn a_boundary(&self) -> (f64, f64) {
        self.a_boundary
    }

    /// Return boundary values `(b_start, b_final)`.
    #[getter]
    fn b_boundary(&self) -> (f64, f64) {
        self.b_boundary
    }

    /// Return stationary-interval upper bounds `(start, end)`.
    #[getter]
    fn num_stationary_max(&self) -> (usize, usize) {
        self.num_stationary_max
    }

    /// Return the third-order linearization denominator floor.
    #[getter]
    fn a_linearization_floor(&self) -> f64 {
        self.a_linearization_floor
    }

    /// Return the number of station samples in this TOPP3 interval.
    #[getter]
    fn s_len(&self) -> usize {
        self.a_linearization.len()
    }

    /// Rebuild the Rust TOPP3 problem and refresh cached jerk linearization.
    fn validate(&self, py: Python<'_>) -> PyResult<()> {
        self.with_rust_problem(py, |_| Ok(()))
    }

    /// Return a compact representation for interactive Python sessions.
    fn __repr__(&self) -> String {
        format!(
            "Topp3Problem(idx_s_start={}, s_len={}, a_boundary=({}, {}), b_boundary=({}, {}), num_stationary_max=({}, {}), a_linearization_floor={})",
            self.idx_s_start,
            self.a_linearization.len(),
            self.a_boundary.0,
            self.a_boundary.1,
            self.b_boundary.0,
            self.b_boundary.1,
            self.num_stationary_max.0,
            self.num_stationary_max.1,
            self.a_linearization_floor
        )
    }
}

impl PyTopp3Problem {
    /// Build a linearized Rust TOPP3 problem and pass it to `f`.
    pub(crate) fn with_rust_problem<R>(
        &self,
        py: Python<'_>,
        f: impl FnOnce(&crate::solver::topp3_lp::Topp3Problem<'_>) -> Result<R, crate::diag::CoppError>,
    ) -> PyResult<R> {
        let _ = py;
        Self::with_shared_problem(
            &self.shared,
            self.idx_s_start,
            &self.a_linearization,
            self.a_boundary,
            self.b_boundary,
            self.num_stationary_max,
            self.a_linearization_floor,
            f,
        )
    }

    /// Build a linearized TOPP3 problem from shared constraints and call `f`.
    #[allow(clippy::too_many_arguments)]
    fn with_shared_problem<R>(
        shared: &SharedRobot,
        idx_s_start: usize,
        a_linearization: &[f64],
        a_boundary: (f64, f64),
        b_boundary: (f64, f64),
        num_stationary_max: (usize, usize),
        a_linearization_floor: f64,
        f: impl FnOnce(&crate::solver::topp3_lp::Topp3Problem<'_>) -> Result<R, crate::diag::CoppError>,
    ) -> PyResult<R> {
        with_shared_constraints_mut_result(shared, |constraints| {
            let problem = Topp3ProblemBuilder3::with_constraint(
                constraints,
                idx_s_start,
                a_linearization,
                a_boundary,
                b_boundary,
            )
            .with_num_stationary_max_pair(num_stationary_max)
            .with_a_linearization_floor(a_linearization_floor)
            .build_with_linearization()
            .map_err(to_py_err)?;
            f(&problem).map_err(to_py_err)
        })
    }

    /// Solve this TOPP3 problem with the LP Clarabel backend.
    fn solve_topp3_lp(
        &self,
        py: Python<'_>,
        options: &RustClarabelOptions,
    ) -> PyResult<PyProfile3rd> {
        let shared = self.shared.clone();
        let idx_s_start = self.idx_s_start;
        let a_linearization = self.a_linearization.clone();
        let a_boundary = self.a_boundary;
        let b_boundary = self.b_boundary;
        let num_stationary_max = self.num_stationary_max;
        let a_linearization_floor = self.a_linearization_floor;
        py.detach(move || {
            Self::with_shared_problem(
                &shared,
                idx_s_start,
                &a_linearization,
                a_boundary,
                b_boundary,
                num_stationary_max,
                a_linearization_floor,
                |problem| rust_topp3_lp(problem, options),
            )
            .map(PyProfile3rd::from_rust)
        })
    }

    /// Solve this TOPP3 problem with expert LP Clarabel diagnostics.
    fn solve_topp3_lp_expert(
        &self,
        py: Python<'_>,
        options: &RustClarabelOptions,
    ) -> PyResult<PyCopp3ClarabelResult> {
        let shared = self.shared.clone();
        let idx_s_start = self.idx_s_start;
        let a_linearization = self.a_linearization.clone();
        let a_boundary = self.a_boundary;
        let b_boundary = self.b_boundary;
        let num_stationary_max = self.num_stationary_max;
        let a_linearization_floor = self.a_linearization_floor;
        py.detach(move || {
            Self::with_shared_problem(
                &shared,
                idx_s_start,
                &a_linearization,
                a_boundary,
                b_boundary,
                num_stationary_max,
                a_linearization_floor,
                |problem| {
                    let expert = rust_topp3_lp_expert(problem, options)?;
                    Ok(PyCopp3ClarabelResult::from_solution(
                        expert.result,
                        expert.solution,
                        expert.linsolver,
                        None,
                    ))
                },
            )
        })
    }

    /// Solve this TOPP3 problem with the SOCP Clarabel backend.
    fn solve_topp3_socp(
        &self,
        py: Python<'_>,
        options: &RustClarabelOptions,
    ) -> PyResult<PyProfile3rd> {
        let shared = self.shared.clone();
        let idx_s_start = self.idx_s_start;
        let a_linearization = self.a_linearization.clone();
        let a_boundary = self.a_boundary;
        let b_boundary = self.b_boundary;
        let num_stationary_max = self.num_stationary_max;
        let a_linearization_floor = self.a_linearization_floor;
        py.detach(move || {
            Self::with_shared_problem(
                &shared,
                idx_s_start,
                &a_linearization,
                a_boundary,
                b_boundary,
                num_stationary_max,
                a_linearization_floor,
                |problem| rust_topp3_socp(problem, options),
            )
            .map(PyProfile3rd::from_rust)
        })
    }

    /// Solve this TOPP3 problem with expert SOCP Clarabel diagnostics.
    fn solve_topp3_socp_expert(
        &self,
        py: Python<'_>,
        options: &RustClarabelOptions,
    ) -> PyResult<PyCopp3ClarabelResult> {
        let shared = self.shared.clone();
        let idx_s_start = self.idx_s_start;
        let a_linearization = self.a_linearization.clone();
        let a_boundary = self.a_boundary;
        let b_boundary = self.b_boundary;
        let num_stationary_max = self.num_stationary_max;
        let a_linearization_floor = self.a_linearization_floor;
        py.detach(move || {
            Self::with_shared_problem(
                &shared,
                idx_s_start,
                &a_linearization,
                a_boundary,
                b_boundary,
                num_stationary_max,
                a_linearization_floor,
                |problem| {
                    let expert = rust_topp3_socp_expert(problem, options)?;
                    Ok(PyCopp3ClarabelResult::from_solution(
                        expert.result,
                        expert.solution,
                        expert.linsolver,
                        None,
                    ))
                },
            )
        })
    }
}

/// Solve TOPP3 with the Clarabel LP backend.
#[pyfunction]
#[pyo3(signature = (problem, options = None), text_signature = "(problem, options=None)")]
fn topp3_lp<'py>(
    py: Python<'py>,
    problem: PyRef<'py, PyTopp3Problem>,
    options: Option<PyRef<'py, PyClarabelOptions>>,
) -> PyResult<PyProfile3rd> {
    let options = clarabel_options_or_default(options)?;
    problem.solve_topp3_lp(py, &options)
}

/// Solve TOPP3 with the Clarabel LP backend and return expert diagnostics.
#[pyfunction]
#[pyo3(signature = (problem, options = None), text_signature = "(problem, options=None)")]
fn topp3_lp_expert<'py>(
    py: Python<'py>,
    problem: PyRef<'py, PyTopp3Problem>,
    options: Option<PyRef<'py, PyClarabelOptions>>,
) -> PyResult<PyCopp3ClarabelResult> {
    let options = clarabel_options_or_default(options)?;
    problem.solve_topp3_lp_expert(py, &options)
}

/// Solve TOPP3 with the Clarabel SOCP backend.
#[pyfunction]
#[pyo3(signature = (problem, options = None), text_signature = "(problem, options=None)")]
fn topp3_socp<'py>(
    py: Python<'py>,
    problem: PyRef<'py, PyTopp3Problem>,
    options: Option<PyRef<'py, PyClarabelOptions>>,
) -> PyResult<PyProfile3rd> {
    let options = clarabel_options_or_default(options)?;
    problem.solve_topp3_socp(py, &options)
}

/// Solve TOPP3 with the Clarabel SOCP backend and return expert diagnostics.
#[pyfunction]
#[pyo3(signature = (problem, options = None), text_signature = "(problem, options=None)")]
fn topp3_socp_expert<'py>(
    py: Python<'py>,
    problem: PyRef<'py, PyTopp3Problem>,
    options: Option<PyRef<'py, PyClarabelOptions>>,
) -> PyResult<PyCopp3ClarabelResult> {
    let options = clarabel_options_or_default(options)?;
    problem.solve_topp3_socp_expert(py, &options)
}
