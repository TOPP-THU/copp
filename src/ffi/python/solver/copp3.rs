//! Python wrappers for COPP3 problem descriptors and solvers.

use super::clarabel::{PyClarabelOptions, PyCopp3ClarabelResult, clarabel_options_or_default};
use super::topp3::NumStationaryMaxArg;
use crate::copp::copp3::opt3::copp3_socp::objective_value_copp3_opt;
use crate::copp::objectives::validate_copp3_objectives;
use crate::diag::CoppError;
use crate::ffi::python::array::array_like_to_vec_f64;
use crate::ffi::python::interpolation::PyProfile3rd;
use crate::ffi::python::objective::{OwnedObjective, parse_objectives};
use crate::ffi::python::robot::{
    PyRobot, PyRobotModel, SharedRobot, shared_has_inverse_dynamics, with_shared_robot_mut_result,
};
use crate::solver::copp2_socp::ClarabelOptions as RustClarabelOptions;
use crate::solver::copp3_socp::{
    Copp3Problem as RustCopp3Problem, Copp3ProblemBuilder as Copp3ProblemBuilder3,
    copp3_socp as rust_copp3_socp, copp3_socp_expert_with_info as rust_copp3_socp_expert,
};
use numpy::PyArray1;
use pyo3::prelude::*;

/// Register COPP3 problem, option, result, and solver APIs.
pub(super) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyCopp3Problem>()?;
    m.add_function(wrap_pyfunction!(copp3_socp, m)?)?;
    m.add_function(wrap_pyfunction!(copp3_socp_expert, m)?)?;
    Ok(())
}

/// Python descriptor for a COPP3 problem.
///
/// Construction copies the linearization reference profile, owns objective
/// descriptors, and immediately calls the Rust COPP3 builder. The Rust build
/// step linearizes cached third-order jerk constraints inside the referenced
/// robot's constraint buffer.
#[pyclass(name = "Copp3Problem", module = "copp_py._native")]
pub(crate) struct PyCopp3Problem {
    /// Robot kept alive for borrowed Rust problem construction.
    robot: Py<PyRobot>,
    /// Shared robot storage used by detached solver calls.
    shared: SharedRobot,
    /// Python-owned objective descriptors.
    objectives: Vec<OwnedObjective>,
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
impl PyCopp3Problem {
    /// Construct and immediately linearize a COPP3 problem descriptor.
    ///
    /// The descriptor stores a reference to a Python robot, Python-owned
    /// objective data, and an owned copy of `a_linearization`. Construction
    /// calls `build_with_linearization()`, so it may update cached third-order
    /// linearization rows inside `robot.constraints`.
    #[new]
    #[pyo3(
        signature = (robot, objectives, a_linearization, *, idx_s_start = 0, a_boundary = (0.0, 0.0), b_boundary = (0.0, 0.0), num_stationary_max = NumStationaryMaxArg((1, 1)), a_linearization_floor = 1.0e-10),
        text_signature = "(robot, objectives, a_linearization, *, idx_s_start=0, a_boundary=(0.0, 0.0), b_boundary=(0.0, 0.0), num_stationary_max=1, a_linearization_floor=1e-10)"
    )]
    fn new(
        py: Python<'_>,
        robot: Py<PyRobot>,
        objectives: &Bound<'_, PyAny>,
        a_linearization: &Bound<'_, PyAny>,
        idx_s_start: usize,
        a_boundary: (f64, f64),
        b_boundary: (f64, f64),
        num_stationary_max: NumStationaryMaxArg,
        a_linearization_floor: f64,
    ) -> PyResult<Self> {
        let a_linearization = array_like_to_vec_f64("a_linearization", a_linearization)?;
        let shared = robot.bind(py).borrow().shared_robot();
        let problem = Self {
            robot,
            shared,
            objectives: parse_objectives(objectives)?,
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

    /// Return the robot referenced by this problem descriptor.
    #[getter]
    fn robot(&self, py: Python<'_>) -> Py<PyRobot> {
        self.robot.clone_ref(py)
    }

    /// Return the number of objectives stored in this descriptor.
    #[getter]
    fn objective_count(&self) -> usize {
        self.objectives.len()
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

    /// Return the number of station samples in this COPP3 interval.
    #[getter]
    fn s_len(&self) -> usize {
        self.a_linearization.len()
    }

    /// Rebuild the Rust COPP3 problem and refresh cached jerk linearization.
    fn validate(&self, py: Python<'_>) -> PyResult<()> {
        self.with_rust_problem(py, |_| Ok(()))
    }

    /// Return a compact representation for interactive Python sessions.
    fn __repr__(&self) -> String {
        format!(
            "Copp3Problem(idx_s_start={}, s_len={}, a_boundary=({}, {}), b_boundary=({}, {}), num_stationary_max=({}, {}), objective_count={})",
            self.idx_s_start,
            self.a_linearization.len(),
            self.a_boundary.0,
            self.a_boundary.1,
            self.b_boundary.0,
            self.b_boundary.1,
            self.num_stationary_max.0,
            self.num_stationary_max.1,
            self.objectives.len()
        )
    }
}

impl PyCopp3Problem {
    /// Build a linearized Rust COPP3 problem and pass it to `f`.
    fn with_rust_problem<R>(
        &self,
        py: Python<'_>,
        f: impl for<'a> FnOnce(&RustCopp3Problem<'a, PyRobotModel>) -> Result<R, CoppError>,
    ) -> PyResult<R> {
        let _ = py;
        Self::with_shared_problem(
            &self.shared,
            &self.objectives,
            &self.a_linearization,
            self.idx_s_start,
            self.a_boundary,
            self.b_boundary,
            self.num_stationary_max,
            self.a_linearization_floor,
            f,
        )
    }

    /// Build a linearized COPP3 problem from shared robot state and call `f`.
    #[allow(clippy::too_many_arguments)]
    fn with_shared_problem<R>(
        shared: &SharedRobot,
        objectives_owned: &[OwnedObjective],
        a_linearization: &[f64],
        idx_s_start: usize,
        a_boundary: (f64, f64),
        b_boundary: (f64, f64),
        num_stationary_max: (usize, usize),
        a_linearization_floor: f64,
        f: impl for<'a> FnOnce(&RustCopp3Problem<'a, PyRobotModel>) -> Result<R, CoppError>,
    ) -> PyResult<R> {
        with_shared_robot_mut_result(shared, |robot| {
            let objectives = objectives_owned
                .iter()
                .map(OwnedObjective::as_rust)
                .collect::<Vec<_>>();
            let problem = Copp3ProblemBuilder3::new(
                robot,
                objectives.as_slice(),
                idx_s_start,
                a_linearization,
                a_boundary,
                b_boundary,
            )
            .with_num_stationary_max_pair(num_stationary_max)
            .with_a_linearization_floor(a_linearization_floor)
            .build_with_linearization()?;
            validate_copp3_objectives(
                "Copp3Problem",
                problem.objectives,
                problem.robot.dim(),
                a_linearization.len(),
            )?;
            f(&problem)
        })
    }

    /// Solve this COPP3 problem with validated Rust Clarabel SOCP options.
    fn solve_copp3_socp(
        &self,
        py: Python<'_>,
        options: &RustClarabelOptions,
    ) -> PyResult<PyProfile3rd> {
        let shared = self.shared.clone();
        let objectives = self.objectives.clone();
        let a_linearization = self.a_linearization.clone();
        let idx_s_start = self.idx_s_start;
        let a_boundary = self.a_boundary;
        let b_boundary = self.b_boundary;
        let num_stationary_max = self.num_stationary_max;
        let a_linearization_floor = self.a_linearization_floor;
        if shared_has_inverse_dynamics(&shared)? {
            Self::with_shared_problem(
                &shared,
                &objectives,
                &a_linearization,
                idx_s_start,
                a_boundary,
                b_boundary,
                num_stationary_max,
                a_linearization_floor,
                |problem| rust_copp3_socp(problem, options),
            )
            .map(PyProfile3rd::from_rust)
        } else {
            py.detach(move || {
                Self::with_shared_problem(
                    &shared,
                    &objectives,
                    &a_linearization,
                    idx_s_start,
                    a_boundary,
                    b_boundary,
                    num_stationary_max,
                    a_linearization_floor,
                    |problem| rust_copp3_socp(problem, options),
                )
                .map(PyProfile3rd::from_rust)
            })
        }
    }

    /// Solve this COPP3 problem with expert Clarabel SOCP diagnostics.
    fn solve_copp3_socp_expert(
        &self,
        py: Python<'_>,
        options: &RustClarabelOptions,
    ) -> PyResult<PyCopp3ClarabelResult> {
        let shared = self.shared.clone();
        let objectives = self.objectives.clone();
        let a_linearization = self.a_linearization.clone();
        let idx_s_start = self.idx_s_start;
        let a_boundary = self.a_boundary;
        let b_boundary = self.b_boundary;
        let num_stationary_max = self.num_stationary_max;
        let a_linearization_floor = self.a_linearization_floor;
        if shared_has_inverse_dynamics(&shared)? {
            Self::with_shared_problem(
                &shared,
                &objectives,
                &a_linearization,
                idx_s_start,
                a_boundary,
                b_boundary,
                num_stationary_max,
                a_linearization_floor,
                |problem| {
                    let expert = rust_copp3_socp_expert(problem, options)?;
                    let objective_breakdown = expert
                        .result
                        .as_ref()
                        .map(|profile| objective_value_copp3_opt(problem, profile.as_parts()));
                    Ok(PyCopp3ClarabelResult::from_solution(
                        expert.result,
                        expert.solution,
                        expert.linsolver,
                        objective_breakdown,
                    ))
                },
            )
        } else {
            py.detach(move || {
                Self::with_shared_problem(
                    &shared,
                    &objectives,
                    &a_linearization,
                    idx_s_start,
                    a_boundary,
                    b_boundary,
                    num_stationary_max,
                    a_linearization_floor,
                    |problem| {
                        let expert = rust_copp3_socp_expert(problem, options)?;
                        let objective_breakdown = expert
                            .result
                            .as_ref()
                            .map(|profile| objective_value_copp3_opt(problem, profile.as_parts()));
                        Ok(PyCopp3ClarabelResult::from_solution(
                            expert.result,
                            expert.solution,
                            expert.linsolver,
                            objective_breakdown,
                        ))
                    },
                )
            })
        }
    }
}

/// Solve COPP3 with the Clarabel SOCP backend.
#[pyfunction]
#[pyo3(signature = (problem, options = None), text_signature = "(problem, options=None)")]
fn copp3_socp<'py>(
    py: Python<'py>,
    problem: PyRef<'py, PyCopp3Problem>,
    options: Option<PyRef<'py, PyClarabelOptions>>,
) -> PyResult<PyProfile3rd> {
    let options = clarabel_options_or_default(options)?;
    problem.solve_copp3_socp(py, &options)
}

/// Solve COPP3-SOCP and return raw Clarabel diagnostics.
#[pyfunction]
#[pyo3(signature = (problem, options = None), text_signature = "(problem, options=None)")]
fn copp3_socp_expert<'py>(
    py: Python<'py>,
    problem: PyRef<'py, PyCopp3Problem>,
    options: Option<PyRef<'py, PyClarabelOptions>>,
) -> PyResult<PyCopp3ClarabelResult> {
    let options = clarabel_options_or_default(options)?;
    problem.solve_copp3_socp_expert(py, &options)
}
