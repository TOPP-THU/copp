//! Python wrappers for COPP2 problem descriptors and solvers.

use super::clarabel::{
    PyClarabelLinearSolverInfo, PyClarabelOptions, PyClarabelSolverStatus,
    clarabel_options_or_default,
};
use crate::copp::copp2::opt2::copp2_socp::objective_value_copp2_opt;
use crate::diag::CoppError;
use crate::ffi::python::objective::{OwnedObjective, parse_objectives};
use crate::ffi::python::robot::{
    PyRobot, PyRobotModel, SharedRobot, shared_has_inverse_dynamics, with_shared_robot_result,
};
use crate::solver::copp2_socp::{
    ClarabelOptions as RustClarabelOptions, Copp2Problem as RustCopp2Problem, Copp2ProblemBuilder,
    copp2_socp as rust_copp2_socp, copp2_socp_expert_with_info as rust_copp2_socp_expert,
};
use clarabel::solver::{DefaultSolution, LinearSolverInfo};
use numpy::PyArray1;
use pyo3::prelude::*;

/// Register COPP2 problem, option, result, and solver APIs.
pub(super) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyCopp2Problem>()?;
    m.add_class::<PyCopp2SocpResult>()?;
    m.add_function(wrap_pyfunction!(copp2_socp, m)?)?;
    m.add_function(wrap_pyfunction!(copp2_socp_expert, m)?)?;
    Ok(())
}

/// Python-owned expert result for COPP2-SOCP.
///
/// The accepted profile and objective breakdown are `None` when the Clarabel
/// status is not accepted by the provided [`PyClarabelOptions`]. Raw Clarabel
/// vectors are still available regardless of status for diagnostics.
#[pyclass(name = "Copp2SocpResult", module = "copp_py._native")]
pub(crate) struct PyCopp2SocpResult {
    /// Accepted `a = (ds/dt)^2` profile, when available.
    a: Option<Vec<f64>>,
    /// Raw Clarabel primal solution vector.
    x: Vec<f64>,
    /// Raw Clarabel dual solution vector.
    z: Vec<f64>,
    /// Raw Clarabel primal-cone slack vector.
    s: Vec<f64>,
    /// Final Clarabel solver status.
    solver_status: PyClarabelSolverStatus,
    /// Primal objective value reported by Clarabel.
    obj_val: f64,
    /// Dual objective value reported by Clarabel.
    obj_val_dual: f64,
    /// Clarabel solve time in seconds.
    solve_time: f64,
    /// Number of interior-point iterations.
    iterations: u32,
    /// Primal residual reported by Clarabel.
    r_prim: f64,
    /// Dual residual reported by Clarabel.
    r_dual: f64,
    /// Linear-solver metadata reported by Clarabel.
    linsolver: PyClarabelLinearSolverInfo,
    /// Weighted COPP objective value computed from accepted `a`, when available.
    objective_value: Option<f64>,
    /// Per-objective unweighted values computed from accepted `a`, when available.
    objective_terms: Option<Vec<f64>>,
}

#[pymethods]
impl PyCopp2SocpResult {
    /// Return the accepted `a` profile, or `None` when unavailable.
    #[getter]
    fn a<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.a.as_ref().map(|a| PyArray1::from_vec(py, a.clone()))
    }

    /// Return the raw Clarabel primal solution vector.
    #[getter]
    fn x<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        PyArray1::from_vec(py, self.x.clone())
    }

    /// Return the raw Clarabel dual solution vector.
    #[getter]
    fn z<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        PyArray1::from_vec(py, self.z.clone())
    }

    /// Return the raw Clarabel primal-cone slack vector.
    #[getter]
    fn s<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        PyArray1::from_vec(py, self.s.clone())
    }

    /// Return the final Clarabel solver status.
    #[getter]
    fn solver_status(&self) -> PyClarabelSolverStatus {
        self.solver_status
    }

    /// Return the primal objective value reported by Clarabel.
    #[getter]
    fn obj_val(&self) -> f64 {
        self.obj_val
    }

    /// Return the dual objective value reported by Clarabel.
    #[getter]
    fn obj_val_dual(&self) -> f64 {
        self.obj_val_dual
    }

    /// Return Clarabel solve time in seconds.
    #[getter]
    fn solve_time(&self) -> f64 {
        self.solve_time
    }

    /// Return the number of interior-point iterations.
    #[getter]
    fn iterations(&self) -> u32 {
        self.iterations
    }

    /// Return the primal residual reported by Clarabel.
    #[getter]
    fn r_prim(&self) -> f64 {
        self.r_prim
    }

    /// Return the dual residual reported by Clarabel.
    #[getter]
    fn r_dual(&self) -> f64 {
        self.r_dual
    }

    /// Return linear-solver metadata reported by Clarabel.
    #[getter]
    fn linsolver(&self) -> PyClarabelLinearSolverInfo {
        self.linsolver.clone()
    }

    /// Return weighted COPP objective value computed from accepted `a`, if any.
    #[getter]
    fn objective_value(&self) -> Option<f64> {
        self.objective_value
    }

    /// Return per-objective unweighted values computed from accepted `a`, if any.
    #[getter]
    fn objective_terms<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.objective_terms
            .as_ref()
            .map(|terms| PyArray1::from_vec(py, terms.clone()))
    }

    /// Return a compact representation for interactive Python sessions.
    fn __repr__(&self) -> String {
        format!(
            "Copp2SocpResult(a_available={}, solver_status={:?}, iterations={}, objective_value={:?})",
            self.a.is_some(),
            self.solver_status,
            self.iterations,
            self.objective_value
        )
    }
}

impl PyCopp2SocpResult {
    /// Convert Rust expert outputs into a Python-owned result object.
    fn from_solution(
        a_profile: Option<Vec<f64>>,
        solution: DefaultSolution<f64>,
        linsolver: LinearSolverInfo,
        objective_breakdown: Option<(f64, Vec<f64>)>,
    ) -> Self {
        let DefaultSolution {
            x,
            z,
            s,
            status,
            obj_val,
            obj_val_dual,
            solve_time,
            iterations,
            r_prim,
            r_dual,
        } = solution;
        let (objective_value, objective_terms) = match objective_breakdown {
            Some((value, terms)) => (Some(value), Some(terms)),
            None => (None, None),
        };

        Self {
            a: a_profile,
            x,
            z,
            s,
            solver_status: PyClarabelSolverStatus::from_rust(status),
            obj_val,
            obj_val_dual,
            solve_time,
            iterations,
            r_prim,
            r_dual,
            linsolver: linsolver.into(),
            objective_value,
            objective_terms,
        }
    }
}

/// Python descriptor for a COPP2 problem.
#[pyclass(name = "Copp2Problem", module = "copp_py._native")]
pub(crate) struct PyCopp2Problem {
    /// Robot kept alive for borrowed Rust problem construction.
    robot: Py<PyRobot>,
    /// Shared robot storage used by detached solver calls.
    shared: SharedRobot,
    /// Python-owned objective descriptors.
    objectives: Vec<OwnedObjective>,
    /// Closed station-index interval `(idx_s_start, idx_s_final)`.
    idx_s_interval: (usize, usize),
    /// Endpoint state tuple `(a_start, a_final)`.
    a_boundary: (f64, f64),
}

#[pymethods]
impl PyCopp2Problem {
    /// Construct a COPP2 problem descriptor from a robot and objective list.
    ///
    /// The descriptor stores the Python robot and owned objective data, then
    /// rebuilds a validated borrowed Rust [`Copp2ProblemBuilder`] problem for
    /// each solver call.
    #[new]
    #[pyo3(
        signature = (robot, objectives, idx_s_interval, a_boundary = (0.0, 0.0)),
        text_signature = "(robot, objectives, idx_s_interval, a_boundary=(0.0, 0.0))"
    )]
    fn new(
        py: Python<'_>,
        robot: Py<PyRobot>,
        objectives: &Bound<'_, PyAny>,
        idx_s_interval: (usize, usize),
        a_boundary: (f64, f64),
    ) -> PyResult<Self> {
        let shared = robot.bind(py).borrow().shared_robot();
        let problem = Self {
            robot,
            shared,
            objectives: parse_objectives(objectives)?,
            idx_s_interval,
            a_boundary,
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

    /// Validate the descriptor against the current robot and objectives.
    fn validate(&self, py: Python<'_>) -> PyResult<()> {
        self.with_rust_problem(py, |_| Ok(()))
    }

    /// Return a compact representation for interactive Python sessions.
    fn __repr__(&self) -> String {
        format!(
            "Copp2Problem(idx_s_interval=({}, {}), a_boundary=({}, {}), objective_count={})",
            self.idx_s_interval.0,
            self.idx_s_interval.1,
            self.a_boundary.0,
            self.a_boundary.1,
            self.objectives.len()
        )
    }
}

impl PyCopp2Problem {
    /// Build a validated Rust COPP2 problem and pass it to `f`.
    fn with_rust_problem<R>(
        &self,
        py: Python<'_>,
        f: impl for<'a> FnOnce(&RustCopp2Problem<'a, PyRobotModel>) -> Result<R, CoppError>,
    ) -> PyResult<R> {
        let _ = py;
        Self::with_shared_problem(
            &self.shared,
            &self.objectives,
            self.idx_s_interval,
            self.a_boundary,
            f,
        )
    }

    /// Build a validated COPP2 problem from shared robot state and call `f`.
    fn with_shared_problem<R>(
        shared: &SharedRobot,
        objectives_owned: &[OwnedObjective],
        idx_s_interval: (usize, usize),
        a_boundary: (f64, f64),
        f: impl for<'a> FnOnce(&RustCopp2Problem<'a, PyRobotModel>) -> Result<R, CoppError>,
    ) -> PyResult<R> {
        with_shared_robot_result(shared, |robot| {
            let objectives = objectives_owned
                .iter()
                .map(OwnedObjective::as_rust)
                .collect::<Vec<_>>();
            let problem =
                Copp2ProblemBuilder::new(robot, idx_s_interval, a_boundary, objectives.as_slice())
                    .build()?;
            f(&problem)
        })
    }

    /// Solve this COPP2 problem with validated Rust Clarabel SOCP options.
    fn solve_copp2_socp(
        &self,
        py: Python<'_>,
        options: &RustClarabelOptions,
    ) -> PyResult<Vec<f64>> {
        let shared = self.shared.clone();
        let objectives = self.objectives.clone();
        let idx_s_interval = self.idx_s_interval;
        let a_boundary = self.a_boundary;
        if shared_has_inverse_dynamics(&shared)? {
            Self::with_shared_problem(
                &shared,
                &objectives,
                idx_s_interval,
                a_boundary,
                |problem| rust_copp2_socp(problem, options),
            )
        } else {
            py.detach(move || {
                Self::with_shared_problem(
                    &shared,
                    &objectives,
                    idx_s_interval,
                    a_boundary,
                    |problem| rust_copp2_socp(problem, options),
                )
            })
        }
    }

    /// Solve this COPP2 problem with expert Clarabel SOCP diagnostics.
    fn solve_copp2_socp_expert(
        &self,
        py: Python<'_>,
        options: &RustClarabelOptions,
    ) -> PyResult<PyCopp2SocpResult> {
        let shared = self.shared.clone();
        let objectives = self.objectives.clone();
        let idx_s_interval = self.idx_s_interval;
        let a_boundary = self.a_boundary;
        if shared_has_inverse_dynamics(&shared)? {
            Self::with_shared_problem(
                &shared,
                &objectives,
                idx_s_interval,
                a_boundary,
                |problem| {
                    let expert = rust_copp2_socp_expert(problem, options)?;
                    let objective_breakdown = expert.result.as_ref().map(|a| {
                        objective_value_copp2_opt(
                            problem.robot,
                            problem.idx_s_interval.0,
                            problem.objectives,
                            a,
                        )
                    });
                    Ok(PyCopp2SocpResult::from_solution(
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
                    idx_s_interval,
                    a_boundary,
                    |problem| {
                        let expert = rust_copp2_socp_expert(problem, options)?;
                        let objective_breakdown = expert.result.as_ref().map(|a| {
                            objective_value_copp2_opt(
                                problem.robot,
                                problem.idx_s_interval.0,
                                problem.objectives,
                                a,
                            )
                        });
                        Ok(PyCopp2SocpResult::from_solution(
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

/// Solve COPP2 with the Clarabel SOCP backend.
#[pyfunction]
#[pyo3(signature = (problem, options = None), text_signature = "(problem, options=None)")]
fn copp2_socp<'py>(
    py: Python<'py>,
    problem: PyRef<'py, PyCopp2Problem>,
    options: Option<PyRef<'py, PyClarabelOptions>>,
) -> PyResult<Bound<'py, PyArray1<f64>>> {
    let options = clarabel_options_or_default(options)?;
    let a = problem.solve_copp2_socp(py, &options)?;
    Ok(PyArray1::from_vec(py, a))
}

/// Solve COPP2 with the Clarabel SOCP backend and return diagnostics.
#[pyfunction]
#[pyo3(signature = (problem, options = None), text_signature = "(problem, options=None)")]
fn copp2_socp_expert<'py>(
    py: Python<'py>,
    problem: PyRef<'py, PyCopp2Problem>,
    options: Option<PyRef<'py, PyClarabelOptions>>,
) -> PyResult<PyCopp2SocpResult> {
    let options = clarabel_options_or_default(options)?;
    problem.solve_copp2_socp_expert(py, &options)
}
