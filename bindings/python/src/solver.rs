use crate::convert::{ndarray1_to_vec, vec_to_ndarray1};
use crate::enums::ClarabelSolverStatus;
use crate::errors::copp_err_to_py;
use crate::objectives::{ObjectiveKind, PyObjective};
use crate::options::{PyClarabelOptions, PyReachSet2Options};
use crate::robot::PyRobot;
use clarabel::solver::{DefaultSolution, LinearSolverInfo};
use copp::InterpolationMode;
use copp::solver::copp2_socp::{
    ClarabelOptionsBuilder, Copp2ProblemBuilder, CoppObjective, a_to_b_topp2 as rust_a_to_b,
    copp2_socp as rust_copp2_socp, copp2_socp_expert_with_info as rust_copp2_socp_expert,
    objective_value_copp2_opt, s_to_t_topp2 as rust_s_to_t2, t_to_s_topp2 as rust_t_to_s2,
};
use copp::solver::copp3_socp::{
    Copp3ProblemBuilder, Topp3Profile, copp3_socp as rust_copp3_socp,
    copp3_socp_expert_with_info as rust_copp3_socp_expert, objective_value_copp3_opt,
};
use copp::solver::reach_set2::{
    ReachSet2OptionsBuilder, Topp2ProblemBuilder, reach_set2_backward as rust_reach_set2_backward,
    reach_set2_bidirectional as rust_reach_set2_bidirectional,
};
use copp::solver::topp2_ra::topp2_ra as rust_topp2_ra;
use copp::solver::topp3_lp::{
    Topp3ProblemBuilder, force_positive_a as rust_force_positive_a, s_to_t_topp3 as rust_s_to_t3,
    t_to_s_topp3 as rust_t_to_s3, topp3_lp as rust_topp3_lp,
    topp3_lp_expert_with_info as rust_topp3_lp_expert,
};
use copp::solver::topp3_socp::{
    topp3_socp as rust_topp3_socp, topp3_socp_expert_with_info as rust_topp3_socp_expert,
};
use numpy::{PyArray1, PyReadonlyArray1, PyReadwriteArray1};
use pyo3::prelude::*;

type PyArray1Bound<'py> = Bound<'py, PyArray1<f64>>;
type PyArrayPair<'py> = (PyArray1Bound<'py>, PyArray1Bound<'py>);
type PyTopp3Result<'py> = (PyArray1Bound<'py>, PyArray1Bound<'py>, (usize, usize));
type PyTopp3ExpertResult = (
    Option<PyTopp3Profile>,
    PyClarabelSolution,
    PyLinearSolverInfo,
);

#[pyclass(name = "Topp3Profile")]
#[derive(Clone)]
pub struct PyTopp3Profile {
    a: Vec<f64>,
    b: Vec<f64>,
    num_stationary: (usize, usize),
}

#[pymethods]
impl PyTopp3Profile {
    #[new]
    fn new(
        a: PyReadonlyArray1<'_, f64>,
        b: PyReadonlyArray1<'_, f64>,
        num_stationary: (usize, usize),
    ) -> Self {
        Self {
            a: ndarray1_to_vec(a),
            b: ndarray1_to_vec(b),
            num_stationary,
        }
    }

    #[getter]
    fn a<'py>(&self, py: Python<'py>) -> PyArray1Bound<'py> {
        vec_to_ndarray1(py, &self.a)
    }

    #[getter]
    fn b<'py>(&self, py: Python<'py>) -> PyArray1Bound<'py> {
        vec_to_ndarray1(py, &self.b)
    }

    #[getter]
    fn num_stationary(&self) -> (usize, usize) {
        self.num_stationary
    }

    fn as_tuple<'py>(&self, py: Python<'py>) -> PyTopp3Result<'py> {
        (
            vec_to_ndarray1(py, &self.a),
            vec_to_ndarray1(py, &self.b),
            self.num_stationary,
        )
    }
}

impl From<Topp3Profile> for PyTopp3Profile {
    fn from(profile: Topp3Profile) -> Self {
        let (a, b, num_stationary) = profile.into_parts();
        Self {
            a,
            b,
            num_stationary,
        }
    }
}

#[pyclass(name = "ClarabelSolution")]
pub struct PyClarabelSolution {
    x: Vec<f64>,
    z: Vec<f64>,
    s: Vec<f64>,
    status: ClarabelSolverStatus,
    obj_val: f64,
    obj_val_dual: f64,
    solve_time: f64,
    iterations: u32,
    r_prim: f64,
    r_dual: f64,
    objective_value: f64,
    objective_terms: Vec<f64>,
}

#[pymethods]
impl PyClarabelSolution {
    #[getter]
    fn x<'py>(&self, py: Python<'py>) -> PyArray1Bound<'py> {
        vec_to_ndarray1(py, &self.x)
    }

    #[getter]
    fn z<'py>(&self, py: Python<'py>) -> PyArray1Bound<'py> {
        vec_to_ndarray1(py, &self.z)
    }

    #[getter]
    fn s<'py>(&self, py: Python<'py>) -> PyArray1Bound<'py> {
        vec_to_ndarray1(py, &self.s)
    }

    #[getter]
    fn status(&self) -> ClarabelSolverStatus {
        self.status
    }

    #[getter]
    fn obj_val(&self) -> f64 {
        self.obj_val
    }

    #[getter]
    fn obj_val_dual(&self) -> f64 {
        self.obj_val_dual
    }

    #[getter]
    fn solve_time(&self) -> f64 {
        self.solve_time
    }

    #[getter]
    fn iterations(&self) -> u32 {
        self.iterations
    }

    #[getter]
    fn r_prim(&self) -> f64 {
        self.r_prim
    }

    #[getter]
    fn r_dual(&self) -> f64 {
        self.r_dual
    }

    #[getter]
    fn objective_value(&self) -> f64 {
        self.objective_value
    }

    #[getter]
    fn objective_terms<'py>(&self, py: Python<'py>) -> PyArray1Bound<'py> {
        vec_to_ndarray1(py, &self.objective_terms)
    }
}

#[pyclass(name = "LinearSolverInfo")]
pub struct PyLinearSolverInfo {
    #[pyo3(get)]
    name: String,
    #[pyo3(get)]
    threads: usize,
    #[pyo3(get)]
    direct: bool,
    #[pyo3(get)]
    nnz_a: usize,
    #[pyo3(get)]
    nnz_l: usize,
}

impl From<LinearSolverInfo> for PyLinearSolverInfo {
    fn from(info: LinearSolverInfo) -> Self {
        Self {
            name: info.name,
            threads: info.threads,
            direct: info.direct,
            nnz_a: info.nnzA,
            nnz_l: info.nnzL,
        }
    }
}

fn py_clarabel_solution(solution: DefaultSolution<f64>) -> PyClarabelSolution {
    py_clarabel_solution_with_obj(solution, None)
}

fn py_clarabel_solution_with_obj(
    solution: DefaultSolution<f64>,
    objective_breakdown: Option<(f64, Vec<f64>)>,
) -> PyClarabelSolution {
    let (objective_value, objective_terms) =
        objective_breakdown.unwrap_or((f64::NAN, Vec::new()));
    PyClarabelSolution {
        x: solution.x,
        z: solution.z,
        s: solution.s,
        status: solution.status.into(),
        obj_val: solution.obj_val,
        obj_val_dual: solution.obj_val_dual,
        solve_time: solution.solve_time,
        iterations: solution.iterations,
        r_prim: solution.r_prim,
        r_dual: solution.r_dual,
        objective_value,
        objective_terms,
    }
}

fn convert_objectives<'a>(objectives: &'a [PyObjective]) -> Vec<CoppObjective<'a>> {
    objectives
        .iter()
        .map(|objective| match &objective.kind {
            ObjectiveKind::Time(weight) => CoppObjective::Time(*weight),
            ObjectiveKind::ThermalEnergy(weight, normalize) => {
                CoppObjective::ThermalEnergy(*weight, normalize.as_slice())
            }
            ObjectiveKind::TotalVariationTorque(weight, normalize) => {
                CoppObjective::TotalVariationTorque(*weight, normalize.as_slice())
            }
            ObjectiveKind::Linear(weight, alpha, beta) => {
                CoppObjective::Linear(*weight, alpha.as_slice(), beta.as_slice())
            }
        })
        .collect()
}

// ─── TOPP2-RA ───

#[pyfunction]
#[pyo3(signature = (robot, idx_s_start, idx_s_final, a_start, a_final, options=None))]
pub fn topp2_ra<'py>(
    py: Python<'py>,
    robot: &PyRobot,
    idx_s_start: usize,
    idx_s_final: usize,
    a_start: f64,
    a_final: f64,
    options: Option<&PyReachSet2Options>,
) -> PyResult<PyArray1Bound<'py>> {
    let problem = Topp2ProblemBuilder::with_constraint(
        &robot.inner.constraints,
        (idx_s_start, idx_s_final),
        (a_start, a_final),
    )
    .build()
    .map_err(copp_err_to_py)?;

    let default_opts;
    let opts = match options {
        Some(o) => &o.inner,
        None => {
            default_opts = ReachSet2OptionsBuilder::new()
                .build()
                .map_err(copp_err_to_py)?;
            &default_opts
        }
    };

    let result = rust_topp2_ra(&problem, opts).map_err(copp_err_to_py)?;
    Ok(vec_to_ndarray1(py, &result))
}

// ─── Reach-Set 2 ───

#[pyfunction]
#[pyo3(signature = (robot, idx_s_start, idx_s_final, a_start, a_final, options=None))]
pub fn reach_set2_backward<'py>(
    py: Python<'py>,
    robot: &PyRobot,
    idx_s_start: usize,
    idx_s_final: usize,
    a_start: f64,
    a_final: f64,
    options: Option<&PyReachSet2Options>,
) -> PyResult<PyArrayPair<'py>> {
    let problem = Topp2ProblemBuilder::with_constraint(
        &robot.inner.constraints,
        (idx_s_start, idx_s_final),
        (a_start, a_final),
    )
    .build()
    .map_err(copp_err_to_py)?;

    let default_opts;
    let opts = match options {
        Some(o) => &o.inner,
        None => {
            default_opts = ReachSet2OptionsBuilder::new()
                .build()
                .map_err(copp_err_to_py)?;
            &default_opts
        }
    };

    let rs = rust_reach_set2_backward(&problem, opts).map_err(copp_err_to_py)?;
    Ok((
        vec_to_ndarray1(py, &rs.a_max),
        vec_to_ndarray1(py, &rs.a_min),
    ))
}

#[pyfunction]
#[pyo3(signature = (robot, idx_s_start, idx_s_final, a_start, a_final, options=None))]
pub fn reach_set2_bidirectional<'py>(
    py: Python<'py>,
    robot: &PyRobot,
    idx_s_start: usize,
    idx_s_final: usize,
    a_start: f64,
    a_final: f64,
    options: Option<&PyReachSet2Options>,
) -> PyResult<PyArrayPair<'py>> {
    let problem = Topp2ProblemBuilder::with_constraint(
        &robot.inner.constraints,
        (idx_s_start, idx_s_final),
        (a_start, a_final),
    )
    .build()
    .map_err(copp_err_to_py)?;

    let default_opts;
    let opts = match options {
        Some(o) => &o.inner,
        None => {
            default_opts = ReachSet2OptionsBuilder::new()
                .build()
                .map_err(copp_err_to_py)?;
            &default_opts
        }
    };

    let rs = rust_reach_set2_bidirectional(&problem, opts).map_err(copp_err_to_py)?;
    Ok((
        vec_to_ndarray1(py, &rs.a_max),
        vec_to_ndarray1(py, &rs.a_min),
    ))
}

// ─── COPP2-SOCP ───

#[pyfunction]
#[pyo3(signature = (robot, idx_s_start, idx_s_final, a_start, a_final, objectives, options=None))]
pub fn copp2_socp<'py>(
    py: Python<'py>,
    robot: &PyRobot,
    idx_s_start: usize,
    idx_s_final: usize,
    a_start: f64,
    a_final: f64,
    objectives: Vec<PyObjective>,
    options: Option<&PyClarabelOptions>,
) -> PyResult<PyArray1Bound<'py>> {
    let copp_objs = convert_objectives(&objectives);

    let problem = Copp2ProblemBuilder::new(
        &robot.inner,
        (idx_s_start, idx_s_final),
        (a_start, a_final),
        &copp_objs,
    )
    .build()
    .map_err(copp_err_to_py)?;

    let default_opts;
    let opts = match options {
        Some(o) => &o.inner,
        None => {
            default_opts = ClarabelOptionsBuilder::new()
                .build()
                .map_err(copp_err_to_py)?;
            &default_opts
        }
    };

    let result = rust_copp2_socp(&problem, opts);
    if let Some(err) = robot.take_callback_error() {
        return Err(err);
    }
    let result = result.map_err(copp_err_to_py)?;
    Ok(vec_to_ndarray1(py, &result))
}

#[pyfunction]
#[pyo3(signature = (robot, idx_s_start, idx_s_final, a_start, a_final, objectives, options=None))]
#[allow(clippy::too_many_arguments)]
pub fn copp2_socp_expert<'py>(
    py: Python<'py>,
    robot: &PyRobot,
    idx_s_start: usize,
    idx_s_final: usize,
    a_start: f64,
    a_final: f64,
    objectives: Vec<PyObjective>,
    options: Option<&PyClarabelOptions>,
) -> PyResult<(
    Option<PyArray1Bound<'py>>,
    PyClarabelSolution,
    PyLinearSolverInfo,
)> {
    let copp_objs = convert_objectives(&objectives);
    let problem = Copp2ProblemBuilder::new(
        &robot.inner,
        (idx_s_start, idx_s_final),
        (a_start, a_final),
        &copp_objs,
    )
    .build()
    .map_err(copp_err_to_py)?;
    let default_opts;
    let opts = match options {
        Some(o) => &o.inner,
        None => {
            default_opts = ClarabelOptionsBuilder::new()
                .build()
                .map_err(copp_err_to_py)?;
            &default_opts
        }
    };
    let result = rust_copp2_socp_expert(&problem, opts);
    if let Some(err) = robot.take_callback_error() {
        return Err(err);
    }
    let info = result.map_err(copp_err_to_py)?;
    let objective_breakdown = info
        .result
        .as_ref()
        .map(|a| objective_value_copp2_opt(&robot.inner, idx_s_start, &copp_objs, a));
    Ok((
        info.result.as_ref().map(|v| vec_to_ndarray1(py, v)),
        py_clarabel_solution_with_obj(info.solution, objective_breakdown),
        info.linsolver.into(),
    ))
}

// ─── TOPP3-LP ───

#[pyfunction]
#[pyo3(signature = (robot, idx_s_start, a_linearization, a_start, a_final, b_start, b_final, options=None, num_stationary_max=None, a_linearization_floor=None))]
#[allow(clippy::too_many_arguments)]
pub fn topp3_lp<'py>(
    py: Python<'py>,
    robot: &mut PyRobot,
    idx_s_start: usize,
    a_linearization: PyReadonlyArray1<'_, f64>,
    a_start: f64,
    a_final: f64,
    b_start: f64,
    b_final: f64,
    options: Option<&PyClarabelOptions>,
    num_stationary_max: Option<(usize, usize)>,
    a_linearization_floor: Option<f64>,
) -> PyResult<PyTopp3Result<'py>> {
    let a_lin = ndarray1_to_vec(a_linearization);

    let mut builder = Topp3ProblemBuilder::with_constraint(
        &mut robot.inner.constraints,
        idx_s_start,
        &a_lin,
        (a_start, a_final),
        (b_start, b_final),
    );
    if let Some(ns) = num_stationary_max {
        builder = builder.with_num_stationary_max_pair(ns);
    }
    if let Some(floor) = a_linearization_floor {
        builder = builder.with_a_linearization_floor(floor);
    }
    let problem = builder.build_with_linearization().map_err(copp_err_to_py)?;

    let default_opts;
    let opts = match options {
        Some(o) => &o.inner,
        None => {
            default_opts = ClarabelOptionsBuilder::new()
                .build()
                .map_err(copp_err_to_py)?;
            &default_opts
        }
    };

    let (a, b, num_st) = rust_topp3_lp(&problem, opts)
        .map_err(copp_err_to_py)?
        .into_parts();
    Ok((vec_to_ndarray1(py, &a), vec_to_ndarray1(py, &b), num_st))
}

#[pyfunction]
#[pyo3(signature = (robot, idx_s_start, a_linearization, a_start, a_final, b_start, b_final, options=None, num_stationary_max=None, a_linearization_floor=None))]
#[allow(clippy::too_many_arguments)]
pub fn topp3_lp_expert<'py>(
    _py: Python<'py>,
    robot: &mut PyRobot,
    idx_s_start: usize,
    a_linearization: PyReadonlyArray1<'_, f64>,
    a_start: f64,
    a_final: f64,
    b_start: f64,
    b_final: f64,
    options: Option<&PyClarabelOptions>,
    num_stationary_max: Option<(usize, usize)>,
    a_linearization_floor: Option<f64>,
) -> PyResult<PyTopp3ExpertResult> {
    let a_lin = ndarray1_to_vec(a_linearization);
    let mut builder = Topp3ProblemBuilder::with_constraint(
        &mut robot.inner.constraints,
        idx_s_start,
        &a_lin,
        (a_start, a_final),
        (b_start, b_final),
    );
    if let Some(ns) = num_stationary_max {
        builder = builder.with_num_stationary_max_pair(ns);
    }
    if let Some(floor) = a_linearization_floor {
        builder = builder.with_a_linearization_floor(floor);
    }
    let problem = builder.build_with_linearization().map_err(copp_err_to_py)?;

    let mut default_opts = None;
    let opts = default_clarabel_options(options, &mut default_opts)?;
    let info = rust_topp3_lp_expert(&problem, opts).map_err(copp_err_to_py)?;
    Ok((
        info.result.map(PyTopp3Profile::from),
        py_clarabel_solution(info.solution),
        info.linsolver.into(),
    ))
}

// ─── TOPP3-SOCP ───

#[pyfunction]
#[pyo3(signature = (robot, idx_s_start, a_linearization, a_start, a_final, b_start, b_final, options=None, num_stationary_max=None, a_linearization_floor=None))]
#[allow(clippy::too_many_arguments)]
pub fn topp3_socp<'py>(
    py: Python<'py>,
    robot: &mut PyRobot,
    idx_s_start: usize,
    a_linearization: PyReadonlyArray1<'_, f64>,
    a_start: f64,
    a_final: f64,
    b_start: f64,
    b_final: f64,
    options: Option<&PyClarabelOptions>,
    num_stationary_max: Option<(usize, usize)>,
    a_linearization_floor: Option<f64>,
) -> PyResult<PyTopp3Result<'py>> {
    let a_lin = ndarray1_to_vec(a_linearization);

    let mut builder = Topp3ProblemBuilder::with_constraint(
        &mut robot.inner.constraints,
        idx_s_start,
        &a_lin,
        (a_start, a_final),
        (b_start, b_final),
    );
    if let Some(ns) = num_stationary_max {
        builder = builder.with_num_stationary_max_pair(ns);
    }
    if let Some(floor) = a_linearization_floor {
        builder = builder.with_a_linearization_floor(floor);
    }
    let problem = builder.build_with_linearization().map_err(copp_err_to_py)?;

    let default_opts;
    let opts = match options {
        Some(o) => &o.inner,
        None => {
            default_opts = ClarabelOptionsBuilder::new()
                .build()
                .map_err(copp_err_to_py)?;
            &default_opts
        }
    };

    let (a, b, num_st) = rust_topp3_socp(&problem, opts)
        .map_err(copp_err_to_py)?
        .into_parts();
    Ok((vec_to_ndarray1(py, &a), vec_to_ndarray1(py, &b), num_st))
}

#[pyfunction]
#[pyo3(signature = (robot, idx_s_start, a_linearization, a_start, a_final, b_start, b_final, options=None, num_stationary_max=None, a_linearization_floor=None))]
#[allow(clippy::too_many_arguments)]
pub fn topp3_socp_expert<'py>(
    _py: Python<'py>,
    robot: &mut PyRobot,
    idx_s_start: usize,
    a_linearization: PyReadonlyArray1<'_, f64>,
    a_start: f64,
    a_final: f64,
    b_start: f64,
    b_final: f64,
    options: Option<&PyClarabelOptions>,
    num_stationary_max: Option<(usize, usize)>,
    a_linearization_floor: Option<f64>,
) -> PyResult<PyTopp3ExpertResult> {
    let a_lin = ndarray1_to_vec(a_linearization);
    let mut builder = Topp3ProblemBuilder::with_constraint(
        &mut robot.inner.constraints,
        idx_s_start,
        &a_lin,
        (a_start, a_final),
        (b_start, b_final),
    );
    if let Some(ns) = num_stationary_max {
        builder = builder.with_num_stationary_max_pair(ns);
    }
    if let Some(floor) = a_linearization_floor {
        builder = builder.with_a_linearization_floor(floor);
    }
    let problem = builder.build_with_linearization().map_err(copp_err_to_py)?;

    let mut default_opts = None;
    let opts = default_clarabel_options(options, &mut default_opts)?;
    let info = rust_topp3_socp_expert(&problem, opts).map_err(copp_err_to_py)?;
    Ok((
        info.result.map(PyTopp3Profile::from),
        py_clarabel_solution(info.solution),
        info.linsolver.into(),
    ))
}

// ─── COPP3-SOCP ───

#[pyfunction]
#[pyo3(signature = (robot, idx_s_start, a_linearization, a_start, a_final, b_start, b_final, objectives, options=None, num_stationary_max=None, a_linearization_floor=None))]
#[allow(clippy::too_many_arguments)]
pub fn copp3_socp<'py>(
    py: Python<'py>,
    robot: &mut PyRobot,
    idx_s_start: usize,
    a_linearization: PyReadonlyArray1<'_, f64>,
    a_start: f64,
    a_final: f64,
    b_start: f64,
    b_final: f64,
    objectives: Vec<PyObjective>,
    options: Option<&PyClarabelOptions>,
    num_stationary_max: Option<(usize, usize)>,
    a_linearization_floor: Option<f64>,
) -> PyResult<PyTopp3Result<'py>> {
    let a_lin = ndarray1_to_vec(a_linearization);
    let copp_objs = convert_objectives(&objectives);

    let mut builder = Copp3ProblemBuilder::new(
        &mut robot.inner,
        &copp_objs,
        idx_s_start,
        &a_lin,
        (a_start, a_final),
        (b_start, b_final),
    );
    if let Some(ns) = num_stationary_max {
        builder = builder.with_num_stationary_max_pair(ns);
    }
    if let Some(floor) = a_linearization_floor {
        builder = builder.with_a_linearization_floor(floor);
    }
    let problem = builder.build_with_linearization().map_err(copp_err_to_py)?;

    let default_opts;
    let opts = match options {
        Some(o) => &o.inner,
        None => {
            default_opts = ClarabelOptionsBuilder::new()
                .build()
                .map_err(copp_err_to_py)?;
            &default_opts
        }
    };

    let result = rust_copp3_socp(&problem, opts);
    if let Some(err) = robot.take_callback_error() {
        return Err(err);
    }
    let (a, b, num_st) = result.map_err(copp_err_to_py)?.into_parts();
    Ok((vec_to_ndarray1(py, &a), vec_to_ndarray1(py, &b), num_st))
}

#[pyfunction]
#[pyo3(signature = (robot, idx_s_start, a_linearization, a_start, a_final, b_start, b_final, objectives, options=None, num_stationary_max=None, a_linearization_floor=None))]
#[allow(clippy::too_many_arguments)]
pub fn copp3_socp_expert<'py>(
    _py: Python<'py>,
    robot: &mut PyRobot,
    idx_s_start: usize,
    a_linearization: PyReadonlyArray1<'_, f64>,
    a_start: f64,
    a_final: f64,
    b_start: f64,
    b_final: f64,
    objectives: Vec<PyObjective>,
    options: Option<&PyClarabelOptions>,
    num_stationary_max: Option<(usize, usize)>,
    a_linearization_floor: Option<f64>,
) -> PyResult<PyTopp3ExpertResult> {
    let a_lin = ndarray1_to_vec(a_linearization);
    let copp_objs = convert_objectives(&objectives);
    let mut builder = Copp3ProblemBuilder::new(
        &mut robot.inner,
        &copp_objs,
        idx_s_start,
        &a_lin,
        (a_start, a_final),
        (b_start, b_final),
    );
    if let Some(ns) = num_stationary_max {
        builder = builder.with_num_stationary_max_pair(ns);
    }
    if let Some(floor) = a_linearization_floor {
        builder = builder.with_a_linearization_floor(floor);
    }
    let problem = builder.build_with_linearization().map_err(copp_err_to_py)?;
    let mut default_opts = None;
    let opts = default_clarabel_options(options, &mut default_opts)?;
    let result = rust_copp3_socp_expert(&problem, opts);
    // Compute the objective breakdown into owned values while `problem` (which
    // borrows `robot.inner`) is still alive. It reads cached constraint data and
    // does not invoke the Python callback, so it is safe before the error check.
    let objective_breakdown = result.as_ref().ok().and_then(|info| {
        info.result
            .as_ref()
            .map(|profile| objective_value_copp3_opt(&problem, profile.as_parts()))
    });
    if let Some(err) = robot.take_callback_error() {
        return Err(err);
    }
    let info = result.map_err(copp_err_to_py)?;
    Ok((
        info.result.map(PyTopp3Profile::from),
        py_clarabel_solution_with_obj(info.solution, objective_breakdown),
        info.linsolver.into(),
    ))
}

// ─── Interpolation (TOPP2) ───

#[pyfunction]
#[pyo3(signature = (s, a, t_offset=0.0))]
pub fn s_to_t_topp2<'py>(
    py: Python<'py>,
    s: PyReadonlyArray1<'_, f64>,
    a: PyReadonlyArray1<'_, f64>,
    t_offset: f64,
) -> PyResult<(f64, Bound<'py, PyArray1<f64>>)> {
    let (tf, ts) =
        rust_s_to_t2(&ndarray1_to_vec(s), &ndarray1_to_vec(a), t_offset).map_err(copp_err_to_py)?;
    Ok((tf, vec_to_ndarray1(py, &ts)))
}

#[pyfunction]
#[pyo3(signature = (s, a, t_s, dt=None, t_samples=None, t0=0.0, include_final=true))]
#[allow(clippy::too_many_arguments)]
pub fn t_to_s_topp2<'py>(
    py: Python<'py>,
    s: PyReadonlyArray1<'_, f64>,
    a: PyReadonlyArray1<'_, f64>,
    t_s: PyReadonlyArray1<'_, f64>,
    dt: Option<f64>,
    t_samples: Option<PyReadonlyArray1<'_, f64>>,
    t0: f64,
    include_final: bool,
) -> PyResult<PyArray1Bound<'py>> {
    let sv = ndarray1_to_vec(s);
    let av = ndarray1_to_vec(a);
    let tsv = ndarray1_to_vec(t_s);

    if let Some(dt_val) = dt {
        let mode = InterpolationMode::UniformTimeGrid(t0, dt_val, include_final);
        let r = rust_t_to_s2(&sv, &av, &tsv, mode).map_err(copp_err_to_py)?;
        return Ok(vec_to_ndarray1(py, &r));
    }
    if let Some(ts) = t_samples {
        let tv = ndarray1_to_vec(ts);
        let mode = InterpolationMode::NonUniformTimeGrid(&tv);
        let r = rust_t_to_s2(&sv, &av, &tsv, mode).map_err(copp_err_to_py)?;
        return Ok(vec_to_ndarray1(py, &r));
    }
    Err(pyo3::exceptions::PyValueError::new_err(
        "Either dt or t_samples must be provided",
    ))
}

#[pyfunction]
pub fn a_to_b_topp2<'py>(
    py: Python<'py>,
    s: PyReadonlyArray1<'_, f64>,
    a: PyReadonlyArray1<'_, f64>,
) -> PyResult<PyArray1Bound<'py>> {
    let r = rust_a_to_b(&ndarray1_to_vec(s), &ndarray1_to_vec(a)).map_err(copp_err_to_py)?;
    Ok(vec_to_ndarray1(py, &r))
}

// ─── Interpolation (TOPP3) ───

#[pyfunction]
#[pyo3(signature = (s, a, b, num_stationary, t_offset=0.0))]
pub fn s_to_t_topp3<'py>(
    py: Python<'py>,
    s: PyReadonlyArray1<'_, f64>,
    a: PyReadonlyArray1<'_, f64>,
    b: PyReadonlyArray1<'_, f64>,
    num_stationary: (usize, usize),
    t_offset: f64,
) -> PyResult<(f64, Bound<'py, PyArray1<f64>>)> {
    let sv = ndarray1_to_vec(s);
    let av = ndarray1_to_vec(a);
    let bv = ndarray1_to_vec(b);
    let (tf, ts) =
        rust_s_to_t3(&sv, (&av, &bv, num_stationary), t_offset).map_err(copp_err_to_py)?;
    Ok((tf, vec_to_ndarray1(py, &ts)))
}

#[pyfunction]
#[pyo3(signature = (s, a, b, num_stationary, t_s, dt=None, t_samples=None, t0=0.0, include_final=true))]
#[allow(clippy::too_many_arguments)]
pub fn t_to_s_topp3<'py>(
    py: Python<'py>,
    s: PyReadonlyArray1<'_, f64>,
    a: PyReadonlyArray1<'_, f64>,
    b: PyReadonlyArray1<'_, f64>,
    num_stationary: (usize, usize),
    t_s: PyReadonlyArray1<'_, f64>,
    dt: Option<f64>,
    t_samples: Option<PyReadonlyArray1<'_, f64>>,
    t0: f64,
    include_final: bool,
) -> PyResult<PyArray1Bound<'py>> {
    let sv = ndarray1_to_vec(s);
    let av = ndarray1_to_vec(a);
    let bv = ndarray1_to_vec(b);
    let tsv = ndarray1_to_vec(t_s);
    let profile = (&av[..], &bv[..], num_stationary);

    if let Some(dt_val) = dt {
        let mode = InterpolationMode::UniformTimeGrid(t0, dt_val, include_final);
        let r = rust_t_to_s3(&sv, profile, &tsv, mode).map_err(copp_err_to_py)?;
        return Ok(vec_to_ndarray1(py, &r));
    }
    if let Some(ts) = t_samples {
        let tv = ndarray1_to_vec(ts);
        let mode = InterpolationMode::NonUniformTimeGrid(&tv);
        let r = rust_t_to_s3(&sv, profile, &tsv, mode).map_err(copp_err_to_py)?;
        return Ok(vec_to_ndarray1(py, &r));
    }
    Err(pyo3::exceptions::PyValueError::new_err(
        "Either dt or t_samples must be provided",
    ))
}

#[pyfunction]
#[pyo3(signature = (s, a, b, num_stationary, a_min))]
pub fn force_positive_a_3rd(
    s: PyReadonlyArray1<'_, f64>,
    mut a: PyReadwriteArray1<'_, f64>,
    mut b: PyReadwriteArray1<'_, f64>,
    num_stationary: (usize, usize),
    a_min: f64,
) -> PyResult<bool> {
    let sv = ndarray1_to_vec(s);
    let a_slice = a.as_slice_mut()?;
    let b_slice = b.as_slice_mut()?;
    rust_force_positive_a((a_slice, b_slice, num_stationary), &sv, a_min).map_err(copp_err_to_py)
}

fn default_clarabel_options<'a>(
    options: Option<&'a PyClarabelOptions>,
    default_opts: &'a mut Option<copp::solver::copp2_socp::ClarabelOptions>,
) -> PyResult<&'a copp::solver::copp2_socp::ClarabelOptions> {
    if let Some(options) = options {
        return Ok(&options.inner);
    }
    *default_opts = Some(
        ClarabelOptionsBuilder::new()
            .build()
            .map_err(copp_err_to_py)?,
    );
    Ok(default_opts.as_ref().unwrap())
}
