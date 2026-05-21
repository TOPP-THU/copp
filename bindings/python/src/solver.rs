use crate::convert::{ndarray1_to_vec, vec_to_ndarray1};
use crate::errors::copp_err_to_py;
use crate::objectives::{ObjectiveKind, PyObjective};
use crate::options::{PyClarabelOptions, PyReachSet2Options};
use crate::robot::PyRobot;
use copp::InterpolationMode;
use copp::solver::copp2_socp::{
    ClarabelOptionsBuilder, Copp2ProblemBuilder, CoppObjective, a_to_b_topp2 as rust_a_to_b,
    copp2_socp as rust_copp2_socp, s_to_t_topp2 as rust_s_to_t2, t_to_s_topp2 as rust_t_to_s2,
};
use copp::solver::copp3_socp::{Copp3ProblemBuilder, copp3_socp as rust_copp3_socp};
use copp::solver::reach_set2::{
    ReachSet2OptionsBuilder, Topp2ProblemBuilder, reach_set2_backward as rust_reach_set2_backward,
    reach_set2_bidirectional as rust_reach_set2_bidirectional,
};
use copp::solver::topp2_ra::topp2_ra as rust_topp2_ra;
use copp::solver::topp3_lp::{
    Topp3ProblemBuilder, s_to_t_topp3 as rust_s_to_t3, t_to_s_topp3 as rust_t_to_s3,
    topp3_lp as rust_topp3_lp,
};
use copp::solver::topp3_socp::topp3_socp as rust_topp3_socp;
use numpy::{PyArray1, PyReadonlyArray1};
use pyo3::prelude::*;

type PyArray1Bound<'py> = Bound<'py, PyArray1<f64>>;
type PyArrayPair<'py> = (PyArray1Bound<'py>, PyArray1Bound<'py>);
type PyTopp3Result<'py> = (PyArray1Bound<'py>, PyArray1Bound<'py>, (usize, usize));

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
#[pyo3(signature = (robot, idx_s_interval, a_boundary, options=None))]
pub fn topp2_ra<'py>(
    py: Python<'py>,
    robot: &PyRobot,
    idx_s_interval: (usize, usize),
    a_boundary: (f64, f64),
    options: Option<&PyReachSet2Options>,
) -> PyResult<PyArray1Bound<'py>> {
    let problem =
        Topp2ProblemBuilder::with_constraint(&robot.inner.constraints, idx_s_interval, a_boundary)
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
#[pyo3(signature = (robot, idx_s_interval, a_boundary, options=None))]
pub fn reach_set2_backward<'py>(
    py: Python<'py>,
    robot: &PyRobot,
    idx_s_interval: (usize, usize),
    a_boundary: (f64, f64),
    options: Option<&PyReachSet2Options>,
) -> PyResult<PyArrayPair<'py>> {
    let problem =
        Topp2ProblemBuilder::with_constraint(&robot.inner.constraints, idx_s_interval, a_boundary)
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
        vec_to_ndarray1(py, &rs.a_min),
        vec_to_ndarray1(py, &rs.a_max),
    ))
}

#[pyfunction]
#[pyo3(signature = (robot, idx_s_interval, a_boundary, options=None))]
pub fn reach_set2_bidirectional<'py>(
    py: Python<'py>,
    robot: &PyRobot,
    idx_s_interval: (usize, usize),
    a_boundary: (f64, f64),
    options: Option<&PyReachSet2Options>,
) -> PyResult<PyArrayPair<'py>> {
    let problem =
        Topp2ProblemBuilder::with_constraint(&robot.inner.constraints, idx_s_interval, a_boundary)
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
        vec_to_ndarray1(py, &rs.a_min),
        vec_to_ndarray1(py, &rs.a_max),
    ))
}

// ─── COPP2-SOCP ───

#[pyfunction]
#[pyo3(signature = (robot, idx_s_interval, a_boundary, objectives, options=None))]
pub fn copp2_socp<'py>(
    py: Python<'py>,
    robot: &PyRobot,
    idx_s_interval: (usize, usize),
    a_boundary: (f64, f64),
    objectives: Vec<PyObjective>,
    options: Option<&PyClarabelOptions>,
) -> PyResult<PyArray1Bound<'py>> {
    let copp_objs = convert_objectives(&objectives);

    let problem = Copp2ProblemBuilder::new(&robot.inner, idx_s_interval, a_boundary, &copp_objs)
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

// ─── TOPP3-LP ───

#[pyfunction]
#[pyo3(signature = (robot, idx_s_start, a_linearization, a_boundary, b_boundary, options=None))]
pub fn topp3_lp<'py>(
    py: Python<'py>,
    robot: &mut PyRobot,
    idx_s_start: usize,
    a_linearization: PyReadonlyArray1<'_, f64>,
    a_boundary: (f64, f64),
    b_boundary: (f64, f64),
    options: Option<&PyClarabelOptions>,
) -> PyResult<PyTopp3Result<'py>> {
    let a_lin = ndarray1_to_vec(a_linearization);

    let problem = Topp3ProblemBuilder::with_constraint(
        &mut robot.inner.constraints,
        idx_s_start,
        &a_lin,
        a_boundary,
        b_boundary,
    )
    .build_with_linearization()
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

    let (a, b, num_st) = rust_topp3_lp(&problem, opts).map_err(copp_err_to_py)?;
    Ok((vec_to_ndarray1(py, &a), vec_to_ndarray1(py, &b), num_st))
}

// ─── TOPP3-SOCP ───

#[pyfunction]
#[pyo3(signature = (robot, idx_s_start, a_linearization, a_boundary, b_boundary, options=None))]
pub fn topp3_socp<'py>(
    py: Python<'py>,
    robot: &mut PyRobot,
    idx_s_start: usize,
    a_linearization: PyReadonlyArray1<'_, f64>,
    a_boundary: (f64, f64),
    b_boundary: (f64, f64),
    options: Option<&PyClarabelOptions>,
) -> PyResult<PyTopp3Result<'py>> {
    let a_lin = ndarray1_to_vec(a_linearization);

    let problem = Topp3ProblemBuilder::with_constraint(
        &mut robot.inner.constraints,
        idx_s_start,
        &a_lin,
        a_boundary,
        b_boundary,
    )
    .build_with_linearization()
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

    let (a, b, num_st) = rust_topp3_socp(&problem, opts).map_err(copp_err_to_py)?;
    Ok((vec_to_ndarray1(py, &a), vec_to_ndarray1(py, &b), num_st))
}

// ─── COPP3-SOCP ───

#[pyfunction]
#[pyo3(signature = (robot, idx_s_start, a_linearization, a_boundary, b_boundary, objectives, options=None))]
#[allow(clippy::too_many_arguments)]
pub fn copp3_socp<'py>(
    py: Python<'py>,
    robot: &mut PyRobot,
    idx_s_start: usize,
    a_linearization: PyReadonlyArray1<'_, f64>,
    a_boundary: (f64, f64),
    b_boundary: (f64, f64),
    objectives: Vec<PyObjective>,
    options: Option<&PyClarabelOptions>,
) -> PyResult<PyTopp3Result<'py>> {
    let a_lin = ndarray1_to_vec(a_linearization);
    let copp_objs = convert_objectives(&objectives);

    let problem = Copp3ProblemBuilder::new(
        &mut robot.inner,
        &copp_objs,
        idx_s_start,
        &a_lin,
        a_boundary,
        b_boundary,
    )
    .build_with_linearization()
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

    let result = rust_copp3_socp(&problem, opts);
    if let Some(err) = robot.take_callback_error() {
        return Err(err);
    }
    let (a, b, num_st) = result.map_err(copp_err_to_py)?;
    Ok((vec_to_ndarray1(py, &a), vec_to_ndarray1(py, &b), num_st))
}

// ─── Interpolation (TOPP2) ───

#[pyfunction]
#[pyo3(signature = (s, a, t_offset=0.0))]
pub fn s_to_t_topp2<'py>(
    py: Python<'py>,
    s: PyReadonlyArray1<'_, f64>,
    a: PyReadonlyArray1<'_, f64>,
    t_offset: f64,
) -> (f64, Bound<'py, PyArray1<f64>>) {
    let (tf, ts) = rust_s_to_t2(&ndarray1_to_vec(s), &ndarray1_to_vec(a), t_offset);
    (tf, vec_to_ndarray1(py, &ts))
}

#[pyfunction]
#[pyo3(signature = (s, a, t_s, dt=None, t_samples=None))]
pub fn t_to_s_topp2<'py>(
    py: Python<'py>,
    s: PyReadonlyArray1<'_, f64>,
    a: PyReadonlyArray1<'_, f64>,
    t_s: PyReadonlyArray1<'_, f64>,
    dt: Option<f64>,
    t_samples: Option<PyReadonlyArray1<'_, f64>>,
) -> PyResult<PyArray1Bound<'py>> {
    let sv = ndarray1_to_vec(s);
    let av = ndarray1_to_vec(a);
    let tsv = ndarray1_to_vec(t_s);

    if let Some(dt_val) = dt {
        let mode = InterpolationMode::UniformTimeGrid(0.0, dt_val, true);
        let r = rust_t_to_s2(&sv, &av, &tsv, mode);
        return Ok(vec_to_ndarray1(py, &r));
    }
    if let Some(ts) = t_samples {
        let tv = ndarray1_to_vec(ts);
        let mode = InterpolationMode::NonUniformTimeGrid(&tv);
        let r = rust_t_to_s2(&sv, &av, &tsv, mode);
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
) -> PyArray1Bound<'py> {
    let r = rust_a_to_b(&ndarray1_to_vec(s), &ndarray1_to_vec(a));
    vec_to_ndarray1(py, &r)
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
) -> (f64, Bound<'py, PyArray1<f64>>) {
    let (tf, ts) = rust_s_to_t3(
        &ndarray1_to_vec(s),
        &ndarray1_to_vec(a),
        &ndarray1_to_vec(b),
        num_stationary,
        t_offset,
    );
    (tf, vec_to_ndarray1(py, &ts))
}

#[pyfunction]
#[pyo3(signature = (s, a, b, num_stationary, t_s, dt=None, t_samples=None))]
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
) -> PyResult<PyArray1Bound<'py>> {
    let sv = ndarray1_to_vec(s);
    let av = ndarray1_to_vec(a);
    let bv = ndarray1_to_vec(b);
    let tsv = ndarray1_to_vec(t_s);

    if let Some(dt_val) = dt {
        let mode = InterpolationMode::UniformTimeGrid(0.0, dt_val, true);
        let r = rust_t_to_s3(&sv, &av, &bv, num_stationary, &tsv, mode);
        return Ok(vec_to_ndarray1(py, &r));
    }
    if let Some(ts) = t_samples {
        let tv = ndarray1_to_vec(ts);
        let mode = InterpolationMode::NonUniformTimeGrid(&tv);
        let r = rust_t_to_s3(&sv, &av, &bv, num_stationary, &tsv, mode);
        return Ok(vec_to_ndarray1(py, &r));
    }
    Err(pyo3::exceptions::PyValueError::new_err(
        "Either dt or t_samples must be provided",
    ))
}
