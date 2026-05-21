mod callback_error;
mod convert;
mod errors;
mod objectives;
mod options;
mod path;
mod robot;
mod solver;

use pyo3::prelude::*;

#[pymodule]
fn _copp(m: &Bound<'_, PyModule>) -> PyResult<()> {
    errors::register(m)?;

    m.add_class::<path::PyJet3>()?;
    m.add_class::<path::PySplineConfig>()?;
    m.add_class::<path::PyPathDerivatives>()?;
    m.add_class::<path::PyPath>()?;
    m.add_function(wrap_pyfunction!(path::sin, m)?)?;
    m.add_function(wrap_pyfunction!(path::cos, m)?)?;
    m.add_function(wrap_pyfunction!(path::exp, m)?)?;
    m.add_function(wrap_pyfunction!(path::ln, m)?)?;
    m.add_function(wrap_pyfunction!(path::sqrt, m)?)?;
    m.add_function(wrap_pyfunction!(path::powi, m)?)?;

    m.add_class::<robot::PyRobot>()?;
    m.add_class::<options::PyReachSet2Options>()?;
    m.add_class::<options::PyClarabelOptions>()?;
    m.add_class::<objectives::PyObjective>()?;

    // Solvers
    m.add_function(wrap_pyfunction!(solver::topp2_ra, m)?)?;
    m.add_function(wrap_pyfunction!(solver::reach_set2_backward, m)?)?;
    m.add_function(wrap_pyfunction!(solver::reach_set2_bidirectional, m)?)?;
    m.add_function(wrap_pyfunction!(solver::copp2_socp, m)?)?;
    m.add_function(wrap_pyfunction!(solver::topp3_lp, m)?)?;
    m.add_function(wrap_pyfunction!(solver::topp3_socp, m)?)?;
    m.add_function(wrap_pyfunction!(solver::copp3_socp, m)?)?;

    // Interpolation
    m.add_function(wrap_pyfunction!(solver::s_to_t_topp2, m)?)?;
    m.add_function(wrap_pyfunction!(solver::t_to_s_topp2, m)?)?;
    m.add_function(wrap_pyfunction!(solver::a_to_b_topp2, m)?)?;
    m.add_function(wrap_pyfunction!(solver::s_to_t_topp3, m)?)?;
    m.add_function(wrap_pyfunction!(solver::t_to_s_topp3, m)?)?;

    Ok(())
}
