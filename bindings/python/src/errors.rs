use copp::diag::{ConstraintError, CoppError, PathError};
use pyo3::create_exception;
use pyo3::prelude::*;

create_exception!(copp, PyCoppError, pyo3::exceptions::PyException);
create_exception!(copp, PyPathError, PyCoppError);
create_exception!(copp, PyConstraintError, PyCoppError);
create_exception!(copp, PyInfeasibleError, PyCoppError);
create_exception!(copp, PyInvalidInputError, PyCoppError);

pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("CoppError", m.py().get_type::<PyCoppError>())?;
    m.add("PathError", m.py().get_type::<PyPathError>())?;
    m.add("ConstraintError", m.py().get_type::<PyConstraintError>())?;
    m.add("InfeasibleError", m.py().get_type::<PyInfeasibleError>())?;
    m.add(
        "InvalidInputError",
        m.py().get_type::<PyInvalidInputError>(),
    )?;
    Ok(())
}

pub fn copp_err_to_py(e: CoppError) -> PyErr {
    match e {
        CoppError::PathError(pe) => PyPathError::new_err(pe.to_string()),
        CoppError::ConstraintError(ce) => PyConstraintError::new_err(ce.to_string()),
        CoppError::Infeasible(ctx, msg) => PyInfeasibleError::new_err(format!("{ctx}: {msg}")),
        CoppError::InvalidInput(ctx, msg) | CoppError::InvalidOptions(ctx, msg) => {
            PyInvalidInputError::new_err(format!("{ctx}: {msg}"))
        }
        other => PyCoppError::new_err(other.to_string()),
    }
}

pub fn path_err_to_py(e: PathError) -> PyErr {
    PyPathError::new_err(e.to_string())
}

pub fn constraint_err_to_py(e: ConstraintError) -> PyErr {
    PyConstraintError::new_err(e.to_string())
}
