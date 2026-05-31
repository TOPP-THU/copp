//! Python wrappers for COPP objective descriptors.
//!
//! Python objective objects own all array data. Solver wrappers convert them
//! into borrowed [`CoppObjective`](crate::copp::CoppObjective) values only for
//! the duration of one Rust solver call.

use crate::copp::CoppObjective;
use numpy::{PyArray1, PyReadonlyArray1};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::{PyAnyMethods, PyDict};

/// Register objective-related classes on the native [`PyModule`].
pub(crate) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyTimeObjective>()?;
    m.add_class::<PyLinearObjective>()?;
    m.add_class::<PyThermalEnergyObjective>()?;
    m.add_class::<PyTotalVariationTorqueObjective>()?;
    Ok(())
}

/// Python-owned objective descriptor used internally by COPP solver wrappers.
#[derive(Clone)]
pub(crate) enum OwnedObjective {
    /// Time objective.
    Time {
        /// Objective weight.
        weight: f64,
    },
    /// Linear objective over second- or third-order profile variables.
    Linear {
        /// Objective weight.
        weight: f64,
        /// Node-based coefficient for `a`.
        alpha: Vec<f64>,
        /// Segment- or node-based coefficient for `b`.
        beta: Vec<f64>,
    },
    /// Thermal-energy objective.
    ThermalEnergy {
        /// Objective weight.
        weight: f64,
        /// Per-axis torque normalization.
        normalize: Vec<f64>,
    },
    /// Total-variation torque objective.
    TotalVariationTorque {
        /// Objective weight.
        weight: f64,
        /// Per-axis torque normalization.
        normalize: Vec<f64>,
    },
}

impl OwnedObjective {
    /// Borrow this owned descriptor as a Rust [`CoppObjective`].
    pub(crate) fn as_rust(&self) -> CoppObjective<'_> {
        match self {
            Self::Time { weight } => CoppObjective::Time(*weight),
            Self::Linear {
                weight,
                alpha,
                beta,
            } => CoppObjective::Linear(*weight, alpha.as_slice(), beta.as_slice()),
            Self::ThermalEnergy { weight, normalize } => {
                CoppObjective::ThermalEnergy(*weight, normalize.as_slice())
            }
            Self::TotalVariationTorque { weight, normalize } => {
                CoppObjective::TotalVariationTorque(*weight, normalize.as_slice())
            }
        }
    }
}

/// Time objective.
#[pyclass(name = "Time", module = "copp_py._native")]
pub(crate) struct PyTimeObjective {
    /// Objective weight.
    weight: f64,
}

#[pymethods]
impl PyTimeObjective {
    /// Construct a time objective.
    #[new]
    #[pyo3(signature = (weight), text_signature = "(weight)")]
    fn new(weight: f64) -> Self {
        Self { weight }
    }

    /// Return the objective weight.
    #[getter]
    fn weight(&self) -> f64 {
        self.weight
    }

    /// Return a compact representation for interactive Python sessions.
    fn __repr__(&self) -> String {
        format!("Time(weight={})", self.weight)
    }
}

/// Linear objective over profile variables.
///
/// In COPP2, `alpha` is node-based with length `s_len`, while `beta` is
/// interval-based with length `s_len - 1`. In COPP3, both arrays are node-based
/// with length `s_len`; Rust objective validation enforces the active solver's
/// length contract.
#[pyclass(name = "Linear", module = "copp_py._native")]
pub(crate) struct PyLinearObjective {
    /// Objective weight.
    weight: f64,
    /// Node-based coefficient for `a`.
    alpha: Vec<f64>,
    /// Segment- or node-based coefficient for `b`.
    beta: Vec<f64>,
}

#[pymethods]
impl PyLinearObjective {
    /// Construct a linear objective.
    #[new]
    #[pyo3(signature = (weight, alpha, beta), text_signature = "(weight, alpha, beta)")]
    fn new(weight: f64, alpha: &Bound<'_, PyAny>, beta: &Bound<'_, PyAny>) -> PyResult<Self> {
        Ok(Self {
            weight,
            alpha: array_like_to_vec("alpha", alpha)?,
            beta: array_like_to_vec("beta", beta)?,
        })
    }

    /// Return the objective weight.
    #[getter]
    fn weight(&self) -> f64 {
        self.weight
    }

    /// Return the `a` coefficient array.
    #[getter]
    fn alpha<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        PyArray1::from_vec(py, self.alpha.clone())
    }

    /// Return the `b` coefficient array.
    #[getter]
    fn beta<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        PyArray1::from_vec(py, self.beta.clone())
    }

    /// Return a compact representation for interactive Python sessions.
    fn __repr__(&self) -> String {
        format!(
            "Linear(weight={}, alpha_len={}, beta_len={})",
            self.weight,
            self.alpha.len(),
            self.beta.len()
        )
    }
}

/// Thermal-energy objective.
#[pyclass(name = "ThermalEnergy", module = "copp_py._native")]
pub(crate) struct PyThermalEnergyObjective {
    /// Objective weight.
    weight: f64,
    /// Per-axis torque normalization.
    normalize: Vec<f64>,
}

#[pymethods]
impl PyThermalEnergyObjective {
    /// Construct a thermal-energy objective.
    #[new]
    #[pyo3(signature = (weight, normalize), text_signature = "(weight, normalize)")]
    fn new(weight: f64, normalize: &Bound<'_, PyAny>) -> PyResult<Self> {
        Ok(Self {
            weight,
            normalize: array_like_to_vec("normalize", normalize)?,
        })
    }

    /// Return the objective weight.
    #[getter]
    fn weight(&self) -> f64 {
        self.weight
    }

    /// Return the per-axis torque normalization.
    #[getter]
    fn normalize<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        PyArray1::from_vec(py, self.normalize.clone())
    }

    /// Return a compact representation for interactive Python sessions.
    fn __repr__(&self) -> String {
        format!(
            "ThermalEnergy(weight={}, normalize_len={})",
            self.weight,
            self.normalize.len()
        )
    }
}

/// Total-variation torque objective.
#[pyclass(name = "TotalVariationTorque", module = "copp_py._native")]
pub(crate) struct PyTotalVariationTorqueObjective {
    /// Objective weight.
    weight: f64,
    /// Per-axis torque normalization.
    normalize: Vec<f64>,
}

#[pymethods]
impl PyTotalVariationTorqueObjective {
    /// Construct a total-variation torque objective.
    #[new]
    #[pyo3(signature = (weight, normalize), text_signature = "(weight, normalize)")]
    fn new(weight: f64, normalize: &Bound<'_, PyAny>) -> PyResult<Self> {
        Ok(Self {
            weight,
            normalize: array_like_to_vec("normalize", normalize)?,
        })
    }

    /// Return the objective weight.
    #[getter]
    fn weight(&self) -> f64 {
        self.weight
    }

    /// Return the per-axis torque normalization.
    #[getter]
    fn normalize<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        PyArray1::from_vec(py, self.normalize.clone())
    }

    /// Return a compact representation for interactive Python sessions.
    fn __repr__(&self) -> String {
        format!(
            "TotalVariationTorque(weight={}, normalize_len={})",
            self.weight,
            self.normalize.len()
        )
    }
}

/// Parse one objective or an iterable of objectives.
pub(crate) fn parse_objectives(objectives: &Bound<'_, PyAny>) -> PyResult<Vec<OwnedObjective>> {
    if let Some(objective) = parse_objective_item(objectives)? {
        return Ok(vec![objective]);
    }

    let iter = objectives.try_iter().map_err(|_| {
        PyValueError::new_err(
            "`objectives` must be an objective object, an objective dict, or an iterable of them",
        )
    })?;
    let mut parsed = Vec::new();
    for item in iter {
        let item = item?;
        let objective = parse_objective_item(&item)?.ok_or_else(|| {
            PyValueError::new_err(
                "each objective must be Time, Linear, ThermalEnergy, TotalVariationTorque, or a dict",
            )
        })?;
        parsed.push(objective);
    }
    Ok(parsed)
}

/// Parse a single objective item.
fn parse_objective_item(obj: &Bound<'_, PyAny>) -> PyResult<Option<OwnedObjective>> {
    if let Ok(value) = obj.extract::<PyRef<'_, PyTimeObjective>>() {
        return Ok(Some(OwnedObjective::Time {
            weight: value.weight,
        }));
    }
    if let Ok(value) = obj.extract::<PyRef<'_, PyLinearObjective>>() {
        return Ok(Some(OwnedObjective::Linear {
            weight: value.weight,
            alpha: value.alpha.clone(),
            beta: value.beta.clone(),
        }));
    }
    if let Ok(value) = obj.extract::<PyRef<'_, PyThermalEnergyObjective>>() {
        return Ok(Some(OwnedObjective::ThermalEnergy {
            weight: value.weight,
            normalize: value.normalize.clone(),
        }));
    }
    if let Ok(value) = obj.extract::<PyRef<'_, PyTotalVariationTorqueObjective>>() {
        return Ok(Some(OwnedObjective::TotalVariationTorque {
            weight: value.weight,
            normalize: value.normalize.clone(),
        }));
    }
    if let Ok(dict) = obj.cast::<PyDict>() {
        return parse_objective_dict(dict).map(Some);
    }
    Ok(None)
}

/// Parse a dict objective descriptor.
fn parse_objective_dict(dict: &Bound<'_, PyDict>) -> PyResult<OwnedObjective> {
    let kind = required_item(dict, "kind")?;
    let kind = normalize_token(
        kind.extract::<&str>()
            .map_err(|_| PyValueError::new_err("objective dict field `kind` must be a string"))?,
    );
    let weight = required_item(dict, "weight")?
        .extract::<f64>()
        .map_err(|_| PyValueError::new_err("objective dict field `weight` must be a float"))?;

    match kind.as_str() {
        "time" => Ok(OwnedObjective::Time { weight }),
        "linear" => Ok(OwnedObjective::Linear {
            weight,
            alpha: array_like_to_vec("alpha", &required_item(dict, "alpha")?)?,
            beta: array_like_to_vec("beta", &required_item(dict, "beta")?)?,
        }),
        "thermal_energy" | "thermalenergy" | "thermal" => Ok(OwnedObjective::ThermalEnergy {
            weight,
            normalize: array_like_to_vec("normalize", &required_item(dict, "normalize")?)?,
        }),
        "total_variation_torque" | "totalvariationtorque" | "tv_torque" | "tvtorque" => {
            Ok(OwnedObjective::TotalVariationTorque {
                weight,
                normalize: array_like_to_vec("normalize", &required_item(dict, "normalize")?)?,
            })
        }
        _ => Err(PyValueError::new_err(
            "objective dict field `kind` must be one of \"time\", \"linear\", \"thermal_energy\", or \"total_variation_torque\"",
        )),
    }
}

/// Return a required dict item.
fn required_item<'py>(dict: &Bound<'py, PyDict>, key: &str) -> PyResult<Bound<'py, PyAny>> {
    dict.get_item(key)?.ok_or_else(|| {
        PyValueError::new_err(format!("objective dict is missing required field `{key}`"))
    })
}

/// Convert a Python array-like value into an owned one-dimensional `float64` vector.
fn array_like_to_vec(name: &str, obj: &Bound<'_, PyAny>) -> PyResult<Vec<f64>> {
    let py = obj.py();
    let numpy = py.import("numpy")?;
    let dtype = numpy.getattr("float64")?;
    let array_obj = numpy.getattr("ascontiguousarray")?.call1((obj, dtype))?;
    let array = array_obj
        .extract::<PyReadonlyArray1<'_, f64>>()
        .map_err(|_| {
            PyValueError::new_err(format!(
                "`{name}` must be convertible to a one-dimensional float64 NumPy array"
            ))
        })?;
    let values = array.as_slice().map_err(|_| {
        PyValueError::new_err(format!(
            "`{name}` must be a contiguous one-dimensional float64 array"
        ))
    })?;
    Ok(values.to_vec())
}

/// Normalize user-facing objective kind strings for lenient matching.
fn normalize_token(text: &str) -> String {
    text.trim().to_ascii_lowercase().replace('-', "_")
}
