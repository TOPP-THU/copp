use pyo3::prelude::*;

#[pyclass(name = "Objective")]
#[derive(Clone)]
pub struct PyObjective {
    pub kind: ObjectiveKind,
}

#[derive(Clone)]
pub enum ObjectiveKind {
    Time(f64),
    ThermalEnergy(f64, Vec<f64>),
    TotalVariationTorque(f64, Vec<f64>),
    Linear(f64, Vec<f64>, Vec<f64>),
}

#[pymethods]
impl PyObjective {
    #[staticmethod]
    #[pyo3(signature = (weight=1.0))]
    fn time(weight: f64) -> Self {
        Self {
            kind: ObjectiveKind::Time(weight),
        }
    }

    #[staticmethod]
    fn thermal_energy(weight: f64, normalize: Vec<f64>) -> Self {
        Self {
            kind: ObjectiveKind::ThermalEnergy(weight, normalize),
        }
    }

    #[staticmethod]
    fn total_variation_torque(weight: f64, normalize: Vec<f64>) -> Self {
        Self {
            kind: ObjectiveKind::TotalVariationTorque(weight, normalize),
        }
    }

    #[staticmethod]
    fn linear(weight: f64, alpha: Vec<f64>, beta: Vec<f64>) -> Self {
        Self {
            kind: ObjectiveKind::Linear(weight, alpha, beta),
        }
    }
}
