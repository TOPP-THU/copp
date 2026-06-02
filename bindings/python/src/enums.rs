//! PyO3 enum types mirroring the C ABI's stable enums.
//!
//! These replace the previous string-typed parameters (verbosity, solver
//! status, direct solve method) so the Python surface matches the C ABI.

use clarabel::solver::SolverStatus;
use copp::diag::Verbosity as RustVerbosity;
use pyo3::prelude::*;

/// Diagnostic verbosity level. Mirrors `copp::diag::Verbosity` and the C ABI
/// `CoppVerbosity`.
#[pyclass(eq, eq_int, hash, frozen, name = "Verbosity")]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub enum Verbosity {
    Silent = 0,
    Summary = 1,
    Debug = 2,
    Trace = 3,
}

impl From<Verbosity> for RustVerbosity {
    fn from(v: Verbosity) -> Self {
        match v {
            Verbosity::Silent => RustVerbosity::Silent,
            Verbosity::Summary => RustVerbosity::Summary,
            Verbosity::Debug => RustVerbosity::Debug,
            Verbosity::Trace => RustVerbosity::Trace,
        }
    }
}

impl From<RustVerbosity> for Verbosity {
    fn from(v: RustVerbosity) -> Self {
        match v {
            RustVerbosity::Silent => Verbosity::Silent,
            RustVerbosity::Summary => Verbosity::Summary,
            RustVerbosity::Debug => Verbosity::Debug,
            RustVerbosity::Trace => Verbosity::Trace,
        }
    }
}

/// Final Clarabel solver status. Mirrors the C ABI `CoppClarabelSolverStatus`.
#[pyclass(eq, eq_int, hash, frozen, name = "ClarabelSolverStatus")]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub enum ClarabelSolverStatus {
    Unsolved = 0,
    Solved = 1,
    PrimalInfeasible = 2,
    DualInfeasible = 3,
    AlmostSolved = 4,
    AlmostPrimalInfeasible = 5,
    AlmostDualInfeasible = 6,
    MaxIterations = 7,
    MaxTime = 8,
    NumericalError = 9,
    InsufficientProgress = 10,
    CallbackTerminated = 11,
}

impl From<SolverStatus> for ClarabelSolverStatus {
    fn from(status: SolverStatus) -> Self {
        match status {
            SolverStatus::Unsolved => Self::Unsolved,
            SolverStatus::Solved => Self::Solved,
            SolverStatus::PrimalInfeasible => Self::PrimalInfeasible,
            SolverStatus::DualInfeasible => Self::DualInfeasible,
            SolverStatus::AlmostSolved => Self::AlmostSolved,
            SolverStatus::AlmostPrimalInfeasible => Self::AlmostPrimalInfeasible,
            SolverStatus::AlmostDualInfeasible => Self::AlmostDualInfeasible,
            SolverStatus::MaxIterations => Self::MaxIterations,
            SolverStatus::MaxTime => Self::MaxTime,
            SolverStatus::NumericalError => Self::NumericalError,
            SolverStatus::InsufficientProgress => Self::InsufficientProgress,
            SolverStatus::CallbackTerminated => Self::CallbackTerminated,
        }
    }
}

/// Clarabel direct linear-solver method. Mirrors the C ABI
/// `CoppClarabelDirectSolveMethod`.
#[pyclass(eq, eq_int, hash, frozen, name = "DirectSolveMethod")]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub enum DirectSolveMethod {
    Auto = 0,
    Qdldl = 1,
    Faer = 2,
    Mkl = 3,
    Panua = 4,
}

impl DirectSolveMethod {
    /// Map a Clarabel `direct_solve_method` string to the stable enum.
    pub fn from_clarabel_name(name: &str) -> Self {
        match name {
            "qdldl" => Self::Qdldl,
            "faer" => Self::Faer,
            "mkl" => Self::Mkl,
            "panua" => Self::Panua,
            _ => Self::Auto,
        }
    }

    /// Map the enum back to the Clarabel `direct_solve_method` string.
    pub fn as_clarabel_name(self) -> &'static str {
        match self {
            Self::Auto => "auto",
            Self::Qdldl => "qdldl",
            Self::Faer => "faer",
            Self::Mkl => "mkl",
            Self::Panua => "panua",
        }
    }
}
