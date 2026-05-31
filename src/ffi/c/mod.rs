//! C ABI surface for COPP.
//!
//! This module exposes only C-compatible `#[repr(C)]` types, opaque handles,
//! and `extern "C"` functions.  Higher-level implementation details stay behind this
//! boundary.

pub mod core;
pub mod formulation;
pub mod interpolation;
pub mod path;
pub mod robot;
pub mod solver;

pub use core::{
    CoppMatrixF64, CoppMatrixLayout, CoppMatrixViewF64, CoppSliceF64, CoppSliceMutF64, CoppStatus,
    CoppVecF64, CoppVecUsize, CoppVerbosity,
};
pub use formulation::{
    Copp2Problem, Copp3Problem, CoppObjective, CoppObjectiveKind, CoppProfile3rd, Topp2Problem,
    Topp3Problem,
};
pub use path::{
    CoppPath, CoppPathEvaluate2ndFn, CoppPathEvaluate3rdFn, CoppPathOptions,
    CoppPathOutOfRangeMode, CoppPathParametrization,
};
pub use robot::{CoppInverseDynamicsFn, CoppRobot};
pub use solver::{
    Copp2SocpResult, Copp3SocpResult, CoppClarabelDirectSolveMethod,
    CoppClarabelLinearSolverInfo, CoppClarabelOptions, CoppClarabelSettings,
    CoppClarabelSolverStatus, CoppReachSet2Result, Topp2RaOptions,
};
