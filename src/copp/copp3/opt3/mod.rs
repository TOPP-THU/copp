//! Optimization backends for third-order path parameterization (Clarabel-based).
//!
//! # Method identity
//! This module provides optimization formulations for third-order problems:
//! - **TOPP3** (time-optimal),
//! - **COPP3** (convex-objective).
//!
//! # Contents
//! - `clarabel_constraints`: shared standard TOPP3/COPP3 constraint assembly.
//! - [`topp3_lp`](crate::solver::topp3_lp::topp3_lp): internal LP baseline for regression comparison.
//! - [`topp3_socp`](crate::solver::topp3_socp::topp3_socp): TOPP3 SOCP backend.
//! - [`copp3_socp`](crate::solver::copp3_socp::copp3_socp): COPP3 SOCP backend with normal/expert/core layering.

use clarabel::solver::{DefaultSolution, LinearSolverInfo};

use crate::copp::copp3::Topp3Profile;

pub(crate) mod clarabel_constraints;
pub(crate) mod copp3_socp;
pub(crate) mod topp3_lp;
pub(crate) mod topp3_socp;

/// Clarabel expert result for third-order optimization backends.
///
/// This extends the public expert tuple `(Option<Topp3Profile>, DefaultSolution<f64>)`
/// with solver-side metadata that Clarabel stores outside [`DefaultSolution`](clarabel::solver::DefaultSolution).
/// The high-level `result` field is `Some(Topp3Profile { .. })` only when
/// the solver status is accepted by the provided [`ClarabelOptions`](crate::solver::copp2_socp::ClarabelOptions).
pub struct ClarabelExpertInfor3rd {
    /// Accepted third-order profile, or `None` when
    /// the solver status is not accepted.
    pub result: Option<Topp3Profile>,
    /// Raw Clarabel solution, including primal/dual/slack vectors and status.
    pub solution: DefaultSolution<f64>,
    /// Clarabel linear-solver metadata captured before the solver object is
    /// consumed.
    pub linsolver: LinearSolverInfo,
}
