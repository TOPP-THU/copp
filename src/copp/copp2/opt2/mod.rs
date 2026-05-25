//! Optimization backends for second-order path parameterization (Clarabel-based).
//!
//! # Method identity
//! This module provides optimization formulations for:
//! - **Time-Optimal Path Parameterization (TOPP2)**,
//! - **Convex-Objective Path Parameterization (COPP2)**.
//!
//! # Contents
//! - `clarabel_constraints`: shared standard TOPP2 constraint assembly.
//! - [`copp2_socp`](crate::solver::copp2_socp::copp2_socp): COPP2 backend via **Second-Order Cone Programming (SOCP)**
//!   with normal/expert/core layering.

use clarabel::solver::{DefaultSolution, LinearSolverInfo};

pub(crate) mod clarabel_constraints;
pub(crate) mod copp2_socp;

/// Clarabel expert result for second-order optimization backends.
///
/// This extends the public expert tuple `(Option<Vec<f64>>, DefaultSolution<f64>)`
/// with solver-side metadata that Clarabel stores outside [`DefaultSolution`](clarabel::solver::DefaultSolution).
/// The high-level `result` field is `Some(a)` only when the solver status is
/// accepted by the provided [`ClarabelOptions`](crate::solver::copp2_socp::ClarabelOptions).
pub struct ClarabelExpertInfor2nd {
    /// Accepted second-order profile `a = ds/dt ^ 2`, or `None` when the solver
    /// status is not accepted.
    pub result: Option<Vec<f64>>,
    /// Raw Clarabel solution, including primal/dual/slack vectors and status.
    pub solution: DefaultSolution<f64>,
    /// Clarabel linear-solver metadata captured before the solver object is
    /// consumed.
    pub linsolver: LinearSolverInfo,
}
