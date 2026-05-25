//! Dynamic-programming family backends for second-order path parameterization.
//!
//! # Method identity
//! This module provides **Reachability Analysis (RA)** style solvers for
//! **Time-Optimal Path Parameterization (TOPP2)**.
//!
//! # Contents
//! - [`reach_set2`](crate::solver::reach_set2): bidirectional reachable-set construction.
//! - [`topp2_ra`](crate::solver::topp2_ra::topp2_ra): **Reachability Analysis (RA)** solver for TOPP2.

pub(crate) mod reach_set2;
pub(crate) mod topp2_ra;
