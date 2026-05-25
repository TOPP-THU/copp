//! Numerical kernels: LP solvers and helper predicates.
//!
//! # Method identity
//! - `general`: vector products and convexity predicates.
//! - `lp`: incremental LP solvers in 1D/2D/3D and warm-start helpers.

mod general;
mod lp;

pub(crate) use general::*;
pub(crate) use lp::*;
