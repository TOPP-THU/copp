//! Shared numerical math kernels.
//!
//! # Method identity
//! This namespace contains reusable low-level utilities used by TOPP/COPP
//! implementations:
//! - `numerical`: LP solvers and scalar helper routines.
//!
//! # Export policy
//! Submodules are crate-private and consumed by higher-level solver layers.

pub(crate) mod numerical;
