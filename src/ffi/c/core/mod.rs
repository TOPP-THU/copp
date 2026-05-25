//! Core C ABI types and utility functions.

pub mod status;
pub mod types;
pub mod verbosity;
pub mod version;

pub use status::CoppStatus;
pub use types::{
    CoppMatrixF64, CoppMatrixLayout, CoppMatrixViewF64, CoppSliceF64, CoppSliceMutF64, CoppVecF64,
    CoppVecUsize,
};
pub use verbosity::CoppVerbosity;
