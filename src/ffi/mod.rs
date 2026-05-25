//! Foreign-function interfaces for COPP.
//!
//! Language-specific ABI surfaces live in submodules.  The C ABI is currently
//! the only implemented surface and is re-exported here for compatibility with
//! existing internal paths.

pub mod c;

pub use c::*;
