//! Shared C ABI verbosity types.

use crate::diag::Verbosity;

/// C ABI verbosity level for solver diagnostics.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CoppVerbosity {
    /// No log will be emitted.
    Silent = 0,
    /// Only summary information will be emitted.
    Summary = 1,
    /// Detailed debug information will be emitted.
    Debug = 2,
    /// Very detailed trace information will be emitted.
    Trace = 3,
}

impl From<CoppVerbosity> for Verbosity {
    fn from(verbosity: CoppVerbosity) -> Self {
        match verbosity {
            CoppVerbosity::Silent => Self::Silent,
            CoppVerbosity::Summary => Self::Summary,
            CoppVerbosity::Debug => Self::Debug,
            CoppVerbosity::Trace => Self::Trace,
        }
    }
}

impl From<Verbosity> for CoppVerbosity {
    fn from(verbosity: Verbosity) -> Self {
        match verbosity {
            Verbosity::Silent => Self::Silent,
            Verbosity::Summary => Self::Summary,
            Verbosity::Debug => Self::Debug,
            Verbosity::Trace => Self::Trace,
        }
    }
}
