//! C ABI wrappers for solver entry points.

pub mod copp2;
pub mod copp3;
pub mod topp2;
pub mod topp3;

pub use copp2::socp::{
    Copp2SocpResult, CoppClarabelDirectSolveMethod, CoppClarabelLinearSolverInfo,
    CoppClarabelOptions, CoppClarabelSettings, CoppClarabelSolverStatus,
};
pub use copp3::socp::Copp3SocpResult;
pub use topp2::ra::Topp2RaOptions;
pub use topp2::reach_set::CoppReachSet2Result;
