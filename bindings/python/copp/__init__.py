"""copp: Convex-objective path parameterization for robotic trajectory planning."""

from copp._copp import (
    # Exceptions
    CoppError,
    PathError,
    ConstraintError,
    InfeasibleError,
    InvalidInputError,
    # Path
    Jet3,
    SplineConfig,
    PathDerivatives,
    Path,
    sin,
    cos,
    exp,
    ln,
    sqrt,
    powi,
    # Robot
    Robot,
    # Enums
    Verbosity,
    ClarabelSolverStatus,
    DirectSolveMethod,
    # Options
    ReachSet2Options,
    ClarabelSettings,
    ClarabelOptions,
    set_verbosity_output,
    set_verbosity_log_file,
    verbosity_output,
    # Objectives
    Objective,
    # Solver diagnostics
    Topp3Profile,
    ClarabelSolution,
    LinearSolverInfo,
    # Solvers
    topp2_ra,
    reach_set2_backward,
    reach_set2_bidirectional,
    copp2_socp,
    copp2_socp_expert,
    topp3_lp,
    topp3_lp_expert,
    topp3_socp,
    topp3_socp_expert,
    copp3_socp,
    copp3_socp_expert,
    # Interpolation
    s_to_t_topp2,
    t_to_s_topp2,
    a_to_b_topp2,
    s_to_t_topp3,
    t_to_s_topp3,
    force_positive_a_3rd,
)

__version__ = "0.2.0"

__all__ = [
    "CoppError", "PathError", "ConstraintError",
    "InfeasibleError", "InvalidInputError",
    "Jet3", "SplineConfig", "PathDerivatives", "Path",
    "sin", "cos", "exp", "ln", "sqrt", "powi",
    "Robot",
    "Verbosity", "ClarabelSolverStatus", "DirectSolveMethod",
    "ReachSet2Options", "ClarabelSettings", "ClarabelOptions",
    "set_verbosity_output", "set_verbosity_log_file", "verbosity_output",
    "Objective",
    "Topp3Profile", "ClarabelSolution", "LinearSolverInfo",
    "topp2_ra", "reach_set2_backward", "reach_set2_bidirectional",
    "copp2_socp", "copp2_socp_expert",
    "topp3_lp", "topp3_lp_expert",
    "topp3_socp", "topp3_socp_expert",
    "copp3_socp", "copp3_socp_expert",
    "s_to_t_topp2", "t_to_s_topp2", "a_to_b_topp2",
    "s_to_t_topp3", "t_to_s_topp3", "force_positive_a_3rd",
]
