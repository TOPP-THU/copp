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
    # Options
    ReachSet2Options,
    ClarabelOptions,
    # Objectives
    Objective,
    # Solvers
    topp2_ra,
    reach_set2_backward,
    reach_set2_bidirectional,
    copp2_socp,
    topp3_lp,
    topp3_socp,
    copp3_socp,
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
    "ReachSet2Options", "ClarabelOptions",
    "Objective",
    "topp2_ra", "reach_set2_backward", "reach_set2_bidirectional",
    "copp2_socp", "topp3_lp", "topp3_socp", "copp3_socp",
    "s_to_t_topp2", "t_to_s_topp2", "a_to_b_topp2",
    "s_to_t_topp3", "t_to_s_topp3", "force_positive_a_3rd",
]
