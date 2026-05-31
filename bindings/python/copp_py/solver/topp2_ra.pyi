"""TOPP2 reachability-analysis solver."""

import numpy as np
from numpy.typing import NDArray

from .._native import (
    ReachSet2 as ReachSet,
    ReachSet2Options as Options,
    Topp2Problem as Problem,
)

def solve(problem: Problem, options: Options | None = None) -> NDArray[np.float64]:
    """Solve a TOPP2 problem with reachability analysis."""
    ...

__all__: list[str]
