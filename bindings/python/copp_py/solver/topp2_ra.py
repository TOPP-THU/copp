r"""TOPP2 reachability-analysis solver."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from .. import _native

Problem = _native.Topp2Problem
Options = _native.ReachSet2Options
ReachSet = _native.ReachSet2


def solve(problem: Problem, options: Options | None = None) -> NDArray[np.float64]:
    """Solve a TOPP2 problem with reachability analysis."""
    return _native.topp2_ra(problem, options)


__all__ = ["Problem", "Options", "ReachSet", "solve"]
