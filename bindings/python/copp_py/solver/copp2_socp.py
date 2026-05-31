r"""COPP2 SOCP solver backed by Clarabel."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from .. import _native

Problem = _native.Copp2Problem
Options = _native.ClarabelOptions
Result = _native.Copp2SocpResult


def solve(problem: Problem, options: Options | None = None) -> NDArray[np.float64]:
    """Solve a COPP2 problem with the Clarabel SOCP backend."""
    return _native.copp2_socp(problem, options)


def solve_expert(problem: Problem, options: Options | None = None) -> Result:
    """Solve COPP2-SOCP and return raw Clarabel diagnostics."""
    return _native.copp2_socp_expert(problem, options)


__all__ = ["Problem", "Options", "Result", "solve", "solve_expert"]
