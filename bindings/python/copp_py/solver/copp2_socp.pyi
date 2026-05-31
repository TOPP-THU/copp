"""COPP2 SOCP solver backed by Clarabel."""

import numpy as np
from numpy.typing import NDArray

from .._native import (
    ClarabelOptions as Options,
    Copp2Problem as Problem,
    Copp2SocpResult as Result,
)

def solve(problem: Problem, options: Options | None = None) -> NDArray[np.float64]:
    """Solve a COPP2 problem with the Clarabel SOCP backend."""
    ...

def solve_expert(problem: Problem, options: Options | None = None) -> Result:
    """Solve COPP2-SOCP and return raw Clarabel diagnostics."""
    ...

__all__: list[str]
