"""COPP3 SOCP solver backed by Clarabel."""

from .._native import (
    ClarabelOptions as Options,
    Copp3ClarabelResult as Result,
    Copp3Problem as Problem,
    Profile3rd as Profile,
)

def solve(problem: Problem, options: Options | None = None) -> Profile:
    """Solve a COPP3 problem with the Clarabel SOCP backend."""
    ...

def solve_expert(problem: Problem, options: Options | None = None) -> Result:
    """Solve COPP3-SOCP and return raw Clarabel diagnostics."""
    ...

__all__: list[str]
