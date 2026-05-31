"""TOPP3 SOCP solver backed by Clarabel."""

from .._native import (
    ClarabelOptions as Options,
    Copp3ClarabelResult as Result,
    Profile3rd as Profile,
    Topp3Problem as Problem,
)

def solve(problem: Problem, options: Options | None = None) -> Profile:
    """Solve a TOPP3 problem with the Clarabel SOCP backend."""
    ...

def solve_expert(problem: Problem, options: Options | None = None) -> Result:
    """Solve TOPP3-SOCP and return raw Clarabel diagnostics."""
    ...

__all__: list[str]
