r"""COPP3 SOCP solver backed by Clarabel."""

from __future__ import annotations

from .. import _native

Problem = _native.Copp3Problem
Options = _native.ClarabelOptions
Profile = _native.Profile3rd
Result = _native.Copp3ClarabelResult


def solve(problem: Problem, options: Options | None = None) -> Profile:
    """Solve a COPP3 problem with the Clarabel SOCP backend."""
    return _native.copp3_socp(problem, options)


def solve_expert(problem: Problem, options: Options | None = None) -> Result:
    """Solve COPP3-SOCP and return raw Clarabel diagnostics."""
    return _native.copp3_socp_expert(problem, options)


__all__ = ["Problem", "Options", "Profile", "Result", "solve", "solve_expert"]
