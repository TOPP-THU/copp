r"""TOPP3 linear-objective approximation backed by Clarabel."""

from __future__ import annotations

from .. import _native

Problem = _native.Topp3Problem
Options = _native.ClarabelOptions
Profile = _native.Profile3rd
Result = _native.Copp3ClarabelResult


def solve(problem: Problem, options: Options | None = None) -> Profile:
    """Solve a TOPP3 problem with the Clarabel LP backend."""
    return _native.topp3_lp(problem, options)


def solve_expert(problem: Problem, options: Options | None = None) -> Result:
    """Solve TOPP3-LP and return raw Clarabel diagnostics."""
    return _native.topp3_lp_expert(problem, options)


__all__ = ["Problem", "Options", "Profile", "Result", "solve", "solve_expert"]
