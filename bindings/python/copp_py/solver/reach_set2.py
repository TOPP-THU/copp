r"""TOPP2 reachable-set construction."""

from __future__ import annotations

from .. import _native

Problem = _native.Topp2Problem
Options = _native.ReachSet2Options
ReachSet = _native.ReachSet2


def backward(problem: Problem, options: Options | None = None) -> ReachSet:
    """Compute backward-only TOPP2 reachable intervals."""
    return _native.reach_set2_backward(problem, options)


def bidirectional(problem: Problem, options: Options | None = None) -> ReachSet:
    """Compute bidirectional TOPP2 reachable intervals."""
    return _native.reach_set2_bidirectional(problem, options)


__all__ = ["Problem", "Options", "ReachSet", "backward", "bidirectional"]
