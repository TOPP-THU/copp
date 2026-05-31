"""TOPP2 reachable-set construction."""

from .._native import (
    ReachSet2 as ReachSet,
    ReachSet2Options as Options,
    Topp2Problem as Problem,
)

def backward(problem: Problem, options: Options | None = None) -> ReachSet:
    """Compute backward-only TOPP2 reachable intervals."""
    ...

def bidirectional(problem: Problem, options: Options | None = None) -> ReachSet:
    """Compute bidirectional TOPP2 reachable intervals."""
    ...

__all__: list[str]
