from __future__ import annotations

from typing import Any, Callable, ClassVar, Literal, Protocol, TypedDict, overload

import numpy as np
from numpy.typing import NDArray

from .core import Verbosity, VerbosityLike
from .constraints import Constraints

class ReachSet2Options:
    """Options for TOPP2 reachable-set based solvers.

    Omitted fields use Rust ``ReachSet2OptionsBuilder::new()`` defaults:
    ``lp_feas_tol=1e-8``, ``a_cmp_abs_tol=1e-8``,
    ``a_cmp_rel_tol=1e-8``, and ``verbosity=Verbosity.SILENT``. The same
    options object is intended to be reused by ``topp2_ra`` and reach-set
    helpers.
    """

    lp_feas_tol: float
    """Feasibility tolerance used by LP subproblems. Rust default: ``1e-8``."""

    a_cmp_abs_tol: float
    """Absolute tolerance for comparing reachable ``a`` bounds. Rust default: ``1e-8``."""

    a_cmp_rel_tol: float
    """Relative tolerance for comparing reachable ``a`` bounds. Rust default: ``1e-8``."""

    verbosity: Verbosity
    """Diagnostic verbosity level. Rust default: ``Verbosity.SILENT``."""

    def __init__(
        self,
        *,
        lp_feas_tol: float = 1.0e-8,
        a_cmp_abs_tol: float = 1.0e-8,
        a_cmp_rel_tol: float = 1.0e-8,
        verbosity: VerbosityLike = Verbosity.SILENT,
    ) -> None:
        """Construct TOPP2 reachability options.

        Defaults mirror Rust and are listed in the class docstring. Invalid
        tolerances are rejected by the Rust options builder and reported as
        ``CoppError``.
        """
        ...


class ReachSet2:
    """Reachable interval bounds for a TOPP2/COPP2 station grid.

    For each station ``k`` in the associated ``Topp2Problem`` interval, the
    reachable state is ``a_min[k] <= a[k] <= a_max[k]`` where
    ``a = (ds/dt)^2``. The result owns its data. Accessing ``a_min`` or
    ``a_max`` returns a NumPy array copy, so mutating the returned array does
    not affect the result object or any Rust solver state.
    """

    a_max: NDArray[np.float64]
    """Upper reachable bound at each station."""

    a_min: NDArray[np.float64]
    """Lower reachable bound at each station."""

    len: int
    """Number of stations covered by this reachable set."""


class Topp2Problem:
    """Borrowed TOPP2 problem descriptor.

    ``Topp2Problem`` stores a reference to a ``Constraints`` proxy plus the
    closed station interval and endpoint state values. It does not copy
    constraint data. Each solver call rebuilds and validates the Rust
    ``Topp2Problem`` from the current constraint buffer.

    Examples
    --------
    Construct the problem from ``robot.constraints`` and solve it with default
    reach-set options::

        problem = copp.solver.topp2_ra.Problem(
            robot.constraints,
            idx_s_interval=(0, len(robot) - 1),
            a_boundary=(0.0, 0.0),
        )
        a = copp.solver.topp2_ra.solve(problem)
    """

    @property
    def constraints(self) -> Constraints:
        """Constraint proxy referenced by this immutable descriptor."""
        ...

    @property
    def idx_s_interval(self) -> tuple[int, int]:
        """Closed station-index interval ``(idx_s_start, idx_s_final)``."""
        ...

    @property
    def a_boundary(self) -> tuple[float, float]:
        """Endpoint state values ``(a_start, a_final)``."""
        ...

    @property
    def s_len(self) -> int:
        """Number of station samples in the closed interval."""
        ...

    def __init__(
        self,
        constraints: Constraints,
        idx_s_interval: tuple[int, int],
        a_boundary: tuple[float, float] = (0.0, 0.0),
    ) -> None:
        """Construct and validate a TOPP2 problem descriptor.

        Parameters
        ----------
        constraints:
            Usually ``robot.constraints``.
        idx_s_interval:
            Closed station-index interval ``(idx_s_start, idx_s_final)``.
        a_boundary:
            Boundary values of ``a = ds/dt squared`` at the start and final
            station.

        Raises
        ------
        CoppError
            If the interval or boundary values are invalid for the current
            constraints.
        """
        ...

    def validate(self) -> None:
        """Validate the descriptor against the current constraint buffer."""
        ...


def reach_set2_backward(
    problem: Topp2Problem,
    options: ReachSet2Options | None = None,
) -> ReachSet2:
    """Compute backward-only TOPP2 reachable intervals.

    The backward pass enforces the terminal boundary in ``problem.a_boundary``
    and checks the start boundary for basic feasibility, but it does not clip
    intervals with a forward pass.

    Parameters
    ----------
    problem:
        TOPP2 problem descriptor, normally created from ``robot.constraints``.
    options:
        Optional reachable-set options. ``None`` uses Rust defaults.

    Returns
    -------
    ReachSet2
        Owned reachable intervals with ``a_min`` and ``a_max`` arrays.

    Raises
    ------
    CoppError
        If Rust problem validation, options validation, or reachable-set
        construction fails.
    """
    ...


def reach_set2_bidirectional(
    problem: Topp2Problem,
    options: ReachSet2Options | None = None,
) -> ReachSet2:
    """Compute bidirectional TOPP2 reachable intervals.

    The bidirectional variant performs the backward pass and then clips the
    result with a forward pass, enforcing both start and terminal boundary
    values in ``problem.a_boundary``.

    Parameters
    ----------
    problem:
        TOPP2 problem descriptor, normally created from ``robot.constraints``.
    options:
        Optional reachable-set options. ``None`` uses Rust defaults.

    Returns
    -------
    ReachSet2
        Owned reachable intervals with ``a_min`` and ``a_max`` arrays.

    Raises
    ------
    CoppError
        If Rust problem validation, options validation, or reachable-set
        construction fails.
    """
    ...


def topp2_ra(
    problem: Topp2Problem,
    options: ReachSet2Options | None = None,
) -> NDArray[np.float64]:
    """Solve a TOPP2 problem with reachability analysis.

    Parameters
    ----------
    problem:
        TOPP2 problem descriptor, normally created from ``robot.constraints``.
    options:
        Optional reachable-set options. ``None`` uses Rust defaults.

    Returns
    -------
    numpy.ndarray
        Node profile ``a`` with length ``problem.s_len`` where
        ``a = (ds/dt)^2``.

    Raises
    ------
    CoppError
        If Rust problem validation, options validation, or the RA solve fails.
    """
    ...
