from __future__ import annotations

from typing import Any, Callable, ClassVar, Literal, Protocol, TypedDict, overload

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .clarabel import ClarabelOptions, Copp3ClarabelResult
from .core import NumStationaryMaxLike
from .interpolation import Profile3rd
from .constraints import Constraints

class Topp3Problem:
    """Linearized TOPP3 problem descriptor.

    ``Topp3Problem`` stores a reference to a ``Constraints`` proxy and an owned
    copy of ``a_linearization``. Construction immediately rebuilds the Rust
    TOPP3 problem and linearizes third-order jerk constraints into the
    constraint buffer's cached affine rows. Calling ``validate()`` later repeats
    that linearization against the current constraints.

    ``num_stationary_max`` accepts either an integer shorthand or an explicit
    ``(start, end)`` pair. The integer shorthand is the recommended Python
    style: ``num_stationary_max=1`` means ``(1, 1)``.

    Examples
    --------
    Construct a TOPP3 problem from ``robot.constraints``::

        a_linearization = np.ones(len(robot), dtype=np.float64)
        problem = copp.solver.topp3_lp.Problem(
            robot.constraints,
            a_linearization,
            idx_s_start=0,
            a_boundary=(0.0, 0.0),
            b_boundary=(0.0, 0.0),
            num_stationary_max=1,
        )
    """

    @property
    def constraints(self) -> Constraints:
        """Constraint proxy referenced by this immutable descriptor."""
        ...

    @property
    def a_linearization(self) -> NDArray[np.float64]:
        """Copy of the reference profile used to linearize third-order constraints."""
        ...

    @property
    def idx_s_start(self) -> int:
        """Start station index of the optimization interval."""
        ...

    @property
    def idx_s_final(self) -> int:
        """Final station index implied by ``a_linearization``."""
        ...

    @property
    def a_boundary(self) -> tuple[float, float]:
        """Boundary values ``(a_start, a_final)``."""
        ...

    @property
    def b_boundary(self) -> tuple[float, float]:
        """Boundary values ``(b_start, b_final)``."""
        ...

    @property
    def num_stationary_max(self) -> tuple[int, int]:
        """Stationary-interval upper bounds as ``(start, end)``."""
        ...

    @property
    def a_linearization_floor(self) -> float:
        """Denominator floor used by third-order linearization."""
        ...

    @property
    def s_len(self) -> int:
        """Number of station samples in this TOPP3 interval."""
        ...

    def __init__(
        self,
        constraints: Constraints,
        a_linearization: ArrayLike,
        *,
        idx_s_start: int = 0,
        a_boundary: tuple[float, float] = (0.0, 0.0),
        b_boundary: tuple[float, float] = (0.0, 0.0),
        num_stationary_max: NumStationaryMaxLike = 1,
        a_linearization_floor: float = 1.0e-10,
    ) -> None:
        """Construct and immediately linearize a TOPP3 problem descriptor.

        Parameters
        ----------
        constraints:
            Usually ``robot.constraints``.
        a_linearization:
            One-dimensional ArrayLike reference profile convertible to
            ``float64``. The descriptor stores a copy.
        idx_s_start:
            Start station index of the optimization interval. The final index
            is ``idx_s_start + len(a_linearization) - 1``.
        a_boundary:
            Boundary values of ``a = (ds/dt)^2`` at the start and final
            station.
        b_boundary:
            Boundary values of ``b = dds/dt`` at the start and final station.
        num_stationary_max:
            Stationary-boundary upper bound. ``1`` means ``(1, 1)``.
        a_linearization_floor:
            Positive denominator floor used when linearizing third-order
            constraints near ``a = 0``.

        Raises
        ------
        ValueError
            If ``a_linearization`` cannot be converted to a one-dimensional
            ``float64`` array, or if ``num_stationary_max`` is not an integer
            or integer pair.
        CoppError
            If Rust validation or third-order constraint linearization fails.
        """
        ...

    def validate(self) -> None:
        """Rebuild the Rust TOPP3 problem and refresh jerk linearization.

        This method may update cached third-order affine rows in the referenced
        constraint buffer.
        """
        ...

def topp3_lp(
    problem: Topp3Problem,
    options: ClarabelOptions | None = None,
) -> Profile3rd:
    """Solve a TOPP3 problem with the Clarabel LP backend.

    ``options=None`` uses Rust ``ClarabelOptionsBuilder`` defaults.
    The returned ``Profile3rd`` is available only when the Clarabel status is
    accepted by ``options``; otherwise this strict API raises ``CoppError``.
    """
    ...


def topp3_lp_expert(
    problem: Topp3Problem,
    options: ClarabelOptions | None = None,
) -> Copp3ClarabelResult:
    """Solve TOPP3-LP and return raw Clarabel diagnostics.

    Non-accepted Clarabel statuses do not raise merely because the status is
    not accepted by ``options``. Inspect ``result.profile`` and
    ``result.solver_status`` to decide whether an accepted high-level profile
    is available. ``result.profile is None`` means no accepted profile was
    extracted. This shares ``Copp3ClarabelResult`` with TOPP3-SOCP and
    COPP3-SOCP.
    """
    ...


def topp3_socp(
    problem: Topp3Problem,
    options: ClarabelOptions | None = None,
) -> Profile3rd:
    """Solve a TOPP3 problem with the Clarabel SOCP backend.

    ``options=None`` uses Rust ``ClarabelOptionsBuilder`` defaults.
    """
    ...


def topp3_socp_expert(
    problem: Topp3Problem,
    options: ClarabelOptions | None = None,
) -> Copp3ClarabelResult:
    """Solve TOPP3-SOCP and return raw Clarabel diagnostics.

    The result type is shared with ``topp3_lp_expert`` and
    ``copp3_socp_expert``.
    """
    ...
