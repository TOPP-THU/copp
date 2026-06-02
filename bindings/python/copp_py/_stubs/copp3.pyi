from __future__ import annotations

from collections.abc import Iterable
from typing import Any, Callable, ClassVar, Literal, Protocol, TypedDict, overload

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .clarabel import ClarabelOptions, Copp3ClarabelResult
from .core import NumStationaryMaxLike
from .interpolation import Profile3rd
from .objective import ObjectiveLike
from .robot import Robot

class Copp3Problem:
    """Linearized COPP3 problem descriptor.

    ``Copp3Problem`` stores a reference to ``Robot``, Python-owned objective
    descriptors, and a copy of ``a_linearization``. Construction immediately
    rebuilds the Rust COPP3 problem and linearizes third-order jerk constraints
    into the robot constraint buffer's cached affine rows.

    ``num_stationary_max`` accepts either an integer shorthand or an explicit
    ``(start, end)`` pair. The recommended Python style is
    ``num_stationary_max=1``.

    Examples
    --------
    Build a COPP3 problem with the same objective list style as COPP2::

        objectives = [
            copp.objective.Time(1.0),
            copp.objective.ThermalEnergy(0.1, np.ones(robot.dim)),
        ]
        problem = copp.solver.copp3_socp.Problem(
            robot,
            objectives,
            np.ones(len(robot), dtype=np.float64),
            idx_s_start=0,
            a_boundary=(0.0, 0.0),
            b_boundary=(0.0, 0.0),
            num_stationary_max=1,
        )
    """

    @property
    def robot(self) -> Robot:
        """Robot referenced by this immutable descriptor."""
        ...

    @property
    def objective_count(self) -> int:
        """Number of objective descriptors."""
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
        """Number of station samples in this COPP3 interval."""
        ...

    def __init__(
        self,
        robot: Robot,
        objectives: ObjectiveLike | Iterable[ObjectiveLike],
        a_linearization: ArrayLike,
        *,
        idx_s_start: int = 0,
        a_boundary: tuple[float, float] = (0.0, 0.0),
        b_boundary: tuple[float, float] = (0.0, 0.0),
        num_stationary_max: NumStationaryMaxLike = 1,
        a_linearization_floor: float = 1.0e-10,
    ) -> None:
        """Construct and immediately linearize a COPP3 problem descriptor."""
        ...

    def validate(self) -> None:
        """Rebuild the Rust COPP3 problem and refresh jerk linearization."""
        ...

def copp3_socp(
    problem: Copp3Problem,
    options: ClarabelOptions | None = None,
) -> Profile3rd:
    """Solve a COPP3 problem with the Clarabel SOCP backend.

    The strict API returns only an accepted ``Profile3rd``. For raw Clarabel
    vectors and status inspection, use ``copp3_socp_expert``.
    """
    ...


def copp3_socp_expert(
    problem: Copp3Problem,
    options: ClarabelOptions | None = None,
) -> Copp3ClarabelResult:
    """Solve COPP3-SOCP and return raw Clarabel diagnostics.

    ``objective_value`` is the weighted COPP objective value computed from the
    accepted profile, and ``objective_terms`` is a copy of the per-objective
    unweighted terms. If Clarabel does not produce an accepted profile,
    ``profile``, ``objective_value``, and ``objective_terms`` are ``None``.
    """
    ...
