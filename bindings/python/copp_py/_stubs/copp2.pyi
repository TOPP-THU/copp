from __future__ import annotations

from collections.abc import Iterable
from typing import Any, Callable, ClassVar, Literal, Protocol, TypedDict, overload

import numpy as np
from numpy.typing import NDArray

from .clarabel import ClarabelOptions, Copp2SocpResult
from .core import Verbosity, VerbosityLike
from .objective import ObjectiveLike
from .robot import Robot

class Copp2Problem:
    """Borrowed COPP2 problem descriptor.

    ``Copp2Problem`` stores a reference to ``Robot`` plus Python-owned
    objective descriptors. It does not copy robot constraints. Each solver call
    rebuilds and validates the Rust ``Copp2Problem`` from the current robot and
    objectives.

    Objective dictionaries use these formats::

        {"kind": "time", "weight": 1.0}
        {"kind": "thermal_energy", "weight": 0.1, "normalize": normalize}
        {"kind": "linear", "weight": 1.0, "alpha": alpha, "beta": beta}
        {"kind": "total_variation_torque", "weight": 1.0, "normalize": normalize}

    Object constructors are exposed through ``copp.objective``::

        objectives = [
            copp.objective.Time(1.0),
            copp.objective.ThermalEnergy(0.1, normalize),
        ]
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
        robot: Robot,
        objectives: ObjectiveLike | Iterable[ObjectiveLike],
        idx_s_interval: tuple[int, int],
        a_boundary: tuple[float, float] = (0.0, 0.0),
    ) -> None:
        """Construct and validate a COPP2 problem descriptor."""
        ...

    def validate(self) -> None:
        """Validate the descriptor against the current robot and objectives."""
        ...


def copp2_socp(
    problem: Copp2Problem,
    options: ClarabelOptions | None = None,
) -> NDArray[np.float64]:
    """Solve a COPP2 problem with the Clarabel SOCP backend.

    Parameters
    ----------
    problem:
        COPP2 problem descriptor, normally created from a ``Robot`` and a list
        of objectives.
    options:
        Optional Clarabel options. ``None`` uses Rust
        ``ClarabelOptionsBuilder::new()`` defaults.

    Returns
    -------
    numpy.ndarray
        Node profile ``a`` with length ``problem.s_len`` where
        ``a = (ds/dt)^2``.

    Raises
    ------
    CoppError
        If Rust problem validation, options validation, Clarabel solve setup,
        or accepted-status extraction fails. ``copp2_socp`` supports
        ``TotalVariationTorque`` objectives.
    """
    ...


def copp2_socp_expert(
    problem: Copp2Problem,
    options: ClarabelOptions | None = None,
) -> Copp2SocpResult:
    """Solve a COPP2 problem with Clarabel and return expert diagnostics.

    Unlike ``copp2_socp``, non-accepted Clarabel statuses do not raise merely
    because the status is not accepted by ``options``. Inspect
    ``result.a`` and ``result.solver_status`` to decide whether an accepted
    high-level profile is available. ``result.a is None`` means no accepted
    profile was extracted. True runtime failures, such as invalid problems or
    Clarabel setup errors, still raise ``CoppError``.
    """
    ...
