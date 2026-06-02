from __future__ import annotations

from typing import Any, Callable, ClassVar, Literal, Protocol, TypedDict, overload

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .core import CoppError

class TimeObjectiveDict(TypedDict):
    """Dictionary form for ``Time`` objectives."""

    kind: Literal["time"]
    weight: float


class LinearObjectiveDict(TypedDict):
    """Dictionary form for ``Linear`` objectives."""

    kind: Literal["linear"]
    weight: float
    alpha: ArrayLike
    beta: ArrayLike


class ThermalEnergyObjectiveDict(TypedDict):
    """Dictionary form for ``ThermalEnergy`` objectives."""

    kind: Literal["thermal_energy", "thermal"]
    weight: float
    normalize: ArrayLike


class TotalVariationTorqueObjectiveDict(TypedDict):
    """Dictionary form for ``TotalVariationTorque`` objectives."""

    kind: Literal["total_variation_torque", "tv_torque"]
    weight: float
    normalize: ArrayLike


ObjectiveDict = (
    TimeObjectiveDict
    | LinearObjectiveDict
    | ThermalEnergyObjectiveDict
    | TotalVariationTorqueObjectiveDict
)


class Time:
    """Time objective ``weight * integral(dt)``.

    Public user code should construct this as ``copp.objective.Time(...)``.
    """

    weight: float
    """Objective weight."""

    def __init__(self, weight: float) -> None:
        """Construct a time objective."""
        ...


class Linear:
    """Linear objective over profile variables.

    Public user code should construct this as ``copp.objective.Linear(...)``.
    For COPP2, ``alpha`` is node-based and must have length ``problem.s_len``;
    ``beta`` is interval-based and must have length ``problem.s_len - 1``.
    For COPP3, both ``alpha`` and ``beta`` are node-based and must have length
    ``problem.s_len``.
    """

    weight: float
    """Objective weight."""

    alpha: NDArray[np.float64]
    """Node-based coefficient for ``a``."""

    beta: NDArray[np.float64]
    """Interval-based coefficient for ``b`` in COPP2."""

    def __init__(
        self,
        weight: float,
        alpha: ArrayLike,
        beta: ArrayLike,
    ) -> None:
        """Construct a linear objective."""
        ...


class ThermalEnergy:
    """Thermal-energy objective using the robot inverse-dynamics model.

    Public user code should construct this as
    ``copp.objective.ThermalEnergy(...)``.
    """

    weight: float
    """Objective weight."""

    normalize: NDArray[np.float64]
    """Per-axis torque normalization vector with length ``robot.dim``."""

    def __init__(self, weight: float, normalize: ArrayLike) -> None:
        """Construct a thermal-energy objective."""
        ...


class TotalVariationTorque:
    """Total-variation torque objective.

    Public user code should construct this as
    ``copp.objective.TotalVariationTorque(...)``.
    This objective is supported by the SOCP-backed COPP solvers.
    """

    weight: float
    """Objective weight."""

    normalize: NDArray[np.float64]
    """Per-axis torque normalization vector with length ``robot.dim``."""

    def __init__(self, weight: float, normalize: ArrayLike) -> None:
        """Construct a total-variation torque objective."""
        ...


ObjectiveLike = (
    Time
    | Linear
    | ThermalEnergy
    | TotalVariationTorque
    | ObjectiveDict
)
