r"""Objective constructors for COPP solvers.

Use as ``copp.objective.Time(...)`` after ``import copp_py as copp``.

TOPP solvers minimize traversal time directly. COPP solvers accept one
objective or a list of objective descriptors and optimize their weighted sum.
The built-in descriptors are:

- ``Time(weight)`` for traversal-time cost.
- ``ThermalEnergy(weight, normalize)`` for torque-energy style cost.
- ``TotalVariationTorque(weight, normalize)`` for torque variation.
- ``Linear(weight, alpha, beta)`` for direct coefficients on profile variables.

For second-order COPP, ``alpha`` is node-based and ``beta`` is interval-based.
For third-order COPP, both arrays are node-based. Solver-side validation checks
the active length contract.
"""

from ._native import Linear, ThermalEnergy, Time, TotalVariationTorque

__all__ = [
    "Linear",
    "ThermalEnergy",
    "Time",
    "TotalVariationTorque",
]
