from __future__ import annotations

from typing import Any, Callable, ClassVar, Literal, Protocol, TypedDict, overload

import numpy as np
from numpy.typing import ArrayLike, NDArray


__version__: str


class CoppError(Exception):
    """Base exception for errors reported by the Rust COPP core.

    Python wrapper argument errors, such as invalid dtype or mutually exclusive
    keyword arguments, use standard Python exceptions like ``ValueError``. This
    exception is reserved for errors returned by the underlying COPP library.
    """
    ...


class ConstraintError(CoppError):
    """Exception for errors reported by Rust constraint storage and query APIs."""
    ...


class PathError(CoppError):
    """Exception for errors reported by Rust path construction and evaluation APIs."""
    ...


class OutOfRangeMode:
    """Policy for evaluating a path outside its configured parameter range."""

    ERROR: ClassVar[OutOfRangeMode]
    """Raise ``CoppError`` when a query value is outside ``[s_min, s_max]``."""

    CLAMP: ClassVar[OutOfRangeMode]
    """Clamp out-of-range query values into ``[s_min, s_max]``."""


class Parametrization:
    """Waypoint parameter assignment policy for spline paths."""

    UNIFORM: ClassVar[Parametrization]
    """Assign waypoints uniformly over the configured path-parameter range."""


class MatrixLayout:
    """Matrix layout used by Python path construction and evaluation."""

    SAMPLE_MAJOR: ClassVar[MatrixLayout]
    """Rows are samples or waypoints and columns are path dimensions."""

    DIM_MAJOR: ClassVar[MatrixLayout]
    """Rows are path dimensions and columns are samples or waypoints."""


class Verbosity:
    """Diagnostic verbosity level used by Rust solvers."""

    SILENT: ClassVar[Verbosity]
    """Disable solver diagnostic logging."""

    SUMMARY: ClassVar[Verbosity]
    """Emit start/end and summary-level solver messages."""

    DEBUG: ClassVar[Verbosity]
    """Emit detailed solver messages."""

    TRACE: ClassVar[Verbosity]
    """Emit very detailed per-step solver messages."""



class ClarabelDirectSolveMethod:
    """Direct linear-solver method forwarded to Clarabel."""

    AUTO: ClassVar[ClarabelDirectSolveMethod]
    """Let Clarabel choose the direct solver."""

    QDLDL: ClassVar[ClarabelDirectSolveMethod]
    """Use Clarabel's QDLDL backend."""

    FAER: ClassVar[ClarabelDirectSolveMethod]
    """Use the optional FAER sparse backend when available."""

    MKL: ClassVar[ClarabelDirectSolveMethod]
    """Use the optional MKL Pardiso backend when available."""

    PANUA: ClassVar[ClarabelDirectSolveMethod]
    """Use the optional Panua Pardiso backend when available."""


class ClarabelSolverStatus:
    """Clarabel solver status returned by SOCP expert APIs."""

    UNSOLVED: ClassVar[ClarabelSolverStatus]
    SOLVED: ClassVar[ClarabelSolverStatus]
    PRIMAL_INFEASIBLE: ClassVar[ClarabelSolverStatus]
    DUAL_INFEASIBLE: ClassVar[ClarabelSolverStatus]
    ALMOST_SOLVED: ClassVar[ClarabelSolverStatus]
    ALMOST_PRIMAL_INFEASIBLE: ClassVar[ClarabelSolverStatus]
    ALMOST_DUAL_INFEASIBLE: ClassVar[ClarabelSolverStatus]
    MAX_ITERATIONS: ClassVar[ClarabelSolverStatus]
    MAX_TIME: ClassVar[ClarabelSolverStatus]
    NUMERICAL_ERROR: ClassVar[ClarabelSolverStatus]
    INSUFFICIENT_PROGRESS: ClassVar[ClarabelSolverStatus]
    CALLBACK_TERMINATED: ClassVar[ClarabelSolverStatus]


OutOfRangeModeLike = OutOfRangeMode | Literal["error", "clamp"]
ParametrizationLike = Parametrization | Literal["uniform"]
MatrixLayoutLike = MatrixLayout | Literal["sample_major", "dim_major"]
VerbosityLike = Verbosity | Literal["silent", "summary", "debug", "trace"]
ClarabelDirectSolveMethodLike = ClarabelDirectSolveMethod | Literal[
    "auto", "qdldl", "faer", "mkl", "panua"
]
LimitLike = ArrayLike
NumStationaryMaxLike = int | tuple[int, int]

def version() -> str:
    """Return the copp-py package version compiled into the native module."""
    ...
