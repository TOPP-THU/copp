r"""Python interface for copp-py.

Import the package as::

    import copp_py as copp

The public package follows the Rust crate layout. Core modeling types are
available through focused namespaces such as ``copp.path``, ``copp.robot``,
``copp.constraints``, ``copp.objective``, ``copp.interpolation``, and
``copp.clarabel``. Solver entry points live under ``copp.solver`` with one
module per algorithm, for example ``copp.solver.topp2_ra.solve`` and
``copp.solver.copp3_socp.solve``.

A few frequently used core types are also re-exported at the package root for
interactive use. Solver-specific problem, option, result, and function names
are intentionally kept inside their algorithm modules.
"""

from . import clarabel as clarabel
from . import constraints as constraints
from . import core as core
from . import interpolation as interpolation
from . import objective as objective
from . import path as path
from . import robot as robot
from . import solver as solver
from .core import (
    ConstraintError,
    CoppError,
    MatrixLayout,
    OutOfRangeMode,
    Parametrization,
    PathError,
    Verbosity,
    __version__,
    version,
)
from .constraints import Constraints
from .interpolation import Profile3rd
from .path import Path, PathDerivatives, SplineConfig
from .robot import Robot

__all__ = [
    "ConstraintError",
    "Constraints",
    "CoppError",
    "MatrixLayout",
    "OutOfRangeMode",
    "Parametrization",
    "Path",
    "PathDerivatives",
    "PathError",
    "Profile3rd",
    "Robot",
    "SplineConfig",
    "Verbosity",
    "__version__",
    "clarabel",
    "constraints",
    "core",
    "interpolation",
    "objective",
    "path",
    "robot",
    "solver",
    "version",
]
