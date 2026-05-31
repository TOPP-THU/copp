"""Public Python facade for copp-py.

Import as ``import copp_py as copp``.
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
    ConstraintError as ConstraintError,
    CoppError as CoppError,
    MatrixLayout as MatrixLayout,
    OutOfRangeMode as OutOfRangeMode,
    Parametrization as Parametrization,
    PathError as PathError,
    Verbosity as Verbosity,
    __version__ as __version__,
    version as version,
)
from .constraints import Constraints as Constraints
from .interpolation import Profile3rd as Profile3rd
from .path import Path as Path, PathDerivatives as PathDerivatives, SplineConfig as SplineConfig
from .robot import Robot as Robot

__all__: list[str]
