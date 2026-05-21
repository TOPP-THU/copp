"""Type stubs for copp._copp native module."""

from typing import Callable, Optional
import numpy as np
from numpy.typing import NDArray

# ─── Exceptions ───

class CoppError(Exception): ...
class PathError(CoppError): ...
class ConstraintError(CoppError): ...
class InfeasibleError(CoppError): ...
class InvalidInputError(CoppError): ...

# ─── Path & Automatic Differentiation ───

class Jet3:
    @property
    def v(self) -> float: ...
    @property
    def d1(self) -> float: ...
    @property
    def d2(self) -> float: ...
    @property
    def d3(self) -> float: ...

    def __init__(self, v: float) -> None: ...
    @staticmethod
    def seed(v: float) -> "Jet3": ...
    @staticmethod
    def constant(v: float) -> "Jet3": ...

    def __add__(self, other: "Jet3 | float") -> "Jet3": ...
    def __radd__(self, other: "Jet3 | float") -> "Jet3": ...
    def __sub__(self, other: "Jet3 | float") -> "Jet3": ...
    def __rsub__(self, other: float) -> "Jet3": ...
    def __mul__(self, other: "Jet3 | float") -> "Jet3": ...
    def __rmul__(self, other: "Jet3 | float") -> "Jet3": ...
    def __truediv__(self, other: "Jet3 | float") -> "Jet3": ...
    def __neg__(self) -> "Jet3": ...
    def __pow__(self, n: int) -> "Jet3": ...
    def __repr__(self) -> str: ...

def sin(x: Jet3) -> Jet3: ...
def cos(x: Jet3) -> Jet3: ...
def exp(x: Jet3) -> Jet3: ...
def ln(x: Jet3) -> Jet3: ...
def sqrt(x: Jet3) -> Jet3: ...
def powi(x: Jet3, n: int) -> Jet3: ...

# ─── SplineConfig ───

class SplineConfig:
    def __init__(
        self,
        order: int = 5,
        s_min: float = 0.0,
        s_max: float = 1.0,
        clamp: bool = False,
    ) -> None: ...

# ─── PathDerivatives ───

class PathDerivatives:
    @property
    def q(self) -> NDArray[np.float64]: ...
    @property
    def dq(self) -> Optional[NDArray[np.float64]]: ...
    @property
    def ddq(self) -> Optional[NDArray[np.float64]]: ...
    @property
    def dddq(self) -> Optional[NDArray[np.float64]]: ...

class _PathDerivatives2(PathDerivatives):
    @property
    def dq(self) -> NDArray[np.float64]: ...
    @property
    def ddq(self) -> NDArray[np.float64]: ...

class _PathDerivatives3(_PathDerivatives2):
    @property
    def dddq(self) -> NDArray[np.float64]: ...

# ─── Path ───

class Path:
    @property
    def dim(self) -> int: ...
    @property
    def s_range(self) -> tuple[float, float]: ...

    @staticmethod
    def from_waypoints(
        waypoints: NDArray[np.float64],
        config: Optional[SplineConfig] = None,
    ) -> "Path": ...
    @staticmethod
    def from_parametric(
        q_fn: Callable[[Jet3], list[Jet3]],
        s_min: float,
        s_max: float,
    ) -> "Path": ...

    def evaluate_q(self, s: NDArray[np.float64]) -> PathDerivatives: ...
    def evaluate_up_to_2nd(self, s: NDArray[np.float64]) -> _PathDerivatives2: ...
    def evaluate_up_to_3rd(self, s: NDArray[np.float64]) -> _PathDerivatives3: ...

# ─── Robot ───

class Robot:
    @property
    def dim(self) -> int: ...

    def __init__(
        self,
        dim: int,
        inverse_dynamics: Optional[Callable[[list[float], list[float], list[float]], list[float]]] = None,
        capacity: int = 0,
    ) -> None: ...

    def with_s(self, s: NDArray[np.float64]) -> None: ...
    def with_q(
        self,
        q: NDArray[np.float64],
        dq: NDArray[np.float64],
        ddq: NDArray[np.float64],
        dddq: Optional[NDArray[np.float64]] = None,
        idx_s: int = 0,
    ) -> None: ...
    def with_axial_velocity(
        self,
        vel_max: NDArray[np.float64],
        vel_min: NDArray[np.float64],
        idx_s: int = 0,
    ) -> None: ...
    def with_axial_acceleration(
        self,
        acc_max: NDArray[np.float64],
        acc_min: NDArray[np.float64],
        idx_s: int = 0,
    ) -> None: ...
    def with_axial_jerk(
        self,
        jerk_max: NDArray[np.float64],
        jerk_min: NDArray[np.float64],
        idx_s: int = 0,
    ) -> None: ...
    def amax_substitute(
        self,
        amax_new: NDArray[np.float64],
        idx_from: int,
    ) -> None: ...

# ─── Options ───

class ReachSet2Options:
    def __init__(
        self,
        a_cmp_abs_tol: Optional[float] = None,
        a_cmp_rel_tol: Optional[float] = None,
        lp_feas_tol: Optional[float] = None,
        verbosity: str = "silent",
    ) -> None: ...

class ClarabelOptions:
    def __init__(
        self,
        verbosity: str = "silent",
        allow_almost_solved: bool = False,
        allow_max_iterations: bool = False,
        allow_max_time: bool = False,
        allow_insufficient_progress: bool = False,
    ) -> None: ...

# ─── Objectives ───

class Objective:
    @staticmethod
    def time(weight: float = 1.0) -> "Objective": ...
    @staticmethod
    def thermal_energy(weight: float, normalize: list[float]) -> "Objective": ...
    @staticmethod
    def total_variation_torque(weight: float, normalize: list[float]) -> "Objective": ...
    @staticmethod
    def linear(weight: float, alpha: list[float], beta: list[float]) -> "Objective": ...

# ─── Solvers ───

def topp2_ra(
    robot: Robot,
    idx_s_interval: tuple[int, int],
    a_boundary: tuple[float, float],
    options: Optional[ReachSet2Options] = None,
) -> NDArray[np.float64]: ...

def reach_set2_backward(
    robot: Robot,
    idx_s_interval: tuple[int, int],
    a_boundary: tuple[float, float],
    options: Optional[ReachSet2Options] = None,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]: ...

def reach_set2_bidirectional(
    robot: Robot,
    idx_s_interval: tuple[int, int],
    a_boundary: tuple[float, float],
    options: Optional[ReachSet2Options] = None,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]: ...

def copp2_socp(
    robot: Robot,
    idx_s_interval: tuple[int, int],
    a_boundary: tuple[float, float],
    objectives: list[Objective],
    options: Optional[ClarabelOptions] = None,
) -> NDArray[np.float64]: ...

def topp3_lp(
    robot: Robot,
    idx_s_start: int,
    a_linearization: NDArray[np.float64],
    a_boundary: tuple[float, float],
    b_boundary: tuple[float, float],
    options: Optional[ClarabelOptions] = None,
) -> tuple[NDArray[np.float64], NDArray[np.float64], tuple[int, int]]: ...

def topp3_socp(
    robot: Robot,
    idx_s_start: int,
    a_linearization: NDArray[np.float64],
    a_boundary: tuple[float, float],
    b_boundary: tuple[float, float],
    options: Optional[ClarabelOptions] = None,
) -> tuple[NDArray[np.float64], NDArray[np.float64], tuple[int, int]]: ...

def copp3_socp(
    robot: Robot,
    idx_s_start: int,
    a_linearization: NDArray[np.float64],
    a_boundary: tuple[float, float],
    b_boundary: tuple[float, float],
    objectives: list[Objective],
    options: Optional[ClarabelOptions] = None,
) -> tuple[NDArray[np.float64], NDArray[np.float64], tuple[int, int]]: ...

# ─── Interpolation ───

def s_to_t_topp2(
    s: NDArray[np.float64],
    a: NDArray[np.float64],
    t_offset: float = 0.0,
) -> tuple[float, NDArray[np.float64]]: ...

def t_to_s_topp2(
    s: NDArray[np.float64],
    a: NDArray[np.float64],
    t_s: NDArray[np.float64],
    dt: Optional[float] = None,
    t_samples: Optional[NDArray[np.float64]] = None,
) -> NDArray[np.float64]: ...

def a_to_b_topp2(
    s: NDArray[np.float64],
    a: NDArray[np.float64],
) -> NDArray[np.float64]: ...

def s_to_t_topp3(
    s: NDArray[np.float64],
    a: NDArray[np.float64],
    b: NDArray[np.float64],
    num_stationary: tuple[int, int],
    t_offset: float = 0.0,
) -> tuple[float, NDArray[np.float64]]: ...

def t_to_s_topp3(
    s: NDArray[np.float64],
    a: NDArray[np.float64],
    b: NDArray[np.float64],
    num_stationary: tuple[int, int],
    t_s: NDArray[np.float64],
    dt: Optional[float] = None,
    t_samples: Optional[NDArray[np.float64]] = None,
) -> NDArray[np.float64]: ...
