"""Type stubs for copp._copp native module."""

from typing import Callable, Optional
from enum import IntEnum
import numpy as np
from numpy.typing import NDArray

Array = NDArray[np.float64]
PathEvaluateQ = Callable[[Array], Array]
PathEvaluate2nd = Callable[[Array], tuple[Array, Array, Array]]
PathEvaluate3rd = Callable[[Array], tuple[Array, Array, Array, Array]]

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
    def __rtruediv__(self, other: float) -> "Jet3": ...
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
    @property
    def order(self) -> int: ...
    @property
    def s_min(self) -> float: ...
    @property
    def s_max(self) -> float: ...
    @property
    def clamp(self) -> bool: ...
    @property
    def start_state(self) -> Optional[Array]: ...
    @property
    def end_state(self) -> Optional[Array]: ...

    def __init__(
        self,
        order: int = 5,
        s_min: float = 0.0,
        s_max: float = 1.0,
        clamp: bool = False,
        start_state: Optional[Array] = None,
        end_state: Optional[Array] = None,
    ) -> None: ...

# ─── PathDerivatives ───

class PathDerivatives:
    @property
    def q(self) -> Array: ...
    @property
    def dq(self) -> Optional[Array]: ...
    @property
    def ddq(self) -> Optional[Array]: ...
    @property
    def dddq(self) -> Optional[Array]: ...

class _PathDerivatives2(PathDerivatives):
    @property
    def dq(self) -> Array: ...
    @property
    def ddq(self) -> Array: ...

class _PathDerivatives3(_PathDerivatives2):
    @property
    def dddq(self) -> Array: ...

# ─── Path ───

class Path:
    @property
    def dim(self) -> int: ...
    @property
    def s_range(self) -> tuple[float, float]: ...

    @staticmethod
    def from_waypoints(
        waypoints: Array,
        config: Optional[SplineConfig] = None,
    ) -> "Path": ...
    @staticmethod
    def from_evaluator_2nd(
        dim: int,
        evaluate_up_to_2nd: PathEvaluate2nd,
        s_min: float,
        s_max: float,
        evaluate_q: Optional[PathEvaluateQ] = None,
    ) -> "Path": ...
    @staticmethod
    def from_evaluator_3rd(
        dim: int,
        evaluate_up_to_3rd: PathEvaluate3rd,
        s_min: float,
        s_max: float,
        evaluate_up_to_2nd: Optional[PathEvaluate2nd] = None,
        evaluate_q: Optional[PathEvaluateQ] = None,
    ) -> "Path": ...
    @staticmethod
    def from_parametric(
        q_fn: Callable[[Jet3], list[Jet3]],
        s_min: float,
        s_max: float,
    ) -> "Path": ...

    def evaluate_q(self, s: Array) -> PathDerivatives: ...
    def evaluate_up_to_2nd(self, s: Array) -> _PathDerivatives2: ...
    def evaluate_up_to_3rd(self, s: Array) -> _PathDerivatives3: ...

# ─── Robot ───

class Robot:
    @property
    def dim(self) -> int: ...
    @property
    def len(self) -> int: ...
    @property
    def is_empty(self) -> bool: ...
    @property
    def idx_s_start(self) -> int: ...
    @property
    def idx_s_end(self) -> int: ...
    @property
    def amax_rows(self) -> int: ...
    @property
    def acc_rows(self) -> int: ...
    @property
    def jerk_rows(self) -> int: ...

    def __init__(
        self,
        dim: int,
        inverse_dynamics: Optional[Callable[[list[float], list[float], list[float]], list[float]]] = None,
        capacity: int = 0,
    ) -> None: ...

    def with_s(self, s: Array) -> None: ...
    def set_inverse_dynamics(
        self,
        inverse_dynamics: Callable[[list[float], list[float], list[float]], list[float]],
    ) -> None: ...
    def clear_inverse_dynamics(self) -> None: ...
    def with_q(
        self,
        q: Array,
        dq: Array,
        ddq: Array,
        dddq: Optional[Array] = None,
        idx_s: int = 0,
    ) -> None: ...
    def with_q_from_path_2nd(
        self,
        path: Path,
        idx_s_from: int,
        idx_s_to: int,
    ) -> None: ...
    def with_q_from_path_3rd(
        self,
        path: Path,
        idx_s_from: int,
        idx_s_to: int,
    ) -> None: ...
    def with_axial_velocity(
        self,
        vel_max: Array,
        vel_min: Array,
        idx_s: int = 0,
    ) -> None: ...
    def with_axial_acceleration(
        self,
        acc_max: Array,
        acc_min: Array,
        idx_s: int = 0,
    ) -> None: ...
    def with_axial_jerk(
        self,
        jerk_max: Array,
        jerk_min: Array,
        idx_s: int = 0,
    ) -> None: ...
    def with_axial_torque(
        self,
        torque_max: Array,
        torque_min: Array,
        idx_s: int = 0,
    ) -> None: ...
    def get_s(self, idx_s: int) -> float: ...
    def get_amax(self, idx_s: int) -> float: ...
    def s_vec(self, idx_s_from: int, idx_s_to: int) -> Array: ...
    def amax_vec(self, idx_s_from: int, idx_s_to: int) -> Array: ...
    def with_constraint_1order(
        self,
        amax: Array,
        idx_s: int = 0,
    ) -> None: ...
    def with_constraint_2order(
        self,
        acc_a: Array,
        acc_b: Array,
        acc_max: Array,
        idx_s: int = 0,
        is_negative: bool = False,
    ) -> None: ...
    def with_constraint_3order(
        self,
        jerk_a: Array,
        jerk_b: Array,
        jerk_c: Array,
        jerk_d: Array,
        jerk_max: Array,
        idx_s: int = 0,
        is_negative: bool = False,
    ) -> None: ...
    def get_acc_constraints(self, idx_s: int) -> tuple[Array, Array, Array]: ...
    def get_jerk_constraints(self, idx_s: int) -> tuple[Array, Array, Array, Array, Array]: ...
    def get_jerk_linear_constraints(self, idx_s: int) -> tuple[Array, Array, Array, Array]: ...
    def pop_front_n(self, n_cols: int) -> None: ...
    def pop_back_n(self, n_cols: int) -> None: ...
    def pop_front_to(self, idx_s_cut: int) -> None: ...
    def pop_back_to(self, idx_s_cut: int) -> None: ...
    def clear(self, keep_idx_s: bool = True) -> None: ...
    def expand_capacity(self, new_capacity: int) -> None: ...
    def amax_substitute(
        self,
        amax_new: Array,
        idx_from: int,
    ) -> None: ...

# ─── Enums ───

class Verbosity(IntEnum):
    Silent = 0
    Summary = 1
    Debug = 2
    Trace = 3

class ClarabelSolverStatus(IntEnum):
    Unsolved = 0
    Solved = 1
    PrimalInfeasible = 2
    DualInfeasible = 3
    AlmostSolved = 4
    AlmostPrimalInfeasible = 5
    AlmostDualInfeasible = 6
    MaxIterations = 7
    MaxTime = 8
    NumericalError = 9
    InsufficientProgress = 10
    CallbackTerminated = 11

class DirectSolveMethod(IntEnum):
    Auto = 0
    Qdldl = 1
    Faer = 2
    Mkl = 3
    Panua = 4

# ─── Options ───

class ReachSet2Options:
    @property
    def lp_feas_tol(self) -> float: ...
    @property
    def a_cmp_abs_tol(self) -> float: ...
    @property
    def a_cmp_rel_tol(self) -> float: ...
    @property
    def verbosity(self) -> Verbosity: ...

    def __init__(
        self,
        a_cmp_abs_tol: Optional[float] = None,
        a_cmp_rel_tol: Optional[float] = None,
        lp_feas_tol: Optional[float] = None,
        verbosity: Verbosity = Verbosity.Silent,
    ) -> None: ...

class ClarabelSettings:
    max_iter: int
    time_limit: float
    verbose: bool
    max_step_fraction: float
    tol_gap_abs: float
    tol_gap_rel: float
    tol_feas: float
    tol_infeas_abs: float
    tol_infeas_rel: float
    tol_ktratio: float
    reduced_tol_gap_abs: float
    reduced_tol_gap_rel: float
    reduced_tol_feas: float
    reduced_tol_infeas_abs: float
    reduced_tol_infeas_rel: float
    reduced_tol_ktratio: float
    equilibrate_enable: bool
    equilibrate_max_iter: int
    equilibrate_min_scaling: float
    equilibrate_max_scaling: float
    linesearch_backtrack_step: float
    min_switch_step_length: float
    min_terminate_step_length: float
    max_threads: int
    direct_kkt_solver: bool
    direct_solve_method: DirectSolveMethod
    static_regularization_enable: bool
    static_regularization_constant: float
    static_regularization_proportional: float
    dynamic_regularization_enable: bool
    dynamic_regularization_eps: float
    dynamic_regularization_delta: float
    iterative_refinement_enable: bool
    iterative_refinement_reltol: float
    iterative_refinement_abstol: float
    iterative_refinement_max_iter: int
    iterative_refinement_stop_ratio: float
    presolve_enable: bool
    input_sparse_dropzeros: bool

    def __init__(self) -> None: ...
    @staticmethod
    def default() -> "ClarabelSettings": ...

class ClarabelOptions:
    @property
    def verbosity(self) -> Verbosity: ...
    @property
    def settings(self) -> ClarabelSettings: ...
    @property
    def allow_almost_solved(self) -> bool: ...
    @property
    def allow_max_iterations(self) -> bool: ...
    @property
    def allow_max_time(self) -> bool: ...
    @property
    def allow_insufficient_progress(self) -> bool: ...
    @property
    def allow_callback_terminated(self) -> bool: ...

    def __init__(
        self,
        verbosity: Verbosity = Verbosity.Silent,
        allow_almost_solved: bool = True,
        allow_max_iterations: bool = False,
        allow_max_time: bool = False,
        allow_insufficient_progress: bool = False,
        allow_callback_terminated: bool = False,
        settings: Optional[ClarabelSettings] = None,
    ) -> None: ...

def set_verbosity_output(output: str, path: Optional[str] = None) -> None: ...
def set_verbosity_log_file(path: str) -> None: ...
def verbosity_output() -> tuple[str, Optional[str]]: ...

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

# ─── Solver Diagnostics ───

class Topp3Profile:
    @property
    def a(self) -> Array: ...
    @property
    def b(self) -> Array: ...
    @property
    def num_stationary(self) -> tuple[int, int]: ...

    def __init__(self, a: Array, b: Array, num_stationary: tuple[int, int]) -> None: ...
    def as_tuple(self) -> tuple[Array, Array, tuple[int, int]]: ...

class ClarabelSolution:
    @property
    def x(self) -> Array: ...
    @property
    def z(self) -> Array: ...
    @property
    def s(self) -> Array: ...
    @property
    def status(self) -> ClarabelSolverStatus: ...
    @property
    def obj_val(self) -> float: ...
    @property
    def obj_val_dual(self) -> float: ...
    @property
    def solve_time(self) -> float: ...
    @property
    def iterations(self) -> int: ...
    @property
    def r_prim(self) -> float: ...
    @property
    def r_dual(self) -> float: ...
    @property
    def objective_value(self) -> float: ...
    @property
    def objective_terms(self) -> Array: ...

class LinearSolverInfo:
    name: str
    threads: int
    direct: bool
    nnz_a: int
    nnz_l: int

# ─── Solvers ───

def topp2_ra(
    robot: Robot,
    idx_s_start: int,
    idx_s_final: int,
    a_start: float,
    a_final: float,
    options: Optional[ReachSet2Options] = None,
) -> Array: ...

def reach_set2_backward(
    robot: Robot,
    idx_s_start: int,
    idx_s_final: int,
    a_start: float,
    a_final: float,
    options: Optional[ReachSet2Options] = None,
) -> tuple[Array, Array]: ...

def reach_set2_bidirectional(
    robot: Robot,
    idx_s_start: int,
    idx_s_final: int,
    a_start: float,
    a_final: float,
    options: Optional[ReachSet2Options] = None,
) -> tuple[Array, Array]: ...

def copp2_socp(
    robot: Robot,
    idx_s_start: int,
    idx_s_final: int,
    a_start: float,
    a_final: float,
    objectives: list[Objective],
    options: Optional[ClarabelOptions] = None,
) -> Array: ...

def copp2_socp_expert(
    robot: Robot,
    idx_s_start: int,
    idx_s_final: int,
    a_start: float,
    a_final: float,
    objectives: list[Objective],
    options: Optional[ClarabelOptions] = None,
) -> tuple[Optional[Array], ClarabelSolution, LinearSolverInfo]: ...

def topp3_lp(
    robot: Robot,
    idx_s_start: int,
    a_linearization: Array,
    a_start: float,
    a_final: float,
    b_start: float,
    b_final: float,
    options: Optional[ClarabelOptions] = None,
    num_stationary_max: Optional[tuple[int, int]] = None,
    a_linearization_floor: Optional[float] = None,
) -> tuple[Array, Array, tuple[int, int]]: ...

def topp3_lp_expert(
    robot: Robot,
    idx_s_start: int,
    a_linearization: Array,
    a_start: float,
    a_final: float,
    b_start: float,
    b_final: float,
    options: Optional[ClarabelOptions] = None,
    num_stationary_max: Optional[tuple[int, int]] = None,
    a_linearization_floor: Optional[float] = None,
) -> tuple[Optional[Topp3Profile], ClarabelSolution, LinearSolverInfo]: ...

def topp3_socp(
    robot: Robot,
    idx_s_start: int,
    a_linearization: Array,
    a_start: float,
    a_final: float,
    b_start: float,
    b_final: float,
    options: Optional[ClarabelOptions] = None,
    num_stationary_max: Optional[tuple[int, int]] = None,
    a_linearization_floor: Optional[float] = None,
) -> tuple[Array, Array, tuple[int, int]]: ...

def topp3_socp_expert(
    robot: Robot,
    idx_s_start: int,
    a_linearization: Array,
    a_start: float,
    a_final: float,
    b_start: float,
    b_final: float,
    options: Optional[ClarabelOptions] = None,
    num_stationary_max: Optional[tuple[int, int]] = None,
    a_linearization_floor: Optional[float] = None,
) -> tuple[Optional[Topp3Profile], ClarabelSolution, LinearSolverInfo]: ...

def copp3_socp(
    robot: Robot,
    idx_s_start: int,
    a_linearization: Array,
    a_start: float,
    a_final: float,
    b_start: float,
    b_final: float,
    objectives: list[Objective],
    options: Optional[ClarabelOptions] = None,
    num_stationary_max: Optional[tuple[int, int]] = None,
    a_linearization_floor: Optional[float] = None,
) -> tuple[Array, Array, tuple[int, int]]: ...

def copp3_socp_expert(
    robot: Robot,
    idx_s_start: int,
    a_linearization: Array,
    a_start: float,
    a_final: float,
    b_start: float,
    b_final: float,
    objectives: list[Objective],
    options: Optional[ClarabelOptions] = None,
    num_stationary_max: Optional[tuple[int, int]] = None,
    a_linearization_floor: Optional[float] = None,
) -> tuple[Optional[Topp3Profile], ClarabelSolution, LinearSolverInfo]: ...

# ─── Interpolation ───

def s_to_t_topp2(
    s: Array,
    a: Array,
    t_offset: float = 0.0,
) -> tuple[float, Array]: ...

def t_to_s_topp2(
    s: Array,
    a: Array,
    t_s: Array,
    dt: Optional[float] = None,
    t_samples: Optional[Array] = None,
    t0: float = 0.0,
    include_final: bool = True,
) -> Array: ...

def a_to_b_topp2(
    s: Array,
    a: Array,
) -> Array: ...

def s_to_t_topp3(
    s: Array,
    a: Array,
    b: Array,
    num_stationary: tuple[int, int],
    t_offset: float = 0.0,
) -> tuple[float, Array]: ...

def t_to_s_topp3(
    s: Array,
    a: Array,
    b: Array,
    num_stationary: tuple[int, int],
    t_s: Array,
    dt: Optional[float] = None,
    t_samples: Optional[Array] = None,
    t0: float = 0.0,
    include_final: bool = True,
) -> Array: ...

def force_positive_a_3rd(
    s: Array,
    a: Array,
    b: Array,
    num_stationary: tuple[int, int],
    a_min: float,
) -> bool: ...
