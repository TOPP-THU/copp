from __future__ import annotations

from typing import Any, Callable, ClassVar, Literal, Protocol, TypedDict, overload

import numpy as np
from numpy.typing import NDArray

from .core import ClarabelDirectSolveMethod, ClarabelDirectSolveMethodLike, ClarabelSolverStatus, Verbosity, VerbosityLike
from .interpolation import Profile3rd

class ClarabelLinearSolverInfo:
    """Linear-solver metadata reported by Clarabel."""

    method: ClarabelDirectSolveMethod
    threads: int
    direct: bool
    nnz_a: int
    nnz_l: int


class ClarabelSettings:
    """Raw Clarabel numerical settings.

    Defaults mirror COPP's finalized Rust Clarabel policy:
    Clarabel defaults, ``equilibrate_max_iter=20``, and ``verbose=False`` for
    the default crate-level verbosity.
    """

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
    direct_solve_method: ClarabelDirectSolveMethod
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

    def __init__(
        self,
        *,
        max_iter: int = 200,
        time_limit: float = np.inf,
        verbose: bool = False,
        max_step_fraction: float = 0.99,
        tol_gap_abs: float = 1.0e-8,
        tol_gap_rel: float = 1.0e-8,
        tol_feas: float = 1.0e-8,
        tol_infeas_abs: float = 1.0e-8,
        tol_infeas_rel: float = 1.0e-8,
        tol_ktratio: float = 1.0e-6,
        reduced_tol_gap_abs: float = 5.0e-5,
        reduced_tol_gap_rel: float = 5.0e-5,
        reduced_tol_feas: float = 1.0e-4,
        reduced_tol_infeas_abs: float = 5.0e-12,
        reduced_tol_infeas_rel: float = 5.0e-5,
        reduced_tol_ktratio: float = 1.0e-4,
        equilibrate_enable: bool = True,
        equilibrate_max_iter: int = 20,
        equilibrate_min_scaling: float = 1.0e-4,
        equilibrate_max_scaling: float = 1.0e4,
        linesearch_backtrack_step: float = 0.8,
        min_switch_step_length: float = 0.1,
        min_terminate_step_length: float = 1.0e-4,
        max_threads: int = 0,
        direct_kkt_solver: bool = True,
        direct_solve_method: ClarabelDirectSolveMethodLike = ClarabelDirectSolveMethod.AUTO,
        static_regularization_enable: bool = True,
        static_regularization_constant: float = 1.0e-8,
        static_regularization_proportional: float = 4.930380657631324e-32,
        dynamic_regularization_enable: bool = True,
        dynamic_regularization_eps: float = 1.0e-13,
        dynamic_regularization_delta: float = 2.0e-7,
        iterative_refinement_enable: bool = True,
        iterative_refinement_reltol: float = 1.0e-13,
        iterative_refinement_abstol: float = 1.0e-12,
        iterative_refinement_max_iter: int = 10,
        iterative_refinement_stop_ratio: float = 5.0,
        presolve_enable: bool = True,
        input_sparse_dropzeros: bool = False,
    ) -> None:
        """Construct raw Clarabel numerical settings."""
        ...


class ClarabelOptions:
    """Shared options for Clarabel-backed SOCP solvers.

    Omitted fields use Rust ``ClarabelOptionsBuilder::new()`` defaults:
    ``verbosity=Verbosity.SILENT``, ``allow_almost_solved=True``,
    ``allow_max_iterations=False``, ``allow_max_time=False``,
    ``allow_callback_terminated=False``,
    ``allow_insufficient_progress=False``, and default
    ``ClarabelSettings``.
    """

    verbosity: Verbosity
    """Diagnostic verbosity level. Rust default: ``Verbosity.SILENT``."""

    allow_almost_solved: bool
    """Accept Clarabel ``AlmostSolved`` as usable. Rust default: ``True``."""

    allow_max_iterations: bool
    """Accept Clarabel ``MaxIterations`` as usable. Rust default: ``False``."""

    allow_max_time: bool
    """Accept Clarabel ``MaxTime`` as usable. Rust default: ``False``."""

    allow_callback_terminated: bool
    """Accept Clarabel ``CallbackTerminated`` as usable. Rust default: ``False``."""

    allow_insufficient_progress: bool
    """Accept Clarabel ``InsufficientProgress`` as usable. Rust default: ``False``."""

    clarabel_settings: ClarabelSettings
    """Raw Clarabel numerical settings.

    The getter returns a copy. Mutating that copy does not affect this options
    object until it is assigned back to ``options.clarabel_settings``.
    """

    def __init__(
        self,
        *,
        verbosity: VerbosityLike = Verbosity.SILENT,
        allow_almost_solved: bool = True,
        allow_max_iterations: bool = False,
        allow_max_time: bool = False,
        allow_callback_terminated: bool = False,
        allow_insufficient_progress: bool = False,
        clarabel_settings: ClarabelSettings | None = None,
    ) -> None:
        """Construct Clarabel-backed solver options.

        ``clarabel_settings=None`` means COPP's Rust default Clarabel settings.
        When ``verbosity`` is ``SILENT`` or ``SUMMARY``, Rust's builder policy
        disables Clarabel internal logging even if ``clarabel_settings.verbose``
        is true.
        """
        ...


class Copp2SocpResult:
    """Expert COPP2-SOCP result with raw Clarabel diagnostics.

    ``a``, ``objective_value``, and ``objective_terms`` are ``None`` when the
    solver status is not accepted by the supplied ``ClarabelOptions``. Raw
    Clarabel vectors remain available for diagnostics either way.
    """

    a: NDArray[np.float64] | None
    """Accepted node profile, or ``None`` when unavailable."""

    x: NDArray[np.float64]
    """Raw Clarabel primal solution vector."""

    z: NDArray[np.float64]
    """Raw Clarabel dual solution vector."""

    s: NDArray[np.float64]
    """Raw Clarabel primal-cone slack vector."""

    solver_status: ClarabelSolverStatus
    """Final Clarabel solver status."""

    obj_val: float
    """Primal objective value reported by Clarabel."""

    obj_val_dual: float
    """Dual objective value reported by Clarabel."""

    solve_time: float
    """Clarabel solve time in seconds."""

    iterations: int
    """Number of interior-point iterations."""

    r_prim: float
    """Primal residual reported by Clarabel."""

    r_dual: float
    """Dual residual reported by Clarabel."""

    linsolver: ClarabelLinearSolverInfo
    """Linear-solver metadata reported by Clarabel."""

    objective_value: float | None
    """Weighted COPP objective value computed from accepted ``a``, if any."""

    objective_terms: NDArray[np.float64] | None
    """Per-objective unweighted values computed from accepted ``a``, if any."""


class Copp3ClarabelResult:
    """Expert result for third-order Clarabel solvers.

    This is the Python-facing name for the result shape shared by
    ``topp3_lp_expert``, ``topp3_socp_expert``, and ``copp3_socp_expert``.
    ``profile`` is ``None`` when the solver status is not accepted by the
    supplied ``ClarabelOptions``. For TOPP3-LP/SOCP, there is no COPP objective
    list, so ``objective_value`` and ``objective_terms`` are ``None``.
    """

    profile: Profile3rd | None
    """Accepted profile, or ``None`` when unavailable."""

    x: NDArray[np.float64]
    """Raw Clarabel primal solution vector."""

    z: NDArray[np.float64]
    """Raw Clarabel dual solution vector."""

    s: NDArray[np.float64]
    """Raw Clarabel primal-cone slack vector."""

    solver_status: ClarabelSolverStatus
    """Final Clarabel solver status."""

    obj_val: float
    """Primal objective value reported by Clarabel."""

    obj_val_dual: float
    """Dual objective value reported by Clarabel."""

    solve_time: float
    """Clarabel solve time in seconds."""

    iterations: int
    """Number of interior-point iterations."""

    r_prim: float
    """Primal residual reported by Clarabel."""

    r_dual: float
    """Dual residual reported by Clarabel."""

    linsolver: ClarabelLinearSolverInfo
    """Linear-solver metadata reported by Clarabel."""

    objective_value: float | None
    """Weighted COPP objective value, if available for COPP3 solvers."""

    objective_terms: NDArray[np.float64] | None
    """Per-objective unweighted values, if available for COPP3 solvers."""

