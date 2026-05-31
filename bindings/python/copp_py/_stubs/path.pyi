from __future__ import annotations

from typing import Any, Callable, ClassVar, Literal, Protocol, TypedDict, overload

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .core import MatrixLayout, MatrixLayoutLike, OutOfRangeMode, OutOfRangeModeLike, Parametrization, ParametrizationLike

class _PathEvaluator2nd(Protocol):
    """Static protocol for ``Path.from_evaluator_2nd``.

    At runtime ``dim`` may be either a positive integer attribute or a
    zero-argument method returning a positive integer. ``evaluate_q`` is
    optional and may be implemented when position-only evaluation is cheaper
    than computing all second-order derivatives.
    """

    dim: int | Callable[[], int]

    def evaluate_up_to_2nd(
        self,
        s: NDArray[np.float64],
    ) -> tuple[ArrayLike, ArrayLike, ArrayLike]:
        """Return ``(q, dq, ddq)`` for path parameters ``s``."""
        ...


class _PathEvaluator3rd(Protocol):
    """Static protocol for ``Path.from_evaluator_3rd`` and ``from_evaluator``.

    At runtime ``dim`` may be either a positive integer attribute or a
    zero-argument method returning a positive integer. ``evaluate_q`` and
    ``evaluate_up_to_2nd`` are optional. If ``evaluate_up_to_2nd`` is omitted,
    second-order evaluation falls back to ``evaluate_up_to_3rd`` and discards
    ``dddq``; this is supported for convenience but not recommended for
    performance-sensitive code.
    """

    dim: int | Callable[[], int]

    def evaluate_up_to_3rd(
        self,
        s: NDArray[np.float64],
    ) -> tuple[
        ArrayLike,
        ArrayLike,
        ArrayLike,
        ArrayLike,
    ]:
        """Return ``(q, dq, ddq, dddq)`` for path parameters ``s``."""
        ...


class SplineConfig:
    """Configuration for waypoint-spline path construction.

    The defaults build a quintic spline over ``s in [0, 1]`` with uniform
    waypoint spacing, zero endpoint derivatives, and out-of-range errors.

    Parameters
    ----------
    order:
        Odd spline order, at least 3. Common values are 3, 5, and 7.
    s_min:
        Lower endpoint of the path parameter range.
    s_max:
        Upper endpoint of the path parameter range. Must be greater than
        ``s_min`` when building a path.
    out_of_range:
        Evaluation policy outside ``[s_min, s_max]``. Strings are accepted for
        script-friendly use; enum values are preferred for stable APIs.
    parametrization:
        Waypoint parameter assignment policy. Only uniform spacing is currently
        supported.
    start_state, end_state:
        Optional ArrayLike matrices convertible to ``float64`` with shape
        ``(dim, (order - 1) // 2)``. Column ``r - 1`` stores the ``r``-th
        derivative at the corresponding endpoint. ``None`` means zero endpoint
        derivatives.

    Raises
    ------
    ValueError
        If a wrapper-level option or boundary-state array format is invalid.
    """

    order: int
    s_min: float
    s_max: float
    out_of_range: OutOfRangeMode
    parametrization: Parametrization
    start_state: NDArray[np.float64] | None
    end_state: NDArray[np.float64] | None

    def __init__(
        self,
        *,
        order: int = 5,
        s_min: float = 0.0,
        s_max: float = 1.0,
        out_of_range: OutOfRangeModeLike = OutOfRangeMode.ERROR,
        parametrization: ParametrizationLike = Parametrization.UNIFORM,
        start_state: ArrayLike | None = None,
        end_state: ArrayLike | None = None,
    ) -> None:
        ...


class PathDerivatives:
    """Batch path-evaluation result.

    Fields that were not requested by the evaluation method are ``None``.
    Array shape follows the ``layout`` used when constructing the path:

    - ``MatrixLayout.SAMPLE_MAJOR``: ``(n_samples, dim)``;
    - ``MatrixLayout.DIM_MAJOR``: ``(dim, n_samples)``.
    """

    q: NDArray[np.float64]
    dq: NDArray[np.float64] | None
    ddq: NDArray[np.float64] | None
    dddq: NDArray[np.float64] | None


class Path:
    """Unified path object backed by the Rust COPP core."""

    dim: int
    """Number of path dimensions."""

    s_range: tuple[float, float]
    """Inclusive valid path-parameter range ``(s_min, s_max)``."""

    @staticmethod
    @overload
    def from_waypoints(
        waypoints: ArrayLike,
        config: SplineConfig,
        *,
        layout: MatrixLayoutLike = MatrixLayout.SAMPLE_MAJOR,
    ) -> Path:
        """Build a waypoint-spline path from an explicit ``SplineConfig``.

        This overload mirrors the Rust API shape most closely: all spline
        options are carried by ``config``, while ``layout`` only controls how
        Python interprets the waypoint matrix and returned derivative arrays.

        With the default ``MatrixLayout.SAMPLE_MAJOR``, ``waypoints`` has shape
        ``(n_points, dim)`` and each row is one waypoint. With
        ``MatrixLayout.DIM_MAJOR``, ``waypoints`` has shape ``(dim, n_points)``
        and each column is one waypoint.

        Parameters
        ----------
        waypoints:
            Two-dimensional ArrayLike value convertible to ``float64``.
        config:
            Spline construction options.
        layout:
            Matrix layout for both input waypoints and returned derivative
            arrays. Strings ``"sample_major"`` and ``"dim_major"`` are
            accepted.

        Raises
        ------
        ValueError
            If array layout, dtype, or wrapper-level options are invalid.
        CoppError
            If the Rust COPP core rejects the path data or spline options.
        """
        ...

    @staticmethod
    @overload
    def from_waypoints(
        waypoints: ArrayLike,
        config: None = None,
        *,
        order: int = 5,
        s_min: float = 0.0,
        s_max: float = 1.0,
        out_of_range: OutOfRangeModeLike = OutOfRangeMode.ERROR,
        parametrization: ParametrizationLike = Parametrization.UNIFORM,
        start_state: ArrayLike | None = None,
        end_state: ArrayLike | None = None,
        layout: MatrixLayoutLike = MatrixLayout.SAMPLE_MAJOR,
    ) -> Path:
        """Build a waypoint-spline path.

        With the default ``MatrixLayout.SAMPLE_MAJOR``, ``waypoints`` has shape
        ``(n_points, dim)`` and each row is one waypoint. With
        ``MatrixLayout.DIM_MAJOR``, ``waypoints`` has shape ``(dim, n_points)``
        and each column is one waypoint.

        Parameters
        ----------
        waypoints:
            Two-dimensional ArrayLike value convertible to ``float64``.
        config:
            Optional spline construction options. If omitted, keyword
            arguments build an equivalent temporary ``SplineConfig``.
        order, s_min, s_max, out_of_range, parametrization, start_state, end_state:
            Spline construction options used only when ``config`` is omitted.
        layout:
            Matrix layout for both input waypoints and returned derivative
            arrays. Strings ``"sample_major"`` and ``"dim_major"`` are
            accepted.

        Raises
        ------
        ValueError
            If array layout, dtype, wrapper-level options, or mixed
            ``config``/keyword options are invalid.
        CoppError
            If the Rust COPP core rejects the path data or spline options.
        """
        ...

    @staticmethod
    def from_jax(
        q_fn: Callable[[Any], Any],
        s_min: float,
        s_max: float,
        *,
        layout: MatrixLayoutLike = MatrixLayout.SAMPLE_MAJOR,
        jit: bool = True,
        require_x64: bool = True,
    ) -> Path:
        """Build a path from a scalar-parametric Python callable using JAX.

        ``from_jax`` is a high-level Python convenience constructor. It
        differentiates ``q_fn(s)`` with JAX, wraps the generated derivatives as
        a Python evaluator, and delegates to ``Path.from_evaluator_3rd``. The
        Rust core still sees the same explicit ``(q, dq, ddq, dddq)`` evaluator
        protocol.

        Parameters
        ----------
        q_fn:
            Callable that receives one scalar path parameter and returns one
            path vector. Use ``jax.numpy`` operations so JAX can trace it.
        s_min, s_max:
            Valid path-parameter range. Parametric paths currently match the
            Rust evaluator API and reject out-of-range queries.
        layout:
            Matrix layout used for returned derivative arrays.
        jit:
            JIT-compile the JAX batched evaluator.
        require_x64:
            Require JAX 64-bit mode before construction.

        Examples
        --------
        Enable x64 before constructing the path when using the default
        ``require_x64=True``::

            import numpy as np
            import jax
            import jax.numpy as jnp
            import copp_py as copp

            jax.config.update("jax_enable_x64", True)

            def q_fn(s):
                return jnp.array([jnp.sin(s), jnp.cos(s), s + 0.1 * s**2])

            path = copp.Path.from_jax(q_fn, 0.0, 1.0)
            samples = np.array([0.0, 0.5, 1.0], dtype=np.float64)
            out = path.evaluate_up_to_3rd(samples)

            q = out.q
            dq = out.dq
            ddq = out.ddq
            dddq = out.dddq

        Raises
        ------
        ImportError
            If the selected optional dependency is missing. Install, for
            example, with ``pip install "copp-py[jax]"``.
        ValueError
            If the JAX configuration or callback output shape is invalid.
        CoppError
            If the Rust COPP core rejects the path range or evaluator
            dimension.
        """
        ...

    @staticmethod
    def from_autograd(
        q_fn: Callable[[Any], Any],
        s_min: float,
        s_max: float,
        *,
        layout: MatrixLayoutLike = MatrixLayout.SAMPLE_MAJOR,
    ) -> Path:
        """Build a path from a scalar-parametric Python callable using Autograd.

        This is the lightweight Python AD alternative to ``Path.from_jax``. It
        differentiates ``q_fn(s)`` with Autograd, wraps the generated
        derivatives as a Python evaluator, and delegates to
        ``Path.from_evaluator_3rd``.

        Parameters
        ----------
        q_fn:
            Callable that receives one scalar path parameter and returns one
            path vector. Use ``autograd.numpy`` operations so Autograd can
            trace it.
        s_min, s_max:
            Valid path-parameter range. Autograd-backed paths currently match
            the Rust evaluator API and reject out-of-range queries.
        layout:
            Matrix layout used for returned derivative arrays.

        Examples
        --------
        The callable should use ``autograd.numpy`` rather than ordinary
        ``numpy`` so Autograd can trace scalar operations::

            import numpy as np
            import autograd.numpy as anp
            import copp_py as copp

            def q_fn(s):
                return anp.array([anp.sin(s), anp.cos(s), s + 0.1 * s**2])

            path = copp.Path.from_autograd(q_fn, 0.0, 1.0)
            out = path.evaluate_up_to_3rd(
                np.array([0.0, 0.5, 1.0], dtype=np.float64)
            )

        Raises
        ------
        ImportError
            If Autograd is missing. Install it with
            ``pip install "copp-py[autograd]"``.
        ValueError
            If the callback output shape is invalid.
        CoppError
            If the Rust COPP core rejects the path range or evaluator
            dimension.
        """
        ...

    @staticmethod
    def from_casadi(
        q_expr: Any,
        *,
        symbol: Any,
        s_min: float = 0.0,
        s_max: float = 1.0,
        layout: MatrixLayoutLike = MatrixLayout.SAMPLE_MAJOR,
    ) -> Path:
        """Build a path from a scalar-parameter CasADi expression.

        This constructor differentiates the CasADi expression in Python and
        delegates the resulting explicit derivative evaluator to the Rust core.

        Parameters
        ----------
        q_expr:
            CasADi ``SX``/``MX`` expression, vector expression, or sequence of
            expressions describing ``q(s)``.
        symbol:
            Scalar CasADi ``SX`` or ``MX`` symbol used as the path parameter.
        s_min, s_max:
            Valid path-parameter range.
        layout:
            Matrix layout used for returned derivative arrays.

        Examples
        --------
        Use a scalar ``SX`` or ``MX`` symbol as the path parameter, then pass a
        vector expression for ``q(s)``::

            import numpy as np
            import casadi as ca
            import copp_py as copp

            s = ca.SX.sym("s")
            q_expr = ca.vertcat(ca.sin(s), ca.cos(s), s + 0.1 * s**2)

            path = copp.Path.from_casadi(
                q_expr,
                symbol=s,
                s_min=0.0,
                s_max=1.0,
            )
            out = path.evaluate_up_to_3rd(
                np.array([0.0, 0.5, 1.0], dtype=np.float64)
            )

        Raises
        ------
        ImportError
            If CasADi is missing. Install it with
            ``pip install "copp-py[casadi]"``.
        ValueError
            If ``symbol`` is not scalar or ``q_expr`` is empty.
        CoppError
            If the Rust COPP core rejects the path range or evaluator
            dimension.
        """
        ...

    @staticmethod
    def from_sympy(
        q_exprs: Any,
        *,
        symbol: Any,
        s_min: float = 0.0,
        s_max: float = 1.0,
        layout: MatrixLayoutLike = MatrixLayout.SAMPLE_MAJOR,
    ) -> Path:
        """Build a path from scalar-parameter SymPy expressions.

        SymPy expressions are differentiated symbolically and lambdified with
        NumPy before the resulting evaluator is passed to the Rust core.

        Parameters
        ----------
        q_exprs:
            A SymPy expression, sequence, or matrix describing ``q(s)``.
        symbol:
            Scalar SymPy symbol used as the path parameter.
        s_min, s_max:
            Valid path-parameter range.
        layout:
            Matrix layout used for returned derivative arrays.

        Examples
        --------
        Pass a SymPy symbol and one expression per path dimension. Expressions
        are differentiated symbolically and evaluated with NumPy::

            import numpy as np
            import sympy as sp
            import copp_py as copp

            s = sp.symbols("s")
            q_exprs = [
                sp.sin(s),
                sp.cos(s),
                s + sp.Rational(1, 10) * s**2,
            ]

            path = copp.Path.from_sympy(
                q_exprs,
                symbol=s,
                s_min=0.0,
                s_max=1.0,
            )
            out = path.evaluate_up_to_3rd(
                np.array([0.0, 0.5, 1.0], dtype=np.float64)
            )

        Raises
        ------
        ImportError
            If SymPy is missing. Install it with
            ``pip install "copp-py[sympy]"``.
        ValueError
            If ``q_exprs`` is empty or evaluates to an unexpected shape.
        CoppError
            If the Rust COPP core rejects the path range or evaluator
            dimension.
        """
        ...

    @staticmethod
    def from_evaluator_2nd(
        evaluator: _PathEvaluator2nd,
        s_min: float,
        s_max: float,
        *,
        layout: MatrixLayoutLike = MatrixLayout.SAMPLE_MAJOR,
    ) -> Path:
        """Build a path from a Python evaluator with derivatives up to second order.

        The evaluator must provide ``dim`` and
        ``evaluate_up_to_2nd(s) -> (q, dq, ddq)``. ``evaluate_q(s) -> q`` is
        optional and used when present. The callback receives ``s`` as a
        one-dimensional contiguous ``float64`` array. Returned values may be
        any ArrayLike convertible to two-dimensional ``float64`` arrays with
        shape determined by ``layout``:

        - ``MatrixLayout.SAMPLE_MAJOR``: ``(n_samples, dim)``;
        - ``MatrixLayout.DIM_MAJOR``: ``(dim, n_samples)``.

        Evaluator-backed paths currently match the Rust API and reject
        out-of-range queries; no ``out_of_range`` option is exposed here.

        Raises
        ------
        ValueError
            If the evaluator protocol or callback return arrays are invalid.
        CoppError
            If the Rust COPP core rejects the path range or evaluator
            dimension.
        Exception
            Any exception raised by the Python evaluator callback is propagated
            unchanged.
        """
        ...

    @staticmethod
    def from_evaluator_3rd(
        evaluator: _PathEvaluator3rd,
        s_min: float,
        s_max: float,
        *,
        layout: MatrixLayoutLike = MatrixLayout.SAMPLE_MAJOR,
    ) -> Path:
        """Build a path from a Python evaluator with derivatives up to third order.

        The evaluator must provide ``dim`` and
        ``evaluate_up_to_3rd(s) -> (q, dq, ddq, dddq)``. ``evaluate_q`` and
        ``evaluate_up_to_2nd`` are optional. If ``evaluate_up_to_2nd`` is
        omitted, second-order evaluation calls ``evaluate_up_to_3rd`` and
        discards ``dddq``; implement the lower-order callback too when this is
        performance-sensitive.

        Callback array layout and error behavior are the same as
        ``from_evaluator_2nd``.
        """
        ...

    @staticmethod
    def from_evaluator(
        evaluator: _PathEvaluator3rd,
        s_min: float,
        s_max: float,
        *,
        layout: MatrixLayoutLike = MatrixLayout.SAMPLE_MAJOR,
    ) -> Path:
        """Compatibility alias for ``Path.from_evaluator_3rd``.

        New code should prefer ``from_evaluator_2nd`` or
        ``from_evaluator_3rd`` so the supported derivative order is explicit.
        """
        ...

    def evaluate_q(self, s: ArrayLike) -> PathDerivatives:
        """Evaluate position ``q`` only at path parameters ``s``.

        ``s`` may be any one-dimensional ArrayLike value convertible to
        ``float64``. The returned ``PathDerivatives`` has ``q`` populated and
        derivative fields set to ``None``.
        """
        ...

    def evaluate_up_to_2nd(self, s: ArrayLike) -> PathDerivatives:
        """Evaluate ``q``, ``dq``, and ``ddq`` at path parameters ``s``."""
        ...

    def evaluate_up_to_3rd(self, s: ArrayLike) -> PathDerivatives:
        """Evaluate ``q``, ``dq``, ``ddq``, and ``dddq`` at path parameters ``s``."""
        ...


