"""Optional parametric-path constructors built on Python AD backends.

This module is intentionally imported lazily by ``copp.Path`` methods. The
default installation only depends on NumPy; optional AD libraries are requested
with extras such as ``copp-py[jax]`` when users call the corresponding
constructor.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import Any

import numpy as np
from numpy.typing import NDArray

from . import _native


_SAMPLE_MAJOR = "sample_major"
_DIM_MAJOR = "dim_major"


def _from_jax(
    q_fn: Callable[[Any], Any],
    s_min: float,
    s_max: float,
    layout: str,
    jit: bool,
    require_x64: bool,
) -> _native.Path:
    """Build a native path from a Python callable using JAX."""
    layout = _normalize_layout(layout)
    evaluator = _JaxParametricEvaluator(
        q_fn,
        layout=layout,
        jit=jit,
        require_x64=require_x64,
        s_probe=0.5 * (s_min + s_max),
    )

    return _native.Path.from_evaluator_3rd(
        evaluator,
        s_min,
        s_max,
        layout=layout,
    )


def _from_autograd(
    q_fn: Callable[[Any], Any],
    s_min: float,
    s_max: float,
    layout: str,
) -> _native.Path:
    """Build a native path from a Python callable using Autograd."""
    layout = _normalize_layout(layout)
    evaluator = _AutogradParametricEvaluator(
        q_fn,
        layout=layout,
        s_probe=0.5 * (s_min + s_max),
    )
    return _native.Path.from_evaluator_3rd(
        evaluator,
        s_min,
        s_max,
        layout=layout,
    )


def _from_casadi(
    q_expr: Any,
    symbol: Any,
    s_min: float,
    s_max: float,
    layout: str,
) -> _native.Path:
    """Build a native path from a CasADi scalar-parameter expression."""
    layout = _normalize_layout(layout)
    evaluator = _CasadiParametricEvaluator(q_expr, symbol, layout=layout)
    return _native.Path.from_evaluator_3rd(
        evaluator,
        s_min,
        s_max,
        layout=layout,
    )


def _from_sympy(
    q_exprs: Any,
    symbol: Any,
    s_min: float,
    s_max: float,
    layout: str,
) -> _native.Path:
    """Build a native path from SymPy scalar-parameter expressions."""
    layout = _normalize_layout(layout)
    evaluator = _SympyParametricEvaluator(q_exprs, symbol, layout=layout)
    return _native.Path.from_evaluator_3rd(
        evaluator,
        s_min,
        s_max,
        layout=layout,
    )


class _JaxParametricEvaluator:
    """JAX-backed evaluator for scalar-parametric vector paths."""

    def __init__(
        self,
        q_fn: Callable[[Any], Any],
        *,
        layout: str,
        jit: bool,
        require_x64: bool,
        s_probe: float,
    ) -> None:
        jax = _import_optional(
            "jax",
            'JAX backend is not installed. Install it with `pip install "copp-py[jax]"`.',
        )
        jnp = _import_optional(
            "jax.numpy",
            'JAX backend is not installed. Install it with `pip install "copp-py[jax]"`.',
        )

        if require_x64 and not _jax_x64_enabled(jax):
            raise ValueError(
                "JAX backend requires 64-bit mode. Enable it before creating "
                'the path with `jax.config.update("jax_enable_x64", True)`, '
                "or pass `require_x64=False`."
            )

        self._jax = jax
        self._jnp = jnp
        self._q_fn = q_fn
        self._layout = layout

        def q_vector(s: Any) -> Any:
            q = jnp.asarray(q_fn(s), dtype=jnp.float64)
            if q.ndim == 0:
                q = q.reshape((1,))
            return jnp.ravel(q)

        dq_vector = jax.jacfwd(q_vector)
        ddq_vector = jax.jacfwd(dq_vector)
        dddq_vector = jax.jacfwd(ddq_vector)

        def batch_eval(s_values: Any) -> tuple[Any, Any, Any, Any]:
            return jax.vmap(
                lambda s: (
                    q_vector(s),
                    dq_vector(s),
                    ddq_vector(s),
                    dddq_vector(s),
                )
            )(s_values)

        self._batch_eval = jax.jit(batch_eval) if jit else batch_eval
        sample = np.asarray(q_vector(jnp.asarray(s_probe, dtype=jnp.float64)), dtype=np.float64)
        self.dim = _positive_dim_from_sample(sample, "q_fn")

    def evaluate_q(self, s: NDArray[np.float64]) -> NDArray[np.float64]:
        q, _, _, _ = self._evaluate(s)
        return q

    def evaluate_up_to_2nd(
        self,
        s: NDArray[np.float64],
    ) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        q, dq, ddq, _ = self._evaluate(s)
        return q, dq, ddq

    def evaluate_up_to_3rd(
        self,
        s: NDArray[np.float64],
    ) -> tuple[
        NDArray[np.float64],
        NDArray[np.float64],
        NDArray[np.float64],
        NDArray[np.float64],
    ]:
        return self._evaluate(s)

    def _evaluate(
        self,
        s: NDArray[np.float64],
    ) -> tuple[
        NDArray[np.float64],
        NDArray[np.float64],
        NDArray[np.float64],
        NDArray[np.float64],
    ]:
        s_jax = self._jnp.asarray(s, dtype=self._jnp.float64)
        q, dq, ddq, dddq = self._batch_eval(s_jax)
        return (
            _format_sample_major_matrix(q, self.dim, s.size, self._layout, "q"),
            _format_sample_major_matrix(dq, self.dim, s.size, self._layout, "dq"),
            _format_sample_major_matrix(ddq, self.dim, s.size, self._layout, "ddq"),
            _format_sample_major_matrix(dddq, self.dim, s.size, self._layout, "dddq"),
        )


class _AutogradParametricEvaluator:
    """Autograd-backed evaluator for scalar-parametric vector paths."""

    def __init__(self, q_fn: Callable[[Any], Any], *, layout: str, s_probe: float) -> None:
        autograd = _import_optional(
            "autograd",
            'Autograd backend is not installed. Install it with `pip install "copp-py[autograd]"`.',
        )
        anp = _import_optional(
            "autograd.numpy",
            'Autograd backend is not installed. Install it with `pip install "copp-py[autograd]"`.',
        )

        self._layout = layout

        def q_vector(s: Any) -> Any:
            q = anp.array(q_fn(s))
            if q.ndim == 0:
                q = q.reshape((1,))
            return anp.ravel(q)

        self._q_vector = q_vector
        self._dq_vector = autograd.jacobian(q_vector)
        self._ddq_vector = autograd.jacobian(self._dq_vector)
        self._dddq_vector = autograd.jacobian(self._ddq_vector)

        sample = np.asarray(q_vector(float(s_probe)), dtype=np.float64)
        self.dim = _positive_dim_from_sample(sample, "q_fn")

    def evaluate_q(self, s: NDArray[np.float64]) -> NDArray[np.float64]:
        return _format_sample_major_matrix(
            _stack_scalar_function(self._q_vector, s, self.dim, "q"),
            self.dim,
            s.size,
            self._layout,
            "q",
        )

    def evaluate_up_to_2nd(
        self,
        s: NDArray[np.float64],
    ) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        q = self.evaluate_q(s)
        dq = _format_sample_major_matrix(
            _stack_scalar_function(self._dq_vector, s, self.dim, "dq"),
            self.dim,
            s.size,
            self._layout,
            "dq",
        )
        ddq = _format_sample_major_matrix(
            _stack_scalar_function(self._ddq_vector, s, self.dim, "ddq"),
            self.dim,
            s.size,
            self._layout,
            "ddq",
        )
        return q, dq, ddq

    def evaluate_up_to_3rd(
        self,
        s: NDArray[np.float64],
    ) -> tuple[
        NDArray[np.float64],
        NDArray[np.float64],
        NDArray[np.float64],
        NDArray[np.float64],
    ]:
        q, dq, ddq = self.evaluate_up_to_2nd(s)
        dddq = _format_sample_major_matrix(
            _stack_scalar_function(self._dddq_vector, s, self.dim, "dddq"),
            self.dim,
            s.size,
            self._layout,
            "dddq",
        )
        return q, dq, ddq, dddq


class _CasadiParametricEvaluator:
    """CasADi-backed evaluator for scalar-parametric vector expressions."""

    def __init__(self, q_expr: Any, symbol: Any, *, layout: str) -> None:
        ca = _import_optional(
            "casadi",
            'CasADi backend is not installed. Install it with `pip install "copp-py[casadi]"`.',
        )
        if isinstance(q_expr, Sequence) and not hasattr(q_expr, "numel"):
            q_expr = ca.vertcat(*q_expr)
        if not hasattr(q_expr, "numel"):
            q_expr = ca.vertcat(q_expr)
        if not hasattr(symbol, "numel") or int(symbol.numel()) != 1:
            raise ValueError("`symbol` must be a scalar CasADi SX or MX symbol")

        dim = int(q_expr.numel())
        if dim == 0:
            raise ValueError("`q_expr` must contain at least one expression")

        q_vec = ca.reshape(q_expr, dim, 1)
        dq_vec = ca.jacobian(q_vec, symbol)
        ddq_vec = ca.jacobian(dq_vec, symbol)
        dddq_vec = ca.jacobian(ddq_vec, symbol)

        self.dim = dim
        self._layout = layout
        self._func = ca.Function("copp_parametric_path", [symbol], [q_vec, dq_vec, ddq_vec, dddq_vec])

    def evaluate_q(self, s: NDArray[np.float64]) -> NDArray[np.float64]:
        q, _, _, _ = self._evaluate(s)
        return q

    def evaluate_up_to_2nd(
        self,
        s: NDArray[np.float64],
    ) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        q, dq, ddq, _ = self._evaluate(s)
        return q, dq, ddq

    def evaluate_up_to_3rd(
        self,
        s: NDArray[np.float64],
    ) -> tuple[
        NDArray[np.float64],
        NDArray[np.float64],
        NDArray[np.float64],
        NDArray[np.float64],
    ]:
        return self._evaluate(s)

    def _evaluate(
        self,
        s: NDArray[np.float64],
    ) -> tuple[
        NDArray[np.float64],
        NDArray[np.float64],
        NDArray[np.float64],
        NDArray[np.float64],
    ]:
        if s.size == 0:
            empty = np.empty((0, self.dim), dtype=np.float64)
            return (
                _format_sample_major_matrix(empty, self.dim, 0, self._layout, "q"),
                _format_sample_major_matrix(empty, self.dim, 0, self._layout, "dq"),
                _format_sample_major_matrix(empty, self.dim, 0, self._layout, "ddq"),
                _format_sample_major_matrix(empty, self.dim, 0, self._layout, "dddq"),
            )

        q_rows: list[NDArray[np.float64]] = []
        dq_rows: list[NDArray[np.float64]] = []
        ddq_rows: list[NDArray[np.float64]] = []
        dddq_rows: list[NDArray[np.float64]] = []

        for value in s:
            q, dq, ddq, dddq = self._func(float(value))
            q_rows.append(_casadi_output_vector(q, self.dim, "q"))
            dq_rows.append(_casadi_output_vector(dq, self.dim, "dq"))
            ddq_rows.append(_casadi_output_vector(ddq, self.dim, "ddq"))
            dddq_rows.append(_casadi_output_vector(dddq, self.dim, "dddq"))

        return (
            _format_sample_major_matrix(np.vstack(q_rows), self.dim, s.size, self._layout, "q"),
            _format_sample_major_matrix(np.vstack(dq_rows), self.dim, s.size, self._layout, "dq"),
            _format_sample_major_matrix(np.vstack(ddq_rows), self.dim, s.size, self._layout, "ddq"),
            _format_sample_major_matrix(
                np.vstack(dddq_rows),
                self.dim,
                s.size,
                self._layout,
                "dddq",
            ),
        )


class _SympyParametricEvaluator:
    """SymPy-backed evaluator for scalar-parametric vector expressions."""

    def __init__(self, q_exprs: Any, symbol: Any, *, layout: str) -> None:
        sp = _import_optional(
            "sympy",
            'SymPy backend is not installed. Install it with `pip install "copp-py[sympy]"`.',
        )
        exprs = _sympy_expr_list(sp, q_exprs)
        if not exprs:
            raise ValueError("`q_exprs` must contain at least one expression")

        self.dim = len(exprs)
        self._layout = layout
        self._functions = [
            sp.lambdify(symbol, [sp.diff(expr, symbol, order) for expr in exprs], modules="numpy")
            for order in range(4)
        ]

    def evaluate_q(self, s: NDArray[np.float64]) -> NDArray[np.float64]:
        return _format_sample_major_matrix(
            _sympy_values_to_sample_major(self._functions[0](s), self.dim, s.size, "q"),
            self.dim,
            s.size,
            self._layout,
            "q",
        )

    def evaluate_up_to_2nd(
        self,
        s: NDArray[np.float64],
    ) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        q = self.evaluate_q(s)
        dq = _format_sample_major_matrix(
            _sympy_values_to_sample_major(self._functions[1](s), self.dim, s.size, "dq"),
            self.dim,
            s.size,
            self._layout,
            "dq",
        )
        ddq = _format_sample_major_matrix(
            _sympy_values_to_sample_major(self._functions[2](s), self.dim, s.size, "ddq"),
            self.dim,
            s.size,
            self._layout,
            "ddq",
        )
        return q, dq, ddq

    def evaluate_up_to_3rd(
        self,
        s: NDArray[np.float64],
    ) -> tuple[
        NDArray[np.float64],
        NDArray[np.float64],
        NDArray[np.float64],
        NDArray[np.float64],
    ]:
        q, dq, ddq = self.evaluate_up_to_2nd(s)
        dddq = _format_sample_major_matrix(
            _sympy_values_to_sample_major(self._functions[3](s), self.dim, s.size, "dddq"),
            self.dim,
            s.size,
            self._layout,
            "dddq",
        )
        return q, dq, ddq, dddq


def _normalize_layout(layout: str) -> str:
    token = layout.strip().lower().replace("-", "_")
    if token in {"sample_major", "samplemajor"}:
        return _SAMPLE_MAJOR
    if token in {"dim_major", "dimmajor"}:
        return _DIM_MAJOR
    raise ValueError('`layout` must be "sample_major" or "dim_major"')


def _import_optional(module_name: str, message: str) -> Any:
    try:
        return __import__(module_name, fromlist=["*"])
    except ImportError as exc:
        raise ImportError(message) from exc


def _jax_x64_enabled(jax: Any) -> bool:
    try:
        return bool(jax.config.read("jax_enable_x64"))
    except Exception:
        return bool(getattr(jax.config, "jax_enable_x64", False))


def _positive_dim_from_sample(sample: NDArray[np.float64], name: str) -> int:
    vector = np.ravel(np.asarray(sample, dtype=np.float64))
    if vector.size == 0:
        raise ValueError(f"`{name}` must return at least one path component")
    return int(vector.size)


def _stack_scalar_function(
    fn: Callable[[float], Any],
    s: NDArray[np.float64],
    dim: int,
    name: str,
) -> NDArray[np.float64]:
    if s.size == 0:
        return np.empty((0, dim), dtype=np.float64)

    rows = [_vector_from_value(fn(float(value)), dim, name) for value in s]
    return np.vstack(rows)


def _vector_from_value(value: Any, dim: int, name: str) -> NDArray[np.float64]:
    vector = np.ravel(np.asarray(value, dtype=np.float64))
    if vector.size != dim:
        raise ValueError(f"`{name}` returned {vector.size} components; expected {dim}")
    return np.ascontiguousarray(vector, dtype=np.float64)


def _format_sample_major_matrix(
    value: Any,
    dim: int,
    n_samples: int,
    layout: str,
    name: str,
) -> NDArray[np.float64]:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.ndim == 1 and dim == 1:
        matrix = matrix.reshape((n_samples, 1))
    if matrix.shape != (n_samples, dim):
        try:
            matrix = np.reshape(matrix, (n_samples, dim))
        except ValueError as exc:
            raise ValueError(
                f"`{name}` has shape {matrix.shape}; expected sample-major shape "
                f"{(n_samples, dim)}"
            ) from exc
    if matrix.shape != (n_samples, dim):
        raise ValueError(
            f"`{name}` has shape {matrix.shape}; expected sample-major shape {(n_samples, dim)}"
        )

    matrix = np.ascontiguousarray(matrix, dtype=np.float64)
    if layout == _DIM_MAJOR:
        return np.ascontiguousarray(matrix.T, dtype=np.float64)
    return matrix


def _casadi_output_vector(value: Any, dim: int, name: str) -> NDArray[np.float64]:
    vector = np.ravel(np.asarray(value, dtype=np.float64))
    if vector.size != dim:
        raise ValueError(f"CasADi `{name}` output has {vector.size} components; expected {dim}")
    return np.ascontiguousarray(vector, dtype=np.float64)


def _sympy_expr_list(sp: Any, q_exprs: Any) -> list[Any]:
    if isinstance(q_exprs, (list, tuple)):
        return list(sp.Matrix(q_exprs))
    if hasattr(q_exprs, "shape"):
        try:
            return list(sp.Matrix(q_exprs))
        except Exception:
            pass
    return [q_exprs]


def _sympy_values_to_sample_major(
    values: Any,
    dim: int,
    n_samples: int,
    name: str,
) -> NDArray[np.float64]:
    if dim == 1 and not isinstance(values, (list, tuple)):
        values = [values]
    if not isinstance(values, (list, tuple)):
        values = list(values)
    if len(values) != dim:
        raise ValueError(f"SymPy `{name}` returned {len(values)} components; expected {dim}")

    columns = [_broadcast_symbolic_component(value, n_samples, name) for value in values]
    return np.ascontiguousarray(np.stack(columns, axis=1), dtype=np.float64)


def _broadcast_symbolic_component(value: Any, n_samples: int, name: str) -> NDArray[np.float64]:
    array = np.asarray(value, dtype=np.float64)
    if array.ndim == 0:
        return np.full(n_samples, float(array), dtype=np.float64)
    array = np.ravel(array)
    if array.size != n_samples:
        raise ValueError(f"SymPy `{name}` component has {array.size} samples; expected {n_samples}")
    return np.ascontiguousarray(array, dtype=np.float64)
