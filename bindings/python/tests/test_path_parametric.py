import numpy as np
import pytest

import copp_py as copp


def test_path_from_jax_smoke():
    jax = pytest.importorskip("jax")
    jnp = pytest.importorskip("jax.numpy")
    jax.config.update("jax_enable_x64", True)

    def q_fn(s):
        return jnp.array([jnp.sin(s), jnp.cos(s), s + 0.1 * s**2])

    path = copp.Path.from_jax(
        q_fn,
        0.0,
        1.0,
        jit=False,
        require_x64=True,
    )
    s = np.array([0.0, 0.5, 1.0], dtype=np.float64)

    out = path.evaluate_up_to_3rd(s)

    assert path.dim == 3
    assert out.q.shape == (3, 3)
    assert out.dq.shape == (3, 3)
    assert out.ddq.shape == (3, 3)
    assert out.dddq.shape == (3, 3)
    assert np.allclose(out.q[:, 0], np.sin(s))
    assert np.allclose(out.dq[:, 2], 1.0 + 0.2 * s)
    assert np.allclose(out.ddq[:, 2], 0.2)
    assert np.allclose(out.dddq[:, 2], 0.0)


def test_path_from_autograd_smoke():
    pytest.importorskip("autograd")
    anp = pytest.importorskip("autograd.numpy")

    def q_fn(s):
        return anp.array([anp.sin(s), anp.cos(s), s + 0.1 * s**2])

    path = copp.Path.from_autograd(q_fn, 0.0, 1.0)
    s = np.array([0.0, 0.5, 1.0], dtype=np.float64)

    out = path.evaluate_up_to_3rd(s)

    assert path.dim == 3
    assert np.allclose(out.q[:, 1], np.cos(s))
    assert np.allclose(out.dq[:, 2], 1.0 + 0.2 * s)
    assert np.allclose(out.ddq[:, 2], 0.2)
    assert np.allclose(out.dddq[:, 2], 0.0)


def test_path_from_casadi_smoke():
    ca = pytest.importorskip("casadi")

    s_sym = ca.SX.sym("s")
    q_expr = ca.vertcat(ca.sin(s_sym), ca.cos(s_sym), s_sym + 0.1 * s_sym**2)

    path = copp.Path.from_casadi(q_expr, symbol=s_sym, s_min=0.0, s_max=1.0)
    s = np.array([0.0, 0.5, 1.0], dtype=np.float64)

    out = path.evaluate_up_to_3rd(s)

    assert path.dim == 3
    assert np.allclose(out.q[:, 0], np.sin(s))
    assert np.allclose(out.dq[:, 2], 1.0 + 0.2 * s)
    assert np.allclose(out.ddq[:, 2], 0.2)
    assert np.allclose(out.dddq[:, 2], 0.0)


def test_path_from_sympy_smoke():
    sp = pytest.importorskip("sympy")

    s_sym = sp.symbols("s")
    q_exprs = [
        sp.sin(s_sym),
        sp.cos(s_sym),
        s_sym + sp.Rational(1, 10) * s_sym**2,
    ]

    path = copp.Path.from_sympy(q_exprs, symbol=s_sym, s_min=0.0, s_max=1.0)
    s = np.array([0.0, 0.5, 1.0], dtype=np.float64)

    out = path.evaluate_up_to_3rd(s)

    assert path.dim == 3
    assert np.allclose(out.q[:, 1], np.cos(s))
    assert np.allclose(out.dq[:, 2], 1.0 + 0.2 * s)
    assert np.allclose(out.ddq[:, 2], 0.2)
    assert np.allclose(out.dddq[:, 2], 0.0)


def test_path_from_jax_reports_missing_backend_dependency(monkeypatch):
    import copp_py._parametric as parametric

    def fake_import_optional(module_name, message):
        if module_name == "jax":
            raise ImportError(message)
        return __import__(module_name, fromlist=["*"])

    monkeypatch.setattr(parametric, "_import_optional", fake_import_optional)

    with pytest.raises(ImportError, match=r"copp-py\[jax\]"):
        copp.Path.from_jax(lambda s: [s], 0.0, 1.0)
