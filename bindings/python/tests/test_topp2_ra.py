import numpy as np
import pytest

import copp_py as copp


EXAMPLE_DIM = 3
EXAMPLE_NUM_POINTS = 1001
EXAMPLE_DT = 1.0e-3


class LissajousPath2nd:
    """Same deterministic 3-axis path used by Rust and C TOPP2-RA examples."""

    dim = EXAMPLE_DIM

    def evaluate_q(self, s):
        q, _, _ = self.evaluate_up_to_2nd(s)
        return q

    def evaluate_up_to_2nd(self, s):
        freq = np.array([2.0 * np.pi, 3.0 * np.pi, 5.0 * np.pi], dtype=np.float64)
        phase = np.array([0.0, 0.3, 0.7], dtype=np.float64)
        x = s[:, None] * freq[None, :] + phase[None, :]

        q = np.sin(x)
        dq = np.cos(x) * freq[None, :]
        ddq = -np.sin(x) * (freq[None, :] ** 2)

        return (
            np.ascontiguousarray(q, dtype=np.float64),
            np.ascontiguousarray(dq, dtype=np.float64),
            np.ascontiguousarray(ddq, dtype=np.float64),
        )


def _build_rust_c_aligned_topp2_example():
    # 1) Deterministic 3-axis Lissajous path q(s), s in [0, 1].
    path = copp.Path.from_evaluator_2nd(LissajousPath2nd(), 0.0, 1.0)
    s = np.linspace(0.0, 1.0, EXAMPLE_NUM_POINTS, dtype=np.float64)

    # 2) Build robot constraints, then apply symmetric limits vel/acc = 1.
    robot = copp.Robot(EXAMPLE_DIM, capacity=EXAMPLE_NUM_POINTS)
    robot.append_s(s)
    robot.set_q_from_path_2nd(path, 0, EXAMPLE_NUM_POINTS)

    upper = np.ones(EXAMPLE_DIM, dtype=np.float64)
    lower = -upper
    robot.add_velocity_limits(
        upper,
        lower,
        start_idx_s=0,
        length=EXAMPLE_NUM_POINTS,
    )
    robot.add_acceleration_limits(
        upper,
        lower,
        start_idx_s=0,
        length=EXAMPLE_NUM_POINTS,
    )

    # 3) Build TOPP2-RA problem with a(0) = 0 and a(1) = 0.
    problem = copp.solver.topp2_ra.Problem(
        robot.constraints,
        idx_s_interval=(0, EXAMPLE_NUM_POINTS - 1),
        a_boundary=(0.0, 0.0),
    )

    return s, path, robot, problem


def test_topp2_ra_matches_rust_and_c_example_setup():
    s, path, robot, problem = _build_rust_c_aligned_topp2_example()
    options = copp.solver.topp2_ra.Options()

    a_ra = copp.solver.topp2_ra.solve(problem, options)

    # 4) Post-process TOPP2-RA results: a(s) -> t(s) -> s(t).
    t_final, t_s = copp.interpolation.s_to_t_topp2(s, a_ra, 0.0)
    s_t = copp.interpolation.t_to_s_topp2_uniform(
        s,
        a_ra,
        t_s,
        EXAMPLE_DT,
        t0=0.0,
        include_final=True,
    )

    assert path.dim == EXAMPLE_DIM
    assert robot.dim == EXAMPLE_DIM
    assert problem.s_len == EXAMPLE_NUM_POINTS
    assert a_ra.shape == (EXAMPLE_NUM_POINTS,)
    assert t_s.shape == (EXAMPLE_NUM_POINTS,)
    assert s_t.ndim == 1
    assert s_t.size > 0

    assert np.all(np.isfinite(a_ra))
    assert np.all(np.isfinite(t_s))
    assert np.all(np.isfinite(s_t))
    assert np.all(a_ra >= -1.0e-10)
    assert np.isclose(a_ra[0], 0.0)
    assert np.isclose(a_ra[-1], 0.0)
    assert t_final > 0.0
    assert np.isclose(t_s[0], 0.0)
    assert np.isclose(t_s[-1], t_final)
    assert np.isclose(s_t[0], 0.0)
    assert np.isclose(s_t[-1], 1.0)

    with pytest.raises(AttributeError):
        problem.idx_s_interval = (0, 1)
    with pytest.raises(AttributeError):
        problem.a_boundary = (0.0, 1.0)


def test_reach_set2_options_accept_string_verbosity():
    options = copp.solver.topp2_ra.Options(
        lp_feas_tol=1.0e-8,
        a_cmp_abs_tol=1.0e-8,
        a_cmp_rel_tol=1.0e-8,
        verbosity="silent",
    )

    assert options.verbosity == copp.Verbosity.SILENT


def test_topp2_ra_accepts_default_options():
    _s, _path, _robot, problem = _build_rust_c_aligned_topp2_example()

    a_ra = copp.solver.topp2_ra.solve(problem)

    assert a_ra.shape == (EXAMPLE_NUM_POINTS,)
