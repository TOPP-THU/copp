import numpy as np

import copp_py as copp


def _build_topp3_problem(n_samples=7, dim=2):
    s = np.linspace(0.0, 1.0, n_samples, dtype=np.float64)
    q = np.empty((n_samples, dim), dtype=np.float64)
    dq = np.empty_like(q)
    ddq = np.empty_like(q)
    dddq = np.empty_like(q)

    for col in range(dim):
        scale = float(col + 1)
        q[:, col] = scale * s
        dq[:, col] = scale
        ddq[:, col] = 0.0
        dddq[:, col] = 0.0

    robot = copp.Robot(dim, capacity=n_samples)
    robot.append_s(s)
    robot.set_q(q, dq, ddq, 0, dddq=dddq)

    upper = np.full(dim, 100.0, dtype=np.float64)
    lower = -upper
    robot.add_velocity_limits(upper, lower, start_idx_s=0)
    robot.add_acceleration_limits(upper, lower, start_idx_s=0)
    robot.add_jerk_limits(upper, lower, start_idx_s=0)

    problem = copp.solver.topp3_socp.Problem(
        robot.constraints,
        np.ones(n_samples, dtype=np.float64),
        idx_s_start=0,
        a_boundary=(0.0, 0.0),
        b_boundary=(0.0, 0.0),
        num_stationary_max=1,
    )
    return s, problem


def _assert_profile(profile, n_samples):
    assert isinstance(profile, copp.Profile3rd)
    assert profile.a.shape == (n_samples,)
    assert profile.b.shape == (n_samples,)
    assert profile.num_stationary == (1, 1)
    assert np.all(np.isfinite(profile.a))
    assert np.all(np.isfinite(profile.b))


def _assert_expert_result(result, n_samples):
    assert isinstance(result, copp.solver.topp3_socp.Result)
    assert result.profile is not None
    _assert_profile(result.profile, n_samples)
    assert result.x.ndim == 1
    assert result.z.ndim == 1
    assert result.s.ndim == 1
    assert result.x.size > 0
    assert result.objective_terms is None
    assert result.objective_value is None


def test_topp3_lp_and_expert_return_shared_result_type():
    s, problem = _build_topp3_problem()

    profile = copp.solver.topp3_lp.solve(problem)
    _assert_profile(profile, s.size)

    result = copp.solver.topp3_lp.solve_expert(problem)
    _assert_expert_result(result, s.size)


def test_topp3_socp_and_expert_return_shared_result_type():
    s, problem = _build_topp3_problem()

    profile = copp.solver.topp3_socp.solve(problem)
    _assert_profile(profile, s.size)

    result = copp.solver.topp3_socp.solve_expert(problem)
    _assert_expert_result(result, s.size)
