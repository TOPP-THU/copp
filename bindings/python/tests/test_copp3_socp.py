import numpy as np

import copp_py as copp


def _build_copp3_problem(n_samples=7, dim=2):
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

    objectives = [
        copp.objective.Time(1.0),
        copp.objective.ThermalEnergy(0.1, np.ones(dim, dtype=np.float64)),
    ]
    problem = copp.solver.copp3_socp.Problem(
        robot,
        objectives,
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


def test_copp3_problem_constructs_with_objectives():
    s, problem = _build_copp3_problem()

    assert isinstance(problem.robot, copp.Robot)
    assert problem.objective_count == 2
    assert problem.s_len == s.size
    assert problem.idx_s_start == 0
    assert problem.idx_s_final == s.size - 1
    assert problem.num_stationary_max == (1, 1)
    np.testing.assert_array_equal(problem.a_linearization, np.ones(s.size))


def test_copp3_socp_and_expert_report_objective_terms():
    s, problem = _build_copp3_problem()

    profile = copp.solver.copp3_socp.solve(problem)
    _assert_profile(profile, s.size)

    result = copp.solver.copp3_socp.solve_expert(problem)
    assert isinstance(result, copp.solver.copp3_socp.Result)
    assert result.profile is not None
    _assert_profile(result.profile, s.size)
    assert result.objective_terms is not None
    assert result.objective_terms.shape == (2,)
    assert result.objective_value is not None
    assert np.isfinite(result.objective_value)

    terms = result.objective_terms
    assert terms is not None
    terms[0] = -1.0
    assert result.objective_terms[0] != -1.0


def test_copp3_linear_objective_uses_node_length_beta():
    s, problem = _build_copp3_problem()
    robot = problem.robot
    alpha = np.zeros(s.size, dtype=np.float64)
    beta = np.zeros(s.size, dtype=np.float64)
    objectives = [
        copp.objective.Time(1.0),
        copp.objective.Linear(0.01, alpha, beta),
    ]

    linear_problem = copp.solver.copp3_socp.Problem(
        robot,
        objectives,
        np.ones(s.size, dtype=np.float64),
        idx_s_start=0,
        a_boundary=(0.0, 0.0),
        b_boundary=(0.0, 0.0),
        num_stationary_max=1,
    )
    result = copp.solver.copp3_socp.solve_expert(linear_problem)

    assert result.profile is not None
    assert result.objective_terms is not None
    assert result.objective_terms.shape == (2,)
