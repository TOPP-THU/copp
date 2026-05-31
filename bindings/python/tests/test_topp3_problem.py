import numpy as np
import pytest

import copp_py as copp


def _build_robot_with_jerk_constraints(n_samples=5, dim=2):
    s = np.linspace(0.0, 1.0, n_samples, dtype=np.float64)
    q = np.empty((n_samples, dim), dtype=np.float64)
    dq = np.empty_like(q)
    ddq = np.empty_like(q)
    dddq = np.empty_like(q)

    for col in range(dim):
        scale = float(col + 1)
        q[:, col] = scale * s
        dq[:, col] = scale
        ddq[:, col] = 0.25 * scale
        dddq[:, col] = 0.1 * scale

    robot = copp.Robot(dim, capacity=n_samples)
    robot.append_s(s)
    robot.set_q(q, dq, ddq, 0, dddq=dddq)
    upper = np.full(dim, 10.0, dtype=np.float64)
    robot.add_jerk_limits(upper, -upper, start_idx_s=0)
    return robot


def test_topp3_problem_constructs_and_copies_linearization():
    robot = _build_robot_with_jerk_constraints()
    a_linearization = np.ones(len(robot), dtype=np.float64)

    problem = copp.solver.topp3_lp.Problem(
        robot.constraints,
        a_linearization,
        idx_s_start=0,
        a_boundary=(0.0, 0.0),
        b_boundary=(0.0, 0.0),
        num_stationary_max=1,
    )

    assert problem.constraints.len == len(robot)
    assert problem.idx_s_start == 0
    assert problem.idx_s_final == len(robot) - 1
    assert problem.s_len == len(robot)
    assert problem.a_boundary == (0.0, 0.0)
    assert problem.b_boundary == (0.0, 0.0)
    assert problem.num_stationary_max == (1, 1)
    assert np.isclose(problem.a_linearization_floor, 1.0e-10)
    assert np.allclose(problem.a_linearization, 1.0)

    a_linearization[0] = 7.0
    copied = problem.a_linearization
    copied[1] = 9.0
    assert np.allclose(problem.a_linearization, 1.0)

    problem.validate()


def test_topp3_problem_accepts_stationary_pair():
    robot = _build_robot_with_jerk_constraints()
    a_linearization = np.ones(len(robot), dtype=np.float64)

    problem = copp.solver.topp3_lp.Problem(
        robot.constraints,
        a_linearization,
        num_stationary_max=(2, 3),
    )

    assert problem.num_stationary_max == (2, 3)


def test_topp3_problem_rejects_invalid_python_arguments():
    robot = _build_robot_with_jerk_constraints()
    a_linearization = np.ones(len(robot), dtype=np.float64)

    with pytest.raises(ValueError, match="num_stationary_max"):
        copp.solver.topp3_lp.Problem(
            robot.constraints,
            a_linearization,
            num_stationary_max="bad",
        )

    with pytest.raises(ValueError, match="a_linearization"):
        copp.solver.topp3_lp.Problem(
            robot.constraints,
            [a_linearization.tolist()],
        )


def test_topp3_problem_rejects_invalid_core_arguments():
    robot = _build_robot_with_jerk_constraints()
    a_linearization = np.ones(len(robot), dtype=np.float64)

    with pytest.raises(copp.CoppError, match="a_linearization_floor"):
        copp.solver.topp3_lp.Problem(
            robot.constraints,
            a_linearization,
            a_linearization_floor=0.0,
        )
