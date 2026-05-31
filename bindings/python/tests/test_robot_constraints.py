import numpy as np
import pytest

import copp_py as copp


def _station_data(n_samples=4, dim=2):
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

    return s, q, dq, ddq, dddq


def test_robot_manual_data_and_limits():
    s, q, dq, ddq, dddq = _station_data()

    robot = copp.Robot(2, capacity=8)
    robot.append_s(s.tolist())
    robot.set_q(q.tolist(), dq.tolist(), ddq.tolist(), 0, dddq=dddq.tolist())

    upper = [2.0, 3.0]
    lower = [-2.0, -3.0]
    robot.add_velocity_limits(upper, lower, start_idx_s=0)
    robot.add_acceleration_limits(upper, lower, start_idx_s=0)
    robot.add_jerk_limits(upper, lower, start_idx_s=0)
    robot.add_torque_limits(upper, lower, start_idx_s=0)

    assert len(robot) == s.size
    assert robot.dim == 2
    assert robot.idx_s_range == (0, s.size)
    assert not robot.has_inverse_dynamics
    assert np.allclose(robot.constraints.s_values(), s)
    assert np.all(np.isfinite(robot.constraints.amax_values()))
    assert issubclass(copp.ConstraintError, copp.CoppError)


def test_robot_constraints_property_returns_shared_proxy():
    robot = copp.Robot(2)
    robot.append_s(np.array([0.0, 0.5, 1.0], dtype=np.float64))

    first_proxy = robot.constraints
    second_proxy = robot.constraints
    assert first_proxy is not second_proxy

    amax = np.full((3, 1), 4.0, dtype=np.float64)
    first_proxy.add_constraint_1st(amax, 0)

    assert np.allclose(second_proxy.amax_values(), 4.0)


def test_robot_raw_second_order_constraint():
    robot = copp.Robot(2)
    robot.append_s(np.array([0.0, 0.5, 1.0], dtype=np.float64))

    zeros = np.zeros((3, 2), dtype=np.float64)
    ones = np.ones((3, 2), dtype=np.float64)
    robot.constraints.add_constraint_2nd(zeros, ones, 10.0 * ones, 0)

    assert robot.constraints.len == 3


def test_python_inverse_dynamics_callable_and_object_protocol():
    s, q, dq, ddq, _ = _station_data(n_samples=3, dim=2)

    def inverse_dynamics(q_curr, dq_curr, ddq_curr):
        return ddq_curr + 0.1 * dq_curr + 0.01 * q_curr

    robot = copp.Robot(2, inverse_dynamics=inverse_dynamics)
    robot.append_s(s)
    robot.set_q(q, dq, ddq, 0)
    robot.add_torque_limits(
        np.array([2.0, 2.5], dtype=np.float64),
        np.array([-2.0, -2.5], dtype=np.float64),
        start_idx_s=0,
    )

    assert robot.has_inverse_dynamics

    class DynamicsObject:
        def inverse_dynamics(self, q_curr, dq_curr, ddq_curr):
            return np.ascontiguousarray(ddq_curr, dtype=np.float64)

    robot.set_inverse_dynamics(DynamicsObject())
    robot.add_torque_limits(
        np.array([3.0, 3.5], dtype=np.float64),
        np.array([-3.0, -3.5], dtype=np.float64),
        start_idx_s=0,
    )


def test_python_inverse_dynamics_exception_is_preserved():
    s, q, dq, ddq, _ = _station_data(n_samples=2, dim=1)

    def broken(_q, _dq, _ddq):
        raise RuntimeError("dynamics failed")

    robot = copp.Robot(1, inverse_dynamics=broken)
    robot.append_s(s)
    robot.set_q(q, dq, ddq, 0)

    with pytest.raises(RuntimeError, match="dynamics failed"):
        robot.add_torque_limits(
            np.array([2.0], dtype=np.float64),
            np.array([-2.0], dtype=np.float64),
            start_idx_s=0,
        )


def test_constraints_can_be_constructed_independently():
    constraints = copp.Constraints(2, capacity=4)
    constraints.append_s([0.0, 0.5, 1.0])

    amax = np.full((3, 1), 4.0, dtype=np.float64)
    constraints.add_constraint_1st(amax, 0)

    assert constraints.dim == 2
    assert constraints.len == 3
    assert constraints.idx_s_range == (0, 3)
    assert np.allclose(constraints.s_values(), [0.0, 0.5, 1.0])
    assert np.allclose(constraints.amax_values(), 4.0)


def test_constraint_core_errors_use_constraint_error():
    constraints = copp.Constraints(1)

    with pytest.raises(copp.ConstraintError):
        constraints.append_s([0.0, 0.0])


def test_independent_constraints_work_with_topp2_problem():
    constraints = copp.Constraints(1)
    constraints.append_s(np.array([0.0, 0.5, 1.0], dtype=np.float64))

    amax = np.full((3, 1), 2.0, dtype=np.float64)
    acc_a = np.zeros((3, 1), dtype=np.float64)
    acc_b = np.ones((3, 1), dtype=np.float64)
    acc_max = np.full((3, 1), 10.0, dtype=np.float64)
    constraints.add_constraint_1st(amax, 0)
    constraints.add_constraint_2nd(acc_a, acc_b, acc_max, 0)

    problem = copp.solver.topp2_ra.Problem(
        constraints,
        idx_s_interval=(0, 2),
        a_boundary=(0.0, 0.0),
    )
    a = copp.solver.topp2_ra.solve(problem)

    assert a.shape == (3,)
    assert np.all(np.isfinite(a))
