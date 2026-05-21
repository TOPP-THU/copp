"""Basic tests for copp Python bindings."""

import math
import numpy as np
import pytest
import copp


class TestJet3:
    def test_constant(self):
        j = copp.Jet3.constant(3.0)
        assert j.v == 3.0
        assert j.d1 == 0.0

    def test_seed(self):
        j = copp.Jet3.seed(2.0)
        assert j.v == 2.0
        assert j.d1 == 1.0

    def test_arithmetic(self):
        a = copp.Jet3.seed(1.0)
        b = copp.Jet3.constant(2.0)
        c = a + b
        assert c.v == 3.0
        d = a * b
        assert d.v == 2.0

    def test_trig(self):
        s = copp.Jet3.seed(0.0)
        r = copp.sin(s)
        assert abs(r.v - 0.0) < 1e-12
        assert abs(r.d1 - 1.0) < 1e-12  # cos(0) = 1


class TestPath:
    def test_from_waypoints(self):
        # Convention: (dim, n_waypoints)
        waypoints = np.array([
            [0.0, 1.0, 2.0, 3.0, 4.0, 5.0],
            [0.0, 1.0, 0.0, 1.0, 0.0, 1.0],
        ])
        path = copp.Path.from_waypoints(waypoints)
        assert path.dim == 2
        s_min, s_max = path.s_range
        assert s_min == 0.0
        assert s_max > 0.0

    def test_from_parametric(self):
        def q_fn(s):
            return [copp.sin(2.0 * math.pi * s), copp.cos(2.0 * math.pi * s)]

        path = copp.Path.from_parametric(q_fn, 0.0, 1.0)
        assert path.dim == 2

    def test_from_parametric_preserves_callback_exception(self):
        def q_fn(_s):
            raise ValueError("path callback failed")

        with pytest.raises(ValueError, match="path callback failed"):
            copp.Path.from_parametric(q_fn, 0.0, 1.0)

    def test_from_parametric_preserves_callback_return_type_error(self):
        def q_fn(_s):
            return [1.0]

        with pytest.raises(TypeError, match="Jet3"):
            copp.Path.from_parametric(q_fn, 0.0, 1.0)

    def test_evaluate_preserves_parametric_callback_exception(self):
        def q_fn(s):
            if abs(s.v - 0.5) < 1e-15:
                return [copp.sin(s)]
            raise ValueError("path evaluate failed")

        path = copp.Path.from_parametric(q_fn, 0.0, 1.0)
        with pytest.raises(ValueError, match="path evaluate failed"):
            path.evaluate_q(np.array([0.0]))

    def test_evaluate(self):
        # Convention: (dim, n_waypoints)
        waypoints = np.array([
            [0.0, 1.0, 2.0, 3.0, 4.0, 5.0],
            [0.0, 1.0, 0.0, 1.0, 0.0, 1.0],
        ])
        path = copp.Path.from_waypoints(waypoints)
        s = np.linspace(*path.s_range, 50)
        derivs = path.evaluate_up_to_2nd(s)
        assert derivs.q.shape == (2, 50)
        assert derivs.dq is not None
        assert derivs.ddq is not None


class TestTopp2Pipeline:
    """End-to-end TOPP2-RA pipeline test."""

    def test_topp2_ra_basic(self):
        # Use waypoints (pure Rust, no Python callback overhead)
        n_wp = 20
        t = np.linspace(0, 2 * math.pi, n_wp)
        waypoints = np.array([np.sin(t), np.cos(t)])  # (2, n_wp)
        path = copp.Path.from_waypoints(waypoints)

        n = 101
        s = np.linspace(*path.s_range, n)
        derivs = path.evaluate_up_to_2nd(s)

        # Build robot
        dim = 2
        robot = copp.Robot(dim=dim, capacity=n)
        robot.with_s(s)
        robot.with_q(derivs.q, derivs.dq, derivs.ddq)

        vel_max = np.full(dim, 5.0)
        vel_min = np.full(dim, -5.0)
        robot.with_axial_velocity(vel_max, vel_min)

        acc_max = np.full(dim, 10.0)
        acc_min = np.full(dim, -10.0)
        robot.with_axial_acceleration(acc_max, acc_min)

        # Solve
        idx_s_interval = (0, n - 1)
        a_boundary = (0.0, 0.0)
        a = copp.topp2_ra(robot, idx_s_interval, a_boundary)

        assert a.shape == (n,)
        assert np.all(a >= -1e-10)  # a = sdot^2 >= 0

        # Interpolation
        t_final, t_s = copp.s_to_t_topp2(s, a)
        assert t_final > 0.0
        assert t_s.shape == (n,)

        s_t = copp.t_to_s_topp2(s, a, t_s, dt=1e-2)
        assert len(s_t) > 0


class TestErrors:
    def test_exception_hierarchy(self):
        assert issubclass(copp.PathError, copp.CoppError)
        assert issubclass(copp.ConstraintError, copp.CoppError)
        assert issubclass(copp.InfeasibleError, copp.CoppError)
        assert issubclass(copp.InvalidInputError, copp.CoppError)

    def test_options_reject_invalid_verbosity(self):
        with pytest.raises(ValueError, match="invalid verbosity"):
            copp.ReachSet2Options(verbosity="not-a-level")

        with pytest.raises(ValueError, match="invalid verbosity"):
            copp.ClarabelOptions(verbosity="not-a-level")

    def test_inverse_dynamics_preserves_callback_exception(self):
        def inverse_dynamics(_q, _dq, _ddq):
            raise ValueError("inverse dynamics failed")

        robot = _robot_with_inverse_dynamics(inverse_dynamics)
        with pytest.raises(ValueError, match="inverse dynamics failed"):
            copp.copp2_socp(
                robot,
                (0, 2),
                (0.0, 0.0),
                [copp.Objective.thermal_energy(1.0, [1.0, 1.0])],
            )

    def test_inverse_dynamics_validates_return_length(self):
        def inverse_dynamics(_q, _dq, _ddq):
            return [0.0]

        robot = _robot_with_inverse_dynamics(inverse_dynamics)
        with pytest.raises(ValueError, match="must return 2 values, got 1"):
            copp.copp2_socp(
                robot,
                (0, 2),
                (0.0, 0.0),
                [copp.Objective.thermal_energy(1.0, [1.0, 1.0])],
            )


def _robot_with_inverse_dynamics(inverse_dynamics):
    robot = copp.Robot(dim=2, inverse_dynamics=inverse_dynamics, capacity=3)
    s = np.array([0.0, 0.5, 1.0])
    q = np.zeros((2, 3))
    robot.with_s(s)
    robot.with_q(q, q, q)
    return robot
