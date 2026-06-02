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
        e = 2.0 / copp.Jet3.seed(2.0)
        assert e.v == 1.0
        assert e.d1 == -0.5

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

    def test_spline_config_exposes_boundary_state(self):
        cfg = copp.SplineConfig(
            order=3,
            start_state=np.array([[2.0]]),
            end_state=np.array([[2.0]]),
        )
        np.testing.assert_allclose(cfg.start_state, [[2.0]])
        np.testing.assert_allclose(cfg.end_state, [[2.0]])

        path = copp.Path.from_waypoints(np.array([[0.0, 1.0]]), cfg)
        derivs = path.evaluate_up_to_2nd(np.array([0.0, 1.0]))
        np.testing.assert_allclose(derivs.dq, [[2.0, 2.0]], atol=1e-12)

    def test_from_evaluator_2nd(self):
        def evaluate_2nd(s):
            q = np.vstack((s, s**2))
            dq = np.vstack((np.ones_like(s), 2.0 * s))
            ddq = np.vstack((np.zeros_like(s), 2.0 * np.ones_like(s)))
            return q, dq, ddq

        path = copp.Path.from_evaluator_2nd(2, evaluate_2nd, 0.0, 1.0)
        derivs = path.evaluate_up_to_2nd(np.array([0.0, 0.5, 1.0]))

        np.testing.assert_allclose(derivs.q, [[0.0, 0.5, 1.0], [0.0, 0.25, 1.0]])
        np.testing.assert_allclose(derivs.dq, [[1.0, 1.0, 1.0], [0.0, 1.0, 2.0]])
        np.testing.assert_allclose(derivs.ddq, [[0.0, 0.0, 0.0], [2.0, 2.0, 2.0]])

    def test_from_evaluator_3rd(self):
        def evaluate_3rd(s):
            q = np.vstack((s**3,))
            dq = np.vstack((3.0 * s**2,))
            ddq = np.vstack((6.0 * s,))
            dddq = np.vstack((6.0 * np.ones_like(s),))
            return q, dq, ddq, dddq

        path = copp.Path.from_evaluator_3rd(1, evaluate_3rd, 0.0, 1.0)
        derivs = path.evaluate_up_to_3rd(np.array([0.0, 0.5, 1.0]))

        np.testing.assert_allclose(derivs.q, [[0.0, 0.125, 1.0]])
        np.testing.assert_allclose(derivs.ddq, [[0.0, 3.0, 6.0]])
        np.testing.assert_allclose(derivs.dddq, [[6.0, 6.0, 6.0]])

    def test_from_evaluator_reports_shape_error(self):
        def evaluate_2nd(s):
            bad = np.zeros((1, len(s)))
            return bad, bad, bad

        path = copp.Path.from_evaluator_2nd(2, evaluate_2nd, 0.0, 1.0)
        with pytest.raises(ValueError, match="q must have shape"):
            path.evaluate_up_to_2nd(np.array([0.0, 1.0]))


class TestRobotConstraints:
    def test_station_dependent_high_level_limits_and_low_level_accessors(self):
        robot = _robot_with_unit_path(dim=2, n=3)
        limit = np.array([[2.0, 3.0], [4.0, 4.0]])

        robot.with_axial_velocity(limit, -limit, idx_s=1)

        assert robot.len == 3
        assert not robot.is_empty
        assert robot.idx_s_start == 0
        assert robot.idx_s_end == 3
        assert robot.amax_rows == 1
        assert robot.acc_rows >= 0
        assert robot.jerk_rows >= 0
        assert robot.get_s(2) == 2.0
        np.testing.assert_allclose(robot.amax_vec(1, 3), [4.0, 9.0])

    def test_low_level_constraint_methods(self):
        robot = _robot_with_unit_path(dim=1, n=3)

        robot.with_constraint_1order(np.array([5.0, 6.0, 7.0]))
        np.testing.assert_allclose(robot.amax_vec(0, 3), [5.0, 6.0, 7.0])

        acc_a = np.array([[1.0, 2.0, 3.0]])
        acc_b = np.array([[4.0, 5.0, 6.0]])
        acc_max = np.array([[7.0, 8.0, 9.0]])
        robot.with_constraint_2order(acc_a, acc_b, acc_max)
        assert robot.acc_rows >= 1
        got_a, got_b, got_max = robot.get_acc_constraints(1)
        np.testing.assert_allclose(got_a, [[2.0]])
        np.testing.assert_allclose(got_b, [[5.0]])
        np.testing.assert_allclose(got_max, [[8.0]])

        jerk_a = np.array([[1.0, 2.0, 3.0]])
        jerk_b = np.array([[4.0, 5.0, 6.0]])
        jerk_c = np.array([[7.0, 8.0, 9.0]])
        jerk_d = np.array([[10.0, 11.0, 12.0]])
        jerk_max = np.array([[13.0, 14.0, 15.0]])
        robot.with_constraint_3order(jerk_a, jerk_b, jerk_c, jerk_d, jerk_max)
        assert robot.jerk_rows >= 1
        got = robot.get_jerk_constraints(2)
        np.testing.assert_allclose(got[0], [[3.0]])
        np.testing.assert_allclose(got[4], [[15.0]])

        robot.pop_front_n(1)
        assert robot.idx_s_start == 1
        robot.pop_back_to(2)
        assert robot.idx_s_end == 2
        robot.clear()
        assert robot.is_empty


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
        idx_s_start, idx_s_final = 0, n - 1
        a = copp.topp2_ra(robot, idx_s_start, idx_s_final, 0.0, 0.0)

        assert a.shape == (n,)
        assert np.all(a >= -1e-10)  # a = sdot^2 >= 0

        # Interpolation
        t_final, t_s = copp.s_to_t_topp2(s, a)
        assert t_final > 0.0
        assert t_s.shape == (n,)

        s_t = copp.t_to_s_topp2(s, a, t_s, dt=1e-2)
        assert len(s_t) > 0

    def test_copp2_expert_returns_solution_and_linsolver_info(self):
        robot = _robot_with_unit_path(dim=1, n=4)
        robot.with_axial_velocity(np.array([2.0]), np.array([-2.0]))
        robot.with_axial_acceleration(np.array([5.0]), np.array([-5.0]))

        a, solution, info = copp.copp2_socp_expert(
            robot,
            0,
            3,
            0.0,
            0.0,
            [copp.Objective.time()],
        )

        assert a is not None
        assert a.shape == (4,)
        assert solution.status in {
            copp.ClarabelSolverStatus.Solved,
            copp.ClarabelSolverStatus.AlmostSolved,
        }
        assert info.name
        assert info.nnz_a > 0

    def test_t_to_s_topp2_accepts_uniform_grid_controls(self):
        s = np.array([0.0, 1.0, 2.0])
        a = np.array([1.0, 1.0, 1.0])
        _, t_s = copp.s_to_t_topp2(s, a)

        with_final = copp.t_to_s_topp2(s, a, t_s, dt=0.5, t0=0.0, include_final=True)
        without_final = copp.t_to_s_topp2(s, a, t_s, dt=0.5, t0=0.0, include_final=False)

        assert with_final[-1] == s[-1]
        assert without_final[-1] < s[-1]


class TestOptions:
    def test_options_expose_rust_defaults_and_settings(self):
        rs = copp.ReachSet2Options(verbosity=copp.Verbosity.Summary)
        assert rs.lp_feas_tol > 0.0
        assert rs.a_cmp_abs_tol > 0.0
        assert rs.a_cmp_rel_tol > 0.0
        assert rs.verbosity == copp.Verbosity.Summary

        settings = copp.ClarabelSettings()
        settings.max_iter = 7
        opts = copp.ClarabelOptions(settings=settings)
        assert opts.allow_almost_solved
        assert opts.settings.max_iter == 7

    def test_verbosity_output_roundtrip(self):
        previous = copp.verbosity_output()
        try:
            copp.set_verbosity_output("println")
            assert copp.verbosity_output() == ("println", None)
        finally:
            copp.set_verbosity_output(previous[0], previous[1])


class TestErrors:
    def test_exception_hierarchy(self):
        assert issubclass(copp.PathError, copp.CoppError)
        assert issubclass(copp.ConstraintError, copp.CoppError)
        assert issubclass(copp.InfeasibleError, copp.CoppError)
        assert issubclass(copp.InvalidInputError, copp.CoppError)

    def test_options_reject_invalid_verbosity(self):
        # verbosity is now a typed enum; passing a string raises TypeError.
        with pytest.raises(TypeError):
            copp.ReachSet2Options(verbosity="not-a-level")

        with pytest.raises(TypeError):
            copp.ClarabelOptions(verbosity="not-a-level")

    def test_inverse_dynamics_preserves_callback_exception(self):
        def inverse_dynamics(_q, _dq, _ddq):
            raise ValueError("inverse dynamics failed")

        robot = _robot_with_inverse_dynamics(inverse_dynamics)
        with pytest.raises(ValueError, match="inverse dynamics failed"):
            copp.copp2_socp(
                robot,
                0,
                2,
                0.0,
                0.0,
                [copp.Objective.thermal_energy(1.0, [1.0, 1.0])],
            )

    def test_inverse_dynamics_validates_return_length(self):
        def inverse_dynamics(_q, _dq, _ddq):
            return [0.0]

        robot = _robot_with_inverse_dynamics(inverse_dynamics)
        with pytest.raises(ValueError, match="must return 2 values, got 1"):
            copp.copp2_socp(
                robot,
                0,
                2,
                0.0,
                0.0,
                [copp.Objective.thermal_energy(1.0, [1.0, 1.0])],
            )

    def test_axial_torque_preserves_callback_return_length_error(self):
        def inverse_dynamics(_q, _dq, _ddq):
            return [0.0]

        robot = _robot_with_inverse_dynamics(inverse_dynamics)
        with pytest.raises(ValueError, match="must return 2 values, got 1"):
            robot.with_axial_torque(np.ones(2), -np.ones(2))


def test_force_positive_a_3rd_is_exported_and_mutates_profile():
    s = np.array([0.0, 1.0, 2.0, 3.0])
    a = np.array([1.0, 1.0, 1.0, 1.0])
    b = np.array([-5.0, -5.0, 5.0, 5.0])

    succeed = copp.force_positive_a_3rd(s, a, b, (0, 0), 0.25)

    assert succeed
    assert np.all(a > 0.0)
    assert not np.allclose(b, [-5.0, -5.0, 5.0, 5.0])


def test_copp2_expert_exposes_objective_breakdown():
    robot = _robot_with_unit_path(dim=1, n=4)
    robot.with_axial_velocity(np.array([2.0]), np.array([-2.0]))
    robot.with_axial_acceleration(np.array([5.0]), np.array([-5.0]))

    _, solution, _ = copp.copp2_socp_expert(
        robot, 0, 3, 0.0, 0.0, [copp.Objective.time()]
    )

    # Weighted objective value and per-term breakdown are populated for COPP2.
    assert np.isfinite(solution.objective_value)
    assert solution.objective_terms.shape == (1,)
    assert np.all(np.isfinite(solution.objective_terms))


def test_reach_set2_returns_a_max_then_a_min():
    robot = _robot_with_unit_path(dim=1, n=5)
    robot.with_axial_velocity(np.array([2.0]), np.array([-2.0]))
    robot.with_axial_acceleration(np.array([5.0]), np.array([-5.0]))

    a_max, a_min = copp.reach_set2_backward(robot, 0, 4, 0.0, 0.0)

    # C ABI orders the result a_max first, then a_min: a_max >= a_min elementwise.
    assert a_max.shape == a_min.shape == (5,)
    assert np.all(a_max + 1e-9 >= a_min)


def test_set_and_clear_inverse_dynamics():
    captured = {"called": False}

    def inverse_dynamics(_q, _dq, _ddq):
        captured["called"] = True
        return [0.0, 0.0]

    robot = copp.Robot(dim=2, capacity=3)
    s = np.array([0.0, 0.5, 1.0])
    q = np.zeros((2, 3))
    robot.with_s(s)
    robot.with_q(q, q, q)

    # Setting after construction installs the callback.
    robot.set_inverse_dynamics(inverse_dynamics)
    robot.with_axial_torque(np.ones(2), -np.ones(2))
    assert captured["called"]

    # Clearing restores the default (tau = ddq); a bad callback is not invoked.
    def bad_inverse_dynamics(_q, _dq, _ddq):
        raise ValueError("should not be called after clear")

    robot.set_inverse_dynamics(bad_inverse_dynamics)
    robot.clear_inverse_dynamics()
    robot.with_axial_torque(np.ones(2), -np.ones(2))


def _robot_with_inverse_dynamics(inverse_dynamics):
    robot = copp.Robot(dim=2, inverse_dynamics=inverse_dynamics, capacity=3)
    s = np.array([0.0, 0.5, 1.0])
    q = np.zeros((2, 3))
    robot.with_s(s)
    robot.with_q(q, q, q)
    return robot


def _robot_with_unit_path(dim, n):
    robot = copp.Robot(dim=dim, capacity=n)
    s = np.arange(n, dtype=float)
    q = np.zeros((dim, n))
    dq = np.ones((dim, n))
    ddq = np.zeros((dim, n))
    dddq = np.zeros((dim, n))
    robot.with_s(s)
    robot.with_q(q, dq, ddq, dddq)
    return robot
