import numpy as np

import copp_py as copp


EXAMPLE_DIM = 3
EXAMPLE_NUM_POINTS = 1001
EXAMPLE_DT = 1.0e-3


class LissajousPath2nd:
    """Deterministic 3-axis path used by the COPP2-SOCP reference example."""

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


def _build_copp2_socp_reference_problem(objectives, num_points=EXAMPLE_NUM_POINTS):
    path = copp.Path.from_evaluator_2nd(LissajousPath2nd(), 0.0, 1.0)
    s = np.linspace(0.0, 1.0, num_points, dtype=np.float64)

    robot = copp.Robot(EXAMPLE_DIM, capacity=num_points)
    robot.append_s(s)
    robot.set_q_from_path_2nd(path, 0, num_points)

    upper = np.ones(EXAMPLE_DIM, dtype=np.float64)
    lower = -upper
    robot.add_velocity_limits(
        upper,
        lower,
        start_idx_s=0,
        length=num_points,
    )
    robot.add_acceleration_limits(
        upper,
        lower,
        start_idx_s=0,
        length=num_points,
    )

    problem = copp.solver.copp2_socp.Problem(
        robot,
        objectives,
        idx_s_interval=(0, num_points - 1),
        a_boundary=(0.0, 0.0),
    )

    return s, robot, problem


def test_clarabel_options_defaults_match_public_policy():
    options = copp.solver.copp2_socp.Options()

    assert options.verbosity == copp.Verbosity.SILENT
    assert options.allow_almost_solved is True
    assert options.allow_max_iterations is False
    assert options.allow_max_time is False
    assert options.allow_callback_terminated is False
    assert options.allow_insufficient_progress is False
    assert options.clarabel_settings.max_iter == 200
    assert options.clarabel_settings.equilibrate_max_iter == 20
    assert options.clarabel_settings.verbose is False


def test_clarabel_options_accept_advanced_settings():
    settings = copp.clarabel.Settings(
        max_iter=120,
        tol_gap_rel=1.0e-7,
        direct_solve_method="qdldl",
    )
    options = copp.solver.copp2_socp.Options(
        verbosity="debug",
        clarabel_settings=settings,
    )

    assert options.clarabel_settings.max_iter == 120
    assert options.clarabel_settings.tol_gap_rel == 1.0e-7
    assert (
        options.clarabel_settings.direct_solve_method
        == copp.clarabel.DirectSolveMethod.QDLDL
    )


def test_copp2_socp_matches_reference_example_setup():
    normalize = np.ones(EXAMPLE_DIM, dtype=np.float64)
    s, robot, problem = _build_copp2_socp_reference_problem(
        [
            copp.objective.Time(1.0),
            copp.objective.ThermalEnergy(0.1, normalize),
        ]
    )
    options = copp.solver.copp2_socp.Options(allow_almost_solved=True)

    a_socp = copp.solver.copp2_socp.solve(problem, options)

    t_final, t_s = copp.interpolation.s_to_t_topp2(s, a_socp, 0.0)
    s_t = copp.interpolation.t_to_s_topp2_uniform(
        s,
        a_socp,
        t_s,
        EXAMPLE_DT,
        t0=0.0,
        include_final=True,
    )

    assert robot.dim == EXAMPLE_DIM
    assert problem.objective_count == 2
    assert problem.s_len == EXAMPLE_NUM_POINTS
    assert a_socp.shape == (EXAMPLE_NUM_POINTS,)
    assert t_s.shape == (EXAMPLE_NUM_POINTS,)
    assert s_t.ndim == 1
    assert s_t.size > 0

    assert np.all(np.isfinite(a_socp))
    assert np.all(np.isfinite(t_s))
    assert np.all(np.isfinite(s_t))
    assert np.all(a_socp >= -1.0e-10)
    assert np.isclose(a_socp[0], 0.0)
    assert np.isclose(a_socp[-1], 0.0)
    assert t_final > 0.0
    assert np.isclose(t_s[0], 0.0)
    assert np.isclose(t_s[-1], t_final)
    assert np.isclose(s_t[0], 0.0)
    assert np.isclose(s_t[-1], 1.0)


def test_copp2_socp_accepts_default_options():
    normalize = np.ones(EXAMPLE_DIM, dtype=np.float64)
    _s, _robot, problem = _build_copp2_socp_reference_problem(
        [
            copp.objective.Time(1.0),
            copp.objective.ThermalEnergy(0.1, normalize),
        ],
        num_points=101,
    )

    a_socp = copp.solver.copp2_socp.solve(problem)

    assert a_socp.shape == (101,)


def test_copp2_socp_accepts_total_variation_torque_objective():
    normalize = np.ones(EXAMPLE_DIM, dtype=np.float64)
    _s, _robot, problem = _build_copp2_socp_reference_problem(
        [
            copp.objective.Time(1.0),
            copp.objective.TotalVariationTorque(0.1, normalize),
        ],
        num_points=101,
    )

    a_socp = copp.solver.copp2_socp.solve(problem)

    assert a_socp.shape == (101,)
    assert np.all(np.isfinite(a_socp))


def test_copp2_socp_expert_returns_diagnostics():
    normalize = np.ones(EXAMPLE_DIM, dtype=np.float64)
    _s, _robot, problem = _build_copp2_socp_reference_problem(
        [
            copp.objective.Time(1.0),
            copp.objective.ThermalEnergy(0.1, normalize),
        ],
        num_points=101,
    )

    result = copp.solver.copp2_socp.solve_expert(problem)

    assert result.a is not None
    assert result.a.shape == (101,)
    assert result.x.ndim == 1
    assert result.z.ndim == 1
    assert result.s.ndim == 1
    assert result.solver_status in (
        copp.clarabel.SolverStatus.SOLVED,
        copp.clarabel.SolverStatus.ALMOST_SOLVED,
    )
    assert result.iterations > 0
    assert result.linsolver.nnz_a > 0
    assert np.isfinite(result.obj_val)
    assert np.isfinite(result.obj_val_dual)
    assert result.objective_value is not None
    assert np.isfinite(result.objective_value)
    assert result.objective_terms is not None
    assert result.objective_terms.shape == (2,)
