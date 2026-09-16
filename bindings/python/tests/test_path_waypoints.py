import numpy as np
import pytest

import copp_py as copp


def test_path_from_waypoints_sample_major_smoke():
    waypoints = np.array(
        [
            [0.0, 0.0],
            [0.3, 0.2],
            [0.7, -0.1],
            [1.0, 0.0],
        ],
        dtype=np.float64,
    )
    cfg = copp.SplineConfig(
        order=5,
        s_min=0.0,
        s_max=1.0,
        out_of_range=copp.OutOfRangeMode.ERROR,
        parametrization=copp.Parametrization.UNIFORM,
        start_state=np.zeros((2, 2), dtype=np.float64),
        end_state=np.zeros((2, 2), dtype=np.float64),
    )

    path = copp.Path.from_waypoints(
        waypoints,
        cfg,
        layout=copp.MatrixLayout.SAMPLE_MAJOR,
    )

    assert path.dim == 2
    assert path.s_range == (0.0, 1.0)

    s = np.array([0.0, 0.5, 1.0], dtype=np.float64)
    out_q = path.evaluate_q(s)
    assert out_q.q.shape == (3, 2)
    assert out_q.dq is None
    assert out_q.ddq is None
    assert out_q.dddq is None
    assert np.allclose(out_q.q[[0, -1]], waypoints[[0, -1]])

    out = path.evaluate_up_to_3rd(s)
    assert out.q.shape == (3, 2)
    assert out.dq.shape == (3, 2)
    assert out.ddq.shape == (3, 2)
    assert out.dddq.shape == (3, 2)


def test_path_from_waypoints_accepts_string_options():
    waypoints = np.array(
        [
            [0.0, 0.0, 0.0],
            [0.5, 0.2, -0.1],
            [1.0, 0.0, 0.1],
        ],
        dtype=np.float64,
    )
    cfg = copp.SplineConfig(out_of_range="clamp", parametrization="uniform")
    path = copp.Path.from_waypoints(waypoints, cfg, layout="sample_major")

    s = np.array([-1.0, 0.0, 1.0, 2.0], dtype=np.float64)
    out = path.evaluate_up_to_2nd(s)

    assert out.q.shape == (4, 3)
    assert out.dq.shape == (4, 3)
    assert out.ddq.shape == (4, 3)
    assert out.dddq is None
    assert np.all(np.isfinite(out.q))


def test_path_from_waypoints_accepts_direct_options():
    waypoints = np.array(
        [
            [0.0, 0.0],
            [0.5, 0.25],
            [1.0, 0.0],
        ],
        dtype=np.float64,
    )

    path = copp.Path.from_waypoints(
        waypoints,
        order=5,
        s_min=0.0,
        s_max=1.0,
        out_of_range="clamp",
    )

    s = np.array([-0.5, 0.0, 1.0, 1.5], dtype=np.float64)
    out = path.evaluate_q(s)

    assert path.dim == 2
    assert path.s_range == (0.0, 1.0)
    assert out.q.shape == (4, 2)
    assert np.allclose(out.q[0], waypoints[0])
    assert np.allclose(out.q[-1], waypoints[-1])


def test_path_arraylike_inputs_and_path_error_subclass():
    path = copp.Path.from_waypoints(
        [[0.0], [0.5], [1.0]],
        order=5,
        s_min=0.0,
        s_max=1.0,
    )

    out = path.evaluate_q([0.0, 0.25, 1.0])

    assert out.q.shape == (3, 1)
    assert issubclass(copp.PathError, copp.CoppError)

    with pytest.raises(copp.PathError):
        path.evaluate_q([-0.1])


def _noisy_waypoints(n_points=41):
    s = np.linspace(0.0, 1.0, n_points, dtype=np.float64)
    rng = np.random.default_rng(7)
    clean = np.column_stack([s, np.sin(2.0 * np.pi * s), 0.5 * s**2])
    return clean + 1.0e-4 * rng.standard_normal(clean.shape)


def test_path_from_waypoints_interpolating_matches_alias():
    waypoints = np.array(
        [
            [0.0, 0.0],
            [0.3, 0.2],
            [0.7, -0.1],
            [1.0, 0.0],
        ],
        dtype=np.float64,
    )
    cfg = copp.SplineConfig(order=5, out_of_range="clamp")
    s = np.linspace(-0.1, 1.1, 13, dtype=np.float64)

    reference = copp.Path.from_waypoints_interpolating(waypoints, cfg).evaluate_up_to_3rd(s)
    candidates = [
        copp.Path.from_waypoints(waypoints, cfg),
        copp.Path.from_waypoints_interpolating(waypoints, order=5, out_of_range="clamp"),
    ]
    for path in candidates:
        out = path.evaluate_up_to_3rd(s)
        for field in ("q", "dq", "ddq", "dddq"):
            np.testing.assert_array_equal(getattr(out, field), getattr(reference, field))
        assert path.smoothing_report is None

    dim_major = copp.Path.from_waypoints_interpolating(waypoints.T, cfg, layout="dim_major")
    np.testing.assert_array_equal(dim_major.evaluate_q(s).q, reference.q.T)


def test_path_from_waypoints_fitting_report_respects_per_axis_tolerance():
    waypoints = _noisy_waypoints()
    tolerance = np.array([1.0e-2, 2.0e-3], dtype=np.float64)

    path = copp.Path.from_waypoints_fitting(waypoints, tolerance=tolerance, axes=[2, 0])
    report = path.smoothing_report

    assert isinstance(report, copp.SmoothingReport)
    assert report.axes == [2, 0]
    assert report.max_errors.shape == (2,)
    assert np.all(report.max_errors <= tolerance)
    assert report.segments >= 1
    assert report.interpolated_segments >= 1
    assert report.checked_intervals > 0
    assert path.s_range == (0.0, 1.0)

    s = np.linspace(0.0, 1.0, waypoints.shape[0], dtype=np.float64)
    q = path.evaluate_q(s).q
    np.testing.assert_allclose(q[[0, -1]], waypoints[[0, -1]], atol=1.0e-9)
    assert np.all(np.abs(q[:, 2] - waypoints[:, 2]) <= tolerance[0])
    assert np.all(np.abs(q[:, 0] - waypoints[:, 0]) <= tolerance[1])
    np.testing.assert_allclose(q[:, 1], waypoints[:, 1], atol=1.0e-9)


def test_path_from_waypoints_fitting_accepts_config_and_parameters():
    waypoints = _noisy_waypoints(21)
    parameters = np.linspace(2.0, 4.0, waypoints.shape[0], dtype=np.float64)
    cfg = copp.SmoothingConfig(tolerance=5.0e-3, parameters=parameters, out_of_range="clamp")

    assert cfg.tolerance == 5.0e-3
    assert cfg.axes is None
    np.testing.assert_array_equal(cfg.parameters, parameters)
    assert cfg.max_refinements == 20
    assert cfg.max_segments == 20000
    assert cfg.out_of_range == copp.OutOfRangeMode.CLAMP

    path = copp.Path.from_waypoints_fitting(waypoints, cfg)
    report = path.smoothing_report
    assert path.s_range == (2.0, 4.0)
    assert report.axes == [0, 1, 2]
    assert report.interpolated_segments == 0
    assert np.all(report.max_errors <= 5.0e-3)
    assert path.evaluate_up_to_3rd([1.0, 3.0, 5.0]).dddq.shape == (3, 3)

    dim_major = copp.Path.from_waypoints_fitting(waypoints.T, cfg, layout="dim_major")
    np.testing.assert_allclose(dim_major.evaluate_q([3.0]).q, path.evaluate_q([3.0]).q.T)

    cfg.tolerance = [1.0e-2, 1.0e-2, 1.0e-2]
    assert isinstance(cfg.tolerance, np.ndarray)
    cfg.axes = (0, 1, 2)
    assert cfg.axes == [0, 1, 2]

    with pytest.raises(ValueError, match="not both"):
        copp.Path.from_waypoints_fitting(waypoints, cfg, tolerance=1.0e-2)


def test_path_from_waypoints_fitting_failures_raise_path_error():
    waypoints = _noisy_waypoints()

    with pytest.raises(copp.PathError):
        copp.Path.from_waypoints_fitting(waypoints, axes=[0, 0])
    with pytest.raises(copp.PathError):
        copp.Path.from_waypoints_fitting(waypoints, tolerance=1.0e-9, max_segments=1)
    with pytest.raises(copp.PathError):
        copp.Path.from_waypoints_fitting(waypoints, tolerance=[1.0e-3, 1.0e-3], axes=[0, 1, 2])
    with pytest.raises(ValueError, match="axes"):
        copp.Path.from_waypoints_fitting(waypoints, axes=[-1])
    with pytest.raises(ValueError, match="tolerance"):
        copp.Path.from_waypoints_fitting(waypoints, tolerance=np.ones((2, 2)))
