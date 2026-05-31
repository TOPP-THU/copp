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
