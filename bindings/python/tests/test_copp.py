"""Basic tests for copp Python bindings."""
import numpy as np
import pytest
import copp


def test_jet3_basic():
    s = copp.Jet3.seed(0.5)
    assert abs(s.v - 0.5) < 1e-15
    assert abs(s.d1 - 1.0) < 1e-15


def test_jet3_sin():
    s = copp.Jet3.seed(0.5)
    result = copp.sin(s)
    assert abs(result.v - np.sin(0.5)) < 1e-14
    assert abs(result.d1 - np.cos(0.5)) < 1e-14


def test_jet3_arithmetic():
    a = copp.Jet3.seed(1.0)
    b = a * 2.0 + copp.Jet3.constant(3.0)
    assert abs(b.v - 5.0) < 1e-15
    assert abs(b.d1 - 2.0) < 1e-15


def test_path_from_waypoints():
    wp = np.array([[0.0, 0.25, 0.5, 0.75, 1.0],
                   [0.0, 0.1, -0.1, 0.2, 0.0]])
    path = copp.Path.from_waypoints(wp)
    derivs = path.evaluate_q(np.array([0.0, 0.5, 1.0]))
    assert derivs.q.shape == (2, 3)
    np.testing.assert_allclose(derivs.q[:, 0], wp[:, 0], atol=1e-10)
    np.testing.assert_allclose(derivs.q[:, -1], wp[:, -1], atol=1e-10)


def test_path_from_parametric():
    def my_path(s):
        return [copp.sin(s), copp.cos(s)]

    path = copp.Path.from_parametric(my_path, 0.0, 1.0)
    assert path.dim == 2
    derivs = path.evaluate_up_to_2nd(np.linspace(0, 1, 50))
    assert derivs.dq is not None
    assert derivs.ddq is not None


def test_robot_basic():
    robot = copp.Robot(dim=3)
    assert robot.dim == 3
