import numpy as np
import pytest

import copp_py as copp


class PolynomialEvaluator:
    dim = 2

    def evaluate_q(self, s):
        q = np.empty((s.size, self.dim), dtype=np.float64)
        q[:, 0] = s**3
        q[:, 1] = s**2 + 1.0
        return q

    def evaluate_up_to_2nd(self, s):
        q = self.evaluate_q(s)

        dq = np.empty_like(q)
        dq[:, 0] = 3.0 * s**2
        dq[:, 1] = 2.0 * s

        ddq = np.empty_like(q)
        ddq[:, 0] = 6.0 * s
        ddq[:, 1] = 2.0

        return q, dq, ddq

    def evaluate_up_to_3rd(self, s):
        q, dq, ddq = self.evaluate_up_to_2nd(s)

        dddq = np.empty_like(q)
        dddq[:, 0] = 6.0
        dddq[:, 1] = 0.0

        return q, dq, ddq, dddq


def test_path_from_evaluator_3rd_sample_major_smoke():
    path = copp.Path.from_evaluator_3rd(PolynomialEvaluator(), -1.0, 1.0)
    s = np.array([-1.0, 0.0, 0.5], dtype=np.float64)

    out = path.evaluate_up_to_3rd(s)

    assert path.dim == 2
    assert path.s_range == (-1.0, 1.0)
    assert out.q.shape == (3, 2)
    assert out.dq.shape == (3, 2)
    assert out.ddq.shape == (3, 2)
    assert out.dddq.shape == (3, 2)
    assert np.allclose(out.q[:, 0], s**3)
    assert np.allclose(out.dq[:, 0], 3.0 * s**2)
    assert np.allclose(out.ddq[:, 0], 6.0 * s)
    assert np.allclose(out.dddq[:, 0], 6.0)


def test_path_from_evaluator_2nd_rejects_third_order_evaluation():
    path = copp.Path.from_evaluator_2nd(PolynomialEvaluator(), -1.0, 1.0)
    s = np.array([-1.0, 0.0, 0.5], dtype=np.float64)

    out = path.evaluate_up_to_2nd(s)

    assert out.q.shape == (3, 2)
    assert out.dq.shape == (3, 2)
    assert out.ddq.shape == (3, 2)
    assert out.dddq is None

    with pytest.raises(copp.CoppError, match="unsupported derivative order"):
        path.evaluate_up_to_3rd(s)


def test_path_from_evaluator_3rd_can_fallback_for_second_order():
    class ThirdOrderOnly:
        dim = 1

        def __init__(self):
            self.calls_3rd = 0

        def evaluate_up_to_3rd(self, s):
            self.calls_3rd += 1
            q = np.ascontiguousarray((s**2)[:, None], dtype=np.float64)
            dq = np.ascontiguousarray((2.0 * s)[:, None], dtype=np.float64)
            ddq = np.full((s.size, 1), 2.0, dtype=np.float64)
            dddq = np.zeros((s.size, 1), dtype=np.float64)
            return q, dq, ddq, dddq

    evaluator = ThirdOrderOnly()
    path = copp.Path.from_evaluator_3rd(evaluator, -1.0, 1.0)
    s = np.array([-0.5, 0.0, 0.5], dtype=np.float64)

    out = path.evaluate_up_to_2nd(s)

    assert evaluator.calls_3rd == 1
    assert out.q.shape == (3, 1)
    assert out.dddq is None
    assert np.allclose(out.q[:, 0], s**2)


def test_path_evaluator_preserves_python_callback_exception():
    class BrokenEvaluator:
        dim = 1

        def evaluate_up_to_2nd(self, s):
            raise RuntimeError("callback failed")

    path = copp.Path.from_evaluator_2nd(BrokenEvaluator(), 0.0, 1.0)
    s = np.array([0.0, 0.5], dtype=np.float64)

    with pytest.raises(RuntimeError, match="callback failed"):
        path.evaluate_up_to_2nd(s)


def test_path_evaluator_invalid_callback_shape_is_value_error():
    class BadShapeEvaluator:
        dim = 1

        def evaluate_up_to_2nd(self, s):
            bad = np.zeros((s.size, 2), dtype=np.float64)
            return bad, bad, bad

    path = copp.Path.from_evaluator_2nd(BadShapeEvaluator(), 0.0, 1.0)
    s = np.array([0.0, 0.5], dtype=np.float64)

    with pytest.raises(ValueError, match="expected shape"):
        path.evaluate_up_to_2nd(s)


def test_path_evaluator_outputs_accept_arraylike():
    class ListEvaluator:
        dim = 1

        def evaluate_up_to_2nd(self, s):
            values = [[float(x)] for x in s]
            zeros = [[0.0] for _ in s]
            return values, values, zeros

    path = copp.Path.from_evaluator_2nd(ListEvaluator(), 0.0, 1.0)
    out = path.evaluate_up_to_2nd([0.0, 0.5, 1.0])

    assert out.q.shape == (3, 1)
    assert np.allclose(out.q[:, 0], [0.0, 0.5, 1.0])
