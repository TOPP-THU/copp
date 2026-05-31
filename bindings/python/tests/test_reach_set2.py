import numpy as np

import copp_py as copp


EXAMPLE_DIM = 3
EXAMPLE_NUM_POINTS = 1001


class LissajousPath2nd:
    """Same deterministic 3-axis path used by Rust and C reach-set examples."""

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


def _build_rust_c_aligned_reach_set2_example():
    path = copp.Path.from_evaluator_2nd(LissajousPath2nd(), 0.0, 1.0)
    s = np.linspace(0.0, 1.0, EXAMPLE_NUM_POINTS, dtype=np.float64)

    robot = copp.Robot(EXAMPLE_DIM, capacity=EXAMPLE_NUM_POINTS)
    robot.append_s(s)
    robot.set_q_from_path_2nd(path, 0, EXAMPLE_NUM_POINTS)

    upper = np.ones(EXAMPLE_DIM, dtype=np.float64)
    lower = -upper
    robot.add_velocity_limits(
        upper,
        lower,
        start_idx_s=0,
        length=EXAMPLE_NUM_POINTS,
    )
    robot.add_acceleration_limits(
        upper,
        lower,
        start_idx_s=0,
        length=EXAMPLE_NUM_POINTS,
    )

    problem = copp.solver.reach_set2.Problem(
        robot.constraints,
        idx_s_interval=(0, EXAMPLE_NUM_POINTS - 1),
        a_boundary=(0.0, 0.0),
    )

    return problem


def test_reach_set2_matches_rust_and_c_example_setup():
    problem = _build_rust_c_aligned_reach_set2_example()
    options = copp.solver.reach_set2.Options()

    reach_back = copp.solver.reach_set2.backward(problem, options)
    reach_bidir = copp.solver.reach_set2.bidirectional(problem, options)

    assert len(reach_back) == EXAMPLE_NUM_POINTS
    assert len(reach_bidir) == EXAMPLE_NUM_POINTS
    assert reach_back.len == EXAMPLE_NUM_POINTS
    assert reach_bidir.len == EXAMPLE_NUM_POINTS

    for reach in (reach_back, reach_bidir):
        assert isinstance(reach, copp.solver.reach_set2.ReachSet)
        assert reach.a_max.shape == (EXAMPLE_NUM_POINTS,)
        assert reach.a_min.shape == (EXAMPLE_NUM_POINTS,)
        assert np.all(np.isfinite(reach.a_max))
        assert np.all(np.isfinite(reach.a_min))
        assert np.all(reach.a_max >= reach.a_min - 1.0e-10)

    assert np.isclose(reach_back.a_min[-1], 0.0)
    assert np.isclose(reach_back.a_max[-1], 0.0)
    assert np.isclose(reach_bidir.a_min[0], 0.0)
    assert np.isclose(reach_bidir.a_max[0], 0.0)
    assert np.isclose(reach_bidir.a_min[-1], 0.0)
    assert np.isclose(reach_bidir.a_max[-1], 0.0)


def test_reach_set2_accepts_default_options():
    problem = _build_rust_c_aligned_reach_set2_example()

    reach = copp.solver.reach_set2.bidirectional(problem)

    assert len(reach) == EXAMPLE_NUM_POINTS
