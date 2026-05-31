import numpy as np

import copp_py as copp


def test_copp2_interpolation_smoke():
    s = np.array([0.0, 0.5, 1.0], dtype=np.float64)
    a = np.array([1.0, 1.0, 1.0], dtype=np.float64)

    b = copp.interpolation.a_to_b_topp2(s, a)
    assert np.allclose(b, [0.0, 0.0])

    t_final, t_s = copp.interpolation.s_to_t_topp2(s, a)
    assert np.isclose(t_final, 1.0)
    assert np.allclose(t_s, [0.0, 0.5, 1.0])

    s_uniform = copp.interpolation.t_to_s_topp2_uniform(s, a, t_s, 0.25)
    assert np.allclose(s_uniform, [0.0, 0.25, 0.5, 0.75, 1.0])

    t_sample = np.array([0.0, 0.2, 0.8, 1.0], dtype=np.float64)
    s_nonuniform = copp.interpolation.t_to_s_topp2_samples(s, a, t_s, t_sample)
    assert np.allclose(s_nonuniform, t_sample)

    s_compat = copp.interpolation.t_to_s_topp2(s, a, t_s, dt=0.25)
    assert np.allclose(s_compat, s_uniform)
