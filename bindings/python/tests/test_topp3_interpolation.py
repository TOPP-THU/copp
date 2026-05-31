import numpy as np
import pytest

import copp_py as copp


def test_profile3rd_properties_are_copies():
    a = np.array([1.0, 1.0, 1.0], dtype=np.float64)
    b = np.array([0.0, 0.0, 0.0], dtype=np.float64)

    profile = copp.Profile3rd(a, b, num_stationary=(0, 0))

    assert len(profile) == 3
    assert profile.len == 3
    assert profile
    assert profile.num_stationary == (0, 0)
    assert profile.num_stationary_start == 0
    assert profile.num_stationary_end == 0
    assert np.allclose(profile.a, a)
    assert np.allclose(profile.b, b)

    a[0] = 9.0
    profile_a = profile.a
    profile_a[1] = 7.0

    assert np.allclose(profile.a, [1.0, 1.0, 1.0])


def test_topp3_interpolation_smoke():
    s = np.array([0.0, 0.5, 1.0], dtype=np.float64)
    profile = copp.Profile3rd(
        np.array([1.0, 1.0, 1.0], dtype=np.float64),
        np.array([0.0, 0.0, 0.0], dtype=np.float64),
    )

    t_final, t_s = copp.interpolation.s_to_t_topp3(s, profile)
    assert np.isclose(t_final, 1.0)
    assert np.allclose(t_s, [0.0, 0.5, 1.0])

    s_uniform = copp.interpolation.t_to_s_topp3_uniform(s, profile, t_s, 0.25)
    assert np.allclose(s_uniform, [0.0, 0.25, 0.5, 0.75, 1.0])

    t_sample = np.array([0.0, 0.2, 0.8, 1.0], dtype=np.float64)
    s_nonuniform = copp.interpolation.t_to_s_topp3_samples(s, profile, t_s, t_sample)
    assert np.allclose(s_nonuniform, t_sample)

    s_compat = copp.interpolation.t_to_s_topp3(s, profile, t_s, dt=0.25)
    assert np.allclose(s_compat, s_uniform)


def test_profile3rd_rejects_mismatched_lengths():
    with pytest.raises(ValueError, match="same length"):
        copp.Profile3rd(
            np.array([1.0, 1.0], dtype=np.float64),
            np.array([0.0], dtype=np.float64),
        )
