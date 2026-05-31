"""Profile interpolation and time-parameter conversion utilities."""

from ._native import (
    Profile3rd as Profile3rd,
    a_to_b_topp2 as a_to_b_topp2,
    s_to_t_topp2 as s_to_t_topp2,
    s_to_t_topp3 as s_to_t_topp3,
    t_to_s_topp2 as t_to_s_topp2,
    t_to_s_topp2_samples as t_to_s_topp2_samples,
    t_to_s_topp2_uniform as t_to_s_topp2_uniform,
    t_to_s_topp3 as t_to_s_topp3,
    t_to_s_topp3_samples as t_to_s_topp3_samples,
    t_to_s_topp3_uniform as t_to_s_topp3_uniform,
)

__all__: list[str]
