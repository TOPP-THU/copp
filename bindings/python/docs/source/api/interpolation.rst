Interpolation
=============

.. currentmodule:: copp_py.interpolation

Interpolation helpers convert solver profiles from the path domain into
arrival times ``t(s)`` and sampled inverse trajectories ``s(t)``.

Second Order
------------

.. autofunction:: a_to_b_topp2

.. autofunction:: s_to_t_topp2

.. autofunction:: t_to_s_topp2_uniform

.. autofunction:: t_to_s_topp2_samples

.. autofunction:: t_to_s_topp2

Third Order
-----------

.. autoclass:: Profile3rd
   :members:

.. autofunction:: s_to_t_topp3

.. autofunction:: t_to_s_topp3_uniform

.. autofunction:: t_to_s_topp3_samples

.. autofunction:: t_to_s_topp3
