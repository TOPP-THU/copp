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

``Profile3rd.force_positive_a(s, a_min=1e-12)`` post-processes a profile in
place so interpolated ``a(s)`` stays strictly positive on every interval.
Apply it before ``s_to_t_topp3`` to any profile whose ``a`` touches zero.
Because it rewrites ``a`` and ``b`` without knowing the
limits, audit the result afterwards with
:meth:`copp_py.constraints.Constraints.exceed_topp3`, which checks the
original nonlinear third-order limits:

.. code-block:: python

   profile.force_positive_a(s)
   exceed_1st, exceed_2nd, exceed_3rd = robot.constraints.exceed_topp3(profile)
   t_final, t_s = copp.interpolation.s_to_t_topp3(s, profile)

.. autofunction:: s_to_t_topp3

.. autofunction:: t_to_s_topp3_uniform

.. autofunction:: t_to_s_topp3_samples

.. autofunction:: t_to_s_topp3
