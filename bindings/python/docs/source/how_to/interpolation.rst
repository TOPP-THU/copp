Convert Profiles to Time Samples
================================

Solvers work in the path domain. Controllers and plots usually need time-domain
samples. The interpolation helpers provide the standard conversion.

Second Order
------------

TOPP2/COPP2 solvers return ``a``. Since

.. math::

   a(s) = \dot{s}^2,

the arrival-time profile is obtained from

.. math::

   \frac{dt}{ds} = \frac{1}{\sqrt{a(s)}}.

Use ``s_to_t_topp2`` first:

.. code-block:: python

   t_final, t_s = copp.interpolation.s_to_t_topp2(s, a, t0=0.0)

Then sample ``s(t)`` on a uniform grid:

.. code-block:: python

   s_t = copp.interpolation.t_to_s_topp2_uniform(s, a, t_s, dt=1.0e-3)

or on explicit query times:

.. code-block:: python

   s_t = copp.interpolation.t_to_s_topp2_samples(s, a, t_s, t_sample)

Third Order
-----------

TOPP3/COPP3 solvers return a ``Profile3rd`` object with node-based ``a`` and
``b`` arrays. Use the third-order helpers:

.. code-block:: python

   t_final, t_s = copp.interpolation.s_to_t_topp3(s, profile, t0=0.0)
   s_t = copp.interpolation.t_to_s_topp3_uniform(s, profile, t_s, dt=1.0e-3)

The compatibility wrappers ``t_to_s_topp2`` and ``t_to_s_topp3`` accept either
``dt`` or ``t_sample``. New code should prefer the explicit ``*_uniform`` and
``*_samples`` names so the sampling mode is obvious.

Common Checks
-------------

``s`` must be one-dimensional and strictly increasing. ``t_s`` must have the
same length as ``s``. Explicit ``t_sample`` arrays must be finite, non-empty,
and strictly increasing. Out-of-range query times are returned as ``NaN`` by
the COPP core.
