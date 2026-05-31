TOPP2-RA
========

TOPP2-RA is the recommended first solver to try. It solves a time-optimal
second-order problem and returns the node profile

.. math::

   a(s) = \dot{s}^2.

The example uses an analytic Lissajous path, samples it on ``N`` stations, adds
symmetric velocity and acceleration limits, and solves with boundary values
``a(0)=0`` and ``a(1)=0``.

Key Calls
---------

``Path.from_evaluator_2nd`` wraps a Python object that provides
``evaluate_up_to_2nd(s) -> (q, dq, ddq)``. ``Robot.set_q_from_path_2nd`` stores
those derivatives in the constraint buffer. ``Robot.add_velocity_limits`` and
``Robot.add_acceleration_limits`` convert physical axis limits into sampled
path-domain rows.

After solving, ``s_to_t_topp2`` computes the arrival-time profile and
``t_to_s_topp2_uniform`` samples ``s(t)`` at a controller-friendly time step.

Runnable Example
----------------

.. literalinclude:: ../../../examples/topp2_ra.py
   :language: python
   :linenos:

Related API
-----------

See :doc:`../api/topp2`, :doc:`../api/path`, :doc:`../api/robot`, and
the interpolation reference in :doc:`../api/interpolation`.
