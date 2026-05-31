COPP2-SOCP
==========

COPP2-SOCP solves a second-order convex-objective path-parameterization
problem with Clarabel. It uses the same path and constraint construction as
TOPP2, but the problem descriptor also owns an objective list:

.. code-block:: python

   objectives = [
       copp.objective.Time(1.0),
       copp.objective.ThermalEnergy(0.1, np.ones(dim)),
   ]

With no inverse-dynamics callback installed, ``Robot`` uses point-mass dynamics
for torque-related objective terms and constraints:

.. math::

   \tau = \ddot{q}.

For real robots, pass a callable or object with ``inverse_dynamics(q, dq, ddq)``
when constructing ``Robot``.

Runnable Example
----------------

.. literalinclude:: ../../../examples/copp2_socp.py
   :language: python
   :linenos:

Related API
-----------

See :doc:`../api/copp2`, :doc:`../api/objective`, and :doc:`../api/clarabel`.
