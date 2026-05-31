Quickstart
==========

The shortest reliable Python workflow is:

1. Build a geometric path ``q(s)``.
2. Sample the path on a station grid ``s`` and store path derivatives in a
   ``Robot`` or ``Constraints`` buffer.
3. Add physical limits.
4. Build a TOPP or COPP problem descriptor.
5. Run a solver and convert the path-domain profile into time-domain samples.

The second-order state used by TOPP2/COPP2 is

.. math::

   a(s) = \dot{s}^2,\qquad b(s) = \ddot{s}.

TOPP2/COPP2 solvers return the node profile ``a``. The helper
``s_to_t_topp2`` integrates it into arrival times, and ``t_to_s_topp2_uniform``
or ``t_to_s_topp2_samples`` inverts the mapping for controller or plotting
samples.

Minimal TOPP2-RA Example
------------------------

The following example constructs an analytic path, applies symmetric velocity
and acceleration limits, solves TOPP2-RA, and samples ``s(t)`` on a uniform
time grid.

.. literalinclude:: ../../examples/topp2_ra.py
   :language: python
   :linenos:

Import Style
------------

The package follows the Rust crate layout. Solver entry points live under
``copp.solver`` with one module per algorithm:

.. code-block:: python

   import copp_py as copp

   problem = copp.solver.topp2_ra.Problem(...)
   options = copp.solver.topp2_ra.Options()
   a_profile = copp.solver.topp2_ra.solve(problem, options)

Modeling helpers live in focused namespaces such as ``copp.path``,
``copp.robot``, ``copp.constraints``, ``copp.objective``,
``copp.interpolation``, and ``copp.clarabel``. A few common modeling types are
also re-exported at package root for interactive use, but solver-specific
classes and functions are intentionally kept in ``copp.solver``.

Next Steps
----------

Read :doc:`concepts/opp` for the variables and constraint model, then choose a
solver with :doc:`concepts/solver_selection`. The tutorial pages embed the
same runnable files stored in ``bindings/python/examples``.
