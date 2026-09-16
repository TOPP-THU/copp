TOPP3-LP
========

TOPP3-LP solves the third-order time-optimal problem with the Clarabel LP
backend. It is the cheapest open-source third-order method, and a good choice
when solve speed matters more than the last percent of traversal time; use
:doc:`topp3_socp` when solution quality matters more.

Like other TOPP3/COPP3 wrappers, the problem descriptor requires an
``a_linearization`` profile. In practice, a TOPP2-RA seed is usually good enough
for the first iteration, and the previous TOPP3-LP solution is a natural seed
for the second iteration. When reseeding from a third-order solution, pass its
``b`` as ``b_linearization`` together with its ``a`` to linearize the jerk rows
adaptively at that feasible state; see :doc:`../how_to/paths_and_constraints`.

The strict API returns a ``Profile3rd``. Use ``solve_expert`` when you need raw
Clarabel status, residuals, primal/dual vectors, or accepted-profile checks.

Runnable Example
----------------

.. literalinclude:: ../../../examples/topp3_lp.py
   :language: python
   :linenos:

Related API
-----------

See :func:`copp_py.solver.topp3_lp.solve`, :func:`copp_py.solver.topp3_lp.solve_expert`, and
the Clarabel reference in :doc:`../api/clarabel`.
