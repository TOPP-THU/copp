TOPP3-SOCP
==========

TOPP3-SOCP solves the third-order time-optimal problem with Clarabel. It is the
recommended open-source third-order method when solution quality matters more
than raw solve speed.

Like other TOPP3/COPP3 wrappers, the problem descriptor requires an
``a_linearization`` profile. In practice, a TOPP2-RA seed is usually good enough
for the first iteration, and the previous TOPP3-SOCP solution is a natural seed
for the second iteration.

The strict API returns a ``Profile3rd``. Use ``socp_expert`` when you need raw
Clarabel status, residuals, primal/dual vectors, or accepted-profile checks.

Runnable Example
----------------

.. literalinclude:: ../../../examples/topp3_socp.py
   :language: python
   :linenos:

Related API
-----------

See :func:`copp_py.solver.topp3_socp.solve`, :func:`copp_py.solver.topp3_socp.solve_expert`, and
the Clarabel reference in :doc:`../api/clarabel`.
