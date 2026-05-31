Errors and Diagnostics
======================

Exception Model
---------------

Python wrapper argument errors use ordinary Python exceptions such as
``ValueError``. Examples include invalid array dtype/shape, mutually exclusive
keyword arguments, or unsupported string tokens.

Errors returned by the solver core use ``CoppError`` and its typed subclasses.
These usually mean the path, constraints, solver options, or numerical backend
rejected an otherwise well-formed Python call.

Verbosity
---------

Most option objects accept ``verbosity``. You may pass either an enum value or a
string:

.. code-block:: python

   options = copp.solver.topp2_ra.Options(verbosity="summary")

The accepted strings are ``"silent"``, ``"summary"``, ``"debug"``, and
``"trace"``. Keep ``"silent"`` for batch runs and use ``"summary"`` or
``"debug"`` when investigating infeasibility.

Clarabel Strict vs Expert Calls
-------------------------------

Strict Clarabel-backed calls return only an accepted high-level profile:

.. code-block:: python

   profile = copp.solver.copp3_socp.solve(problem, options)

Expert calls expose raw diagnostics:

.. code-block:: python

   result = copp.solver.copp3_socp.solve_expert(problem, options)
   print(result.solver_status)
   print(result.iterations)
   print(result.r_prim, result.r_dual)

If ``result.profile`` is ``None``, Clarabel did not produce a profile accepted
by the configured status policy. The raw status and residuals are still useful
for tuning tolerances or debugging scaling.

Reachability Debugging
----------------------

For TOPP2, inspect ``ReachSet2.a_min`` and ``ReachSet2.a_max`` to find where
the feasible interval collapses. For TOPP3/COPP3, use the Clarabel expert
results to inspect solver status, residuals, and accepted-profile availability
near the suspected failure.
