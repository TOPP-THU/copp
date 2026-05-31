Solver Selection
================

The Python namespaces follow the Rust solver-module layout:

.. list-table::
   :header-rows: 1

   * - Problem class
     - Python modules
     - Typical output
     - Use when
   * - TOPP2
     - ``copp.solver.topp2_ra``, ``copp.solver.reach_set2``
     - ``a`` profile
     - You need a fast time-optimal baseline under second-order constraints.
   * - COPP2
     - ``copp.solver.copp2_socp``
     - ``a`` profile
     - You need convex objectives under second-order constraints.
   * - TOPP3
     - ``copp.solver.topp3_lp``, ``copp.solver.topp3_socp``
     - ``Profile3rd(a, b)``
     - You need jerk-aware time-optimal timing.
   * - COPP3
     - ``copp.solver.copp3_socp``
     - ``Profile3rd(a, b)``
     - You need jerk-aware convex-objective timing.

Recommended Starting Points
---------------------------

Use ``copp.solver.topp2_ra.solve`` first when you need a robust second-order
baseline or a seed for a third-order run. It is the simplest solver to get
working because it needs only ``Constraints``, an interval, and boundary values
for ``a``.

Use ``copp.solver.topp3_socp.solve`` when you want the strongest open-source
third-order optimality quality and can afford a conic solve. In most workflows,
first call ``copp.solver.topp2_ra.solve`` to build ``a_linearization``.

Use ``copp.solver.topp3_lp.solve`` as a faster approximation only after
benchmarking on your own paths. It can be attractive when jerk bounds are
loose, but it can be suboptimal when jerk limits are tight.

Use ``copp.solver.copp2_socp.solve`` or ``copp.solver.copp3_socp.solve`` when
traversal time is not the only objective. These solvers accept objective
descriptors such as
``Time``, ``ThermalEnergy``, ``TotalVariationTorque``, and ``Linear``.

Strict and Expert APIs
----------------------

Clarabel-backed solvers have two forms:

.. code-block:: python

   profile = copp.solver.topp3_socp.solve(problem, options)
   result = copp.solver.topp3_socp.solve_expert(problem, options)

The strict form returns a profile or raises ``CoppError`` when no accepted
profile is available. The expert form returns raw solver diagnostics so you can
inspect ``solver_status``, residuals, objective values, and whether
``result.profile`` is present.

Reachability Artifacts
----------------------

Reachability solvers expose intermediate data because it is often useful for
debugging feasibility. ``copp.solver.reach_set2.backward`` and
``copp.solver.reach_set2.bidirectional`` return one-dimensional lower/upper
intervals for ``a``.
