TOPP2 Reach Sets
================

Reach-set construction exposes the feasible range of the second-order state
``a`` along the path. For station ``k``, the result stores a lower and upper
bound:

.. math::

   a_{\min,k} \le a_k \le a_{\max,k}.

The backward-only reach set enforces the terminal boundary. The bidirectional
reach set enforces both the start and terminal boundaries, so it is the better
debugging view when a TOPP2 problem appears infeasible.

Runnable Example
----------------

.. literalinclude:: ../../../examples/reach_set2.py
   :language: python
   :linenos:

Related API
-----------

See :class:`copp_py.solver.reach_set2.ReachSet`,
:func:`copp_py.solver.reach_set2.backward`, and
:func:`copp_py.solver.reach_set2.bidirectional`.
