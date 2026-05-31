COPP3-SOCP
==========

COPP3-SOCP extends the third-order conic formulation to convex objectives. The
path and robot setup is the same as TOPP3-SOCP, but the problem descriptor
stores a ``Robot`` and an objective list instead of only a raw constraint proxy.

This distinction matters for objective terms such as thermal energy and torque
variation. Those objectives need robot derivative data and, when available, an
inverse-dynamics callback.

Runnable Example
----------------

.. literalinclude:: ../../../examples/copp3_socp.py
   :language: python
   :linenos:

Related API
-----------

See :doc:`../api/copp3`, :doc:`../api/objective`, and :doc:`../api/clarabel`.
