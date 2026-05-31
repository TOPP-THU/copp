Robot And Constraints
=====================

``Robot`` is the recommended high-level container for sampled path derivatives,
physical limits, optional inverse dynamics, and the underlying raw constraint
buffer. Use ``Constraints`` directly when you already have path-domain rows and
do not need robot derivative storage.

.. autoclass:: copp_py.robot.Robot
   :members:

.. autoclass:: copp_py.constraints.Constraints
   :members:
