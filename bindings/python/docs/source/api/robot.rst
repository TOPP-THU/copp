Robot And Constraints
=====================

``Robot`` is the recommended high-level container for sampled path derivatives,
physical limits, optional inverse dynamics, and the underlying raw constraint
buffer. Use ``Constraints`` directly when you already have path-domain rows and
do not need robot derivative storage.

``Constraints.exceed_topp2`` and ``Constraints.exceed_topp3`` audit a solved
profile against the stored limits and return maximum violation magnitudes.
Each value is ``<= 0`` when the profile is feasible and positive when violated.
All values are ``NaN`` when fewer than two stations are given, the station
range is unavailable, the station grid is not strictly increasing, the profile
shapes are inconsistent, or the profile contains non-finite values.

.. autoclass:: copp_py.robot.Robot
   :members:

.. autoclass:: copp_py.constraints.Constraints
   :members:
