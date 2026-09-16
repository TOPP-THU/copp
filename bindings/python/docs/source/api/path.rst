Path
====

.. currentmodule:: copp_py.path

Path objects own the geometric path ``q(s)`` used by every solver family.
Python defaults to sample-major matrices, where rows are station samples or
waypoints and columns are dimensions. Pass ``layout="dim_major"`` only when
sharing arrays with an external dim-major pipeline.

All path evaluation methods return ``PathDerivatives``. ``evaluate_q`` is the
position-only form and fills only ``out.q``; ``evaluate_up_to_2nd`` and
``evaluate_up_to_3rd`` fill ``out.q`` plus the requested derivative fields.
Use the fields explicitly:

.. code-block:: python

   q = path.evaluate_q(s).q
   out = path.evaluate_up_to_2nd(s)
   q, dq, ddq = out.q, out.dq, out.ddq

Waypoint paths are built with ``Path.from_waypoints_interpolating``, which
passes through every waypoint (``Path.from_waypoints`` is an equivalent
alias), or with ``Path.from_waypoints_fitting``, a tolerance-bounded C4
quintic fit whose diagnostics are available from ``Path.smoothing_report``.
Both constructor families are unstable: names, signatures, and configuration
types may change as more waypoint algorithms are added.

.. autoclass:: SplineConfig
   :members:

.. autoclass:: SmoothingConfig
   :members:

.. autoclass:: SmoothingReport
   :members:

.. autoclass:: PathDerivatives
   :members:

.. autoclass:: Path
   :members:
