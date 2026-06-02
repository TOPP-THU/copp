copp-py Python Documentation
============================

``copp_py`` is the Python interface to the COPP path-parameterization core.
Most examples import it as ``import copp_py as copp`` for a shorter local alias.
It converts a geometric path

.. math::

   q = q(s)

into a time law

.. math::

   s = s(t)

that satisfies path, velocity, acceleration, torque, and jerk constraints while
optimizing either traversal time or a convex objective. The Python API groups
the solver families into namespace modules, uses NumPy arrays, and supports
optional Python automatic-differentiation backends for path construction.


.. toctree::
   :maxdepth: 2
   :caption: Guide:

   quickstart
   concepts/index
   tutorials/index
   how_to/index

.. toctree::
   :maxdepth: 2
   :caption: Reference:

   api/index
