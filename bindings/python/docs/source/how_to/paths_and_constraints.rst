Build Paths and Constraints
===========================

Path Options
------------

Use ``Path.from_waypoints`` when your path is a sequence of waypoints. The
default Python matrix layout is ``sample_major``:

.. code-block:: text

   waypoints.shape == (n_points, dim)

Use ``Path.from_evaluator_2nd`` or ``Path.from_evaluator_3rd`` when you already
have analytic derivatives. The evaluator receives a one-dimensional ``float64``
array of path parameters and returns contiguous arrays in the selected layout.

Use ``Path.from_jax``, ``Path.from_autograd``, ``Path.from_casadi``, or
``Path.from_sympy`` when you want Python to differentiate a scalar-parametric
function or symbolic expression. These optional backends are imported lazily
only when the corresponding constructor is called.

Path Evaluation
---------------

Path evaluation methods return a consistent ``PathDerivatives`` object rather
than switching between arrays and tuples. For position-only evaluation,
``evaluate_q`` fills only the ``q`` field:

.. code-block:: python

   out = path.evaluate_q(s)
   q = out.q
   assert out.dq is None
   assert out.ddq is None
   assert out.dddq is None

Higher-order calls fill the corresponding derivative fields on the same result
type:

.. code-block:: python

   out = path.evaluate_up_to_2nd(s)
   q = out.q
   dq = out.dq
   ddq = out.ddq

Robot vs Constraints
--------------------

``Robot`` owns station samples, sampled path derivatives, optional inverse
dynamics, and a raw ``Constraints`` buffer. It is the recommended entry point
for most applications:

.. code-block:: python

   robot = copp.Robot(dim, capacity=n)
   robot.append_s(s)
   robot.set_q_from_path_3rd(path, 0, n)
   robot.add_velocity_limits(upper, lower, start_idx_s=0, length=n)
   robot.add_acceleration_limits(upper, lower, start_idx_s=0, length=n)
   robot.add_jerk_limits(upper, lower, start_idx_s=0, length=n)

``Constraints`` can be constructed directly when you already have path-domain
rows and do not need robot derivative storage or torque objectives.

Problem Descriptors Reference Live Buffers
------------------------------------------

Solver-specific ``Problem`` classes keep references to the data they are built
from. A TOPP problem built from ``robot.constraints`` continues to see that
same constraint buffer; a COPP problem built from ``robot`` continues to see
that same robot, including later constraint and path-sample changes.

This is useful for workflows that update limits or station windows, but it also
means a problem is not a frozen snapshot:

.. code-block:: python

   problem = copp.solver.topp2_ra.Problem(
       robot.constraints,
       idx_s_interval=(0, len(robot) - 1),
       a_boundary=(0.0, 0.0),
   )

   robot.add_velocity_limits(new_upper, new_lower, start_idx_s=0, length=len(robot))
   problem.validate()  # validates against the updated constraints

The problem's own configuration is immutable. Build a new problem when changing
the interval, boundary values, objective list, or third-order
``a_linearization``.

Third-order problem construction and validation also refresh cached affine jerk
rows in the referenced constraint buffer. This mirrors Rust
``build_with_linearization()``: nonlinear jerk constraints are left intact, and
the linearized rows are regenerated from the problem's copied
``a_linearization``. To refine a third-order solution, construct a new problem
with the new ``profile.a``.

Limit Shapes
------------

One-dimensional limits have shape ``(dim,)`` and are broadcast across the
station interval. Two-dimensional limits must match the station interval and
selected layout. With the default ``sample_major`` layout:

.. code-block:: text

   limits.shape == (length, dim)

The lower limit arrays should be negative for symmetric bounds such as
``[-1, 1]``. The upper arrays should be positive.

Inverse Dynamics
----------------

If no inverse-dynamics callback is installed, torque methods use point-mass
dynamics:

.. math::

   \tau = \ddot{q}.

For robot-specific torque limits or torque objectives, construct ``Robot`` with
a callable:

.. code-block:: python

   def inverse_dynamics(q, dq, ddq):
       return tau

   robot = copp.Robot(dim, capacity=n, inverse_dynamics=inverse_dynamics)

The callback receives contiguous one-dimensional ``float64`` arrays of shape
``(dim,)`` and must return a value convertible to the same shape.
