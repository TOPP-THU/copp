Optimal Path Parameterization
=============================

COPP solves optimal path-parameterization problems. The geometric path is fixed:

.. math::

   q = q(s),\qquad s \in [s_0, s_f].

The solver chooses a monotone time law ``s(t)`` so that the executed trajectory
``q(s(t))`` satisfies the configured limits. This separation is useful in
robotics because geometric planning and timing can be handled independently:
the COPP stage never moves the path away from ``q(s)``.

Second-Order Variables
----------------------

For velocity, acceleration, and many torque constraints, the useful path-domain
variables are

.. math::

   a(s) = \dot{s}^2,\qquad b(s) = \ddot{s}.

The chain rule gives

.. math::

   \dot{q} = q_s \dot{s},

and

.. math::

   \ddot{q} = q_{ss}\dot{s}^2 + q_s\ddot{s}
            = q_{ss}a + q_s b.

After sampling the path, second-order limits are stored as rows of the form

.. math::

   u_i(s_k)a_k + v_i(s_k)b_k \le h_i(s_k),

plus first-order upper bounds on ``a``. TOPP2/COPP2 solvers optimize over this
state and return a node-based ``a`` profile.

Third-Order Variables
---------------------

Third-order solvers add a control-like term

.. math::

   c(s) = \frac{\dddot{s}}{\dot{s}},

which is convenient when working in the path domain. Jerk-level rows use the
sampled form

.. math::

   \sqrt{a_k}\,(r_i a_k + s_i b_k + t_i c_k + d_i) \le f_i.

Because these rows depend on ``sqrt(a)``, the Python TOPP3/COPP3 problem
descriptors require an ``a_linearization`` profile. A common practical pattern
is:

1. Solve TOPP2-RA to obtain a feasible second-order seed ``a``.
2. Optionally tighten ``robot.constraints`` with ``amax_substitute``.
3. Build a TOPP3 or COPP3 problem with that seed.
4. Rebuild the third-order problem with the previous third-order solution when
   running a second refinement iteration.

Problem Descriptors and Live Data
---------------------------------

Python ``Problem`` objects are lightweight descriptors over live model data.
``Topp2Problem`` stores a reference to a ``Constraints`` proxy, while
``Copp2Problem`` stores a reference to the ``Robot`` plus Python-owned
objective descriptors. They do not copy the robot or constraint buffer. If you
mutate the referenced ``robot`` or ``constraints`` after constructing the
problem, later ``validate()`` and solver calls use the current contents.

The descriptor configuration itself is immutable. To change an interval,
boundary value, objective list, or third-order linearization profile, construct
a new problem object.

For third-order problems, construction follows the Rust
``build_with_linearization()`` model. ``Topp3Problem`` and ``Copp3Problem``
copy the supplied ``a_linearization`` into the descriptor, then build a Rust
problem and refresh the affine jerk-linearization rows cached in the referenced
constraint buffer. Raw nonlinear jerk rows remain unchanged. Calling
``validate()`` or a third-order solver later repeats that refresh against the
current robot or constraint data and the descriptor's copied
``a_linearization``.

When refining a third-order solve with a new profile, build a new descriptor:

.. code-block:: python

   problem = copp.solver.topp3_socp.Problem(
       robot.constraints,
       profile.a,
       idx_s_start=0,
       a_boundary=(0.0, 0.0),
       b_boundary=(0.0, 0.0),
   )

TOPP vs COPP
------------

TOPP means time-optimal path parameterization. The objective is traversal time:

.. math::

   J = t_f.

COPP means convex-objective path parameterization. The solver optimizes a
weighted convex objective over the path-domain profile. In Python, objective
terms are created under ``copp.objective``:

.. code-block:: python

   objectives = [
       copp.objective.Time(1.0),
       copp.objective.ThermalEnergy(0.1, normalize),
   ]

The same path and constraints can therefore be used for time-optimal planning
or for application-specific convex cost tradeoffs.

Python Data Contract
--------------------

Numerical inputs accept NumPy-compatible ``ArrayLike`` values and are copied
into contiguous ``float64`` buffers at the native extension boundary. Most sampled
matrices default to ``sample_major`` layout:

.. code-block:: text

   shape = (n_samples, dim)

The alternative ``dim_major`` layout stores dimensions first:

.. code-block:: text

   shape = (dim, n_samples)

Both layouts are accepted where the API exposes a ``layout`` argument. New
Python code should use the default ``sample_major`` layout unless it is sharing
arrays directly with an external dim-major pipeline.
