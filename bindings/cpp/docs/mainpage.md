# COPP C++ API

COPP solves Optimal Path Parameterization problems.  A geometric path is already
known,

@f[
q = q(s), \qquad s \in [s_{\min}, s_{\max}],
@f]

and COPP computes a time law

@f[
s = s(t)
@f]

so that the time-domain trajectory `q(s(t))` satisfies velocity,
acceleration, torque, jerk, or user-supplied path constraints while optimizing
the selected objective.  In other words, the library turns a geometry-space path
into a time-space schedule:

@f[
q(s) \quad \Longrightarrow \quad s(t).
@f]

The C++ facade follows the same public concepts as the Rust and Python APIs, but
uses C++ ownership, RAII, namespaces, exceptions, and optional no-throw overloads.
For normal user code, include the umbrella header:

```cpp
#include <copp/copp.hpp>
```

All public symbols live in namespace `copp`. Solver namespaces mirror the Rust
and Python layout, for example `copp::solver::topp2_ra`,
`copp::solver::copp2_socp`, and `copp::solver::topp3_socp`.

## TOPP And COPP

- **TOPP** means Time-Optimal Path Parameterization.  The usual objective is
  minimum traversal time.
- **COPP** means Convex-Objective Path Parameterization.  It supports convex
  objectives such as time, thermal energy, linear torque costs, and selected
  torque-variation costs.

Second-order and third-order families use different state variables:

@f[
a(s) = \dot{s}^2, \qquad b(s) = \ddot{s}, \qquad
c(s) = \frac{s^{(3)}}{\dot{s}},
\qquad s^{(3)} = \frac{d^3s}{dt^3}.
@f]

- **TOPP2/COPP2** optimize `a(s)` under first- and second-order constraints.
  They cover common velocity, acceleration, and torque workflows.
- **TOPP3/COPP3** optimize both `a(s)` and `b(s)` under third-order constraints.
  They are used when jerk-level effects or third-order convex objectives matter.

## Recommended Workflow

1. Build or sample a `copp::Path`.
2. Store station samples and physical limits in `copp::Robot`, or use
   `copp::Constraints` directly for raw mathematical constraints.
3. Choose a solver namespace under `copp::solver`.
4. Convert the returned profile to cumulative time with
   `copp::interpolation`.
5. Sample `s(t)` if the downstream controller needs a uniform time grid.

## Complete First Example

The example below starts with waypoints, builds high-level robot constraints,
solves a TOPP2-RA problem, and converts the optimized profile to time.

```cpp
#include <copp/copp.hpp>

#include <iostream>
#include <vector>

int main()
{
    // Each inner list is one waypoint vector.  COPP stores waypoints internally
    // as a (dim x n_points) column-major matrix.
    auto path = copp::Path::from_waypoints({
        {0.0, 0.0},
        {0.5, 0.25},
        {1.0, 1.0},
    });

    std::vector<double> s{0.0, 0.25, 0.5, 0.75, 1.0};
    std::vector<double> v_upper{2.0, 2.0};
    std::vector<double> v_lower{-2.0, -2.0};
    std::vector<double> a_upper{4.0, 4.0};
    std::vector<double> a_lower{-4.0, -4.0};

    copp::Robot robot(2, s.size());
    robot.append_s(s)
        .set_q_from_path_2nd(path, 0, s.size())
        .add_velocity_limits(v_upper, v_lower, 0, s.size())
        .add_acceleration_limits(a_upper, a_lower, 0, s.size());

    namespace topp2 = copp::solver::topp2_ra;

    topp2::Problem problem{
        robot.constraints(),
        copp::IndexInterval{0, s.size() - 1},
        copp::Boundary2{0.0, 0.0},
    };

    auto a = topp2::solve(problem);
    auto time = copp::interpolation::s_to_t_topp2(s, a);

    std::cout << "final time = " << time.t_final << "\n";
}
```

## Where To Go Next

- @ref cpp_core_types explains `Span`, `Matrix`, ownership, errors, and Eigen
  adapters.
- @ref cpp_path explains waypoint, parametric, and batch-evaluator paths.
- @ref cpp_robot_constraints explains physical robot limits and raw
  mathematical constraints.
- @ref cpp_interpolation explains TOPP2/TOPP3 time conversion and profile
  slicing.
- @ref cpp_objectives_clarabel explains objective descriptors and Clarabel
  settings.
- @ref cpp_solvers explains TOPP2, COPP2, TOPP3, COPP3 solver entry points.

The generated output also includes the public headers and all tutorial example
source files.  Private bridge headers under `include/copp/detail` are
intentionally excluded from this public reference.
