\page cpp_objectives_clarabel Objectives And Clarabel

COPP solvers optimize convex objectives in addition to satisfying TOPP-style
constraints.  Clarabel-backed solvers also expose raw solver settings and expert
diagnostics.

## Include

```cpp
#include <copp/objective.hpp>
#include <copp/clarabel.hpp>
```

or:

```cpp
#include <copp/copp.hpp>
```

## Objective Descriptors

Objective factories live in `copp::objective`:

```cpp
std::vector<copp::Objective> objectives{
    copp::objective::Time(1.0),
    copp::objective::ThermalEnergy(0.1, {1.0, 1.0}),
};
```

The built-in objective support is:

| Objective | COPP2-SOCP | COPP3-SOCP |
|---|---:|---:|
| `Time` | yes | yes |
| `ThermalEnergy` | yes | yes |
| `Linear` | yes | yes |
| `TotalVariationTorque` | yes | yes |

For `Linear`, expected vector lengths depend on solver order:

- COPP2: `alpha.size() == s_len`, `beta.size() == s_len - 1`
- COPP3: `alpha.size() == s_len`, `beta.size() == s_len`

Torque-related objectives use the robot's inverse-dynamics callback when one is
installed.  Otherwise they use point-mass dynamics `tau = ddq`.

## Clarabel Options

`copp::clarabel::Options` combines COPP's acceptance policy with raw Clarabel
settings:

```cpp
copp::clarabel::Options options;
options.allow_almost_solved = true;
options.clarabel_settings.max_iter = 200;
options.clarabel_settings.verbose = false;
```

`copp::clarabel::Settings` mirrors the current raw Clarabel setting surface, so
advanced users can tune tolerances, iteration limits, linear solver choices, and
equilibration behavior without dropping down to Rust.

## Expert Results

`solve` returns only an accepted profile:

```cpp
auto a = copp::solver::copp2_socp::solve(problem, options);
```

`solve_expert` returns diagnostics even when no profile is accepted:

```cpp
auto expert = copp::solver::copp2_socp::solve_expert(problem, options);

if (expert.a) {
    std::cout << "accepted profile length = " << expert.a->size() << "\n";
} else {
    std::cout << "Clarabel status = "
              << static_cast<int>(expert.solver_status)
              << ", iterations = "
              << expert.iterations
              << "\n";
}
```

Expert results include raw Clarabel vectors, residuals, status, objective value
where applicable, per-objective terms, and linear-solver metadata.

## Complete COPP2-SOCP Example

```cpp
std::vector<double> s{0.0, 0.5, 1.0};
std::vector<double> amax{1.0, 1.0, 1.0};

copp::Robot robot(1, s.size());
robot.append_s(s);
robot.constraints().add_constraint_1st(amax, 0);

namespace copp2 = copp::solver::copp2_socp;

copp2::Problem problem{
    robot,
    {copp::objective::Time(1.0)},
    copp::IndexInterval{0, s.size() - 1},
    copp::Boundary2{0.0, 0.0},
};

copp::clarabel::Options options;
options.allow_almost_solved = true;

auto expert = copp2::solve_expert(problem, options);
if (expert.a) {
    auto time = copp::interpolation::s_to_t_topp2(s, *expert.a);
}
```

## Tutorial Sources

- `bindings/cpp/examples/copp2_socp.cpp`
- `bindings/cpp/examples/copp3_socp.cpp`
- `bindings/cpp/examples/robot_inverse_dynamics.cpp`
