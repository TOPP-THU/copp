\page cpp_solvers Solver Namespaces

Solver entry points live under `copp::solver`.  The namespace layout follows the
Rust and Python APIs: each solver family owns its `Problem`, `Options`, `Result`,
and `solve` names when that is natural for C++.

## Solver Selection

| Namespace | Order | Objective | Backend | Typical use |
|---|---:|---|---|---|
| `topp2_ra` | 2 | time | reachability analysis | fast TOPP2 baseline |
| `reach_set2` | 2 | none | reachable set | feasibility diagnostics |
| `copp2_socp` | 2 | convex | Clarabel SOCP | conic formulation and diagnostics |
| `topp3_lp` | 3 | time | Clarabel LP | linearized TOPP3 baseline |
| `topp3_socp` | 3 | time | Clarabel SOCP | iterative convex refinement |
| `copp3_socp` | 3 | convex | Clarabel SOCP | third-order convex optimization |

## TOPP2-RA

Input: a `ConstraintsRef`, closed station interval, endpoint values for
`a = dot{s}^2`, and optional RA tolerances.

Output: optimized `std::vector<double>` node profile `a(s)`.

```cpp
namespace topp2 = copp::solver::topp2_ra;

topp2::Problem problem{
    constraints.ref(),
    copp::IndexInterval{0, s.size() - 1},
    copp::Boundary2{0.0, 0.0},
};

topp2::Options options;
options.verbosity = copp::Verbosity::Silent;

auto a = topp2::solve(problem, options);
auto reach = topp2::reach_set_bidirectional(problem, options);
```

## COPP2-SOCP

Input: a `Robot`, objective list, closed station interval, endpoint values, and
Clarabel options.

Output: accepted `a(s)` profile or expert Clarabel diagnostics.

```cpp
namespace copp2 = copp::solver::copp2_socp;

copp2::Problem problem{
    robot,
    {copp::objective::Time(1.0)},
    copp::IndexInterval{0, s.size() - 1},
    copp::Boundary2{0.0, 0.0},
};

copp::clarabel::Options options;
options.allow_almost_solved = true;

auto a = copp2::solve(problem, options);
auto expert = copp2::solve_expert(problem, options);
```

## Shared TOPP3 Problem

TOPP3-LP, TOPP3-SOCP, and COPP3-SOCP use the shared third-order
problem descriptor.  The solver namespaces provide aliases, so user code can
write `topp3_socp::Problem` even though the underlying type is shared.

```cpp
namespace topp3_socp = copp::solver::topp3_socp;

topp3_socp::Problem problem{
    robot.constraints(),
    a_linearization,
    0,
    copp::Boundary3{0.0, 0.0, 0.0, 0.0},
};
```

Constructing a third-order problem immediately refreshes Rust's linearized
third-order constraint cache in the borrowed `Robot` or `Constraints` storage.
Do not construct or solve third-order problems concurrently against the same
constraint owner.

By default every jerk row is linearized directly at `a_linearization[k]`.  The
optional trailing `b_linearization` argument of `topp3::Problem` and
`copp3_socp::Problem` selects adaptive anchoring: each
row is anchored where it is nearly active at the feasible state
`(a_linearization, b_linearization)`.  Both profiles must be the `a` and `b` of
the same previously solved third-order profile (never a TOPP2 profile, whose
`b` is discretized differently).  Prefer it when `a` spans a wide range, and
especially when `a` can approach zero (around `1e-10`): direct linearization
anchors the upper and lower row of one axis at the same small
`a_linearization[k]`, which cancels `b` and `c` and leaves
`a[k] <= 3 * a_linearization[k]`.  On profiles well away from zero the default
is cheaper and nearly identical.

```cpp
topp3_socp::Problem adaptive{
    robot.constraints(),
    profile.a,
    0,
    copp::Boundary3{0.0, 0.0, 0.0, 0.0},
    copp::solver::topp3::StationaryBounds{},
    1.0e-10,
    profile.b,
};
```

## TOPP3-LP And TOPP3-SOCP

LP is the lighter linearized formulation.  SOCP is usually used as a sequential
convex refinement: solve once, rebuild the problem with the returned `profile.a`,
then solve again.

```cpp
namespace topp3_socp = copp::solver::topp3_socp;

copp::clarabel::Options options;
options.allow_almost_solved = true;

auto profile = topp3_socp::solve(problem, options);

topp3_socp::Problem next_problem{
    robot.constraints(),
    profile.a,
    0,
    copp::Boundary3{0.0, 0.0, 0.0, 0.0},
};

profile = topp3_socp::solve(next_problem, options);
```

Third-order profiles whose `a` comes close to zero can have an interpolated
`a(s)` that dips below zero between stations.  Call
`Profile3rd::force_positive_a` before `s_to_t_topp3` and then check the
delivered profile with `ConstraintsRef::exceed_topp3`:

```cpp
profile.force_positive_a(s);
auto exceed = robot.constraints().exceed_topp3(profile); // each entry <= 0 when feasible
```

## COPP3-SOCP

COPP3-SOCP has its own problem class because it needs a `Robot` and objective
list in addition to the third-order linearization profile.

```cpp
namespace copp3 = copp::solver::copp3_socp;

copp3::Problem problem{
    robot,
    {copp::objective::Time(1.0),
     copp::objective::ThermalEnergy(0.1, {1.0})},
    a_linearization,
    0,
    copp::Boundary3{0.0, 0.0, 0.0, 0.0},
};

auto expert = copp3::solve_expert(problem, options);
if (expert.profile) {
    auto time = copp::interpolation::s_to_t_topp3(s, *expert.profile);
}
```

## Tutorial Sources

- `bindings/cpp/examples/topp2_ra.cpp`
- `bindings/cpp/examples/reach_set2.cpp`
- `bindings/cpp/examples/copp2_socp.cpp`
- `bindings/cpp/examples/topp3_lp.cpp`
- `bindings/cpp/examples/topp3_socp.cpp`
- `bindings/cpp/examples/copp3_socp.cpp`
