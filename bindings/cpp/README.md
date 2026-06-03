# COPP C++ Binding

[![License: MIT](https://img.shields.io/badge/license-MIT-yellow.svg)](../../LICENSE) [![Website](https://img.shields.io/badge/website-copp.pro-2ff0d8)](https://copp.pro/) [![Docs](https://img.shields.io/badge/docs-docs.copp.pro-1f6feb)](https://docs.copp.pro/) [![C++](https://img.shields.io/badge/C%2B%2B-bindings-00599c)](#copp-c-binding)

## Convex-Objective Path Parameterization

This directory contains the C++ facade for COPP. The C++ API is designed to feel like a small RAII wrapper over the Rust/Python concepts rather than a thin exposure of the C ABI: public types live in `namespace copp`, objects own their resources, and normal calls throw `copp::Error`.

COPP solves Optimal Path Parameterization (OPP) problems. A geometric path is known,

$$
q = q(s), \qquad s \in [s_{\min}, s_{\max}],
$$

and COPP computes a time law

$$
s = s(t)
$$

so that the executed trajectory `q(s(t))` satisfies velocity, acceleration, torque, jerk, or user-provided constraints while optimizing the selected objective.

Second- and third-order profiles use the same notation as the Rust crate:

$$
a(s) = \dot{s}^2, \qquad b(s) = \ddot{s}, \qquad c(s) = \frac{\dddot{s}}{\dot{s}}.
$$

The algorithmic background, open-source algorithm availability, benchmark tables, citation information, and collaboration contact details live in the [COPP repository README](https://github.com/TOPP-THU/copp#readme). This page focuses on building, linking, installing, and using the C++ interface.

> **Open-source / PRO note:** this README documents the open-source C++ binding and its CMake target `copp::copp`. COPP PRO provides additional licensed solvers, support options, and commercial collaboration paths; see the repository-level PRO section or contact [hello@copp.pro](mailto:hello@copp.pro) if those capabilities are relevant to your application. Future prebuilt SDK packages should be checked from [GitHub Releases](https://github.com/TOPP-THU/copp/releases).

## API Availability

| Problem class  | C++ API                                                                                                                                            |
| -------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| Core utilities | `copp::version`, `copp::Error`, `copp::Expected`, `copp::Span`, `copp::Matrix`, `copp::MatrixView`, `copp::MatrixRef`, boundary and interval types |
| Path           | waypoint splines, scalar-parametric `Jet3` paths, batch 2nd/3rd-order evaluator paths, derivative evaluation                                       |
| Robot          | station grids, path sampling, velocity/acceleration/jerk/torque limits, raw constraints, inverse-dynamics callbacks                                |
| TOPP2          | `copp::solver::topp2_ra`, `copp::solver::reach_set2`, TOPP2 interpolation helpers                                                                  |
| COPP2          | `copp::solver::copp2_socp`, objective descriptors, expert results                                                                                  |
| TOPP3          | `copp::solver::topp3_lp`, `copp::solver::topp3_socp`, TOPP3 interpolation helpers                                                                  |
| COPP3          | `copp::solver::copp3_socp`, objective descriptors, expert results                                                                                  |
| Clarabel       | raw `copp::clarabel::Settings`, `copp::clarabel::Options`, solver status, residuals, linear-solver information                                     |
| Eigen          | optional adapters in `copp/eigen.hpp` for borrowing Eigen vectors/matrices and mapping returned `copp::Matrix` values                              |

Runnable examples are in [examples](examples/). They are built as CMake targets named `example_*`.

## Quick Start

### Prerequisites

You need:

- Cargo and a Rust toolchain;
- CMake 3.16 or newer;
- a C++17 compiler toolchain;
- Eigen3 if you enable `COPP_CPP_WITH_EIGEN=ON`;
- Doxygen and Graphviz if you want to generate local C++ API documentation.

The source-tree CMake flow is Cargo-first: it imports an existing Rust/C++ native library from `target/release/` and does not rebuild Rust automatically. Run the commands below from the repository root, and rerun the Cargo command after changing Rust code, `src/ffi/cpp`, `bindings/cpp/src`, or public C++ headers; otherwise CMake may link an old native artifact.

### Build From Source

Build only the C++ facade feature. The Cargo command builds the Rust native artifact that CMake imports:

```powershell
cargo build --release --lib --features cpp
cmake -S bindings/cpp -B bindings/cpp/build `
  -DCOPP_CPP_WITH_EIGEN=OFF `
  -DCOPP_INSTALL_C_ABI_TARGET=OFF
cmake --build bindings/cpp/build --config Release
```

Build the C++ facade and export a separate C ABI target from the same CMake package. Use this when downstream consumers should also receive the C ABI target:

```powershell
cargo build --release --lib --features cpp,c
cmake -S bindings/cpp -B bindings/cpp/build -DCOPP_CPP_WITH_EIGEN=OFF
cmake --build bindings/cpp/build --config Release
```

`COPP_LINK_STATIC=ON` is the default. Set `-DCOPP_LINK_STATIC=OFF` to use the shared C++ facade library (`copp_cpp.dll`, `libcopp_cpp.so`, or `libcopp_cpp.dylib`). The installed CMake target propagates the required C++ import macro for shared builds. On Windows source-tree dynamic builds, the CMake example/test targets copy the runtime DLL next to the executable.

If Eigen is installed and discoverable by CMake, omit `-DCOPP_CPP_WITH_EIGEN=OFF` to enable `copp/eigen.hpp`.

### Install And Link

Install the headers, native library, and CMake package config:

```powershell
cmake --install bindings/cpp/build --config Release --prefix <install-prefix>
```

Downstream CMake projects can then link the installed package:

```cmake
find_package(copp CONFIG REQUIRED)

add_executable(app main.cpp)
target_link_libraries(app PRIVATE copp::copp)
```

Configure the downstream project with the install prefix:

```powershell
cmake -S app -B app/build -DCMAKE_PREFIX_PATH=<install-prefix>
cmake --build app/build --config Release
```

When `COPP_INSTALL_C_ABI_TARGET=ON`, the same static package also exports `copp::c_abi` for downstream code that explicitly wants the C ABI target. C++ applications should normally link `copp::copp`. In static packages, `copp::c_abi` receives the same platform link libraries and C++ link-language metadata as the C++ facade because the imported Cargo artifact may contain C++ bridge objects. Shared C++ packages use a CMake-built facade library and should keep `COPP_INSTALL_C_ABI_TARGET=OFF`; use the C ABI package/workflow for shared C consumers.

For static packages, the exported target records common platform native libraries. If a platform or toolchain needs extra native libraries, configure COPP with:

```powershell
cmake -S bindings/cpp -B bindings/cpp/build -DCOPP_LINK_STATIC=ON -DCOPP_STATIC_LINK_LIBRARIES="lib1;lib2"
```

For future prebuilt packages, check [GitHub Releases](https://github.com/TOPP-THU/copp/releases). Source builds remain useful when you are working from a development branch or changing Rust/C++ binding code.

### Build Examples And Tests

Tests and examples are disabled by default for SDK-style consumption. Enable them explicitly when validating the binding locally:

```powershell
cmake -S bindings/cpp -B bindings/cpp/build `
  -DCOPP_CPP_WITH_EIGEN=OFF `
  -DCOPP_INSTALL_C_ABI_TARGET=OFF `
  -DCOPP_BUILD_CPP_TESTS=ON `
  -DCOPP_BUILD_CPP_EXAMPLES=ON
cmake --build bindings/cpp/build --config Release
ctest --test-dir bindings/cpp/build --output-on-failure -C Release
```

Register examples as CTest tests:

```powershell
cmake -S bindings/cpp -B bindings/cpp/build `
  -DCOPP_CPP_WITH_EIGEN=OFF `
  -DCOPP_INSTALL_C_ABI_TARGET=OFF `
  -DCOPP_BUILD_CPP_EXAMPLES=ON `
  -DCOPP_TEST_CPP_EXAMPLES=ON
cmake --build bindings/cpp/build --config Release
ctest --test-dir bindings/cpp/build --output-on-failure -C Release
```

Build one example target:

```powershell
cmake --build bindings/cpp/build --config Release --target example_topp2_ra
```

Keep tests or examples disabled when configuring a consumer-style build:

```powershell
cmake -S bindings/cpp -B bindings/cpp/build -DCOPP_BUILD_CPP_TESTS=OFF
cmake -S bindings/cpp -B bindings/cpp/build -DCOPP_BUILD_CPP_EXAMPLES=OFF
```

## General Workflow

Most C++ examples follow the same shape:

1. Build a path from waypoints, a scalar-parametric `Jet3` callback, or a batch evaluator callback.
2. Build a strictly increasing station grid `s`.
3. Create a `copp::Robot` or an owning raw `copp::Constraints` buffer.
4. Append stations and sample path derivatives into the robot when using robot-level limits or objectives.
5. Add velocity, acceleration, jerk, torque, or raw mathematical constraints.
6. Build a borrowed solver problem descriptor such as `topp2_ra::Problem`, `copp2_socp::Problem`, `topp3::Problem`, or `copp3_socp::Problem`.
7. Call the solver namespace function.
8. Convert the returned path-domain profile into timing data with `copp::interpolation`.
9. Evaluate the original path at sampled `s(t)` values to generate downstream trajectory references.

For second-order problems, solvers usually return an `a` profile sampled on the station grid, where $a(s)=\dot{s}^2$. For third-order problems, solvers return `copp::Profile3rd`, containing `a`, `b`, and stationary-boundary metadata.

## Minimal Program

```cpp
#include <copp/copp.hpp>

#include <iostream>

int main()
{
    std::cout << "COPP version: " << copp::version() << "\n";
    return 0;
}
```

## TOPP2-RA Example

The example below mirrors the Rust TOPP2-RA quick-start pattern: define a smooth path, sample it on a station grid, add velocity/acceleration limits, solve for `a(s) = dot{s}^2`, and convert the profile to time.

```cpp
#include <copp/copp.hpp>

#include <cmath>
#include <iostream>
#include <vector>

int main()
{
    constexpr std::size_t dim = 3;
    constexpr std::size_t n = 1001;

    auto path = copp::Path::from_evaluator_2nd(
        dim,
        0.0,
        1.0,
        [](copp::Span<const double> s,
           copp::MatrixRef q,
           copp::MatrixRef dq,
           copp::MatrixRef ddq)
        {
            constexpr double pi = 3.14159265358979323846;
            const double freq[3] = {2.0 * pi, 3.0 * pi, 5.0 * pi};
            const double phase[3] = {0.0, 0.3, 0.7};

            for (std::size_t j = 0; j < s.size(); ++j)
            {
                for (std::size_t axis = 0; axis < 3; ++axis)
                {
                    const double x = freq[axis] * s[j] + phase[axis];
                    q(axis, j) = std::sin(x);
                    dq(axis, j) = freq[axis] * std::cos(x);
                    ddq(axis, j) = -freq[axis] * freq[axis] * std::sin(x);
                }
            }
        });

    std::vector<double> s(n);
    for (std::size_t k = 0; k < n; ++k)
    {
        s[k] = static_cast<double>(k) / static_cast<double>(n - 1);
    }

    const std::vector<double> upper(dim, 1.0);
    const std::vector<double> lower(dim, -1.0);

    copp::Robot robot(dim, n);
    robot.append_s(s)
        .set_q_from_path_2nd(path, 0, n)
        .add_velocity_limits(upper, lower, 0, n)
        .add_acceleration_limits(upper, lower, 0, n);

    namespace topp2 = copp::solver::topp2_ra;

    topp2::Problem problem{
        robot.constraints(),
        copp::IndexInterval{0, n - 1},
        copp::Boundary2{0.0, 0.0},
    };

    auto a = topp2::solve(problem);
    auto time = copp::interpolation::s_to_t_topp2(s, a, 0.0);
    auto s_t = copp::interpolation::t_to_s_topp2_uniform(s, a, time.t_s, 1.0e-3);

    std::cout << "TOPP2-RA done.\n";
    std::cout << "t_final = " << time.t_final << " s\n";
    std::cout << "a_profile.len() = " << a.size() << "\n";
    std::cout << "s(t) samples = " << s_t.size() << "\n";

    return 0;
}
```

The checked-in source is available at [`examples/topp2_ra.cpp`](examples/topp2_ra.cpp).

## Path Construction

Waypoint paths are the shortest route when you already have discrete path points. Each inner initializer list is one waypoint vector; the facade converts it to the `(dim x n_points)` matrix shape expected internally.

```cpp
auto path = copp::Path::from_waypoints({
    {0.0, 0.0},
    {0.5, 0.25},
    {1.0, 1.0},
});

std::vector<double> s{0.0, 0.5, 1.0};
auto out = path.evaluate_up_to_2nd(s);
double q0_mid = out.q(0, 1);
```

Use `copp::SplineConfig` when you need a non-default spline range, clamping policy, order, or endpoint derivative states. Boundary derivative matrices use shape `(dim, m)` where `m = (order - 1) / 2`; column `r` stores the `(r + 1)`-th derivative at that endpoint. Leaving `start_state` or `end_state` empty means zero boundary derivatives.

{% raw %}
```cpp
copp::SplineConfig config;
config.order = 3;
config.start_state = copp::Matrix::from_columns({{2.0}});
config.end_state = copp::Matrix::from_columns({{2.0}});

auto path = copp::Path::from_waypoints({{0.0}, {1.0}}, config);
```
{% endraw %}

Scalar-parametric paths use `copp::Jet3` to propagate derivatives up to third order through ordinary arithmetic.

```cpp
auto path = copp::Path::from_parametric(
    [](copp::Jet3 s)
    {
        return std::vector<copp::Jet3>{
            copp::sin(s),
            copp::cos(2.0 * s),
        };
    },
    0.0,
    1.0);

auto out = path.evaluate_up_to_3rd(std::vector<double>{0.0, 0.5, 1.0});
```

Batch evaluator paths are useful when you already have a C++ path library and want to fill derivative matrices directly. The callback receives all query samples in one call, which avoids crossing the C++/Rust boundary once per station.

```cpp
auto path = copp::Path::from_evaluator_3rd(
    1,
    0.0,
    1.0,
    [](copp::Span<const double> s,
       copp::MatrixRef q,
       copp::MatrixRef dq,
       copp::MatrixRef ddq,
       copp::MatrixRef dddq)
    {
        for (std::size_t j = 0; j < s.size(); ++j)
        {
            const double x = s[j];
            q(0, j) = x * x * x;
            dq(0, j) = 3.0 * x * x;
            ddq(0, j) = 6.0 * x;
            dddq(0, j) = 6.0;
        }
    });
```

`evaluate_q`, `evaluate_up_to_2nd`, and `evaluate_up_to_3rd` all return `copp::PathDerivatives`. The requested derivative order determines which optional fields are populated.

## Robot And Constraints

`copp::Robot` is the high-level entry for physical limits. It stores stations, path samples, and optional inverse dynamics. If no inverse-dynamics callback is installed, torque limits and torque objectives use the point-mass fallback `tau = ddq`.

```cpp
copp::Robot robot(dim, s.size());
robot.append_s(s)
    .set_q_from_path_2nd(path, 0, s.size())
    .add_velocity_limits(upper, lower, 0, s.size())
    .add_acceleration_limits(upper, lower, 0, s.size());
```

Install inverse dynamics when torque constraints or torque-dependent objectives should use your robot model.

```cpp
robot.set_inverse_dynamics(
    [](copp::Span<const double> q,
       copp::Span<const double> dq,
       copp::Span<const double> ddq,
       copp::Span<double> tau)
    {
        for (std::size_t i = 0; i < tau.size(); ++i)
        {
            tau[i] = ddq[i] + 0.05 * dq[i] + std::sin(q[i]);
        }
    });
```

`copp::Constraints` is an owning raw mathematical constraint buffer for workflows that do not need robot-side physical-limit conversion. `Robot::constraints()` and `Constraints::ref()` return borrowed `ConstraintsRef` values used by TOPP-style problem descriptors.

```cpp
copp::Constraints constraints(1, s.size());
constraints.append_s(s)
    .add_constraint_1st(std::vector<double>{1.0, 1.0, 1.0}, 0);

auto ref = constraints.ref();
```

Raw constraints follow the Rust/Python mathematical convention:

- first-order: `a <= amax`;
- second-order: `acc_a * a + acc_b * b <= acc_max`;
- third-order: `sqrt(a) * (jerk_a*a + jerk_b*b + jerk_c*c + jerk_d) <= jerk_max`.

## Solver Namespaces

| Namespace                  | Order | Objective | Backend               | Typical use                                     |
| -------------------------- | ----: | --------- | --------------------- | ----------------------------------------------- |
| `copp::solver::topp2_ra`   |     2 | time      | reachability analysis | fast second-order time-optimal planning         |
| `copp::solver::reach_set2` |     2 | none      | reachable set         | second-order feasibility diagnostics            |
| `copp::solver::copp2_socp` |     2 | convex    | Clarabel SOCP         | conic optimization and expert diagnostics       |
| `copp::solver::topp3_lp`   |     3 | time      | Clarabel LP           | linearized third-order baseline                 |
| `copp::solver::topp3_socp` |     3 | time      | Clarabel SOCP         | iterative third-order convex refinement         |
| `copp::solver::copp3_socp` |     3 | convex    | Clarabel SOCP         | third-order convex-objective conic optimization |

`topp2_ra` solves second-order time-optimal problems and returns an accepted `std::vector<double>` profile `a`. Use `reach_set2` when reachable intervals themselves are the main output rather than the recovered profile.

`copp2_socp` solves second-order convex-objective problems through Clarabel. Use `solve` for the accepted profile and `solve_expert` when application code needs solver status, residuals, raw vectors, objective value, or per-objective terms.
`n`topp3_lp` and `topp3_socp` solve third-order time-optimal problems from the shared `copp::solver::topp3::Problem` descriptor. Both return `Profile3rd`; expert variants expose Clarabel diagnostics.
`n`copp3_socp` solves third-order convex-objective problems through Clarabel and exposes expert diagnostics.

## Third-Order Workflow

Third-order solvers operate on the state pair `(a,b)` and usually need a linearization profile `a_linearization`. A common workflow is to start from a simple feasible profile or a TOPP2 result, solve one third-order round, then rebuild the problem with the recovered `profile.a` and solve again.

```cpp
namespace topp3 = copp::solver::topp3;
namespace topp3_socp = copp::solver::topp3_socp;

copp::Boundary3 boundary{0.25, 0.25, 0.0, 0.0};
std::vector<double> a0(s.size(), 0.25);

topp3::Problem first{
    constraints.ref(),
    a0,
    0,
    boundary,
};

copp::clarabel::Options options;
options.allow_almost_solved = true;

auto profile1 = topp3_socp::solve(first, options);

topp3::Problem second{
    constraints.ref(),
    profile1.a,
    0,
    boundary,
};

auto profile2 = topp3_socp::solve(second, options);
auto time = copp::interpolation::s_to_t_topp3(s, profile2, 0.0);
```

`Profile3rd` owns `a`, `b`, and stationary-boundary metadata. Use `Profile3rdRef` or `profile.slice(i, j)` when interpolating or passing only a sub-window without copying profile storage.
`n## Objectives And Clarabel

COPP solvers accept objective descriptors:

```cpp
std::vector<copp::Objective> objectives{
    copp::objective::Time(1.0),
    copp::objective::ThermalEnergy(0.1, {1.0, 1.0}),
};
```

Built-in objective support:

| Objective | COPP2-SOCP | COPP3-SOCP |`n|---|---:|---:|`n| `Time` | yes | yes |`n| `ThermalEnergy` | yes | yes |`n| `Linear` | yes | yes |`n| `TotalVariationTorque` | yes | yes |

Clarabel-backed solvers use `copp::clarabel::Options`:

```cpp
copp::clarabel::Options options;
options.allow_almost_solved = true;
options.clarabel_settings.max_iter = 200;
```

Use `solve_expert` when you need raw Clarabel vectors, status, residuals, objective terms, or linear-solver metadata.

## Data Conventions

`copp::Span<T>` is a non-owning view over a contiguous one-dimensional array. It can be constructed from `std::vector<T>`, `std::array<T, N>`, pointer plus length, or other contiguous storage that outlives the COPP call.

```cpp
std::vector<double> values{0.0, 0.5, 1.0};
copp::Span<const double> from_vector(values);
copp::Span<const double> from_pointer(values.data(), values.size());
```

`copp::Matrix` is an owned column-major storage type. It is intentionally small: it stores matrix data and shape, but it is not a linear algebra package.

```cpp
copp::Matrix matrix(2, 3);
matrix(0, 0) = 0.0;
matrix(1, 0) = 1.0;
```

`Matrix::from_rows` treats each inner list as one row. `Matrix::from_columns` treats each inner list as one column.

```cpp
auto table = copp::Matrix::from_rows({
    {0.0, 0.5, 1.0},
    {0.0, 0.25, 1.0},
});

auto waypoints = copp::Matrix::from_columns({
    {0.0, 0.0},
    {0.5, 0.25},
    {1.0, 1.0},
});
```

{% raw %}
`Path::from_waypoints({{...}, {...}})` uses waypoint-list notation: each inner list is one waypoint vector. If you pass a `MatrixView` instead, the matrix shape is `(dim x n_points)`, where each column is one waypoint.
{% endraw %}

`copp::MatrixView` is a borrowed input view. It accepts column-major, row-major, and strided column-major layouts. Column-major input is the zero-copy fast path. Row-major input may be copied once before entering the Rust core, depending on the operation.

```cpp
auto cm = copp::MatrixView::column_major(data, rows, cols);
auto rm = copp::MatrixView::row_major(data, rows, cols);
auto scm = copp::MatrixView::strided_column_major(data, rows, cols, leading_dim);
```

`copp::MatrixRef` is a writable borrowed column-major matrix view used by evaluator callbacks. Output matrices in callbacks have shape `(dim, s.size())` and are indexed as `(row, column)`.

If `COPP_CPP_WITH_EIGEN=ON`, include `copp/eigen.hpp` to borrow Eigen storage or map returned COPP matrices:

```cpp
#include <copp/eigen.hpp>

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> waypoints(2, 3);
auto path = copp::Path::from_waypoints(copp::eigen::matrix_view(waypoints));

auto out = path.evaluate_q(std::vector<double>{0.0, 1.0});
auto q_eigen = copp::eigen::map(out.q);
```

## Error Handling

The default C++ API throws `copp::Error`. It carries a machine-readable `copp::Status` and an owned diagnostic message.

{% raw %}
```cpp
try
{
    auto path = copp::Path::from_waypoints({{0.0}, {1.0}});
}
catch (const copp::Error &error)
{
    std::cerr << error.what() << "\n";
}
```
{% endraw %}

Selected APIs provide no-throw overloads with `copp::no_throw` as the final argument. The function name remains the same, but the return type becomes `copp::Expected<T>`. This is a status-return convenience API implemented by catching exceptions from the normal throwing API; it is not a promise that the C++ binding can be compiled with C++ exceptions disabled.

{% raw %}
```cpp
auto result = copp::Path::from_waypoints({{0.0}, {1.0}}, copp::no_throw);
if (!result)
{
    std::cerr << result.error().message << "\n";
    return;
}

auto path = std::move(result).value();
```
{% endraw %}

Callbacks may throw `copp::Error` or another `std::exception`. The facade converts callback failures into COPP diagnostics when they cross back into Rust. For predictable diagnostics, prefer throwing `copp::Error` with a specific status when the callback detects invalid user data.

## Header Model

Use the umbrella header for application code:

```cpp
#include <copp/copp.hpp>
```

Public headers are hand-written and live under `bindings/cpp/include/copp/`. Generated Rust-C++ bridge headers are private implementation details and are not part of the installed user-facing API.

| Header                   | Purpose                                                                                 |
| ------------------------ | --------------------------------------------------------------------------------------- |
| `copp/copp.hpp`          | umbrella header                                                                         |
| `copp/core.hpp`          | `Span`, `Matrix`, `MatrixView`, `MatrixRef`, `Expected`, `Error`, boundaries, intervals |
| `copp/path.hpp`          | waypoint paths, Jet3 parametric paths, batch evaluator paths                            |
| `copp/robot.hpp`         | `Robot`, `Constraints`, `ConstraintsRef`, inverse dynamics callbacks                    |
| `copp/interpolation.hpp` | TOPP2/TOPP3 `s_to_t`, `t_to_s`, `Profile3rd`, `Profile3rdRef`                           |
| `copp/objective.hpp`     | `Objective` and `copp::objective::*` factories                                          |
| `copp/clarabel.hpp`      | raw Clarabel settings, options, statuses, solver diagnostics                            |
| `copp/eigen.hpp`         | optional Eigen adapters                                                                 |
| `copp/solver/*.hpp`      | TOPP/COPP solver namespaces                                                             |

## Package Layout

```text
bindings/cpp/
  README.md
  CMakeLists.txt
  include/copp/          # public C++ headers
  src/                   # public facade implementation
  cmake/                 # installed package config templates
  examples/              # runnable tutorial examples
  tests/                 # lightweight C++ smoke tests
  docs/                  # Doxygen source pages and generated HTML
  scripts/               # documentation-generation scripts
```

## Documentation

Generate C++ Doxygen docs:

```powershell
powershell -ExecutionPolicy Bypass -File bindings/cpp/scripts/generate_docs.ps1
```

Important generated pages:

- `bindings/cpp/docs/html/index.html`;
- `bindings/cpp/docs/html/cpp_core_types.html`;
- `bindings/cpp/docs/html/cpp_path.html`;
- `bindings/cpp/docs/html/cpp_robot_constraints.html`;
- `bindings/cpp/docs/html/cpp_interpolation.html`;
- `bindings/cpp/docs/html/cpp_objectives_clarabel.html`;
- `bindings/cpp/docs/html/cpp_solvers.html`.

The generated reference includes every public header, namespace summaries, tutorial pages, and example source pages.

## Examples

Path construction:

- [`examples/path_from_waypoints.cpp`](examples/path_from_waypoints.cpp);
- [`examples/path_from_parametric.cpp`](examples/path_from_parametric.cpp);
- [`examples/path_from_evaluator_2nd.cpp`](examples/path_from_evaluator_2nd.cpp);
- [`examples/path_from_evaluator_3rd.cpp`](examples/path_from_evaluator_3rd.cpp).

Robot and dynamics:

- [`examples/robot_inverse_dynamics.cpp`](examples/robot_inverse_dynamics.cpp).

Solvers:

- [`examples/topp2_ra.cpp`](examples/topp2_ra.cpp);
- [`examples/reach_set2.cpp`](examples/reach_set2.cpp);
- [`examples/copp2_socp.cpp`](examples/copp2_socp.cpp);
- [`examples/topp3_lp.cpp`](examples/topp3_lp.cpp);
- [`examples/topp3_socp.cpp`](examples/topp3_socp.cpp);
- [`examples/copp3_socp.cpp`](examples/copp3_socp.cpp).

## Troubleshooting

### CMake Cannot Find The COPP Library

Run the matching Cargo build before configuring CMake:

```powershell
cargo build --release --lib --features cpp
```

If `COPP_INSTALL_C_ABI_TARGET=ON`, use:

```powershell
cargo build --release --lib --features cpp,c
```

Pass `-DCOPP_TARGET_DIR=<path-to-target-release>` if you are using a non-default Cargo target directory.

### Eigen Cannot Be Found

Either install Eigen3 so `find_package(Eigen3 CONFIG REQUIRED)` succeeds, or configure with:

```powershell
cmake -S bindings/cpp -B bindings/cpp/build -DCOPP_CPP_WITH_EIGEN=OFF
```

Do not include `copp/eigen.hpp` when `COPP_CPP_WITH_EIGEN=OFF`.

### Downstream `find_package` Fails

Make sure the downstream project sees the install prefix:

```powershell
cmake -S app -B app/build -DCMAKE_PREFIX_PATH=<install-prefix>
```

During this private-branch stage the package name is `copp`, so downstream code should call `find_package(copp CONFIG REQUIRED)`.

### Runtime DLL Is Missing On Windows

For dynamic linking, ensure `copp_cpp.dll` is next to your executable or visible through `PATH`. The source-tree examples/tests copy the DLL automatically, but your own downstream executable may need an install step or an explicit post-build copy.

### Static Linking Fails With Missing Native Symbols

Pass additional native libraries through:

```powershell
cmake -S bindings/cpp -B bindings/cpp/build -DCOPP_LINK_STATIC=ON -DCOPP_STATIC_LINK_LIBRARIES="lib1;lib2"
```

### Doxygen Formulas Or Graphs Do Not Render

Install Doxygen and Graphviz, then regenerate the docs from the repository root:

```powershell
powershell -ExecutionPolicy Bypass -File bindings/cpp/scripts/generate_docs.ps1
```

The C++ Doxygen config uses MathJax for formulas, so generated HTML may need browser access to the MathJax CDN unless a local MathJax path is configured.
