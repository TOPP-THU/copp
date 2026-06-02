# COPP MATLAB Bindings

[![License: MIT](https://img.shields.io/badge/license-MIT-yellow.svg)](../../LICENSE) [![Website](https://img.shields.io/badge/website-copp.pro-2ff0d8)](https://copp.pro/) [![Docs](https://img.shields.io/badge/docs-docs.copp.pro-1f6feb)](https://docs.copp.pro/) [![MATLAB](https://img.shields.io/badge/MATLAB-bindings-e16737)](#copp-matlab-bindings)

## Convex-Objective Path Parameterization

This directory contains the MATLAB package for COPP. The package is imported as `copp`. It wraps the Rust/C solver core through a MATLAB MEX gateway while presenting a MATLAB-friendly interface for paths, robot constraints, solver options, and post-processing helpers.

COPP solves optimal path-parameterization problems. A geometric path

$$
q = q(s)
$$

is converted into a time law

$$
s = s(t)
$$

so the executed trajectory `q(s(t))` satisfies velocity, acceleration, jerk, torque, or user-supplied constraints. Second-order solvers optimize

$$
a(s) = \dot{s}^2,
$$

and third-order solvers optimize the pair

$$
a(s) = \dot{s}^2,\qquad b(s) = \ddot{s}.
$$

The MATLAB API is a workflow facade, not a one-to-one mirror of the C ABI. It prioritizes the public `Path -> Robot -> Problem/Options -> solve -> interpolation` workflow and hides raw native pointers, owned C buffers, status-code plumbing, and most low-level debug/introspection calls.

For the full project overview, benchmark tables, citation information, and collaboration contact details, see the [repository README](../../README.md).

> **Open-source / PRO note:** this README documents the open-source MATLAB package, imported as `copp`. COPP PRO is a separate licensed extension with additional solvers; those extension-only APIs are intentionally not presented as part of this open-source MATLAB interface.

The MATLAB binding follows a small set of rules:

- add `bindings/matlab` to the MATLAB path;
- call public APIs through `copp.*`;
- use `double` inputs for predictable behavior and fewer conversions;
- pass path and robot sample matrices as `dim x N`, with one station/query sample per column;
- use 1-based public station indices in MATLAB;
- use solver-specific `Problem` and `Options` classes for readability;
- use `copp.interpolation` for profile-to-time conversion helpers.

## API Availability

| Problem class | MATLAB API |
| --- | --- |
| Core utilities | `copp.version`, `copp.diag.CoppError`, `copp.diag.last_error`, `copp.diag.Verbosity` |
| Path | `copp.Path`, waypoint paths, evaluator paths, symbolic/CasADi/Jet3 parametric paths, derivative evaluation |
| Robot | `copp.Robot`, station grids, sampled path derivatives, velocity/acceleration/jerk/torque limits, raw constraints, inverse-dynamics callbacks |
| TOPP2 | `copp.solver.topp2_ra.solve`, `copp.solver.reach_set2.backward`, `bidirectional` |
| COPP2 | `copp.solver.copp2_socp.solve`, `solve_expert` |
| TOPP3 | `copp.solver.topp3_lp.solve`, `copp.solver.topp3_socp.solve`, expert variants |
| COPP3 | `copp.solver.copp3_socp.solve`, `solve_expert` |
| Clarabel | `copp.clarabel.Options`, `Settings`, `DirectSolveMethod`, solver-local Clarabel aliases |
| Objectives | `copp.objective.time`, `linear`, `thermal_energy`, `total_variation_torque` |
| Interpolation | `s_to_t_topp2`, `t_to_s_topp2`, explicit uniform/sample variants, TOPP3 counterparts, `Profile3rd` |

Runnable examples are in [examples](examples/). Targeted tests are in [tests](tests/).

## Quick Start

### Installation

There are two expected installation paths for MATLAB users:

- download a prebuilt MATLAB package from the [COPP GitHub Releases page](https://github.com/TOPP-THU/copp/releases);
- build the MEX gateway from source after cloning <https://github.com/TOPP-THU/copp>.

A MATLAB Toolbox (`.mltbx`) installer is planned for future releases. Until that is packaged, source builds are the development path and release archives are the prebuilt distribution path.

### Prerequisites

For prebuilt release archives, you need MATLAB on the operating system and architecture named by the archive asset.

For a source build, you also need:

- MATLAB with the `arguments` block syntax and modern MEX support;
- a MATLAB-supported C++ compiler configured with `mex -setup C++`;
- Rust and Cargo;
- a native compiler toolchain compatible with Rust and MATLAB MEX.

Optional path constructors require optional MATLAB packages:

- Symbolic Math Toolbox for `Path.from_symbolic`;
- CasADi for `Path.from_casadi`.

### Install From GitHub Releases

Download the MATLAB release archive for your platform from:

```text
https://github.com/TOPP-THU/copp/releases
```

Then add the extracted MATLAB binding directory to the MATLAB path and verify the package:

```matlab
addpath("<path-to-copp-matlab>")
rehash
copp.version()
```

When `.mltbx` packages are available, installation will use MATLAB's toolbox installer:

```matlab
matlab.addons.toolbox.installToolbox("copp-matlab-v0.1.0-windows-x86_64.mltbx", true)
copp.version()
```

Do not unpack `.mltbx` files manually. They are MATLAB toolbox installers, and installation manages the MATLAB path automatically.

### Build From Source

Use this path when you are developing COPP, using a platform without a prebuilt archive, or testing local changes.

Run these commands in a system terminal, not inside MATLAB. They are the same on Windows PowerShell, Windows Command Prompt, macOS Terminal, and Linux shells:

```sh
git clone https://github.com/TOPP-THU/copp.git
cd copp
cargo build --release --lib --features matlab
```

The Cargo command builds the Rust native artifact for MATLAB. The `matlab` feature reuses the C ABI internally, without enabling MATLAB code in default pure-Rust builds.

Then run these commands in the MATLAB Command Window. Only the `cd` path style differs by operating system:

```matlab
% Windows path example:
cd("C:\path\to\copp\bindings\matlab")

% macOS/Linux path example:
cd("/path/to/copp/bindings/matlab")

mex -setup C++   % only needed the first time, or when changing compiler
build()
addpath(pwd)
rehash
copp.version()
```

`build()` uses static linking by default. Static linking produces a larger MEX file but avoids a separate runtime library search at MATLAB startup, so it is the recommended build for ordinary users. Dynamic linking keeps the MEX smaller and lets developers swap the native library:

```matlab
build(Linkage="dynamic")
```

See [Dynamic MEX Cannot Find The Native Runtime](#dynamic-mex-cannot-find-the-native-runtime) for runtime lookup details.

### Run Examples

From a source checkout, run examples from the repository root after installing a release archive or building the MEX from source:

```matlab
cd("<repo-root>")
addpath("bindings/matlab")
run("bindings/matlab/examples/topp2_ra.m")
run("bindings/matlab/examples/copp2_socp.m")
run("bindings/matlab/examples/topp3_lp.m")
run("bindings/matlab/examples/topp3_socp.m")
run("bindings/matlab/examples/copp3_socp.m")
```

### Run Tests

Run targeted MATLAB binding tests:

```matlab
cd bindings/matlab
run_tests()
```

To run one test file:

```matlab
cd bindings/matlab
run_tests("tests/test_topp2_ra.m")
```

Some optional tests are skipped when optional dependencies such as CasADi are not installed.

## General Workflow

Most MATLAB scripts follow the same shape:

1. Build a path from waypoints, an evaluator callback, or a parametric formula.
2. Build a strictly increasing station grid `s`.
3. Create a `copp.Robot`.
4. Append stations and sample path derivatives into the robot.
5. Add velocity, acceleration, jerk, torque, or raw constraints.
6. Build a solver `Problem` descriptor.
7. Build solver `Options` when defaults are not enough.
8. Call a solver.
9. Convert the returned path-domain profile into `t(s)` or `s(t)` samples.
10. Evaluate the original path at sampled `s(t)` values for downstream control or plotting.

For second-order problems, solver output is usually an `N x 1` profile `a`. For third-order problems, solver output is a `copp.Profile3rd` object containing `a`, `b`, and stationary-boundary metadata.

## Minimal Program

```matlab
addpath("bindings/matlab")
fprintf("COPP version: %s\n", copp.version());
```

## TOPP2-RA Example

This complete example builds a three-axis parametric path, samples second-order path derivatives into a robot, adds symmetric velocity and acceleration limits, solves TOPP2-RA, and converts the result into uniform time samples.

```matlab
addpath("bindings/matlab")

dim = 3;
n = 1001;
dt = 1.0e-3;
s = linspace(0.0, 1.0, n).';

% 1) Define q(s). Path.from_parametric uses copp.Jet3 internally and differentiates this scalar formula up to third order.
path = copp.Path.from_parametric( ...
    @(x) [ ...
        sin(2*pi*x); ...
        sin(3*pi*x + 0.3); ...
        sin(5*pi*x + 0.7)], ...
    s_range=[0, 1], ...
    dim=dim);
path_cleanup = onCleanup(@() path.release());

% 2) Build robot constraints, then apply symmetric velocity and acceleration limits in [-1, 1].
robot = copp.Robot(dim, Capacity=n);
robot_cleanup = onCleanup(@() robot.release());

robot.append_s(s);
robot.set_q_from_path_2nd(path);

vel_max = ones(dim, 1);
vel_min = -vel_max;
acc_max = ones(dim, 1);
acc_min = -acc_max;

robot.add_velocity_limits(vel_max, vel_min);
robot.add_acceleration_limits(acc_max, acc_min);

% 3) Solve TOPP2-RA with boundary values a(0) = 0 and a(1) = 0.
problem = copp.solver.topp2_ra.Problem( ...
    robot, ...
    idx_s_interval=[1, n], ...
    a_boundary=[0, 0]);
options = copp.solver.topp2_ra.Options();

a_profile = copp.solver.topp2_ra.solve(problem, options);

% 4) Post-process TOPP2-RA results: a(s) -> t(s) -> s(t).
[t_final, t_s] = copp.interpolation.s_to_t_topp2(s, a_profile);
s_t = copp.interpolation.t_to_s_topp2( ...
    s, a_profile, t_s, dt=dt, include_final=true);

% 5) Print the tutorial summary.
fprintf("TOPP2-RA done.\n");
fprintf("dim = %d, N = %d\n", dim, n);
fprintf("t_final = %.6f s\n", t_final);
fprintf("a_profile.len() = %d\n", numel(a_profile));
fprintf("s(t) samples = %d\n", numel(s_t));
```

The same structure extends to COPP2 by replacing the TOPP2 problem with `copp.solver.copp2_socp.Problem` and objective descriptors, and to third-order solvers by using `set_q_from_path_3rd`, jerk constraints, a third-order problem descriptor, and TOPP3 interpolation helpers.

## Solver Namespaces

### `copp.solver.topp2_ra` and `copp.solver.reach_set2`

TOPP2 is the second-order time-optimal family. It optimizes `a(s)` under first- and second-order constraints. Use `topp2_ra.solve` for the reachability-analysis solver and `reach_set2.backward` / `bidirectional` when you need reachable intervals directly.

### `copp.solver.copp2_socp`

COPP2 solves second-order convex-objective problems through Clarabel. Objectives are constructed through `copp.objective`, for example:

```matlab
objectives = { ...
    copp.objective.time(1.0), ...
    copp.objective.thermal_energy(0.1, ones(dim, 1))};
```

Use `solve` for an accepted profile and `solve_expert` when application code needs solver status, residuals, raw vectors, objective value, or per-objective terms.

### `copp.solver.topp3_lp` and `copp.solver.topp3_socp`

TOPP3 is the third-order time-optimal family. It uses the `(a,b)` state and supports jerk-aware constraints. A common pattern is to generate an initial `a` profile with TOPP2-RA, substitute it into third-order constraints when needed, and then solve with LP or SOCP.

The shared descriptor `copp.solver.topp3.Problem` is accepted by `topp3_lp` and `topp3_socp`. Solver-specific `Problem` classes are readability facades, not strict namespace gates.

### `copp.solver.copp3_socp`

COPP3 combines third-order constraints with convex objectives and solves the conic formulation through Clarabel. Simple solvers return `Profile3rd`; expert variants expose Clarabel diagnostics.

## Data Conventions

MATLAB uses a consistent shape convention across the binding:

- `dim` is the robot/path dimension;
- `N` is the number of station or query samples;
- path and robot matrices `q`, `dq`, `ddq`, and `dddq` are `dim x N`;
- evaluator callbacks receive `s` as a `1 x N` row vector and return `dim x N` matrices;
- profile outputs such as `a`, `b`, `t_s`, and `s_t` are `N x 1` double columns;
- per-axis limit vectors have length `dim`;
- per-axis station-varying limit matrices are `dim x N`;
- raw constraint matrices are `R x N`;
- public MATLAB station indices are 1-based.

Boundary values are path-domain variables:

- `a_boundary=[a_start, a_final]` fixes `a = (ds/dt)^2`;
- `b_boundary=[b_start, b_final]` fixes `b = d2s/dt2` for third-order problems.

## Error Handling

Most native failures are raised as MATLAB `MException` objects with identifiers such as `copp:InvalidArgument`, `copp:ConstraintError`, or `copp:SolverError`.

```matlab
try
    a_profile = copp.solver.topp2_ra.solve(problem, options);
catch ME
    fprintf("COPP failed: %s\n", ME.message);
end
```

`copp.diag.CoppError` is a MATLAB facade for native status/message/detail snapshots. `copp.diag.last_error()` reads the latest native diagnostic detail stored by the MEX gateway.

For Clarabel-based solvers, simple solver functions return an accepted profile or raise. Expert variants expose solver status, residuals, raw vectors, and objective values for applications that need status-aware behavior.

## Documentation

Package-level help is available in MATLAB:

```matlab
help copp
help copp.Path
help copp.Robot
```

Generated MATLAB HTML docs are built from:

```text
bindings/matlab/docs/source/
```

Generate local HTML docs from MATLAB:

```matlab
cd bindings/matlab/docs
build_docs()
```

Generated HTML is written to:

```text
bindings/matlab/docs/html/
```

The generated entry point is `bindings/matlab/docs/html/index.html`. The build also writes `helptoc.xml`, and `bindings/matlab/info.xml` points MATLAB's Help Browser at the generated documentation when this binding directory is on the MATLAB path. Public API reference pages are generated under `bindings/matlab/docs/html/ref/` from the existing MATLAB help comments. `build_docs()` replaces MATLAB Publish's default PNG equation output with MathJax-rendered TeX by default. Use `build_docs(MathRenderer="none")` only if you need the original image-based equations. To create a local Help Browser search database, run:

```matlab
build_docs(BuildSearchDatabase=true)
```

## Package Layout

```text
bindings/matlab/
  README.md
  build.m                  # build the MEX gateway; static is the default
  install_copp.m           # download and install a prebuilt MATLAB package
  run_tests.m              # run targeted MATLAB tests
  +copp/
    Path.m
    Robot.m
    Profile3rd.m
    diag/
    clarabel/
    objective/
    interpolation/
    solver/
    internal/              # private evaluator helpers and generated MEX/runtime files
  docs/
    source/                # authored MATLAB publish pages
    html/                  # generated HTML output
  examples/                # runnable MATLAB examples
  tests/                   # matlab.unittest smoke tests
  src/
    copp_mex.cpp           # single-command-dispatch MEX gateway
```

## Troubleshooting

### `copp.version` Cannot Be Found

Add the MATLAB binding directory to the path:

```matlab
addpath("bindings/matlab")
rehash
copp.version()
```

### MEX Build Cannot Find `copp.dll.lib` or `libcopp.a`

Build the Rust C ABI artifact first:

```powershell
cargo build --release --lib --features matlab
```

Then run the MATLAB build again:

```matlab
cd bindings/matlab
build()
```

### Dynamic MEX Cannot Find The Native Runtime

Prefer the default static build for ordinary use:

```matlab
build()
```

For dynamic builds, rerun:

```matlab
build(Linkage="dynamic")
```

The build copies `copp.dll`, `libcopp.so`, or `libcopp.dylib` next to the MEX. Linux builds use `$ORIGIN` and macOS builds use `@loader_path`, so the copied runtime is the first place MATLAB should look.

### Matrix Shape Errors

Check the matrix orientation before calling into COPP. Robot/path sample matrices are `dim x N`, not `N x dim`.

```matlab
assert(size(q, 1) == robot.dim)
assert(size(q, 2) == robot.len)
```

Use column vectors for profile-like data when possible:

```matlab
s = s(:);
a = a(:);
```
