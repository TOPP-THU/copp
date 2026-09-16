# COPP MATLAB Bindings

[![License: MIT](https://img.shields.io/badge/license-MIT-yellow.svg)](https://github.com/TOPP-THU/copp/blob/main/LICENSE) [![Website](https://img.shields.io/badge/website-copp.pro-2ff0d8)](https://copp.pro/) [![Docs](https://img.shields.io/badge/docs-docs.copp.pro-1f6feb)](https://docs.copp.pro/) [![Crates.io](https://img.shields.io/crates/v/copp.svg?color=b7410e)](https://crates.io/crates/copp) [![PyPI](https://img.shields.io/pypi/v/copp-py.svg?color=3A6DA8)](https://pypi.org/project/copp-py/)
[![Rust](https://img.shields.io/badge/Rust-native-b7410e)](https://docs.rs/copp/latest/copp/) [![C](https://img.shields.io/badge/C-ABI-a8b9cc)](https://github.com/TOPP-THU/copp/blob/main/bindings/c/README.md) [![Python](https://img.shields.io/badge/Python-bindings-ffd43b)](https://github.com/TOPP-THU/copp/blob/main/bindings/python/README.md) [![C++](https://img.shields.io/badge/C%2B%2B-bindings-00599c)](https://github.com/TOPP-THU/copp/blob/main/bindings/cpp/README.md) [![MATLAB](https://img.shields.io/badge/MATLAB-toolbox-e16737)](https://www.mathworks.com/matlabcentral/fileexchange/184715-copp-convex-objective-path-parameterization)

## <font color="#C00000">C</font>onvex-<font color="#C00000">O</font>bjective <font color="#C00000">P</font>ath <font color="#C00000">P</font>arameterization

This library targets **Optimal Path Parameterization (OPP)** for robotic trajectory generation. Typical application domains include robotic motion planning and CNC machining.


Given an $n$-dimensional geometric path parameterization

$$
\boldsymbol{q} = \boldsymbol{q}(s),\text{ }s\in[0,s_\text{f}],\text{ (}s_\text{f}\text{ is known)}
$$

the objective is to schedule a dynamically feasible time parameterization

$$
s = s(t),\text{ }t\in[0,t_\text{f}],\text{ (}t_\text{f}\text{ is unknown)}
$$

so that the specified system constraints  are satisfied while an objective $J$ is optimized. In this way, the problem is transformed from geometry-space description to time-space scheduling along a fixed path.

At a high level, this project unifies two system orders and two objective families:

- **2nd-order models**: constraints on velocity, acceleration, torque, etc. The constraint can be written as $\boldsymbol{f}(\boldsymbol{q}(s),\dot{\boldsymbol{q}}(s),\ddot{\boldsymbol{q}}(s);s)\leq\boldsymbol{0}$, and the objective is $\min J=\int_0^{t_\text{f}}L(\boldsymbol{q}(s),\dot{\boldsymbol{q}}(s),\ddot{\boldsymbol{q}}(s);s)\mathrm{d}t$.
- **3rd-order models**: additionally include jerk-related effects. The constraint can be written as $\boldsymbol{f}(\boldsymbol{q}(s),\dot{\boldsymbol{q}}(s),\ddot{\boldsymbol{q}}(s),\dddot{\boldsymbol{q}}(s);s)\leq\boldsymbol{0}$, and the objective is $\min J=\int_0^{t_\text{f}}L(\boldsymbol{q}(s),\dot{\boldsymbol{q}}(s),\ddot{\boldsymbol{q}}(s),\dddot{\boldsymbol{q}}(s);s)\mathrm{d}t$.
- **TOPP** (Time-Optimal Path Parameterization): minimizes traversal time, i.e., $L\equiv1$ and the objective is $J=t_\text{f}$.
- **COPP** (Convex-Objective Path Parameterization): supports broader convex objectives. The objective $L$ should be convex with respect to the state and control: $(\dot{s}^2,\ddot{s})$ in 2nd-order models and $(\dot{s}^2,\ddot{s},\frac{\dddot{s}}{\dot{s}})$ in 3rd-order models.

The resulting taxonomy is summarized below.

| Smoothness order                                     | Time-optimal objective | General convex objective |
| ---------------------------------------------------- | ---------------------- | ------------------------ |
| 2nd-order (velocity/acceleration/torque constraints) | TOPP2                  | COPP2                    |
| 3rd-order (+ jerk constraints)                       | TOPP3                  | COPP3                    |

This document covers the MATLAB toolbox for COPP. The package is imported as `copp`. It wraps the Rust/C solver core through a MATLAB MEX gateway while presenting a MATLAB-friendly interface for paths, robot constraints, solver options, and post-processing helpers.

The MATLAB API is a workflow facade, not a one-to-one mirror of the C ABI. It prioritizes the public `Path -> Robot -> Problem/Options -> solve -> interpolation` workflow and hides raw native pointers, owned C buffers, status-code plumbing, and most low-level debug/introspection calls.

The MATLAB binding follows a small set of rules:

- make the directory that contains `+copp` visible to MATLAB (the prebuilt toolbox does this for you; from a clone, `addpath` that directory);
- call public APIs through `copp.*`;
- use `double` inputs for predictable behavior and fewer conversions;
- pass path and robot sample matrices as `dim x N`, with one station/query sample per column;
- use 1-based public station indices in MATLAB;
- use solver-specific `Problem` and `Options` classes for readability;
- use `copp.interpolation` for profile-to-time conversion helpers.

## Algorithm availability

This section focuses on open-source algorithms. If you need the best possible performance for difficult large-scale problems, please see [PRO](#pro).

| Problem class | Algorithm  | Notes                                                                                                                                                                        |
| ------------- | ---------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| TOPP2         | TOPP2-RA   | Ultra-fast reachability-analysis-based method; near-global-optimal in common benchmarks, with relative error typically below $10^{-4}$ versus global optimization baselines. |
| COPP2         | COPP2-SOCP | Solved as an SOCP using `clarabel`; globally optimal under the convex formulation, with moderate-to-high runtime cost.                                                       |
| TOPP3         | TOPP3-SOCP | `clarabel`-based conic formulation; returns KKT solutions with strong optimality quality, and may incur higher computational cost on specific datasets.                      |
| TOPP3         | TOPP3-LP   | Linear-objective approximation of TOPP3-SOCP; usually faster, but can become sub-optimal under tight jerk constraints (recommended mainly when jerk bounds are loose).       |
| COPP3         | COPP3-SOCP | `clarabel`-based conic formulation; returns KKT solutions with strong practical optimality, at relatively high computational cost.                                           |

### Algorithm Selection Guide

| Scenario / primary priority                               | Recommended algorithm                                               | Why this is recommended                                                             | Typical caveat                                         | Alternative                                             |
| --------------------------------------------------------- | ------------------------------------------------------------------- | ----------------------------------------------------------------------------------- | ------------------------------------------------------ | ------------------------------------------------------- |
| 2nd-order, time-optimal planning with very low runtime    | TOPP2-RA                                                            | Excellent speed-performance trade-off; near-global-optimal in benchmarks            | Objective is fixed to minimum-time style               |                                                         |
| 2nd-order, convex objective with strong global guarantees | COPP2-SOCP                                                          | Convex conic formulation with global optimality under model assumptions             | Higher runtime than RA-style methods                   | See [PRO](#pro) for higher-performance solvers          |
| 3rd-order, best open-source optimality quality            | TOPP3-SOCP (time objective) / COPP3-SOCP (general convex objective) | Strong KKT-quality solutions and broad applicability                                | On specific datasets, computational cost may be higher | TOPP3-LP or higher-performance solvers in [PRO](#pro)   |
| 3rd-order, faster open-source approximation               | TOPP3-LP                                                            | Use this only when your own path-dataset benchmark shows better runtime/performance | May become sub-optimal under tight jerk bounds         | TOPP3-SOCP or higher-performance solvers in [PRO](#pro) |

### Benchmark

The tests are provided in [test_random_spline.rs](https://github.com/TOPP-THU/copp/blob/main/tests/test_random_spline.rs). Condition: 

+ `release, --include-ignored`
+ CPU: Intel(R) Core(TM) Ultra 9 285K.
+ Dataset: 100 random 7-DOF spline paths, each discretized into 1000 intervals.

All metrics are listed in the form of "mean ± std".

#### Time-Optimal

| Method     |  Computation time (ms) |   Traversal time (s) |
| ---------- | ---------------------: | -------------------: |
| TOPP2-RA   |    0.665447 ± 0.278833 | 40.903420 ± 1.378671 |
| COPP2-SOCP | 161.544955 ± 13.342861 | 40.900036 ± 1.378611 |
| TOPP3-LP   | 346.354708 ± 38.865584 | 41.422937 ± 1.381852 |
| TOPP3-SOCP | 312.118867 ± 20.544208 | 41.418608 ± 1.381202 |
| COPP3-SOCP | 305.931003 ± 22.910681 | 41.418608 ± 1.381202 |

#### Convex-Objective

In this test, TOPP methods still use traversal time as the optimization objective.

| Method     |  Computation time (ms) |       Objective value |
| ---------- | ---------------------: | --------------------: |
| TOPP2-RA   |    0.696362 ± 0.300599 | 223.896965 ± 9.485003 |
| COPP2-SOCP | 300.428479 ± 71.154448 |  97.746537 ± 2.652869 |
| TOPP3-LP   | 384.399012 ± 79.626532 | 217.858895 ± 9.324830 |
| TOPP3-SOCP | 340.468745 ± 59.209470 | 218.026329 ± 9.285519 |
| COPP3-SOCP | 376.119382 ± 88.865232 |  97.871570 ± 2.645819 |

## Why use COPP instead of re-implementing from scratch?

Unless you are a specialist researcher in TOPP/COPP, we strongly recommend using this library directly. In practical deployment, many critical implementation details are easy to overlook, for example:

+ strict constraint satisfaction rather than soft-constraint relaxation;
+ guaranteed geometric consistency: the executed trajectory $\boldsymbol{q}=\boldsymbol{q}(s(t))$ remains exactly on the original geometric path $\boldsymbol{q}=\boldsymbol{q}(s)$, avoiding additional contour error introduced by the COPP stage;
+ prevention of reverse motion and zero-velocity singularities (i.e., enforcing $\dot{s}>0$ strictly almost everywhere);
+ [certifiable feasibility guarantees, especially for long-path online planning](https://doi.org/10.1016/j.ijmachtools.2025.104355), a challenge only recently addressed in the literature;
+ [boundary acceleration continuity handling](https://doi.org/10.1016/j.ijmachtools.2025.104355), which is frequently problematic in existing methods. Some algorithms bypass this issue by ignoring boundary-acceleration constraints or by disallowing stationary boundary conditions $\dot{s}=0,\ddot{s}=0$, instead requiring $\dot{s}>\delta$.

## Citing

If your work uses the open-source TOPP3/COPP3 functionalities, please cite:

```tex
@article{wang2026online,
  title={Online time-optimal trajectory planning along parametric toolpaths with strict constraint satisfaction and certifiable feasibility guarantee},
  author={Wang, Yunan and Hu, Chuxiong and Li, Yuanshenglong and Yu, Jichuan and Yan, Jizhou and Liang, Yixuan and Jin, Zhao},
  journal={International Journal of Machine Tools and Manufacture},
  volume={215},
  pages={104355},
  year={2026}
}
```

If your work uses RDDP methods from the PRO release, please cite:

```tex
@article{wang2026reachability,
  title={Reachability-augmented dual dynamic programming for optimal path parameterization},
  author={Yunan Wang and Jizhou Yan and Chuxiong Hu and Zeyang Li},
  journal={arXiv preprint arXiv:2605.19089},
  year={2026}
}
```

For other use cases, please cite:

```tex
@inproceedings{wang2026copp,
  title={{COPP}: An Open-Source Ultra-Fast Library for Convex-Objective Path Parameterization},
  author={Wang, Yunan and He, Suqin and Lin, Shize and Hu, Chuxiong},
  booktitle={American Control Conference},
  pages={2335},
  year={2026},
  organization={IEEE}
}
```

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

Runnable examples are in [examples](examples/) and ship with the toolbox. Targeted tests are in [tests](https://github.com/TOPP-THU/copp/tree/main/bindings/matlab/tests) in the source repository; the prebuilt toolbox does not include them.

## Quick Start

### General workflow

+ **Inputs**
    + path grid: prepare a strictly increasing station grid $0=s_0<s_1<\dots<s_{n-1}=s_\text{f}$;
    + path data: provide path derivatives through `copp.Path` evaluators or sampled matrices. 2nd-order problems use $\boldsymbol{q}$, $\frac{\mathrm{d}\boldsymbol{q}}{\mathrm{d}s}$, and $\frac{\mathrm{d}^2\boldsymbol{q}}{\mathrm{d}s^2}$; 3rd-order problems additionally use $\frac{\mathrm{d}^3\boldsymbol{q}}{\mathrm{d}s^3}$;
    + robot and constraints: create a `copp.Robot`, attach the station grid with `append_s`, then fill path data with `set_q_from_path_2nd` or `set_q_from_path_3rd`. Standard constraint APIs include `add_velocity_limits`, `add_acceleration_limits`, `add_torque_limits`, and `add_jerk_limits` for 3rd-order problems. Asymmetric and station-dependent limits are supported;
    + objective: TOPP solvers use traversal time as the objective. COPP solvers take a list of built-in objective terms, such as `copp.objective.time`, `copp.objective.thermal_energy`, `copp.objective.total_variation_torque`, and `copp.objective.linear`.
+ **Problem construction**
    + 2nd-order: build a `copp.solver.topp2_ra.Problem` or `copp.solver.copp2_socp.Problem`;
    + 3rd-order: first prepare a feasible linearization profile `a_linearization`, commonly from `copp.solver.topp2_ra.solve`. Optionally tighten the first-order upper bound with `robot.add_raw_constraint_1st(...)`. Then pass the profile to `copp.solver.topp3_lp.Problem`, `copp.solver.topp3_socp.Problem`, or `copp.solver.copp3_socp.Problem`; the shared descriptor `copp.solver.topp3.Problem` is accepted by both TOPP3 solvers.
+ **Solving**
    + choose the solver namespace according to the problem class and algorithm, for example `copp.solver.topp2_ra`, `copp.solver.copp2_socp`, `copp.solver.topp3_lp`, `copp.solver.topp3_socp`, or `copp.solver.copp3_socp`;
    + build solver options with the corresponding `Options` class, then call the namespace's `solve` function.
+ **Outputs and post-processing**
    + 2nd-order solvers return an `a` profile, where $a(s)=\dot{s}^2$;
    + 3rd-order solvers return a `copp.Profile3rd`, containing `a`, `b`, and stationary-boundary metadata;
    + convert path-domain profiles to timing data with `copp.interpolation.s_to_t_topp2` or `s_to_t_topp3`. For a `copp.Profile3rd`, pass `profile.a` and `profile.b`;
    + convert timing data to sampled inverse parameterization `s(t)` with `copp.interpolation.t_to_s_topp2` or `t_to_s_topp3`;
    + evaluate the original path at `s(t)` to generate position, velocity, and acceleration references for downstream controllers.

For second-order problems, solver output is usually an `N x 1` profile `a`. For third-order problems, solver output is a `copp.Profile3rd` object containing `a`, `b`, and stationary-boundary metadata.

### Installation

COPP for MATLAB reaches you in one of three forms. The rest of this document uses these three names, and any instruction that is not valid for all three says which form it belongs to.

- **Installed toolbox** — the prebuilt `copp-matlab.mltbx`, installed from [File Exchange](https://www.mathworks.com/matlabcentral/fileexchange/184715-copp-convex-objective-path-parameterization), from the [COPP GitHub Releases page](https://github.com/TOPP-THU/copp/releases), or by `install_copp`. Installation manages the MATLAB path for you, so never `addpath` anything for it. It contains `+copp/` and `examples/`; it does not contain the tests, the documentation sources, `build.m`, or `install_copp.m`.
- **Source clone** — a clone of <https://github.com/TOPP-THU/copp>, where the MATLAB binding is the `bindings/matlab` directory. This is the only form that can build the MEX gateway.
- **File Exchange mirror** — a clone of <https://github.com/TOPP-THU/copp-matlab>, a read-only republication of `bindings/matlab` at that repository's root, so there is no `bindings/` level. It deliberately ships **no MEX gateway, no gateway sources, and no build script**, so nothing in it can run on its own: adding the clone to the MATLAB path only yields `Unrecognized function or variable 'copp.internal.copp_mex'`. Read the sources there, and install the prebuilt toolbox to run anything — that repository's Releases page carries the same `.mltbx`.

Throughout this document, **the binding directory** means the directory that contains `+copp`: the installed toolbox root, `bindings/matlab` in a source clone, or the repository root in the mirror. Once MATLAB can find COPP, that directory is `fileparts(fileparts(which("copp.version")))`.

One toolbox covers every supported platform. The `.mltbx` ships the MEX gateway for Windows, Linux, and both macOS architectures, and MATLAB loads the one matching the machine it runs on, so there is no per-platform download to choose between.

### Prerequisites

For the prebuilt toolbox, you need MATLAB R2024b or newer on one of the supported platforms: Windows x86-64, Linux x86-64, macOS arm64, or macOS x86-64.

For a source build, you also need:

- MATLAB with the `arguments` block syntax and modern MEX support;
- a MATLAB-supported C++ compiler configured with `mex -setup C++`;
- Rust and Cargo;
- a native compiler toolchain compatible with Rust and MATLAB MEX.

Optional path constructors require optional MATLAB packages:

- Symbolic Math Toolbox for `Path.from_symbolic`;
- CasADi for `Path.from_casadi`.

### Install the Prebuilt Toolbox

The shortest route is MATLAB's own Add-On Explorer: open the [File Exchange page](https://www.mathworks.com/matlabcentral/fileexchange/184715-copp-convex-objective-path-parameterization) and install from there.

To install from a GitHub Release instead, download `copp-matlab.mltbx` from:

```text
https://github.com/TOPP-THU/copp/releases
```

Then install it and verify the package. Run this from the folder your browser saved the file in, or pass the full path to the `.mltbx`:

```matlab
matlab.addons.toolbox.installToolbox("copp-matlab.mltbx", true)
copp.version()
```

`install_copp.m` automates the same steps and resolves the newest release on its own. It is not part of the installed toolbox: download the single file <https://raw.githubusercontent.com/TOPP-THU/copp/main/bindings/matlab/install_copp.m> (in the File Exchange mirror it sits at the repository root), then `cd` in MATLAB to the folder you saved it in — a folder that does not itself contain a `+copp` — and run:

```matlab
install_copp()
```

Every release publishes the toolbox twice: `copp-matlab.mltbx` always points at the newest release, and `copp-matlab-<tag>.mltbx` is the immutable copy for that tag. Pass `install_copp(Version="v0.2.3")` to pin one.

Do not unpack `.mltbx` files manually. They are MATLAB toolbox installers, and installation manages the MATLAB path automatically.

### Build From Source

Use this path when you are developing COPP, using a platform the prebuilt toolbox does not cover, or testing local changes. It applies to a source clone only: the installed toolbox already ships a statically linked gateway, and the File Exchange mirror contains neither `build.m` nor the gateway sources.

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
cd("C:\path\to\your\clone\of\copp\bindings\matlab")

% macOS/Linux path example:
cd("/path/to/your/clone/of/copp/bindings/matlab")

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

See [Dynamic MEX Cannot Find The Native Runtime](#dynamic-mex-cannot-find-the-native-runtime-source-clone-only) for runtime lookup details.

### Run Examples

The examples ship with the installed toolbox and with both repositories. Run them out of the binding directory — each example calls `ExampleCommon.setup_path()` itself, so you do not need an `addpath` of your own:

```matlab
coppRoot = fileparts(fileparts(which("copp.version")));   % the binding directory
run(fullfile(coppRoot, "examples", "topp2_ra.m"))
run(fullfile(coppRoot, "examples", "copp2_socp.m"))
run(fullfile(coppRoot, "examples", "topp3_lp.m"))
run(fullfile(coppRoot, "examples", "topp3_socp.m"))
run(fullfile(coppRoot, "examples", "copp3_socp.m"))
run(fullfile(coppRoot, "examples", "reach_set2.m"))
run(fullfile(coppRoot, "examples", "version_demo.m"))
```

Do not run a clone's examples while the prebuilt toolbox is installed unless that clone has a built MEX gateway. `ExampleCommon.setup_path()` adds the example folder's parent, so a gateway-less `+copp` — a File Exchange mirror clone, or a source clone before `build()` — would shadow the installed package and the example would fail on `copp.internal.copp_mex`.

### Run Tests

The test suite ships with both repositories but not with the installed toolbox, and it needs a working MEX gateway, which the File Exchange mirror does not have — in practice this means a source clone on which `build()` has already run. From the binding directory (`bindings/matlab` in a source clone), run:

```matlab
run_tests()
```

To run one test file:

```matlab
run_tests(fullfile("tests", "test_topp2_ra.m"))
```

Some optional tests are skipped when optional dependencies such as CasADi are not installed.

### Documentation

The latest documentation is available at <https://docs.copp.pro/>. For unreleased updates on the main branch, we recommend generating the documentation locally.

Package-level help is available in MATLAB:

```matlab
help copp
help copp.Path
help copp.Robot
```

Generated MATLAB HTML docs are built from `docs/source/` inside the binding directory (`bindings/matlab/docs/source/` in a source clone). These sources ship with both repositories, but not with the installed toolbox — for the installed toolbox use <https://docs.copp.pro/>.

```text
docs/source/
```

Generate local HTML docs from MATLAB, from the `docs` folder of the binding directory:

```matlab
cd docs            % bindings/matlab/docs in a source clone
build_docs()
```

Publishing executes the pages, so `build_docs()` requires a working MEX gateway. In a tree that has none — a File Exchange mirror clone — use `build_docs(EvalCode=false)` to render the pages without executing them.

Generated HTML is written to:

```text
docs/html/
```

The generated entry point is `docs/html/index.html` inside the binding directory (`bindings/matlab/docs/html/index.html` in a source clone). The build also writes `helptoc.xml`, and the `info.xml` in the binding directory points MATLAB's Help Browser at it once that directory is on the MATLAB path — but only after `build_docs()` has created `docs/html`, which is not checked in and is not shipped by either repository. The installed toolbox contains neither `info.xml` nor the documentation sources; use <https://docs.copp.pro/>. Public API reference pages are generated under `docs/html/ref/` from the existing MATLAB help comments. `build_docs()` replaces MATLAB Publish's default PNG equation output with MathJax-rendered TeX by default. Use `build_docs(MathRenderer="none")` only if you need the original image-based equations. To create a local Help Browser search database, run:

```matlab
build_docs(BuildSearchDatabase=true)
```

### MATLAB

With the prebuilt toolbox installed, the MATLAB API is available immediately — do not add anything to the path. From a clone with a built MEX gateway, `cd` to the binding directory and `addpath(pwd)` once. Either way:

```matlab
fprintf("COPP version: %s\n", copp.version());
```

Complete runnable examples are available in [the examples directory](examples/). A [quick example (2nd-order)](examples/topp2_ra.m) is as follows. It builds a three-axis parametric path, samples second-order path derivatives into a robot, adds symmetric velocity and acceleration limits, solves TOPP2-RA, and converts the result into uniform time samples.

```matlab
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

### API for Other languages

- Rust: add the `copp` crate dependency with `copp = "*"`, then refer to [the repository README](https://github.com/TOPP-THU/copp/blob/main/README.md) and <https://docs.rs/copp/latest/copp/>.
- C: enable the Cargo `c` feature with `cargo build --release --lib --features c`, then refer to [README.md for C](https://github.com/TOPP-THU/copp/blob/main/bindings/c/README.md).
- Python: Please refer to [README.md for Python](https://github.com/TOPP-THU/copp/blob/main/bindings/python/README.md).
- C++: Please refer to [README.md for C++](https://github.com/TOPP-THU/copp/blob/main/bindings/cpp/README.md).

If you have suggestions for these language interfaces, please feel free to [contact us](#contact-us), open an issue, or submit a pull request.

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

## Package Layout

```text
Source clone (github.com/TOPP-THU/copp), under bindings/matlab/:

  README.md                # this document
  build.m                  # build the MEX gateway; static is the default
  install_copp.m           # download and install a prebuilt MATLAB package
  run_tests.m              # run targeted MATLAB tests
  info.xml                 # Help Browser registration; needs docs/html to exist
  +copp/
    Path.m
    Robot.m
    Profile3rd.m
    +diag/
    +clarabel/
    +objective/
    +interpolation/
    +solver/
    +internal/             # private evaluator helpers; also holds the built MEX
                           # gateway and, for dynamic builds, the native runtime
  docs/
    build_docs.m
    source/                # authored MATLAB publish pages
    html/                  # generated by build_docs(); not checked in
  examples/                # runnable MATLAB examples
  tests/                   # matlab.unittest smoke tests
  src/
    copp_mex.cpp           # single-command-dispatch MEX gateway
```

**File Exchange mirror** (github.com/TOPP-THU/copp-matlab): the same contents at the repository root, without `src/`, `build.m`, and without any MEX or native runtime file under `+copp/+internal/`. Nothing in that tree can be run or built as it stands.

**Installed toolbox**: `+copp/` (with the gateway for all four platforms in `+copp/+internal/`), `examples/`, `MATLAB-README.md` (this document, renamed), `CHANGELOG.md`, and `LICENSE`. The tests, the documentation sources, `info.xml`, `build.m`, and `install_copp.m` are not part of it.

## Troubleshooting

### `copp.version` Cannot Be Found

First find out which copy MATLAB is resolving, and whether it has a MEX gateway:

```matlab
which copp.version               % empty means MATLAB cannot see any +copp
dir(fullfile(fileparts(fileparts(which("copp.version"))), "+copp", "+internal", "copp_mex.*"))
```

The two failures look different and have different remedies.

- **`Unrecognized function or variable 'copp.internal.copp_mex'`** — MATLAB found a `+copp` package but it has no MEX gateway. Adding more directories to the path will not help. A File Exchange mirror clone never has one: install the prebuilt toolbox, and `cd` out of the clone so its `+copp` stops shadowing the installed package. A source clone needs `build()` first.
- **`Unrecognized function or variable 'copp.version'`** — no `+copp` is visible at all. With the prebuilt toolbox, run `rehash toolboxcache` and check `matlab.addons.toolbox.installedToolboxes`; if COPP is not listed, reinstall the `.mltbx`. From a clone, `cd` to the binding directory and run `addpath(pwd); rehash`.

If `which copp.version` points into a clone while the toolbox is installed, the clone is shadowing the installation: `cd` elsewhere and remove that directory from the path with `rmpath`.

### MEX Build Cannot Find `copp.dll.lib` or `libcopp.a` (source clone only)

Build the Rust C ABI artifact first:

```powershell
cargo build --release --lib --features matlab
```

Then run the MATLAB build again, from `bindings/matlab` in your clone. The installed toolbox and the File Exchange mirror contain no `build.m` and no gateway sources, so there is nothing to build there:

```matlab
cd bindings/matlab
build()
```

### Dynamic MEX Cannot Find The Native Runtime (source clone only)

This applies only to a MEX you built yourself. The prebuilt toolbox ships a statically linked gateway and is unaffected. Prefer the default static build for ordinary use:

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

## PRO

> **Open-source / PRO note:** the MATLAB package documented above is the open-source package, imported as `copp`. COPP PRO is a separate licensed extension with additional solvers; those extension-only APIs are intentionally not presented as part of this open-source MATLAB interface.

### Open-source vs <font color="#C00000">**PRO**</font>

The open-source release and PRO release provide complementary solvers for the above problem classes. Performance evaluations for the open-source methods are documented in the corresponding Rust test/example source files; all methods are summarized below. For challenging trajectory-planning tasks that require both high solution quality and robust numerical behavior, we recommend the PRO solvers. If you are interested in COPP PRO licensing or collaboration, please see [Contact Us](#contact-us).

| Problem class | Algorithm  | Availability                         | Notes                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| ------------- | ---------- | ------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| TOPP2         | TOPP2-RA   | Open-source                          | Ultra-fast reachability-analysis-based method; near-global-optimal in common benchmarks, with relative error typically below $10^{-4}$ versus global optimization baselines.                                                                                                                                                                                                                                                                                                                                                                |
| COPP2         | COPP2-SOCP | Open-source                          | Solved as an SOCP using `clarabel`; globally optimal under the convex formulation, with moderate-to-high runtime cost.                                                                                                                                                                                                                                                                                                                                                                                                                      |
| COPP2         | COPP2-RDDP | <font color="#C00000">**PRO**</font> | Ultra-fast original method; globally optimal and substantially faster than COPP2-SOCP.                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| TOPP3         | TOPP3-SOCP | Open-source                          | `clarabel`-based conic formulation; returns KKT solutions with strong optimality quality, and may incur higher computational cost on specific datasets.                                                                                                                                                                                                                                                                                                                                                                                     |
| TOPP3         | TOPP3-LP   | Open-source                          | Linear-objective approximation of TOPP3-SOCP; usually faster, but can become sub-optimal under tight jerk constraints (recommended mainly when jerk bounds are loose).                                                                                                                                                                                                                                                                                                                                                                      |
| TOPP3         | TOPP3-RA   | <font color="#C00000">**PRO**</font> | Ultra-fast reachability-analysis-based method; may be sub-optimal under tight jerk constraints (recommended mainly when jerk bounds are loose).                                                                                                                                                                                                                                                                                                                                                                                             |
| COPP3         | COPP3-SOCP | Open-source                          | `clarabel`-based conic formulation; returns KKT solutions with strong practical optimality, at relatively high computational cost.                                                                                                                                                                                                                                                                                                                                                                                                          |
| COPP3         | COPP3-RDDP | <font color="#C00000">**PRO**</font> | Fast original method; returns KKT-quality solutions comparable to TOPP3-SOCP while running substantially faster than TOPP3-SOCP, TOPP3-LP, and COPP3-SOCP. COPP3-RDDP can also be used as a TOPP3 solver, with significantly better time-optimality than TOPP3-RA and TOPP3-LP in many cases. For very long paths, COPP3-RDDP may even exhibit better practical optimality and numerical stability than COPP3-SOCP, since large-scale conic optimization can become limited by convergence behavior and computational-resource constraints. |

### Algorithm Selection Guide

| Scenario / primary priority                                                | Recommended algorithm                                               | Availability                         | Why this is recommended                                                                           | Typical caveat                                         | Alternative                            |
| -------------------------------------------------------------------------- | ------------------------------------------------------------------- | ------------------------------------ | ------------------------------------------------------------------------------------------------- | ------------------------------------------------------ | -------------------------------------- |
| 2nd-order, time-optimal planning with very low runtime                     | TOPP2-RA                                                            | Open-source                          | Excellent speed-performance trade-off; near-global-optimal in typical benchmarks                  | Objective is fixed to minimum-time style               |                                        |
| 2nd-order, convex objective with strong global guarantees                  | COPP2-SOCP                                                          | Open-source                          | Convex conic formulation with global optimality under model assumptions                           | Higher runtime than RA/RDDP methods                    | COPP2-RDDP (PRO) for major speed gains |
| 2nd-order, convex objective with maximum efficiency                        | COPP2-RDDP                                                          | <font color="#C00000">**PRO**</font> | Global-optimal quality with substantially improved speed                                          | PRO license required                                   | COPP2-SOCP (Open-source)               |
| 3rd-order, best open-source optimality quality                             | TOPP3-SOCP (time objective) / COPP3-SOCP (general convex objective) | Open-source                          | Strong KKT-quality solutions and broad applicability                                              | On specific datasets, computational cost may be higher | COPP3-RDDP (PRO) for major speed gains |
| 3rd-order, faster open-source approximation                                | TOPP3-LP                                                            | Open-source                          | Use this only when your own path-dataset benchmark shows better runtime/performance               | May become sub-optimal under tight jerk bounds         | COPP3-RDDP (PRO) for major speed gains |
| 3rd-order, ultra-fast RA-style method under loose jerk bounds              | TOPP3-RA                                                            | <font color="#C00000">**PRO**</font> | Very low computational cost                                                                       | Can be sub-optimal when jerk constraints are tight     | COPP3-RDDP or TOPP3-SOCP               |
| 3rd-order, high-quality + high-stability planning for difficult long paths | COPP3-RDDP                                                          | <font color="#C00000">**PRO**</font> | Strong practical optimality with significantly better runtime; often robust on very long horizons | PRO license required                                   | COPP3-SOCP (Open-source)               |

### Benchmark-PRO

All settings are the same as those in [benchmark](#benchmark).

All metrics are listed in the form of "mean ± std".

#### Time-Optimal

| Method                                                  |    Computation time (ms) |       Traversal time (s) |
| ------------------------------------------------------- | -----------------------: | -----------------------: |
| TOPP2-RA                                                |      0.615425 ± 0.244409 |     40.903420 ± 1.378671 |
| COPP2-SOCP                                              |    149.969964 ± 9.364334 |     40.900039 ± 1.378613 |
| <font color="#C00000">**COPP2-RDDP**</font>             |  **5.436142** ± 0.465495 | **40.900135** ± 1.378613 |
| TOPP3-LP                                                |   327.074029 ± 28.893341 |     41.422945 ± 1.381874 |
| TOPP3-SOCP                                              |   289.654071 ± 12.862133 |     41.418608 ± 1.381202 |
| COPP3-SOCP                                              |   285.004302 ± 13.471264 |     41.418608 ± 1.381202 |
| <font color="#C00000">**TOPP3-RA**</font> (Iteration 1) | **10.571045** ± 0.857653 | **41.499200** ± 1.385735 |
| <font color="#C00000">**TOPP3-RA**</font> (Iteration 2) | **20.300932** ± 1.237908 | **41.399867** ± 1.386791 |

#### Convex-Objective

In this test, TOPP methods still use traversal time as the optimization objective.

| Method                                      |    Computation time (ms) |          Objective value |
| ------------------------------------------- | -----------------------: | -----------------------: |
| TOPP2-RA                                    |      0.534700 ± 0.069296 |   217.444861 ± 12.462360 |
| COPP2-SOCP                                  |   270.059250 ± 52.073677 |     96.517354 ± 3.641154 |
| <font color="#C00000">**COPP2-RDDP**</font> | **12.667700** ± 0.429214 | **96.525785** ± 3.639733 |
| TOPP3-LP                                    |    348.000000 ± 9.326314 |   211.611085 ± 12.367224 |
| TOPP3-SOCP                                  |   301.227000 ± 12.938498 |   211.974066 ± 12.323865 |
| COPP3-SOCP                                  |   301.227000 ± 12.938498 |     96.634962 ± 3.613264 |
| <font color="#C00000">**COPP3-RDDP**</font> | **65.823050** ± 0.087893 | **98.708998** ± 3.354004 |

## Contact Us

For COPP PRO licensing, commercial collaboration, technical consulting, or general inquiries, please contact us at [hello@copp.pro](mailto:hello@copp.pro).

Project maintainers:

+ [Mr. Yunan Wang](https://scholar.google.com/citations?user=RXaTo_kAAAAJ): [wang-yn22@mails.tsinghua.edu.cn](mailto:wang-yn22@mails.tsinghua.edu.cn)
+ [Dr. Suqin He](https://github.com/hsqthu2012): [hsq_thu2012@163.com](mailto:hsq_thu2012@163.com)
+ [Dr. Shize Lin](https://github.com/thume4zzzz): [linszthume@gmail.com](mailto:linszthume@gmail.com)
+ [Prof. Chuxiong Hu](https://www.me.tsinghua.edu.cn/en/info/1275/2062.htm): [cxhu@tsinghua.edu.cn](mailto:cxhu@tsinghua.edu.cn)

Furthermore, we would like to thank [Jizhou Yan](https://github.com/yixing312) for his expertise on Rust and robotics, and to thank [Daoming Chen](https://github.com/Daoming-Chen) for his contribution to Python bindings.
