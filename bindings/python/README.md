# COPP Python Bindings

[![License: MIT](https://img.shields.io/badge/license-MIT-yellow.svg)](../../LICENSE) [![Website](https://img.shields.io/badge/website-copp.pro-2ff0d8)](https://copp.pro/) [![Docs](https://img.shields.io/badge/docs-docs.copp.pro-1f6feb)](https://docs.copp.pro/) [![PyPI](https://img.shields.io/pypi/v/copp-py.svg?color=3A6DA8)](https://pypi.org/project/copp-py/) [![Python](https://img.shields.io/badge/Python-bindings-3776ab)](#copp-python-bindings)

## Convex-Objective Path Parameterization

This directory contains the Python package for COPP. Install the open-source distribution as `copp-py` and import the Python package with `import copp_py as copp`. It wraps the Rust solver core through PyO3 while presenting a NumPy-friendly interface for paths, robot constraints, solver options, and post-processing helpers.

COPP solves optimal path-parameterization problems. A geometric path

$$
q = q(s)
$$

is converted into a time law

$$
s = s(t)
$$

so the executed trajectory `q(s(t))` satisfies velocity, acceleration, jerk, torque, or user-supplied constraints. The second-order solvers optimize the profile

$$
a(s) = \dot{s}^2
$$

and the third-order solvers optimize the pair

$$
a(s) = \dot{s}^2,\qquad b(s) = \ddot{s}.
$$

The Python API follows the Rust crate layout: core modeling namespaces live under `copp_py.path`, `copp_py.robot`, `copp_py.constraints`, `copp_py.objective`, `copp_py.interpolation`, and `copp_py.clarabel`, while algorithms live under `copp_py.solver.<algorithm>`. Examples import `copp_py as copp`, so user code can use the short `copp.Path` and `copp.solver.*` aliases. This README focuses on installing, building, running examples, and using the Python interface. For the full project overview, benchmark tables, citation information, and collaboration contact details, see the [COPP GitHub README](https://github.com/TOPP-THU/copp#readme).

> **Open-source / PRO note:** this README documents the open-source Python package distributed as `copp-py` and imported as `copp_py`. COPP PRO provides additional licensed solvers and support options; see the repository-level PRO section or contact [hello@copp.pro](mailto:hello@copp.pro) if those capabilities are relevant to your application.

The Python bindings follow a deliberately small set of rules:

- install the distribution as `copp-py` and import the Python module with `import copp_py as copp`;
- pass numerical data as NumPy-compatible arrays or ordinary Python sequences;
- use `float64` data for predictable behavior and fewer copies;
- build paths with `copp.Path`, constraints with `copp.Robot`, and solver inputs with solver-specific `Problem` classes;
- call algorithms through Rust-like solver modules such as `copp.solver.topp2_ra`, `copp.solver.copp2_socp`, and `copp.solver.copp3_socp`;
- use `copp.interpolation` for profile-to-time conversion helpers.

## API Availability

| Problem class  | Python API                                                                                                                                  |
| -------------- | ------------------------------------------------------------------------------------------------------------------------------------------- |
| Core utilities | `copp.core`, root aliases for `version`, `__version__`, errors, and common enums                                                            |
| Path           | `copp.path.Path`, spline paths, evaluator paths, path derivative evaluation                                                                 |
| Robot          | `copp.robot.Robot`, station grids, sampled path derivatives, velocity/acceleration/jerk limits, raw constraints, inverse-dynamics callbacks |
| TOPP2          | `copp.solver.topp2_ra.solve`, `copp.solver.reach_set2.backward`, `copp.solver.reach_set2.bidirectional`                                     |
| COPP2          | `copp.solver.copp2_socp.solve`, `copp.solver.copp2_socp.solve_expert`                                                                       |
| TOPP3          | `copp.solver.topp3_lp.solve`, `copp.solver.topp3_lp.solve_expert`, `copp.solver.topp3_socp.solve`, `copp.solver.topp3_socp.solve_expert`    |
| COPP3          | `copp.solver.copp3_socp.solve`, `copp.solver.copp3_socp.solve_expert`                                                                       |
| Objectives     | `copp.objective.Time`, `ThermalEnergy`, `TotalVariationTorque`, `Linear`                                                                    |
| Interpolation  | `copp.interpolation.s_to_t_topp2`, `t_to_s_topp2_uniform`, `s_to_t_topp3`, `t_to_s_topp3_uniform`, sample-based variants                    |

Runnable examples are in [examples](examples/). The Sphinx tutorials include those same files with `literalinclude`, so examples and documentation stay aligned.

## Quick Start

### Prerequisites

For the published Python package, you need:

- Python 3.9 or newer;
- `numpy`;
- `jax` for examples that build differentiable paths with `Path.from_jax`.

Create and activate any Python environment you prefer before running the commands below. The commands intentionally avoid machine-specific activation scripts, user directories, or environment names.

### Install from [PyPI](https://pypi.org/project/copp-py/)

Install COPP with pip:

```sh
python -m pip install -U pip
python -m pip install -U copp-py
```

For examples that use `Path.from_jax`, install JAX as well:

```sh
python -m pip install -U jax
```

After installation:

```sh
python -c "import copp_py as copp; print(copp.version())"
```

### Build and Install from Source

Use the source build path when you are working from a checkout or changing the Rust/Python binding code. You also need Rust, Cargo, a native compiler toolchain suitable for Rust extension modules, and `maturin`.

Install the local build tools once:

```sh
python -m pip install -U maturin numpy jax
```

Run from the repository root:

```sh
cargo build --release --lib --features python
maturin develop --release --features python
```

The Cargo command makes the Rust/Python feature build explicit. `maturin develop` then builds and installs the Python extension module into the active Python environment. After the build:

```sh
python -c "import copp_py as copp; print(copp.version())"
```

### Run Examples

Run examples from the repository root after installing the package:

```sh
python bindings/python/examples/topp2_ra.py
python bindings/python/examples/copp2_socp.py
python bindings/python/examples/topp3_socp.py
python bindings/python/examples/copp3_socp.py
python bindings/python/examples/reach_set2.py
```

## General Workflow

Most Python scripts follow the same shape:

1. Build a path from waypoints or a Python evaluator object.
2. Build a station grid `s`.
3. Create a `copp.Robot`.
4. Append stations and sample path derivatives into the robot.
5. Add velocity, acceleration, jerk, torque, or raw constraints.
6. Build a `Problem` descriptor for the chosen solver family.
7. Call a solver.
8. Convert the returned path-domain profile into `t(s)` or `s(t)` samples.
9. Evaluate the original path at `s(t)` for downstream control or plotting.

For second-order problems, the solver output is usually an `a` profile sampled on the station grid. For third-order problems, the output is a `Profile3rd` object with `a` and `b` profiles.

Path evaluation helpers return a consistent `PathDerivatives` object. For
position-only evaluation, use the `.q` field:

```python
out = path.evaluate_q(s)
q = out.q
```

Higher-order calls fill more fields on the same result type:

```python
out = path.evaluate_up_to_2nd(s)
q = out.q
dq = out.dq
ddq = out.ddq
```

## Minimal Program

```python
import copp_py as copp

print("COPP version:", copp.version())
```

## TOPP2-RA Example

This complete example builds a three-axis path with JAX, lets `Path.from_jax` provide the path derivatives, adds symmetric velocity and acceleration limits, solves TOPP2-RA, and converts the result into uniform time samples.

```python
import numpy as np
import copp_py as copp


def main() -> None:
    try:
        import jax
        import jax.numpy as jnp
    except ImportError as exc:
        raise SystemExit("Install JAX to run this example: python -m pip install jax") from exc

    jax.config.update("jax_enable_x64", True)

    dim = 3
    n = 1001
    dt = 1.0e-3

    # 1) Define q(s). Path.from_jax differentiates it up to third order.
    def q_fn(s):
        freq = jnp.array([2.0 * jnp.pi, 3.0 * jnp.pi, 5.0 * jnp.pi], dtype=jnp.float64)
        phase = jnp.array([0.0, 0.3, 0.7], dtype=jnp.float64)
        return jnp.sin(freq * s + phase)

    path = copp.Path.from_jax(q_fn, 0.0, 1.0)
    s = np.linspace(0.0, 1.0, n, dtype=np.float64)

    # 2) Build robot constraints, then apply symmetric velocity and acceleration limits in [-1, 1].
    robot = copp.Robot(dim, capacity=n)
    robot.append_s(s)
    robot.set_q_from_path_2nd(path, 0, n)

    upper = np.ones(dim, dtype=np.float64)
    lower = -upper
    robot.add_velocity_limits(upper, lower, start_idx_s=0, length=n)
    robot.add_acceleration_limits(upper, lower, start_idx_s=0, length=n)

    # 3) Solve TOPP2-RA with boundary values a(0) = 0 and a(1) = 0.
    problem = copp.solver.topp2_ra.Problem(
        robot.constraints,
        idx_s_interval=(0, n - 1),
        a_boundary=(0.0, 0.0),
    )
    options = copp.solver.topp2_ra.Options()
    a_profile = copp.solver.topp2_ra.solve(problem, options)

    # 4) Post-process TOPP2-RA results: a(s) -> t(s) -> s(t).
    t_final, t_s = copp.interpolation.s_to_t_topp2(s, a_profile, 0.0)
    s_t = copp.interpolation.t_to_s_topp2_uniform(
        s,
        a_profile,
        t_s,
        dt,
        t0=0.0,
        include_final=True,
    )

    # 5) Print the tutorial summary.
    print("TOPP2-RA done.")
    print(f"dim = {dim}, N = {n}")
    print(f"t_final = {t_final:.6f} s")
    print(f"a_profile.len() = {len(a_profile)}")
    print(f"s(t) samples = {len(s_t)}")


if __name__ == "__main__":
    main()
```

The same structure extends to COPP2 by replacing the TOPP2 problem with `copp.solver.copp2_socp.Problem` and an objective list, and to third-order solvers by using `set_q_from_path_3rd`, jerk constraints, `copp.solver.topp3_socp.Problem` or `copp.solver.copp3_socp.Problem`, and the TOPP3 interpolation helpers.

## Solver Namespaces

### `copp.solver.topp2_ra` and `copp.solver.reach_set2`

TOPP2 is the second-order time-optimal family. It optimizes `a(s)` under first- and second-order constraints. Use `copp.solver.topp2_ra.solve` for the reachability-analysis solver and `copp.solver.reach_set2.backward` / `bidirectional` when you need reachable-set bounds directly.

### `copp.solver.copp2_socp`

COPP2 solves second-order convex-objective problems. Objectives are constructed through `copp.objective`, for example:

```python
objectives = [
    copp.objective.Time(1.0),
    copp.objective.ThermalEnergy(0.1, np.ones(dim, dtype=np.float64)),
]
```

Use `copp.solver.copp2_socp.solve` for the Clarabel SOCP formulation. Use `copp.solver.copp2_socp.solve_expert` when application code needs solver status and diagnostics instead of only the accepted profile.

### `copp.solver.topp3_lp` and `copp.solver.topp3_socp`

TOPP3 is the third-order time-optimal family. It uses the `(a,b)` state and supports jerk-aware constraints. Use `copp.solver.topp3_lp.solve` for the linear-objective approximation or `copp.solver.topp3_socp.solve` for the Clarabel conic formulation. A common pattern is to generate an initial `a` profile with TOPP2-RA, substitute it into the constraints, then solve the third-order problem with LP or SOCP.

### `copp.solver.copp3_socp`

COPP3 combines third-order constraints with convex objectives. Use `copp.solver.copp3_socp.solve` for the Clarabel SOCP formulation. Third-order solvers return `Profile3rd` objects that can be post-processed with `copp.interpolation.s_to_t_topp3` and `copp.interpolation.t_to_s_topp3_uniform`.

## Data Conventions

Python inputs are accepted as NumPy-compatible array-like values. At the wrapper boundary, arrays are validated and converted into contiguous `float64` buffers when needed. To reduce copies in hot loops, pass `numpy.ndarray` values with `dtype=np.float64` and C-contiguous layout unless the function documents another layout.

Path-sampled matrices commonly use sample-major layout, where each row is one station and each column is one axis. The `MatrixLayout` enum and path helpers document the accepted alternatives.

`Path.evaluate_q`, `Path.evaluate_up_to_2nd`, and `Path.evaluate_up_to_3rd`
all return `PathDerivatives`. `evaluate_q` fills only `out.q`; derivative
fields are `None`. This keeps path evaluation calls structurally consistent
while making the requested derivative order explicit in the method name.

Boundary values are expressed in path-domain variables:

- `a_boundary=(a_start, a_final)` fixes `a = ds/dt * ds/dt`;
- `b_boundary=(b_start, b_final)` fixes `b = d2s/dt2` for third-order problems.

## Error Handling

Python argument-format errors are reported as standard Python exceptions such as `TypeError` or `ValueError`. Errors returned by the Rust core are exposed as `copp.CoppError` and typed subclasses such as `PathError` and `ConstraintError`.

```python
try:
    a_profile = copp.solver.topp2_ra.solve(problem, options)
except copp.CoppError as exc:
    print("COPP failed:", exc)
```

For Clarabel-based solvers, the simple solver functions return an accepted profile or raise an exception. Expert variants such as `copp.solver.copp2_socp.solve_expert` and `copp.solver.topp3_socp.solve_expert` expose solver status, residuals, and other diagnostic fields for applications that need status-aware behavior.

## Documentation

The Python documentation is generated with Sphinx from:

```text
bindings/python/docs/source/
```

Install documentation dependencies:

```sh
python -m pip install -U sphinx
```

Build the HTML documentation from the repository root:

```sh
python -m sphinx -E -b html bindings/python/docs/source bindings/python/docs/build/html
```

Open the generated entry page after the build:

```text
bindings/python/docs/build/html/index.html
```

The documentation is organized as:

- Guide: quick start, mathematical concepts, solver selection, tutorials, and how-to pages;
- Reference: API pages generated from Python modules and PyO3 docstrings.

Guide pages use the same path-parameterization variables as the Rust docs, and tutorial pages include runnable files from `bindings/python/examples`.

## Package Layout

```text
bindings/python/
  README.md
  copp_py/
    __init__.py          # public package facade
    core.py              # shared enums, version, and errors
    path.py              # path constructors and evaluation
    robot.py             # robot sampling and high-level constraints
    constraints.py       # raw constraint buffer namespace
    objective.py         # objective constructors
    interpolation.py     # profile/time conversion helpers
    clarabel.py          # Clarabel options and diagnostics
    solver/              # solver namespaces
      topp2_ra.py
      reach_set2.py
      copp2_socp.py
      topp3_lp.py
      topp3_socp.py
      copp3_socp.py
  docs/
    source/              # Sphinx source
    build/html/          # generated HTML output
  examples/              # runnable Python examples
```

The native extension module is built as `copp_py._native`.

## Troubleshooting

### `import copp_py as copp` Fails

Install the published package into the active Python environment:

```sh
python -m pip install -U copp-py
```

If you are working from a source checkout, build and install the local extension instead:

```sh
cargo build --release --lib --features python
maturin develop --release --features python
```

Then verify that the same interpreter can import the package:

```sh
python -c "import sys, copp_py as copp; print(sys.executable); print(copp.version())"
```

### Sphinx Cannot Import `copp_py`

Install the package first with `python -m pip install -U copp-py`, or build the local checkout with `cargo build --release --lib --features python` followed by `maturin develop --release --features python`. Then run the Sphinx command using the same Python interpreter.

### Native Build Fails

For source builds, check that Rust, Cargo, Python headers, and the platform compiler toolchain are available. On Windows, install a Visual Studio C++ build toolchain compatible with your Python interpreter. On Linux and macOS, ensure that the usual compiler and linker tools are available on `PATH`.

### Array Shape or Type Errors

Convert inputs explicitly before calling into COPP:

```python
values = np.ascontiguousarray(values, dtype=np.float64)
```

For path samples, verify the intended matrix layout and station count. Most robot-building helpers expect lengths to match the station grid already stored in `Robot`.
