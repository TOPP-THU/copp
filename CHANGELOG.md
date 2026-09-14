# Changelog

All notable changes to this project are documented in this file.

This changelog follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.2]

### Added

- [C] Added `CoppJet3` parametric path construction through `copp_path_from_parametric`, with inline helper functions such as `copp_add`, `copp_mul`, `copp_sin`, and `copp_powi` for third-order automatic derivative propagation from C callbacks.
- [C++] Added the initial C++ wrapper, including public headers, CMake integration, examples, tests, and Doxygen documentation for the supported COPP API surface.
- [MATLAB] Added the initial MATLAB wrapper, including the `+copp` package, MEX build flow, examples, tests, and generated documentation sources.

### Fixed

- [Rust] Fixed catastrophic cancellation in the second-order inverse time interpolation kernel used by `t_to_s_topp2` (and the matching `c2 = 0` branch of the third-order kernel used by `t_to_s_topp3`). When the per-interval slope of `a(s)` is tiny but nonzero (for example a numerically constant profile whose adjacent nodes differ by one ulp), the closed form `((sqrt(a0) + c1*dt/2)^2 - a0) / c1` lost all significant digits and produced sampled `s(t)` errors of about 0.01 mm that appeared as spurious acceleration spikes on uniformly resampled trajectories. The kernel now evaluates the algebraically equivalent, cancellation-free form `sqrt(a0)*dt + c1*dt^2/4`.
- [Rust] Fixed TOPP2-RA reachable-set construction clamping the squared path speed `a = sdot^2` at `1e6`. The incremental 2-D LP that propagates `a` between stations started from the fixed box bound `LP_BOUND = 1e6` as a stand-in for "+infinity"; whenever the true bound exceeded `1e6` (for example CNC feeds above 1000 mm/s with the path parameter in millimetres) the LP could only move downwards, so the speed profile was capped at `a = 1e6`, lost its cruise segment, and oscillated. The LP now starts from a per-station upper bound derived from the first-order and acceleration rows themselves, so the result is independent of the units and scale of `a`.
- [Rust] Made the incremental LP kernels behind TOPP2-RA scale-invariant. Rows are now always normalized to unit normals (the former `1e-3` floor left rows with tiny norms unnormalized, so absolute tolerances silently changed meaning with the unit of the path parameter); constraint violation, the elimination of one variable along an active row, and the reconciliation of 1-D bounds now use a relative rounding tolerance in addition to the absolute feasibility tolerance, so rows that are parallel to within rounding are treated as parallel instead of producing intersections made of rounding noise; and reachable-interval comparisons use the magnitude of the neighbouring state as tolerance reference. This removes spurious `reach_set2 ... a_max = NaN` infeasibility reports on fine station grids over large-radius arcs (for example a 10 m radius sampled every 0.01 mm at F80000) and wrong profiles when `a = sdot^2` reaches `1e16` and beyond.

## [0.2.1]

### Added

- [Rust] Made `Constraints::new`, `Constraints::with_capacity`, `Constraints::with_s`, and `Constraints::with_q` public for advanced low-level constraint construction.
- [Rust] Added documentation examples for direct constraint construction, problem builders, custom path/robot integration, custom objective/sampler interfaces, and Clarabel expert status handling.
- [Python] Added the initial Python API surface, including path and robot bindings, problem/options/result types, and solver entry points.
- [c] Added a Cargo `c` feature for the C ABI; source builds now use `cargo build --release --lib --features c` when producing C libraries.
- [MATLAB] Added a Cargo `matlab` feature aliasing the C ABI for MATLAB MEX builds; source builds now use `cargo build --release --lib --features matlab` for MATLAB native artifacts.

## [0.2.0]

### Breaking Changes

- [Rust] Changed the `RobotTorque::inverse_dynamics` contract to return `Result<(), RobotDynamicsError>`, allowing robot models to report inverse-dynamics failures with user-facing error messages.
- [Rust] Wrapped third-order trajectory results in dedicated profile types (`Topp3Profile` in Rust and `CoppProfile3rd` in the C API), replacing parts of the previous public API.

### Added

- [Rust] Added dry-friction support to inverse-dynamics evaluation through `RobotTorque`. The current model supports biased Coulomb friction; viscous friction is not supported.
- [Rust] Added `Path::from_evaluator` as a compatibility constructor for third-order path evaluators.
- [C] Added the initial C API surface, including core types, path and robot bindings, formulation helpers, solver entry points, generated headers, and C integration tests.

### Changed

- [Rust] Changed `ClarabelOptionsBuilder` defaults to accept Clarabel `AlmostSolved` status by default, aligning the Rust solver policy with the intended production-friendly default.
- [Rust] Made the `Robot::with_*` and `Constraints::with_*` builder APIs chainable.

## [0.1.0] - Initial Release

### Added

- [Rust] Initial Rust implementation of convex-objective path parameterization primitives and solvers.
