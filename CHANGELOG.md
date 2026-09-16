# Changelog

All notable changes to this project are documented in this file.

This changelog follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.3] - Unreleased

### Breaking

- [Rust] `Topp3ProblemBuilder` and `Copp3ProblemBuilder` gained the public field `linearization_mode`. Code that builds either builder with a struct literal no longer compiles; add `linearization_mode: LinearizationModeTopp3::Direct` to keep the previous behavior, or construct the builder through `new` and the `with_*` methods, which are unaffected.
- [Rust] `PathError` gained the `Smoothing` variant for waypoint-fitting failures, so an exhaustive `match` on `PathError` needs a new arm.
- [Rust] The public API now exposes nalgebra 0.35 types instead of nalgebra 0.34 ones. A downstream crate that shares nalgebra values with this one must upgrade to nalgebra 0.35; the two versions are distinct types and do not interoperate.
- [Rust] The crate no longer enables nalgebra's `serde-serialize` feature. Code that relied on it being enabled transitively must now enable it on its own nalgebra dependency.
- [Rust] The minimum supported Rust version rises from 1.88 to 1.89.

### Added

- [All languages] Added adaptive third-order linearization (`LinearizationModeTopp3::GivenFeasibleAdaptive`), letting each third-order row take its own linearization point when the `a` and `b` of a previously solved third-order profile are both available; recommended where `a` spans a wide range and especially where it approaches zero, with the uniform `a_linearization` kept as the default. Rust selects it through `with_linearization_mode` on the third-order problem builders, and the bindings by passing the optional `b_linearization` alongside `a_linearization`; the builders reject a `b_linearization` whose length differs from `a_linearization`. For this, the C `Topp3Problem` and `Copp3Problem` gained a trailing `b_linearization` field and the C++ `topp3::Problem` and `copp3_socp::Problem` constructors a trailing `b_linearization` parameter with an empty default; existing source still compiles, but binaries built against 0.2.2 must be recompiled.
- [All languages] Added tolerance-bounded waypoint fitting through `from_waypoints_fitting` (`Path::from_waypoints_fitting` and its borrowed-view variant `Path::from_waypoints_fitting_view` in Rust, `copp_path_from_waypoints_fitting` with `CoppSmoothingOptions` in C, `SmoothingConfig` in C++ and Python), using adaptive nonuniform `C4` quintic B-splines with uniform or per-axis tolerances, selectable fitting axes, custom waypoint parameters, whole-interval Bernstein error auditing, budget-limited local refinement, and construction diagnostics through `smoothing_report` (`copp_path_smoothing_report` in C).
- [All languages] Added `exceed_topp2` and `exceed_topp3` (the now-public `Constraints::exceed_topp2` and `Constraints::exceed_topp3` in Rust, `copp_robot_exceed_topp2` and `copp_robot_exceed_topp3` in C) for checking a delivered profile against the original constraints: they return the maximum first- and second-order violations of a TOPP2 profile, and the maximum first-, second- and third-order violations of a TOPP3 profile. The third-order term is evaluated in its original nonlinear `sqrt(a)` form rather than the linearized one, so a profile can be re-checked after post-processing such as `force_positive_a`.
- [All languages] Made `force_positive_a` available in every language: `Profile3rd` gained it in C++, Python and MATLAB, matching the Rust function and C's `copp_force_positive_a_3rd`. MATLAB returns the adjusted copy because its `Profile3rd` is a value class.
- [C] Added `COPP_STATUS_PATH_SMOOTHING` (309) for waypoint-fitting failures, the borrowed `CoppSliceUsize` slice type, and the `COPP_VERSION_MAJOR`, `COPP_VERSION_MINOR`, `COPP_VERSION_PATCH` and `COPP_VERSION_STRING` macros in `core.h`.

### Changed

- [All languages] Renamed the interpolating waypoint constructor to `from_waypoints_interpolating` (`copp_path_from_waypoints_interpolating` in C). In Rust, `Path::from_waypoints` and `Path::from_waypoints_view` remain as hidden deprecated forwarding aliases of `Path::from_waypoints_interpolating` and `Path::from_waypoints_interpolating_view`; in the bindings, `from_waypoints` remains an equivalent alias. The waypoint interpolation and fitting constructor families, including their `_view` variants, are now explicitly documented as unstable while additional algorithm-selection options are developed.
- [All languages] Third-order boundary states are now classified by exact zero: a zero initial `a` requires `b >= 0` and a zero terminal `a` requires `b <= 0` (previously both required `b == 0` within `f64::EPSILON`), non-finite boundary values are rejected, and a strictly positive `a` below `f64::EPSILON` is no longer treated as a stationary endpoint.
- [All languages] Third-order problem construction now rejects a window whose station intervals `s.len() - 1` do not exceed `num_stationary.0.max(1) + num_stationary.1.max(1)`, computed from the effective stationary counts, with an invalid-input error; such windows were previously accepted.
- [MATLAB] The toolbox is now published as a single universal `copp-matlab.mltbx` that carries the MEX gateway for every supported platform (Windows x86-64, Linux x86-64, macOS arm64 and macOS x86-64), replacing the per-platform toolbox assets. Each release attaches that one file under both its tagged name and the stable `copp-matlab.mltbx`, `install_copp` resolves it without detecting the platform first, and the toolbox is also listed on MATLAB File Exchange.

### Fixed

- [Rust] Ensured Clarabel-based third-order optimization profiles keep the reconstructed `a = s_dot^2` nonnegative throughout each ordinary moving interval by adding shared conservative Bernstein constraints.
- [All languages] Improved the numerical robustness of the interpolation helpers behind `s_to_t_topp2`, `t_to_s_topp2`, `s_to_t_topp3` and `t_to_s_topp3`.
- [Rust] Fixed numerical stability issues in LP2D and TOPP2-RA.
- [All languages] Fixed TOPP2-RA clamping `a = s_dot^2` at `1e6`: the backward reachability LP started from that fixed box bound and could only move down, so with the path parameter in millimetres any feed above 1000 mm/s lost its cruise segment or dipped to exactly `a = 1e6` on arcs. The LP now starts from a per-station upper bound on `a[k]`.
- [All languages] Made the incremental LP kernels and the TOPP2-RA passes independent of the unit of the path parameter: the 2-D violation test, deferred correction and interval reconciliation use a relative rounding tolerance, and the interval comparisons in `reach_set2` and the TOPP2-RA forward pass take their tolerance from the magnitude of the neighbouring state, so the same problem in m, mm, um or nm gives the same traversal time and a fine grid over a large-radius arc no longer reports a spurious `a_max = NaN`.
- [All languages] Fixed the row alignment of the linearized third-order constraints, which could fall out of sync with the third-order constraint rows added by `with_constraint_3order`.
- [All languages] Fixed COPP2-SOCP misaligning its first-order constraint rows by one row, which dropped `a >= 0` at the last interior station and loosened the first finite `amax` bound (or, without any finite `amax`, the first second-order bound) by that station's `a`, so the returned profile could exceed that bound where it was active, typically when the profile starts near full speed rather than at rest.

## [0.2.2]

### Added

- [C] Added `CoppJet3` parametric path construction through `copp_path_from_parametric`, with inline helper functions such as `copp_add`, `copp_mul`, `copp_sin`, and `copp_powi` for third-order automatic derivative propagation from C callbacks.
- [C++] Added the initial C++ wrapper, including public headers, CMake integration, examples, tests, and Doxygen documentation for the supported COPP API surface.
- [MATLAB] Added the initial MATLAB wrapper, including the `+copp` package, MEX build flow, examples, tests, and generated documentation sources.

## [0.2.1]

### Added

- [Rust] Made `Constraints::new`, `Constraints::with_capacity`, `Constraints::with_s`, and `Constraints::with_q` public for advanced low-level constraint construction.
- [Rust] Added documentation examples for direct constraint construction, problem builders, custom path/robot integration, custom objective/sampler interfaces, and Clarabel expert status handling.
- [Python] Added the initial Python API surface, including path and robot bindings, problem/options/result types, and solver entry points.
- [C] Added a Cargo `c` feature for the C ABI; source builds now use `cargo build --release --lib --features c` when producing C libraries.
- [MATLAB] Added a Cargo `matlab` feature aliasing the C ABI for MATLAB MEX builds; source builds now use `cargo build --release --lib --features matlab` for MATLAB native artifacts.

## [0.2.0]

### Breaking

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
