# Changelog

All notable changes to this project are documented in this file.

This changelog follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.1] Unreleased

### Added

- [Rust] Made `Constraints::new`, `Constraints::with_capacity`, `Constraints::with_s`, and `Constraints::with_q` public for advanced low-level constraint construction.
- [Rust] Added documentation examples for direct constraint construction, problem builders, custom path/robot integration, custom objective/sampler interfaces, and Clarabel expert status handling.
- [Python] Added the initial Python API surface, including path and robot bindings, problem/options/result types, and solver entry points.
- [c] Added a Cargo `c` feature for the C ABI; source builds now use `cargo build --release --lib --features c` when producing C libraries.

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
