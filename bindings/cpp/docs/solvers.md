# Solver Namespaces {#cpp_solvers}

The C++ facade groups algorithms under `copp::solver`, matching the Rust,
Python, C, and MATLAB naming model. Solver-specific `Problem`, `Options`, and
`Result` types are lightweight descriptors around COPP's path-domain
constraints and profiles.

## Second-Order Solvers

TOPP2 and COPP2 optimize the scalar profile

@f[
a(s) = \dot{s}^2.
@f]

Use `copp::solver::topp2_ra` for time-optimal second-order profiles and
`copp::solver::reach_set2` when you need reachable intervals directly.

Use `copp::solver::copp2_socp` for convex objectives through Clarabel. The
strict `solve` overload returns the accepted profile, while `solve_expert`
returns solver status, residuals, raw vectors, and objective diagnostics.

## Third-Order Solvers

TOPP3 and COPP3 optimize a `copp::Profile3rd`:

@f[
a(s) = \dot{s}^2, \qquad b(s) = \ddot{s}.
@f]

Use `copp::solver::topp3_lp` for the linear-programming formulation and
`copp::solver::topp3_socp` for the conic formulation. Both share the
`copp::solver::topp3::Problem` descriptor and can return expert Clarabel
diagnostics.

Use `copp::solver::copp3_socp` when third-order constraints must be combined
with convex objective descriptors.

## Interpolation

Second-order solvers return `std::vector<double>` profiles. Convert them with
`copp::interpolation::s_to_t_topp2` and sample with the TOPP2 `t_to_s` helpers.

Third-order solvers return `copp::Profile3rd`. Convert and sample those results
with the TOPP3 interpolation helpers.
