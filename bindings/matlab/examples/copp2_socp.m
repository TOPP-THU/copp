%COPP2_SOCP Second-order convex-objective SOCP example.
%
% This mirrors examples/copp2_socp.rs: solve the N=1001 3-axis path with
% velocity/acceleration limits in [-1,1] and a hybrid objective
% 1.0*time + 0.1*thermal_energy.

ExampleCommon.setup_path();

% 1) Build the analytic path, station grid, and second-order robot limits.
ctx = ExampleCommon.second_order_context();

% 2) Build COPP2 objectives and the Clarabel-backed SOCP problem.
objectives = ExampleCommon.convex_objectives(ctx.dim);
problem = copp.solver.copp2_socp.Problem( ...
    ctx.robot, ...
    objectives, ...
    idx_s_interval=ctx.idx_s_interval, ...
    a_boundary=ctx.a_boundary);
options = copp.solver.copp2_socp.Options();

% 3) Solve COPP2-SOCP and post-process a(s) -> t(s) -> uniform s(t).
a_profile = copp.solver.copp2_socp.solve(problem, options);
[t_final, ~, s_t] = ExampleCommon.postprocess_topp2(ctx, a_profile);

% 4) Print a compact summary for command-line / CI usage.
ExampleCommon.print_second_order("COPP2-SOCP", ctx, t_final, a_profile, s_t);
