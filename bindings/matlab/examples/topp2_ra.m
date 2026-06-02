%TOPP2_RA Second-order time-optimal path parameterization example.
%
% This mirrors examples/topp2_ra.rs: N=1001 samples of a 3-axis Lissajous
% path, symmetric velocity/acceleration limits in [-1,1], TOPP2-RA solve,
% and conversion from a(s) to t(s) and uniform s(t) samples.

ExampleCommon.setup_path();

% 1) Build the analytic path, station grid, and second-order robot limits.
ctx = ExampleCommon.second_order_context();

% 2) Solve TOPP2-RA with boundary values a(0)=0 and a(1)=0.
problem = copp.solver.topp2_ra.Problem( ...
    ctx.robot, ...
    idx_s_interval=ctx.idx_s_interval, ...
    a_boundary=ctx.a_boundary);
options = copp.solver.topp2_ra.Options();
a_profile = copp.solver.topp2_ra.solve(problem, options);

% 3) Post-process TOPP2-RA results: a(s) -> t(s) -> uniform s(t).
[t_final, ~, s_t] = ExampleCommon.postprocess_topp2(ctx, a_profile);

% 4) Print a compact summary for command-line / CI usage.
ExampleCommon.print_second_order("TOPP2-RA", ctx, t_final, a_profile, s_t);
