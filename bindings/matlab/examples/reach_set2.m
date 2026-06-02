%REACH_SET2 Second-order reachable-set example.
%
% This mirrors examples/reach_set2.rs: compute both backward-only and
% bidirectional reachable intervals for the same N=1001 3-axis path used by
% the TOPP2/COPP2 examples.

ExampleCommon.setup_path();

% 1) Build the analytic path, station grid, and second-order robot limits.
ctx = ExampleCommon.second_order_context();

% 2) Build the TOPP2 problem descriptor shared by reach-set solvers.
problem = copp.solver.reach_set2.Problem( ...
    ctx.robot, ...
    idx_s_interval=ctx.idx_s_interval, ...
    a_boundary=ctx.a_boundary);
options = copp.solver.reach_set2.Options();

% 3) Compute backward-only and bidirectional reachable intervals.
reach_back = copp.solver.reach_set2.backward(problem, options);
reach_bidir = copp.solver.reach_set2.bidirectional(problem, options);

% 4) Print interval counts and representative start/mid/end bounds.
ExampleCommon.print_reach_set2(ctx, reach_back, reach_bidir);
