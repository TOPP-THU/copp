%% TOPP2 and COPP2 Solvers
% Second-order solvers consume a station grid, sampled path derivatives, and
% constraints stored in |copp.Robot|. They return a column vector \(a\)
% sampled on the closed station interval. Convert \(a\) to arrival times with
% |copp.interpolation.s_to_t_topp2|, then sample \(s(t)\) with the
% |t_to_s_topp2_*| helpers when you need playback or plotting data.
%
% Shape summary:
%
% * Station vector \(s\) has length \(N\) and may be row or column input.
% * Robot derivative matrices \(q\), \(\dot{q}\), \(\ddot{q}\) are \(\mathrm{dim} \times N\).
% * TOPP2/COPP2 solver outputs \(a\) are \(s_\mathrm{len} \times 1\) columns.
% * TOPP2 edge accelerations from |a_to_b_topp2| have length \(s_\mathrm{len}-1\).
% * ReachSet2 stores \(a_\mathrm{min}\) and \(a_\mathrm{max}\) as \(s_\mathrm{len} \times 1\) columns.

%% Which second-order solver should I use?
% * |solver.topp2_ra.solve| returns a time-optimal profile using
%   reachability analysis. It is the recommended first solver.
% * |solver.reach_set2.backward| and |bidirectional| expose reachable
%   acceleration intervals directly.
% * |solver.copp2_socp.solve| solves convex-objective second-order problems
%   with Clarabel.
% * |solver.copp2_socp.solve_expert| returns Clarabel diagnostics.

%% Build a shared second-order robot

n = 9;
s = linspace(0.0, 1.0, n).';

robot = copp.Robot(1, Capacity=n);
cleanup = onCleanup(@() robot.release());
robot.append_s(s);
robot.set_q_2nd(s.', ones(1, n), zeros(1, n));
robot.add_velocity_limits(1, -1);
robot.add_acceleration_limits(1, -1);

%% TOPP2-RA

topp_problem = copp.solver.topp2_ra.Problem( ...
    robot, ...
    idx_s_interval=[1, n], ...
    a_boundary=[0, 0]);
a_topp = copp.solver.topp2_ra.solve(topp_problem);
[t_topp, ~] = copp.interpolation.s_to_t_topp2(s, a_topp);

fprintf("TOPP2-RA final time: %.6f seconds\n", t_topp);
fprintf("TOPP2-RA a shape: %d x %d\n", size(a_topp, 1), size(a_topp, 2));

%% ReachSet2
% The backward reach set stores feasible upper/lower \(a\) bounds at every
% station.

reach_problem = copp.solver.reach_set2.Problem( ...
    robot, ...
    idx_s_interval=[1, n], ...
    a_boundary=[0, 0]);
reach = copp.solver.reach_set2.backward(reach_problem);

fprintf("ReachSet2 a_min/a_max shapes: %d x %d / %d x %d\n", ...
    size(reach.a_min, 1), size(reach.a_min, 2), ...
    size(reach.a_max, 1), size(reach.a_max, 2));

disp(table(s, reach.a_min, reach.a_max, ...
    'VariableNames', {'s', 'a_min', 'a_max'}))

%% COPP2-SOCP
% COPP2-SOCP uses objective descriptors. This example combines traversal time
% and a small thermal-energy term.
% The thermal normalize vector has length \(\mathrm{dim}\). COPP2 linear objectives, if
% used, require \(\alpha\) length \(s_\mathrm{len}\) and \(\beta\) length \(s_\mathrm{len}-1\).

objectives = {
    copp.objective.time(1.0)
    copp.objective.thermal_energy(0.05, ones(robot.dim, 1))
};
coppblem = copp.solver.copp2_socp.Problem( ...
    robot, ...
    objectives, ...
    idx_s_interval=[1, n], ...
    a_boundary=[0, 0]);

a_socp = copp.solver.copp2_socp.solve(coppblem);
result_socp = copp.solver.copp2_socp.solve_expert(coppblem);
[t_socp, ~] = copp.interpolation.s_to_t_topp2(s, a_socp);

fprintf("COPP2-SOCP final time: %.6f seconds\n", t_socp);
fprintf("COPP2-SOCP expert status: %s, iterations=%d\n", ...
    result_socp.solver_status, result_socp.iterations);

%% Compare profiles

profile_summary = table(s, a_topp, a_socp, ...
    'VariableNames', {'s', 'topp2_ra', 'copp2_socp'});
disp(profile_summary)

%% Indexing and boundary notes
% |idx_s_interval=[1,n]| is a closed MATLAB interval. Boundary values
% |a_boundary=[0,0]| mean rest-to-rest in path speed. If your station grid is a
% subwindow of a larger robot buffer, pass the corresponding 1-based interval.
