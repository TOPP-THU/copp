%% TOPP3 and COPP3 Solvers
% Third-order solvers add jerk-level feasibility and return
% |copp.Profile3rd| objects. A |Profile3rd| stores the node profiles and
% stationary-prefix metadata needed by the third-order interpolation helpers.
% User code usually treats it as an opaque solver result: pass it to
% |copp.interpolation.s_to_t_topp3| or inspect its public fields when
% comparing solver output.
%
% Shape summary:
%
% * Station vector \(s\) has length \(N\) and may be row or column input.
% * Robot derivative matrices \(q\), \(\dot{q}\), \(\ddot{q}\), and \(q^{(3)}\) are \(\mathrm{dim} \times N\).
% * |a_linearization| may be \(1 \times N\) or \(N \times 1\) and is stored as an \(N \times 1\)
%   column inside Problem objects. The optional |b_linearization| follows
%   the same convention and is empty by default.
% * |Profile3rd.a| and |Profile3rd.b| are \(s_\mathrm{len} \times 1\) columns.

%% Which third-order solver should I use?
% * |solver.topp3_lp.solve| solves a third-order TOPP linear program through
%   Clarabel.
% * |solver.topp3_socp.solve| solves a third-order TOPP conic formulation.
% * |solver.copp3_socp.solve| optimizes convex objectives through Clarabel.

%% Lazy linearization
% Third-order Problem objects store MATLAB-owned descriptors. They do not
% refresh native third-order linearized constraints during construction. The
% |solve| call performs lazy linearization using the selected
% |a_linearization| profile and mutates the robot's native third-order cache.
% Rebuild Problem descriptors after changing the robot or the linearization
% profile.

%% Build a shared third-order robot

n = 7;
s = linspace(0.0, 1.0, n).';
scales = [1; 2];

robot = copp.Robot(2, Capacity=n);
cleanup = onCleanup(@() robot.release());
robot.append_s(s);
robot.set_q_3rd( ...
    scales .* s.', ...
    repmat(scales, 1, n), ...
    zeros(2, n), ...
    zeros(2, n));

upper = 100.0 * ones(robot.dim, 1);
robot.add_velocity_limits(upper, -upper);
robot.add_acceleration_limits(upper, -upper);
robot.add_jerk_limits(upper, -upper);

a_linearization = ones(n, 1);

%% TOPP3-LP and TOPP3-SOCP

lp_problem = copp.solver.topp3_lp.Problem( ...
    robot, ...
    a_linearization, ...
    idx_s_start=1, ...
    a_boundary=[0, 0], ...
    b_boundary=[0, 0], ...
    num_stationary_max=1);
profile_lp = copp.solver.topp3_lp.solve(lp_problem);

socp_problem = copp.solver.topp3_socp.Problem( ...
    robot, ...
    a_linearization, ...
    idx_s_start=1, ...
    a_boundary=[0, 0], ...
    b_boundary=[0, 0], ...
    num_stationary_max=1);
profile_socp = copp.solver.topp3_socp.solve(socp_problem);

fprintf("TOPP3-LP max(a)=%.6f, TOPP3-SOCP max(a)=%.6f\n", ...
    max(profile_lp.a), max(profile_socp.a));
fprintf("TOPP3 profile shapes a/b: %d x %d / %d x %d\n", ...
    size(profile_lp.a, 1), size(profile_lp.a, 2), ...
    size(profile_lp.b, 1), size(profile_lp.b, 2));

%% Adaptive linearization around a solved profile
% By default every third-order row of station \(k\) is linearized at
% |a_linearization(k)|. Passing |b_linearization| as well selects adaptive
% linearization: each row is anchored where it is nearly active at the
% feasible state \((a_\mathrm{lin}, b_\mathrm{lin})\), so a slack row is
% anchored high and a tight one stays near the state. Both vectors must have
% the same length and come from one previously solved third-order
% |Profile3rd|; the \(b\) of a TOPP2 profile is discretized differently and
% must not be used.
%
% Prefer this mode when \(a\) spans a wide range, and especially when \(a\)
% approaches zero (about \(10^{-10}\)): anchoring both rows of an axis at the
% same small \(a_\mathrm{lin}(k)\) leaves \(a(k) \le 3 a_\mathrm{lin}(k)\). On
% profiles well away from zero the default direct mode is cheaper and nearly
% identical. Every third-order Problem class accepts |b_linearization|.

refined_problem = copp.solver.topp3_lp.Problem( ...
    robot, ...
    max(profile_lp.a, 0), ...
    idx_s_start=1, ...
    a_boundary=[0, 0], ...
    b_boundary=[0, 0], ...
    num_stationary_max=1, ...
    b_linearization=profile_lp.b);
profile_refined = copp.solver.topp3_lp.solve(refined_problem);

fprintf("Refined TOPP3-LP max(a)=%.6f\n", max(profile_refined.a));

%% COPP3-SOCP

objectives = {
    copp.objective.time(1.0)
    copp.objective.thermal_energy(0.1, ones(robot.dim, 1))
};
% COPP3 linear objectives, if used, require both alpha and beta to have length
% s_len. Torque objective normalize vectors have length dim=robot.dim.
copp3_problem = copp.solver.copp3_socp.Problem( ...
    robot, ...
    objectives, ...
    a_linearization, ...
    idx_s_start=1, ...
    a_boundary=[0, 0], ...
    b_boundary=[0, 0], ...
    num_stationary_max=1);
profile_copp3 = copp.solver.copp3_socp.solve(copp3_problem);
result_copp3 = copp.solver.copp3_socp.solve_expert(copp3_problem);

fprintf("COPP3-SOCP status=%s, objective=%.6f\n", ...
    result_copp3.solver_status, result_copp3.objective_value);

%% Positive-a repair and constraint audit
% A third-order profile whose \(a\) touches zero can have an interpolated
% \(a(s)\) that dips below zero between stations.
% |Profile3rd.force_positive_a(s)| returns an adjusted copy whose
% interpolated \(a(s)\) stays strictly positive on each interval; its second
% output is |false| when the adjustment fails, which is not an error. The
% repair rewrites \(a\) and \(b\) without knowing the robot limits, so audit
% the delivered profile with |Robot.exceed_topp3|, which returns the maximum
% first-order, second-order, and original nonlinear third-order violations.
% Values \(\le 0\) are feasible.

[profile_safe, repaired] = profile_socp.force_positive_a(s);
[exceed_1st, exceed_2nd, exceed_3rd] = robot.exceed_topp3(profile_safe);
fprintf("force_positive_a succeeded=%d, max violations: %.3g / %.3g / %.3g\n", ...
    repaired, exceed_1st, exceed_2nd, exceed_3rd);

%% Interpolate a Profile3rd

[t_final, t_s] = copp.interpolation.s_to_t_topp3(s, profile_socp);
profile_table = table(s, profile_socp.a, profile_socp.b, t_s, ...
    'VariableNames', {'s', 'a', 'b', 't_s'});
disp(head(profile_table))
fprintf("TOPP3-SOCP final traversal time: %.6f seconds\n", t_final);

%% Expert results
% |topp3_lp.solve_expert|, |topp3_socp.solve_expert|, and
% |copp3_socp.solve_expert| return Result objects with raw Clarabel vectors,
% solver status, iteration count, residuals, objective terms, and linear-solver
% metadata. Use them for diagnostics; use normal |solve| functions when you
% only need an accepted |Profile3rd|.
