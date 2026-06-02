%% Path Module
% |copp.Path| represents a geometric path \(q(s)\). Solvers consume sampled
% derivatives of this path through |Robot|. A |Path| is useful when the geometry
% is easier to describe as waypoints or formulas than as precomputed
% \(q\), \(\dot{q}\), \(\ddot{q}\), and \(q^{(3)}\) arrays.
%
% MATLAB path matrices are always \(\mathrm{dim} \times N\). Each column is one path sample.
% This is different from Python's common sample-major layout, but matches
% MATLAB column-major arrays and the C ABI's zero-copy convention.
%
% Every evaluation call treats the input \(s\) as a vector with \(N=\mathrm{numel}(s)\).
% \(s\) may be \(1 \times N\) or \(N \times 1\), but |evaluate_q|,
% |evaluate_up_to_2nd|, and |evaluate_up_to_3rd| always return \(\mathrm{dim} \times N\)
% matrices. Callback constructors are stricter: native callbacks receive \(s\)
% as \(1 \times N\) and must return \(\mathrm{dim} \times N\).

%% Choosing a path constructor
% Use the constructor that matches the source of your geometry:
%
% * |Path.from_waypoints| for spline paths through sampled configurations.
% * |Path.from_evaluator_2nd| when a batch callback can return \(q,\dot{q},\ddot{q}\).
% * |Path.from_evaluator_3rd| when a batch callback can return
%   \(q,\dot{q},\ddot{q},q^{(3)}\).
% * |Path.from_parametric| for pure MATLAB scalar formulas using |Jet3|
%   automatic differentiation.
% * |Path.from_symbolic| when Symbolic Math Toolbox is available.
% * |Path.from_casadi| when CasADi is available and the formula already lives
%   in SX/MX form.
%
% Evaluator callbacks receive \(s\) as a \(1 \times N\) row vector and must return
% derivative matrices with shape \(\mathrm{dim} \times N\).

%% Waypoint path
% Waypoints are passed as a \(\mathrm{dim} \times n_\mathrm{waypoints}\) matrix. If boundary derivative
% states are supplied, |start_state| and |end_state| are \(\mathrm{dim} \times K\) matrices
% whose columns store derivative orders. The path object owns a native handle
% through a MATLAB-side registry. Users never see a raw pointer.

waypoints = [ ...
    0.0, 0.5, 1.0, 1.5, 2.0; ...
    0.0, 1.0, 0.0, -1.0, 0.0];
path_wp = copp.Path.from_waypoints( ...
    waypoints, ...
    s_range=[0, 2], ...
    order=3, ...
    out_of_range_mode="clamp");
cleanup_wp = onCleanup(@() path_wp.release());

[q_wp, dq_wp, ddq_wp] = path_wp.evaluate_up_to_2nd([0, 1, 2]);
fprintf("q_wp shape: %d x %d\n", size(q_wp, 1), size(q_wp, 2));
fprintf("Waypoint path dim: %d\n", path_wp.dim);
disp(table([0; 1; 2], q_wp(1,:).', q_wp(2,:).', ...
    'VariableNames', {'s', 'q1', 'q2'}))

%% Jet3 parametric path
% |Path.from_parametric| is the lightest formula constructor. The callback is
% written as a scalar formula:
%
%   @(s) [sin(s); cos(2*s); s + 0.1*s^2]
%
% During construction and evaluation, \(s\) is a scalar |copp.Jet3| value.
% The returned |Jet3| vector carries value and derivatives through third order.

path_formula = copp.Path.from_parametric( ...
    @(s) [sin(s); cos(2*s); s + 0.1*s^2], ...
    s_range=[0, 1]);
cleanup_formula = onCleanup(@() path_formula.release());

samples = [0, 0.5, 1.0];
[q, dq, ddq, dddq] = path_formula.evaluate_up_to_3rd(samples);
fprintf("Formula evaluation q shape: %d x %d\n", size(q, 1), size(q, 2));
disp(table(samples.', q(1,:).', dq(1,:).', ddq(1,:).', dddq(1,:).', ...
    'VariableNames', {'s', 'q1', 'dq1', 'ddq1', 'dddq1'}))

%% Batch evaluator path
% If the path model already has vectorized analytic derivatives, pass a
% function handle directly. The callback below accepts \(s\) as a row vector and
% returns three \(\mathrm{dim} \times N\) matrices. This callback is batch-only: it must handle
% all \(N\) samples in one call.

eval2 = @(s) deal( ...
    [s; s.^2], ...
    [ones(size(s)); 2*s], ...
    [zeros(size(s)); 2*ones(size(s))]);
path_eval = copp.Path.from_evaluator_2nd(eval2, dim=2, s_range=[0, 1]);
cleanup_eval = onCleanup(@() path_eval.release());

out = path_eval.evaluate_up_to_2nd([0, 0.25, 0.5]);
disp(out.q)

%% Sampling a path into a Robot
% Solvers operate on |Robot| station buffers rather than directly on |Path|.
% The usual workflow is:
%
% # Build a |Path|.
% # Append the station grid to a |Robot|.
% # Call |set_q_from_path_2nd| or |set_q_from_path_3rd|.
% # Add limits and construct a solver problem.

n = 9;
s_grid = linspace(0, 1, n).';
robot = copp.Robot(path_formula.dim, Capacity=n);
cleanup_robot = onCleanup(@() robot.release());
robot.append_s(s_grid);
robot.set_q_from_path_3rd(path_formula);
robot.add_velocity_limits(10 * ones(robot.dim, 1), -10 * ones(robot.dim, 1));
robot.add_acceleration_limits(20 * ones(robot.dim, 1), -20 * ones(robot.dim, 1));
robot.add_jerk_limits(50 * ones(robot.dim, 1), -50 * ones(robot.dim, 1));

fprintf("Robot now stores %d stations sampled from the formula path.\n", robot.len);

%% Error and ownership notes
% |Path.release| is optional in ordinary scripts because |delete| releases the
% native handle automatically. It is useful in tests and long sessions.
%
% Callback-backed paths keep the MATLAB callback or evaluator object alive for
% as long as the native path exists. If a callback throws, MEX catches the
% MATLAB exception and reports a |copp:*| exception rather than allowing an
% exception to cross the C ABI boundary.
