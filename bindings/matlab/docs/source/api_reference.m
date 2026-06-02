%% API Reference
% This page summarizes the public COPP MATLAB API. It is organized by
% package namespace and is intended as a quick map from a workflow need to the
% corresponding class or function.
%
% The package name is |copp|. All APIs below are public unless explicitly
% marked as diagnostic or advanced. The |internal| package is intentionally
% omitted.
%
% During documentation builds, each public class and function also gets a
% generated reference page under |ref/| using the help comments in the source
% file. Those pages are listed in the generated |helptoc.xml| so MATLAB's Help
% Browser and static website builds can expose per-API documentation.

%% Core package
% * |copp.version| - return the native COPP library version.
% * |copp.Path| - owned native path handle with waypoint, evaluator,
%   symbolic, CasADi, and Jet3-parametric constructors.
% * |copp.Robot| - station-indexed native robot buffer for samples,
%   limits, raw constraints, and inverse dynamics.
% * |copp.Profile3rd| - MATLAB-owned TOPP3/COPP3 profile containing
%   \(a\), \(b\), and stationary metadata.
% * |copp.Jet3| - scalar third-order automatic-differentiation value used
%   by |Path.from_parametric|.
%
% Diagnostics:
%
% * |copp.diag.Verbosity| - native diagnostic verbosity enumeration.
% * |copp.diag.CoppError|, |copp.diag.is_copp_error|, and
%   |copp.diag.last_error| - MATLAB and native diagnostic helpers.

%% Path
% |copp.Path| constructors:
%
% * |Path.from_waypoints(waypoints, ...)| - spline path through
%   \(\mathrm{dim} \times n_\mathrm{waypoints}\) samples.
% * |Path.from_evaluator_2nd(evaluator, dim=..., ...)| - callback path with
%   \(q\), \(\dot{q}\), and \(\ddot{q}\).
% * |Path.from_evaluator_3rd(evaluator, dim=..., ...)| - callback path with
%   \(q\) through \(q^{(3)}\).
% * |Path.from_parametric(formula, ...)| - pure MATLAB formula differentiated
%   through |Jet3|.
% * |Path.from_symbolic(q_expr, ...)| - Symbolic Math Toolbox formula.
% * |Path.from_casadi(q_expr, ...)| - CasADi SX/MX formula.
%
% Evaluation and lifetime:
%
% * |evaluate_q(s)| returns \(q(s)\).
% * |evaluate_up_to_2nd(s)| returns \(q\), \(\dot{q}\), and \(\ddot{q}\).
% * |evaluate_up_to_3rd(s)| returns \(q\), \(\dot{q}\), \(\ddot{q}\), and \(q^{(3)}\).
% * |release| explicitly releases the native handle; |delete| also releases
%   it during ordinary MATLAB cleanup.

%% Robot and constraints
% |copp.Robot| stores station grids and constraints consumed by solver
% Problem objects.
%
% Station and derivative data:
%
% * |append_s(s)| appends strictly increasing station samples.
% * |set_q_2nd(q,dq,ddq, ...)| stores second-order derivatives.
% * |set_q_3rd(q,dq,ddq,dddq, ...)| stores third-order derivatives.
% * |set_q_from_path_2nd(path, ...)| and |set_q_from_path_3rd(path, ...)|
%   sample a |Path| into the robot buffer.
%
% High-level constraints:
%
% * |add_velocity_limits(upper, lower, ...)|
% * |add_acceleration_limits(upper, lower, ...)|
% * |add_jerk_limits(upper, lower, ...)|
% * |add_torque_limits(upper, lower, ...)|
%
% Advanced constraints and callbacks:
%
% * |add_raw_constraint_1st|, |add_raw_constraint_2nd|, and
%   |add_raw_constraint_3rd| add native path-domain inequalities.
% * |set_inverse_dynamics(callback)| installs a pointwise MATLAB inverse
%   dynamics callback for torque constraints.
% * |clear_inverse_dynamics| restores default point dynamics.
% * |clear_constraints|, |pop_front_n|, and |pop_back_n| mutate the station
%   buffer for reuse or receding-horizon workflows.

%% Objective descriptors
% Objective descriptors are lightweight MATLAB structs created by factory
% functions in |copp.objective|.
%
% * |time(weight)| - traversal-time objective.
% * |linear(weight, alpha, beta)| - linear profile objective.
% * |thermal_energy(weight, normalize)| - torque-energy style objective.
% * |total_variation_torque(weight, normalize)| - torque total-variation
%   objective.
%
% COPP2 linear objectives use \(\alpha\) length \(s_\mathrm{len}\) and \(\beta\) length
% \(s_\mathrm{len}-1\). COPP3 linear objectives use node-based \(\alpha\) and \(\beta\) payloads
% aligned with the third-order profile length.

%% Clarabel options and diagnostics
% Clarabel-backed solvers share |copp.clarabel.Options| and
% |copp.clarabel.Settings|. Clarabel-backed solver namespaces also re-export
% solver-local aliases such as |copp.solver.copp3_socp.ClarabelOptions|.
%
% * |copp.clarabel.Options| controls native verbosity, accepted solver-status
%   policy, and optional raw Clarabel settings.
% * |copp.clarabel.Settings| exposes advanced numerical settings such as
%   tolerances, iteration limits, time limits, and direct-solver policy.
% * |copp.clarabel.DirectSolveMethod| selects the direct KKT solver method when
%   Clarabel supports that choice.
%
% Normal |solve| functions return an accepted profile or raise. Expert
% |solve_expert| functions return Result objects with solver status, raw
% vectors, residuals, timing, objective value, objective terms, and linear
% solver metadata.

%% Second-order solvers
% TOPP2 and COPP2 operate on the scalar node profile
% \(a = (ds/dt)^2\).
% Solver namespaces that return second-order profiles re-export |s_to_t|,
% |t_to_s|, and |a_to_b| as local aliases for the global interpolation helpers.
%
% * |copp.solver.topp2_ra.Problem|, |Options|, and |solve| - time-optimal
%   reachability-analysis TOPP2.
% * |copp.solver.reach_set2.Problem|, |Options|, |ReachSet|,
%   |backward|, and |bidirectional| - reachable interval construction.
% * |copp.solver.copp2_socp.Problem|, |Options|, |Result|, |solve|, and
%   |solve_expert| - Clarabel/SOCP COPP2.

%% Third-order solvers
% TOPP3 and COPP3 operate on |Profile3rd|, which stores node profiles \(a\) and
% \(b\) plus stationary metadata.
% Solver namespaces that return third-order profiles re-export |Profile3rd|,
% |s_to_t|, and |t_to_s| as local aliases for the global profile/interpolation
% helpers.
%
% * |copp.solver.topp3.Problem| - shared third-order descriptor accepted
%   by TOPP3 and COPP3 facades.
% * |copp.solver.topp3_lp.Problem|, |Options|, |Result|, |solve|, and
%   |solve_expert| - Clarabel/LP TOPP3.
% * |copp.solver.topp3_socp.Problem|, |Options|, |Result|, |solve|, and
%   |solve_expert| - Clarabel/SOCP TOPP3.
% * |copp.solver.copp3_socp.Problem|, |Options|, |Result|, |solve|, and
%   |solve_expert| - Clarabel/SOCP COPP3.

%% Interpolation
% Interpolation helpers convert solver profiles into time-domain samples.
%
% TOPP2:
%
% * |s_to_t_topp2(s,a, ...)| - cumulative arrival times.
% * |a_to_b_topp2(s,a)| - interval accelerations from node profile \(a\).
% * |t_to_s_topp2_uniform(s,a,t_s,dt, ...)| - uniform time-grid samples.
% * |t_to_s_topp2_samples(s,a,t_s,t_sample)| - explicit time samples.
% * |t_to_s_topp2| - compatibility wrapper around uniform/sample helpers.
%
% TOPP3:
%
% * |s_to_t_topp3(s,profile, ...)| - cumulative arrival times for
%   |Profile3rd|.
% * |t_to_s_topp3_uniform|, |t_to_s_topp3_samples|, and |t_to_s_topp3| -
%   third-order time-to-path sampling helpers.

%% Optional dependencies
% The core MATLAB binding does not require Symbolic Math Toolbox or CasADi.
% Only the corresponding path constructors require those packages:
%
% * |Path.from_symbolic| requires Symbolic Math Toolbox.
% * |Path.from_casadi| requires CasADi on the MATLAB path.
%
% |Path.from_parametric| uses the bundled |Jet3| helper and does not require
% an external automatic-differentiation package.

%% See also
% |copp.Path|, |copp.Robot|, |copp.Profile3rd|,
% |copp.solver.topp2_ra.solve|, |copp.solver.copp3_socp.solve|,
% |copp.interpolation.s_to_t_topp2|
