%% COPP for MATLAB
% COPP solves optimal path-parameterization problems. A geometric path is
% written as
%
% $$ q = q(s), \qquad s \in [s_{\min}, s_{\max}], $$
%
% where \(q\) is robot configuration and \(s\) is a scalar path coordinate. The
% solver does not change the geometric path. It computes a time law
%
% $$ s = s(t) $$
%
% so the robot traverses the path while respecting velocity, acceleration,
% torque, jerk, and user-supplied constraints. In short, COPP turns a
% geometry-space path into a feasible and optimized time schedule.
%
% The MATLAB package is named |copp|. Public code should call APIs through
% that package namespace.

%% Solver profiles
% Solvers return compact path-speed profiles rather than time-sampled robot
% trajectories. In second-order solvers the main output is \(a\), the squared
% path speed at station nodes. Third-order solvers return |copp.Profile3rd|,
% which stores the node profiles needed by the third-order interpolation
% helpers.
%
% Use the interpolation functions to convert solver output into time-domain
% data:
%
% * |copp.interpolation.s_to_t_topp2| converts a TOPP2/COPP2 profile into
%   cumulative arrival times.
% * |copp.interpolation.s_to_t_topp3| converts |copp.Profile3rd| into
%   cumulative arrival times.
% * |t_to_s_*| helpers sample the path position at requested times.

%% TOPP and COPP
% *TOPP* means time-optimal path parameterization. A TOPP solver seeks the
% shortest feasible traversal time under a selected constraint model.
%
% *COPP* means convex-objective path parameterization. COPP solvers keep the
% same path-domain constraints, but optimize convex objective terms such as
% time, linear penalties, thermal energy, or total variation of torque.
%
% The MATLAB binding exposes both second-order and third-order families:
%
% * |solver.topp2_ra| - second-order reachability-analysis TOPP.
% * |solver.reach_set2| - second-order reachable-set construction.
% * |solver.copp2_socp| - second-order Clarabel/SOCP COPP.
% * |solver.topp3_lp| and |solver.topp3_socp| - third-order Clarabel TOPP.
% * |solver.copp3_socp| - third-order COPP.

%% Package layout
% The MATLAB API mirrors the Rust and Python structure while using MATLAB
% idioms:
%
% * |copp.Path| builds and evaluates \(q(s)\).
% * |copp.Robot| stores station grids, path derivatives, and constraints.
% * |copp.objective.*| creates objective descriptors for COPP solvers.
% * |copp.solver.<algorithm>| contains Problem, Options, Result, and solve
%   entry points.
% * |copp.interpolation.*| converts path-domain profiles into time-domain
%   samples.
% * |copp.diag.CoppError| and |copp.diag.last_error| help diagnose failures.
%
% MATLAB public station indices are 1-based. Matrices use MATLAB's natural
% column-major convention: a \(\mathrm{dim} \times N\) matrix stores one station/sample per
% column.
%
% The MATLAB binding is a workflow facade, not a one-to-one C ABI mirror. It
% prioritizes the public |Path|, |Robot|, solver, objective, and interpolation
% workflows. Raw native pointers, owned C buffers, status-code plumbing, and
% low-level debug/introspection calls are intentionally hidden unless they make
% a user-facing workflow clearer.

%% Shape glossary
% The binding deliberately uses one convention across all MATLAB entry points:
%
% * \(\mathrm{dim}\) is the robot/path dimension, equal to |robot.dim| or |path.dim|.
% * \(N\) is the number of station or query samples covered by the current call.
% * Path and robot derivative matrices \(q\), \(\dot{q}\), \(\ddot{q}\), and \(q^{(3)}\) are always
%   \(\mathrm{dim} \times N\). Column \(k\) belongs to station/query sample \(k\).
% * Query vectors such as \(s\), \(a\), \(b\), \(t_s\), and \(t_\mathrm{sample}\) may be row or
%   column inputs, but profile and interpolation outputs are returned as
%   double column vectors.
% * Evaluator callbacks receive \(s\) as a \(1 \times N\) row vector and return
%   \(\mathrm{dim} \times N\) derivative matrices.
% * Per-axis limit vectors have length \(\mathrm{dim}\) and are broadcast over stations.
%   Per-axis limit matrices are \(\mathrm{dim} \times N\). Raw constraint matrices are \(R \times N\),
%   where \(R\) is the number of inequality rows per station.
%
% A quick mental check: if a matrix is about robot coordinates, rows are axes
% and columns are stations. If a matrix is about raw inequalities, rows are
% inequality rows and columns are stations.

%% First complete workflow
% This example solves a compact TOPP2-RA problem. The path is \(q(s)=s\), sampled
% on seven stations. The robot starts and ends at rest, with unit velocity and
% acceleration bounds.

n = 7;
s = linspace(0.0, 1.0, n).';

robot = copp.Robot(1, Capacity=n);
cleanup = onCleanup(@() robot.release());
robot.append_s(s);
% q, dq, and ddq are dim x N matrices. Here dim=1 and N=n.
robot.set_q_2nd(s.', ones(1, n), zeros(1, n));
robot.add_velocity_limits(1, -1);
robot.add_acceleration_limits(1, -1);

problem = copp.solver.topp2_ra.Problem( ...
    robot, ...
    idx_s_interval=[1, n], ...
    a_boundary=[0, 0]);
a = copp.solver.topp2_ra.solve(problem);
[t_final, t_s] = copp.interpolation.s_to_t_topp2(s, a);

fprintf("a shape: %d x %d, t_s shape: %d x %d\n", ...
    size(a, 1), size(a, 2), size(t_s, 1), size(t_s, 2));
summary = table(s, a, t_s, 'VariableNames', {'s', 'a', 't_s'});
disp(summary)
fprintf("Final traversal time: %.6f seconds\n", t_final);

%% Reading the rest of the documentation
% A useful learning order is:
%
% # |Path Module| - choose how to represent geometry.
% # |Robot and Constraints| - convert geometry into station-indexed limits.
% # |TOPP2 and COPP2 Solvers| - solve the easiest full problems first.
% # |TOPP3 and COPP3 Solvers| - add jerk-level modeling.
% # |Interpolation| - turn \(a(s)\) or |Profile3rd| into \(s(t)\).
% # |Errors and Diagnostics| - understand exceptions, expert solver statuses,
%   and verbosity.
