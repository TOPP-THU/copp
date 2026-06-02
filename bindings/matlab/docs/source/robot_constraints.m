%% Robot and Constraints
% |copp.Robot| is the station-indexed data container used by all MATLAB
% solver Problem objects. It stores:
%
% * a strictly increasing station grid \(s\);
% * sampled path derivatives \(q\), \(\dot{q}\), \(\ddot{q}\), and optionally \(q^{(3)}\);
% * velocity, acceleration, torque, jerk, and raw inequality constraints.
%
% Public MATLAB station indices are 1-based. The MEX layer converts them to the
% native 0-based indices used by Rust/C/C++.

%% Matrix and interval convention
% A \(\mathrm{dim} \times N\) matrix stores one station per column. If |start_idx_s=3| and a
% derivative matrix has four columns, it writes stations 3, 4, 5, and 6.
%
% Vector limit inputs such as \(\mathrm{upper}=\mathbf{1}_{\mathrm{dim} \times 1}\) are broadcast over an
% interval. Matrix limit inputs such as \(\mathrm{upper}=\mathbf{1}_{\mathrm{dim} \times N}\) are station
% varying and do not need a separate |length| argument.
%
% One-dimensional station/profile vectors such as \(s\) may be \(1 \times N\) or
% \(N \times 1\) on input. The Robot stores station samples internally; solver and
% interpolation profile outputs are returned as \(N \times 1\) columns.
%
% Raw constraints do not use \(\mathrm{dim}\) rows. They use \(R \times N\) matrices, where \(R\)
% is the number of inequality rows per station. For example, \(\mathrm{zeros}(2,N)\) adds
% two raw rows at each of the \(N\) stations.

%% Build a second-order robot
% This example uses \(q(s)=s\) in one dimension. It is intentionally small but it
% follows the same API shape as a multi-axis robot.

n = 7;
s = linspace(0.0, 1.0, n).';

robot2 = copp.Robot(1, Capacity=n);
cleanup2 = onCleanup(@() robot2.release());
robot2.append_s(s);
robot2.set_q_2nd(s.', ones(1, n), zeros(1, n));
fprintf("set_q_2nd used q/dq/ddq shapes: %d x %d\n", size(s.', 1), size(s.', 2));
robot2.add_velocity_limits(1, -1);
robot2.add_acceleration_limits(1, -1);

fprintf("Second-order robot: dim=%d, len=%d, capacity=%d\n", ...
    robot2.dim, robot2.len, robot2.capacity);

%% Sample \(q(s)\) from a Path
% Formula or waypoint paths can populate the robot derivative buffers. This
% keeps user code close to the mathematical path and avoids manual derivative
% array assembly.

path = copp.Path.from_parametric(@(u) [sin(u); cos(2*u)], s_range=[0, 1]);
cleanup_path = onCleanup(@() path.release());

robot3 = copp.Robot(path.dim, Capacity=n);
cleanup3 = onCleanup(@() robot3.release());
robot3.append_s(s);
robot3.set_q_from_path_3rd(path);

upper = 100.0 * ones(robot3.dim, 1);
robot3.add_velocity_limits(upper, -upper);
robot3.add_acceleration_limits(upper, -upper);
robot3.add_jerk_limits(upper, -upper);

fprintf("Third-order robot sampled from Path: dim=%d, len=%d\n", ...
    robot3.dim, robot3.len);

%% Torque limits and inverse dynamics
% Torque limits use the robot's inverse-dynamics callback. Without a callback,
% the native default is point dynamics, equivalent to \(\tau=\ddot{q}\). For quick
% modeling or tests, install a MATLAB callback. The callback is pointwise, not
% batched: \(q\), \(\dot{q}\), \(\ddot{q}\), and returned \(\tau\) are all \(\mathrm{dim} \times 1\) vectors.

robot2.set_inverse_dynamics(@(q, dq, ddq) ddq + 0.05*dq + 0.0*q);
robot2.add_torque_limits(10, -10);
robot2.clear_inverse_dynamics();

fprintf("Torque limits accepted with a MATLAB inverse-dynamics callback.\n");

%% Raw constraints
% High-level limit functions are preferred for ordinary use. Raw constraints
% are available for advanced users who already have inequality rows in native
% path-domain form.
%
% First order:
%
% $$ a \le a_{\max}. $$
%
% Second order:
%
% $$ A a + B b \le h. $$
%
% Third order:
%
% $$ A a + B b + C c + D \le h. $$
%
% The MATLAB facade exposes both direct robot methods and a lightweight
% |robot.constraints| forwarding facade.
%
% In the calls below, each raw array is \(1 \times N\), meaning one inequality row at
% each station. To provide \(R\) raw rows per station, pass \(R \times N\) arrays with
% matching shapes.

robot2.constraints.add_raw_constraint_1st(ones(1, n), start_idx_s=1);
robot2.constraints.add_raw_constraint_2nd( ...
    zeros(1, n), zeros(1, n), 100 * ones(1, n), start_idx_s=1);

fprintf("Raw first- and second-order constraints added.\n");

%% Queue-style operations
% Receding-horizon code can discard stations from either end. These operations
% mutate the native robot buffer, so any Problem object built before mutation
% should be rebuilt.

robot2.pop_front_n(1);
robot2.pop_back_n(1);
fprintf("After pop operations, robot2.len=%d\n", robot2.len);

%% Clearing constraints
% |clear_constraints(keep_idx_s=true)| keeps the station grid but removes path
% derivatives and constraints. Use |keep_idx_s=false| to clear everything.

robot2.clear_constraints(keep_idx_s=true);
fprintf("After clearing constraints with stations kept, robot2.len=%d\n", robot2.len);

%% Practical checklist
% Before constructing a solver Problem:
%
% # Append a strictly increasing station grid.
% # Fill \(q,\dot{q},\ddot{q}\) for TOPP2/COPP2 or \(q,\dot{q},\ddot{q},q^{(3)}\) for TOPP3/COPP3.
% # Add enough limits or raw constraints to make the problem meaningful.
% # Use 1-based |idx_s_interval| or |idx_s_start| values in MATLAB.
% # Rebuild Problem objects after mutating the robot cache.
