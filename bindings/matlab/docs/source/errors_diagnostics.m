%% Errors and Diagnostics
% COPP MATLAB calls raise ordinary MATLAB |MException| objects. The exception
% identifier has the package prefix |copp:|, for example:
%
% * |copp:InvalidArgument|
% * |copp:InvalidHandle|
% * |copp:ConstraintError|
% * |copp:PathError|
% * |copp:SolverError|
%
% |copp.diag.CoppError| is a MATLAB facade that wraps those exceptions into a
% small typed diagnostic object. This keeps MATLAB catch blocks idiomatic while
% still making COPP category/detail data easy to inspect.

%% Catch and wrap a COPP error

robot = copp.Robot(1, Capacity=2);
cleanup = onCleanup(@() robot.release());

try
    robot.append_s([0.0, 0.0]);
catch err
    wrapped = copp.diag.CoppError.from_exception(err);
    fprintf("Caught identifier: %s\n", err.identifier);
    fprintf("COPP category: %s\n", wrapped.category);
    fprintf("Message: %s\n", wrapped.message);
end

%% Last-error snapshot
% |copp.diag.last_error| reads the native C ABI last-error snapshot. Most
% users should prefer |try/catch| plus |diag.CoppError.from_exception|, but
% |last_error| is useful when debugging low-level native behavior.

info = copp.diag.last_error();
disp(info)

%% Solver status versus MATLAB exception
% Clarabel-backed expert solvers return solver status as data. A non-accepted
% Clarabel status can produce |result.has_a=false| or |result.has_profile=false|
% without throwing. MATLAB exceptions are reserved for malformed inputs, invalid
% native handles, callback failures, model construction failures, or normal
% non-expert |solve| calls that require an accepted profile.
%
% Diagnostic fields commonly available on expert Result objects include:
%
% * |solver_status|
% * |iterations|
% * |r_prim| and |r_dual|
% * |linsolver|
% * |objective_value| and |objective_terms|
% * raw conic vectors such as |x|, |z|, and |s|

%% Verbosity
% Most solver Options expose |verbosity|. MATLAB accepts strings or
% |copp.diag.Verbosity| enumeration values.

opts1 = copp.solver.topp2_ra.Options(verbosity="summary");
opts2 = copp.solver.copp2_socp.Options(verbosity=copp.diag.Verbosity.silent);

fprintf("TOPP2-RA verbosity: %s\n", opts1.verbosity);
fprintf("COPP2-SOCP verbosity: %s\n", opts2.verbosity);

%% Callback failures
% MATLAB callbacks are used by |Path.from_evaluator_2nd|,
% |Path.from_evaluator_3rd|, |Path.from_parametric|, and
% |Robot.set_inverse_dynamics|. If a callback throws, the MEX gateway catches
% it and converts the failure into a COPP MATLAB exception. The callback
% exception does not cross the C ABI or C++ callback boundary.
%
% A callback failure normally looks like this:
%
%   bad_path = copp.Path.from_evaluator_2nd( ...
%       @(s) error("demo:BadPath", "intentional callback failure"), ...
%       dim=1, s_range=[0, 1]);
%   try
%       bad_path.evaluate_up_to_2nd(0.5);
%   catch err
%       fprintf("Callback failure became: %s\n", err.identifier);
%   end
%
% The snippet is shown but not executed during documentation generation so the
% generated page remains focused on user-facing diagnostics rather than an
% intentionally failing callback.

%% Recommended diagnostic workflow
% # For ordinary application code, use |try/catch|.
% # If the error comes from COPP, call |copp.diag.CoppError.from_exception(err)|.
% # For SOCP solver diagnostics, call |solve_expert| and inspect the Result.
% # Use |copp.diag.last_error| only when native detail state matters.
% # Increase |verbosity| when solver progress needs to be inspected.
