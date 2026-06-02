%% Objectives and Clarabel
% TOPP solvers optimize traversal time. COPP solvers accept a list of convex
% objective descriptors. MATLAB descriptors are ordinary structs created by
% |copp.objective.*| functions; Problem objects copy the descriptor payload
% into the MEX call when a solver runs.

%% Objective terms
% The built-in objective factories are:
%
% * |copp.objective.time(weight)| - minimize traversal time.
% * |copp.objective.linear(weight, alpha, beta)| - add linear profile
%   penalties.
% * |copp.objective.thermal_energy(weight, normalize)| - penalize
%   squared/normalized torque-like effort.
% * |copp.objective.total_variation_torque(weight, normalize)| - penalize
%   torque variation in SOCP formulations.
%
% COPP2 and COPP3 use the same factory names, but their discrete profile
% layouts differ. In COPP2, \(a\) is node-based and \(b\) is interval-based, so a
% linear objective uses \(\mathrm{length}(\beta) = s_\mathrm{len} - 1\). In COPP3, \(a\) and \(b\) are
% both node-based, so \(\mathrm{length}(\beta) = s_\mathrm{len}\).
%
% Shape rules:
%
% * \(\alpha\) and \(\beta\) may be row or column vectors at construction time, but
%   descriptors store them as double columns.
% * |normalize| is a per-axis vector with length \(\mathrm{dim}=\mathrm{robot.dim}\) and is stored
%   as a \(\mathrm{dim} \times 1\) double column.
% * |time| carries no vector payloads; its \(\alpha\), \(\beta\), and |normalize|
%   fields are empty \(0 \times 1\) columns.

%% Solver support
% The practical support matrix is:
%
% * |time| - supported by COPP2-SOCP and COPP3-SOCP.
% * |linear| - supported by COPP2-SOCP and COPP3-SOCP.
% * |thermal_energy| - supported by COPP2-SOCP and COPP3-SOCP.
% * |total_variation_torque| - supported by SOCP backends.

%% Construct objective descriptors

n = 7;
dim = 2;

time_obj = copp.objective.time(1.0);
linear2 = copp.objective.linear(0.05, zeros(n, 1), zeros(n - 1, 1));
linear3 = copp.objective.linear(0.05, zeros(n, 1), zeros(n, 1));
thermal = copp.objective.thermal_energy(0.1, ones(dim, 1));
tv_torque = copp.objective.total_variation_torque(0.01, ones(dim, 1));

fprintf("linear2 alpha/beta lengths: %d / %d\n", ...
    numel(linear2.alpha), numel(linear2.beta));
fprintf("thermal normalize shape: %d x %d\n", ...
    size(thermal.normalize, 1), size(thermal.normalize, 2));

objective_table = table( ...
    ["time"; "linear2"; "linear3"; "thermal"; "tv_torque"], ...
    [time_obj.weight; linear2.weight; linear3.weight; thermal.weight; tv_torque.weight], ...
    'VariableNames', {'name', 'weight'});
disp(objective_table)

%% Clarabel options
% Clarabel-backed solvers share |copp.clarabel.Options| and
% |copp.clarabel.Settings|. Solver-local Options classes subclass the shared
% type for namespace readability:
%
% * |copp.solver.copp2_socp.Options|
% * |copp.solver.topp3_lp.Options|
% * |copp.solver.topp3_socp.Options|
% * |copp.solver.copp3_socp.Options|
%
% Options can be built from defaults, name-value pairs, or structs. Enum-like
% values accept strings or MATLAB enumeration values.

settings = copp.clarabel.Settings( ...
    max_iter=120, ...
    tol_gap_rel=1.0e-7, ...
    direct_solve_method="qdldl");
options = copp.solver.copp2_socp.Options( ...
    verbosity=copp.diag.Verbosity.summary, ...
    allow_almost_solved=true, ...
    clarabel_settings=settings);

fprintf("Clarabel max_iter=%d, direct method=%s\n", ...
    options.clarabel_settings.max_iter, ...
    string(options.clarabel_settings.direct_solve_method));

struct_options = copp.clarabel.Options(struct( ...
    "max_iter", 80, ...
    "tol_feas", 1.0e-7, ...
    "allow_max_time", true));
fprintf("Struct options max_iter=%d, allow_max_time=%d\n", ...
    struct_options.clarabel_settings.max_iter, ...
    struct_options.allow_max_time);

%% Normal and expert solver APIs
% Normal |solve| functions return only accepted profiles:
%
%   a = copp.solver.copp2_socp.solve(problem, options)
%   profile = copp.solver.copp3_socp.solve(problem, options)
%
% Expert functions return diagnostic |Result| objects. A non-accepted Clarabel
% status is represented by |result.has_a=false| or |result.has_profile=false|
% rather than by a MATLAB exception. True model-construction or native runtime
% failures still throw |MException| values with identifiers such as
% |copp:SolverError|.
%
% Use expert APIs when you need raw vectors, solver status, iteration counts,
% residuals, linear-solver metadata, or objective-term breakdowns.

%% Status policy
% |copp.clarabel.Options| controls which Clarabel statuses are accepted as usable:
%
% * |allow_almost_solved|
% * |allow_max_iterations|
% * |allow_max_time|
% * |allow_callback_terminated|
% * |allow_insufficient_progress|
%
% The default accepts |AlmostSolved| but rejects time/iteration termination.
% This matches the Rust/Python idea that solver status is diagnostic data, while
% MATLAB exceptions are reserved for malformed inputs, native errors, and
% non-expert hard failures.
