% COPP  Workflow facade for COPP path-parameterization solvers.
%
% COPP computes a time law s(t) for a fixed geometric path q(s) while
% respecting velocity, acceleration, torque, jerk, and user-supplied
% constraints. The MATLAB binding is intentionally a workflow facade, not a
% one-to-one mirror of the C ABI. It prioritizes the public Path -> Robot ->
% Problem/Options -> solve -> interpolation workflow and hides raw pointers,
% C status codes, owned buffers, and most low-level debug/introspection calls.
%
% Matrix and vector conventions:
%   q, dq, ddq, dddq        dim-by-N, one station/query sample per column
%   evaluator input s       1-by-N row vector
%   profile outputs         N-by-1 double column vectors
%   raw constraints         R-by-N, one station sample per column
%   public station indices  1-based
%
% Core classes:
%   Path             - Geometric path q(s), with waypoint/evaluator/parametric constructors.
%   Robot            - Native robot station buffer, path samples, and constraints.
%   Profile3rd       - Third-order node profile with a, b, and stationary counts.
%   Jet3             - Scalar third-order automatic-differentiation value.
%
% Diagnostic package:
%   diag.CoppError   - Error facade for native status/message/detail diagnostics.
%   diag.last_error  - Native last-error diagnostic snapshot.
%   diag.Verbosity   - Native diagnostic verbosity enumeration.
%
% Clarabel package:
%   clarabel.Options           - Shared Clarabel backend policy.
%   clarabel.Settings          - Advanced raw Clarabel numerical settings.
%   clarabel.DirectSolveMethod - Direct KKT solver selection.
%
% Solver packages:
%   solver.topp2_ra      - Second-order reachability-analysis TOPP.
%   solver.reach_set2    - Second-order reachable-set construction.
%   solver.copp2_socp    - Second-order Clarabel/SOCP COPP.
%   solver.topp3_lp      - Third-order Clarabel/LP TOPP.
%   solver.topp3_socp    - Third-order Clarabel/SOCP TOPP.
%   solver.copp3_socp    - Third-order Clarabel/SOCP COPP.
%
% Utility packages:
%   objective.*       - COPP objective descriptors.
%   interpolation.*   - Convert path-domain profiles into time-domain samples.
%   internal.*        - Private MEX/evaluator helpers; not public API.
%
% See also copp.Path, copp.Robot, copp.version,
% copp.diag.CoppError, copp.clarabel.Options,
% copp.interpolation.s_to_t_topp2, copp.solver.topp2_ra.solve
