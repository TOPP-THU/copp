classdef Options
    %OPTIONS TOPP2-RA numerical options.
    %
    % Defaults mirror the Rust/Python TOPP2 reachable-set options:
    %   lp_feas_tol    = 1e-8
    %   a_cmp_abs_tol  = 1e-8
    %   a_cmp_rel_tol  = 1e-8
    %   verbosity      = "silent"
    %
    % The same option names are used in MATLAB, Python, and the C ABI to make
    % cross-language examples easy to compare.

    properties
        %LP_FEAS_TOL Feasibility tolerance used by TOPP2 LP subproblems.
        %
        % Smaller values request stricter feasibility checks. Values must be
        % finite and positive.
        lp_feas_tol (1,1) double {mustBeReal, mustBeFinite, mustBePositive} = 1.0e-8

        %A_CMP_ABS_TOL Absolute tolerance for comparing reachable a bounds.
        %
        % Used by reachable-set interval comparisons in TOPP2-RA.
        a_cmp_abs_tol (1,1) double {mustBeReal, mustBeFinite, mustBePositive} = 1.0e-8

        %A_CMP_REL_TOL Relative tolerance for comparing reachable a bounds.
        %
        % Used together with a_cmp_abs_tol for scale-aware comparisons.
        a_cmp_rel_tol (1,1) double {mustBeReal, mustBeFinite, mustBePositive} = 1.0e-8

        %VERBOSITY Native diagnostic verbosity.
        %
        % Accepted values are "silent", "summary", "debug", and "trace". The
        % MEX gateway maps these strings to CoppVerbosity.
        verbosity (1,1) string {mustBeMember(verbosity, ["silent", "summary", "debug", "trace"])} = "silent"
    end

    methods
        function obj = Options(opts)
            %OPTIONS Construct TOPP2-RA options with optional overrides.
            %
            % O = copp.solver.topp2_ra.Options() uses all native defaults.
            % O = ...Options(lp_feas_tol=1e-9, verbosity="summary") overrides
            % selected fields by name.
            arguments
                opts.lp_feas_tol (1,1) double {mustBeReal, mustBeFinite, mustBePositive} = 1.0e-8
                opts.a_cmp_abs_tol (1,1) double {mustBeReal, mustBeFinite, mustBePositive} = 1.0e-8
                opts.a_cmp_rel_tol (1,1) double {mustBeReal, mustBeFinite, mustBePositive} = 1.0e-8
                opts.verbosity (1,1) string {mustBeMember(opts.verbosity, ["silent", "summary", "debug", "trace"])} = "silent"
            end

            obj.lp_feas_tol = opts.lp_feas_tol;
            obj.a_cmp_abs_tol = opts.a_cmp_abs_tol;
            obj.a_cmp_rel_tol = opts.a_cmp_rel_tol;
            obj.verbosity = opts.verbosity;
        end
    end
end
