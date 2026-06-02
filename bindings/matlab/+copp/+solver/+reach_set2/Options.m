classdef Options < copp.solver.topp2_ra.Options
    %OPTIONS TOPP2 reachable-set numerical options.
    %
    % The fields and defaults match topp2_ra.Options:
    %   lp_feas_tol    = 1e-8
    %   a_cmp_abs_tol  = 1e-8
    %   a_cmp_rel_tol  = 1e-8
    %   verbosity      = "silent"

    methods
        function obj = Options(opts)
            %OPTIONS Construct TOPP2 reach-set options with optional overrides.
            arguments
                opts.lp_feas_tol (1,1) double {mustBeReal, mustBeFinite, mustBePositive} = 1.0e-8
                opts.a_cmp_abs_tol (1,1) double {mustBeReal, mustBeFinite, mustBePositive} = 1.0e-8
                opts.a_cmp_rel_tol (1,1) double {mustBeReal, mustBeFinite, mustBePositive} = 1.0e-8
                opts.verbosity (1,1) string {mustBeMember(opts.verbosity, ["silent", "summary", "debug", "trace"])} = "silent"
            end

            obj@copp.solver.topp2_ra.Options( ...
                lp_feas_tol=opts.lp_feas_tol, ...
                a_cmp_abs_tol=opts.a_cmp_abs_tol, ...
                a_cmp_rel_tol=opts.a_cmp_rel_tol, ...
                verbosity=opts.verbosity);
        end
    end
end
