classdef Problem < copp.solver.topp3.Problem
    %PROBLEM TOPP3-LP problem descriptor.
    %
    % This class is a package-specific facade over the shared
    % copp.solver.topp3.Problem descriptor. topp3_lp.solve accepts the
    % shared descriptor too; use this facade when the solver namespace improves
    % readability. Construction is side-effect free with respect to native
    % third-order linearization; solve() performs lazy linearization and
    % mutates the Robot's native cache.

    methods
        function obj = Problem(robot, a_linearization, opts)
            %PROBLEM Construct a TOPP3-LP problem descriptor.
            %
            % P = copp.solver.topp3_lp.Problem(ROBOT, A_LINEARIZATION)
            % creates a descriptor for topp3_lp.solve. Name-value options are
            % the same as copp.solver.topp3.Problem. A shared
            % copp.solver.topp3.Problem can also be passed to solve().
            arguments
                robot (1,1) copp.Robot
                a_linearization {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
                opts.idx_s_start (1,1) {mustBeNumeric, mustBeReal, mustBeInteger, mustBePositive} = 1
                opts.a_boundary (1,2) {mustBeNumeric, mustBeReal, mustBeFinite, mustBeNonnegative} = [0, 0]
                opts.b_boundary (1,2) {mustBeNumeric, mustBeReal, mustBeFinite} = [0, 0]
                opts.num_stationary_max = 1
                opts.a_linearization_floor (1,1) {mustBeNumeric, mustBeReal, mustBeFinite, mustBePositive} = 1.0e-10
            end

            obj@copp.solver.topp3.Problem( ...
                robot, ...
                a_linearization, ...
                idx_s_start=opts.idx_s_start, ...
                a_boundary=opts.a_boundary, ...
                b_boundary=opts.b_boundary, ...
                num_stationary_max=opts.num_stationary_max, ...
                a_linearization_floor=opts.a_linearization_floor);
        end
    end
end
