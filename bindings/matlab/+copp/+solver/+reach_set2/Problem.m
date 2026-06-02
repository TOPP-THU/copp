classdef Problem < copp.solver.topp2_ra.Problem
    %PROBLEM Borrowed TOPP2 reachable-set problem descriptor.
    %
    % This class intentionally mirrors topp2_ra.Problem: it stores a live
    % copp.Robot reference, a closed 1-based station interval, and boundary
    % values for a = (ds/dt)^2. The native reach-set routines borrow the Robot
    % only during backward() or bidirectional().
    %
    % Shape convention matches topp2_ra.Problem: idx_s_interval and a_boundary
    % are 1-by-2 vectors; ReachSet outputs store a_min/a_max as s_len-by-1
    % columns.

    methods
        function obj = Problem(robot, opts)
            %PROBLEM Construct and validate a TOPP2 reach-set descriptor.
            %
            % P = copp.solver.reach_set2.Problem(ROBOT) covers all stored
            % stations and uses a_boundary=[0, 0].
            %
            % P = ...Problem(ROBOT, idx_s_interval=[I,J], a_boundary=[A0,AF])
            % uses a closed 1-based station interval.
            arguments
                robot (1,1) copp.Robot
                opts.idx_s_interval = []
                opts.a_boundary (1,2) {mustBeNumeric, mustBeReal, mustBeFinite} = [0, 0]
            end

            obj@copp.solver.topp2_ra.Problem( ...
                robot, ...
                idx_s_interval=opts.idx_s_interval, ...
                a_boundary=opts.a_boundary);
        end
    end
end
