classdef Problem
    %PROBLEM Borrowed TOPP2-RA problem descriptor.
    %
    % Problem stores a reference to a copp.Robot plus a closed station
    % interval and boundary values for a = (ds/dt)^2. It does not copy robot
    % constraints. The Robot must remain live for every solve() call.
    %
    % Shape convention:
    %   idx_s_interval is a 1-by-2 vector [idx_s_start, idx_s_final].
    %   a_boundary is a 1-by-2 vector [a_start, a_final]. solve() returns an
    %   s_len-by-1 profile column vector.
    %
    % MATLAB indexing:
    %   idx_s_interval is public 1-based and closed: [first, final]. The hidden
    %   native_descriptor() method converts it to the 0-based closed interval
    %   expected by the C ABI Topp2Problem descriptor.

    properties (SetAccess = private)
        %ROBOT Referenced Robot that owns station samples and constraints.
        %
        % This reference keeps the MATLAB Robot object reachable while the
        % Problem exists. The native solver borrows the underlying CoppRobot
        % only during solve().
        robot

        %IDX_S_INTERVAL Closed 1-based station interval [idx_s_start, idx_s_final].
        %
        % Both endpoints are included. For a full robot grid with N stations,
        % the default interval is [1, N].
        idx_s_interval

        %A_BOUNDARY Endpoint values [a_start, a_final].
        %
        % a is the squared path speed, a = (ds/dt)^2. TOPP2-RA commonly uses
        % [0, 0] for rest-to-rest motion.
        a_boundary

        %S_LEN Number of stations covered by idx_s_interval.
        %
        % This equals idx_s_interval(2) - idx_s_interval(1) + 1 and matches the
        % length of the profile returned by solve().
        s_len
    end

    methods
        function obj = Problem(robot, opts)
            %PROBLEM Construct and validate a TOPP2-RA problem descriptor.
            %
            % P = copp.solver.topp2_ra.Problem(ROBOT) covers all currently
            % stored stations in ROBOT and uses a_boundary=[0, 0].
            %
            % P = ...Problem(ROBOT, idx_s_interval=[I, J],
            % a_boundary=[A0, AF]) uses a closed 1-based station interval and
            % explicit boundary values. idx_s_interval and a_boundary are both
            % 1-by-2 vectors. Validation is intentionally performed on the
            % MATLAB side for shape, indexing, and finite values before the
            % native solver performs deeper feasibility checks.
            arguments
                robot (1,1) copp.Robot
                opts.idx_s_interval = []
                opts.a_boundary (1,2) {mustBeNumeric, mustBeReal, mustBeFinite} = [0, 0]
            end

            if ~robot.is_valid()
                error("copp:InvalidHandle", "Robot native handle is not valid.");
            end

            if isempty(opts.idx_s_interval)
                idx = [1, robot.len];
            else
                idx = double(opts.idx_s_interval);
            end

            validateattributes(idx, {'numeric'}, ...
                {'real', 'finite', 'integer', 'positive', 'numel', 2}, ...
                'Problem', 'idx_s_interval');
            validateattributes(opts.a_boundary, {'numeric'}, ...
                {'real', 'finite', 'nonnegative', 'numel', 2}, ...
                'Problem', 'a_boundary');

            idx = reshape(idx, 1, 2);
            if idx(2) < idx(1)
                error("copp:InvalidArgument", ...
                    "idx_s_interval must be a closed increasing 1-based interval.");
            end
            if idx(2) > robot.len
                error("copp:InvalidArgument", ...
                    "idx_s_interval exceeds the robot station range.");
            end

            obj.robot = robot;
            obj.idx_s_interval = idx;
            obj.a_boundary = reshape(double(opts.a_boundary), 1, 2);
            obj.s_len = idx(2) - idx(1) + 1;
        end
    end

    methods (Hidden)
        function [robot_id, idx_s_start, idx_s_final, a_start, a_final] = native_descriptor(obj)
            %NATIVE_DESCRIPTOR Convert this MATLAB descriptor to MEX arguments.
            %
            % Returns:
            %   robot_id:
            %       uint64 MEX registry id for the referenced Robot.
            %   idx_s_start, idx_s_final:
            %       0-based closed native station interval.
            %   a_start, a_final:
            %       Boundary values forwarded to Topp2Problem.
            if ~obj.robot.is_valid()
                error("copp:InvalidHandle", "Problem refers to an invalid Robot native handle.");
            end

            robot_id = obj.robot.native_id_for_mex();
            idx_s_start = double(obj.idx_s_interval(1) - 1);
            idx_s_final = double(obj.idx_s_interval(2) - 1);
            a_start = double(obj.a_boundary(1));
            a_final = double(obj.a_boundary(2));
        end
    end
end
