classdef Problem
    %PROBLEM Shared TOPP3 problem descriptor.
    %
    % This descriptor is shared by solver.topp3_lp, solver.topp3_socp, and
    % COPP3 facades that inherit from it. Solver-specific Problem classes are convenience facades for
    % readability; they are not strict namespace gates. The shared descriptor
    % stores a borrowed Robot reference, an owned a-linearization profile, and
    % boundary/stationary metadata. The constructor only validates and copies
    % MATLAB data; it does not linearize third-order constraints.
    %
    % Shape convention:
    %   a_linearization may be a row or column vector but is stored as an
    %   s_len-by-1 column. a_boundary and b_boundary are 1-by-2 vectors.
    %   num_stationary_max is a scalar or 1-by-2 vector [start, end]. TOPP3 and
    %   COPP3 solve() functions return Profile3rd objects whose a and b fields
    %   are s_len-by-1 double columns.
    %
    % Lazy linearization:
    %   TOPP3-LP/SOCP solve() calls pass this descriptor to the native solver.
    %   The native solver linearizes third-order constraints at solve time and
    %   mutates the Robot's internal third-order linearized-constraint cache.
    %   Reusing the same Robot with a different a_linearization is valid, but
    %   the most recent solve determines the native cache contents.

    properties (SetAccess = private)
        %ROBOT Referenced Robot that owns station samples and constraints.
        robot

        %A_LINEARIZATION Owned node profile used for third-order linearization.
        a_linearization

        %IDX_S_START Public 1-based first station covered by this problem.
        idx_s_start

        %IDX_S_FINAL Public 1-based final station covered by this problem.
        idx_s_final

        %IDX_S_INTERVAL Closed 1-based interval [idx_s_start, idx_s_final].
        idx_s_interval

        %A_BOUNDARY Endpoint values [a_start, a_final].
        a_boundary

        %B_BOUNDARY Endpoint values [b_start, b_final].
        b_boundary

        %NUM_STATIONARY_MAX_START Stationary-node cap at the start.
        num_stationary_max_start

        %NUM_STATIONARY_MAX_END Stationary-node cap at the end.
        num_stationary_max_end

        %NUM_STATIONARY_MAX Two-element vector [start, end].
        num_stationary_max

        %A_LINEARIZATION_FLOOR Native floor used while linearizing a.
        a_linearization_floor

        %S_LEN Number of stations covered by a_linearization.
        s_len
    end

    methods
        function obj = Problem(robot, a_linearization, opts)
            %PROBLEM Construct a shared TOPP3 problem descriptor.
            %
            % P = copp.solver.topp3.Problem(ROBOT, A_LINEARIZATION)
            % covers numel(A_LINEARIZATION) stations starting at station 1 and
            % uses zero a/b endpoint values. A_LINEARIZATION may be 1-by-N or
            % N-by-1 and is stored as an N-by-1 column.
            %
            % Name-value options:
            %   idx_s_start:
            %       Public 1-based first station. Defaults to 1.
            %   a_boundary:
            %       1-by-2 nonnegative endpoint values [a_start, a_final].
            %   b_boundary:
            %       1-by-2 endpoint accelerations [b_start, b_final].
            %   num_stationary_max:
            %       Scalar or [start, end] nonnegative integer stationary cap.
            %   a_linearization_floor:
            %       Positive finite floor forwarded to the native solver.
            arguments
                robot (1,1) copp.Robot
                a_linearization {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
                opts.idx_s_start (1,1) {mustBeNumeric, mustBeReal, mustBeInteger, mustBePositive} = 1
                opts.a_boundary (1,2) {mustBeNumeric, mustBeReal, mustBeFinite, mustBeNonnegative} = [0, 0]
                opts.b_boundary (1,2) {mustBeNumeric, mustBeReal, mustBeFinite} = [0, 0]
                opts.num_stationary_max = 1
                opts.a_linearization_floor (1,1) {mustBeNumeric, mustBeReal, mustBeFinite, mustBePositive} = 1.0e-10
            end

            if ~robot.is_valid()
                error("copp:InvalidHandle", "Robot native handle is not valid.");
            end

            a_lin = reshape(double(a_linearization), [], 1);
            if isempty(a_lin)
                error("copp:InvalidArgument", ...
                    "a_linearization must contain at least one station.");
            end
            if any(a_lin < 0)
                error("copp:InvalidArgument", ...
                    "a_linearization must be nonnegative.");
            end

            num_stationary = double(opts.num_stationary_max);
            validateattributes(num_stationary, {'numeric'}, ...
                {'real', 'finite', 'integer', 'nonnegative', 'vector'}, ...
                'Problem', 'num_stationary_max');
            if isscalar(num_stationary)
                num_stationary = [num_stationary, num_stationary];
            elseif numel(num_stationary) == 2
                num_stationary = reshape(num_stationary, 1, 2);
            else
                error("copp:InvalidArgument", ...
                    "num_stationary_max must be a scalar or a two-element vector.");
            end

            idx_start = double(opts.idx_s_start);
            idx_final = idx_start + numel(a_lin) - 1;
            if idx_final > robot.len
                error("copp:InvalidArgument", ...
                    "a_linearization extends past the robot station range.");
            end

            obj.robot = robot;
            obj.a_linearization = a_lin;
            obj.idx_s_start = idx_start;
            obj.idx_s_final = idx_final;
            obj.idx_s_interval = [idx_start, idx_final];
            obj.a_boundary = reshape(double(opts.a_boundary), 1, 2);
            obj.b_boundary = reshape(double(opts.b_boundary), 1, 2);
            obj.num_stationary_max_start = num_stationary(1);
            obj.num_stationary_max_end = num_stationary(2);
            obj.num_stationary_max = num_stationary;
            obj.a_linearization_floor = double(opts.a_linearization_floor);
            obj.s_len = numel(a_lin);
        end
    end

    methods (Hidden)
        function varargout = native_descriptor(obj)
            %NATIVE_DESCRIPTOR Convert this MATLAB descriptor to MEX arguments.
            %
            % The first returned index is native 0-based. a_linearization is
            % borrowed by MEX only during the solve call. The solve call is the
            % point where the native Robot's linearized third-order constraint
            % cache may be mutated.
            if ~obj.robot.is_valid()
                error("copp:InvalidHandle", ...
                    "Problem refers to an invalid Robot native handle.");
            end

            varargout = { ...
                obj.robot.native_id_for_mex(), ...
                double(obj.idx_s_start - 1), ...
                double(obj.a_linearization), ...
                double(obj.a_boundary(1)), ...
                double(obj.a_boundary(2)), ...
                double(obj.b_boundary(1)), ...
                double(obj.b_boundary(2)), ...
                double(obj.num_stationary_max_start), ...
                double(obj.num_stationary_max_end), ...
                double(obj.a_linearization_floor)};
        end
    end
end
