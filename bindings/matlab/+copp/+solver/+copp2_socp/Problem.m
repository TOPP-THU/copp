classdef Problem
    %PROBLEM Borrowed COPP2-SOCP problem descriptor.
    %
    % Problem stores a reference to a copp.Robot plus MATLAB-owned
    % objective descriptors, a closed station interval, and endpoint values for
    % a = (ds/dt)^2. It does not copy robot constraints. The Robot must remain
    % live for every solve() call.
    %
    % Shape convention:
    %   idx_s_interval is a 1-by-2 vector [idx_s_start, idx_s_final].
    %   a_boundary is a 1-by-2 vector [a_start, a_final]. Linear objective
    %   alpha/beta payloads are column vectors packed from user row/column
    %   input; COPP2 requires numel(alpha)=s_len and numel(beta)=s_len-1.
    %   solve() returns an s_len-by-1 a profile.
    %
    % MATLAB indexing:
    %   idx_s_interval is public 1-based and closed: [first, final]. The hidden
    %   native_descriptor() method converts it to the 0-based closed interval
    %   expected by the C ABI Copp2Problem descriptor.

    properties (SetAccess = private)
        %ROBOT Referenced Robot that owns station samples and constraints.
        robot

        %OBJECTIVES Cell array of MATLAB-owned objective descriptor structs.
        %
        % Create descriptors with copp.objective.time,
        % copp.objective.linear, copp.objective.thermal_energy, or
        % copp.objective.total_variation_torque.
        objectives

        %OBJECTIVE_COUNT Number of objective descriptors.
        objective_count

        %IDX_S_INTERVAL Closed 1-based station interval [idx_s_start, idx_s_final].
        idx_s_interval

        %A_BOUNDARY Endpoint values [a_start, a_final].
        a_boundary

        %S_LEN Number of stations covered by idx_s_interval.
        s_len
    end

    properties (Access = private)
        objective_kinds
        objective_weights
        objective_alpha_lengths
        objective_beta_lengths
        objective_normalize_lengths
        objective_alpha_data
        objective_beta_data
        objective_normalize_data
    end

    methods
        function obj = Problem(robot, objectives, opts)
            %PROBLEM Construct and validate a COPP2-SOCP problem descriptor.
            %
            % P = copp.solver.copp2_socp.Problem(ROBOT, OBJECTIVES) covers
            % all currently stored stations in ROBOT and uses a_boundary=[0,0].
            %
            % OBJECTIVES may be a single descriptor, a struct array, or a cell
            % array. The usual user-facing spelling is:
            %
            %   objectives = {
            %       copp.objective.time(1.0)
            %       copp.objective.thermal_energy(0.1, ones(robot.dim,1))
            %   };
            %
            % P = ...Problem(..., idx_s_interval=[I,J], a_boundary=[A0,AF])
            % uses explicit 1-by-2 closed interval and boundary vectors.
            arguments
                robot (1,1) copp.Robot
                objectives
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

            packed = copp.internal.pack_objectives_for_mex(objectives);

            obj.robot = robot;
            obj.objectives = packed.objectives;
            obj.objective_count = packed.count;
            obj.idx_s_interval = idx;
            obj.a_boundary = reshape(double(opts.a_boundary), 1, 2);
            obj.s_len = idx(2) - idx(1) + 1;

            obj.objective_kinds = packed.kinds;
            obj.objective_weights = packed.weights;
            obj.objective_alpha_lengths = packed.alpha_lengths;
            obj.objective_beta_lengths = packed.beta_lengths;
            obj.objective_normalize_lengths = packed.normalize_lengths;
            obj.objective_alpha_data = packed.alpha_data;
            obj.objective_beta_data = packed.beta_data;
            obj.objective_normalize_data = packed.normalize_data;

            obj.validate_objective_lengths();
        end
    end

    methods (Access = private)
        function validate_objective_lengths(obj)
            %VALIDATE_OBJECTIVE_LENGTHS Check COPP2 objective length contracts.
            for i = 1:obj.objective_count
                kind = obj.objective_kinds(i);
                alpha_len = obj.objective_alpha_lengths(i);
                beta_len = obj.objective_beta_lengths(i);
                normalize_len = obj.objective_normalize_lengths(i);

                switch kind
                    case 0
                        if alpha_len ~= 0 || beta_len ~= 0 || normalize_len ~= 0
                            error("copp:InvalidArgument", ...
                                "time objective must not carry alpha, beta, or normalize payloads.");
                        end
                    case 1
                        if alpha_len ~= obj.s_len || beta_len + 1 ~= obj.s_len
                            error("copp:InvalidArgument", ...
                                "linear objective requires numel(alpha)=s_len and numel(beta)=s_len-1 for COPP2.");
                        end
                        if normalize_len ~= 0
                            error("copp:InvalidArgument", ...
                                "linear objective must not carry a normalize payload.");
                        end
                    case {2, 3}
                        if normalize_len ~= obj.robot.dim
                            error("copp:InvalidArgument", ...
                                "torque objective normalize length must equal robot.dim.");
                        end
                        if alpha_len ~= 0 || beta_len ~= 0
                            error("copp:InvalidArgument", ...
                                "torque objectives must not carry alpha or beta payloads.");
                        end
                    otherwise
                        error("copp:InvalidArgument", "Unknown objective kind code.");
                end
            end
        end
    end

    methods (Hidden)
        function varargout = native_descriptor(obj)
            %NATIVE_DESCRIPTOR Convert this MATLAB descriptor to MEX arguments.
            %
            % Returns the Robot registry id, 0-based native station interval,
            % boundary values, and packed objective arrays. The returned arrays
            % are borrowed by MEX only during the solver call.
            if ~obj.robot.is_valid()
                error("copp:InvalidHandle", "Problem refers to an invalid Robot native handle.");
            end

            varargout = { ...
                obj.robot.native_id_for_mex(), ...
                double(obj.idx_s_interval(1) - 1), ...
                double(obj.idx_s_interval(2) - 1), ...
                double(obj.a_boundary(1)), ...
                double(obj.a_boundary(2)), ...
                double(obj.objective_kinds), ...
                double(obj.objective_weights), ...
                double(obj.objective_alpha_lengths), ...
                double(obj.objective_beta_lengths), ...
                double(obj.objective_normalize_lengths), ...
                double(obj.objective_alpha_data), ...
                double(obj.objective_beta_data), ...
                double(obj.objective_normalize_data)};
        end
    end
end
