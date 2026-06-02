classdef Problem < copp.solver.topp3.Problem
    %PROBLEM COPP3-SOCP problem descriptor.
    %
    % This extends the shared TOPP3 descriptor with MATLAB-owned objective
    % descriptors. Construction does not mutate native caches; solve() performs
    % third-order linearization lazily.
    %
    % Shape convention:
    %   a_linearization is stored as an s_len-by-1 column by the base TOPP3
    %   descriptor. Linear objective alpha and beta are stored as columns and
    %   both must have length s_len for COPP3. Torque objective normalize
    %   vectors must have length robot.dim and are stored as dim-by-1 columns.

    properties (SetAccess = private)
        %OBJECTIVES Cell array of MATLAB objective descriptor structs.
        objectives

        %OBJECTIVE_COUNT Number of objective descriptors.
        objective_count
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
        function obj = Problem(robot, objectives, a_linearization, opts)
            %PROBLEM Construct and validate a COPP3-SOCP problem descriptor.
            arguments
                robot (1,1) copp.Robot
                objectives
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

            packed = copp.internal.pack_objectives_for_mex(objectives);
            obj.objectives = packed.objectives;
            obj.objective_count = packed.count;
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
            %VALIDATE_OBJECTIVE_LENGTHS Check COPP3 objective length contracts.
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
                        if alpha_len ~= obj.s_len || beta_len ~= obj.s_len
                            error("copp:InvalidArgument", ...
                                "linear objective requires numel(alpha)=s_len and numel(beta)=s_len for COPP3.");
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
            %NATIVE_DESCRIPTOR Convert this descriptor to MEX arguments.
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
                double(obj.a_linearization_floor), ...
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
