classdef Robot < handle
    %ROBOT Native robot model and station-indexed constraint buffer.
    %
    % Robot owns a native CoppRobot handle. The handle stores station samples,
    % sampled path derivatives, and constraints used by TOPP/COPP solvers.
    % This MATLAB class exposes the workflow-oriented surface used by TOPP and
    % COPP solvers: station append, path derivative input, limit helpers, raw
    % constraints, inverse dynamics callbacks, and solver metadata.
    %
    % Matrix convention:
    %   MATLAB inputs use dim-by-N column-major matrices for robot/path data.
    %   Here dim is robot.dim and N is the number of station samples covered by
    %   the call. Each column corresponds to one station sample. One-dimensional
    %   station/profile inputs may be row or column vectors, but solver/profile
    %   outputs are returned as double column vectors.
    %
    % Constraint convention:
    %   Per-axis limit vectors have shape dim-by-1 or 1-by-dim and are
    %   broadcast over the selected station interval. Per-axis station-varying
    %   limit matrices have shape dim-by-N. Raw constraint matrices have shape
    %   R-by-N, where R is the number of inequality rows per station.
    %
    % Indexing convention:
    %   Public MATLAB station indices are 1-based. The MEX gateway converts to
    %   the native 0-based indices used by Rust, C, C++, and Python.

    properties (Access = private)
        %NATIVE_ID Registry id for the native CoppRobot handle.
        %
        % The private MEX registry maps this uint64 id to a CoppRobot* and owns
        % the actual pointer. A value of uint64(0) means this MATLAB object has
        % no live native resource.
        native_id uint64 = uint64(0)
    end

    properties (Dependent)
        %DIM Robot/path dimension.
        %
        % The dimension is fixed at construction and must match the row count
        % of q, dq, ddq, and axial limit vectors/matrices.
        dim

        %LEN Number of logical station samples currently stored.
        %
        % This increases when append_s() accepts new station values. Limit
        % methods use len to infer a default interval length.
        len

        %CAPACITY Allocated native station-buffer capacity.
        %
        % Capacity is a performance/storage detail. The native buffer may grow
        % beyond the constructor's initial capacity after appending samples.
        capacity

        %CONSTRAINTS Lightweight facade that forwards to Robot constraint APIs.
        constraints
    end

    methods
        function obj = Robot(dim, capacity, opts)
            %ROBOT Construct a native robot and empty constraint buffer.
            %
            % OBJ = copp.Robot(DIM) creates an empty DIM-dimensional robot.
            % OBJ = copp.Robot(DIM, CAPACITY) also gives the native
            % station buffer an initial capacity hint. CAPACITY may be zero.
            % OBJ = copp.Robot(DIM, Capacity=CAPACITY) is the name-value
            % equivalent, matching the style used by the rest of the MATLAB
            % facade.
            %
            % The native handle is owned by OBJ and released by delete() or
            % release().
            arguments
                dim (1,1) {mustBeNumeric, mustBeReal, mustBeInteger, mustBePositive}
                capacity (1,1) {mustBeNumeric, mustBeReal, mustBeInteger, mustBeNonnegative} = 0
                opts.Capacity (1,1) {mustBeNumeric, mustBeReal, mustBeInteger, mustBeNonnegative} = capacity
            end

            obj.native_id = copp.internal.copp_mex( ...
                'robot_create', double(dim), double(opts.Capacity));
        end

        function delete(obj)
            %DELETE Release the native robot handle during MATLAB cleanup.
            obj.release();
        end

        function released = release(obj)
            %RELEASE Explicitly release the native CoppRobot handle.
            %
            % RELEASED = release(OBJ) returns true when this call released a
            % live native handle and false when OBJ was already released.
            % Problems that reference a released Robot become invalid.
            released = false;
            if obj.native_id ~= uint64(0)
                released = copp.internal.copp_mex('release', obj.native_id);
                obj.native_id = uint64(0);
            end
        end

        function tf = is_valid(obj)
            %IS_VALID Return whether this object still owns a live native robot.
            if obj.native_id == uint64(0)
                tf = false;
                return
            end
            tf = copp.internal.copp_mex('is_valid', obj.native_id, 'Robot');
        end

        function value = get.dim(obj)
            %GET.DIM Query the native robot dimension.
            value = copp.internal.copp_mex('robot_dim', obj.native_id_for_mex());
        end

        function value = get.len(obj)
            %GET.LEN Query the number of stored station samples.
            value = copp.internal.copp_mex('robot_len', obj.native_id_for_mex());
        end

        function value = get.capacity(obj)
            %GET.CAPACITY Query the native station-buffer capacity.
            value = copp.internal.copp_mex('robot_capacity', obj.native_id_for_mex());
        end

        function value = get.constraints(obj)
            %GET.CONSTRAINTS Return a facade for constraint-oriented methods.
            value = copp.internal.ConstraintsRef(obj);
        end

        function obj = append_s(obj, s)
            %APPEND_S Append strictly increasing path station samples.
            %
            % append_s(OBJ, S) appends a real finite vector of station values.
            % S may be 1-by-N or N-by-1. S must be strictly increasing, and if
            % OBJ already stores stations then S(1) must be greater than the
            % current last station.
            %
            % The method returns OBJ to support chaining.
            arguments
                obj (1,1) copp.Robot
                s {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
            end

            copp.internal.copp_mex( ...
                'robot_append_s', obj.native_id_for_mex(), double(s));
        end

        function obj = set_q_2nd(obj, q, dq, ddq, opts)
            %SET_Q_2ND Store q, dq/ds, and d2q/ds2 over a station interval.
            %
            % set_q_2nd(OBJ, Q, DQ, DDQ) writes second-order geometric path
            % derivatives starting at station 1. Q, DQ, and DDQ must have the
            % same dim-by-N shape. Column k corresponds to station
            % start_idx_s + k - 1.
            %
            % Name-value options:
            %   start_idx_s:
            %       1-based global station index where the first column is
            %       written. Defaults to 1.
            %
            % This mirrors Python Robot.set_q(..., dddq=None), while MATLAB
            % keeps the explicit second-order name for readability.
            arguments
                obj (1,1) copp.Robot
                q {mustBeNumeric, mustBeReal, mustBeFinite}
                dq {mustBeNumeric, mustBeReal, mustBeFinite}
                ddq {mustBeNumeric, mustBeReal, mustBeFinite}
                opts.start_idx_s (1,1) {mustBeNumeric, mustBeReal, mustBeInteger, mustBePositive} = 1
            end

            copp.internal.copp_mex( ...
                'robot_set_q_2nd', ...
                obj.native_id_for_mex(), ...
                double(opts.start_idx_s - 1), ...
                double(q), ...
                double(dq), ...
                double(ddq));
        end

        function obj = set_q_3rd(obj, q, dq, ddq, dddq, opts)
            %SET_Q_3RD Store q and derivatives through third order.
            %
            % set_q_3rd(OBJ, Q, DQ, DDQ, DDDQ) writes third-order geometric
            % path derivatives starting at station 1. All matrices must have
            % the same dim-by-N shape, with one column per station.
            %
            % Name-value options:
            %   start_idx_s:
            %       1-based global station index where the first column is
            %       written. Defaults to 1.
            arguments
                obj (1,1) copp.Robot
                q {mustBeNumeric, mustBeReal, mustBeFinite}
                dq {mustBeNumeric, mustBeReal, mustBeFinite}
                ddq {mustBeNumeric, mustBeReal, mustBeFinite}
                dddq {mustBeNumeric, mustBeReal, mustBeFinite}
                opts.start_idx_s (1,1) {mustBeNumeric, mustBeReal, mustBeInteger, mustBePositive} = 1
            end

            copp.internal.copp_mex( ...
                'robot_set_q_3rd', ...
                obj.native_id_for_mex(), ...
                double(opts.start_idx_s - 1), ...
                double(q), ...
                double(dq), ...
                double(ddq), ...
                double(dddq));
        end

        function obj = set_q_from_path_2nd(obj, path, opts)
            %SET_Q_FROM_PATH_2ND Sample a Path and store second-order derivatives.
            %
            % set_q_from_path_2nd(OBJ, PATH) evaluates PATH at the station
            % values already stored in OBJ and writes q, dq/ds, and d2q/ds2
            % from station 1 through OBJ.len.
            %
            % set_q_from_path_2nd(..., idx_s_from=I, idx_s_to=J) samples the
            % closed 1-based MATLAB interval [I, J]. The MEX gateway converts
            % this to the native 0-based half-open interval [I-1, J).
            %
            % PATH must remain valid only for the duration of this call; the
            % sampled derivatives are copied into the native Robot.
            arguments
                obj (1,1) copp.Robot
                path (1,1) copp.Path
                opts.idx_s_from (1,1) {mustBeNumeric, mustBeReal, mustBeInteger, mustBePositive} = 1
                opts.idx_s_to = []
            end

            if isempty(opts.idx_s_to)
                idx_s_to = obj.len;
            else
                validateattributes(opts.idx_s_to, {'numeric'}, ...
                    {'real', 'finite', 'integer', 'positive', 'scalar'}, ...
                    'set_q_from_path_2nd', 'idx_s_to');
                idx_s_to = double(opts.idx_s_to);
            end

            idx_s_from = double(opts.idx_s_from);
            if idx_s_to < idx_s_from
                error("copp:InvalidArgument", ...
                    "idx_s_to must be greater than or equal to idx_s_from.");
            end
            if idx_s_to > obj.len
                error("copp:InvalidArgument", ...
                    "idx_s_to exceeds the robot station range.");
            end

            copp.internal.copp_mex( ...
                'robot_sample_path_2nd', ...
                obj.native_id_for_mex(), ...
                path.native_id_for_mex(), ...
                double(idx_s_from - 1), ...
                double(idx_s_to));
        end

        function obj = set_q_from_path_3rd(obj, path, opts)
            %SET_Q_FROM_PATH_3RD Sample a Path and store third-order derivatives.
            %
            % set_q_from_path_3rd(OBJ, PATH) evaluates PATH at OBJ's stored
            % station values and writes q, dq/ds, d2q/ds2, and d3q/ds3 from
            % station 1 through OBJ.len.
            %
            % set_q_from_path_3rd(..., idx_s_from=I, idx_s_to=J) samples the
            % closed 1-based MATLAB interval [I, J]. The MEX gateway converts
            % this to the native 0-based half-open interval [I-1, J).
            arguments
                obj (1,1) copp.Robot
                path (1,1) copp.Path
                opts.idx_s_from (1,1) {mustBeNumeric, mustBeReal, mustBeInteger, mustBePositive} = 1
                opts.idx_s_to = []
            end

            if isempty(opts.idx_s_to)
                idx_s_to = obj.len;
            else
                validateattributes(opts.idx_s_to, {'numeric'}, ...
                    {'real', 'finite', 'integer', 'positive', 'scalar'}, ...
                    'set_q_from_path_3rd', 'idx_s_to');
                idx_s_to = double(opts.idx_s_to);
            end

            idx_s_from = double(opts.idx_s_from);
            if idx_s_to < idx_s_from
                error("copp:InvalidArgument", ...
                    "idx_s_to must be greater than or equal to idx_s_from.");
            end
            if idx_s_to > obj.len
                error("copp:InvalidArgument", ...
                    "idx_s_to exceeds the robot station range.");
            end

            copp.internal.copp_mex( ...
                'robot_sample_path_3rd', ...
                obj.native_id_for_mex(), ...
                path.native_id_for_mex(), ...
                double(idx_s_from - 1), ...
                double(idx_s_to));
        end

        function obj = add_velocity_limits(obj, upper, lower, opts)
            %ADD_VELOCITY_LIMITS Add axial velocity bounds.
            %
            % add_velocity_limits(OBJ, UPPER, LOWER) adds per-axis bounds over
            % the interval from station 1 through OBJ.len. Vector inputs must
            % have length dim (dim-by-1 or 1-by-dim) and are broadcast over the
            % interval. Matrix inputs must have shape dim-by-N and specify
            % station-varying limits, where N is the selected interval length.
            %
            % UPPER must be positive and LOWER must be negative according to
            % the native signed-bound rules. Inf and -Inf are allowed as open
            % bounds, but NaN is rejected by the MEX gateway.
            %
            % Name-value options:
            %   start_idx_s:
            %       1-based first station receiving the limits. Defaults to 1.
            %   length:
            %       Number of stations for broadcast vector limits. When
            %       omitted, length is inferred as OBJ.len - start_idx_s + 1.
            %
            % The method returns OBJ to support chaining.
            arguments
                obj (1,1) copp.Robot
                upper {mustBeNumeric, mustBeReal}
                lower {mustBeNumeric, mustBeReal}
                opts.start_idx_s (1,1) {mustBeNumeric, mustBeReal, mustBeInteger, mustBePositive} = 1
                opts.length = []
            end

            copp.internal.copp_mex( ...
                'robot_add_velocity_limits', ...
                obj.native_id_for_mex(), ...
                double(opts.start_idx_s - 1), ...
                double(obj.limit_length(opts.start_idx_s, opts.length)), ...
                double(upper), ...
                double(lower));
        end

        function obj = add_acceleration_limits(obj, upper, lower, opts)
            %ADD_ACCELERATION_LIMITS Add axial acceleration bounds.
            %
            % This method has the same vector/matrix shape, indexing, Inf, and
            % length inference rules as add_velocity_limits(), but it writes
            % second-order acceleration constraints used by TOPP2/COPP2.
            %
            % The method returns OBJ to support chaining.
            arguments
                obj (1,1) copp.Robot
                upper {mustBeNumeric, mustBeReal}
                lower {mustBeNumeric, mustBeReal}
                opts.start_idx_s (1,1) {mustBeNumeric, mustBeReal, mustBeInteger, mustBePositive} = 1
                opts.length = []
            end

            copp.internal.copp_mex( ...
                'robot_add_acceleration_limits', ...
                obj.native_id_for_mex(), ...
                double(opts.start_idx_s - 1), ...
                double(obj.limit_length(opts.start_idx_s, opts.length)), ...
                double(upper), ...
                double(lower));
        end

        function obj = add_jerk_limits(obj, upper, lower, opts)
            %ADD_JERK_LIMITS Add axial jerk bounds for third-order solvers.
            %
            % This method has the same vector/matrix shape, indexing, Inf, and
            % length inference rules as add_velocity_limits(), but it writes
            % third-order jerk constraints used by TOPP3/COPP3 solvers. The
            % robot must already contain third-order path derivative data over
            % the target interval.
            arguments
                obj (1,1) copp.Robot
                upper {mustBeNumeric, mustBeReal}
                lower {mustBeNumeric, mustBeReal}
                opts.start_idx_s (1,1) {mustBeNumeric, mustBeReal, mustBeInteger, mustBePositive} = 1
                opts.length = []
            end

            copp.internal.copp_mex( ...
                'robot_add_jerk_limits', ...
                obj.native_id_for_mex(), ...
                double(opts.start_idx_s - 1), ...
                double(obj.limit_length(opts.start_idx_s, opts.length)), ...
                double(upper), ...
                double(lower));
        end

        function obj = add_torque_limits(obj, upper, lower, opts)
            %ADD_TORQUE_LIMITS Add axial torque bounds using inverse dynamics.
            %
            % This method has the same vector/matrix shape, indexing, Inf, and
            % length inference rules as add_velocity_limits(): vector bounds
            % have length dim and are broadcast, while matrix bounds are
            % dim-by-N with one column per station. Torque evaluation uses the
            % currently installed inverse-dynamics callback, or the native
            % default point dynamics tau = ddq when no callback is installed.
            arguments
                obj (1,1) copp.Robot
                upper {mustBeNumeric, mustBeReal}
                lower {mustBeNumeric, mustBeReal}
                opts.start_idx_s (1,1) {mustBeNumeric, mustBeReal, mustBeInteger, mustBePositive} = 1
                opts.length = []
            end

            copp.internal.copp_mex( ...
                'robot_add_torque_limits', ...
                obj.native_id_for_mex(), ...
                double(opts.start_idx_s - 1), ...
                double(obj.limit_length(opts.start_idx_s, opts.length)), ...
                double(upper), ...
                double(lower));
        end

        function obj = add_raw_constraint_1st(obj, amax, opts)
            %ADD_RAW_CONSTRAINT_1ST Add rows a <= amax over a station interval.
            %
            % AMAX may be a vector of length N for one raw row per station, or
            % an R-by-N matrix for R raw inequality rows per station. Column k
            % corresponds to station start_idx_s + k - 1.
            arguments
                obj (1,1) copp.Robot
                amax {mustBeNumeric, mustBeReal}
                opts.start_idx_s (1,1) {mustBeNumeric, mustBeReal, mustBeInteger, mustBePositive} = 1
            end

            copp.internal.copp_mex( ...
                'robot_add_raw_constraint_1st', ...
                obj.native_id_for_mex(), ...
                double(opts.start_idx_s - 1), ...
                double(amax));
        end

        function obj = add_raw_constraint_2nd(obj, acc_a, acc_b, acc_max, opts)
            %ADD_RAW_CONSTRAINT_2ND Add rows acc_a*a + acc_b*b <= acc_max.
            %
            % ACC_A, ACC_B, and ACC_MAX must have identical shapes. Use a
            % length-N vector for one row per station, or an R-by-N matrix for
            % R raw rows per station. Columns are station samples.
            arguments
                obj (1,1) copp.Robot
                acc_a {mustBeNumeric, mustBeReal, mustBeFinite}
                acc_b {mustBeNumeric, mustBeReal, mustBeFinite}
                acc_max {mustBeNumeric, mustBeReal}
                opts.start_idx_s (1,1) {mustBeNumeric, mustBeReal, mustBeInteger, mustBePositive} = 1
            end

            copp.internal.copp_mex( ...
                'robot_add_raw_constraint_2nd', ...
                obj.native_id_for_mex(), ...
                double(opts.start_idx_s - 1), ...
                double(acc_a), ...
                double(acc_b), ...
                double(acc_max));
        end

        function obj = add_raw_constraint_3rd(obj, jerk_a, jerk_b, jerk_c, jerk_d, jerk_max, opts)
            %ADD_RAW_CONSTRAINT_3RD Add third-order raw jerk inequalities.
            %
            % JERK_A, JERK_B, JERK_C, JERK_D, and JERK_MAX must have identical
            % shapes. Use a length-N vector for one row per station, or an
            % R-by-N matrix for R raw rows per station. Columns are station
            % samples.
            arguments
                obj (1,1) copp.Robot
                jerk_a {mustBeNumeric, mustBeReal, mustBeFinite}
                jerk_b {mustBeNumeric, mustBeReal, mustBeFinite}
                jerk_c {mustBeNumeric, mustBeReal, mustBeFinite}
                jerk_d {mustBeNumeric, mustBeReal, mustBeFinite}
                jerk_max {mustBeNumeric, mustBeReal}
                opts.start_idx_s (1,1) {mustBeNumeric, mustBeReal, mustBeInteger, mustBePositive} = 1
            end

            copp.internal.copp_mex( ...
                'robot_add_raw_constraint_3rd', ...
                obj.native_id_for_mex(), ...
                double(opts.start_idx_s - 1), ...
                double(jerk_a), ...
                double(jerk_b), ...
                double(jerk_c), ...
                double(jerk_d), ...
                double(jerk_max));
        end

        function obj = clear_constraints(obj, opts)
            %CLEAR_CONSTRAINTS Clear station samples, derivatives, and constraints.
            %
            % keep_idx_s=true keeps the 1-by-N/N-by-1 station vector S and
            % clears derivative and constraint buffers. keep_idx_s=false clears
            % the station vector as well.
            arguments
                obj (1,1) copp.Robot
                opts.keep_idx_s (1,1) logical = false
            end

            copp.internal.copp_mex( ...
                'robot_clear_constraints', ...
                obj.native_id_for_mex(), ...
                double(opts.keep_idx_s));
        end

        function obj = pop_front_n(obj, n)
            %POP_FRONT_N Remove N logical stations from the front.
            arguments
                obj (1,1) copp.Robot
                n (1,1) {mustBeNumeric, mustBeReal, mustBeFinite, mustBeInteger, mustBeNonnegative}
            end

            copp.internal.copp_mex( ...
                'robot_pop_front_n', obj.native_id_for_mex(), double(n));
        end

        function obj = pop_back_n(obj, n)
            %POP_BACK_N Remove N logical stations from the back.
            arguments
                obj (1,1) copp.Robot
                n (1,1) {mustBeNumeric, mustBeReal, mustBeFinite, mustBeInteger, mustBeNonnegative}
            end

            copp.internal.copp_mex( ...
                'robot_pop_back_n', obj.native_id_for_mex(), double(n));
        end

        function obj = set_inverse_dynamics(obj, callback)
            %SET_INVERSE_DYNAMICS Install a MATLAB inverse-dynamics callback.
            %
            % CALLBACK is called pointwise, not batched:
            %
            %   tau = callback(q, dq, ddq)
            %
            % q, dq, and ddq are dim-by-1 double column vectors for one station
            % and one candidate path acceleration. The returned tau must be a
            % real finite vector with dim elements; it is normalized to
            % dim-by-1 by the MEX gateway.
            arguments
                obj (1,1) copp.Robot
                callback
            end

            copp.internal.copp_mex( ...
                'robot_set_inverse_dynamics', ...
                obj.native_id_for_mex(), ...
                callback);
        end

        function obj = clear_inverse_dynamics(obj)
            %CLEAR_INVERSE_DYNAMICS Restore default point dynamics tau = ddq.
            copp.internal.copp_mex( ...
                'robot_clear_inverse_dynamics', obj.native_id_for_mex());
        end

        function [e1, e2] = exceed_topp2(obj, a, opts)
            %EXCEED_TOPP2 Maximum TOPP2 constraint violations of an a profile.
            %
            % [E1, E2] = exceed_topp2(OBJ, A) evaluates the node profile
            % A = (ds/dt)^2 on the stations idx_s_start through
            % idx_s_start + numel(A) - 1. The path acceleration of each
            % interval is reconstructed by the TOPP2 relation
            % b(k) = (a(k+1) - a(k)) / (2*(s(k+1) - s(k))), and the
            % second-order rows of both endpoint stations are checked against
            % that interval's b.
            %
            % E1 is the maximum violation of 0 <= a(k) <= amax(k) and E2 the
            % maximum violation of acc_a*a + acc_b*b <= acc_max. Each value is
            % <= 0 when the profile satisfies that constraint order and
            % positive when it is violated. Both are NaN when the station range
            % is not stored in the robot or b cannot be reconstructed from A.
            %
            % Name-value options:
            %   idx_s_start:
            %       1-based station of A(1). Defaults to 1.
            arguments
                obj (1,1) copp.Robot
                a {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
                opts.idx_s_start (1,1) {mustBeNumeric, mustBeReal, mustBeInteger, mustBePositive} = 1
            end

            [e1, e2] = copp.internal.copp_mex( ...
                'robot_exceed_topp2', ...
                obj.native_id_for_mex(), ...
                double(opts.idx_s_start - 1), ...
                double(a));
        end

        function [e1, e2, e3] = exceed_topp3(obj, a, b, opts)
            %EXCEED_TOPP3 Maximum TOPP3 constraint violations of a third-order profile.
            %
            % [E1, E2, E3] = exceed_topp3(OBJ, A, B) evaluates the node
            % profiles A = (ds/dt)^2 and B = d2s/dt2 on the stations
            % idx_s_start through idx_s_start + numel(A) - 1.
            %
            % [E1, E2, E3] = exceed_topp3(OBJ, PROFILE) takes a, b, and
            % num_stationary from a copp.Profile3rd. B and num_stationary
            % must not be passed in this form.
            %
            % E1 is the maximum violation of 0 <= a(k) <= amax(k), E2 of
            % acc_a*a(k) + acc_b*b(k) <= acc_max, and E3 of the original
            % nonlinear third-order rows
            % sqrt(a)*(jerk_a*a + jerk_b*b + jerk_c*c + jerk_d) <= jerk_max,
            % not their linearization. Each interval contributes
            % c = (b(k+1) - b(k)) / (s(k+1) - s(k)) at both of its endpoints;
            % the two stationary blocks are skipped. Each value is <= 0 when the
            % profile satisfies that constraint order and positive when it is
            % violated. All three are NaN when the station range is not stored
            % in the robot, A and B differ in length or have fewer than two
            % entries, a station interval is not strictly increasing, or
            % PROFILE holds a non-finite a or b (A and B vectors must be finite).
            %
            % This audits the profile that is actually delivered, so call it
            % after post-processing such as Profile3rd.force_positive_a.
            %
            % Name-value options:
            %   num_stationary:
            %       [NS0, NSF] stationary interval counts of the profile, as
            %       stored in Profile3rd.num_stationary. Empty (default) means
            %       [0, 0] for A/B vectors; it must stay empty for a
            %       Profile3rd.
            %   idx_s_start:
            %       1-based station of A(1). Defaults to 1.
            arguments
                obj (1,1) copp.Robot
                a
                b = []
                opts.num_stationary = []
                opts.idx_s_start (1,1) {mustBeNumeric, mustBeReal, mustBeInteger, mustBePositive} = 1
            end

            if isa(a, 'copp.Profile3rd')
                if ~isscalar(a) || ~isempty(b) || ~isempty(opts.num_stationary)
                    error("copp:InvalidArgument", ...
                        "exceed_topp3 takes b and num_stationary from a scalar Profile3rd; do not pass them separately.");
                end
                a_values = a.a;
                b_values = a.b;
                num_stationary = a.num_stationary;
            else
                if isempty(b)
                    error("copp:InvalidArgument", ...
                        "exceed_topp3 requires b unless a is a Profile3rd.");
                end
                validateattributes(a, {'numeric'}, {'real', 'finite', 'vector'}, 'exceed_topp3', 'a');
                validateattributes(b, {'numeric'}, {'real', 'finite', 'vector'}, 'exceed_topp3', 'b');
                a_values = double(a);
                b_values = double(b);
                num_stationary = [0, 0];
                if ~isempty(opts.num_stationary)
                    num_stationary = double(opts.num_stationary);
                    validateattributes(num_stationary, {'numeric'}, ...
                        {'real', 'finite', 'integer', 'nonnegative', 'numel', 2}, ...
                        'exceed_topp3', 'num_stationary');
                end
            end

            [e1, e2, e3] = copp.internal.copp_mex( ...
                'robot_exceed_topp3', ...
                obj.native_id_for_mex(), ...
                double(opts.idx_s_start - 1), ...
                a_values, ...
                b_values, ...
                double(num_stationary(1)), ...
                double(num_stationary(2)));
        end
    end

    methods (Hidden)
        function id = native_id_for_mex(obj)
            %NATIVE_ID_FOR_MEX Return the registry id after validating liveness.
            %
            % This hidden method is for package-internal calls into the private
            % MEX gateway. It keeps the raw native pointer hidden behind the
            % registry id.
            if obj.native_id == uint64(0) || ~obj.is_valid()
                error("copp:InvalidHandle", "Robot native handle is not valid.");
            end
            id = obj.native_id;
        end
    end

    methods (Access = private)
        function n = limit_length(obj, start_idx_s, requested)
            %LIMIT_LENGTH Normalize the public length option for limit methods.
            %
            % For broadcast vector limits, the C ABI requires an explicit
            % station count. MATLAB users may omit length, in which case we use
            % the current closed interval [start_idx_s, obj.len].
            start_idx_s = double(start_idx_s);
            if isempty(requested)
                n = obj.len - start_idx_s + 1;
            else
                validateattributes(requested, {'numeric'}, ...
                    {'real', 'finite', 'integer', 'nonnegative', 'scalar'}, ...
                    'add limits', 'length');
                n = double(requested);
            end
            if n < 0
                error("copp:InvalidArgument", ...
                    "start_idx_s is outside the robot station range.");
            end
        end
    end
end
