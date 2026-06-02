classdef Path < handle
    %PATH Owned geometric path handle backed by the native COPP library.
    %
    % Path represents q(s), where s is the scalar path parameter and q is a
    % dim-dimensional robot configuration. MATLAB objects own a native
    % CoppPath handle through the private native_id property. The native handle
    % is released automatically by delete(), and may also be released
    % explicitly with release().
    %
    % The public API uses MATLAB-native column-major matrices: waypoints and
    % evaluator outputs are dim-by-N matrices, with each column storing one
    % path sample. Query vectors s may be 1-by-N or N-by-1, but all evaluated
    % q/dq/ddq/dddq outputs are dim-by-N. This matches the C ABI zero-copy
    % layout and differs from Python's common sample-major layout.
    %
    % Indexing note:
    %   Path itself does not expose station indices. Robot and solver APIs use
    %   1-based MATLAB station indices at the public boundary.

    properties (Access = private)
        %NATIVE_ID Registry id for the native CoppPath handle.
        %
        % MATLAB never stores or exposes raw native pointers. The MEX gateway
        % owns a registry that maps this uint64 id to a CoppPath*. A value of
        % uint64(0) means this MATLAB object no longer owns a native handle.
        native_id uint64 = uint64(0)
    end

    properties (Dependent)
        %DIM Robot/path dimension.
        %
        % This is the number of rows in q(s). For waypoint paths it equals the
        % number of rows in the dim-by-N waypoint matrix passed to
        % Path.from_waypoints.
        dim

        %S_RANGE Valid scalar path-parameter range [s_min, s_max].
        %
        % Values outside this range are handled according to the path's
        % out_of_range_mode option.
        s_range
    end

    methods (Access = private)
        function obj = Path(native_id)
            %PATH Private constructor from an already-created native handle id.
            %
            % Use Path.from_waypoints for user-facing construction. Keeping the
            % constructor private prevents users from forging arbitrary native
            % ids and centralizes lifetime management in from_native_id().
            obj.native_id = native_id;
        end
    end

    methods (Static)
        function obj = from_waypoints(waypoints, opts)
            %FROM_WAYPOINTS Construct a waypoint spline path.
            %
            % OBJ = copp.Path.from_waypoints(WAYPOINTS) builds a quintic
            % spline over s in [0, 1]. WAYPOINTS must be a real finite dim-by-N
            % matrix with each column storing one waypoint q(:, k). At least
            % enough waypoints for the selected native spline order must be
            % provided.
            %
            % Name-value options:
            %   s_range:
            %       1-by-2 finite vector [s_min, s_max]. Defaults to [0, 1].
            %   order:
            %       Odd spline order accepted by the native path module.
            %       Defaults to 5, matching Rust/Python defaults.
            %   out_of_range_mode:
            %       "error" to reject out-of-range evaluation, or "clamp" to
            %       clamp query values into s_range.
            %   start_state, end_state:
            %       Optional dim-by-K boundary derivative matrices. Column j
            %       stores the j-th boundary derivative. Empty arrays request
            %       zero boundary derivatives from the native default path
            %       options.
            %
            % The returned object owns the native path handle. release() may be
            % called explicitly, but ordinary MATLAB object cleanup is also
            % sufficient.
            arguments
                waypoints {mustBeNumeric, mustBeReal, mustBeFinite}
                opts.s_range (1,2) {mustBeNumeric, mustBeReal, mustBeFinite} = [0, 1]
                opts.order (1,1) {mustBeNumeric, mustBeReal, mustBeInteger, mustBePositive} = 5
                opts.out_of_range_mode (1,1) string {mustBeMember(opts.out_of_range_mode, ["error", "clamp"])} = "error"
                opts.start_state {mustBeNumeric, mustBeReal, mustBeFinite} = []
                opts.end_state {mustBeNumeric, mustBeReal, mustBeFinite} = []
            end

            id = copp.internal.copp_mex( ...
                'path_from_waypoints', ...
                double(waypoints), ...
                double(opts.s_range(1)), ...
                double(opts.s_range(2)), ...
                double(opts.order), ...
                char(opts.out_of_range_mode), ...
                double(opts.start_state), ...
                double(opts.end_state));
            obj = copp.Path.from_native_id(id);
        end

        function obj = from_evaluator_2nd(evaluator, opts)
            %FROM_EVALUATOR_2ND Construct a callback-backed second-order path.
            %
            % OBJ = copp.Path.from_evaluator_2nd(F, dim=D, s_range=[A,B])
            % wraps a MATLAB function handle F. During native path evaluation
            % the MEX gateway calls:
            %
            %   [q, dq, ddq] = F(s)
            %
            % The callback input s is always a 1-by-N row vector. The three
            % outputs must be real finite D-by-N matrices. The native path
            % borrows the callback, so this Path object keeps the MATLAB
            % function handle alive until release()/delete().
            %
            % OBJ = copp.Path.from_evaluator_2nd(E, ...) also accepts an
            % object or handle object E that implements:
            %
            %   [q, dq, ddq] = E.evaluate_up_to_2nd(s)
            %
            % Function handles are handy for scripts and quick demos. Objects
            % are better when the evaluator owns parameters, caches, or helper
            % methods that should be tested independently.
            %
            % Name-value options:
            %   dim:
            %       Positive integer path dimension D.
            %   s_range:
            %       1-by-2 finite vector [s_min, s_max]. Defaults to [0, 1].
            %
            % The resulting native path supports evaluate_q() and
            % evaluate_up_to_2nd(). Third-order evaluation raises a path
            % unsupported-derivative error.
            arguments
                evaluator
                opts.dim {mustBeNumeric, mustBeReal, mustBeInteger, mustBePositive} = []
                opts.s_range (1,2) {mustBeNumeric, mustBeReal, mustBeFinite} = [0, 1]
            end

            dim = copp.Path.required_evaluator_dim(opts.dim, "from_evaluator_2nd");
            callback = copp.Path.second_order_callback(evaluator);
            id = copp.internal.copp_mex( ...
                'path_from_evaluator_2nd', ...
                callback, ...
                double(dim), ...
                double(opts.s_range(1)), ...
                double(opts.s_range(2)));
            obj = copp.Path.from_native_id(id);
        end

        function obj = from_evaluator_3rd(evaluator, opts)
            %FROM_EVALUATOR_3RD Construct a callback-backed third-order path.
            %
            % OBJ = copp.Path.from_evaluator_3rd(F, dim=D, s_range=[A,B])
            % wraps a MATLAB function handle F. During native third-order path
            % evaluation the MEX gateway calls:
            %
            %   [q, dq, ddq, dddq] = F(s)
            %
            % The callback input s is always a 1-by-N row vector. All outputs
            % must be real finite D-by-N matrices. If the native library only
            % needs second-order derivatives, it may still call F and discard
            % dddq.
            %
            % OBJ = copp.Path.from_evaluator_3rd(E, ...) also accepts an
            % object or handle object E that implements:
            %
            %   [q, dq, ddq, dddq] = E.evaluate_up_to_3rd(s)
            %
            % If E also implements evaluate_up_to_2nd(s), native second-order
            % evaluation uses that lighter callback. Otherwise it falls back
            % to evaluate_up_to_3rd(s) and discards dddq, matching the C/Python
            % binding semantics.
            %
            % Name-value options:
            %   dim:
            %       Positive integer path dimension D.
            %   s_range:
            %       1-by-2 finite vector [s_min, s_max]. Defaults to [0, 1].
            arguments
                evaluator
                opts.dim {mustBeNumeric, mustBeReal, mustBeInteger, mustBePositive} = []
                opts.s_range (1,2) {mustBeNumeric, mustBeReal, mustBeFinite} = [0, 1]
            end

            dim = copp.Path.required_evaluator_dim(opts.dim, "from_evaluator_3rd");
            [callback3, callback2, has_callback2] = copp.Path.third_order_callbacks(evaluator);
            id = copp.internal.copp_mex( ...
                'path_from_evaluator_3rd', ...
                callback3, ...
                callback2, ...
                double(has_callback2), ...
                double(dim), ...
                double(opts.s_range(1)), ...
                double(opts.s_range(2)));
            obj = copp.Path.from_native_id(id);
        end

        function obj = from_symbolic(q_expr, opts)
            %FROM_SYMBOLIC Construct a third-order path from symbolic formulas.
            %
            % OBJ = copp.Path.from_symbolic(Q_EXPR) differentiates a
            % Symbolic Math Toolbox vector expression Q_EXPR with respect to
            % its only symbolic variable, then wraps the generated batch
            % evaluator with Path.from_evaluator_3rd().
            %
            % OBJ = copp.Path.from_symbolic(Q_EXPR, symbol=S) uses the
            % scalar symbolic variable S explicitly. Passing symbol= is
            % required when Q_EXPR contains zero variables or more than one
            % variable.
            %
            % Name-value options:
            %   symbol:
            %       Scalar sym variable used as the path parameter. If omitted,
            %       it is inferred only when symvar(Q_EXPR) has exactly one
            %       variable.
            %   s_range:
            %       1-by-2 finite vector [s_min, s_max]. Defaults to [0, 1].
            %
            % Q_EXPR must be a symbolic row or column vector. Matrix-valued
            % expressions are rejected so the path dimension is unambiguous.
            % The callback created by this constructor receives s as a 1-by-N
            % row vector and returns q/dq/ddq/dddq as dim-by-N double matrices.
            %
            % This constructor requires MATLAB Symbolic Math Toolbox. It
            % throws copp:MissingDependency when the toolbox is unavailable.
            %
            % Example:
            %   syms s
            %   q = [sin(s); cos(2*s); s + sym(1)/10*s^2];
            %   path = copp.Path.from_symbolic(q, symbol=s, s_range=[0, 1]);
            arguments
                q_expr
                opts.symbol = []
                opts.s_range (1,2) {mustBeNumeric, mustBeReal, mustBeFinite} = [0, 1]
            end

            evaluator = copp.internal.SymbolicParametricEvaluator(q_expr, opts.symbol);
            obj = copp.Path.from_evaluator_3rd( ...
                evaluator, ...
                dim=evaluator.dim, ...
                s_range=opts.s_range);
        end

        function obj = from_casadi(q_expr, opts)
            %FROM_CASADI Construct a third-order path from a CasADi expression.
            %
            % OBJ = copp.Path.from_casadi(Q_EXPR, symbol=S) differentiates
            % a scalar-parameter CasADi SX/MX vector expression Q_EXPR up to
            % third order with respect to scalar symbol S, then wraps the
            % generated batch evaluator with Path.from_evaluator_3rd().
            %
            % Name-value options:
            %   symbol:
            %       Required scalar CasADi SX or MX symbol used as the path
            %       parameter.
            %   s_range:
            %       1-by-2 finite vector [s_min, s_max]. Defaults to [0, 1].
            %
            % Q_EXPR must be a CasADi row or column vector. Matrix-valued
            % expressions are rejected so the path dimension is unambiguous.
            % The evaluator first tries CasADi Function.map(N) for batch
            % evaluation and falls back to a scalar loop if map evaluation is
            % not supported by the installed CasADi version.
            %
            % This constructor requires the CasADi MATLAB package on the MATLAB
            % path. It throws copp:MissingDependency when CasADi is
            % unavailable.
            %
            % Example:
            %   import casadi.*
            %   s = SX.sym('s');
            %   q = [sin(s); cos(2*s); s + 0.1*s^2];
            %   path = copp.Path.from_casadi(q, symbol=s, s_range=[0, 1]);
            arguments
                q_expr
                opts.symbol = []
                opts.s_range (1,2) {mustBeNumeric, mustBeReal, mustBeFinite} = [0, 1]
            end

            evaluator = copp.internal.CasadiParametricEvaluator(q_expr, opts.symbol);
            obj = copp.Path.from_evaluator_3rd( ...
                evaluator, ...
                dim=evaluator.dim, ...
                s_range=opts.s_range);
        end

        function obj = from_parametric(q_fn, opts)
            %FROM_PARAMETRIC Construct a third-order path from a MATLAB formula.
            %
            % OBJ = copp.Path.from_parametric(F, s_range=[A,B]) calls F
            % with a scalar copp.Jet3 path parameter. F must return a
            % numeric vector or a Jet3 vector. The returned value defines q(s),
            % and Jet3 automatically supplies dq/ds, d2q/ds2, and d3q/ds3.
            %
            % Name-value options:
            %   s_range:
            %       1-by-2 finite vector [s_min, s_max]. Defaults to [0, 1].
            %   dim:
            %       Optional positive integer path dimension. If omitted, the
            %       dimension is inferred by probing F at mean(s_range).
            %
            % The generated evaluator is batch-compatible with the native MEX
            % callback path. Internally it receives s as a 1-by-N row vector,
            % evaluates the scalar Jet3 formula once per sample, and returns
            % dim-by-N q/dq/ddq/dddq matrices.
            %
            % Example:
            %   path = copp.Path.from_parametric( ...
            %       @(s) [sin(s); cos(2*s); s + 0.1*s^2], ...
            %       s_range=[0, 1]);
            arguments
                q_fn (1,1) function_handle
                opts.s_range (1,2) {mustBeNumeric, mustBeReal, mustBeFinite} = [0, 1]
                opts.dim = []
            end

            [q0, ~, ~, ~] = copp.internal.Jet3ParametricEvaluator.evaluate_sample( ...
                q_fn, mean(double(opts.s_range)), "Path.from_parametric");
            dim = copp.Path.parametric_dim(opts.dim, numel(q0));

            evaluator = copp.internal.Jet3ParametricEvaluator(q_fn, dim);
            obj = copp.Path.from_evaluator_3rd( ...
                evaluator, ...
                dim=dim, ...
                s_range=opts.s_range);
        end
    end

    methods
        function q = evaluate_q(obj, s)
            %EVALUATE_Q Evaluate path position q(s).
            %
            % Q = evaluate_q(OBJ, S) returns a dim-by-N matrix containing the
            % path position at each sample in S, where N = numel(S). S may be
            % a row or column vector, but must be real and finite. The output
            % orientation is always dim-by-N, matching evaluate_up_to_2nd().
            arguments
                obj (1,1) copp.Path
                s {mustBeNumeric, mustBeReal, mustBeFinite}
            end

            [q, ~, ~] = obj.evaluate_up_to_2nd(s);
        end

        function varargout = evaluate_up_to_2nd(obj, s)
            %EVALUATE_UP_TO_2ND Evaluate q(s), dq/ds, and d2q/ds2.
            %
            % OUT = evaluate_up_to_2nd(OBJ, S) returns a struct with fields:
            %   q:
            %       dim-by-N path position matrix.
            %   dq:
            %       dim-by-N first derivative matrix.
            %   ddq:
            %       dim-by-N second derivative matrix.
            %
            % [Q, DQ, DDQ] = evaluate_up_to_2nd(OBJ, S) returns the same
            % matrices as separate outputs. S may be a 1-by-N or N-by-1 vector,
            % and all outputs are always dim-by-N.
            arguments
                obj (1,1) copp.Path
                s {mustBeNumeric, mustBeReal, mustBeFinite}
            end
            nargoutchk(0, 3);

            [q, dq, ddq] = copp.internal.copp_mex( ...
                'path_evaluate_up_to_2nd', ...
                obj.native_id_for_mex(), ...
                double(s));

            if nargout == 0
                return
            elseif nargout == 1
                varargout{1} = struct('q', q, 'dq', dq, 'ddq', ddq);
            else
                values = {q, dq, ddq};
                varargout = values(1:nargout);
            end
        end

        function varargout = evaluate_up_to_3rd(obj, s)
            %EVALUATE_UP_TO_3RD Evaluate q(s) and derivatives through d3q/ds3.
            %
            % OUT = evaluate_up_to_3rd(OBJ, S) returns a struct with fields:
            %   q:
            %       dim-by-N path position matrix.
            %   dq:
            %       dim-by-N first derivative matrix.
            %   ddq:
            %       dim-by-N second derivative matrix.
            %   dddq:
            %       dim-by-N third derivative matrix.
            %
            % [Q, DQ, DDQ, DDDQ] = evaluate_up_to_3rd(OBJ, S) returns the same
            % dim-by-N matrices as separate outputs, where N = numel(S).
            % Second-order-only evaluator paths raise copp:PathError for
            % this method.
            arguments
                obj (1,1) copp.Path
                s {mustBeNumeric, mustBeReal, mustBeFinite}
            end
            nargoutchk(0, 4);

            [q, dq, ddq, dddq] = copp.internal.copp_mex( ...
                'path_evaluate_up_to_3rd', ...
                obj.native_id_for_mex(), ...
                double(s));

            if nargout == 0
                return
            elseif nargout == 1
                varargout{1} = struct('q', q, 'dq', dq, 'ddq', ddq, 'dddq', dddq);
            else
                values = {q, dq, ddq, dddq};
                varargout = values(1:nargout);
            end
        end

        function delete(obj)
            %DELETE Release the native path handle during MATLAB cleanup.
            %
            % delete() is idempotent through release(); it is safe for MATLAB
            % to call it after the user has already called release().
            obj.release();
        end

        function released = release(obj)
            %RELEASE Explicitly release the native CoppPath handle.
            %
            % RELEASED = release(OBJ) returns true when this call released a
            % live native handle and false when OBJ was already released. After
            % release(), dependent metadata access and solver calls through
            % this object raise copp:InvalidHandle.
            released = false;
            if obj.native_id ~= uint64(0)
                released = copp.internal.copp_mex('release', obj.native_id);
                obj.native_id = uint64(0);
            end
        end

        function tf = is_valid(obj)
            %IS_VALID Return whether this object still owns a live native path.
            %
            % A false result means either release() has been called or the MEX
            % registry no longer contains this handle id.
            if obj.native_id == uint64(0)
                tf = false;
                return
            end
            tf = copp.internal.copp_mex('is_valid', obj.native_id, 'Path');
        end

        function value = get.dim(obj)
            %GET.DIM Query the native path dimension.
            value = copp.internal.copp_mex('path_dim', obj.native_id_for_mex());
        end

        function value = get.s_range(obj)
            %GET.S_RANGE Query the native path parameter range.
            value = copp.internal.copp_mex('path_s_range', obj.native_id_for_mex());
        end
    end

    methods (Static, Access = private)
        function dim = required_evaluator_dim(value, constructor_name)
            %REQUIRED_EVALUATOR_DIM Validate required dim= name-value input.
            if isempty(value) || ~isscalar(value)
                error( ...
                    "copp:InvalidArgument", ...
                    "%s requires a scalar positive integer dim= name-value argument.", ...
                    char(constructor_name));
            end
            dim = value;
        end

        function callback = second_order_callback(evaluator)
            %SECOND_ORDER_CALLBACK Normalize function-handle/object evaluator.
            if isa(evaluator, 'function_handle')
                callback = evaluator;
                return
            end

            if copp.Path.has_public_method(evaluator, 'evaluate_up_to_2nd')
                callback = @(s) evaluator.evaluate_up_to_2nd(s);
                return
            end

            error( ...
                "copp:InvalidArgument", ...
                "from_evaluator_2nd requires a function handle or an object with evaluate_up_to_2nd(s).");
        end

        function [callback3, callback2, has_callback2] = third_order_callbacks(evaluator)
            %THIRD_ORDER_CALLBACKS Normalize third-order evaluator callbacks.
            if isa(evaluator, 'function_handle')
                callback3 = evaluator;
                callback2 = evaluator;
                has_callback2 = false;
                return
            end

            if ~copp.Path.has_public_method(evaluator, 'evaluate_up_to_3rd')
                error( ...
                    "copp:InvalidArgument", ...
                    "from_evaluator_3rd requires a function handle or an object with evaluate_up_to_3rd(s).");
            end

            callback3 = @(s) evaluator.evaluate_up_to_3rd(s);
            has_callback2 = copp.Path.has_public_method(evaluator, 'evaluate_up_to_2nd');
            if has_callback2
                callback2 = @(s) evaluator.evaluate_up_to_2nd(s);
            else
                callback2 = callback3;
            end
        end

        function tf = has_public_method(evaluator, method_name)
            %HAS_PUBLIC_METHOD Return true when evaluator exposes method_name.
            try
                tf = ismethod(evaluator, method_name);
            catch
                tf = false;
            end
        end

        function dim = parametric_dim(value, inferred_dim)
            %PARAMETRIC_DIM Validate optional dim= for Path.from_parametric.
            if inferred_dim <= 0
                error("copp:InvalidArgument", ...
                    "Path.from_parametric formula must return a non-empty vector.");
            end

            if isempty(value)
                dim = inferred_dim;
                return
            end

            validateattributes(value, {'numeric'}, ...
                {'real', 'finite', 'integer', 'positive', 'scalar'}, ...
                'Path.from_parametric', 'dim');
            dim = double(value);
            if dim ~= inferred_dim
                error("copp:InvalidArgument", ...
                    "Path.from_parametric dim=%d does not match formula output dimension %d.", ...
                    dim, inferred_dim);
            end
        end
    end

    methods (Hidden)
        function id = native_id_for_mex(obj)
            %NATIVE_ID_FOR_MEX Return the registry id after validating liveness.
            %
            % This hidden method is for other MATLAB package functions that
            % need to pass a Path into the private MEX gateway. It deliberately
            % returns the registry id, not a raw pointer.
            if obj.native_id == uint64(0) || ~obj.is_valid()
                error("copp:InvalidHandle", "Path native handle is not valid.");
            end
            id = obj.native_id;
        end
    end

    methods (Static, Hidden)
        function obj = from_native_id(native_id)
            %FROM_NATIVE_ID Wrap a MEX-created path registry id.
            %
            % The MEX gateway creates native resources and returns opaque ids.
            % This hidden constructor is the single internal path from such an
            % id to a MATLAB owner object.
            obj = copp.Path(native_id);
        end
    end
end
