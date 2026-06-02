classdef SymbolicParametricEvaluator < handle
    %SYMBOLICPARAMETRICEVALUATOR Symbolic Math Toolbox path evaluator.
    %
    % This internal helper converts a symbolic vector expression q(s) into
    % batch callbacks compatible with copp.Path.from_evaluator_3rd. Public
    % users should construct it through copp.Path.from_symbolic().
    %
    % The evaluator keeps one generated matlabFunction handle per component
    % and derivative order. This component-wise layout handles constant
    % expressions robustly because scalar outputs can be broadcast to the
    % incoming sample vector. Callback input s is normalized to 1-by-N and all
    % derivative outputs are dim-by-N.

    properties (SetAccess = private)
        %DIM Path dimension, equal to numel(q_expr(:)).
        dim (1,1) double
    end

    properties (Access = private)
        %COMPONENT_FUNCTIONS dim-by-4 cell array for q, dq, ddq, dddq.
        component_functions
    end

    methods
        function obj = SymbolicParametricEvaluator(q_expr, symbol)
            %SYMBOLICPARAMETRICEVALUATOR Build generated derivative callbacks.
            if ~copp.internal.SymbolicParametricEvaluator.has_symbolic_toolbox()
                error( ...
                    "copp:MissingDependency", ...
                    "Path.from_symbolic requires MATLAB Symbolic Math Toolbox.");
            end

            if ~isa(q_expr, 'sym')
                error( ...
                    "copp:InvalidArgument", ...
                    "Path.from_symbolic requires q_expr to be a symbolic vector expression.");
            end
            if isempty(q_expr) || ~isvector(q_expr)
                error( ...
                    "copp:InvalidArgument", ...
                    "Path.from_symbolic requires q_expr to be a nonempty symbolic row or column vector.");
            end

            symbol = copp.internal.SymbolicParametricEvaluator.normalize_symbol(q_expr, symbol);
            q_vec = q_expr(:);
            obj.dim = double(numel(q_vec));

            exprs = cell(1, 4);
            exprs{1} = q_vec;
            for order = 1:3
                exprs{order + 1} = diff(q_vec, symbol, order);
            end

            fns = cell(obj.dim, 4);
            for order_index = 1:4
                for component_index = 1:obj.dim
                    fns{component_index, order_index} = matlabFunction( ...
                        exprs{order_index}(component_index), ...
                        'Vars', symbol);
                end
            end
            obj.component_functions = fns;
        end

        function q = evaluate_q(obj, s)
            %EVALUATE_Q Evaluate q(s) for a 1-by-N sample row.
            q = obj.evaluate_order(s, 1, "q");
        end

        function [q, dq, ddq] = evaluate_up_to_2nd(obj, s)
            %EVALUATE_UP_TO_2ND Evaluate q(s), dq/ds, and d2q/ds2.
            %
            % S is treated as 1-by-N; outputs are dim-by-N.
            q = obj.evaluate_order(s, 1, "q");
            dq = obj.evaluate_order(s, 2, "dq");
            ddq = obj.evaluate_order(s, 3, "ddq");
        end

        function [q, dq, ddq, dddq] = evaluate_up_to_3rd(obj, s)
            %EVALUATE_UP_TO_3RD Evaluate q(s) and derivatives through order 3.
            %
            % S is treated as 1-by-N; outputs are dim-by-N.
            q = obj.evaluate_order(s, 1, "q");
            dq = obj.evaluate_order(s, 2, "dq");
            ddq = obj.evaluate_order(s, 3, "ddq");
            dddq = obj.evaluate_order(s, 4, "dddq");
        end
    end

    methods (Access = private)
        function matrix = evaluate_order(obj, s, order_index, name)
            %EVALUATE_ORDER Evaluate one derivative order into dim-by-N form.
            s = copp.internal.SymbolicParametricEvaluator.normalize_s(s);
            n = numel(s);
            matrix = zeros(obj.dim, n);

            for component_index = 1:obj.dim
                fn = obj.component_functions{component_index, order_index};
                value = fn(s);
                matrix(component_index, :) = ...
                    copp.internal.SymbolicParametricEvaluator.normalize_component( ...
                        value, ...
                        n, ...
                        name, ...
                        component_index);
            end
        end
    end

    methods (Static, Access = private)
        function tf = has_symbolic_toolbox()
            %HAS_SYMBOLIC_TOOLBOX Return true when Symbolic Math Toolbox exists.
            tf = exist('sym', 'class') == 8 || exist('sym', 'file') == 2;
        end

        function symbol = normalize_symbol(q_expr, symbol)
            %NORMALIZE_SYMBOL Validate or infer the scalar symbolic variable.
            if isempty(symbol)
                vars = symvar(q_expr);
                if numel(vars) ~= 1
                    error( ...
                        "copp:InvalidArgument", ...
                        "Path.from_symbolic can infer symbol only when q_expr contains exactly one symbolic variable; pass symbol=... explicitly.");
                end
                symbol = vars(1);
                return
            end

            if ~isa(symbol, 'sym') || numel(symbol) ~= 1
                error( ...
                    "copp:InvalidArgument", ...
                    "Path.from_symbolic requires symbol= to be a scalar symbolic variable.");
            end
        end

        function s = normalize_s(s)
            %NORMALIZE_S Convert callback input into a finite 1-by-N row.
            s = double(s);
            s = reshape(s, 1, []);
            if any(~isfinite(s))
                error( ...
                    "copp:InvalidArgument", ...
                    "Symbolic path evaluator received non-finite s samples.");
            end
        end

        function row = normalize_component(value, n, name, component_index)
            %NORMALIZE_COMPONENT Broadcast and validate one component output.
            try
                value = double(value);
            catch err
                error( ...
                    "copp:InvalidArgument", ...
                    "Symbolic path %s component %d could not be converted to double: %s", ...
                    char(name), ...
                    component_index, ...
                    err.message);
            end

            if n == 0
                if isempty(value) || isscalar(value)
                    row = zeros(1, 0);
                    return
                end
            end

            if isscalar(value)
                row = repmat(value, 1, n);
            else
                row = reshape(value, 1, []);
            end

            if numel(row) ~= n
                error( ...
                    "copp:InvalidArgument", ...
                    "Symbolic path %s component %d returned %d samples; expected %d.", ...
                    char(name), ...
                    component_index, ...
                    numel(row), ...
                    n);
            end
            if any(~isfinite(row))
                error( ...
                    "copp:InvalidArgument", ...
                    "Symbolic path %s component %d returned non-finite samples.", ...
                    char(name), ...
                    component_index);
            end
        end
    end
end
