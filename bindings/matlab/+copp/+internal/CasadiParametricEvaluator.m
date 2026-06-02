classdef CasadiParametricEvaluator < handle
    %CASADIPARAMETRICEVALUATOR CasADi-backed third-order path evaluator.
    %
    % This internal helper converts a CasADi SX/MX vector expression q(s) into
    % batch callbacks compatible with copp.Path.from_evaluator_3rd. Public
    % users should construct it through copp.Path.from_casadi().
    %
    % Evaluation first tries CasADi Function.map(N), cached per batch size. If
    % the installed CasADi MATLAB package cannot map the function in this form,
    % evaluation falls back to a scalar loop over the samples. Callback input s
    % is normalized to 1-by-N and all derivative outputs are dim-by-N.

    properties (SetAccess = private)
        %DIM Path dimension, equal to q_expr.numel().
        dim (1,1) double
    end

    properties (Access = private)
        %FUNC Scalar CasADi Function returning q, dq, ddq, and dddq.
        func

        %MAPPED_FUNCTIONS containers.Map from batch size to mapped Function.
        mapped_functions
    end

    methods
        function obj = CasadiParametricEvaluator(q_expr, symbol)
            %CASADIPARAMETRICEVALUATOR Build CasADi derivative function.
            if ~copp.internal.CasadiParametricEvaluator.has_casadi()
                error( ...
                    "copp:MissingDependency", ...
                    "Path.from_casadi requires the CasADi MATLAB package on the MATLAB path.");
            end

            if isempty(symbol)
                error( ...
                    "copp:InvalidArgument", ...
                    "Path.from_casadi requires symbol= to be a scalar CasADi SX or MX symbol.");
            end
            symbol_numel = copp.internal.CasadiParametricEvaluator.casadi_numel(symbol, "symbol");
            if symbol_numel ~= 1
                error( ...
                    "copp:InvalidArgument", ...
                    "Path.from_casadi requires symbol= to contain exactly one scalar variable.");
            end

            dim = copp.internal.CasadiParametricEvaluator.casadi_numel(q_expr, "q_expr");
            if dim < 1
                error( ...
                    "copp:InvalidArgument", ...
                    "Path.from_casadi requires q_expr to contain at least one expression.");
            end
            [rows, cols] = copp.internal.CasadiParametricEvaluator.casadi_size(q_expr, "q_expr");
            if rows * cols ~= dim || ~(rows == 1 || cols == 1)
                error( ...
                    "copp:InvalidArgument", ...
                    "Path.from_casadi requires q_expr to be a CasADi row or column vector, not a matrix.");
            end

            q_vec = copp.internal.CasadiParametricEvaluator.casadi_reshape(q_expr, dim, 1);
            dq_vec = copp.internal.CasadiParametricEvaluator.casadi_jacobian(q_vec, symbol, "dq");
            ddq_vec = copp.internal.CasadiParametricEvaluator.casadi_jacobian(dq_vec, symbol, "ddq");
            dddq_vec = copp.internal.CasadiParametricEvaluator.casadi_jacobian(ddq_vec, symbol, "dddq");

            try
                obj.func = casadi.Function( ...
                    'copp_parametric_path', ...
                    {symbol}, ...
                    {q_vec, dq_vec, ddq_vec, dddq_vec});
            catch err
                error( ...
                    "copp:InvalidArgument", ...
                    "Path.from_casadi could not create a CasADi Function: %s", ...
                    err.message);
            end

            obj.dim = double(dim);
            obj.mapped_functions = containers.Map('KeyType', 'char', 'ValueType', 'any');
        end

        function q = evaluate_q(obj, s)
            %EVALUATE_Q Evaluate q(s) for a 1-by-N sample row.
            [q, ~, ~, ~] = obj.evaluate_all(s);
        end

        function [q, dq, ddq] = evaluate_up_to_2nd(obj, s)
            %EVALUATE_UP_TO_2ND Evaluate q(s), dq/ds, and d2q/ds2.
            %
            % S is treated as 1-by-N; outputs are dim-by-N.
            [q, dq, ddq, ~] = obj.evaluate_all(s);
        end

        function [q, dq, ddq, dddq] = evaluate_up_to_3rd(obj, s)
            %EVALUATE_UP_TO_3RD Evaluate q(s) and derivatives through order 3.
            %
            % S is treated as 1-by-N; outputs are dim-by-N.
            [q, dq, ddq, dddq] = obj.evaluate_all(s);
        end
    end

    methods (Access = private)
        function [q, dq, ddq, dddq] = evaluate_all(obj, s)
            %EVALUATE_ALL Evaluate all derivative orders in dim-by-N layout.
            s = copp.internal.CasadiParametricEvaluator.normalize_s(s);
            n = numel(s);
            if n == 0
                empty = zeros(obj.dim, 0);
                q = empty;
                dq = empty;
                ddq = empty;
                dddq = empty;
                return
            end

            try
                [q, dq, ddq, dddq] = obj.evaluate_mapped(s, n);
                return
            catch
                % Some CasADi MATLAB builds do not accept the same mapped-call
                % shape. The scalar loop keeps the public constructor usable.
            end

            [q, dq, ddq, dddq] = obj.evaluate_scalar_loop(s, n);
        end

        function [q, dq, ddq, dddq] = evaluate_mapped(obj, s, n)
            %EVALUATE_MAPPED Evaluate with cached Function.map(N).
            mapped = obj.mapped_function(n);
            [q_raw, dq_raw, ddq_raw, dddq_raw] = mapped(s);
            q = copp.internal.CasadiParametricEvaluator.normalize_matrix(q_raw, obj.dim, n, "q");
            dq = copp.internal.CasadiParametricEvaluator.normalize_matrix(dq_raw, obj.dim, n, "dq");
            ddq = copp.internal.CasadiParametricEvaluator.normalize_matrix(ddq_raw, obj.dim, n, "ddq");
            dddq = copp.internal.CasadiParametricEvaluator.normalize_matrix(dddq_raw, obj.dim, n, "dddq");
        end

        function [q, dq, ddq, dddq] = evaluate_scalar_loop(obj, s, n)
            %EVALUATE_SCALAR_LOOP Evaluate the scalar CasADi Function per sample.
            q = zeros(obj.dim, n);
            dq = zeros(obj.dim, n);
            ddq = zeros(obj.dim, n);
            dddq = zeros(obj.dim, n);

            for sample_index = 1:n
                try
                    [q_raw, dq_raw, ddq_raw, dddq_raw] = obj.func(s(sample_index));
                catch err
                    error( ...
                        "copp:InvalidArgument", ...
                        "CasADi path evaluator failed at sample %d: %s", ...
                        sample_index, ...
                        err.message);
                end

                q(:, sample_index) = ...
                    copp.internal.CasadiParametricEvaluator.normalize_matrix(q_raw, obj.dim, 1, "q");
                dq(:, sample_index) = ...
                    copp.internal.CasadiParametricEvaluator.normalize_matrix(dq_raw, obj.dim, 1, "dq");
                ddq(:, sample_index) = ...
                    copp.internal.CasadiParametricEvaluator.normalize_matrix(ddq_raw, obj.dim, 1, "ddq");
                dddq(:, sample_index) = ...
                    copp.internal.CasadiParametricEvaluator.normalize_matrix(dddq_raw, obj.dim, 1, "dddq");
            end
        end

        function mapped = mapped_function(obj, n)
            %MAPPED_FUNCTION Return cached Function.map(N).
            key = sprintf('%d', n);
            if ~isKey(obj.mapped_functions, key)
                obj.mapped_functions(key) = obj.func.map(n);
            end
            mapped = obj.mapped_functions(key);
        end
    end

    methods (Static, Access = private)
        function tf = has_casadi()
            %HAS_CASADI Return true when the CasADi package is visible.
            tf = exist('casadi.Function', 'class') == 8 || ...
                exist('casadi.Function', 'file') == 2;
        end

        function count = casadi_numel(value, name)
            %CASADI_NUMEL Read CasADi symbolic element count.
            try
                count = double(value.numel());
                return
            catch method_err
                try
                    count = double(numel(value));
                    return
                catch err
                    error( ...
                        "copp:InvalidArgument", ...
                        "Path.from_casadi could not read numel(%s): %s", ...
                        char(name), ...
                        sprintf('%s %s', err.message, method_err.message));
                end
            end
        end

        function [rows, cols] = casadi_size(value, name)
            %CASADI_SIZE Read CasADi symbolic matrix shape.
            try
                rows = double(value.size1());
                cols = double(value.size2());
                return
            catch method_err
                try
                    sz = size(value);
                    rows = double(sz(1));
                    if numel(sz) >= 2
                        cols = double(sz(2));
                    else
                        cols = 1;
                    end
                    return
                catch err
                    error( ...
                        "copp:InvalidArgument", ...
                        "Path.from_casadi could not read size(%s): %s", ...
                        char(name), ...
                        sprintf('%s %s', err.message, method_err.message));
                end
            end
        end

        function value = casadi_reshape(value, rows, cols)
            %CASADI_RESHAPE Reshape through CasADi or overloaded MATLAB reshape.
            try
                value = casadi.reshape(value, rows, cols);
                return
            catch package_err
                try
                    value = reshape(value, rows, cols);
                    return
                catch reshape_err
                    try
                        value = value(:);
                        return
                    catch colon_err
                        error( ...
                            "copp:InvalidArgument", ...
                            "Path.from_casadi could not reshape q_expr into a column vector: %s", ...
                            sprintf('%s %s %s', package_err.message, reshape_err.message, colon_err.message));
                    end
                end
            end
        end

        function value = casadi_jacobian(expr, symbol, name)
            %CASADI_JACOBIAN Differentiate one derivative order.
            try
                value = casadi.jacobian(expr, symbol);
                return
            catch package_err
                try
                    value = jacobian(expr, symbol);
                    return
                catch err
                    error( ...
                        "copp:InvalidArgument", ...
                        "Path.from_casadi could not compute %s with CasADi jacobian: %s", ...
                        char(name), ...
                        sprintf('%s %s', package_err.message, err.message));
                end
            end
        end

        function s = normalize_s(s)
            %NORMALIZE_S Convert callback input into a finite 1-by-N row.
            s = double(s);
            s = reshape(s, 1, []);
            if any(~isfinite(s))
                error( ...
                    "copp:InvalidArgument", ...
                    "CasADi path evaluator received non-finite s samples.");
            end
        end

        function matrix = normalize_matrix(value, dim, n, name)
            %NORMALIZE_MATRIX Convert CasADi output into dim-by-N double form.
            try
                value = full(value);
            catch
                % Numeric outputs already support double conversion.
            end

            try
                value = double(value);
            catch err
                error( ...
                    "copp:InvalidArgument", ...
                    "CasADi path %s output could not be converted to double: %s", ...
                    char(name), ...
                    err.message);
            end

            if isequal(size(value), [dim, n])
                matrix = value;
            elseif n == 1 && isvector(value) && numel(value) == dim
                matrix = reshape(value, dim, 1);
            elseif dim == 1 && isvector(value) && numel(value) == n
                matrix = reshape(value, 1, n);
            elseif isequal(size(value), [n, dim]) && dim ~= n
                matrix = value.';
            elseif numel(value) == dim * n
                matrix = reshape(value, dim, n);
            else
                error( ...
                    "copp:InvalidArgument", ...
                    "CasADi path %s output has shape %s; expected %d-by-%d.", ...
                    char(name), ...
                    mat2str(size(value)), ...
                    dim, ...
                    n);
            end

            if any(~isfinite(matrix(:)))
                error( ...
                    "copp:InvalidArgument", ...
                    "CasADi path %s output contains non-finite samples.", ...
                    char(name));
            end
        end
    end
end
