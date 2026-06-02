classdef Jet3ParametricEvaluator
    %JET3PARAMETRICEVALUATOR Batch adapter for Path.from_parametric.
    %
    % Native callback input s is normalized to a 1-by-N row vector. Public
    % callback outputs q, dq, ddq, and dddq are dim-by-N matrices.

    properties (Access = private)
        q_fn
        dim (1,1) double
    end

    methods
        function obj = Jet3ParametricEvaluator(q_fn, dim)
            obj.q_fn = q_fn;
            obj.dim = double(dim);
        end

        function [q, dq, ddq] = evaluate_up_to_2nd(obj, s)
            %EVALUATE_UP_TO_2ND Return q and derivatives through second order.
            [q, dq, ddq, ~] = obj.evaluate_up_to_3rd(s);
        end

        function [q, dq, ddq, dddq] = evaluate_up_to_3rd(obj, s)
            %EVALUATE_UP_TO_3RD Return q and derivatives through third order.
            [q, dq, ddq, dddq] = copp.internal.Jet3ParametricEvaluator.evaluate_batch( ...
                obj.q_fn, s, obj.dim);
        end
    end

    methods (Static)
        function [q, dq, ddq, dddq] = evaluate_batch(q_fn, s, dim)
            %EVALUATE_BATCH Adapt a scalar Jet3 formula to batch API.
            %
            % S may arrive as a row or column vector and is normalized to
            % 1-by-N. Returned derivative arrays are dim-by-N.
            s = reshape(double(s), 1, []);
            n = numel(s);
            q = zeros(dim, n);
            dq = zeros(dim, n);
            ddq = zeros(dim, n);
            dddq = zeros(dim, n);

            for k = 1:n
                [qk, dqk, ddqk, dddqk] = copp.internal.Jet3ParametricEvaluator.evaluate_sample( ...
                    q_fn, s(k), "Path.from_parametric");
                if numel(qk) ~= dim
                    error("copp:InvalidArgument", ...
                        "Path.from_parametric formula returned dimension %d at sample %d, expected %d.", ...
                        numel(qk), k, dim);
                end
                q(:, k) = qk;
                dq(:, k) = dqk;
                ddq(:, k) = ddqk;
                dddq(:, k) = dddqk;
            end
        end

        function [q, dq, ddq, dddq] = evaluate_sample(q_fn, s_value, context)
            %EVALUATE_SAMPLE Evaluate and validate one Jet3 sample.
            %
            % The user formula returns a vector with dim elements for one
            % scalar sample. Components are normalized to dim-by-1 columns.
            try
                raw = q_fn(copp.Jet3.seed(s_value));
            catch err
                error("copp:InvalidArgument", ...
                    "%s callback failed at s=%.17g: %s", ...
                    char(context), double(s_value), err.message);
            end

            try
                [q, dq, ddq, dddq] = copp.Jet3.components(raw);
            catch err
                error("copp:InvalidArgument", ...
                    "%s callback returned an unsupported value: %s", ...
                    char(context), err.message);
            end

            if ~(isvector(q) || isscalar(q))
                error("copp:InvalidArgument", ...
                    "%s callback must return a vector, not a matrix.", char(context));
            end
            if ~isequal(size(q), size(dq), size(ddq), size(dddq))
                error("copp:InvalidArgument", ...
                    "%s callback returned inconsistent Jet3 component shapes.", char(context));
            end

            q = double(q(:));
            dq = double(dq(:));
            ddq = double(ddq(:));
            dddq = double(dddq(:));
            if ~all(isfinite([q; dq; ddq; dddq]))
                error("copp:InvalidArgument", ...
                    "%s callback returned NaN or Inf.", char(context));
            end
        end
    end
end
