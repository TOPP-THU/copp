classdef Jet3
    %JET3 Scalar third-order automatic-differentiation value.
    %
    % Jet3 stores a real value and its first three derivatives with respect to
    % one scalar path parameter. Path.from_parametric passes a seeded Jet3 into
    % a user formula, then extracts q, dq/ds, d2q/ds2, and d3q/ds3 from the
    % returned Jet3 vector.
    %
    % Supported operations cover the first lightweight MATLAB formula surface:
    % +, -, .*, *, ./, /, .^, ^, sin, cos, exp, log, and sqrt. The class also
    % supports vertical and horizontal concatenation so formulas may return
    % vectors such as [sin(s); cos(2*s); s^2]. Path.from_parametric treats the
    % formula return as a vector with dim elements and stores each sample as a
    % dim-by-1 column inside the final dim-by-N batch output; matrix-valued
    % formulas are rejected.

    properties (SetAccess = private)
        %VALUE Function value.
        value double = 0

        %D1 First derivative.
        d1 double = 0

        %D2 Second derivative.
        d2 double = 0

        %D3 Third derivative.
        d3 double = 0
    end

    methods
        function obj = Jet3(value, d1, d2, d3)
            %JET3 Construct a value with explicit derivatives.
            if nargin == 0
                return
            end
            if nargin < 2
                d1 = zeros(size(value));
            end
            if nargin < 3
                d2 = zeros(size(value));
            end
            if nargin < 4
                d3 = zeros(size(value));
            end

            obj.value = double(value);
            obj.d1 = double(d1);
            obj.d2 = double(d2);
            obj.d3 = double(d3);
        end

        function y = double(x)
            %DOUBLE Return the numeric value component.
            y = x.value;
        end

        function y = uplus(x)
            y = x;
        end

        function y = uminus(x)
            x = copp.Jet3.coerce(x);
            y = copp.Jet3(-x.value, -x.d1, -x.d2, -x.d3);
        end

        function y = plus(a, b)
            [a, b] = copp.Jet3.coerce_pair(a, b);
            y = copp.Jet3(a.value + b.value, a.d1 + b.d1, a.d2 + b.d2, a.d3 + b.d3);
        end

        function y = minus(a, b)
            [a, b] = copp.Jet3.coerce_pair(a, b);
            y = copp.Jet3(a.value - b.value, a.d1 - b.d1, a.d2 - b.d2, a.d3 - b.d3);
        end

        function y = times(a, b)
            [a, b] = copp.Jet3.coerce_pair(a, b);
            y = copp.Jet3( ...
                a.value .* b.value, ...
                a.d1 .* b.value + a.value .* b.d1, ...
                a.d2 .* b.value + 2 .* a.d1 .* b.d1 + a.value .* b.d2, ...
                a.d3 .* b.value + 3 .* a.d2 .* b.d1 + 3 .* a.d1 .* b.d2 + a.value .* b.d3);
        end

        function y = mtimes(a, b)
            [a, b] = copp.Jet3.coerce_pair(a, b);
            if isscalar(a.value) || isscalar(b.value)
                y = times(a, b);
                return
            end

            y = copp.Jet3( ...
                a.value * b.value, ...
                a.d1 * b.value + a.value * b.d1, ...
                a.d2 * b.value + 2 .* (a.d1 * b.d1) + a.value * b.d2, ...
                a.d3 * b.value + 3 .* (a.d2 * b.d1) + 3 .* (a.d1 * b.d2) + a.value * b.d3);
        end

        function y = rdivide(a, b)
            [a, b] = copp.Jet3.coerce_pair(a, b);
            y = times(a, b.reciprocal());
        end

        function y = mrdivide(a, b)
            [a, b] = copp.Jet3.coerce_pair(a, b);
            if isscalar(b.value)
                y = rdivide(a, b);
                return
            end

            error("copp:UnsupportedOperation", ...
                "Jet3 matrix right-division is not supported beyond scalar divisors.");
        end

        function y = power(a, b)
            if isa(a, 'copp.Jet3') && ~isa(b, 'copp.Jet3')
                a = copp.Jet3.coerce(a);
                if isscalar(b) && isfinite(b) && b == floor(b) && b >= 0
                    y = copp.Jet3.constant(ones(size(a.value)));
                    for k = 1:double(b)
                        y = y .* a;
                    end
                    return
                end
                if all(b == 0, 'all')
                    y = copp.Jet3.constant(ones(size(a.value)));
                    return
                end
                y = copp.Jet3.compose_unary( ...
                    a, ...
                    a.value .^ b, ...
                    b .* a.value .^ (b - 1), ...
                    b .* (b - 1) .* a.value .^ (b - 2), ...
                    b .* (b - 1) .* (b - 2) .* a.value .^ (b - 3));
                return
            end

            [a, b] = copp.Jet3.coerce_pair(a, b);
            y = exp(b .* log(a));
        end

        function y = mpower(a, b)
            y = power(a, b);
        end

        function y = sin(x)
            x = copp.Jet3.coerce(x);
            y = copp.Jet3.compose_unary(x, sin(x.value), cos(x.value), -sin(x.value), -cos(x.value));
        end

        function y = cos(x)
            x = copp.Jet3.coerce(x);
            y = copp.Jet3.compose_unary(x, cos(x.value), -sin(x.value), -cos(x.value), sin(x.value));
        end

        function y = exp(x)
            x = copp.Jet3.coerce(x);
            ex = exp(x.value);
            y = copp.Jet3.compose_unary(x, ex, ex, ex, ex);
        end

        function y = log(x)
            x = copp.Jet3.coerce(x);
            y = copp.Jet3.compose_unary( ...
                x, ...
                log(x.value), ...
                1 ./ x.value, ...
                -1 ./ (x.value .^ 2), ...
                2 ./ (x.value .^ 3));
        end

        function y = sqrt(x)
            x = copp.Jet3.coerce(x);
            root = sqrt(x.value);
            y = copp.Jet3.compose_unary( ...
                x, ...
                root, ...
                1 ./ (2 .* root), ...
                -1 ./ (4 .* x.value .^ 1.5), ...
                3 ./ (8 .* x.value .^ 2.5));
        end

        function y = transpose(x)
            x = copp.Jet3.coerce(x);
            y = copp.Jet3(x.value.', x.d1.', x.d2.', x.d3.');
        end

        function y = ctranspose(x)
            x = copp.Jet3.coerce(x);
            y = transpose(x);
        end

        function y = vertcat(varargin)
            y = copp.Jet3.cat_components(1, varargin{:});
        end

        function y = horzcat(varargin)
            y = copp.Jet3.cat_components(2, varargin{:});
        end
    end

    methods (Access = private)
        function y = reciprocal(x)
            y = copp.Jet3.compose_unary( ...
                x, ...
                1 ./ x.value, ...
                -1 ./ (x.value .^ 2), ...
                2 ./ (x.value .^ 3), ...
                -6 ./ (x.value .^ 4));
        end
    end

    methods (Static)
        function obj = constant(value)
            %CONSTANT Construct a constant Jet3 with zero derivatives.
            obj = copp.Jet3(value, zeros(size(value)), zeros(size(value)), zeros(size(value)));
        end

        function obj = seed(value)
            %SEED Construct the scalar path parameter jet d/ds value = 1.
            obj = copp.Jet3(value, ones(size(value)), zeros(size(value)), zeros(size(value)));
        end
    end

    methods (Static, Hidden)
        function [value, d1, d2, d3] = components(input)
            %COMPONENTS Extract numeric components from a Jet3 or numeric value.
            if isa(input, 'copp.Jet3')
                if isscalar(input)
                    value = input.value;
                    d1 = input.d1;
                    d2 = input.d2;
                    d3 = input.d3;
                    return
                end

                value = reshape(arrayfun(@(x) x.value, input), size(input));
                d1 = reshape(arrayfun(@(x) x.d1, input), size(input));
                d2 = reshape(arrayfun(@(x) x.d2, input), size(input));
                d3 = reshape(arrayfun(@(x) x.d3, input), size(input));
                return
            end

            if isnumeric(input)
                value = double(input);
                d1 = zeros(size(value));
                d2 = zeros(size(value));
                d3 = zeros(size(value));
                return
            end

            error("copp:InvalidArgument", ...
                "Path.from_parametric formulas must return numeric values or copp.Jet3 values.");
        end
    end

    methods (Static, Access = private)
        function obj = coerce(value)
            if isa(value, 'copp.Jet3')
                obj = value;
            elseif isnumeric(value)
                obj = copp.Jet3.constant(value);
            else
                error("copp:InvalidArgument", ...
                    "Jet3 operations support numeric values and copp.Jet3 values.");
            end
        end

        function [a, b] = coerce_pair(a, b)
            a = copp.Jet3.coerce(a);
            b = copp.Jet3.coerce(b);
        end

        function y = compose_unary(x, g0, g1, g2, g3)
            y = copp.Jet3( ...
                g0, ...
                g1 .* x.d1, ...
                g2 .* (x.d1 .^ 2) + g1 .* x.d2, ...
                g3 .* (x.d1 .^ 3) + 3 .* g2 .* x.d1 .* x.d2 + g1 .* x.d3);
        end

        function y = cat_components(dim, varargin)
            values = cell(1, nargin - 1);
            d1s = cell(1, nargin - 1);
            d2s = cell(1, nargin - 1);
            d3s = cell(1, nargin - 1);
            for k = 1:(nargin - 1)
                item = copp.Jet3.coerce(varargin{k});
                values{k} = item.value;
                d1s{k} = item.d1;
                d2s{k} = item.d2;
                d3s{k} = item.d3;
            end
            y = copp.Jet3(cat(dim, values{:}), cat(dim, d1s{:}), cat(dim, d2s{:}), cat(dim, d3s{:}));
        end
    end
end
