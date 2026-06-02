classdef ReachSet
    %REACHSET TOPP2 reachable a(s) interval result.
    %
    % ReachSet stores one feasible interval per station:
    %   a_min(k) <= a(k) <= a_max(k)
    % where a = (ds/dt)^2. Bounds must be finite, nonnegative, and ordered up
    % to a small numerical tolerance. Inputs may be row or column vectors, but
    % a_min and a_max are stored as MATLAB-owned N-by-1 double column vectors.

    properties (SetAccess = private)
        %A_MAX Upper reachable bound at each station as an N-by-1 vector.
        a_max

        %A_MIN Lower reachable bound at each station as an N-by-1 vector.
        a_min
    end

    properties (Dependent)
        %LEN Number of station intervals stored in this result.
        len
    end

    methods
        function obj = ReachSet(a_max, a_min)
            %REACHSET Construct a MATLAB-owned reachable-set result.
            %
            % A_MAX and A_MIN may be 1-by-N or N-by-1, but both must contain
            % the same number of station samples. Stored properties are N-by-1.
            % Tiny negative values caused by roundoff are clamped to zero.
            arguments
                a_max {mustBeNumeric, mustBeReal, mustBeVector}
                a_min {mustBeNumeric, mustBeReal, mustBeVector}
            end

            if numel(a_max) ~= numel(a_min)
                error("copp:InvalidArgument", ...
                    "a_max and a_min must have the same number of elements.");
            end

            a_max_col = reshape(double(a_max), [], 1);
            a_min_col = reshape(double(a_min), [], 1);
            if any(~isfinite(a_max_col)) || any(~isfinite(a_min_col))
                error("copp:InvalidArgument", ...
                    "ReachSet2 a_max and a_min must be finite.");
            end
            tol = 1.0e-12;
            if any(a_max_col < -tol) || any(a_min_col < -tol)
                error("copp:InvalidArgument", ...
                    "ReachSet2 a_max and a_min must be nonnegative because a=(ds/dt)^2.");
            end

            a_max_col(a_max_col < 0.0) = 0.0;
            a_min_col(a_min_col < 0.0) = 0.0;

            if any(a_min_col > a_max_col + tol)
                error("copp:InvalidArgument", ...
                    "ReachSet2 bounds must satisfy a_min(k) <= a_max(k).");
            end
            a_max_col = max(a_max_col, a_min_col);

            obj.a_max = a_max_col;
            obj.a_min = a_min_col;
        end

        function value = get.len(obj)
            %GET.LEN Return the number of station bounds.
            value = numel(obj.a_max);
        end
    end
end
