classdef Profile3rd
    %PROFILE3RD Third-order path-speed profile.
    %
    % A Profile3rd stores the two station profiles used by TOPP3/COPP3:
    %
    %   a(k) = (ds/dt)^2
    %   b(k) = d2s/dt2
    %
    % The stationary counters describe how many grid points at each end are
    % treated as stationary by the third-order interpolation formulas. Solver
    % outputs are copied into MATLAB-owned double column vectors of length N,
    % so users do not manage native buffers or handles. A and B may be passed
    % as row or column vectors, but obj.a and obj.b are always N-by-1.
    %
    % Example:
    %   profile = copp.Profile3rd(ones(3,1), zeros(3,1), ...
    %       num_stationary=[0, 0]);

    properties (SetAccess = private)
        %A Node profile a(s) = (ds/dt)^2 as an N-by-1 double column vector.
        a

        %B Node profile b(s) = d2s/dt2 as an N-by-1 double column vector.
        b

        %NUM_STATIONARY_START Number of stationary nodes at the profile start.
        num_stationary_start

        %NUM_STATIONARY_END Number of stationary nodes at the profile end.
        num_stationary_end
    end

    properties (Dependent)
        %LEN Number of station samples in a and b.
        len

        %NUM_STATIONARY Two-element vector [start, end].
        num_stationary
    end

    methods
        function obj = Profile3rd(a, b, opts)
            %PROFILE3RD Construct a MATLAB-owned third-order profile.
            %
            % P = copp.Profile3rd(A, B) copies real finite vectors A and B
            % into N-by-1 double column vectors. A and B may be 1-by-N or N-by-1,
            % but they must have the same number of elements. A must be
            % nonnegative because a(k) stores (ds/dt)^2.
            %
            % P = ...Profile3rd(..., num_stationary=[NS0, NSF]) stores the
            % stationary-node metadata returned by TOPP3 solvers.
            arguments
                a {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
                b {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
                opts.num_stationary = [0, 0]
            end

            if numel(a) ~= numel(b)
                error("copp:InvalidArgument", ...
                    "Profile3rd a and b must have the same length.");
            end

            num_stationary = double(opts.num_stationary);
            validateattributes(num_stationary, {'numeric'}, ...
                {'real', 'finite', 'integer', 'nonnegative', 'numel', 2}, ...
                'Profile3rd', 'num_stationary');

            a_col = reshape(double(a), [], 1);
            b_col = reshape(double(b), [], 1);
            if numel(a_col) < 2
                error("copp:InvalidArgument", ...
                    "Profile3rd must contain at least two station samples.");
            end
            if any(a_col < 0)
                error("copp:InvalidArgument", ...
                    "Profile3rd a must be nonnegative.");
            end
            if sum(num_stationary) > numel(a_col) - 2
                error("copp:InvalidArgument", ...
                    "Profile3rd num_stationary must leave at least one non-stationary interval.");
            end

            obj.a = a_col;
            obj.b = b_col;
            obj.num_stationary_start = double(num_stationary(1));
            obj.num_stationary_end = double(num_stationary(2));
        end

        function value = get.len(obj)
            %LEN Number of profile samples.
            value = numel(obj.a);
        end

        function value = get.num_stationary(obj)
            %NUM_STATIONARY Return [num_stationary_start, num_stationary_end].
            value = [obj.num_stationary_start, obj.num_stationary_end];
        end

        function value = length(obj)
            %LENGTH Number of station samples in scalar Profile3rd objects.
            value = obj.len;
        end

        function [obj, succeeded] = force_positive_a(obj, s, opts)
            %FORCE_POSITIVE_A Adjust a and b so interpolated a(s) stays positive.
            %
            % [P, SUCCEEDED] = force_positive_a(P, S) returns a copy of profile
            % P whose a and b node values are adjusted so the third-order
            % interpolated a(s) stays strictly positive on every interval of
            % the station grid S. S must be a strictly increasing real finite
            % vector with numel(S) == P.len and at least four stations. The
            % returned object keeps the class of P, including solver-local
            % Profile3rd aliases, and its num_stationary metadata.
            %
            % SUCCEEDED is false when the adjustment failed for some interval.
            % This is not an error, but other intervals may still have been
            % adjusted, so keep using the original profile in that case.
            % Invalid input raises copp:InvalidArgument: a length mismatch,
            % fewer than four stations, negative a or a_min, non-finite values,
            % non-increasing S, or stationary counts that leave no motion
            % interval.
            %
            % This is a numerical safety pass for profiles whose a values may
            % touch zero because of finite precision, applied before
            % s_to_t_topp3. It rewrites a and
            % b without knowing the robot limits, so re-check the delivered
            % profile with Robot.exceed_topp3 afterwards.
            %
            % Name-value options:
            %   a_min:
            %       Nonnegative lower target for interpolated a. Defaults to
            %       1e-12.
            arguments
                obj (1,1) copp.Profile3rd
                s {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
                opts.a_min (1,1) {mustBeNumeric, mustBeReal, mustBeFinite, mustBeNonnegative} = 1.0e-12
            end

            [a_adjusted, b_adjusted, native_succeeded] = copp.internal.copp_mex( ...
                'force_positive_a_3rd', ...
                double(s), ...
                obj.a, ...
                obj.b, ...
                double(obj.num_stationary_start), ...
                double(obj.num_stationary_end), ...
                double(opts.a_min));

            obj.a = reshape(double(a_adjusted), [], 1);
            obj.b = reshape(double(b_adjusted), [], 1);
            succeeded = logical(native_succeeded);
        end
    end
end
