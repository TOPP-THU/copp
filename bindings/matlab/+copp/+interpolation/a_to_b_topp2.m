function b = a_to_b_topp2(s, a)
%A_TO_B_TOPP2 Convert a TOPP2 a(s) profile into edge b values.
%
% B = copp.interpolation.a_to_b_topp2(S, A) returns the piecewise edge
% acceleration profile associated with node values A, where A(k) stores
% (ds/dt)^2 at station S(k). S and A must be real finite vectors of the same
% length N; row or column inputs are accepted. B is returned as an (N-1)-by-1
% double column vector because TOPP2 b is interval-based.
arguments
    s {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
    a {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
end

if numel(s) ~= numel(a)
    error("copp:InvalidArgument", ...
        "s and a must have the same number of elements.");
end

b = copp.internal.copp_mex('a_to_b_topp2', s, a);
end
