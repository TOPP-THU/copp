function [t_final, t_s] = s_to_t_topp2(s, a, opts)
%S_TO_T_TOPP2 Convert a TOPP2 a(s) profile into cumulative arrival times.
%
% [T_FINAL, T_S] = copp.interpolation.s_to_t_topp2(S, A) converts a
% second-order path-speed profile into arrival times. S and A must be real
% finite vectors of the same length N; either 1-by-N or N-by-1 input is
% accepted. A(k) stores (ds/dt)^2 at S(k).
%
% [T_FINAL, T_S] = ...s_to_t_topp2(S, A, t0=T0) starts the cumulative time at
% T0. T_S is returned as an N-by-1 double column vector, matching solver
% profile outputs.
arguments
    s {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
    a {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
    opts.t0 (1,1) {mustBeNumeric, mustBeReal, mustBeFinite} = 0
end

if numel(s) ~= numel(a)
    error("copp:InvalidArgument", ...
        "s and a must have the same number of elements.");
end

[t_final, t_s] = copp.internal.copp_mex( ...
    's_to_t_topp2', s, a, double(opts.t0));
end
