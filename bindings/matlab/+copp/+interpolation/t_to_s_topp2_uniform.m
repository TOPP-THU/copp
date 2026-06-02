function s_t = t_to_s_topp2_uniform(s, a, t_s, dt, opts)
%T_TO_S_TOPP2_UNIFORM Sample s(t) from a TOPP2 profile on a uniform time grid.
%
% S_T = copp.interpolation.t_to_s_topp2_uniform(S, A, T_S, DT) samples the
% path position at times T0, T0+DT, T0+2*DT, ... up to the final time in T_S.
% S, A, and T_S must be real finite vectors of the same length N; row or
% column inputs are accepted. A(k) stores (ds/dt)^2 at S(k), and T_S is
% usually the N-by-1 column returned by s_to_t_topp2(). S_T is returned as an
% M-by-1 double column vector, where M is the number of generated time samples.
%
% Name-value options:
%   t0:
%       First sample time. Defaults to 0.
%   include_final:
%       Append the final path position when the uniform grid does not land
%       exactly on the final time. Defaults to true.
arguments
    s {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
    a {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
    t_s {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
    dt (1,1) {mustBeNumeric, mustBeReal, mustBeFinite, mustBePositive}
    opts.t0 (1,1) {mustBeNumeric, mustBeReal, mustBeFinite} = 0
    opts.include_final (1,1) logical = true
end

if numel(s) ~= numel(a) || numel(s) ~= numel(t_s)
    error("copp:InvalidArgument", ...
        "s, a, and t_s must have the same number of elements.");
end

s_t = copp.internal.copp_mex( ...
    't_to_s_topp2_uniform', ...
    s, ...
    a, ...
    t_s, ...
    double(opts.t0), ...
    double(dt), ...
    double(opts.include_final));
end
