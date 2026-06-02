function s_t = t_to_s_topp3_uniform(s, profile, t_s, dt, opts)
%T_TO_S_TOPP3_UNIFORM Sample s(t) from a TOPP3 profile on a uniform grid.
%
% S_T = copp.interpolation.t_to_s_topp3_uniform(S, PROFILE, T_S, DT)
% samples path position at T0, T0+DT, T0+2*DT, ... using the third-order
% profile represented by PROFILE. S and T_S must have length N, matching the
% N-by-1 PROFILE.a and PROFILE.b vectors. T_S is usually returned by
% s_to_t_topp3(). S_T is returned as an M-by-1 double column vector, where M is
% the number of generated time samples.
%
% Name-value options:
%   t0:
%       First sample time. Defaults to 0.
%   include_final:
%       Append the final path position when the uniform grid does not land
%       exactly on the final time. Defaults to true.
arguments
    s {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
    profile (1,1) copp.Profile3rd
    t_s {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
    dt (1,1) {mustBeNumeric, mustBeReal, mustBeFinite, mustBePositive}
    opts.t0 (1,1) {mustBeNumeric, mustBeReal, mustBeFinite} = 0
    opts.include_final (1,1) logical = true
end

if numel(s) ~= profile.len || numel(s) ~= numel(t_s)
    error("copp:InvalidArgument", ...
        "s, profile, and t_s must have the same number of elements.");
end

s_t = copp.internal.copp_mex( ...
    't_to_s_topp3_uniform', ...
    s, ...
    profile.a, ...
    profile.b, ...
    double(profile.num_stationary_start), ...
    double(profile.num_stationary_end), ...
    t_s, ...
    double(opts.t0), ...
    double(dt), ...
    double(opts.include_final));
end
