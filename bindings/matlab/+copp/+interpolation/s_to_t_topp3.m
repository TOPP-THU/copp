function [t_final, t_s] = s_to_t_topp3(s, profile, opts)
%S_TO_T_TOPP3 Convert a TOPP3 Profile3rd into cumulative arrival times.
%
% [T_FINAL, T_S] = copp.interpolation.s_to_t_topp3(S, PROFILE) converts
% a third-order profile into arrival times. S must be a real finite vector
% with length N, matching the N-by-1 PROFILE.a and PROFILE.b vectors. Row or
% column S input is accepted. PROFILE is usually returned by
% copp.solver.topp3_lp.solve or
% copp.solver.topp3_socp.solve.
%
% [T_FINAL, T_S] = ...s_to_t_topp3(..., t0=T0) starts the cumulative time at
% T0. T_S is returned as an N-by-1 double column vector.
arguments
    s {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
    profile (1,1) copp.Profile3rd
    opts.t0 (1,1) {mustBeNumeric, mustBeReal, mustBeFinite} = 0
end

if numel(s) ~= profile.len
    error("copp:InvalidArgument", ...
        "s must have the same number of elements as profile.a and profile.b.");
end

[t_final, t_s] = copp.internal.copp_mex( ...
    's_to_t_topp3', ...
    s, ...
    profile.a, ...
    profile.b, ...
    double(profile.num_stationary_start), ...
    double(profile.num_stationary_end), ...
    double(opts.t0));
end
