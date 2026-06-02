function s_t = t_to_s_topp3_samples(s, profile, t_s, t_sample)
%T_TO_S_TOPP3_SAMPLES Sample s(t) from a TOPP3 profile at explicit times.
%
% S_T = copp.interpolation.t_to_s_topp3_samples(S, PROFILE, T_S,
% T_SAMPLE) maps each requested time in T_SAMPLE to a path position. S and
% T_S must have length N, matching the N-by-1 PROFILE.a and PROFILE.b vectors.
% T_SAMPLE must be a real finite vector with M elements inside the represented
% time range. S_T is returned as an M-by-1 double column vector.
arguments
    s {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
    profile (1,1) copp.Profile3rd
    t_s {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
    t_sample {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
end

if numel(s) ~= profile.len || numel(s) ~= numel(t_s)
    error("copp:InvalidArgument", ...
        "s, profile, and t_s must have the same number of elements.");
end

s_t = copp.internal.copp_mex( ...
    't_to_s_topp3_samples', ...
    s, ...
    profile.a, ...
    profile.b, ...
    double(profile.num_stationary_start), ...
    double(profile.num_stationary_end), ...
    t_s, ...
    t_sample);
end
