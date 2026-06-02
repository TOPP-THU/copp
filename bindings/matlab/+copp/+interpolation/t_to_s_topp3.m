function s_t = t_to_s_topp3(s, profile, t_s, opts)
%T_TO_S_TOPP3 Compatibility wrapper for TOPP3 time-to-path interpolation.
%
% S_T = copp.interpolation.t_to_s_topp3(S, PROFILE, T_S, dt=DT) samples
% a uniform grid and delegates to t_to_s_topp3_uniform(). S and T_S must have
% length N, matching the N-by-1 PROFILE.a and PROFILE.b vectors.
%
% S_T = ...t_to_s_topp3(..., t_sample=T_SAMPLE) samples explicit times and
% delegates to t_to_s_topp3_samples(). T_SAMPLE has M elements, and S_T is
% returned as an M-by-1 double column vector.
%
% Exactly one of dt or t_sample must be supplied. New code may call the
% explicit uniform/samples helpers directly.
arguments
    s {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
    profile (1,1) copp.Profile3rd
    t_s {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
    opts.dt = []
    opts.t_sample = []
    opts.t0 (1,1) {mustBeNumeric, mustBeReal, mustBeFinite} = 0
    opts.include_final (1,1) logical = true
end

has_dt = ~isempty(opts.dt);
has_t_sample = ~isempty(opts.t_sample);
if has_dt == has_t_sample
    error("copp:InvalidArgument", ...
        "Specify exactly one of dt or t_sample.");
end

if has_dt
    validateattributes(opts.dt, {'numeric'}, ...
        {'real', 'finite', 'positive', 'scalar'}, ...
        't_to_s_topp3', 'dt');
    s_t = copp.interpolation.t_to_s_topp3_uniform( ...
        s, profile, t_s, double(opts.dt), ...
        t0=opts.t0, ...
        include_final=opts.include_final);
else
    validateattributes(opts.t_sample, {'numeric'}, ...
        {'real', 'finite', 'vector'}, ...
        't_to_s_topp3', 't_sample');
    s_t = copp.interpolation.t_to_s_topp3_samples( ...
        s, profile, t_s, opts.t_sample);
end
end
