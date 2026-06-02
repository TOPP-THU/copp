function s_t = t_to_s_topp2(s, a, t_s, opts)
%T_TO_S_TOPP2 Compatibility wrapper for TOPP2 time-to-path interpolation.
%
% S_T = copp.interpolation.t_to_s_topp2(S, A, T_S, dt=DT) samples a
% uniform time grid and delegates to t_to_s_topp2_uniform(). S, A, and T_S
% must have length N. A stores the TOPP2 node profile a(k)=(ds/dt)^2.
%
% S_T = ...t_to_s_topp2(..., t_sample=T_SAMPLE) samples explicit times and
% delegates to t_to_s_topp2_samples(). T_SAMPLE has M elements, and S_T is
% returned as an M-by-1 double column vector.
%
% Exactly one of dt or t_sample must be supplied. New code may call the
% explicit uniform/samples helpers directly.
arguments
    s {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
    a {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
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
        't_to_s_topp2', 'dt');
    s_t = copp.interpolation.t_to_s_topp2_uniform( ...
        s, a, t_s, double(opts.dt), ...
        t0=opts.t0, ...
        include_final=opts.include_final);
else
    validateattributes(opts.t_sample, {'numeric'}, ...
        {'real', 'finite', 'vector'}, ...
        't_to_s_topp2', 't_sample');
    s_t = copp.interpolation.t_to_s_topp2_samples( ...
        s, a, t_s, opts.t_sample);
end
end
