function s_t = t_to_s_topp2_samples(s, a, t_s, t_sample)
%T_TO_S_TOPP2_SAMPLES Sample s(t) from a TOPP2 profile at explicit times.
%
% S_T = copp.interpolation.t_to_s_topp2_samples(S, A, T_S, T_SAMPLE)
% maps each requested time in T_SAMPLE to a path position. S, A, and T_S must
% be real finite vectors of the same length N; row or column inputs are
% accepted. T_SAMPLE must be a real finite vector with M elements in the time
% range represented by T_S. S_T is returned as an M-by-1 double column vector.
arguments
    s {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
    a {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
    t_s {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
    t_sample {mustBeNumeric, mustBeReal, mustBeVector, mustBeFinite}
end

if numel(s) ~= numel(a) || numel(s) ~= numel(t_s)
    error("copp:InvalidArgument", ...
        "s, a, and t_s must have the same number of elements.");
end

s_t = copp.internal.copp_mex( ...
    't_to_s_topp2_samples', ...
    s, ...
    a, ...
    t_s, ...
    t_sample);
end
