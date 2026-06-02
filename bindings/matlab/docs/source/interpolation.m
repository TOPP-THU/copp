%% Interpolation
% Solvers return path-domain profiles. Interpolation helpers convert those
% profiles into time-domain data:
%
% * |s_to_t_topp2| and |s_to_t_topp3| compute cumulative arrival times \(t_s\).
% * |t_to_s_topp2_uniform| and |t_to_s_topp3_uniform| sample \(s(t)\) on a
%   uniform time grid.
% * |t_to_s_topp2_samples| and |t_to_s_topp3_samples| sample \(s(t)\) at
%   user-supplied time points.
% * |t_to_s_topp2| and |t_to_s_topp3| are compatibility wrappers that accept
%   either |dt=...| or |t_sample=...| and dispatch to the explicit helper.
% * |a_to_b_topp2| converts a TOPP2 node profile \(a\) to interval
%   accelerations \(b\).
%
% Shape convention:
%
% * Input vectors \(s\), \(a\), \(b\), \(t_s\), and \(t_\mathrm{sample}\) may be row or column
%   vectors.
% * Outputs \(t_s\) and \(s_t\) are always double column vectors.
% * TOPP2 \(a\) has length \(N\) and TOPP2 \(b\) from |a_to_b_topp2| has length
%   \(N-1\).
% * TOPP3 |Profile3rd.a| and |Profile3rd.b| both have length \(N\).

%% TOPP2 profile to time
% In TOPP2, \(a(k) = (ds/dt)^2\) is stored at the nodes of the station grid.

s = linspace(0.0, 1.0, 9).';
a = 0.25 + sin(pi*s).^2;

[t_final2, t_s2] = copp.interpolation.s_to_t_topp2(s, a);
s_uniform2 = copp.interpolation.t_to_s_topp2_uniform( ...
    s, a, t_s2, 0.1, include_final=true);
s_samples2 = copp.interpolation.t_to_s_topp2_samples( ...
    s, a, t_s2, linspace(0, t_final2, 5));
s_wrapper2 = copp.interpolation.t_to_s_topp2( ...
    s, a, t_s2, dt=0.1, include_final=true);
b_interval = copp.interpolation.a_to_b_topp2(s, a);

fprintf("TOPP2 final time: %.6f seconds\n", t_final2);
fprintf("TOPP2 t_s shape: %d x %d, wrapper s_t shape: %d x %d, b shape: %d x %d\n", ...
    size(t_s2, 1), size(t_s2, 2), ...
    size(s_wrapper2, 1), size(s_wrapper2, 2), ...
    size(b_interval, 1), size(b_interval, 2));
disp(table(s, a, t_s2, 'VariableNames', {'s', 'a', 't_s'}))
disp(table((1:numel(b_interval)).', b_interval, ...
    'VariableNames', {'interval', 'b'}))

%% TOPP3 profile to time
% In TOPP3, both \(a\) and \(b\) are node-based and are stored in
% |copp.Profile3rd|. The stationary counters tell the interpolation code
% how many endpoint nodes are treated as stationary by solver output.

profile = copp.Profile3rd(ones(size(s)), zeros(size(s)), num_stationary=[0, 0]);
[t_final3, t_s3] = copp.interpolation.s_to_t_topp3(s, profile);
s_uniform3 = copp.interpolation.t_to_s_topp3_uniform( ...
    s, profile, t_s3, 0.1, include_final=true);
s_samples3 = copp.interpolation.t_to_s_topp3_samples( ...
    s, profile, t_s3, linspace(0, t_final3, 5));

fprintf("TOPP3 final time: %.6f seconds\n", t_final3);
fprintf("TOPP3 t_s shape: %d x %d, uniform s_t shape: %d x %d\n", ...
    size(t_s3, 1), size(t_s3, 2), size(s_uniform3, 1), size(s_uniform3, 2));
disp(table(s, profile.a, profile.b, t_s3, ...
    'VariableNames', {'s', 'a', 'b', 't_s'}))

%% Uniform versus explicit samples
% Uniform helpers are convenient for playback loops:
%
%   s_t = t_to_s_topp2_uniform(s, a, t_s, dt)
%   s_t = t_to_s_topp2(s, a, t_s, dt=dt)
%
% Explicit-sample helpers are better when another system chooses the timestamp
% grid:
%
%   s_t = t_to_s_topp2_samples(s, a, t_s, t_sample)
%   s_t = t_to_s_topp2(s, a, t_s, t_sample=t_sample)

disp(table((1:numel(s_uniform2)).', s_uniform2, ...
    'VariableNames', {'uniform_id', 's_topp2'}))
disp(table(linspace(0, t_final3, 5).', s_samples3, ...
    'VariableNames', {'t_sample', 's_topp3'}))

%% Practical rules
% * \(s\) must be strictly increasing for solver-generated profiles.
% * \(s\), \(a\), \(b\), and \(t_s\) must be aligned and finite.
% * TOPP2 \(b\) is interval-based and has length \(\mathrm{numel}(s)-1\).
% * TOPP3 \(b\) is node-based and has length \(\mathrm{numel}(s)\).
% * Out-of-range target times in inverse interpolation are represented by the
%   native interpolation policy; prefer sample grids inside \([t_s(1), t_s(\mathrm{end})]\).

%% See also
% |copp.interpolation.s_to_t_topp2|,
% |copp.interpolation.t_to_s_topp2|,
% |copp.interpolation.t_to_s_topp2_uniform|,
% |copp.interpolation.t_to_s_topp2_samples|,
% |copp.interpolation.s_to_t_topp3|,
% |copp.Profile3rd|
