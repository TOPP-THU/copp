%% s_to_t_topp2
% Convert a TOPP2 path-speed profile into cumulative arrival times.
%
% |copp.interpolation.s_to_t_topp2| converts a station grid and a TOPP2
% node profile to a cumulative time grid. Each profile value is
%
% $$ a(k) = \left(\frac{ds}{dt}\right)^2 $$
%
% so \(\sqrt{a}\) is the path speed at station \(s(k)\). Use this helper after a
% TOPP2 solver when you need arrival times for plotting, interpolation, or
% time-indexed trajectory playback.

%% Syntax
%
%   [t_final,t_s] = copp.interpolation.s_to_t_topp2(s,a)
%   [t_final,t_s] = copp.interpolation.s_to_t_topp2(s,a,t0=t0)

%% Description
% |[t_final,t_s] = copp.interpolation.s_to_t_topp2(s,a)| converts the
% TOPP2 node profile \(a\) on station grid \(s\) into cumulative arrival times.
% The returned \(t_s\) has the same number of elements as \(s\) and \(a\), and
% \(t_\mathrm{final}\) is the final element of the cumulative time grid. \(s\) and \(a\) may
% be row or column vectors; \(t_s\) is always an \(N \times 1\) double column vector.
%
% |[t_final,t_s] = copp.interpolation.s_to_t_topp2(s,a,t0=t0)| starts
% the cumulative time grid at \(t_0\) instead of zero. Use this form when a
% TOPP2 segment is appended after earlier trajectory segments.

%% Examples
% Convert a compact TOPP2 profile to arrival times and inspect the result.

s = linspace(0, 1, 9).';
a = 0.20 + 1.30 * sin(pi * s).^2;

path_speed = sqrt(a);
profile_table = table(s, a, path_speed, ...
    'VariableNames', {'s', 'a', 'ds_dt'});
disp(profile_table)

[t_final, t_s] = copp.interpolation.s_to_t_topp2(s, a);
fprintf("Input length N=%d, output t_s shape=%d x %d\n", ...
    numel(s), size(t_s, 1), size(t_s, 2));

time_table = table(s, a, t_s, ...
    'VariableNames', {'s', 'a', 't_s'});
disp(time_table)
fprintf("Final traversal time: %.6f seconds\n", t_final);

%%
% Start the cumulative time grid at a nonzero time.

[t_final_shifted, t_s_shifted] = copp.interpolation.s_to_t_topp2( ...
    s, a, t0=2.5);

fprintf("Shifted final time: %.6f seconds\n", t_final_shifted);
disp(table(s, t_s_shifted, 'VariableNames', {'s', 't_s_shifted'}))

%%
% Plot the squared path-speed profile and the resulting time law.

figure('Color', 'white');
tiledlayout(2, 1, 'TileSpacing', 'compact');

nexttile
plot(s, a, 'o-', 'LineWidth', 1.3)
grid on
xlabel('s')
ylabel('a = (ds/dt)^2')
title('TOPP2 node profile')

nexttile
plot(t_s, s, 'o-', 'LineWidth', 1.3)
grid on
xlabel('time')
ylabel('s')
title('Path position over time')

%% Input Arguments
% *|s| - Path stations*
%
% Strictly increasing real finite vector. The length of \(s\) must match the
% length of \(a\). The output \(t_s\) reports the cumulative arrival time at
% these same stations. Shape may be \(1 \times N\) or \(N \times 1\).
%
% Data Types: |single|, |double|
%
% *|a| - TOPP2 node profile*
%
% Real finite vector with the same number of elements as \(s\). Each element
% stores \((ds/dt)^2\) at the corresponding station. Profiles returned by
% TOPP2 solvers are \(N \times 1\) column vectors and can be passed directly. Row
% vectors are also accepted.
%
% Data Types: |single|, |double|

%% Name-Value Arguments
% *|t0| - Initial time*
%
% Initial cumulative time, specified as a real finite scalar. The default
% value is \(0\). Passing \(t_0\) shifts both \(t_s\) and \(t_\mathrm{final}\).
%
% Default: |0|
%
% Data Types: |single|, |double|

%% Output Arguments
% *|t_final| - Final arrival time*
%
% Scalar final time at the last station. When \(t_0\) is supplied, \(t_\mathrm{final}\)
% includes that initial offset.
%
% *|t_s| - Cumulative arrival times*
%
% Double column vector with one value per station. The first element is
% \(t_0\), and the last element equals \(t_\mathrm{final}\). Shape is \(N \times 1\) regardless of
% whether \(s\) and \(a\) were rows or columns.

%% More About
% TOPP2 solvers optimize or propagate a profile in the path domain. This
% function performs the standard conversion from path-domain squared speed to
% a time-domain cumulative grid. To sample \(s(t)\) after this conversion, use
% |copp.interpolation.t_to_s_topp2_uniform| or
% |copp.interpolation.t_to_s_topp2_samples|. The convenience wrapper
% |copp.interpolation.t_to_s_topp2| accepts either |dt=...| or
% |t_sample=...| and dispatches to the matching explicit helper.

%% See Also
% |copp.interpolation.a_to_b_topp2|,
% |copp.interpolation.t_to_s_topp2|,
% |copp.interpolation.t_to_s_topp2_uniform|,
% |copp.interpolation.t_to_s_topp2_samples|,
% |copp.solver.topp2_ra.solve|
