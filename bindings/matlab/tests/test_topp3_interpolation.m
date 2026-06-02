classdef test_topp3_interpolation < matlab.unittest.TestCase
    %TEST_TOPP3_INTERPOLATION Public examples for TOPP3 profile helpers.
    %
    % These tests document the user-facing third-order post-processing API:
    %   1. construct a MATLAB-owned Profile3rd from a/b vectors;
    %   2. convert a third-order profile from s-space to arrival times;
    %   3. sample s(t) either on a uniform time grid or at explicit times.

    methods (Test)
        function profile3rd_properties_are_copies(testCase)
            % Profile3rd normalizes row/column inputs to double column vectors.
            a = [1.0, 1.0, 1.0];
            b = [0.0, 0.0, 0.0];

            profile = copp.Profile3rd(a, b, num_stationary=[0, 0]);

            testCase.verifyClass(profile, 'copp.Profile3rd');
            testCase.verifyEqual(profile.len, 3);
            testCase.verifyEqual(length(profile), 3);
            testCase.verifyEqual(profile.num_stationary, [0, 0]);
            testCase.verifyEqual(profile.num_stationary_start, 0);
            testCase.verifyEqual(profile.num_stationary_end, 0);
            testCase.verifyEqual(profile.a, ones(3, 1), AbsTol=1e-12);
            testCase.verifyEqual(profile.b, zeros(3, 1), AbsTol=1e-12);

            % Reading a value-class double property gives the caller a copy.
            profile_a = profile.a;
            profile_a(1) = 9.0;
            testCase.verifyEqual(profile.a, ones(3, 1), AbsTol=1e-12);
        end

        function topp3_interpolation_identity_profile(testCase)
            % With a(s)=1 and b(s)=0 over s in [0,1], s(t) is the identity.
            s = [0; 0.5; 1.0];
            profile = copp.Profile3rd(ones(3, 1), zeros(3, 1));

            [t_final, t_s] = copp.interpolation.s_to_t_topp3(s, profile);

            testCase.verifyEqual(t_final, 1.0, AbsTol=1e-12);
            testCase.verifyEqual(t_s, s, AbsTol=1e-12);
            testCase.verifySize(t_s, [3, 1]);

            s_uniform = copp.interpolation.t_to_s_topp3_uniform( ...
                s, profile, t_s, 0.25);
            testCase.verifyEqual(s_uniform, (0:0.25:1).', AbsTol=1e-12);

            t_sample = [0; 0.2; 0.8; 1.0];
            s_samples = copp.interpolation.t_to_s_topp3_samples( ...
                s, profile, t_s, t_sample);
            testCase.verifyEqual(s_samples, t_sample, AbsTol=1e-12);
        end

        function topp3_interpolation_compatibility_wrapper(testCase)
            % t_to_s_topp3 dispatches to the explicit uniform/samples helpers.
            s = [0; 0.5; 1.0];
            profile = copp.Profile3rd(ones(3, 1), zeros(3, 1));
            [~, t_s] = copp.interpolation.s_to_t_topp3(s, profile);

            s_uniform = copp.interpolation.t_to_s_topp3( ...
                s, profile, t_s, dt=0.25);
            s_samples = copp.interpolation.t_to_s_topp3( ...
                s, profile, t_s, t_sample=[0; 0.25; 0.5]);

            testCase.verifyEqual(s_uniform, (0:0.25:1).', AbsTol=1e-12);
            testCase.verifyEqual(s_samples, [0; 0.25; 0.5], AbsTol=1e-12);
            testCase.verifyError( ...
                @() copp.interpolation.t_to_s_topp3(s, profile, t_s), ...
                'copp:InvalidArgument');
        end

        function profile3rd_rejects_mismatched_lengths(testCase)
            % A and B describe the same station grid and must agree in length.
            testCase.verifyError( ...
                @() copp.Profile3rd([1; 1], 0), ...
                'copp:InvalidArgument');
        end

        function profile3rd_rejects_invalid_invariants(testCase)
            % A stores squared path speed and cannot be negative.
            testCase.verifyError( ...
                @() copp.Profile3rd([1; -1], [0; 0]), ...
                'copp:InvalidArgument');

            % Stationary counters must leave at least one ordinary interval.
            testCase.verifyError( ...
                @() copp.Profile3rd(ones(3, 1), zeros(3, 1), ...
                    num_stationary=[1, 1]), ...
                'copp:InvalidArgument');
        end

        function solver_namespace_interpolation_aliases_match_global_helpers(testCase)
            % Solver namespaces re-export common TOPP3/COPP3 post-processing helpers.
            s = [0; 0.5; 1.0];
            profile = copp.solver.copp3_socp.Profile3rd(ones(3, 1), zeros(3, 1));

            [t_final, t_s] = copp.solver.copp3_socp.s_to_t(s, profile);
            s_t = copp.solver.copp3_socp.t_to_s(s, profile, t_s, dt=0.25);

            testCase.verifyClass(profile, 'copp.solver.copp3_socp.Profile3rd');
            testCase.verifyTrue(isa(profile, 'copp.Profile3rd'));
            testCase.verifyEqual(t_final, 1.0, AbsTol=1e-12);
            testCase.verifyEqual(t_s, s, AbsTol=1e-12);
            testCase.verifyEqual(s_t, (0:0.25:1).', AbsTol=1e-12);
        end
    end
end
