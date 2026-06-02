classdef test_topp2_interpolation < matlab.unittest.TestCase
    %TEST_TOPP2_INTERPOLATION Public examples for TOPP2 interpolation helpers.
    %
    % These tests document the profile post-processing API used after a TOPP2
    % solver returns a(s). They call only the MATLAB facade, not private MEX
    % commands.

    methods (Test)
        function s_to_t_topp2_identity_profile(testCase)
            % Unit speed over s in [0, 1] should produce t(s) = s.
            s = [0; 0.5; 1.0];
            a = ones(3, 1);

            [t_final, t_s] = copp.interpolation.s_to_t_topp2(s, a, t0=0);

            testCase.verifyEqual(t_final, 1.0, AbsTol=1e-12);
            testCase.verifyEqual(t_s, s, AbsTol=1e-12);
            testCase.verifySize(t_s, [3, 1]);
        end

        function s_to_t_topp2_accepts_single(testCase)
            % The public MATLAB wrapper accepts single inputs and returns
            % normalized double column-vector outputs from the MEX gateway.
            s = single([0, 0.5, 1.0]);
            a = single([1, 1, 1]);

            [t_final, t_s] = copp.interpolation.s_to_t_topp2(s, a, t0=single(0));

            testCase.verifyEqual(t_final, 1.0, AbsTol=1e-12);
            testCase.verifyEqual(t_s, [0; 0.5; 1.0], AbsTol=1e-12);
        end

        function a_to_b_topp2_converts_constant_speed(testCase)
            % Constant a(s)=1 has zero edge acceleration on both intervals.
            s = [0; 0.5; 1.0];
            a = ones(3, 1);

            b = copp.interpolation.a_to_b_topp2(s, a);

            testCase.verifyEqual(b, [0; 0], AbsTol=1e-12);
            testCase.verifySize(b, [2, 1]);
        end

        function t_to_s_topp2_samples_uniform_time_grid(testCase)
            % With t(s)=s, uniform time samples map back to the same values.
            s = [0; 0.5; 1.0];
            a = ones(3, 1);
            [~, t_s] = copp.interpolation.s_to_t_topp2(s, a);

            s_t = copp.interpolation.t_to_s_topp2_uniform(s, a, t_s, 0.25);

            testCase.verifyEqual(s_t, (0:0.25:1).', AbsTol=1e-12);
        end

        function t_to_s_topp2_wrapper_dispatches_uniform(testCase)
            % The compatibility wrapper accepts dt=DT and returns the same
            % column-vector output as the explicit uniform helper.
            s = [0; 0.5; 1.0];
            a = ones(3, 1);
            [~, t_s] = copp.interpolation.s_to_t_topp2(s, a);

            s_t = copp.interpolation.t_to_s_topp2(s, a, t_s, dt=0.25);

            testCase.verifyEqual(s_t, (0:0.25:1).', AbsTol=1e-12);
        end

        function t_to_s_topp2_samples_explicit_times(testCase)
            % Explicit sample times use the same interpolation path as the
            % uniform helper, but the caller controls every requested time.
            s = [0; 0.5; 1.0];
            a = ones(3, 1);
            [~, t_s] = copp.interpolation.s_to_t_topp2(s, a);
            t_sample = [0; 0.2; 0.8; 1.0];

            s_t = copp.interpolation.t_to_s_topp2_samples(s, a, t_s, t_sample);

            testCase.verifyEqual(s_t, t_sample, AbsTol=1e-12);
        end

        function t_to_s_topp2_wrapper_dispatches_samples(testCase)
            % The compatibility wrapper accepts t_sample=T_SAMPLE and returns
            % the same column-vector output as the explicit samples helper.
            s = [0; 0.5; 1.0];
            a = ones(3, 1);
            [~, t_s] = copp.interpolation.s_to_t_topp2(s, a);
            t_sample = [0; 0.2; 0.8; 1.0];

            s_t = copp.interpolation.t_to_s_topp2(s, a, t_s, ...
                t_sample=t_sample);

            testCase.verifyEqual(s_t, t_sample, AbsTol=1e-12);
        end

        function solver_namespace_interpolation_aliases_match_global_helpers(testCase)
            % Solver namespaces re-export common TOPP2 post-processing helpers.
            s = [0; 0.5; 1.0];
            a = ones(3, 1);

            [t_final, t_s] = copp.solver.copp2_socp.s_to_t(s, a);
            b = copp.solver.copp2_socp.a_to_b(s, a);
            s_t = copp.solver.copp2_socp.t_to_s(s, a, t_s, dt=0.25);

            testCase.verifyEqual(t_final, 1.0, AbsTol=1e-12);
            testCase.verifyEqual(t_s, s, AbsTol=1e-12);
            testCase.verifyEqual(b, [0; 0], AbsTol=1e-12);
            testCase.verifyEqual(s_t, (0:0.25:1).', AbsTol=1e-12);
        end
    end
end
