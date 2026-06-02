classdef test_topp3_lp_socp < matlab.unittest.TestCase
    %TEST_TOPP3_LP_SOCP Public examples for TOPP3-LP/SOCP non-expert usage.
    %
    % These tests intentionally use only public MATLAB APIs. They show the
    % minimum third-order workflow:
    %   1. build a Robot with q/dq/ddq/dddq samples;
    %   2. add velocity, acceleration, and jerk limits;
    %   3. construct a TOPP3 problem with an a-linearization profile;
    %   4. solve with either LP or SOCP and post-process the Profile3rd.
    %
    % TOPP3 Problem construction is a pure MATLAB descriptor step. The native
    % Robot's third-order linearized-constraint cache is refreshed lazily inside
    % solve(), because the solve call owns the chosen a_linearization.

    methods (Test)
        function topp3_problem_validation_and_metadata(testCase)
            % Problem stores MATLAB-owned metadata and keeps a Robot reference.
            n = 7;
            [robot, cleaner] = simple_topp3_robot(n);

            problem = copp.solver.topp3_lp.Problem( ...
                robot, ...
                ones(n, 1), ...
                idx_s_start=1, ...
                a_boundary=[0, 0], ...
                b_boundary=[0, 0], ...
                num_stationary_max=[1, 1]);

            testCase.verifyClass(problem, 'copp.solver.topp3_lp.Problem');
            testCase.verifyEqual(problem.s_len, n);
            testCase.verifyEqual(problem.idx_s_interval, [1, n]);
            testCase.verifyEqual(problem.a_linearization, ones(n, 1));
            testCase.verifyEqual(problem.a_boundary, [0, 0]);
            testCase.verifyEqual(problem.b_boundary, [0, 0]);
            testCase.verifyEqual(problem.num_stationary_max, [1, 1]);

            testCase.verifyError( ...
                @() copp.solver.topp3_lp.Problem(robot, -ones(n, 1)), ...
                'copp:InvalidArgument');

            clear cleaner
        end

        function topp3_lp_solves_minimal_profile(testCase)
            % TOPP3-LP returns a Profile3rd accepted by interpolation helpers.
            n = 7;
            [robot, cleaner, s] = simple_topp3_robot(n);

            problem = copp.solver.topp3_lp.Problem( ...
                robot, ...
                ones(n, 1), ...
                idx_s_start=1, ...
                a_boundary=[0, 0], ...
                b_boundary=[0, 0], ...
                num_stationary_max=1);
            options = copp.solver.topp3_lp.Options();

            profile = copp.solver.topp3_lp.solve(problem, options);
            [t_final, t_s] = copp.interpolation.s_to_t_topp3(s, profile);

            verify_topp3_profile(testCase, profile, n);
            testCase.verifyGreaterThan(t_final, 0.0);
            testCase.verifyEqual(t_s(1), 0.0, AbsTol=1.0e-12);
            testCase.verifyEqual(t_s(end), t_final, AbsTol=1.0e-10);

            clear cleaner
        end

        function topp3_lp_expert_returns_diagnostics(testCase)
            % Expert mode exposes raw Clarabel vectors plus accepted profile.
            n = 7;
            [robot, cleaner] = simple_topp3_robot(n);

            problem = copp.solver.topp3_lp.Problem( ...
                robot, ones(n, 1), ...
                idx_s_start=1, a_boundary=[0, 0], b_boundary=[0, 0], ...
                num_stationary_max=1);

            result = copp.solver.topp3_lp.solve_expert(problem);

            testCase.verifyClass(result, 'copp.solver.topp3_lp.Result');
            testCase.verifyTrue(result.has_profile);
            verify_topp3_profile(testCase, result.profile, n);
            testCase.verifyEqual(result.solver_status, "solved");
            testCase.verifyTrue(isstruct(result.linsolver));
            testCase.verifyTrue(isfield(result.linsolver, 'method'));

            clear cleaner
        end

        function topp3_socp_solves_minimal_profile(testCase)
            % TOPP3-SOCP uses the same descriptor shape and Profile3rd result.
            n = 7;
            [robot, cleaner, s] = simple_topp3_robot(n);

            problem = copp.solver.topp3_socp.Problem( ...
                robot, ...
                ones(n, 1), ...
                idx_s_start=1, ...
                a_boundary=[0, 0], ...
                b_boundary=[0, 0], ...
                num_stationary_max=1);
            options = copp.solver.topp3_socp.Options();

            profile = copp.solver.topp3_socp.solve(problem, options);
            [t_final, t_s] = copp.interpolation.s_to_t_topp3(s, profile);

            verify_topp3_profile(testCase, profile, n);
            testCase.verifyGreaterThan(t_final, 0.0);
            testCase.verifyEqual(t_s(1), 0.0, AbsTol=1.0e-12);
            testCase.verifyEqual(t_s(end), t_final, AbsTol=1.0e-10);

            clear cleaner
        end

        function topp3_socp_expert_returns_diagnostics(testCase)
            % SOCP expert mode shares the COPP3-style Clarabel result shape.
            n = 7;
            [robot, cleaner] = simple_topp3_robot(n);

            problem = copp.solver.topp3_socp.Problem( ...
                robot, ones(n, 1), ...
                idx_s_start=1, a_boundary=[0, 0], b_boundary=[0, 0], ...
                num_stationary_max=1);

            result = copp.solver.topp3_socp.solve_expert(problem);

            testCase.verifyClass(result, 'copp.solver.topp3_socp.Result');
            testCase.verifyTrue(result.has_profile);
            verify_topp3_profile(testCase, result.profile, n);
            testCase.verifyEqual(result.solver_status, "solved");
            testCase.verifyTrue(isstruct(result.linsolver));
            testCase.verifyTrue(isfield(result.linsolver, 'method'));

            clear cleaner
        end
    end
end

function [robot, cleaner, s] = simple_topp3_robot(n)
%SIMPLE_TOPP3_ROBOT Build a 2D affine path with generous third-order limits.
robot = copp.Robot(2, n);
cleaner = onCleanup(@() robot.release());

s = linspace(0.0, 1.0, n).';
scales = [1; 2];
q = scales .* s.';
dq = repmat(scales, 1, n);
ddq = zeros(2, n);
dddq = zeros(2, n);

robot.append_s(s);
robot.set_q_3rd(q, dq, ddq, dddq);

upper = 100.0 * ones(robot.dim, 1);
lower = -upper;
robot.add_velocity_limits(upper, lower);
robot.add_acceleration_limits(upper, lower);
robot.add_jerk_limits(upper, lower);
end

function verify_topp3_profile(testCase, profile, n)
%VERIFY_TOPP3_PROFILE Check the public Profile3rd result contract.
testCase.verifyClass(profile, 'copp.Profile3rd');
testCase.verifySize(profile.a, [n, 1]);
testCase.verifySize(profile.b, [n, 1]);
testCase.verifyEqual(profile.num_stationary, [1, 1]);
testCase.verifyTrue(all(isfinite(profile.a)));
testCase.verifyTrue(all(isfinite(profile.b)));
testCase.verifyGreaterThanOrEqual(profile.a, -1.0e-8);
testCase.verifyEqual(profile.a(1), 0.0, AbsTol=1.0e-7);
testCase.verifyEqual(profile.a(end), 0.0, AbsTol=1.0e-7);
testCase.verifyEqual(profile.b(1), 0.0, AbsTol=1.0e-7);
testCase.verifyEqual(profile.b(end), 0.0, AbsTol=1.0e-7);
end
