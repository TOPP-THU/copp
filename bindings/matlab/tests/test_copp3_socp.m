classdef test_copp3_socp < matlab.unittest.TestCase
    %TEST_COPP3_SOCP Public examples for COPP3-SOCP usage.

    methods (Test)
        function copp3_problem_constructs_with_objectives(testCase)
            n = 7;
            [robot, cleaner] = simple_topp3_robot_for_copp3_socp(n);
            objectives = {
                copp.objective.time(1.0)
                copp.objective.thermal_energy(0.1, ones(robot.dim, 1))
            };

            problem = copp.solver.copp3_socp.Problem( ...
                robot, objectives, ones(n, 1), ...
                idx_s_start=1, ...
                a_boundary=[0, 0], ...
                b_boundary=[0, 0], ...
                num_stationary_max=1);

            testCase.verifyClass(problem, 'copp.solver.copp3_socp.Problem');
            testCase.verifyEqual(problem.objective_count, 2);
            testCase.verifyEqual(problem.s_len, n);
            testCase.verifyEqual(problem.idx_s_interval, [1, n]);
            testCase.verifyEqual(problem.num_stationary_max, [1, 1]);

            clear cleaner
        end

        function copp3_socp_solves_and_expert_reports_terms(testCase)
            n = 7;
            [robot, cleaner, s] = simple_topp3_robot_for_copp3_socp(n);
            objectives = {
                copp.objective.time(1.0)
                copp.objective.thermal_energy(0.1, ones(robot.dim, 1))
            };
            problem = copp.solver.copp3_socp.Problem( ...
                robot, objectives, ones(n, 1), ...
                idx_s_start=1, ...
                a_boundary=[0, 0], ...
                b_boundary=[0, 0], ...
                num_stationary_max=1);

            profile = copp.solver.copp3_socp.solve(problem);
            result = copp.solver.copp3_socp.solve_expert(problem);
            [t_final, ~] = copp.interpolation.s_to_t_topp3(s, profile);

            verify_copp3_profile(testCase, profile, n);
            testCase.verifyClass(result, 'copp.solver.copp3_socp.Result');
            testCase.verifyTrue(result.has_profile);
            verify_copp3_profile(testCase, result.profile, n);
            testCase.verifyEqual(numel(result.objective_terms), 2);
            testCase.verifyTrue(isfinite(result.objective_value));
            testCase.verifyGreaterThan(t_final, 0.0);

            clear cleaner
        end

        function copp3_linear_objective_uses_node_length_beta(testCase)
            n = 7;
            [robot, cleaner] = simple_topp3_robot_for_copp3_socp(n);
            objectives = {
                copp.objective.time(1.0)
                copp.objective.linear(0.01, zeros(n, 1), zeros(n, 1))
            };

            problem = copp.solver.copp3_socp.Problem( ...
                robot, objectives, ones(n, 1), ...
                idx_s_start=1, ...
                a_boundary=[0, 0], ...
                b_boundary=[0, 0], ...
                num_stationary_max=1);
            result = copp.solver.copp3_socp.solve_expert(problem);

            testCase.verifyTrue(result.has_profile);
            testCase.verifyEqual(numel(result.objective_terms), 2);

            clear cleaner
        end
    end
end

function [robot, cleaner, s] = simple_topp3_robot_for_copp3_socp(n)
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
robot.add_velocity_limits(upper, -upper);
robot.add_acceleration_limits(upper, -upper);
robot.add_jerk_limits(upper, -upper);
end

function verify_copp3_profile(testCase, profile, n)
testCase.verifyClass(profile, 'copp.Profile3rd');
testCase.verifySize(profile.a, [n, 1]);
testCase.verifySize(profile.b, [n, 1]);
testCase.verifyEqual(profile.num_stationary, [1, 1]);
testCase.verifyTrue(all(isfinite(profile.a)));
testCase.verifyTrue(all(isfinite(profile.b)));
end
