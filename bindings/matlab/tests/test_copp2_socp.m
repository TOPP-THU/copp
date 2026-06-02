classdef test_copp2_socp < matlab.unittest.TestCase
    %TEST_COPP2_SOCP Public examples for COPP2-SOCP non-expert usage.
    %
    % These tests intentionally use only public MATLAB APIs. They show the
    % first convex-objective workflow:
    %   1. create objective descriptors with copp.objective.*;
    %   2. build a Robot with station-indexed q/dq/ddq and limits;
    %   3. construct a solver.copp2_socp.Problem;
    %   4. call solver.copp2_socp.solve to get the optimized a(s) profile.

    methods (Test)
        function objective_factories_and_problem_validation(testCase)
            % Objective factories return MATLAB-owned descriptor structs.
            n = 5;
            [robot, cleaner] = simple_robot(n);

            time = copp.objective.time(1.0);
            linear = copp.objective.linear(0.25, ones(n, 1), zeros(n - 1, 1));
            thermal = copp.objective.thermal_energy(0.1, ones(robot.dim, 1));
            tv_torque = copp.objective.total_variation_torque(0.05, ones(robot.dim, 1));

            problem = copp.solver.copp2_socp.Problem( ...
                robot, ...
                {time, linear, thermal, tv_torque}, ...
                idx_s_interval=[1, n], ...
                a_boundary=[0, 0]);

            testCase.verifyEqual(time.kind, "time");
            testCase.verifyEqual(linear.alpha, ones(n, 1));
            testCase.verifyEqual(linear.beta, zeros(n - 1, 1));
            testCase.verifyEqual(thermal.normalize, ones(robot.dim, 1));
            testCase.verifyEqual(problem.objective_count, 4);
            testCase.verifyEqual(problem.s_len, n);

            testCase.verifyError( ...
                @() copp.solver.copp2_socp.Problem( ...
                    robot, ...
                    copp.objective.linear(1.0, ones(n - 1, 1), zeros(n - 1, 1)), ...
                    idx_s_interval=[1, n]), ...
                'copp:InvalidArgument');

            clear cleaner
        end

        function copp2_socp_solves_minimal_profile(testCase)
            % A small rest-to-rest 1D problem solves with time + thermal terms.
            n = 7;
            [robot, cleaner, s] = simple_robot(n);

            objectives = {
                copp.objective.time(1.0)
                copp.objective.thermal_energy(0.05, ones(robot.dim, 1))
            };
            problem = copp.solver.copp2_socp.Problem( ...
                robot, ...
                objectives, ...
                idx_s_interval=[1, n], ...
                a_boundary=[0, 0]);
            options = copp.solver.copp2_socp.Options();

            a = copp.solver.copp2_socp.solve(problem, options);
            [t_final, t_s] = copp.interpolation.s_to_t_topp2(s, a, t0=0);

            testCase.verifyEqual(problem.objective_count, 2);
            testCase.verifySize(a, [n, 1]);
            testCase.verifyTrue(all(isfinite(a)));
            testCase.verifyGreaterThanOrEqual(a, -1.0e-8);
            testCase.verifyEqual(a(1), 0.0, AbsTol=1.0e-7);
            testCase.verifyEqual(a(end), 0.0, AbsTol=1.0e-7);
            testCase.verifyGreaterThan(t_final, 0.0);
            testCase.verifyEqual(t_s(1), 0.0, AbsTol=1.0e-12);
            testCase.verifyEqual(t_s(end), t_final, AbsTol=1.0e-10);

            clear cleaner
        end

        function copp2_socp_expert_returns_diagnostics(testCase)
            % Expert mode returns both the accepted a-profile and raw Clarabel
            % diagnostic vectors used to inspect solver behavior.
            n = 7;
            [robot, cleaner] = simple_robot(n);

            objectives = {
                copp.objective.time(1.0)
                copp.objective.thermal_energy(0.05, ones(robot.dim, 1))
            };
            problem = copp.solver.copp2_socp.Problem( ...
                robot, ...
                objectives, ...
                idx_s_interval=[1, n], ...
                a_boundary=[0, 0]);

            result = copp.solver.copp2_socp.solve_expert(problem);

            testCase.verifyClass(result, 'copp.solver.copp2_socp.Result');
            testCase.verifyTrue(result.has_a);
            testCase.verifySize(result.a, [n, 1]);
            testCase.verifyTrue(any(result.solver_status == ["solved", "almost_solved"]));
            testCase.verifyGreaterThan(numel(result.x), 0);
            testCase.verifyGreaterThan(numel(result.z), 0);
            testCase.verifyGreaterThan(numel(result.s), 0);
            testCase.verifyGreaterThanOrEqual(result.iterations, 0);
            testCase.verifyTrue(isstruct(result.linsolver));
            testCase.verifyGreaterThan(result.linsolver.nnz_a, 0);
            testCase.verifyEqual(numel(result.objective_terms), problem.objective_count);
            testCase.verifyTrue(isfinite(result.objective_value));

            clear cleaner
        end
    end
end

function [robot, cleaner, s] = simple_robot(n)
%SIMPLE_ROBOT Build a 1D q(s)=s robot with symmetric limits.
robot = copp.Robot(1, n);
cleaner = onCleanup(@() robot.release());

s = linspace(0.0, 1.0, n).';
robot.append_s(s);
robot.set_q_2nd(s.', ones(1, n), zeros(1, n));
robot.add_velocity_limits(1, -1);
robot.add_acceleration_limits(1, -1);
end
