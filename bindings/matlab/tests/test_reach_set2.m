classdef test_reach_set2 < matlab.unittest.TestCase
    %TEST_REACH_SET2 Public examples for TOPP2 reachable-set helpers.

    methods (Test)
        function backward_and_bidirectional_return_bounds(testCase)
            % This exercises Robot.set_q_from_path_2nd by sampling q(s)=s
            % from a callback-backed Path before adding second-order limits.
            n = 7;
            [robot, cleaner] = path_sampled_robot(n);
            problem = copp.solver.reach_set2.Problem( ...
                robot, ...
                idx_s_interval=[1, n], ...
                a_boundary=[0, 0]);
            options = copp.solver.reach_set2.Options();

            reach_back = copp.solver.reach_set2.backward(problem, options);
            reach_bidir = copp.solver.reach_set2.bidirectional(problem, options);

            testCase.verifyClass(reach_back, 'copp.solver.reach_set2.ReachSet');
            testCase.verifyEqual(reach_back.len, n);
            testCase.verifyEqual(reach_bidir.len, n);
            testCase.verifySize(reach_back.a_max, [n, 1]);
            testCase.verifySize(reach_back.a_min, [n, 1]);
            testCase.verifyTrue(all(isfinite(reach_back.a_max)));
            testCase.verifyTrue(all(isfinite(reach_back.a_min)));
            testCase.verifyTrue(all(reach_back.a_max >= reach_back.a_min - 1.0e-10));
            testCase.verifyEqual(reach_back.a_max(end), 0.0, AbsTol=1.0e-9);
            testCase.verifyEqual(reach_back.a_min(end), 0.0, AbsTol=1.0e-9);
            testCase.verifyEqual(reach_bidir.a_max(1), 0.0, AbsTol=1.0e-9);
            testCase.verifyEqual(reach_bidir.a_min(1), 0.0, AbsTol=1.0e-9);
            testCase.verifyEqual(reach_bidir.a_max(end), 0.0, AbsTol=1.0e-9);
            testCase.verifyEqual(reach_bidir.a_min(end), 0.0, AbsTol=1.0e-9);

            clear cleaner
        end

        function reach_set2_rejects_nonfinite_manual_values(testCase)
            % Manually constructed value objects should reject NaN/Inf instead
            % of storing invalid bounds that later code might trust.
            testCase.verifyError( ...
                @() copp.solver.reach_set2.ReachSet([1; Inf], [0; 0]), ...
                'copp:InvalidArgument');
            testCase.verifyError( ...
                @() copp.solver.reach_set2.ReachSet([1; 1], [0; NaN]), ...
                'copp:InvalidArgument');
        end

        function reach_set2_rejects_invalid_manual_bounds(testCase)
            % Since a=(ds/dt)^2, manually supplied reachable intervals must
            % stay nonnegative and ordered at every station.
            testCase.verifyError( ...
                @() copp.solver.reach_set2.ReachSet([1; -1.0e-6], [0; 0]), ...
                'copp:InvalidArgument');
            testCase.verifyError( ...
                @() copp.solver.reach_set2.ReachSet([1; 0.5], [0; 0.75]), ...
                'copp:InvalidArgument');

            reach = copp.solver.reach_set2.ReachSet([1; -1.0e-14], [0; -2.0e-14]);
            testCase.verifyEqual(reach.a_max(2), 0.0, AbsTol=0.0);
            testCase.verifyEqual(reach.a_min(2), 0.0, AbsTol=0.0);
        end
    end
end

function [robot, cleaner] = path_sampled_robot(n)
%PATH_SAMPLED_ROBOT Build a 1D robot by sampling q(s)=s from a Path.
robot = copp.Robot(1, n);
cleaner = onCleanup(@() robot.release());

s = linspace(0.0, 1.0, n).';
robot.append_s(s);
path = copp.Path.from_evaluator_2nd(@linear_path_2nd, dim=1, s_range=[0, 1]);
path_cleaner = onCleanup(@() path.release());
robot.set_q_from_path_2nd(path);
robot.add_velocity_limits(1, -1);
robot.add_acceleration_limits(1, -1);
clear path_cleaner
end

function [q, dq, ddq] = linear_path_2nd(s)
%LINEAR_PATH_2ND Batch evaluator for q(s)=s.
s = reshape(double(s), 1, []);
q = s;
dq = ones(1, numel(s));
ddq = zeros(1, numel(s));
end
