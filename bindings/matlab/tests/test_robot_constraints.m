classdef test_robot_constraints < matlab.unittest.TestCase
    %TEST_ROBOT_CONSTRAINTS Advanced Robot constraint and dynamics APIs.

    methods (Test)
        function robot_accepts_capacity_name_value(testCase)
            % Capacity can be supplied positionally or as a MATLAB name-value.
            robot = copp.Robot(2, Capacity=5);
            cleaner = onCleanup(@() robot.release());

            testCase.verifyEqual(robot.dim, 2);
            testCase.verifyEqual(robot.len, 0);
            testCase.verifyGreaterThanOrEqual(robot.capacity, 5);

            clear cleaner
        end

        function torque_limits_and_inverse_dynamics_callback(testCase)
            n = 5;
            robot = simple_second_order_robot(n);
            cleaner = onCleanup(@() robot.release());

            robot.add_torque_limits(10 * ones(robot.dim, 1), -10 * ones(robot.dim, 1));
            robot.set_inverse_dynamics(@(q, dq, ddq) ddq + 0.1 * dq + 0.0 * q);
            robot.add_torque_limits(20 * ones(robot.dim, n), -20 * ones(robot.dim, n));
            robot.clear_inverse_dynamics();
            robot.constraints.add_torque_limits(30 * ones(robot.dim, 1), -30 * ones(robot.dim, 1));

            testCase.verifyEqual(robot.len, n);
            clear cleaner
        end

        function raw_constraints_pop_and_clear(testCase)
            n = 6;
            robot = simple_third_order_robot(n);
            cleaner = onCleanup(@() robot.release());

            robot.add_raw_constraint_1st(100 * ones(2, n));
            robot.add_raw_constraint_2nd(zeros(2, n), zeros(2, n), 100 * ones(2, n));
            robot.add_raw_constraint_3rd(zeros(2, n), zeros(2, n), zeros(2, n), zeros(2, n), 100 * ones(2, n));

            robot.pop_front_n(1);
            testCase.verifyEqual(robot.len, n - 1);
            robot.pop_back_n(2);
            testCase.verifyEqual(robot.len, n - 3);
            robot.clear_constraints();
            testCase.verifyEqual(robot.len, 0);

            clear cleaner
        end
    end
end

function robot = simple_second_order_robot(n)
robot = copp.Robot(2, n);
s = linspace(0.0, 1.0, n).';
q = [s.'; 2 * s.'];
dq = repmat([1; 2], 1, n);
ddq = zeros(2, n);
robot.append_s(s);
robot.set_q_2nd(q, dq, ddq);
robot.add_velocity_limits(100 * ones(2, 1), -100 * ones(2, 1));
robot.add_acceleration_limits(100 * ones(2, 1), -100 * ones(2, 1));
end

function robot = simple_third_order_robot(n)
robot = simple_second_order_robot(n);
q = zeros(2, n);
dq = ones(2, n);
ddq = zeros(2, n);
dddq = zeros(2, n);
robot.set_q_3rd(q, dq, ddq, dddq);
robot.add_jerk_limits(100 * ones(2, 1), -100 * ones(2, 1));
end
