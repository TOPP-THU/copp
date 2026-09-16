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

        function exceed_topp2_reports_violations(testCase)
            % Nonpositive values mean feasible; NaN means the range is unavailable.
            n = 5;
            robot = simple_second_order_robot(n);
            cleaner = onCleanup(@() robot.release());

            [e1, e2] = robot.exceed_topp2(0.5 * ones(n, 1));
            testCase.verifyLessThanOrEqual([e1, e2], 0);

            [e1_fast, ~] = robot.exceed_topp2(1.0e6 * ones(n, 1));
            testCase.verifyGreaterThan(e1_fast, 0);

            [e1_tail, e2_tail] = robot.constraints.exceed_topp2(0.5 * ones(n - 1, 1), idx_s_start=2);
            testCase.verifyLessThanOrEqual([e1_tail, e2_tail], 0);

            [e1_nan, e2_nan] = robot.exceed_topp2(ones(n, 1), idx_s_start=2);
            testCase.verifyTrue(isnan(e1_nan) && isnan(e2_nan));

            clear cleaner
        end

        function exceed_topp3_accepts_vectors_and_profiles(testCase)
            n = 6;
            robot = simple_third_order_robot(n);
            cleaner = onCleanup(@() robot.release());
            a = ones(n, 1);
            b = zeros(n, 1);

            [e1, e2, e3] = robot.exceed_topp3(a, b);
            testCase.verifyLessThanOrEqual([e1, e2, e3], 0);

            % The jump of b inside the first and last intervals violates the
            % jerk rows unless num_stationary=[1, 1] skips those end blocks.
            b_ends = [0; 1.0e3 * ones(n - 2, 1); 0];
            [~, ~, e3_moving] = robot.exceed_topp3(a, b_ends);
            [~, ~, e3_stationary] = robot.exceed_topp3(a, b_ends, num_stationary=[1, 1]);
            testCase.verifyGreaterThan(e3_moving, 0);
            testCase.verifyLessThan(e3_stationary, e3_moving);

            % A Profile3rd supplies a, b, and num_stationary together.
            profile = copp.Profile3rd(a, b_ends, num_stationary=[1, 1]);
            [p1, p2, p3] = robot.exceed_topp3(profile);
            [v1, v2, v3] = robot.constraints.exceed_topp3(a, b_ends, num_stationary=[1, 1]);
            testCase.verifyEqual([p1, p2, p3], [v1, v2, v3]);
            testCase.verifyEqual(p3, e3_stationary);

            [~, ~, e3_fast] = robot.exceed_topp3(a, 1.0e4 * (1:n).');
            testCase.verifyGreaterThan(e3_fast, 0);

            [n1, n2, n3] = robot.exceed_topp3(a, zeros(n - 1, 1));
            testCase.verifyTrue(all(isnan([n1, n2, n3])));

            testCase.verifyError(@() robot.exceed_topp3(profile, b), 'copp:InvalidArgument');
            testCase.verifyError( ...
                @() robot.exceed_topp3(profile, num_stationary=[0, 0]), ...
                'copp:InvalidArgument');
            testCase.verifyError(@() robot.exceed_topp3(a), 'copp:InvalidArgument');

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
