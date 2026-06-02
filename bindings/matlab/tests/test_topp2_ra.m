classdef test_topp2_ra < matlab.unittest.TestCase
    %TEST_TOPP2_RA Smoke tests that double as user-facing MATLAB examples.
    %
    % These tests intentionally use only public MATLAB APIs. They document the
    % first supported end-to-end workflow:
    %   1. query the native library version,
    %   2. construct and inspect a waypoint Path,
    %   3. construct a Robot and fill its station-indexed data,
    %   4. add velocity and acceleration limits,
    %   5. solve a TOPP2-RA problem.
    %
    % The examples use MATLAB's public 1-based station indices. The MEX gateway
    % converts those indices to the native 0-based representation.

    methods (Test)
        function version_and_path_metadata(testCase)
            %VERSION_AND_PATH_METADATA documents basic package and Path usage.
            %
            % copp.version() is the cheapest smoke test because it exercises
            % the MEX gateway without allocating native Path/Robot resources.
            value = copp.version();
            testCase.verifyClass(value, 'string');
            testCase.verifyNotEmpty(char(value));

            % Waypoints are dim-by-N. This two-dimensional example has three
            % waypoint columns over the path range s in [0, 1].
            path = copp.Path.from_waypoints( ...
                [0, 1, 2; 0, 1, 0], ...
                s_range=[0, 1]);
            cleaner = onCleanup(@() path.release());

            % Metadata properties query the native path handle through the
            % private MEX registry. Users do not see native pointers.
            testCase.verifyTrue(path.is_valid());
            testCase.verifyEqual(path.dim, 2);
            testCase.verifyEqual(path.s_range, [0, 1], AbsTol=1e-12);

            % Explicit release is optional in normal code, but the test checks
            % the handle lifecycle contract directly.
            clear cleaner
            testCase.verifyFalse(path.is_valid());
        end

        function topp2_ra_solves_minimal_three_point_profile(testCase)
            %TOPP2_RA_SOLVES_MINIMAL_THREE_POINT_PROFILE shows the E2E call.
            %
            % This mirrors the small C++ TOPP2-RA smoke test: a 1D path with
            % three stations, unit velocity and acceleration limits, and
            % rest-to-rest boundary values a(1)=a(3)=0.
            robot = copp.Robot(1, 3);
            cleaner = onCleanup(@() robot.release());

            % Station grid. append_s requires a strictly increasing finite
            % vector. Public station ids are now 1, 2, and 3.
            s = [0; 0.5; 1.0];
            robot.append_s(s);

            % q, dq/ds, and d2q/ds2 are dim-by-N. For q(s)=s in one dimension,
            % q = [0, 0.5, 1], dq = [1, 1, 1], and ddq = [0, 0, 0].
            robot.set_q_2nd([0, 0.5, 1.0], [1, 1, 1], [0, 0, 0]);

            % Scalar-looking one-element vectors are broadcast over the whole
            % stored station interval by default. Equivalent explicit spelling:
            %   robot.add_velocity_limits(1, -1, start_idx_s=1, length=3)
            robot.add_velocity_limits(1, -1);
            robot.add_acceleration_limits(1, -1);

            % Problem stores a borrowed reference to robot. idx_s_interval is a
            % closed 1-based MATLAB interval, so [1, 3] covers all stations.
            problem = copp.solver.topp2_ra.Problem( ...
                robot, ...
                idx_s_interval=[1, 3], ...
                a_boundary=[0, 0]);
            options = copp.solver.topp2_ra.Options();

            % solve returns a column vector of a(s) = (ds/dt)^2 values over the
            % closed interval. For this toy case the middle station reaches the
            % unit velocity bound.
            a = copp.solver.topp2_ra.solve(problem, options);

            testCase.verifyEqual(problem.s_len, 3);
            testCase.verifySize(a, [3, 1]);
            testCase.verifyEqual(a(1), 0.0, AbsTol=1e-12);
            testCase.verifyEqual(a(end), 0.0, AbsTol=1e-12);
            testCase.verifyGreaterThanOrEqual(a, -1e-10);
            testCase.verifyTrue(all(isfinite(a)));
            testCase.verifyLessThanOrEqual(a(2), 1.0 + 1e-10);

            % Releasing the robot invalidates existing Problem descriptors that
            % refer to it. This test only checks the Robot lifecycle flag.
            clear cleaner
            testCase.verifyFalse(robot.is_valid());
        end
    end
end
