classdef test_path_waypoints < matlab.unittest.TestCase
    %TEST_PATH_WAYPOINTS Public waypoint Path construction examples.

    methods (Test)
        function waypoint_path_metadata_and_evaluation(testCase)
            % Waypoints are dim-by-N. Each column stores one q sample.
            waypoints = [ ...
                0.0, 0.5, 1.0, 1.5, 2.0; ...
                0.0, 1.0, 0.0, -1.0, 0.0];

            path = copp.Path.from_waypoints( ...
                waypoints, ...
                s_range=[0, 2], ...
                order=3, ...
                out_of_range_mode="clamp");
            cleaner = onCleanup(@() path.release());

            [q, dq, ddq] = path.evaluate_up_to_2nd([0, 1, 2]);

            testCase.verifyTrue(path.is_valid());
            testCase.verifyEqual(path.dim, 2);
            testCase.verifyEqual(path.s_range, [0, 2], AbsTol=1e-12);
            testCase.verifySize(q, [2, 3]);
            testCase.verifySize(dq, [2, 3]);
            testCase.verifySize(ddq, [2, 3]);
            testCase.verifyEqual(q(:, 1), waypoints(:, 1), AbsTol=1e-10);
            testCase.verifyEqual(q(:, end), waypoints(:, end), AbsTol=1e-10);
            testCase.verifyTrue(all(isfinite(q), 'all'));
            testCase.verifyTrue(all(isfinite(dq), 'all'));
            testCase.verifyTrue(all(isfinite(ddq), 'all'));

            clear cleaner
            testCase.verifyFalse(path.is_valid());
        end
    end
end
