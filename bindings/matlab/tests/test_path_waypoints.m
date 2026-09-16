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

        function interpolating_constructor_matches_alias(testCase)
            % from_waypoints is an equivalent alias of the interpolating
            % constructor, and interpolated paths carry no smoothing report.
            waypoints = [ ...
                0.0, 0.5, 1.0, 1.5, 2.0; ...
                0.0, 1.0, 0.0, -1.0, 0.0];

            path = copp.Path.from_waypoints_interpolating(waypoints, s_range=[0, 2]);
            cleaner = onCleanup(@() path.release());
            alias = copp.Path.from_waypoints(waypoints, s_range=[0, 2]);
            alias_cleaner = onCleanup(@() alias.release());

            s = linspace(0, 2, 9);
            testCase.verifyEqual(path.evaluate_q(s), alias.evaluate_q(s), AbsTol=1e-12);
            testCase.verifyEqual(path.evaluate_q([0, 0.5, 1, 1.5, 2]), waypoints, AbsTol=1e-10);
            testCase.verifyEmpty(path.smoothing_report());

            clear cleaner alias_cleaner
        end

        function fitting_constructor_reports_tolerance_bounds(testCase)
            % Fitted paths stay within tolerance of the waypoint polyline and
            % keep the first and last waypoints.
            waypoints = [ ...
                0.0, 0.25, 0.5, 0.75, 1.0; ...
                0.0, 0.4, 0.5, 0.4, 0.0];
            tolerance = 1.0e-3;

            path = copp.Path.from_waypoints_fitting(waypoints, tolerance=tolerance);
            cleaner = onCleanup(@() path.release());
            report = path.smoothing_report();

            testCase.verifyTrue(isstruct(report));
            testCase.verifyEqual(report.axes, [1; 2]);
            testCase.verifySize(report.max_errors, [2, 1]);
            testCase.verifyLessThanOrEqual(report.max_errors, tolerance);
            testCase.verifyGreaterThan(report.segments, 0);
            testCase.verifyEqual(report.interpolated_segments, 0);
            testCase.verifyGreaterThanOrEqual(report.refinements, 0);
            testCase.verifyGreaterThan(report.fitting_rows, 0);
            testCase.verifyGreaterThan(report.checked_intervals, 0);
            testCase.verifyEqual(path.dim, 2);
            testCase.verifyEqual(path.s_range, [0, 1], AbsTol=1e-12);

            q = path.evaluate_q(linspace(0, 1, 5));
            testCase.verifyEqual(q(:, [1, end]), waypoints(:, [1, end]), AbsTol=1e-12);
            testCase.verifyLessThanOrEqual(abs(q - waypoints), tolerance + 1e-12);
            [~, ~, ~, dddq] = path.evaluate_up_to_3rd([0.1, 0.6]);
            testCase.verifyTrue(all(isfinite(dddq), 'all'));

            clear cleaner
        end

        function fitting_constructor_selects_axes_and_parameters(testCase)
            % axes are 1-based rows; vector tolerances follow axes order.
            waypoints = [ ...
                0.0, 0.25, 0.5, 0.75, 1.0; ...
                0.0, 0.4, 0.5, 0.4, 0.0];

            path = copp.Path.from_waypoints_fitting( ...
                waypoints, ...
                tolerance=[1.0e-2, 1.0e-3], ...
                axes=[2, 1], ...
                parameters=[0, 0.4, 1.0, 1.6, 2.0]);
            cleaner = onCleanup(@() path.release());
            report = path.smoothing_report();

            testCase.verifyEqual(report.axes, [2; 1]);
            testCase.verifyLessThanOrEqual(report.max_errors(1), 1.0e-2);
            testCase.verifyLessThanOrEqual(report.max_errors(2), 1.0e-3);
            testCase.verifyEqual(path.s_range, [0, 2], AbsTol=1e-12);

            single = copp.Path.from_waypoints_fitting(waypoints, axes=2);
            single_cleaner = onCleanup(@() single.release());
            single_report = single.smoothing_report();
            testCase.verifyEqual(single_report.axes, 2);
            testCase.verifySize(single_report.max_errors, [1, 1]);
            testCase.verifyGreaterThan(single_report.interpolated_segments, 0);

            % The unselected row still interpolates every waypoint.
            q = single.evaluate_q(linspace(0, 1, 5));
            testCase.verifyEqual(q(1, :), waypoints(1, :), AbsTol=1e-10);

            clear cleaner single_cleaner
        end

        function fitting_constructor_failures_raise_path_error(testCase)
            % Native fitting contract violations surface as PathError.
            waypoints = [ ...
                0.0, 0.25, 0.5, 0.75, 1.0; ...
                0.0, 0.4, 0.5, 0.4, 0.0];

            testCase.verifyError( ...
                @() copp.Path.from_waypoints_fitting( ...
                    waypoints, parameters=[0, 0.5, 0.4, 0.8, 1.0]), ...
                'copp:PathError');
            testCase.verifyError( ...
                @() copp.Path.from_waypoints_fitting(waypoints, axes=3), ...
                'copp:PathError');
            testCase.verifyError( ...
                @() copp.Path.from_waypoints_fitting( ...
                    waypoints, tolerance=[1.0e-3, 1.0e-3, 1.0e-3]), ...
                'copp:PathError');
        end
    end
end
