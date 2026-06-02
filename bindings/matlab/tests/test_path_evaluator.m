classdef test_path_evaluator < matlab.unittest.TestCase
    %TEST_PATH_EVALUATOR Public examples for callback-backed Path objects.
    %
    % These tests intentionally use only copp.Path public APIs. They show
    % the two supported MATLAB evaluator styles:
    %   1. a function handle, useful for compact scripts and demos;
    %   2. an object evaluator, useful when the path owns parameters or state.
    %
    % MATLAB callback inputs are always 1-by-N row vectors. Callback outputs
    % are dim-by-N matrices: one column per path sample.

    methods (Test)
        function function_handle_second_order_evaluator(testCase)
            % A function-handle evaluator returns q, dq, and ddq.
            path = copp.Path.from_evaluator_2nd( ...
                @quadratic_2nd, ...
                dim=2, ...
                s_range=[-1, 1]);
            cleaner = onCleanup(@() path.release());

            % Users may pass row or column s vectors into evaluate_* methods.
            % The native callback receives a normalized row vector.
            s = [-1, 0, 0.5];
            [q, dq, ddq] = path.evaluate_up_to_2nd(s);

            expected_q = [s; s.^2 + 1.0];
            expected_dq = [ones(size(s)); 2.0 .* s];
            expected_ddq = [zeros(size(s)); 2.0 .* ones(size(s))];

            testCase.verifyEqual(path.dim, 2);
            testCase.verifyEqual(path.s_range, [-1, 1], AbsTol=1e-12);
            testCase.verifyEqual(q, expected_q, AbsTol=1e-12);
            testCase.verifyEqual(dq, expected_dq, AbsTol=1e-12);
            testCase.verifyEqual(ddq, expected_ddq, AbsTol=1e-12);

            % One-output form returns a struct with named derivative fields.
            out = path.evaluate_up_to_2nd(s.');
            testCase.verifyEqual(out.q, expected_q, AbsTol=1e-12);
            testCase.verifyEqual(out.dq, expected_dq, AbsTol=1e-12);
            testCase.verifyEqual(out.ddq, expected_ddq, AbsTol=1e-12);

            % A second-order evaluator path deliberately does not support
            % third-order evaluation.
            testCase.verifyError( ...
                @() path.evaluate_up_to_3rd(s), ...
                'copp:PathError');

            clear cleaner
            testCase.verifyFalse(path.is_valid());
        end

        function object_third_order_evaluator_uses_optional_second_order_method(testCase)
            % An object evaluator can keep parameters and counters as state.
            test_dir = fileparts(mfilename('fullpath'));
            path_cleaner = onCleanup(@() rmpath(test_dir));
            addpath(test_dir);

            evaluator = QuadraticPathEvaluator();
            path = copp.Path.from_evaluator_3rd( ...
                evaluator, ...
                dim=2, ...
                s_range=[0, 1]);
            native_cleaner = onCleanup(@() path.release());

            s = [0, 0.25, 1.0];
            [q, dq, ddq, dddq] = path.evaluate_up_to_3rd(s);

            testCase.verifyEqual(evaluator.calls_3rd, 1);
            testCase.verifyEqual(q(1, :), s.^3, AbsTol=1e-12);
            testCase.verifyEqual(dq(1, :), 3.0 .* s.^2, AbsTol=1e-12);
            testCase.verifyEqual(ddq(1, :), 6.0 .* s, AbsTol=1e-12);
            testCase.verifyEqual(dddq(1, :), 6.0 .* ones(size(s)), AbsTol=1e-12);

            % Because the object also implements evaluate_up_to_2nd, the
            % native path uses that lighter callback for second-order queries.
            out = path.evaluate_up_to_2nd(s);
            testCase.verifyEqual(evaluator.calls_2nd, 1);
            testCase.verifyEqual(evaluator.calls_3rd, 1);
            testCase.verifyEqual(out.q, q, AbsTol=1e-12);

            clear native_cleaner
            clear path_cleaner
        end

        function function_handle_third_order_falls_back_for_second_order(testCase)
            % When a third-order function handle is evaluated only up to 2nd
            % order, the native path calls the third-order callback and drops
            % dddq internally.
            path = copp.Path.from_evaluator_3rd( ...
                @quadratic_3rd, ...
                dim=2, ...
                s_range=[-1, 1]);
            cleaner = onCleanup(@() path.release());

            s = [-0.5, 0.0, 0.5];
            out = path.evaluate_up_to_2nd(s);

            testCase.verifyEqual(out.q(1, :), s.^3, AbsTol=1e-12);
            testCase.verifyEqual(out.dq(1, :), 3.0 .* s.^2, AbsTol=1e-12);
            testCase.verifyEqual(out.ddq(1, :), 6.0 .* s, AbsTol=1e-12);

            clear cleaner
        end
    end
end

function [q, dq, ddq] = quadratic_2nd(s)
%QUADRATIC_2ND Two-dimensional second-order path evaluator for tests.
assert(isrow(s), 'test_path_evaluator:ExpectedRowS', 'callback input s must be a row vector');
q = [s; s.^2 + 1.0];
dq = [ones(size(s)); 2.0 .* s];
ddq = [zeros(size(s)); 2.0 .* ones(size(s))];
end

function [q, dq, ddq, dddq] = quadratic_3rd(s)
%QUADRATIC_3RD Two-dimensional third-order path evaluator for tests.
assert(isrow(s), 'test_path_evaluator:ExpectedRowS', 'callback input s must be a row vector');
q = [s.^3; s.^2 + 1.0];
dq = [3.0 .* s.^2; 2.0 .* s];
ddq = [6.0 .* s; 2.0 .* ones(size(s))];
dddq = [6.0 .* ones(size(s)); zeros(size(s))];
end
