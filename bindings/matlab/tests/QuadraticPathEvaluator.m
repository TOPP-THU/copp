classdef QuadraticPathEvaluator < handle
    %QUADRATICPATHEVALUATOR Test helper object for Path.from_evaluator_3rd.
    %
    % This helper intentionally lives in tests/ rather than the public package.
    % It demonstrates the object-evaluator protocol expected from user code:
    % evaluate_up_to_3rd(s) is required for a third-order path, while
    % evaluate_up_to_2nd(s) is optional and used as a lighter second-order
    % callback when present.

    properties
        %CALLS_2ND Number of evaluate_up_to_2nd callback invocations.
        calls_2nd = 0

        %CALLS_3RD Number of evaluate_up_to_3rd callback invocations.
        calls_3rd = 0
    end

    methods
        function [q, dq, ddq] = evaluate_up_to_2nd(obj, s)
            %EVALUATE_UP_TO_2ND Return q, dq, and ddq for a batch of samples.
            assert(isrow(s), 'QuadraticPathEvaluator:ExpectedRowS', 'callback input s must be a row vector');
            obj.calls_2nd = obj.calls_2nd + 1;

            q = [s.^3; s.^2 + 1.0];
            dq = [3.0 .* s.^2; 2.0 .* s];
            ddq = [6.0 .* s; 2.0 .* ones(size(s))];
        end

        function [q, dq, ddq, dddq] = evaluate_up_to_3rd(obj, s)
            %EVALUATE_UP_TO_3RD Return q, dq, ddq, and dddq for samples.
            assert(isrow(s), 'QuadraticPathEvaluator:ExpectedRowS', 'callback input s must be a row vector');
            obj.calls_3rd = obj.calls_3rd + 1;

            q = [s.^3; s.^2 + 1.0];
            dq = [3.0 .* s.^2; 2.0 .* s];
            ddq = [6.0 .* s; 2.0 .* ones(size(s))];
            dddq = [6.0 .* ones(size(s)); zeros(size(s))];
        end
    end
end
