classdef test_error_handling < matlab.unittest.TestCase
    %TEST_ERROR_HANDLING CoppError facade over MEX-raised MException values.

    methods (Test)
        function copp_error_wraps_native_exception(testCase)
            robot = copp.Robot(1, 2);
            cleaner = onCleanup(@() robot.release());

            try
                robot.append_s([0.0, 0.0]);
                testCase.verifyFail("append_s should have rejected non-increasing stations.");
            catch err
                wrapped = copp.diag.CoppError.from_exception(err);
                testCase.verifyTrue(copp.diag.is_copp_error(err));
                testCase.verifyTrue(copp.diag.CoppError.is_copp_error(wrapped));
                testCase.verifyEqual(wrapped.identifier, "copp:ConstraintError");
                testCase.verifyEqual(wrapped.category, "ConstraintError");
                testCase.verifyNotEmpty(wrapped.message);

                last = copp.diag.last_error();
                testCase.verifyTrue(isstruct(last));
                testCase.verifyTrue(isfield(last, 'native_status'));
                testCase.verifyTrue(isfield(last, 'status'));
                testCase.verifyTrue(isfield(last, 'detail'));
                testCase.verifyGreaterThan(last.native_status, 0);
                testCase.verifyNotEmpty(char(last.status));
            end

            clear cleaner
        end
    end
end
