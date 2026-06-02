classdef test_version < matlab.unittest.TestCase
    %TEST_VERSION Minimal public package smoke test.

    methods (Test)
        function version_returns_nonempty_string(testCase)
            % copp.version is the cheapest way to check that the MATLAB
            % package can load the MEX gateway and call the native library.
            value = copp.version();

            testCase.verifyClass(value, 'string');
            testCase.verifyNotEmpty(char(value));
        end
    end
end
