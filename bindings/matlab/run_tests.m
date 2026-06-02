function results = run_tests(varargin)
%RUN_TESTS Run MATLAB binding targeted tests.
%
% RESULTS = run_tests() runs all tests under bindings/matlab/tests.
% RESULTS = run_tests(NAME) forwards NAME to runtests, for example
% run_tests("tests/test_copp2_socp.m").

root = fileparts(mfilename('fullpath'));
addpath(root);

if nargin == 0
    results = runtests(fullfile(root, 'tests'));
else
    results = runtests(varargin{:});
end

disp(table(results));
assert(~any([results.Failed]), "copp:TestFailure", "One or more MATLAB tests failed.");
end
