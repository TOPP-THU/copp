function results = run_tests(varargin)
%RUN_TESTS Run MATLAB binding targeted tests.
%
% RESULTS = run_tests() runs every test in the tests/ folder next to this
% file.
% RESULTS = run_tests(NAME) forwards NAME to runtests verbatim, so a relative
% NAME is resolved against the current folder, not against this file. Pass an
% absolute path to be independent of where you are:
%   run_tests(fullfile(fileparts(which("run_tests")), "tests", "test_copp2_socp.m"))
%
% The test suite needs a working MEX gateway. run_tests adds its own folder to
% the MATLAB path, so running it from a tree whose +copp/+internal has no
% copp_mex.<mexext> -- a clone of TOPP-THU/copp-matlab, or a clone of
% TOPP-THU/copp before build() has run -- makes that gateway-less +copp shadow
% an installed toolbox, and every test then fails on copp.internal.copp_mex.
% Run build() first, or run the tests from a source clone.

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
