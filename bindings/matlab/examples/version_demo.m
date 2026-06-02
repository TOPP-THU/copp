%VERSION_DEMO Print the native COPP version through the MATLAB facade.
%
% This is intentionally tiny: it is the quickest check that MATLAB can find
% the package directory and load the generated MEX gateway.

ExampleCommon.setup_path();

fprintf("COPP native version: %s\n", copp.version());
