function value = version()
%VERSION Return the native COPP library version.
%
% VALUE = copp.version() returns the version string compiled into the
% linked native copp library. This is useful for smoke tests because it
% exercises the MEX gateway without creating native handles.
value = string(copp.internal.copp_mex('version'));
end
