function info = last_error()
%LAST_ERROR Return the native COPP last-error diagnostic snapshot.
%
% INFO = copp.diag.last_error() returns a struct with fields:
%   native_status:
%       Numeric CoppStatus code currently stored by the native C ABI.
%   status:
%       Short native status message for native_status.
%   detail:
%       Detailed native error message, when available.
%
% Ordinary MATLAB users usually handle COPP failures with try/catch and
% copp.diag.CoppError.from_exception(err). This helper is mainly useful
% for diagnostics after calling lower-level MEX-backed APIs.

[native_status, status, detail] = copp.internal.copp_mex('last_error');
info = struct( ...
    'native_status', native_status, ...
    'status', string(status), ...
    'detail', string(detail));
end
