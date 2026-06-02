function objective = time(weight)
%TIME Construct a traversal-time objective descriptor.
%
% OBJ = copp.objective.time(WEIGHT) returns a lightweight MATLAB
% descriptor for the COPP time objective. COPP solvers minimize the weighted
% sum of objective terms, so WEIGHT must be finite and nonnegative.
%
% The descriptor is MATLAB-owned data. It carries no vector payloads:
% alpha, beta, and normalize are empty 0-by-1 double columns. Solver Problem
% objects borrow it only while packing a native C ABI CoppObjective array for
% a solve call.
arguments
    weight (1,1) double {mustBeReal, mustBeFinite, mustBeNonnegative}
end

objective = struct( ...
    'kind', "time", ...
    'kind_code', 0.0, ...
    'weight', double(weight), ...
    'alpha', zeros(0, 1), ...
    'beta', zeros(0, 1), ...
    'normalize', zeros(0, 1));
end
