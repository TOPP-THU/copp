function objective = linear(weight, alpha, beta)
%LINEAR Construct a linear objective over COPP profile variables.
%
% OBJ = copp.objective.linear(WEIGHT, ALPHA, BETA) returns a descriptor
% with node coefficients ALPHA and edge/node coefficients BETA. ALPHA and BETA
% may be row or column vectors; the descriptor stores both as double column
% vectors.
%
% For a COPP2 problem covering S_LEN stations, ALPHA must contain S_LEN values
% and BETA must contain S_LEN - 1 values. For COPP3, both ALPHA and BETA must
% contain S_LEN values. Problem construction validates the active length
% contract against the chosen robot interval.
arguments
    weight (1,1) double {mustBeReal, mustBeFinite, mustBeNonnegative}
    alpha {mustBeNumeric, mustBeReal, mustBeFinite}
    beta {mustBeNumeric, mustBeReal, mustBeFinite}
end

if ~isvector(alpha)
    error("copp:InvalidArgument", "linear objective alpha must be a vector.");
end
if ~isvector(beta)
    error("copp:InvalidArgument", "linear objective beta must be a vector.");
end

objective = struct( ...
    'kind', "linear", ...
    'kind_code', 1.0, ...
    'weight', double(weight), ...
    'alpha', double(alpha(:)), ...
    'beta', double(beta(:)), ...
    'normalize', zeros(0, 1));
end
