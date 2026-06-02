function objective = total_variation_torque(weight, normalize)
%TOTAL_VARIATION_TORQUE Construct a torque total-variation objective.
%
% OBJ = copp.objective.total_variation_torque(WEIGHT, NORMALIZE) returns a
% COPP objective that penalizes total variation of torque. This objective is
% supported by SOCP backends. NORMALIZE is a per-axis nonnegative vector with
% length dim=robot.dim; it may be 1-by-dim or dim-by-1 and is stored as a
% dim-by-1 double column.
arguments
    weight (1,1) double {mustBeReal, mustBeFinite, mustBeNonnegative}
    normalize {mustBeNumeric, mustBeReal, mustBeFinite, mustBeNonnegative}
end

if isempty(normalize) || ~isvector(normalize)
    error("copp:InvalidArgument", "total_variation_torque objective normalize must be a nonempty vector.");
end

objective = struct( ...
    'kind', "total_variation_torque", ...
    'kind_code', 3.0, ...
    'weight', double(weight), ...
    'alpha', zeros(0, 1), ...
    'beta', zeros(0, 1), ...
    'normalize', double(normalize(:)));
end
