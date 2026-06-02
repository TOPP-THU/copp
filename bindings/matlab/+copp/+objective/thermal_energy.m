function objective = thermal_energy(weight, normalize)
%THERMAL_ENERGY Construct a thermal-energy objective descriptor.
%
% OBJ = copp.objective.thermal_energy(WEIGHT, NORMALIZE) returns a torque
% energy style objective. NORMALIZE is a per-axis nonnegative vector with
% length dim=robot.dim; it may be 1-by-dim or dim-by-1 and is stored as a
% dim-by-1 double column. If no inverse-dynamics callback is installed on the
% Robot, the native solver uses point dynamics tau = ddq.
arguments
    weight (1,1) double {mustBeReal, mustBeFinite, mustBeNonnegative}
    normalize {mustBeNumeric, mustBeReal, mustBeFinite, mustBeNonnegative}
end

if isempty(normalize) || ~isvector(normalize)
    error("copp:InvalidArgument", "thermal_energy objective normalize must be a nonempty vector.");
end

objective = struct( ...
    'kind', "thermal_energy", ...
    'kind_code', 2.0, ...
    'weight', double(weight), ...
    'alpha', zeros(0, 1), ...
    'beta', zeros(0, 1), ...
    'normalize', double(normalize(:)));
end
