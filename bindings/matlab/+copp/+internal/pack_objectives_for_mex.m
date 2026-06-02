function packed = pack_objectives_for_mex(objectives)
%PACK_OBJECTIVES_FOR_MEX Normalize MATLAB objective descriptors for MEX.
%
% PACKED = copp.internal.pack_objectives_for_mex(OBJECTIVES) accepts one
% descriptor, a struct array of descriptors, or a cell array of descriptors.
% Descriptors are normally created by copp.objective.* factory functions.
%
% The returned scalar struct owns concatenated alpha/beta/normalize payloads and
% per-objective metadata arrays. MEX uses those arrays to build temporary
% CoppObjective descriptors for exactly one native solver call.

if iscell(objectives)
    items = reshape(objectives, 1, []);
elseif isstruct(objectives)
    raw = num2cell(objectives);
    items = reshape(raw, 1, []);
else
    error( ...
        "copp:InvalidArgument", ...
        "objectives must be an objective descriptor, a struct array, or a cell array.");
end

count = numel(items);
if count == 0
    error("copp:InvalidArgument", "objectives must not be empty.");
end

kinds = zeros(1, count);
weights = zeros(1, count);
alpha_lengths = zeros(1, count);
beta_lengths = zeros(1, count);
normalize_lengths = zeros(1, count);
alpha_data = zeros(0, 1);
beta_data = zeros(0, 1);
normalize_data = zeros(0, 1);
normalized_items = cell(1, count);
kind_names = strings(1, count);

for i = 1:count
    objective = normalize_objective(items{i}, i);

    kinds(i) = objective.kind_code;
    weights(i) = objective.weight;
    alpha_lengths(i) = numel(objective.alpha);
    beta_lengths(i) = numel(objective.beta);
    normalize_lengths(i) = numel(objective.normalize);
    alpha_data = [alpha_data; objective.alpha]; %#ok<AGROW>
    beta_data = [beta_data; objective.beta]; %#ok<AGROW>
    normalize_data = [normalize_data; objective.normalize]; %#ok<AGROW>
    normalized_items{i} = objective;
    kind_names(i) = objective.kind;
end

packed = struct();
packed.objectives = normalized_items;
packed.kind_names = kind_names;
packed.kinds = kinds;
packed.weights = weights;
packed.alpha_lengths = alpha_lengths;
packed.beta_lengths = beta_lengths;
packed.normalize_lengths = normalize_lengths;
packed.alpha_data = alpha_data;
packed.beta_data = beta_data;
packed.normalize_data = normalize_data;
packed.count = count;
end

function objective = normalize_objective(objective, index)
%NORMALIZE_OBJECTIVE Validate one objective descriptor struct.
required_fields = ["kind", "kind_code", "weight", "alpha", "beta", "normalize"];
if ~isstruct(objective) || numel(objective) ~= 1
    error( ...
        "copp:InvalidArgument", ...
        "objectives{%d} must be a scalar objective descriptor struct.", ...
        index);
end
for field = required_fields
    field_name = char(field);
    if ~isfield(objective, field_name)
        error( ...
            "copp:InvalidArgument", ...
            "objectives{%d} is missing required field '%s'.", ...
            index, ...
            field_name);
    end
end

kind_code = double(objective.kind_code);
if ~isscalar(kind_code) || ~isfinite(kind_code) || kind_code < 0 || kind_code > 3 || fix(kind_code) ~= kind_code
    error( ...
        "copp:InvalidArgument", ...
        "objectives{%d}.kind_code must be an integer in [0, 3].", ...
        index);
end

weight = double(objective.weight);
if ~isscalar(weight) || ~isfinite(weight) || weight < 0
    error( ...
        "copp:InvalidArgument", ...
        "objectives{%d}.weight must be finite and nonnegative.", ...
        index);
end

alpha = vector_payload(objective.alpha, "alpha", index, false);
beta = vector_payload(objective.beta, "beta", index, false);
normalize = vector_payload(objective.normalize, "normalize", index, true);

objective = struct( ...
    'kind', string(objective.kind), ...
    'kind_code', kind_code, ...
    'weight', weight, ...
    'alpha', alpha, ...
    'beta', beta, ...
    'normalize', normalize);
end

function values = vector_payload(values, name, index, require_nonnegative)
%VECTOR_PAYLOAD Convert row/column payloads into finite column vectors.
if isempty(values)
    values = zeros(0, 1);
    return
end
if ~isnumeric(values) || ~isreal(values) || ~isvector(values)
    error( ...
        "copp:InvalidArgument", ...
        "objectives{%d}.%s must be a real vector.", ...
        index, ...
        name);
end
values = double(values(:));
if any(~isfinite(values))
    error( ...
        "copp:InvalidArgument", ...
        "objectives{%d}.%s must contain only finite values.", ...
        index, ...
        name);
end
if require_nonnegative && any(values < 0)
    error( ...
        "copp:InvalidArgument", ...
        "objectives{%d}.%s must contain only nonnegative values.", ...
        index, ...
        name);
end
end
