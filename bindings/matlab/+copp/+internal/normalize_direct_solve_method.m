function value = normalize_direct_solve_method(value)
%NORMALIZE_DIRECT_SOLVE_METHOD Convert direct-solver method to native string.

raw = string(value);
if strlength(raw) == 0
    error("copp:InvalidArgument", "direct_solve_method must not be empty.");
end
parts = split(raw, ".");
raw = lower(parts(end));
if raw == "panua_pardiso"
    raw = "panua";
end

valid = ["auto", "qdldl", "faer", "mkl", "panua"];
if ~any(raw == valid)
    error("copp:InvalidArgument", ...
        "direct_solve_method must be 'auto', 'qdldl', 'faer', 'mkl', or 'panua'.");
end
value = raw;
end
