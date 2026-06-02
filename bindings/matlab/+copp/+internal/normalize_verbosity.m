function value = normalize_verbosity(value)
%NORMALIZE_VERBOSITY Convert string/char/enum verbosity to lower-snake string.

raw = string(value);
if strlength(raw) == 0
    error("copp:InvalidArgument", "verbosity must not be empty.");
end
parts = split(raw, ".");
raw = lower(parts(end));

valid = ["silent", "summary", "debug", "trace"];
if ~any(raw == valid)
    error("copp:InvalidArgument", ...
        "verbosity must be 'silent', 'summary', 'debug', or 'trace'.");
end
value = raw;
end
