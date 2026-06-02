function name = clarabel_direct_solve_method_name(code)
%CLARABEL_DIRECT_SOLVE_METHOD_NAME Convert native direct-solver code to string.

names = ["auto", "qdldl", "faer", "mkl", "panua"];
idx = double(code) + 1;
if idx >= 1 && idx <= numel(names) && idx == floor(idx)
    name = names(idx);
else
    name = "unknown";
end
end
