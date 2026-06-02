function name = clarabel_solver_status_name(code)
%CLARABEL_SOLVER_STATUS_NAME Convert native Clarabel status code to string.

names = [
    "unsolved"
    "solved"
    "primal_infeasible"
    "dual_infeasible"
    "almost_solved"
    "almost_primal_infeasible"
    "almost_dual_infeasible"
    "max_iterations"
    "max_time"
    "numerical_error"
    "insufficient_progress"
    "callback_terminated"
    ];

idx = double(code) + 1;
if idx >= 1 && idx <= numel(names) && idx == floor(idx)
    name = names(idx);
else
    name = "unknown";
end
end
