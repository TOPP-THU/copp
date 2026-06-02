classdef Result
    %RESULT COPP2-SOCP expert result with Clarabel diagnostics.
    %
    % Result is returned by solve_expert(). The accepted a(s) profile is stored
    % in A only when HAS_A is true. A is an s_len-by-1 double column vector.
    % Raw Clarabel vectors X, Z, and S are also stored as double column
    % vectors. Diagnostics are populated regardless of whether the final solver
    % status is accepted.

    properties (SetAccess = private)
        %HAS_A Whether A contains an accepted a(s) profile.
        has_a

        %A Accepted s_len-by-1 node profile a=(ds/dt)^2, or [].
        a

        %X Raw Clarabel primal solution vector, stored as a column.
        x

        %Z Raw Clarabel dual solution vector, stored as a column.
        z

        %S Raw Clarabel slack solution vector, stored as a column.
        s

        %SOLVER_STATUS Clarabel status as a lower-snake-case string.
        solver_status

        %OBJ_VAL Raw Clarabel primal objective value.
        obj_val

        %OBJ_VAL_DUAL Raw Clarabel dual objective value.
        obj_val_dual

        %SOLVE_TIME Clarabel-reported solve time in seconds.
        solve_time

        %ITERATIONS Number of Clarabel iterations.
        iterations

        %R_PRIM Final primal residual reported by Clarabel.
        r_prim

        %R_DUAL Final dual residual reported by Clarabel.
        r_dual

        %LINSOLVER Struct with method, threads, direct, nnz_a, and nnz_l.
        linsolver

        %OBJECTIVE_VALUE Weighted COPP objective value, or [] when unavailable.
        objective_value

        %OBJECTIVE_TERMS Per-objective unweighted values as a column, or [].
        objective_terms
    end

    methods
        function obj = Result(has_a, a, x, z, s, solver_status, obj_val, ...
                obj_val_dual, solve_time, iterations, r_prim, r_dual, ...
                linsolver, objective_value, objective_terms)
            %RESULT Construct a MATLAB-owned expert result.
            %
            % Vector inputs may be row or column vectors. Stored vector
            % properties are normalized to double columns.
            arguments
                has_a (1,1) logical
                a {mustBeNumeric, mustBeReal, mustBeVector}
                x {mustBeNumeric, mustBeReal, mustBeVector}
                z {mustBeNumeric, mustBeReal, mustBeVector}
                s {mustBeNumeric, mustBeReal, mustBeVector}
                solver_status (1,1) string
                obj_val (1,1) {mustBeNumeric, mustBeReal}
                obj_val_dual (1,1) {mustBeNumeric, mustBeReal}
                solve_time (1,1) {mustBeNumeric, mustBeReal}
                iterations (1,1) {mustBeNumeric, mustBeReal, mustBeNonnegative}
                r_prim (1,1) {mustBeNumeric, mustBeReal}
                r_dual (1,1) {mustBeNumeric, mustBeReal}
                linsolver (1,1) struct
                objective_value (1,1) {mustBeNumeric, mustBeReal}
                objective_terms {mustBeNumeric, mustBeReal, mustBeVector}
            end

            obj.has_a = has_a;
            if has_a
                obj.a = reshape(double(a), [], 1);
            else
                obj.a = [];
            end
            obj.x = reshape(double(x), [], 1);
            obj.z = reshape(double(z), [], 1);
            obj.s = reshape(double(s), [], 1);
            obj.solver_status = solver_status;
            obj.obj_val = double(obj_val);
            obj.obj_val_dual = double(obj_val_dual);
            obj.solve_time = double(solve_time);
            obj.iterations = double(iterations);
            obj.r_prim = double(r_prim);
            obj.r_dual = double(r_dual);
            obj.linsolver = linsolver;

            if has_a
                obj.objective_value = double(objective_value);
                obj.objective_terms = reshape(double(objective_terms), [], 1);
            else
                obj.objective_value = [];
                obj.objective_terms = [];
            end
        end
    end
end
