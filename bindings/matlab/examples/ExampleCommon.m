classdef ExampleCommon
    %EXAMPLECOMMON Shared utilities for runnable COPP MATLAB examples.
    %
    % The public examples mirror the Rust examples: a deterministic 3-axis
    % Lissajous path, N=1001 station samples on s in [0,1], symmetric
    % velocity/acceleration/jerk limits in [-1,1], and a uniform 1 ms time
    % sampling pass after solving.

    methods (Static)
        function setup_path()
            %SETUP_PATH Add the MATLAB binding and examples directory to path.
            example_dir = fileparts(mfilename('fullpath'));
            addpath(example_dir);
            addpath(fileparts(example_dir));
        end

        function ctx = second_order_context(n)
            %SECOND_ORDER_CONTEXT Build the common TOPP2/COPP2 example robot.
            if nargin < 1
                n = 1001;
            end
            ctx = ExampleCommon.base_context(n);

            % 1) Deterministic 3-axis Lissajous path q(s), s in [0,1].
            path = ExampleCommon.lissajous_path(ctx.dim);
            ctx.path = path;
            ctx.path_cleanup = onCleanup(@() path.release());

            % 2) Sample q,dq,ddq into Robot and add vel/acc limits in [-1,1].
            robot = copp.Robot(ctx.dim, Capacity=ctx.n);
            ctx.robot = robot;
            ctx.robot_cleanup = onCleanup(@() robot.release());
            ctx.robot.append_s(ctx.s);
            ctx.robot.set_q_from_path_2nd(ctx.path);
            ctx.robot.add_velocity_limits(ctx.limit_max, ctx.limit_min);
            ctx.robot.add_acceleration_limits(ctx.limit_max, ctx.limit_min);
        end

        function ctx = third_order_context(n)
            %THIRD_ORDER_CONTEXT Build the common TOPP3/COPP3 example robot.
            if nargin < 1
                n = 1001;
            end
            ctx = ExampleCommon.base_context(n);

            % 1) Deterministic 3-axis Lissajous path q(s), s in [0,1].
            path = ExampleCommon.lissajous_path(ctx.dim);
            ctx.path = path;
            ctx.path_cleanup = onCleanup(@() path.release());

            % 2) Sample q,dq,ddq,dddq and add vel/acc/jerk limits in [-1,1].
            robot = copp.Robot(ctx.dim, Capacity=ctx.n);
            ctx.robot = robot;
            ctx.robot_cleanup = onCleanup(@() robot.release());
            ctx.robot.append_s(ctx.s);
            ctx.robot.set_q_from_path_3rd(ctx.path);
            ctx.robot.add_velocity_limits(ctx.limit_max, ctx.limit_min);
            ctx.robot.add_acceleration_limits(ctx.limit_max, ctx.limit_min);
            ctx.robot.add_jerk_limits(ctx.limit_max, ctx.limit_min);
        end

        function a_seed = solve_topp2_seed(robot, n)
            %SOLVE_TOPP2_SEED Build a cheap a(s) seed for third-order examples.
            problem = copp.solver.topp2_ra.Problem( ...
                robot, ...
                idx_s_interval=[1, n], ...
                a_boundary=[0, 0]);
            options = copp.solver.topp2_ra.Options();
            a_seed = copp.solver.topp2_ra.solve(problem, options);
        end

        function objectives = convex_objectives(dim)
            %CONVEX_OBJECTIVES Use the same hybrid objective as the Rust examples.
            objectives = { ...
                copp.objective.time(1.0), ...
                copp.objective.thermal_energy(0.1, ones(dim, 1))};
        end

        function [t_final, t_s, s_t] = postprocess_topp2(ctx, a_profile)
            %POSTPROCESS_TOPP2 Convert a(s) -> t(s) -> uniform samples s(t).
            [t_final, t_s] = copp.interpolation.s_to_t_topp2(ctx.s, a_profile);
            s_t = copp.interpolation.t_to_s_topp2( ...
                ctx.s, a_profile, t_s, dt=ctx.dt, include_final=true);
        end

        function [t_final, t_s, s_t] = postprocess_topp3(ctx, profile)
            %POSTPROCESS_TOPP3 Convert Profile3rd -> t(s) -> uniform s(t).
            [t_final, t_s] = copp.interpolation.s_to_t_topp3(ctx.s, profile);
            s_t = copp.interpolation.t_to_s_topp3( ...
                ctx.s, profile, t_s, dt=ctx.dt, include_final=true);
        end

        function print_second_order(label, ctx, t_final, a_profile, s_t)
            %PRINT_SECOND_ORDER Print the common TOPP2/COPP2 summary.
            fprintf("%s done.\n", label);
            fprintf("dim = %d, N = %d\n", ctx.dim, ctx.n);
            fprintf("t_final = %.6f s\n", t_final);
            fprintf("a_profile.len() = %d\n", numel(a_profile));
            fprintf("s(t) samples = %d\n", numel(s_t));
        end

        function print_third_order(label, ctx, t_final, profile, s_t, previous_t_final)
            %PRINT_THIRD_ORDER Print the common TOPP3/COPP3 summary.
            if nargin < 6
                previous_t_final = [];
            end

            fprintf("%s done.\n", label);
            fprintf("dim = %d, N = %d\n", ctx.dim, ctx.n);
            if isempty(previous_t_final)
                fprintf("t_final = %.6f s\n", t_final);
            else
                fprintf("t_final = %.6f s <= %.6f s\n", t_final, previous_t_final);
            end
            fprintf("a_profile.len() = %d\n", numel(profile.a));
            fprintf("b_profile.len() = %d\n", numel(profile.b));
            fprintf("s(t) samples = %d\n", numel(s_t));
        end

        function print_reach_set2(ctx, reach_back, reach_bidir)
            %PRINT_REACH_SET2 Print second-order reachable-set bounds.
            k0 = 1;
            km = floor(ctx.n / 2) + 1;
            k1 = ctx.n;

            fprintf("reach_set2 done.\n");
            fprintf("dim = %d, N = %d\n", ctx.dim, ctx.n);
            fprintf("backward-only: a_max.len() = %d, a_min.len() = %d\n", ...
                numel(reach_back.a_max), numel(reach_back.a_min));
            fprintf("bidirectional: a_max.len() = %d, a_min.len() = %d\n", ...
                numel(reach_bidir.a_max), numel(reach_bidir.a_min));
            fprintf("bidirectional bounds @k=0/mid/end: ");
            fprintf("[%.6f, %.6f], ", reach_bidir.a_min(k0), reach_bidir.a_max(k0));
            fprintf("[%.6f, %.6f], ", reach_bidir.a_min(km), reach_bidir.a_max(km));
            fprintf("[%.6f, %.6f]\n", reach_bidir.a_min(k1), reach_bidir.a_max(k1));
        end

    end

    methods (Static, Access = private)
        function ctx = base_context(n)
            ctx = struct();
            ctx.dim = 3;
            ctx.n = n;
            ctx.dt = 1.0e-3;
            ctx.s = linspace(0.0, 1.0, n).';
            ctx.idx_s_interval = [1, n];
            ctx.a_boundary = [0, 0];
            ctx.b_boundary = [0, 0];
            ctx.num_stationary_max = 1;
            ctx.limit_max = ones(ctx.dim, 1);
            ctx.limit_min = -ctx.limit_max;
        end

        function path = lissajous_path(dim)
            path = copp.Path.from_parametric( ...
                @(x) [ ...
                    sin(2*pi*x); ...
                    sin(3*pi*x + 0.3); ...
                    sin(5*pi*x + 0.7)], ...
                s_range=[0, 1], ...
                dim=dim);
        end
    end
end
