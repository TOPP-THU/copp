classdef test_clarabel_options < matlab.unittest.TestCase
    %TEST_CLARABEL_OPTIONS Shared Clarabel options/settings public behavior.

    methods (Test)
        function defaults_and_name_value_overrides(testCase)
            settings = copp.clarabel.Settings(max_iter=120, ...
                tol_gap_rel=1.0e-7, direct_solve_method="qdldl");
            options = copp.clarabel.Options( ...
                verbosity=copp.diag.Verbosity.summary, ...
                clarabel_settings=settings, ...
                allow_max_time=true);

            args = options.native_descriptor();

            testCase.verifyNumElements(args, 45);
            testCase.verifyEqual(options.verbosity, "summary");
            testCase.verifyTrue(options.allow_almost_solved);
            testCase.verifyTrue(options.allow_max_time);
            testCase.verifyEqual(options.clarabel_settings.max_iter, 120);
            testCase.verifyEqual(options.clarabel_settings.tol_gap_rel, 1.0e-7);
            testCase.verifyEqual(options.clarabel_settings.direct_solve_method, "qdldl");
            testCase.verifyEqual(args{1}, 'summary');
            testCase.verifyEqual(args{7}, 120);
            testCase.verifyEqual(args{32}, 'qdldl');
        end

        function struct_overrides_and_solver_namespace_options(testCase)
            options = copp.solver.copp2_socp.Options(struct( ...
                'verbosity', "debug", ...
                'max_iter', 80, ...
                'direct_solve_method', copp.clarabel.DirectSolveMethod.auto));

            testCase.verifyClass(options, 'copp.solver.copp2_socp.Options');
            testCase.verifyTrue(isa(options, 'copp.solver.copp2_socp.ClarabelOptions'));
            testCase.verifyTrue(isa(options, 'copp.clarabel.Options'));
            testCase.verifyEqual(options.verbosity, "debug");
            testCase.verifyEqual(options.clarabel_settings.max_iter, 80);
            testCase.verifyEqual(options.clarabel_settings.direct_solve_method, "auto");
        end

        function solver_namespace_aliases_construct_shared_options(testCase)
            settings = copp.solver.copp3_socp.ClarabelSettings( ...
                max_iter=44, ...
                direct_solve_method=copp.solver.copp3_socp.ClarabelDirectSolveMethod.qdldl);
            options = copp.solver.copp3_socp.ClarabelOptions( ...
                verbosity=copp.diag.Verbosity.summary, ...
                clarabel_settings=settings);

            testCase.verifyClass(settings, 'copp.solver.copp3_socp.ClarabelSettings');
            testCase.verifyTrue(isa(settings, 'copp.clarabel.Settings'));
            testCase.verifyClass(options, 'copp.solver.copp3_socp.ClarabelOptions');
            testCase.verifyTrue(isa(options, 'copp.clarabel.Options'));
            testCase.verifyEqual(options.verbosity, "summary");
            testCase.verifyEqual(options.clarabel_settings.max_iter, 44);
            testCase.verifyEqual(options.clarabel_settings.direct_solve_method, "qdldl");
        end
    end
end
