#include "copp/clarabel.hpp"

// Clarabel option/default conversion.
//
// Public C++ defaults are fetched from Rust instead of duplicated here. That
// keeps C++, Python, C, and Rust aligned when Clarabel defaults or COPP's
// accepted-status policy changes.

#include <string>

#include "rust/cxx.h"
#include "copp/src/ffi/cpp/mod.rs.h"

namespace
{

    copp::clarabel::DirectSolveMethod method_from_bridge(uint8_t method)
    {
        switch (method)
        {
        case 0:
            return copp::clarabel::DirectSolveMethod::Auto;
        case 1:
            return copp::clarabel::DirectSolveMethod::Qdldl;
        case 2:
            return copp::clarabel::DirectSolveMethod::Faer;
        case 3:
            return copp::clarabel::DirectSolveMethod::Mkl;
        case 4:
            return copp::clarabel::DirectSolveMethod::Panua;
        }
        throw copp::Error(copp::Status::invalid_input, "unsupported Clarabel direct solve method");
    }

    copp::clarabel::Settings settings_from_bridge(
        const copp::bridge::ClarabelSettingsBridge &settings)
    {
        copp::clarabel::Settings out(copp::clarabel::detail::no_init);
        out.max_iter = settings.max_iter;
        out.time_limit = settings.time_limit;
        out.verbose = settings.verbose;
        out.max_step_fraction = settings.max_step_fraction;
        out.tol_gap_abs = settings.tol_gap_abs;
        out.tol_gap_rel = settings.tol_gap_rel;
        out.tol_feas = settings.tol_feas;
        out.tol_infeas_abs = settings.tol_infeas_abs;
        out.tol_infeas_rel = settings.tol_infeas_rel;
        out.tol_ktratio = settings.tol_ktratio;
        out.reduced_tol_gap_abs = settings.reduced_tol_gap_abs;
        out.reduced_tol_gap_rel = settings.reduced_tol_gap_rel;
        out.reduced_tol_feas = settings.reduced_tol_feas;
        out.reduced_tol_infeas_abs = settings.reduced_tol_infeas_abs;
        out.reduced_tol_infeas_rel = settings.reduced_tol_infeas_rel;
        out.reduced_tol_ktratio = settings.reduced_tol_ktratio;
        out.equilibrate_enable = settings.equilibrate_enable;
        out.equilibrate_max_iter = settings.equilibrate_max_iter;
        out.equilibrate_min_scaling = settings.equilibrate_min_scaling;
        out.equilibrate_max_scaling = settings.equilibrate_max_scaling;
        out.linesearch_backtrack_step = settings.linesearch_backtrack_step;
        out.min_switch_step_length = settings.min_switch_step_length;
        out.min_terminate_step_length = settings.min_terminate_step_length;
        out.max_threads = settings.max_threads;
        out.direct_kkt_solver = settings.direct_kkt_solver;
        out.direct_solve_method = method_from_bridge(settings.direct_solve_method);
        out.static_regularization_enable = settings.static_regularization_enable;
        out.static_regularization_constant = settings.static_regularization_constant;
        out.static_regularization_proportional = settings.static_regularization_proportional;
        out.dynamic_regularization_enable = settings.dynamic_regularization_enable;
        out.dynamic_regularization_eps = settings.dynamic_regularization_eps;
        out.dynamic_regularization_delta = settings.dynamic_regularization_delta;
        out.iterative_refinement_enable = settings.iterative_refinement_enable;
        out.iterative_refinement_reltol = settings.iterative_refinement_reltol;
        out.iterative_refinement_abstol = settings.iterative_refinement_abstol;
        out.iterative_refinement_max_iter = settings.iterative_refinement_max_iter;
        out.iterative_refinement_stop_ratio = settings.iterative_refinement_stop_ratio;
        out.presolve_enable = settings.presolve_enable;
        out.input_sparse_dropzeros = settings.input_sparse_dropzeros;
        return out;
    }

    copp::clarabel::Options options_from_bridge(
        const copp::bridge::ClarabelOptionsBridge &options)
    {
        copp::clarabel::Options out(copp::clarabel::detail::no_init);
        switch (options.verbosity)
        {
        case 0:
            out.verbosity = copp::Verbosity::Silent;
            break;
        case 1:
            out.verbosity = copp::Verbosity::Summary;
            break;
        case 2:
            out.verbosity = copp::Verbosity::Debug;
            break;
        case 3:
            out.verbosity = copp::Verbosity::Trace;
            break;
        default:
            throw copp::Error(copp::Status::invalid_input, "unsupported Clarabel verbosity");
        }
        out.allow_almost_solved = options.allow_almost_solved;
        out.allow_max_iterations = options.allow_max_iterations;
        out.allow_max_time = options.allow_max_time;
        out.allow_callback_terminated = options.allow_callback_terminated;
        out.allow_insufficient_progress = options.allow_insufficient_progress;
        out.clarabel_settings = settings_from_bridge(options.clarabel_settings);
        return out;
    }

} // namespace

namespace copp::clarabel
{

    Settings::Settings() : Settings(Settings::defaults()) {}

    Settings Settings::defaults()
    {
        return settings_from_bridge(copp::bridge::clarabel_default_settings());
    }

    Options::Options() : Options(Options::defaults()) {}

    Options Options::defaults()
    {
        return options_from_bridge(copp::bridge::clarabel_default_options());
    }

} // namespace copp::clarabel
