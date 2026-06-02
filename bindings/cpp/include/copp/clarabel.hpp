#pragma once

#include <cstddef>
#include <cstdint>

#include "copp/core.hpp"

namespace copp::clarabel
{
    namespace detail
    {
        struct NoInitTag
        {
            explicit constexpr NoInitTag() = default;
        };

        inline constexpr NoInitTag no_init{};
    } // namespace detail

    /// Direct linear solver method forwarded to Clarabel.
    enum class DirectSolveMethod
    {
        Auto = 0,
        Qdldl = 1,
        Faer = 2,
        Mkl = 3,
        Panua = 4,
    };

    /// Clarabel termination status reported by expert SOCP results.
    enum class SolverStatus
    {
        Unsolved = 0,
        Solved = 1,
        PrimalInfeasible = 2,
        DualInfeasible = 3,
        AlmostSolved = 4,
        AlmostPrimalInfeasible = 5,
        AlmostDualInfeasible = 6,
        MaxIterations = 7,
        MaxTime = 8,
        NumericalError = 9,
        InsufficientProgress = 10,
        CallbackTerminated = 11,
    };

    /// Linear solver metadata captured from Clarabel after a solve.
    struct LinearSolverInfo
    {
        DirectSolveMethod method = DirectSolveMethod::Auto;
        std::size_t threads = 0;
        bool direct = false;
        std::size_t nnz_a = 0;
        std::size_t nnz_l = 0;
    };

    /// Advanced raw Clarabel settings.
    ///
    /// The field list mirrors Rust/Python `ClarabelSettings`, which mirrors the
    /// current `clarabel::solver::DefaultSettings<f64>`. Constructing
    /// `Settings{}` fetches COPP's Rust defaults through the private bridge, so
    /// C++ defaults stay aligned with Rust/Python if Clarabel defaults change.
    struct COPP_API Settings
    {
        Settings();
        explicit Settings(detail::NoInitTag) noexcept {}

        /// Return COPP's current Rust-side Clarabel defaults.
        static Settings defaults();

        std::uint32_t max_iter;
        double time_limit;
        bool verbose;
        double max_step_fraction;
        double tol_gap_abs;
        double tol_gap_rel;
        double tol_feas;
        double tol_infeas_abs;
        double tol_infeas_rel;
        double tol_ktratio;
        double reduced_tol_gap_abs;
        double reduced_tol_gap_rel;
        double reduced_tol_feas;
        double reduced_tol_infeas_abs;
        double reduced_tol_infeas_rel;
        double reduced_tol_ktratio;
        bool equilibrate_enable;
        std::uint32_t equilibrate_max_iter;
        double equilibrate_min_scaling;
        double equilibrate_max_scaling;
        double linesearch_backtrack_step;
        double min_switch_step_length;
        double min_terminate_step_length;
        std::uint32_t max_threads;
        bool direct_kkt_solver;
        DirectSolveMethod direct_solve_method;
        bool static_regularization_enable;
        double static_regularization_constant;
        double static_regularization_proportional;
        bool dynamic_regularization_enable;
        double dynamic_regularization_eps;
        double dynamic_regularization_delta;
        bool iterative_refinement_enable;
        double iterative_refinement_reltol;
        double iterative_refinement_abstol;
        std::uint32_t iterative_refinement_max_iter;
        double iterative_refinement_stop_ratio;
        bool presolve_enable;
        bool input_sparse_dropzeros;
    };

    /// Shared Clarabel options for COPP/TOPP SOCP-style solvers.
    ///
    /// `allow_*` switches control which non-ideal statuses may still produce an
    /// accepted profile in normal solver APIs. `Solved` is always accepted.
    struct COPP_API Options
    {
        Options();
        explicit Options(detail::NoInitTag) noexcept : clarabel_settings(detail::no_init) {}

        /// Return COPP's current Rust-side default options.
        static Options defaults();

        Verbosity verbosity;
        bool allow_almost_solved;
        bool allow_max_iterations;
        bool allow_max_time;
        bool allow_callback_terminated;
        bool allow_insufficient_progress;
        Settings clarabel_settings;
    };

} // namespace copp::clarabel
