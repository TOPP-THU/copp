#include "copp/solver/topp3.hpp"

// Third-order TOPP/COPP facade implementation.
//
// This translation unit collects the shared bridge plumbing for TOPP3-LP,
// TOPP3-SOCP, and COPP3-SOCP. Problem
// constructors copy `a_linearization`, immediately ask Rust to linearize
// third-order constraints, and keep only lightweight descriptors for later
// solver calls.

#include "copp/solver/copp3_socp.hpp"
#include "copp/solver/topp3_lp.hpp"
#include "copp/solver/topp3_socp.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rust/cxx.h"
#include "copp/src/ffi/cpp/mod.rs.h"

namespace
{

    struct ObjectiveBridgeData
    {
        std::vector<copp::bridge::ObjectiveDescriptor> descriptors;
        std::vector<double> data;
    };

    std::string to_std_string(const rust::Error &error)
    {
        return std::string(error.what());
    }

    [[noreturn]] void throw_bridge_error(const rust::Error &error)
    {
        throw copp::Error(copp::Status::invalid_input, to_std_string(error));
    }

    template <typename Fn>
    auto call_bridge(Fn &&fn) -> decltype(fn())
    {
        try
        {
            return fn();
        }
        catch (const rust::Error &error)
        {
            throw_bridge_error(error);
        }
    }

    uint8_t to_bridge(copp::Verbosity verbosity)
    {
        switch (verbosity)
        {
        case copp::Verbosity::Silent:
            return 0;
        case copp::Verbosity::Summary:
            return 1;
        case copp::Verbosity::Debug:
            return 2;
        case copp::Verbosity::Trace:
            return 3;
        }
        throw copp::Error(copp::Status::invalid_input, "unsupported Verbosity");
    }

    uint8_t to_bridge(copp::clarabel::DirectSolveMethod method)
    {
        switch (method)
        {
        case copp::clarabel::DirectSolveMethod::Auto:
            return 0;
        case copp::clarabel::DirectSolveMethod::Qdldl:
            return 1;
        case copp::clarabel::DirectSolveMethod::Faer:
            return 2;
        case copp::clarabel::DirectSolveMethod::Mkl:
            return 3;
        case copp::clarabel::DirectSolveMethod::Panua:
            return 4;
        }
        throw copp::Error(copp::Status::invalid_input, "unsupported Clarabel direct solve method");
    }

    copp::clarabel::SolverStatus status_from_bridge(uint8_t status)
    {
        switch (status)
        {
        case 0:
            return copp::clarabel::SolverStatus::Unsolved;
        case 1:
            return copp::clarabel::SolverStatus::Solved;
        case 2:
            return copp::clarabel::SolverStatus::PrimalInfeasible;
        case 3:
            return copp::clarabel::SolverStatus::DualInfeasible;
        case 4:
            return copp::clarabel::SolverStatus::AlmostSolved;
        case 5:
            return copp::clarabel::SolverStatus::AlmostPrimalInfeasible;
        case 6:
            return copp::clarabel::SolverStatus::AlmostDualInfeasible;
        case 7:
            return copp::clarabel::SolverStatus::MaxIterations;
        case 8:
            return copp::clarabel::SolverStatus::MaxTime;
        case 9:
            return copp::clarabel::SolverStatus::NumericalError;
        case 10:
            return copp::clarabel::SolverStatus::InsufficientProgress;
        case 11:
            return copp::clarabel::SolverStatus::CallbackTerminated;
        }
        throw copp::Error(copp::Status::bridge_error, "unsupported Clarabel solver status");
    }

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
        throw copp::Error(copp::Status::bridge_error, "unsupported Clarabel direct solve method");
    }

    copp::bridge::ClarabelSettingsBridge to_bridge(const copp::clarabel::Settings &settings)
    {
        return copp::bridge::ClarabelSettingsBridge{
            settings.max_iter,
            settings.time_limit,
            settings.verbose,
            settings.max_step_fraction,
            settings.tol_gap_abs,
            settings.tol_gap_rel,
            settings.tol_feas,
            settings.tol_infeas_abs,
            settings.tol_infeas_rel,
            settings.tol_ktratio,
            settings.reduced_tol_gap_abs,
            settings.reduced_tol_gap_rel,
            settings.reduced_tol_feas,
            settings.reduced_tol_infeas_abs,
            settings.reduced_tol_infeas_rel,
            settings.reduced_tol_ktratio,
            settings.equilibrate_enable,
            settings.equilibrate_max_iter,
            settings.equilibrate_min_scaling,
            settings.equilibrate_max_scaling,
            settings.linesearch_backtrack_step,
            settings.min_switch_step_length,
            settings.min_terminate_step_length,
            settings.max_threads,
            settings.direct_kkt_solver,
            to_bridge(settings.direct_solve_method),
            settings.static_regularization_enable,
            settings.static_regularization_constant,
            settings.static_regularization_proportional,
            settings.dynamic_regularization_enable,
            settings.dynamic_regularization_eps,
            settings.dynamic_regularization_delta,
            settings.iterative_refinement_enable,
            settings.iterative_refinement_reltol,
            settings.iterative_refinement_abstol,
            settings.iterative_refinement_max_iter,
            settings.iterative_refinement_stop_ratio,
            settings.presolve_enable,
            settings.input_sparse_dropzeros,
        };
    }

    copp::bridge::ClarabelOptionsBridge to_bridge(const copp::clarabel::Options &options)
    {
        return copp::bridge::ClarabelOptionsBridge{
            to_bridge(options.verbosity),
            options.allow_almost_solved,
            options.allow_max_iterations,
            options.allow_max_time,
            options.allow_callback_terminated,
            options.allow_insufficient_progress,
            to_bridge(options.clarabel_settings),
        };
    }

    rust::Slice<const double> to_rust_slice(const std::vector<double> &values) noexcept
    {
        return rust::Slice<const double>(values.data(), values.size());
    }

    rust::Slice<const double> to_rust_slice(copp::Span<const double> values) noexcept
    {
        return rust::Slice<const double>(values.data(), values.size());
    }

    rust::Slice<const copp::bridge::ObjectiveDescriptor> to_rust_slice(
        const std::vector<copp::bridge::ObjectiveDescriptor> &values) noexcept
    {
        return rust::Slice<const copp::bridge::ObjectiveDescriptor>(values.data(), values.size());
    }

    std::pair<std::size_t, std::size_t> append_payload(
        ObjectiveBridgeData &out,
        copp::Span<const double> payload)
    {
        const std::size_t offset = out.data.size();
        if (!payload.empty())
        {
            out.data.insert(out.data.end(), payload.data(), payload.data() + payload.size());
        }
        return {offset, payload.size()};
    }

    ObjectiveBridgeData to_bridge(const std::vector<copp::Objective> &objectives)
    {
        ObjectiveBridgeData out;
        out.descriptors.reserve(objectives.size());

        for (const auto &objective : objectives)
        {
            std::pair<std::size_t, std::size_t> first{0, 0};
            std::pair<std::size_t, std::size_t> second{0, 0};
            uint8_t kind = 0;

            switch (objective.kind())
            {
            case copp::ObjectiveKind::Time:
                kind = 0;
                break;
            case copp::ObjectiveKind::Linear:
                kind = 1;
                first = append_payload(out, objective.alpha());
                second = append_payload(out, objective.beta());
                break;
            case copp::ObjectiveKind::ThermalEnergy:
                kind = 2;
                first = append_payload(out, objective.normalize());
                break;
            case copp::ObjectiveKind::TotalVariationTorque:
                kind = 3;
                first = append_payload(out, objective.normalize());
                break;
            }

            out.descriptors.push_back(copp::bridge::ObjectiveDescriptor{
                kind,
                objective.weight(),
                first.first,
                first.second,
                second.first,
                second.second,
            });
        }

        return out;
    }

    copp::bridge::RobotHandle *checked_handle(copp::ConstraintsRef constraints)
    {
        auto *handle = static_cast<copp::bridge::RobotHandle *>(
            copp::detail::constraints_handle_mut(constraints));
        if (handle == nullptr)
        {
            throw copp::Error(copp::Status::invalid_input, "TOPP3 problem received an empty ConstraintsRef");
        }
        return handle;
    }

    copp::bridge::RobotHandle *checked_handle(copp::Robot &robot)
    {
        auto *handle = static_cast<copp::bridge::RobotHandle *>(
            copp::detail::robot_handle_mut(robot));
        if (handle == nullptr)
        {
            throw copp::Error(copp::Status::invalid_input, "COPP3 problem received an empty Robot");
        }
        return handle;
    }

    std::vector<double> copy_span(copp::Span<const double> values)
    {
        if (values.empty())
        {
            return {};
        }
        return std::vector<double>(values.data(), values.data() + values.size());
    }

    std::vector<double> copy_slice(rust::Slice<const double> values)
    {
        return std::vector<double>(values.begin(), values.end());
    }

    copp::Profile3rd copy_profile(const copp::bridge::Profile3rdResult &profile)
    {
        return copp::Profile3rd{
            copy_slice(profile.a()),
            copy_slice(profile.b()),
            profile.num_stationary_start(),
            profile.num_stationary_end(),
        };
    }

    copp::Profile3rd copy_profile(const rust::Box<copp::bridge::Profile3rdResult> &profile)
    {
        return copy_profile(*profile);
    }

    copp::clarabel::LinearSolverInfo linsolver_from_bridge(
        copp::bridge::LinearSolverInfoBridge info)
    {
        return copp::clarabel::LinearSolverInfo{
            method_from_bridge(info.method),
            info.threads,
            info.direct,
            info.nnz_a,
            info.nnz_l,
        };
    }

    std::optional<copp::Profile3rd> copy_optional_profile(
        const rust::Box<copp::bridge::Topp3SolverResult> &result)
    {
        if (!result->has_profile())
        {
            return std::nullopt;
        }
        return copp::Profile3rd{
            copy_slice(result->profile_a()),
            copy_slice(result->profile_b()),
            result->profile_num_stationary_start(),
            result->profile_num_stationary_end(),
        };
    }

    copp::solver::topp3::Result copy_result(
        const rust::Box<copp::bridge::Topp3SolverResult> &result)
    {
        copp::solver::topp3::Result out;
        out.profile = copy_optional_profile(result);
        out.x = copy_slice(result->x());
        out.z = copy_slice(result->z());
        out.s = copy_slice(result->s());
        out.solver_status = status_from_bridge(result->solver_status());
        out.obj_val = result->obj_val();
        out.obj_val_dual = result->obj_val_dual();
        out.solve_time = result->solve_time();
        out.iterations = result->iterations();
        out.r_prim = result->r_prim();
        out.r_dual = result->r_dual();
        out.linsolver = linsolver_from_bridge(result->linsolver());
        if (result->has_objective_value())
        {
            out.objective_value = result->objective_value();
        }
        out.objective_terms = copy_slice(result->objective_terms());
        return out;
    }

    void apply_info(
        copp::bridge::Topp3ProblemInfoBridge info,
        std::size_t &idx_s_final,
        copp::solver::topp3::StationaryBounds &num_stationary)
    {
        idx_s_final = info.idx_s_final;
        num_stationary = copp::solver::topp3::StationaryBounds{
            info.num_stationary_start,
            info.num_stationary_end,
        };
    }

} // namespace

namespace copp::solver::topp3
{

    Problem::Problem(
        ConstraintsRef constraints,
        Span<const double> a_linearization,
        std::size_t idx_s_start,
        Boundary3 boundary,
        StationaryBounds num_stationary_max,
        double a_linearization_floor)
        : constraints_(constraints),
          a_linearization_(copy_span(a_linearization)),
          idx_s_start_(idx_s_start),
          boundary_(boundary),
          num_stationary_max_(num_stationary_max),
          a_linearization_floor_(a_linearization_floor)
    {
        auto *handle = checked_handle(constraints_);
        const auto info = call_bridge([&]
                                      { return handle->topp3_problem_prepare(
                                            idx_s_start_,
                                            to_rust_slice(a_linearization_),
                                            boundary_.a_start,
                                            boundary_.a_final,
                                            boundary_.b_start,
                                            boundary_.b_final,
                                            num_stationary_max_.start,
                                            num_stationary_max_.end,
                                            a_linearization_floor_); });
        apply_info(info, idx_s_final_, num_stationary_);
    }

    Problem::Problem(
        ConstraintsRef constraints,
        Span<const double> a_linearization,
        std::size_t idx_s_start,
        Boundary3 boundary,
        std::size_t symmetric_num_stationary_max,
        double a_linearization_floor)
        : Problem(
              constraints,
              a_linearization,
              idx_s_start,
              boundary,
              StationaryBounds::both(symmetric_num_stationary_max),
              a_linearization_floor) {}

} // namespace copp::solver::topp3

namespace copp::solver::topp3_lp
{

    Profile3rd solve(const Problem &problem, const clarabel::Options &options)
    {
        auto *handle = checked_handle(problem.constraints());
        auto profile = call_bridge([&]
                                   { return handle->topp3_lp_solve(
                                         problem.idx_s_start(),
                                         to_rust_slice(problem.a_linearization()),
                                         problem.boundary().a_start,
                                         problem.boundary().a_final,
                                         problem.boundary().b_start,
                                         problem.boundary().b_final,
                                         problem.num_stationary_max().start,
                                         problem.num_stationary_max().end,
                                         problem.a_linearization_floor(),
                                         to_bridge(options)); });
        return copy_profile(profile);
    }

    Result solve_expert(const Problem &problem, const clarabel::Options &options)
    {
        auto *handle = checked_handle(problem.constraints());
        auto result = call_bridge([&]
                                  { return handle->topp3_lp_solve_expert(
                                        problem.idx_s_start(),
                                        to_rust_slice(problem.a_linearization()),
                                        problem.boundary().a_start,
                                        problem.boundary().a_final,
                                        problem.boundary().b_start,
                                        problem.boundary().b_final,
                                        problem.num_stationary_max().start,
                                        problem.num_stationary_max().end,
                                        problem.a_linearization_floor(),
                                        to_bridge(options)); });
        return copy_result(result);
    }

} // namespace copp::solver::topp3_lp

namespace copp::solver::topp3_socp
{

    Profile3rd solve(const Problem &problem, const clarabel::Options &options)
    {
        auto *handle = checked_handle(problem.constraints());
        auto profile = call_bridge([&]
                                   { return handle->topp3_socp_solve(
                                         problem.idx_s_start(),
                                         to_rust_slice(problem.a_linearization()),
                                         problem.boundary().a_start,
                                         problem.boundary().a_final,
                                         problem.boundary().b_start,
                                         problem.boundary().b_final,
                                         problem.num_stationary_max().start,
                                         problem.num_stationary_max().end,
                                         problem.a_linearization_floor(),
                                         to_bridge(options)); });
        return copy_profile(profile);
    }

    Result solve_expert(const Problem &problem, const clarabel::Options &options)
    {
        auto *handle = checked_handle(problem.constraints());
        auto result = call_bridge([&]
                                  { return handle->topp3_socp_solve_expert(
                                        problem.idx_s_start(),
                                        to_rust_slice(problem.a_linearization()),
                                        problem.boundary().a_start,
                                        problem.boundary().a_final,
                                        problem.boundary().b_start,
                                        problem.boundary().b_final,
                                        problem.num_stationary_max().start,
                                        problem.num_stationary_max().end,
                                        problem.a_linearization_floor(),
                                        to_bridge(options)); });
        return copy_result(result);
    }

} // namespace copp::solver::topp3_socp

namespace copp::solver::copp3_socp
{

    Problem::Problem(
        Robot &robot,
        std::vector<Objective> objectives,
        Span<const double> a_linearization,
        std::size_t idx_s_start,
        Boundary3 boundary,
        StationaryBounds num_stationary_max,
        double a_linearization_floor)
        : robot_(&robot),
          objectives_(std::move(objectives)),
          a_linearization_(copy_span(a_linearization)),
          idx_s_start_(idx_s_start),
          boundary_(boundary),
          num_stationary_max_(num_stationary_max),
          a_linearization_floor_(a_linearization_floor)
    {
        auto *handle = checked_handle(*robot_);
        const auto objective_bridge = to_bridge(objectives_);
        const auto info = call_bridge([&]
                                      { return handle->copp3_problem_prepare(
                                            idx_s_start_,
                                            to_rust_slice(a_linearization_),
                                            boundary_.a_start,
                                            boundary_.a_final,
                                            boundary_.b_start,
                                            boundary_.b_final,
                                            num_stationary_max_.start,
                                            num_stationary_max_.end,
                                            a_linearization_floor_,
                                            to_rust_slice(objective_bridge.descriptors),
                                            to_rust_slice(objective_bridge.data)); });
        apply_info(info, idx_s_final_, num_stationary_);
    }

    Problem::Problem(
        Robot &robot,
        std::initializer_list<Objective> objectives,
        Span<const double> a_linearization,
        std::size_t idx_s_start,
        Boundary3 boundary,
        StationaryBounds num_stationary_max,
        double a_linearization_floor)
        : Problem(
              robot,
              std::vector<Objective>(objectives),
              a_linearization,
              idx_s_start,
              boundary,
              num_stationary_max,
              a_linearization_floor) {}

    Problem::Problem(
        Robot &robot,
        std::vector<Objective> objectives,
        Span<const double> a_linearization,
        std::size_t idx_s_start,
        Boundary3 boundary,
        std::size_t symmetric_num_stationary_max,
        double a_linearization_floor)
        : Problem(
              robot,
              std::move(objectives),
              a_linearization,
              idx_s_start,
              boundary,
              StationaryBounds::both(symmetric_num_stationary_max),
              a_linearization_floor) {}

    Profile3rd solve(const Problem &problem, const clarabel::Options &options)
    {
        auto *handle = checked_handle(problem.robot());
        const auto objective_bridge = to_bridge(problem.objectives());
        auto profile = call_bridge([&]
                                   { return handle->copp3_socp_solve(
                                         problem.idx_s_start(),
                                         to_rust_slice(problem.a_linearization()),
                                         problem.boundary().a_start,
                                         problem.boundary().a_final,
                                         problem.boundary().b_start,
                                         problem.boundary().b_final,
                                         problem.num_stationary_max().start,
                                         problem.num_stationary_max().end,
                                         problem.a_linearization_floor(),
                                         to_rust_slice(objective_bridge.descriptors),
                                         to_rust_slice(objective_bridge.data),
                                         to_bridge(options)); });
        return copy_profile(profile);
    }

    Result solve_expert(const Problem &problem, const clarabel::Options &options)
    {
        auto *handle = checked_handle(problem.robot());
        const auto objective_bridge = to_bridge(problem.objectives());
        auto result = call_bridge([&]
                                  { return handle->copp3_socp_solve_expert(
                                        problem.idx_s_start(),
                                        to_rust_slice(problem.a_linearization()),
                                        problem.boundary().a_start,
                                        problem.boundary().a_final,
                                        problem.boundary().b_start,
                                        problem.boundary().b_final,
                                        problem.num_stationary_max().start,
                                        problem.num_stationary_max().end,
                                        problem.a_linearization_floor(),
                                        to_rust_slice(objective_bridge.descriptors),
                                        to_rust_slice(objective_bridge.data),
                                        to_bridge(options)); });
        return copy_result(result);
    }

} // namespace copp::solver::copp3_socp

