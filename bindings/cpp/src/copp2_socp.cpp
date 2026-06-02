#include "copp/solver/copp2_socp.hpp"

// COPP2-SOCP facade implementation.
//
// Objective descriptors are flattened before crossing the bridge: small C++
// objects keep user-friendly vectors, while Rust receives compact descriptors
// plus one contiguous payload buffer. Expert results copy Clarabel diagnostics
// back into public value types.

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

    const copp::bridge::RobotHandle *checked_handle(const copp::Robot &robot)
    {
        const auto *handle = static_cast<const copp::bridge::RobotHandle *>(
            copp::detail::robot_handle(robot));
        if (handle == nullptr)
        {
            throw copp::Error(copp::Status::invalid_input, "COPP2 problem received an empty Robot");
        }
        return handle;
    }

    std::vector<double> copy_vec(const rust::Box<copp::bridge::VecF64Result> &values)
    {
        const auto slice = values->values();
        return std::vector<double>(slice.begin(), slice.end());
    }

    std::vector<double> copy_slice(rust::Slice<const double> values)
    {
        return std::vector<double>(values.begin(), values.end());
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

    copp::solver::copp2_socp::Result copy_result(
        const rust::Box<copp::bridge::Copp2SocpResult> &result)
    {
        copp::solver::copp2_socp::Result out;
        if (result->has_a())
        {
            out.a = copy_slice(result->a());
        }
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

} // namespace

namespace copp::solver::copp2_socp
{

    Problem::Problem(
        const Robot &robot,
        std::vector<Objective> objectives,
        IndexInterval idx_s_interval,
        Boundary2 a_boundary)
        : robot_(&robot),
          objectives_(std::move(objectives)),
          idx_s_interval_(idx_s_interval),
          a_boundary_(a_boundary)
    {
        const auto *handle = checked_handle(*robot_);
        const auto objective_bridge = to_bridge(objectives_);
        s_len_ = call_bridge([&]
                             { return handle->copp2_problem_s_len(
                                   idx_s_interval_.idx_s_start,
                                   idx_s_interval_.idx_s_final,
                                   a_boundary_.a_start,
                                   a_boundary_.a_final,
                                   to_rust_slice(objective_bridge.descriptors),
                                   to_rust_slice(objective_bridge.data)); });
    }

    Problem::Problem(
        const Robot &robot,
        std::initializer_list<Objective> objectives,
        IndexInterval idx_s_interval,
        Boundary2 a_boundary)
        : Problem(robot, std::vector<Objective>(objectives), idx_s_interval, a_boundary) {}

    std::vector<double> solve(
        const Problem &problem,
        const clarabel::Options &options)
    {
        const auto *handle = checked_handle(problem.robot());
        const auto objective_bridge = to_bridge(problem.objectives());
        auto result = call_bridge([&]
                                  { return handle->copp2_socp_solve(
                                        problem.idx_s_interval().idx_s_start,
                                        problem.idx_s_interval().idx_s_final,
                                        problem.a_boundary().a_start,
                                        problem.a_boundary().a_final,
                                        to_rust_slice(objective_bridge.descriptors),
                                        to_rust_slice(objective_bridge.data),
                                        to_bridge(options)); });
        return copy_vec(result);
    }

    Result solve_expert(
        const Problem &problem,
        const clarabel::Options &options)
    {
        const auto *handle = checked_handle(problem.robot());
        const auto objective_bridge = to_bridge(problem.objectives());
        auto result = call_bridge([&]
                                  { return handle->copp2_socp_solve_expert(
                                        problem.idx_s_interval().idx_s_start,
                                        problem.idx_s_interval().idx_s_final,
                                        problem.a_boundary().a_start,
                                        problem.a_boundary().a_final,
                                        to_rust_slice(objective_bridge.descriptors),
                                        to_rust_slice(objective_bridge.data),
                                        to_bridge(options)); });
        return copy_result(result);
    }

} // namespace copp::solver::copp2_socp
