#include "copp/solver/topp2_ra.hpp"

// TOPP2 reachable-analysis facade.
//
// Public C++ problem objects are non-owning descriptors over a `ConstraintsRef`.
// This file checks that the referenced Rust handle exists, forwards interval and
// tolerance data to Rust, then copies reachable intervals or accepted profiles
// into STL containers.

#include <string>
#include <vector>

#include "rust/cxx.h"
#include "copp/src/ffi/cpp/mod.rs.h"

namespace
{

    std::string to_std_string(const rust::Error &error)
    {
        return std::string(error.what());
    }

    [[noreturn]] void throw_bridge_error(const rust::Error &error)
    {
        throw copp::Error(copp::Status::invalid_input, to_std_string(error));
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

    const copp::bridge::RobotHandle *checked_handle(copp::ConstraintsRef constraints)
    {
        const auto *handle = static_cast<const copp::bridge::RobotHandle *>(
            copp::detail::constraints_handle(constraints));
        if (handle == nullptr)
        {
            throw copp::Error(copp::Status::invalid_input, "TOPP2 problem received an empty ConstraintsRef");
        }
        return handle;
    }

    std::vector<double> copy_vec(const rust::Box<copp::bridge::VecF64Result> &values)
    {
        const auto slice = values->values();
        return std::vector<double>(slice.begin(), slice.end());
    }

    copp::solver::topp2_ra::ReachSet copy_reach_set(
        const rust::Box<copp::bridge::ReachSet2Result> &reach)
    {
        const auto a_max = reach->a_max();
        const auto a_min = reach->a_min();
        return copp::solver::topp2_ra::ReachSet{
            std::vector<double>(a_max.begin(), a_max.end()),
            std::vector<double>(a_min.begin(), a_min.end()),
        };
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

} // namespace

namespace copp::solver::topp2_ra
{

    Problem::Problem(
        ConstraintsRef constraints,
        IndexInterval idx_s_interval,
        Boundary2 a_boundary)
        : constraints_(constraints), idx_s_interval_(idx_s_interval), a_boundary_(a_boundary)
    {
        const auto *handle = checked_handle(constraints_);
        s_len_ = call_bridge([&]
                             { return handle->topp2_problem_s_len(
                                   idx_s_interval_.idx_s_start,
                                   idx_s_interval_.idx_s_final,
                                   a_boundary_.a_start,
                                   a_boundary_.a_final); });
    }

    ReachSet reach_set_backward(const Problem &problem, const Options &options)
    {
        const auto *handle = checked_handle(problem.constraints());
        auto reach = call_bridge([&]
                                 { return handle->reach_set2_backward(
                                       problem.idx_s_interval().idx_s_start,
                                       problem.idx_s_interval().idx_s_final,
                                       problem.a_boundary().a_start,
                                       problem.a_boundary().a_final,
                                       options.lp_feas_tol,
                                       options.a_cmp_abs_tol,
                                       options.a_cmp_rel_tol,
                                       to_bridge(options.verbosity)); });
        return copy_reach_set(reach);
    }

    ReachSet reach_set_bidirectional(const Problem &problem, const Options &options)
    {
        const auto *handle = checked_handle(problem.constraints());
        auto reach = call_bridge([&]
                                 { return handle->reach_set2_bidirectional(
                                       problem.idx_s_interval().idx_s_start,
                                       problem.idx_s_interval().idx_s_final,
                                       problem.a_boundary().a_start,
                                       problem.a_boundary().a_final,
                                       options.lp_feas_tol,
                                       options.a_cmp_abs_tol,
                                       options.a_cmp_rel_tol,
                                       to_bridge(options.verbosity)); });
        return copy_reach_set(reach);
    }

    std::vector<double> solve(const Problem &problem, const Options &options)
    {
        const auto *handle = checked_handle(problem.constraints());
        auto a = call_bridge([&]
                             { return handle->topp2_ra_solve(
                                   problem.idx_s_interval().idx_s_start,
                                   problem.idx_s_interval().idx_s_final,
                                   problem.a_boundary().a_start,
                                   problem.a_boundary().a_final,
                                   options.lp_feas_tol,
                                   options.a_cmp_abs_tol,
                                   options.a_cmp_rel_tol,
                                   to_bridge(options.verbosity)); });
        return copy_vec(a);
    }

} // namespace copp::solver::topp2_ra
