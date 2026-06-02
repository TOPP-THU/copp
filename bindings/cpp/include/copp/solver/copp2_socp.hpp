#pragma once

#include <initializer_list>
#include <optional>
#include <vector>

#include "copp/clarabel.hpp"
#include "copp/core.hpp"
#include "copp/objective.hpp"
#include "copp/robot.hpp"

namespace copp::solver::copp2_socp
{

    /// COPP2-SOCP problem descriptor.
    ///
    /// The problem borrows a `Robot` and owns a copy of the objective
    /// descriptors. The referenced robot must outlive the problem and solver
    /// calls. Construction validates the closed station interval, boundary
    /// values, and objective vector lengths against the current robot state.
    ///
    /// Torque objectives use the robot's installed inverse-dynamics callback
    /// when present. Without a callback, the robot uses point-mass dynamics
    /// (`tau = ddq`).
    class COPP_API Problem
    {
    public:
        Problem(
            const Robot &robot,
            std::vector<Objective> objectives,
            IndexInterval idx_s_interval,
            Boundary2 a_boundary = {});

        Problem(
            const Robot &robot,
            std::initializer_list<Objective> objectives,
            IndexInterval idx_s_interval,
            Boundary2 a_boundary = {});

        const Robot &robot() const noexcept { return *robot_; }
        const std::vector<Objective> &objectives() const noexcept { return objectives_; }
        IndexInterval idx_s_interval() const noexcept { return idx_s_interval_; }
        Boundary2 a_boundary() const noexcept { return a_boundary_; }
        std::size_t s_len() const noexcept { return s_len_; }

    private:
        const Robot *robot_ = nullptr;
        std::vector<Objective> objectives_;
        IndexInterval idx_s_interval_;
        Boundary2 a_boundary_;
        std::size_t s_len_ = 0;
    };

    /// Expert COPP2-SOCP result with raw Clarabel diagnostics.
    ///
    /// `a` contains a profile only when the Clarabel status was accepted by the
    /// provided options. The raw `x`, `z`, and `s` vectors are always copied from
    /// Clarabel when the solve itself succeeds. `objective_value` is populated
    /// only when `a` is populated; `objective_terms` contains per-objective
    /// unweighted values in objective-list order.
    struct Result
    {
        std::optional<std::vector<double>> a;
        std::vector<double> x;
        std::vector<double> z;
        std::vector<double> s;
        clarabel::SolverStatus solver_status = clarabel::SolverStatus::Unsolved;
        double obj_val = 0.0;
        double obj_val_dual = 0.0;
        double solve_time = 0.0;
        std::uint32_t iterations = 0;
        double r_prim = 0.0;
        double r_dual = 0.0;
        clarabel::LinearSolverInfo linsolver;
        std::optional<double> objective_value;
        std::vector<double> objective_terms;

        bool has_a() const noexcept { return a.has_value(); }
    };

    /// Solve COPP2-SOCP and return an accepted node profile `a(s)`.
    ///
    /// This is the normal production API. Non-accepted Clarabel statuses are
    /// converted into `copp::Error`. Use `solve_expert` to inspect diagnostics
    /// even when no profile is accepted.
    COPP_API std::vector<double> solve(
        const Problem &problem,
        const clarabel::Options &options = {});
    inline Expected<std::vector<double>> solve(
        const Problem &problem,
        const clarabel::Options &options,
        NoThrowTag)
    {
        return detail::expected_from([&] { return solve(problem, options); });
    }
    inline Expected<std::vector<double>> solve(const Problem &problem, NoThrowTag tag)
    {
        return solve(problem, clarabel::Options{}, tag);
    }

    /// Solve COPP2-SOCP and always return Clarabel diagnostics.
    COPP_API Result solve_expert(
        const Problem &problem,
        const clarabel::Options &options = {});
    inline Expected<Result> solve_expert(
        const Problem &problem,
        const clarabel::Options &options,
        NoThrowTag)
    {
        return detail::expected_from([&] { return solve_expert(problem, options); });
    }
    inline Expected<Result> solve_expert(const Problem &problem, NoThrowTag tag)
    {
        return solve_expert(problem, clarabel::Options{}, tag);
    }

} // namespace copp::solver::copp2_socp
