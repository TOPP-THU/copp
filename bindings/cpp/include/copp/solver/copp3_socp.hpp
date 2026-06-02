#pragma once

#include <initializer_list>
#include <vector>

#include "copp/clarabel.hpp"
#include "copp/interpolation.hpp"
#include "copp/objective.hpp"
#include "copp/robot.hpp"
#include "copp/solver/topp3.hpp"

namespace copp::solver::copp3_socp
{

    using Result = copp::solver::topp3::Result;
    using StationaryBounds = copp::solver::topp3::StationaryBounds;

    /// COPP3-SOCP problem descriptor.
    ///
    /// Construction borrows a mutable `Robot`, owns objective descriptors and
    /// `a_linearization`, infers the final station, and immediately linearizes
    /// third-order constraints in the robot's constraint cache. This class is
    /// intentionally independent rather than inheriting from TOPP3, matching
    /// Rust's separate `Copp3Problem`.
    ///
    /// @code
    /// copp::solver::copp3_socp::Problem problem{
    ///     robot,
    ///     {copp::objective::Time(1.0)},
    ///     a_linearization,
    ///     0,
    ///     copp::Boundary3{0.25, 0.25, 0.0, 0.0},
    /// };
    /// @endcode
    class COPP_API Problem
    {
    public:
        Problem(
            Robot &robot,
            std::vector<Objective> objectives,
            Span<const double> a_linearization,
            std::size_t idx_s_start = 0,
            Boundary3 boundary = {},
            StationaryBounds num_stationary_max = {},
            double a_linearization_floor = 1.0e-10);

        Problem(
            Robot &robot,
            std::initializer_list<Objective> objectives,
            Span<const double> a_linearization,
            std::size_t idx_s_start = 0,
            Boundary3 boundary = {},
            StationaryBounds num_stationary_max = {},
            double a_linearization_floor = 1.0e-10);

        Problem(
            Robot &robot,
            std::vector<Objective> objectives,
            Span<const double> a_linearization,
            std::size_t idx_s_start,
            Boundary3 boundary,
            std::size_t symmetric_num_stationary_max,
            double a_linearization_floor = 1.0e-10);

        Robot &robot() const noexcept { return *robot_; }
        const std::vector<Objective> &objectives() const noexcept { return objectives_; }
        const std::vector<double> &a_linearization() const noexcept { return a_linearization_; }
        std::size_t idx_s_start() const noexcept { return idx_s_start_; }
        std::size_t idx_s_final() const noexcept { return idx_s_final_; }
        Boundary3 boundary() const noexcept { return boundary_; }
        StationaryBounds num_stationary_max() const noexcept { return num_stationary_max_; }
        StationaryBounds num_stationary() const noexcept { return num_stationary_; }
        double a_linearization_floor() const noexcept { return a_linearization_floor_; }
        std::size_t s_len() const noexcept { return a_linearization_.size(); }

    private:
        Robot *robot_ = nullptr;
        std::vector<Objective> objectives_;
        std::vector<double> a_linearization_;
        std::size_t idx_s_start_ = 0;
        std::size_t idx_s_final_ = 0;
        Boundary3 boundary_;
        StationaryBounds num_stationary_max_;
        StationaryBounds num_stationary_;
        double a_linearization_floor_ = 1.0e-10;
    };

    /// Solve COPP3-SOCP and return an accepted third-order profile.
    ///
    /// Non-accepted Clarabel statuses are surfaced as `copp::Error` unless the
    /// provided `clarabel::Options` explicitly allows them.
    COPP_API Profile3rd solve(const Problem &problem, const clarabel::Options &options = {});
    inline Expected<Profile3rd> solve(
        const Problem &problem,
        const clarabel::Options &options,
        NoThrowTag)
    {
        return detail::expected_from([&] { return solve(problem, options); });
    }
    inline Expected<Profile3rd> solve(const Problem &problem, NoThrowTag tag)
    {
        return solve(problem, clarabel::Options{}, tag);
    }

    /// Solve COPP3-SOCP and always return Clarabel diagnostics.
    ///
    /// The returned `Result::profile` is optional. Raw `x/z/s` vectors and
    /// solver status are still available when a profile is not accepted.
    COPP_API Result solve_expert(const Problem &problem, const clarabel::Options &options = {});
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

} // namespace copp::solver::copp3_socp
