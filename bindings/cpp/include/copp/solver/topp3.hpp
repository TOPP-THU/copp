#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "copp/clarabel.hpp"
#include "copp/core.hpp"
#include "copp/interpolation.hpp"
#include "copp/robot.hpp"

namespace copp::solver::topp3
{

    /// User-supplied upper bound for stationary boundary intervals.
    ///
    /// The Rust builder derives the effective stationary counts from
    /// `Boundary3` and this upper bound during immediate linearization.
    ///
    /// Stationary intervals are counted in path segments, not nodes. For
    /// example, `both(1)` allows one zero-speed interval at each boundary.
    struct StationaryBounds
    {
        std::size_t start = 1;
        std::size_t end = 1;

        static constexpr StationaryBounds both(std::size_t value) noexcept
        {
            return StationaryBounds{value, value};
        }
    };

    /// Prepared TOPP3 problem descriptor.
    ///
    /// Construction borrows a `ConstraintsRef`, owns a copy of
    /// `a_linearization`, infers `idx_s_final`, and immediately asks Rust to
    /// linearize third-order constraints into the underlying constraint cache.
    /// The owning `Robot` or `Constraints` object must outlive this problem and
    /// solver calls using it.
    class COPP_API Problem
    {
    public:
        /// Prepare a third-order problem and immediately linearize constraints.
        ///
        /// @param constraints Borrowed constraint buffer. The owner must outlive
        /// this problem and all solver calls.
        /// @param a_linearization Node profile used for linearizing jerk
        /// constraints.
        /// @param idx_s_start Global station id of `a_linearization[0]`.
        /// @param boundary Endpoint values for `a = dot{s}^2` and `b = ddot{s}`.
        /// @param num_stationary_max Upper bounds for inferred stationary
        /// boundary intervals.
        /// @param a_linearization_floor Lower numerical floor applied by Rust.
        /// @throws copp::Error if the interval, boundary, or linearization is invalid.
        Problem(
            ConstraintsRef constraints,
            Span<const double> a_linearization,
            std::size_t idx_s_start = 0,
            Boundary3 boundary = {},
            StationaryBounds num_stationary_max = {},
            double a_linearization_floor = 1.0e-10);

        Problem(
            ConstraintsRef constraints,
            Span<const double> a_linearization,
            std::size_t idx_s_start,
            Boundary3 boundary,
            std::size_t symmetric_num_stationary_max,
            double a_linearization_floor = 1.0e-10);

        ConstraintsRef constraints() const noexcept { return constraints_; }
        const std::vector<double> &a_linearization() const noexcept { return a_linearization_; }
        std::size_t idx_s_start() const noexcept { return idx_s_start_; }
        std::size_t idx_s_final() const noexcept { return idx_s_final_; }
        Boundary3 boundary() const noexcept { return boundary_; }
        StationaryBounds num_stationary_max() const noexcept { return num_stationary_max_; }
        StationaryBounds num_stationary() const noexcept { return num_stationary_; }
        double a_linearization_floor() const noexcept { return a_linearization_floor_; }
        std::size_t s_len() const noexcept { return a_linearization_.size(); }

    private:
        ConstraintsRef constraints_;
        std::vector<double> a_linearization_;
        std::size_t idx_s_start_ = 0;
        std::size_t idx_s_final_ = 0;
        Boundary3 boundary_;
        StationaryBounds num_stationary_max_;
        StationaryBounds num_stationary_;
        double a_linearization_floor_ = 1.0e-10;
    };

    /// Expert third-order Clarabel result.
    ///
    /// `profile` is populated only when the solver status was accepted by the
    /// provided options. Raw Clarabel vectors and linear-solver metadata are
    /// copied whenever the solve itself succeeds. COPP3 fills
    /// `objective_value/objective_terms`; TOPP3 leaves them empty.
    ///
    /// Use normal `solve` APIs when only the accepted profile matters; use
    /// `solve_expert` when diagnostics, solver status, or objective breakdowns
    /// are part of the workflow.
    struct Result
    {
        std::optional<Profile3rd> profile;
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

        bool has_profile() const noexcept { return profile.has_value(); }
    };

} // namespace copp::solver::topp3
