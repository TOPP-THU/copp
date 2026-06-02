#pragma once

#include <cstddef>
#include <vector>

#include "copp/core.hpp"
#include "copp/robot.hpp"

namespace copp::solver::topp2_ra
{

    /// Options shared by TOPP2-RA and TOPP2 reachable-set construction.
    ///
    /// Defaults mirror Rust/Python:
    ///
    /// - `lp_feas_tol = 1e-8`
    /// - `a_cmp_abs_tol = 1e-8`
    /// - `a_cmp_rel_tol = 1e-8`
    /// - `verbosity = Verbosity::Silent`
    struct Options
    {
        double lp_feas_tol = 1.0e-8;
        double a_cmp_abs_tol = 1.0e-8;
        double a_cmp_rel_tol = 1.0e-8;
        Verbosity verbosity = Verbosity::Silent;
    };

    /// Reachable interval result for second-order state `a = (ds/dt)^2`.
    ///
    /// For each station in the problem interval, the feasible interval is
    /// `a_min[k] <= a[k] <= a_max[k]`.
    struct ReachSet
    {
        std::vector<double> a_max;
        std::vector<double> a_min;

        std::size_t len() const noexcept { return a_max.size(); }
        bool empty() const noexcept { return a_max.empty(); }
    };

    /// TOPP2 problem descriptor.
    ///
    /// This object is a non-owning descriptor over a `ConstraintsRef`. The
    /// referenced `Robot` or owning `Constraints` object must outlive the
    /// problem and all solver calls using it. Construction validates the closed
    /// station interval and boundary values against the current constraints.
    class COPP_API Problem
    {
    public:
        Problem(
            ConstraintsRef constraints,
            IndexInterval idx_s_interval,
            Boundary2 a_boundary = {});

        ConstraintsRef constraints() const noexcept { return constraints_; }
        IndexInterval idx_s_interval() const noexcept { return idx_s_interval_; }
        Boundary2 a_boundary() const noexcept { return a_boundary_; }
        std::size_t s_len() const noexcept { return s_len_; }

    private:
        ConstraintsRef constraints_;
        IndexInterval idx_s_interval_;
        Boundary2 a_boundary_;
        std::size_t s_len_ = 0;
    };

    /// Compute backward-only TOPP2 reachable intervals.
    ///
    /// @param problem Borrowed problem descriptor.
    /// @param options Numeric tolerances and verbosity.
    /// @return Reachable `a` interval at each station.
    /// @throws copp::Error if Rust reachability fails.
    COPP_API ReachSet reach_set_backward(const Problem &problem, const Options &options = {});
    inline Expected<ReachSet> reach_set_backward(
        const Problem &problem,
        const Options &options,
        NoThrowTag)
    {
        return detail::expected_from([&] { return reach_set_backward(problem, options); });
    }
    inline Expected<ReachSet> reach_set_backward(const Problem &problem, NoThrowTag tag)
    {
        return reach_set_backward(problem, Options{}, tag);
    }

    /// Compute bidirectional TOPP2 reachable intervals, enforcing both boundaries.
    ///
    /// This is the reachable-set diagnostic companion to `solve`: it keeps both
    /// lower and upper interval envelopes instead of selecting one profile.
    COPP_API ReachSet reach_set_bidirectional(const Problem &problem, const Options &options = {});
    inline Expected<ReachSet> reach_set_bidirectional(
        const Problem &problem,
        const Options &options,
        NoThrowTag)
    {
        return detail::expected_from([&] { return reach_set_bidirectional(problem, options); });
    }
    inline Expected<ReachSet> reach_set_bidirectional(const Problem &problem, NoThrowTag tag)
    {
        return reach_set_bidirectional(problem, Options{}, tag);
    }

    /// Solve TOPP2 with reachability analysis.
    ///
    /// Returns the node profile `a(s) = (ds/dt)^2` over the problem's closed
    /// station interval.
    ///
    /// @code
    /// copp::solver::topp2_ra::Problem problem{
    ///     robot.constraints(),
    ///     copp::IndexInterval{0, s.size() - 1},
    ///     copp::Boundary2{0.0, 0.0},
    /// };
    /// auto a = copp::solver::topp2_ra::solve(problem);
    /// @endcode
    COPP_API std::vector<double> solve(const Problem &problem, const Options &options = {});
    inline Expected<std::vector<double>> solve(
        const Problem &problem,
        const Options &options,
        NoThrowTag)
    {
        return detail::expected_from([&] { return solve(problem, options); });
    }
    inline Expected<std::vector<double>> solve(const Problem &problem, NoThrowTag tag)
    {
        return solve(problem, Options{}, tag);
    }

} // namespace copp::solver::topp2_ra
