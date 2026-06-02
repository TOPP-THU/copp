#pragma once

#include "copp/clarabel.hpp"
#include "copp/interpolation.hpp"
#include "copp/solver/topp3.hpp"

namespace copp::solver::topp3_socp
{

    using Problem = copp::solver::topp3::Problem;
    using Result = copp::solver::topp3::Result;

    /// Solve TOPP3-SOCP and return an accepted third-order profile.
    ///
    /// A common SCP workflow solves once with an initial `a_linearization`,
    /// then rebuilds `topp3_socp::Problem` with the returned `profile.a` and
    /// solves again. `topp3_socp::Problem` is an alias for the shared
    /// third-order `copp::solver::topp3::Problem` descriptor used by TOPP3-LP,
    /// TOPP3-SOCP, and COPP3-SOCP.
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

    /// Solve TOPP3-SOCP and always return Clarabel diagnostics.
    ///
    /// TOPP3 expert results leave COPP objective diagnostics empty because no
    /// COPP objective list is part of the problem.
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

} // namespace copp::solver::topp3_socp
