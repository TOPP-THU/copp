#pragma once

#include "copp/clarabel.hpp"
#include "copp/interpolation.hpp"
#include "copp/solver/topp3.hpp"

namespace copp::solver::topp3_lp
{

    using Problem = copp::solver::topp3::Problem;
    using Result = copp::solver::topp3::Result;

    /// Solve TOPP3-LP and return an accepted third-order profile.
    ///
    /// LP is the lightest Clarabel-backed third-order formulation. It uses the
    /// already linearized problem rows prepared by `topp3::Problem`.
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

    /// Solve TOPP3-LP and always return Clarabel diagnostics.
    ///
    /// Use this to inspect solver status, residuals, raw conic variables, and
    /// linear-solver metadata even when no profile is accepted.
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

} // namespace copp::solver::topp3_lp
