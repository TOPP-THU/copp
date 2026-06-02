#pragma once

#include "copp/solver/topp2_ra.hpp"

namespace copp::solver::reach_set2
{

    using Options = copp::solver::topp2_ra::Options;
    using Problem = copp::solver::topp2_ra::Problem;
    using ReachSet = copp::solver::topp2_ra::ReachSet;

    /// Compute backward-only TOPP2 reachable intervals.
    inline ReachSet backward(const Problem &problem, const Options &options = {})
    {
        return copp::solver::topp2_ra::reach_set_backward(problem, options);
    }
    inline Expected<ReachSet> backward(const Problem &problem, const Options &options, NoThrowTag)
    {
        return detail::expected_from([&] { return backward(problem, options); });
    }
    inline Expected<ReachSet> backward(const Problem &problem, NoThrowTag tag)
    {
        return backward(problem, Options{}, tag);
    }

    /// Compute bidirectional TOPP2 reachable intervals, enforcing both boundaries.
    inline ReachSet bidirectional(const Problem &problem, const Options &options = {})
    {
        return copp::solver::topp2_ra::reach_set_bidirectional(problem, options);
    }
    inline Expected<ReachSet> bidirectional(const Problem &problem, const Options &options, NoThrowTag)
    {
        return detail::expected_from([&] { return bidirectional(problem, options); });
    }
    inline Expected<ReachSet> bidirectional(const Problem &problem, NoThrowTag tag)
    {
        return bidirectional(problem, Options{}, tag);
    }

} // namespace copp::solver::reach_set2
