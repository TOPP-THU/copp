// Test purpose: solve TOPP3-LP and validate both the normal accepted-profile
// path and the expert-result path with Clarabel diagnostics.

#include <cmath>
#include <iostream>
#include <vector>

#include <copp/copp.hpp>

namespace
{

    int fail(const char *message)
    {
        std::cerr << message << "\n";
        return 1;
    }

    bool near(double lhs, double rhs)
    {
        return std::abs(lhs - rhs) < 1.0e-8;
    }

    bool accepted(copp::clarabel::SolverStatus status)
    {
        return status == copp::clarabel::SolverStatus::Solved ||
               status == copp::clarabel::SolverStatus::AlmostSolved;
    }

} // namespace

int main()
{
    const std::vector<double> s{0.0, 0.5, 1.0};
    const std::vector<double> amax{1.0, 1.0, 1.0};
    const std::vector<double> a_linearization{0.25, 0.25, 0.25};

    copp::Constraints constraints(1, s.size());
    constraints.append_s(s)
        .add_constraint_1st(amax, 0);

    const copp::Boundary3 boundary{0.25, 0.25, 0.0, 0.0};
    copp::solver::topp3::Problem problem{
        constraints.ref(),
        a_linearization,
        0,
        boundary,
    };

    copp::clarabel::Options options;
    options.allow_almost_solved = true;

    const auto profile = copp::solver::topp3_lp::solve(problem, options);
    if (profile.len() != s.size())
    {
        return fail("unexpected TOPP3-LP profile length");
    }
    if (!near(profile.a.front(), boundary.a_start) || !near(profile.a.back(), boundary.a_final))
    {
        return fail("unexpected TOPP3-LP a boundary");
    }
    if (!near(profile.b.front(), boundary.b_start) || !near(profile.b.back(), boundary.b_final))
    {
        return fail("unexpected TOPP3-LP b boundary");
    }

    const auto expert = copp::solver::topp3_lp::solve_expert(problem, options);
    if (!accepted(expert.solver_status))
    {
        return fail("unexpected TOPP3-LP Clarabel status");
    }
    if (!expert.has_profile() || expert.profile->len() != s.size())
    {
        return fail("TOPP3-LP expert result did not include an accepted profile");
    }
    if (expert.x.empty() || expert.s.empty() || expert.z.empty())
    {
        return fail("TOPP3-LP expert result missed raw Clarabel vectors");
    }
    if (expert.objective_value.has_value() || !expert.objective_terms.empty())
    {
        return fail("TOPP3-LP should not report COPP objective diagnostics");
    }

    const auto time = copp::interpolation::s_to_t_topp3(s, profile);
    if (!std::isfinite(time.t_final) || time.t_s.size() != s.size())
    {
        return fail("unexpected TOPP3-LP interpolation result");
    }

    return 0;
}
