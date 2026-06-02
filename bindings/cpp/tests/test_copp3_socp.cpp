// Test purpose: solve COPP3-SOCP and verify profile recovery, Clarabel expert
// diagnostics, and COPP objective value/term copying.

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

    copp::Robot robot(1, s.size());
    robot.append_s(s);
    robot.constraints().add_constraint_1st(amax, 0);

    namespace copp3 = copp::solver::copp3_socp;

    const copp::Boundary3 boundary{0.25, 0.25, 0.0, 0.0};
    copp3::Problem problem{
        robot,
        {copp::objective::Time(1.0)},
        a_linearization,
        0,
        boundary,
    };

    if (problem.s_len() != s.size() || problem.idx_s_final() != s.size() - 1)
    {
        return fail("unexpected COPP3-SOCP problem interval");
    }

    copp::clarabel::Options options;
    options.allow_almost_solved = true;

    const auto profile = copp3::solve(problem, options);
    if (profile.len() != s.size())
    {
        return fail("unexpected COPP3-SOCP profile length");
    }
    if (!near(profile.a.front(), boundary.a_start) || !near(profile.a.back(), boundary.a_final))
    {
        return fail("unexpected COPP3-SOCP a boundary");
    }

    const auto expert = copp3::solve_expert(problem, options);
    if (!accepted(expert.solver_status))
    {
        return fail("unexpected COPP3-SOCP Clarabel status");
    }
    if (!expert.has_profile() || expert.profile->len() != s.size())
    {
        return fail("COPP3-SOCP expert result did not include an accepted profile");
    }
    if (!expert.objective_value.has_value() || expert.objective_terms.size() != 1)
    {
        return fail("COPP3-SOCP expert result missed objective diagnostics");
    }

    return 0;
}
