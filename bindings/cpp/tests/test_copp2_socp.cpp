// Test purpose: solve COPP2-SOCP in both normal and expert modes. The expert
// branch verifies Clarabel status conversion, raw vector copying, and objective
// diagnostics.

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

    copp::Robot robot(1, s.size());
    robot.append_s(s);
    robot.constraints().add_constraint_1st(amax, 0);

    namespace copp2 = copp::solver::copp2_socp;

    copp2::Problem problem{
        robot,
        {copp::objective::Time(1.0)},
        copp::IndexInterval{0, s.size() - 1},
        copp::Boundary2{0.0, 0.0},
    };

    if (problem.s_len() != s.size())
    {
        return fail("unexpected COPP2 problem length");
    }

    copp::clarabel::Options options;
    options.clarabel_settings.max_iter = 200;
    options.allow_almost_solved = true;

    const auto a = copp2::solve(problem, options);
    if (a.size() != s.size())
    {
        return fail("unexpected a profile length");
    }
    if (!near(a.front(), 0.0) || !near(a.back(), 0.0))
    {
        return fail("unexpected boundary values");
    }
    if (!std::isfinite(a[1]) || a[1] < -1.0e-10)
    {
        return fail("unexpected interior a value");
    }

    const auto expert = copp2::solve_expert(problem, options);
    if (!accepted(expert.solver_status))
    {
        return fail("unexpected Clarabel status");
    }
    if (!expert.has_a() || expert.a->size() != s.size())
    {
        return fail("expert result did not include accepted a");
    }
    if (expert.x.empty() || expert.s.empty() || expert.z.empty())
    {
        return fail("expert result missed raw Clarabel vectors");
    }
    if (!expert.objective_value.has_value() || expert.objective_terms.size() != 1)
    {
        return fail("expert result missed objective diagnostics");
    }

    return 0;
}
