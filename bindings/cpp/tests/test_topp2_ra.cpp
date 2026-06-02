// Test purpose: solve a minimal TOPP2-RA problem through the C++ facade and
// verify the returned `a(s)` profile respects boundary and upper-bound checks.

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
        return std::abs(lhs - rhs) < 1.0e-10;
    }
} // namespace

int main()
{
    const std::vector<double> s{0.0, 0.5, 1.0};
    const std::vector<double> amax{1.0, 1.0, 1.0};

    copp::Constraints constraints(1, s.size());
    constraints.append_s(s).add_constraint_1st(amax, 0);

    namespace topp2 = copp::solver::topp2_ra;

    topp2::Problem problem{
        constraints.ref(),
        copp::IndexInterval{0, s.size() - 1},
        copp::Boundary2{0.0, 0.0},
    };

    if (problem.s_len() != s.size())
    {
        return fail("unexpected TOPP2 problem length");
    }

    const auto reach_backward = topp2::reach_set_backward(problem);
    const auto reach_bidirectional = topp2::reach_set_bidirectional(problem);
    const auto a = topp2::solve(problem);

    if (reach_backward.len() != s.size() || reach_bidirectional.len() != s.size())
    {
        return fail("unexpected reach set length");
    }
    if (a.size() != s.size())
    {
        return fail("unexpected a profile length");
    }
    if (!near(a.front(), 0.0) || !near(a.back(), 0.0))
    {
        return fail("unexpected boundary values");
    }
    if (!std::isfinite(a[1]) || a[1] < 0.0 || a[1] > 1.0 + 1.0e-8)
    {
        return fail("unexpected interior a value");
    }

    const auto time = copp::interpolation::s_to_t_topp2(s, a, 0.0);
    const auto s_t = copp::interpolation::t_to_s_topp2_uniform(s, a, time.t_s, 0.25);
    if (s_t.empty() || !near(s_t.front(), 0.0) || !near(s_t.back(), 1.0))
    {
        return fail("unexpected interpolated s(t)");
    }

    return 0;
}
