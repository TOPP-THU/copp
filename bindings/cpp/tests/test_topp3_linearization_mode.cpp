// Test purpose: verify the optional `b_linearization` problem argument. A
// solved third-order profile is fed back as `(a_linearization,
// b_linearization)` to select adaptive anchoring for TOPP3 and COPP3.

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

} // namespace

int main()
{
    const std::vector<double> s{0.0, 0.1666666667, 0.3333333333, 0.5, 0.6666666667, 0.8333333333, 1.0};
    const std::vector<double> upper{1.0, 1.0};
    const std::vector<double> lower{-1.0, -1.0};
    const std::vector<double> a_linearization(s.size(), 1.0);
    const copp::Boundary3 boundary{0.0, 0.0, 0.0, 0.0};

    auto path = copp::Path::from_parametric(
        [](copp::Jet3 x)
        {
            return std::vector<copp::Jet3>{x, 2.0 * x};
        },
        0.0,
        1.0);

    copp::Robot robot(2, s.size());
    robot.append_s(s)
        .set_q_from_path_3rd(path, 0, s.size())
        .add_velocity_limits(upper, lower, 0, s.size())
        .add_acceleration_limits(upper, lower, 0, s.size())
        .add_jerk_limits(upper, lower, 0, s.size());

    copp::clarabel::Options options;
    options.allow_almost_solved = true;
    options.allow_max_iterations = true;
    options.allow_insufficient_progress = true;

    copp::solver::topp3::Problem problem{robot.constraints(), a_linearization, 0, boundary, 1};
    if (!problem.b_linearization().empty())
    {
        return fail("default TOPP3 problem should use direct linearization");
    }

    const auto first = copp::solver::topp3_lp::solve(problem, options);

    copp::solver::topp3::Problem refined{
        robot.constraints(),
        first.a,
        0,
        boundary,
        1,
        1.0e-10,
        first.b,
    };
    if (refined.b_linearization() != first.b || refined.a_linearization() != first.a)
    {
        return fail("TOPP3 problem should copy the linearization profile");
    }

    // Adaptive anchoring must still yield a solvable third-order problem on
    // both Clarabel backends.
    const auto second = copp::solver::topp3_lp::solve(refined, options);
    if (second.len() != s.size())
    {
        return fail("unexpected adaptive TOPP3-LP profile");
    }
    const auto time = copp::interpolation::s_to_t_topp3(s, second);
    if (!std::isfinite(time.t_final) || time.t_final <= 0.0)
    {
        return fail("unexpected adaptive TOPP3-LP time interpolation");
    }

    const auto second_socp = copp::solver::topp3_socp::solve(refined, options);
    if (second_socp.len() != s.size())
    {
        return fail("unexpected adaptive TOPP3-SOCP profile");
    }

    const std::vector<double> short_b(first.b.begin(), first.b.end() - 1);
    try
    {
        copp::solver::topp3::Problem mismatched{
            robot.constraints(),
            first.a,
            0,
            boundary,
            1,
            1.0e-10,
            short_b,
        };
        return fail("expected mismatched b_linearization to throw");
    }
    catch (const copp::Error &error)
    {
        if (error.status() != copp::Status::invalid_input)
        {
            return fail("unexpected mismatched b_linearization error status");
        }
    }

    namespace copp3 = copp::solver::copp3_socp;
    copp3::Problem copp_problem{
        robot,
        {copp::objective::Time(1.0)},
        first.a,
        0,
        boundary,
        1,
        1.0e-10,
        first.b,
    };
    if (copp_problem.b_linearization() != first.b)
    {
        return fail("COPP3 problem should copy b_linearization");
    }

    const auto copp_profile = copp3::solve(copp_problem, options);
    if (copp_profile.len() != s.size())
    {
        return fail("unexpected adaptive COPP3-SOCP profile");
    }

    return 0;
}
