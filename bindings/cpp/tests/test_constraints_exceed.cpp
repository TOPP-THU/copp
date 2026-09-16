// Test purpose: verify `exceed_topp2` / `exceed_topp3` on `Constraints` and
// `ConstraintsRef`: solver profiles report non-positive (or tiny) violations,
// obviously infeasible profiles report positive violations, and unusable
// inputs report NaN.

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
    const std::vector<double> s{0.0, 0.5, 1.0};
    const std::vector<double> amax{1.0, 1.0, 1.0};

    copp::Constraints constraints(1, s.size());
    constraints.append_s(s).add_constraint_1st(amax, 0);

    copp::solver::topp2_ra::Problem problem2{
        constraints.ref(),
        copp::IndexInterval{0, s.size() - 1},
        copp::Boundary2{0.0, 0.0},
    };
    const auto a = copp::solver::topp2_ra::solve(problem2);
    const auto feasible2 = constraints.exceed_topp2(a);
    if (!(feasible2[0] <= 1.0e-8) || !(feasible2[1] <= 1.0e-8))
    {
        return fail("TOPP2-RA profile should satisfy its constraints");
    }
    const std::vector<double> too_fast{0.5, 2.0, 0.5};
    const auto violated2 = constraints.ref().exceed_topp2(too_fast);
    if (!(violated2[0] > 0.5))
    {
        return fail("exceed_topp2 should report the first-order violation");
    }
    const auto unavailable2 = constraints.exceed_topp2(a, 5);
    if (!std::isnan(unavailable2[0]) || !std::isnan(unavailable2[1]))
    {
        return fail("exceed_topp2 should return NaN for an unavailable station range");
    }

    const std::vector<double> grid{0.0, 0.1666666667, 0.3333333333, 0.5, 0.6666666667, 0.8333333333, 1.0};
    const std::vector<double> upper{1.0, 1.0};
    const std::vector<double> lower{-1.0, -1.0};
    auto path = copp::Path::from_parametric(
        [](copp::Jet3 x)
        {
            return std::vector<copp::Jet3>{x, 2.0 * x};
        },
        0.0,
        1.0);
    copp::Robot robot(2, grid.size());
    robot.append_s(grid)
        .set_q_from_path_3rd(path, 0, grid.size())
        .add_velocity_limits(upper, lower, 0, grid.size())
        .add_acceleration_limits(upper, lower, 0, grid.size())
        .add_jerk_limits(upper, lower, 0, grid.size());
    copp::solver::topp3::Problem problem3{
        robot.constraints(),
        std::vector<double>(grid.size(), 1.0),
        0,
        copp::Boundary3{0.0, 0.0, 0.0, 0.0},
        1,
    };
    copp::clarabel::Options options;
    options.allow_almost_solved = true;
    const auto profile = copp::solver::topp3_lp::solve(problem3, options);

    const auto feasible3 = robot.constraints().exceed_topp3(profile);
    for (double value : feasible3)
    {
        if (!(value <= 1.0e-6))
        {
            std::cerr << "exceed_topp3 = {" << feasible3[0] << ", " << feasible3[1] << ", "
                      << feasible3[2] << "}\n";
            return fail("TOPP3-LP profile should satisfy its nonlinear constraints");
        }
    }

    auto too_fast3 = profile;
    too_fast3.a[grid.size() / 2] = 1.0e6;
    const auto violated3 = robot.constraints().exceed_topp3(
        too_fast3.a,
        too_fast3.b,
        too_fast3.num_stationary_start,
        too_fast3.num_stationary_end);
    if (!(violated3[0] > 0.0))
    {
        return fail("exceed_topp3 should report the first-order violation");
    }

    const std::vector<double> short_b(profile.b.begin(), profile.b.end() - 1);
    const auto mismatched3 = robot.constraints().exceed_topp3(profile.a, short_b, 1, 1);
    if (!std::isnan(mismatched3[0]) || !std::isnan(mismatched3[1]) || !std::isnan(mismatched3[2]))
    {
        return fail("exceed_topp3 should return NaN for mismatched profile lengths");
    }

    const std::vector<double> flat_b{0.0, 0.0, 0.0};
    const auto raw3 = constraints.exceed_topp3(a, flat_b, 0, 0);
    if (!(raw3[0] <= 1.0e-8))
    {
        return fail("Constraints::exceed_topp3 should accept a feasible first-order profile");
    }

    return 0;
}
