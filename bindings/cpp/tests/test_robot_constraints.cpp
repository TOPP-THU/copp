// Test purpose: exercise the C++ Robot and independent Constraints facades.
//
// It verifies station management, path-derivative storage, physical limits,
// raw first-/second-/third-order constraints, and buffer pop/clear operations.

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
    const std::vector<double> upper{3.0, 3.0};
    const std::vector<double> lower{-3.0, -3.0};

    auto path = copp::Path::from_parametric(
        [](copp::Jet3 x)
        {
            return std::vector<copp::Jet3>{
                copp::powi(x, 3),
                copp::sin(x),
            };
        },
        0.0,
        1.0);

    copp::Robot robot(2, s.size());
    robot.append_s(s)
        .set_q_from_path_3rd(path, 0, s.size())
        .add_velocity_limits(upper, lower, 0, s.size())
        .add_acceleration_limits(upper, lower, 0, s.size())
        .add_jerk_limits(upper, lower, 0, s.size())
        .add_torque_limits(upper, lower, 0, s.size());

    if (robot.dim() != 2 || robot.len() != s.size())
    {
        return fail("unexpected robot metadata");
    }

    auto constraints = robot.constraints();
    const auto range = constraints.idx_s_range();
    if (range.first != 0 || range.second != s.size())
    {
        return fail("unexpected constraint range");
    }

    const auto exported_s = constraints.s_values(0, s.size());
    if (exported_s.size() != s.size() || !near(exported_s[1], 0.5))
    {
        return fail("unexpected exported station values");
    }
    if (!std::isfinite(constraints.amax_value(1)))
    {
        return fail("expected finite velocity-derived amax");
    }

    const std::vector<double> replacement_amax{1.0, 0.8, 1.0};
    constraints.amax_substitute(replacement_amax, 0);
    const auto amax = constraints.amax_values(0, s.size());
    if (!near(amax[1], 0.8))
    {
        return fail("unexpected substituted amax");
    }

    copp::Constraints raw(1, s.size());
    raw.append_s(s).add_constraint_1st(replacement_amax, 0);
    if (!near(raw.amax_value(1), 0.8))
    {
        return fail("unexpected raw first-order constraint");
    }

    auto zeros = copp::Matrix::from_rows({
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
    });
    auto signed_rows = copp::Matrix::from_rows({
        {1.0, 1.0, 1.0},
        {-1.0, -1.0, -1.0},
    });
    auto bounds = copp::Matrix::from_rows({
        {2.0, 2.0, 2.0},
        {2.0, 2.0, 2.0},
    });

    raw.add_constraint_2nd(zeros.view(), signed_rows.view(), bounds.view(), 0)
        .add_constraint_3rd(
            zeros.view(),
            zeros.view(),
            signed_rows.view(),
            zeros.view(),
            bounds.view(),
            0);

    raw.pop_front_n(1);
    if (raw.idx_s_range().first != 1)
    {
        return fail("unexpected pop_front_n result");
    }
    raw.pop_back_until(2);
    if (raw.idx_s_range().second != 2)
    {
        return fail("unexpected pop_back_until result");
    }
    raw.clear();
    if (!raw.is_empty())
    {
        return fail("raw constraints should be empty after clear");
    }

    return 0;
}
