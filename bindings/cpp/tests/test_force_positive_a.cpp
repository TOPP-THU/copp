// Test purpose: verify `Profile3rd::force_positive_a` in-place
// post-processing, its typical use before `s_to_t_topp3` on a solved
// third-order profile, and its invalid-input error paths.

#include <algorithm>
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
    const std::vector<double> s{0.0, 0.25, 0.5, 0.75, 1.0};

    copp::Profile3rd flat{std::vector<double>(s.size(), 1.0), std::vector<double>(s.size(), 0.0)};
    if (!flat.force_positive_a(s))
    {
        return fail("positive profile should be accepted");
    }
    if (flat.a != std::vector<double>(s.size(), 1.0) || flat.b != std::vector<double>(s.size(), 0.0))
    {
        return fail("positive profile should stay unchanged");
    }

    // b changes sign across [s[2], s[3]] while a[2] is small, so the
    // interpolated a(s) would become negative inside that interval.
    // On [s[2], s[3]], a(s) = a[2] + 2 b[2] t + (b[3] - b[2]) / ds t^2 with t = s - s[2].
    const auto min_a_on_dip_interval = [&s](const copp::Profile3rd &profile)
    {
        const double ds = s[3] - s[2];
        double min_a = profile.a[2];
        for (int k = 0; k <= 200; ++k)
        {
            const double t = ds * static_cast<double>(k) / 200.0;
            const double a_t = profile.a[2] + 2.0 * profile.b[2] * t +
                               (profile.b[3] - profile.b[2]) / ds * t * t;
            min_a = std::min(min_a, a_t);
        }
        return min_a;
    };
    copp::Profile3rd dip{{1.0, 1.0, 1.0e-3, 1.0, 1.0}, {0.0, 0.0, -1.0, 1.0, 0.0}};
    if (!(min_a_on_dip_interval(dip) < 0.0))
    {
        return fail("test profile should dip below zero before repair");
    }
    const auto before = dip;
    if (!dip.force_positive_a(s, 1.0e-12))
    {
        return fail("dipping profile should be repaired");
    }
    if (dip.a == before.a && dip.b == before.b)
    {
        return fail("dipping profile should be adjusted in place");
    }
    for (double a_node : dip.a)
    {
        if (!(a_node > 0.0))
        {
            return fail("repaired profile should keep a positive at the nodes");
        }
    }
    if (!(min_a_on_dip_interval(dip) > 0.0))
    {
        return fail("repaired profile should keep a(s) positive on the dipping interval");
    }

    const auto short_s = std::vector<double>{0.0, 0.5, 1.0};
    copp::Profile3rd short_profile{{1.0, 1.0, 1.0}, {0.0, 0.0, 0.0}};
    try
    {
        (void)short_profile.force_positive_a(short_s);
        return fail("expected fewer than four stations to throw");
    }
    catch (const copp::Error &error)
    {
        if (error.status() != copp::Status::invalid_input)
        {
            return fail("unexpected force_positive_a error status");
        }
    }
    if (flat.force_positive_a(short_s, copp::no_throw))
    {
        return fail("expected no_throw length mismatch failure");
    }
    const auto no_throw_ok = flat.force_positive_a(s, 1.0e-12, copp::no_throw);
    if (!no_throw_ok || !no_throw_ok.value())
    {
        return fail("unexpected no_throw force_positive_a result");
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
    copp::solver::topp3::Problem problem{
        robot.constraints(),
        std::vector<double>(grid.size(), 1.0),
        0,
        copp::Boundary3{0.0, 0.0, 0.0, 0.0},
        1,
    };
    copp::clarabel::Options options;
    options.allow_almost_solved = true;

    auto profile = copp::solver::topp3_lp::solve(problem, options);
    (void)profile.force_positive_a(grid);
    if (profile.len() != grid.size())
    {
        return fail("force_positive_a changed the profile length");
    }
    const auto time = copp::interpolation::s_to_t_topp3(grid, profile);
    if (!std::isfinite(time.t_final) || time.t_final <= 0.0)
    {
        return fail("unexpected time interpolation after force_positive_a");
    }

    return 0;
}
