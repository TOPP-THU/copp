// Test purpose: cover the interpolating waypoint constructors, the
// `from_waypoints` alias, and tolerance-bounded waypoint fitting with
// `smoothing_report()`, including throwing and no-throw error paths.

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <copp/copp.hpp>

namespace
{
    bool near(double lhs, double rhs)
    {
        return std::abs(lhs - rhs) < 1.0e-9;
    }

    int fail(const char *message)
    {
        std::cerr << message << "\n";
        return 1;
    }
} // namespace

int main()
{
    auto interpolated = copp::Path::from_waypoints_interpolating({
        {0.0, 0.0},
        {0.5, 0.25},
        {1.0, 1.0},
    });
    const std::vector<double> s{0.0, 0.25, 0.5, 0.75, 1.0};
    const auto interpolated_q = interpolated.evaluate_q(s);
    if (!near(interpolated_q.q(0, 2), 0.5) || !near(interpolated_q.q(1, 2), 0.25))
    {
        return fail("interpolating path missed the middle waypoint");
    }
    if (interpolated.smoothing_report().has_value())
    {
        return fail("interpolated path should not carry a smoothing report");
    }

    auto waypoints = copp::Matrix::from_columns({
        {0.0, 0.0},
        {0.5, 0.25},
        {1.0, 1.0},
    });
    const auto alias_q = copp::Path::from_waypoints(waypoints.view()).evaluate_q(s);
    const auto view_q = copp::Path::from_waypoints_interpolating(waypoints.view()).evaluate_q(s);
    for (std::size_t j = 0; j < s.size(); ++j)
    {
        if (!near(alias_q.q(0, j), view_q.q(0, j)) || !near(alias_q.q(1, j), view_q.q(1, j)) ||
            !near(view_q.q(0, j), interpolated_q.q(0, j)))
        {
            return fail("from_waypoints and from_waypoints_interpolating disagree");
        }
    }
    if (copp::Path::from_waypoints_interpolating({{0.0}}, copp::no_throw))
    {
        return fail("expected invalid interpolating path to fail");
    }

    auto noisy = copp::Matrix::from_columns({
        {0.0, 0.0},
        {0.25, 0.1},
        {0.5, -0.1},
        {0.75, 0.2},
        {1.0, 0.0},
    });
    const copp::SmoothingConfig config;
    auto fitted = copp::Path::from_waypoints_fitting(noisy.view(), config);
    const auto report = fitted.smoothing_report();
    if (!report)
    {
        return fail("fitted path should carry a smoothing report");
    }
    if (report->axes != std::vector<std::size_t>{0, 1} || report->max_errors.size() != 2 ||
        report->segments == 0)
    {
        return fail("unexpected smoothing report shape");
    }
    for (double error : report->max_errors)
    {
        if (!(error <= config.tolerance))
        {
            return fail("smoothing report exceeds the requested tolerance");
        }
    }
    const auto fitted_q = fitted.evaluate_q(std::vector<double>{0.0, 1.0});
    if (!near(fitted_q.q(0, 0), 0.0) || !near(fitted_q.q(1, 0), 0.0) ||
        !near(fitted_q.q(0, 1), 1.0) || !near(fitted_q.q(1, 1), 0.0))
    {
        return fail("fitted path should retain endpoint positions");
    }

    copp::SmoothingConfig partial_config;
    partial_config.axes = std::vector<std::size_t>{1};
    partial_config.tolerance_per_axis = {1.0e-2};
    partial_config.parameters = std::vector<double>{2.0, 2.5, 3.0, 3.5, 4.0};
    auto partial = copp::Path::from_waypoints_fitting(
        {
            {0.0, 0.0},
            {0.25, 0.1},
            {0.5, -0.1},
            {0.75, 0.2},
            {1.0, 0.0},
        },
        partial_config);
    const auto range = partial.s_range();
    if (!near(range.first, 2.0) || !near(range.second, 4.0))
    {
        return fail("explicit fitting parameters should define the path range");
    }
    const auto partial_report = partial.smoothing_report();
    if (!partial_report || partial_report->axes != std::vector<std::size_t>{1} ||
        partial_report->max_errors.size() != 1 || !(partial_report->max_errors[0] <= 1.0e-2))
    {
        return fail("unexpected per-axis smoothing report");
    }
    const auto partial_q = partial.evaluate_q(std::vector<double>{2.5, 3.0});
    if (!near(partial_q.q(0, 0), 0.25) || !near(partial_q.q(0, 1), 0.5))
    {
        return fail("unselected axis should interpolate every waypoint");
    }

    copp::SmoothingConfig invalid_config;
    invalid_config.tolerance = 0.0;
    try
    {
        (void)copp::Path::from_waypoints_fitting(noisy.view(), invalid_config);
        return fail("expected invalid fitting tolerance to throw");
    }
    catch (const copp::Error &error)
    {
        if (error.status() != copp::Status::invalid_input || std::string(error.what()).empty())
        {
            return fail("unexpected fitting error payload");
        }
    }

    auto failed = copp::Path::from_waypoints_fitting(noisy.view(), invalid_config, copp::no_throw);
    if (failed || failed.error().status != copp::Status::invalid_input || failed.error().message.empty())
    {
        return fail("expected no_throw fitting failure");
    }

    copp::SmoothingConfig empty_axes;
    empty_axes.axes = std::vector<std::size_t>{};
    if (copp::Path::from_waypoints_fitting(noisy.view(), empty_axes, copp::no_throw))
    {
        return fail("an explicit empty axis list should be rejected");
    }

    auto fitted_no_throw = copp::Path::from_waypoints_fitting(noisy.view(), copp::no_throw);
    if (!fitted_no_throw || !fitted_no_throw.value().smoothing_report())
    {
        return fail("unexpected no_throw fitting result");
    }

    return 0;
}
