// Test purpose: cover all first-version C++ Path construction routes.
//
// The test checks waypoint splines, second-/third-order batch evaluators, and
// Jet3 parametric paths. It also verifies result matrix shapes and derivative
// availability (`evaluate_q`, `evaluate_up_to_2nd`, `evaluate_up_to_3rd`).

#include <cmath>
#include <iostream>
#include <vector>

#include <copp/copp.hpp>

namespace
{
    bool near(double lhs, double rhs)
    {
        return std::abs(lhs - rhs) < 1.0e-10;
    }

    int fail(const char *message)
    {
        std::cerr << message << "\n";
        return 1;
    }
} // namespace

int main()
{
    auto path = copp::Path::from_waypoints({
        {0.0, 0.0},
        {0.5, 0.25},
        {1.0, 1.0},
    });
    if (path.dim() != 2)
    {
        return fail("unexpected path dimension");
    }
    const auto range = path.s_range();
    if (!near(range.first, 0.0) || !near(range.second, 1.0))
    {
        return fail("unexpected path range");
    }

    const std::vector<double> s{0.0, 0.5, 1.0};
    const auto out = path.evaluate_up_to_2nd(s);
    if (out.q.rows() != 2 || out.q.cols() != 3)
    {
        return fail("unexpected q shape");
    }
    if (!out.dq.has_value() || !out.ddq.has_value() || out.dddq.has_value())
    {
        return fail("unexpected derivative availability");
    }

    if (!near(out.q(0, 0), 0.0) || !near(out.q(1, 0), 0.0))
    {
        return fail("unexpected first waypoint");
    }
    if (!near(out.q(0, 1), 0.5) || !near(out.q(1, 1), 0.25))
    {
        return fail("unexpected middle waypoint");
    }
    if (!near(out.q(0, 2), 1.0) || !near(out.q(1, 2), 1.0))
    {
        return fail("unexpected final waypoint");
    }

    const auto q_only = path.evaluate_q(s);
    if (q_only.dq.has_value() || q_only.ddq.has_value() || q_only.dddq.has_value())
    {
        return fail("evaluate_q returned derivatives");
    }

    copp::SplineConfig boundary_config;
    boundary_config.order = 3;
    boundary_config.start_state = copp::Matrix::from_columns({{2.0}});
    boundary_config.end_state = copp::Matrix::from_columns({{2.0}});
    auto boundary_path = copp::Path::from_waypoints({{0.0}, {1.0}}, boundary_config);
    const auto boundary_out = boundary_path.evaluate_up_to_2nd(std::vector<double>{0.0, 1.0});
    if (!near(boundary_out.dq.value()(0, 0), 2.0) || !near(boundary_out.dq.value()(0, 1), 2.0))
    {
        return fail("spline boundary derivative state was not applied");
    }

    auto evaluator_path = copp::Path::from_evaluator_2nd(
        2,
        0.0,
        1.0,
        [](copp::Span<const double> samples,
           copp::MatrixRef q,
           copp::MatrixRef dq,
           copp::MatrixRef ddq)
        {
            for (std::size_t j = 0; j < samples.size(); ++j)
            {
                const double x = samples[j];
                q(0, j) = x;
                dq(0, j) = 1.0;
                ddq(0, j) = 0.0;

                q(1, j) = x * x;
                dq(1, j) = 2.0 * x;
                ddq(1, j) = 2.0;
            }
        });

    const auto eval_out = evaluator_path.evaluate_up_to_2nd(s);
    if (!near(eval_out.q(0, 2), 1.0) || !near(eval_out.q(1, 2), 1.0))
    {
        return fail("unexpected evaluator q");
    }
    if (!near(eval_out.dq.value()(0, 2), 1.0) || !near(eval_out.dq.value()(1, 2), 2.0))
    {
        return fail("unexpected evaluator dq");
    }
    if (!near(eval_out.ddq.value()(0, 2), 0.0) || !near(eval_out.ddq.value()(1, 2), 2.0))
    {
        return fail("unexpected evaluator ddq");
    }

    auto evaluator3_path = copp::Path::from_evaluator_3rd(
        2,
        0.0,
        1.0,
        [](copp::Span<const double> samples,
           copp::MatrixRef q,
           copp::MatrixRef dq,
           copp::MatrixRef ddq,
           copp::MatrixRef dddq)
        {
            for (std::size_t j = 0; j < samples.size(); ++j)
            {
                const double x = samples[j];
                q(0, j) = x * x * x;
                dq(0, j) = 3.0 * x * x;
                ddq(0, j) = 6.0 * x;
                dddq(0, j) = 6.0;

                q(1, j) = x * x + 1.0;
                dq(1, j) = 2.0 * x;
                ddq(1, j) = 2.0;
                dddq(1, j) = 0.0;
            }
        });

    const auto eval3_out = evaluator3_path.evaluate_up_to_3rd(s);
    if (!eval3_out.dq.has_value() || !eval3_out.ddq.has_value() || !eval3_out.dddq.has_value())
    {
        return fail("third-order evaluator did not return all derivatives");
    }
    if (!near(eval3_out.q(0, 2), 1.0) || !near(eval3_out.q(1, 2), 2.0))
    {
        return fail("unexpected third-order evaluator q");
    }
    if (!near(eval3_out.dddq.value()(0, 2), 6.0) || !near(eval3_out.dddq.value()(1, 2), 0.0))
    {
        return fail("unexpected third-order evaluator dddq");
    }

    auto parametric_path = copp::Path::from_parametric(
        [](copp::Jet3 x)
        {
            return std::vector<copp::Jet3>{
                copp::powi(x, 3) + x,
                copp::sin(x),
            };
        },
        0.0,
        1.0);

    const auto parametric_out = parametric_path.evaluate_up_to_3rd(s);
    if (!near(parametric_out.q(0, 2), 2.0) || !near(parametric_out.dq.value()(0, 2), 4.0) ||
        !near(parametric_out.ddq.value()(0, 2), 6.0) || !near(parametric_out.dddq.value()(0, 2), 6.0))
    {
        return fail("unexpected vector parametric polynomial derivatives");
    }
    if (!near(parametric_out.q(1, 2), std::sin(1.0)) ||
        !near(parametric_out.dq.value()(1, 2), std::cos(1.0)) ||
        !near(parametric_out.ddq.value()(1, 2), -std::sin(1.0)) ||
        !near(parametric_out.dddq.value()(1, 2), -std::cos(1.0)))
    {
        return fail("unexpected vector parametric trigonometric derivatives");
    }

    auto writer_parametric_path = copp::Path::from_parametric(
        1,
        0.0,
        1.0,
        [](copp::Jet3 x, copp::Span<copp::Jet3> q)
        {
            q[0] = x * x + 1.0;
        });

    const auto writer_out = writer_parametric_path.evaluate_up_to_2nd(s);
    if (!near(writer_out.q(0, 2), 2.0) || !near(writer_out.dq.value()(0, 2), 2.0) ||
        !near(writer_out.ddq.value()(0, 2), 2.0))
    {
        return fail("unexpected writer parametric derivatives");
    }

    return 0;
}
