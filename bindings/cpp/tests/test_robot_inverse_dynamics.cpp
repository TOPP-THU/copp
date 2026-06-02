// Test purpose: verify that a C++ inverse-dynamics callback is owned by Robot
// and used by torque-limit / COPP objective paths without exposing callback
// details through the public Rust bridge.

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
} // namespace

int main()
{
    const std::vector<double> s{0.0, 0.5, 1.0};
    const std::vector<double> torque_upper{10.0, 10.0};
    const std::vector<double> torque_lower{-10.0, -10.0};

    auto path = copp::Path::from_parametric(
        [](copp::Jet3 x)
        {
            return std::vector<copp::Jet3>{
                x,
                copp::powi(x, 2),
            };
        },
        0.0,
        1.0);

    std::size_t calls = 0;
    copp::Robot robot(
        2,
        [&calls](
            copp::Span<const double> q,
            copp::Span<const double> dq,
            copp::Span<const double> ddq,
            copp::Span<double> tau)
        {
            ++calls;
            for (std::size_t i = 0; i < tau.size(); ++i)
            {
                tau[i] = 2.0 * ddq[i] + 0.1 * dq[i] + std::sin(q[i]);
            }
        },
        s.size());

    if (!robot.has_inverse_dynamics())
    {
        return fail("expected inverse dynamics to be installed");
    }

    robot.append_s(s)
        .set_q_from_path_2nd(path, 0, s.size())
        .add_torque_limits(torque_upper, torque_lower, 0, s.size());

    if (calls == 0)
    {
        return fail("inverse dynamics was not called");
    }

    robot.clear_inverse_dynamics();
    if (robot.has_inverse_dynamics())
    {
        return fail("expected inverse dynamics to be cleared");
    }

    auto constraints = robot.constraints();
    const auto exported_s = constraints.s_values(0, s.size());
    if (exported_s.size() != s.size() || !near(exported_s.back(), 1.0))
    {
        return fail("unexpected station export");
    }

    return 0;
}
