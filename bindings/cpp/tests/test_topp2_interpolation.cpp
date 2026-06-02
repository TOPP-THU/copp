// Test purpose: validate TOPP2 interpolation wrappers.
//
// It covers `a_to_b_topp2`, `s_to_t_topp2`, uniform inverse sampling, and
// explicit inverse sampling on a profile with analytically simple timing.

#include <cmath>
#include <iostream>
#include <vector>

#include <copp/copp.hpp>

namespace
{

    bool near(double lhs, double rhs)
    {
        return std::abs(lhs - rhs) < 1.0e-12;
    }

    int fail(const char *message)
    {
        std::cerr << message << "\n";
        return 1;
    }

} // namespace

int main()
{
    const std::vector<double> s{0.0, 0.5, 1.0};
    const std::vector<double> a{1.0, 1.0, 1.0};

    const auto b = copp::interpolation::a_to_b_topp2(s, a);
    const auto profile = copp::interpolation::s_to_t_topp2(s, a, 0.0);
    const auto s_t_uniform = copp::interpolation::t_to_s_topp2_uniform(s, a, profile.t_s, 0.25);
    const std::vector<double> t_sample{0.0, 0.25, 0.5, 0.75, 1.0};
    const auto s_t_samples = copp::interpolation::t_to_s_topp2_samples(s, a, profile.t_s, t_sample);

    if (b.size() != s.size() - 1)
    {
        return fail("unexpected b length");
    }
    for (double value : b)
    {
        if (!near(value, 0.0))
        {
            return fail("unexpected b value");
        }
    }
    if (!near(profile.t_final, 1.0))
    {
        return fail("unexpected t_final");
    }
    if (profile.t_s.size() != s.size())
    {
        return fail("unexpected t_s length");
    }
    for (std::size_t i = 0; i < s.size(); ++i)
    {
        if (!near(profile.t_s[i], s[i]))
        {
            return fail("unexpected t_s value");
        }
    }
    if (s_t_uniform.size() != 5 || s_t_samples.size() != 5)
    {
        return fail("unexpected s(t) length");
    }
    for (std::size_t i = 0; i < t_sample.size(); ++i)
    {
        if (!near(s_t_uniform[i], t_sample[i]) || !near(s_t_samples[i], t_sample[i]))
        {
            return fail("unexpected s(t) value");
        }
    }

    return 0;
}
