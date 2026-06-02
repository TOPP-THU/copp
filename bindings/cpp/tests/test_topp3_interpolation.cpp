// Test purpose: validate TOPP3 interpolation wrappers and profile slicing.
//
// It checks owned `Profile3rd`, borrowed `Profile3rdRef`, segment/slice views,
// stationary-boundary counts, and both uniform and explicit `s(t)` sampling.

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
    const copp::Profile3rd profile{
        {1.0, 1.0, 1.0},
        {0.0, 0.0, 0.0},
    };

    const auto time = copp::interpolation::s_to_t_topp3(s, profile, 0.0);
    const auto s_t_uniform = copp::interpolation::t_to_s_topp3_uniform(s, profile, time.t_s, 0.25);
    const std::vector<double> t_sample{0.0, 0.25, 0.5, 0.75, 1.0};
    const auto s_t_samples = copp::interpolation::t_to_s_topp3_samples(s, profile, time.t_s, t_sample);

    if (profile.len() != s.size() || profile.num_stationary() != std::make_pair<std::size_t, std::size_t>(0, 0))
    {
        return fail("unexpected profile shape");
    }
    if (!near(time.t_final, 1.0))
    {
        return fail("unexpected t_final");
    }
    for (std::size_t i = 0; i < s.size(); ++i)
    {
        if (!near(time.t_s[i], s[i]))
        {
            return fail("unexpected t_s value");
        }
    }
    for (std::size_t i = 0; i < t_sample.size(); ++i)
    {
        if (!near(s_t_uniform[i], t_sample[i]) || !near(s_t_samples[i], t_sample[i]))
        {
            return fail("unexpected s(t) value");
        }
    }

    const auto part = profile.slice(1, 2);
    const auto part_time = copp::interpolation::s_to_t_topp3(copp::Span<const double>(s.data() + 1, 2), part);
    if (!near(part_time.t_final, 0.5))
    {
        return fail("unexpected sliced t_final");
    }

    return 0;
}
