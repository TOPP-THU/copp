// Test purpose: verify TOPP3 problem preparation. Construction should copy the
// linearization profile, infer the final station, and report stationary counts
// after Rust-side immediate linearization.

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
    const std::vector<double> a_linearization{0.25, 0.25, 0.25};

    copp::Constraints constraints(1, s.size());
    constraints.append_s(s)
        .add_constraint_1st(amax, 0);

    copp::Boundary3 boundary{0.25, 0.25, 0.0, 0.0};
    copp::solver::topp3::Problem problem(
        constraints.ref(),
        a_linearization,
        0,
        boundary);

    if (problem.s_len() != s.size() || problem.idx_s_final() != 2)
    {
        return fail("unexpected TOPP3 problem interval");
    }
    if (problem.num_stationary().start != 0 || problem.num_stationary().end != 0)
    {
        return fail("unexpected TOPP3 effective stationary counts");
    }

    return 0;
}
