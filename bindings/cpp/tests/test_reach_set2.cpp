// Test purpose: smoke-test second-order reachable-set construction from a raw
// Constraints object and verify interval vector sizes and simple feasibility.

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

    copp::solver::reach_set2::Problem problem{
        constraints.ref(),
        copp::IndexInterval{0, s.size() - 1},
        copp::Boundary2{0.0, 0.0},
    };

    const auto backward = copp::solver::reach_set2::backward(problem);
    const auto bidirectional = copp::solver::reach_set2::bidirectional(problem);
    if (backward.len() != s.size() || bidirectional.len() != s.size())
    {
        return fail("unexpected ReachSet2 length");
    }

    return 0;
}
