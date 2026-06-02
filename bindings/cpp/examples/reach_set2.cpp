// Tutorial example: inspect second-order reachable intervals.
//
// Reach-set APIs are useful for debugging or visualizing feasibility without
// directly asking for the final TOPP2 profile.

#include <iostream>
#include <vector>

#include <copp/copp.hpp>

int main()
{
    // Step 1: Create a raw one-dimensional constraint buffer. A first-order
    // bound is already enough for a tiny reachable-set smoke problem.
    std::vector<double> s{0.0, 0.5, 1.0};
    std::vector<double> amax{1.0, 1.0, 1.0};

    copp::Constraints constraints(1, s.size());
    constraints.append_s(s).add_constraint_1st(amax, 0);

    // Step 2: Build a TOPP2/ReachSet2 problem over the closed station interval.
    copp::solver::reach_set2::Problem problem{
        constraints.ref(),
        copp::IndexInterval{0, s.size() - 1},
        copp::Boundary2{0.0, 0.0},
    };

    // Step 3: Bidirectional reachability enforces both start and final boundary.
    auto reach = copp::solver::reach_set2::bidirectional(problem);

    std::cout << "ReachSet2 done.\n";
    std::cout << "stations = " << reach.len() << "\n";
    std::cout << "a_max(mid) = " << reach.a_max[1] << "\n";

    return 0;
}
