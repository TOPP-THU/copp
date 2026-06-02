// Tutorial example: solve a minimal COPP3-SOCP problem.
//
// COPP3 combines third-order constraints with COPP objectives. This example
// uses a single time objective and expert output so objective diagnostics can be
// inspected when Clarabel accepts the profile.

#include <iostream>
#include <vector>

#include <copp/copp.hpp>

int main()
{
    // Step 1: Minimal raw constraints: station grid plus first-order bound.
    std::vector<double> s{0.0, 0.5, 1.0};
    std::vector<double> amax{1.0, 1.0, 1.0};
    std::vector<double> a_linearization{0.25, 0.25, 0.25};

    copp::Robot robot(1, s.size());
    robot.append_s(s);
    robot.constraints().add_constraint_1st(amax, 0);

    const copp::Boundary3 boundary{0.25, 0.25, 0.0, 0.0};

    namespace copp3 = copp::solver::copp3_socp;

    // Step 2: Build a COPP3 problem. The constructor borrows `robot`, copies
    // objectives and `a_linearization`, then linearizes third-order constraints.
    copp3::Problem problem{
        robot,
        {copp::objective::Time(1.0)},
        a_linearization,
        0,
        boundary,
    };

    // Step 3: Allow AlmostSolved for this tiny example and request diagnostics.
    copp::clarabel::Options options;
    options.allow_almost_solved = true;

    auto expert = copp3::solve_expert(problem, options);
    if (!expert.profile)
    {
        std::cerr << "Clarabel did not return an accepted profile\n";
        return 1;
    }

    // Step 4: Integrate the accepted third-order profile.
    auto time = copp::interpolation::s_to_t_topp3(s, *expert.profile, 0.0);

    std::cout << "COPP3-SOCP done.\n";
    std::cout << "profile length = " << expert.profile->len() << "\n";
    std::cout << "t_final = " << time.t_final << "\n";
    if (expert.objective_value)
    {
        std::cout << "objective = " << *expert.objective_value << "\n";
    }

    return 0;
}
