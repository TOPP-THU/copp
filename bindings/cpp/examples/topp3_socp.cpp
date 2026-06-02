// Tutorial example: solve TOPP3-SOCP with two SCP refinement rounds.
//
// The SOCP formulation depends on a linearization profile. A common workflow is
// to solve once with a simple initial `a0`, then rebuild the problem with the
// recovered `profile.a` and solve again.

#include <iostream>
#include <vector>

#include <copp/copp.hpp>

int main()
{
    // Step 1: Minimal one-dimensional constraint setup.
    std::vector<double> s{0.0, 0.5, 1.0};
    std::vector<double> amax{1.0, 1.0, 1.0};
    std::vector<double> a0{0.25, 0.25, 0.25};

    copp::Constraints constraints(1, s.size());
    constraints.append_s(s)
        .add_constraint_1st(amax, 0);

    // Step 2: Configure Clarabel and endpoint boundary values.
    const copp::Boundary3 boundary{0.25, 0.25, 0.0, 0.0};

    copp::clarabel::Options options;
    options.allow_almost_solved = true;

    // Step 3: First SCP round using the hand-chosen linearization profile.
    copp::solver::topp3::Problem first_problem{
        constraints.ref(),
        a0,
        0,
        boundary,
    };
    auto profile1 = copp::solver::topp3_socp::solve(first_problem, options);

    // Step 4: Second SCP round using the previous `a` profile as the new
    // linearization. More rounds can be added by repeating this pattern.
    copp::solver::topp3::Problem second_problem{
        constraints.ref(),
        profile1.a,
        0,
        boundary,
    };
    auto profile2 = copp::solver::topp3_socp::solve(second_problem, options);

    // Step 5: Interpolate the final third-order profile in time.
    auto time = copp::interpolation::s_to_t_topp3(s, profile2, 0.0);

    std::cout << "TOPP3-SOCP done with 2 refinement rounds.\n";
    std::cout << "profile length = " << profile2.len() << "\n";
    std::cout << "t_final = " << time.t_final << "\n";

    return 0;
}
