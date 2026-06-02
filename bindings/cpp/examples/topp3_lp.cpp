// Tutorial example: solve a minimal third-order TOPP3-LP problem.
//
// TOPP3 introduces the node profile `a = dot{s}^2` and `b = ddot{s}`. A
// `topp3::Problem` immediately linearizes third-order constraints at the
// supplied `a_linearization` before the LP/SOCP solver is called.

#include <iostream>
#include <vector>

#include <copp/copp.hpp>

int main()
{
    // Step 1: Build a raw one-dimensional station/first-order constraint setup.
    std::vector<double> s{0.0, 0.5, 1.0};
    std::vector<double> amax{1.0, 1.0, 1.0};
    std::vector<double> a_linearization{0.25, 0.25, 0.25};

    copp::Constraints constraints(1, s.size());
    constraints.append_s(s)
        .add_constraint_1st(amax, 0);

    // Step 2: Choose boundary values and an initial linearization profile.
    // Here the boundary speed is nonzero, so no stationary interval is inferred.
    const copp::Boundary3 boundary{0.25, 0.25, 0.0, 0.0};
    copp::solver::topp3::Problem problem{
        constraints.ref(),
        a_linearization,
        0,
        boundary,
    };

    // Step 3: Solve the LP formulation and convert the profile to time.
    copp::clarabel::Options options;
    options.allow_almost_solved = true;

    auto profile = copp::solver::topp3_lp::solve(problem, options);
    auto time = copp::interpolation::s_to_t_topp3(s, profile, 0.0);

    std::cout << "TOPP3-LP done.\n";
    std::cout << "profile length = " << profile.len() << "\n";
    std::cout << "stationary start/end = "
              << profile.num_stationary_start << " / "
              << profile.num_stationary_end << "\n";
    std::cout << "t_final = " << time.t_final << "\n";

    return 0;
}
