// Tutorial example: solve a second-order COPP2-SOCP problem with Clarabel.
//
// COPP extends TOPP by optimizing an objective list. This example uses the
// default point-mass torque model (`tau = ddq`) and asks for expert diagnostics
// so we can print the accepted objective value.

#include <iostream>
#include <vector>

#include <copp/copp.hpp>

int main()
{
    // Step 1: Define a small station grid and symmetric joint limits.
    std::vector<double> s{0.0, 0.25, 0.5, 0.75, 1.0};
    std::vector<double> upper{10.0, 10.0};
    std::vector<double> lower{-10.0, -10.0};

    // Step 2: Build a parametric path. `powi` propagates derivatives through
    // the Jet3 scalar, so q/dq/ddq are available for constraint construction.
    auto path = copp::Path::from_parametric(
        [](copp::Jet3 x) {
            return std::vector<copp::Jet3>{
                x,
                copp::powi(x, 2),
            };
        },
        0.0,
        1.0);

    // Step 3: Populate the robot with path derivatives and physical limits.
    // Without a custom inverse-dynamics callback, torque limits use tau = ddq.
    copp::Robot robot(2, s.size());
    robot.append_s(s)
         .set_q_from_path_2nd(path, 0, s.size())
         .add_velocity_limits(upper, lower, 0, s.size())
         .add_acceleration_limits(upper, lower, 0, s.size())
         .add_torque_limits(upper, lower, 0, s.size());

    namespace copp2 = copp::solver::copp2_socp;

    // Step 4: Objective descriptors are copied into the problem. The robot is
    // borrowed, so it must remain alive through the solve.
    copp2::Problem problem{
        robot,
        {
            copp::objective::Time(1.0),
            copp::objective::ThermalEnergy(0.05, {1.0, 1.0}),
        },
        copp::IndexInterval{0, s.size() - 1},
        copp::Boundary2{0.0, 0.0},
    };

    // Step 5: Adjust Clarabel acceptance/settings and request expert output.
    copp::clarabel::Options options;
    options.allow_almost_solved = true;
    options.clarabel_settings.max_iter = 200;

    auto expert = copp2::solve_expert(problem, options);
    if (!expert.a)
    {
        std::cerr << "Clarabel did not return an accepted profile\n";
        return 1;
    }

    // Step 6: Convert the accepted profile into a time trajectory.
    auto time = copp::interpolation::s_to_t_topp2(s, *expert.a, 0.0);
    std::cout << "accepted a length = " << expert.a->size() << "\n";
    std::cout << "t_final = " << time.t_final << "\n";
    std::cout << "objective = " << *expert.objective_value << "\n";
    return 0;
}
