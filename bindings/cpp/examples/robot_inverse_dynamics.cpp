// Tutorial example: install a C++ inverse-dynamics callback.
//
// The callback is used by torque limits and torque objectives. This example
// keeps the dynamics intentionally small so the data flow is visible:
// tau = inertia * ddq + damping * dq + gravity * sin(q).

#include <cmath>
#include <iostream>
#include <vector>

#include <copp/copp.hpp>

struct ToyArmDynamics
{
    std::vector<double> inertia{1.2, 0.8};
    std::vector<double> damping{0.08, 0.05};
    std::vector<double> gravity{0.4, 0.25};

    void operator()(
        copp::Span<const double> q,
        copp::Span<const double> dq,
        copp::Span<const double> ddq,
        copp::Span<double> tau) const
    {
        // The callback receives one state at a time. All spans have length equal
        // to the robot dimension, and `tau` must be fully overwritten.
        for (std::size_t i = 0; i < tau.size(); ++i)
        {
            tau[i] = inertia[i] * ddq[i] + damping[i] * dq[i] + gravity[i] * std::sin(q[i]);
        }
    }
};

int main()
{
    // Step 1: Create stations and joint/torque limits. Torque limits will use
    // the callback below instead of the default point-mass model.
    std::vector<double> s{0.0, 0.25, 0.5, 0.75, 1.0};
    std::vector<double> upper{3.0, 3.0};
    std::vector<double> lower{-3.0, -3.0};
    std::vector<double> torque_upper{50.0, 50.0};
    std::vector<double> torque_lower{-50.0, -50.0};

    // Step 2: Build a path with derivatives available through Jet3.
    auto path = copp::Path::from_parametric(
        [](copp::Jet3 x)
        {
            return std::vector<copp::Jet3>{
                x,
                copp::sin(x),
            };
        },
        0.0,
        1.0);

    ToyArmDynamics dynamics;

    // Step 3: Install the callback. Captured objects must remain valid for the
    // period in which torque constraints/objectives are constructed.
    copp::Robot robot(2, s.size());
    robot.set_inverse_dynamics(
        [dynamics](
            copp::Span<const double> q,
            copp::Span<const double> dq,
            copp::Span<const double> ddq,
            copp::Span<double> tau)
        {
            dynamics(q, dq, ddq, tau);
        });

    // Step 4: Add velocity, acceleration, and torque constraints. The torque
    // call crosses the Rust/C++ bridge into `ToyArmDynamics`.
    robot.append_s(s)
        .set_q_from_path_2nd(path, 0, s.size())
        .add_velocity_limits(upper, lower, 0, s.size())
        .add_acceleration_limits(upper, lower, 0, s.size())
        .add_torque_limits(torque_upper, torque_lower, 0, s.size());

    namespace copp2 = copp::solver::copp2_socp;

    // Step 5: ThermalEnergy uses inverse dynamics as part of the objective.
    copp2::Problem problem{
        robot,
        {
            copp::objective::Time(1.0),
            copp::objective::ThermalEnergy(0.05, {1.0, 1.0}),
        },
        copp::IndexInterval{0, s.size() - 1},
        copp::Boundary2{0.0, 0.0},
    };

    // Step 6: Expert output exposes accepted profile and objective diagnostics.
    auto expert = copp2::solve_expert(problem);
    if (!expert.a)
    {
        std::cerr << "Clarabel did not return an accepted profile\n";
        return 1;
    }

    auto time = copp::interpolation::s_to_t_topp2(s, *expert.a, 0.0);
    std::cout << "has_inverse_dynamics = " << robot.has_inverse_dynamics() << "\n";
    std::cout << "accepted a length = " << expert.a->size() << "\n";
    std::cout << "t_final = " << time.t_final << "\n";
    std::cout << "objective = " << *expert.objective_value << "\n";

    return 0;
}
