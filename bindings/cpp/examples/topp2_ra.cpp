// Tutorial example: solve a second-order TOPP2-RA problem.
//
// This demonstrates the common workflow: construct a path, sample it on a
// station grid, add velocity/acceleration limits, solve for `a(s) = dot{s}^2`,
// then convert the station profile into a time trajectory.

#include <cmath>
#include <iostream>
#include <vector>

#include <copp/copp.hpp>

int main()
{
    constexpr std::size_t dim = 3;
    constexpr std::size_t n = 1001;

    // Step 1: Provide a smooth path with analytic derivatives up to second
    // order. TOPP2 does not need third derivatives.
    auto path = copp::Path::from_evaluator_2nd(
        dim,
        0.0,
        1.0,
        [](copp::Span<const double> s,
           copp::MatrixRef q,
           copp::MatrixRef dq,
           copp::MatrixRef ddq)
        {
            const double pi_local = 3.14159265358979323846;
            const double freq[3] = {2.0 * pi_local, 3.0 * pi_local, 5.0 * pi_local};
            const double phase[3] = {0.0, 0.3, 0.7};

            for (std::size_t j = 0; j < s.size(); ++j)
            {
                for (std::size_t axis = 0; axis < 3; ++axis)
                {
                    const double x = freq[axis] * s[j] + phase[axis];
                    q(axis, j) = std::sin(x);
                    dq(axis, j) = freq[axis] * std::cos(x);
                    ddq(axis, j) = -freq[axis] * freq[axis] * std::sin(x);
                }
            }
        });

    // Step 2: Build a uniform station grid on the path parameter interval.
    std::vector<double> s(n);
    for (std::size_t k = 0; k < n; ++k)
    {
        s[k] = static_cast<double>(k) / static_cast<double>(n - 1);
    }

    const std::vector<double> upper(dim, 1.0);
    const std::vector<double> lower(dim, -1.0);

    // Step 3: Sample path derivatives into the robot constraint buffer and add
    // physical velocity/acceleration limits.
    copp::Robot robot(dim, n);
    robot.append_s(s)
        .set_q_from_path_2nd(path, 0, n)
        .add_velocity_limits(upper, lower, 0, n)
        .add_acceleration_limits(upper, lower, 0, n);

    namespace topp2 = copp::solver::topp2_ra;

    // Step 4: The problem borrows `robot.constraints()`. The robot must outlive
    // the problem and solver call.
    topp2::Problem problem{
        robot.constraints(),
        copp::IndexInterval{0, n - 1},
        copp::Boundary2{0.0, 0.0},
    };

    // Step 5: Solve for the node profile `a`, integrate time, and create a
    // uniform `s(t)` sampling.
    auto a = topp2::solve(problem);
    auto time = copp::interpolation::s_to_t_topp2(s, a, 0.0);
    auto s_t = copp::interpolation::t_to_s_topp2_uniform(s, a, time.t_s, 1.0e-3);

    std::cout << "TOPP2-RA done.\n";
    std::cout << "dim = " << dim << ", N = " << n << "\n";
    std::cout << "t_final = " << time.t_final << " s\n";
    std::cout << "a_profile.len() = " << a.size() << "\n";
    std::cout << "s(t) samples = " << s_t.size() << "\n";

    return 0;
}
