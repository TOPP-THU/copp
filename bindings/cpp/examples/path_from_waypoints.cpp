// Tutorial example: construct a C++ `copp::Path` from waypoint-list notation.
//
// This is the shortest path-construction route for users who already have a
// small list of waypoints. Each inner initializer list is one waypoint vector;
// the facade converts it to the `(dim x n_points)` matrix expected by Rust.

#include <iostream>
#include <vector>

#include <copp/copp.hpp>

int main()
{
    // Step 1: Build a two-dimensional quintic spline over the default range
    // `[0, 1]`. The three columns are q(0), q(0.5), and q(1).
    auto path = copp::Path::from_waypoints({
        {0.0, 0.0},
        {0.5, 0.25},
        {1.0, 1.0},
    });

    // Step 2: Evaluate the path at the waypoint stations. `out.q`, `out.dq`,
    // and `out.ddq` are column-major matrices with shape `(dim, samples.size())`.
    std::vector<double> s{0.0, 0.5, 1.0};
    auto out = path.evaluate_up_to_2nd(s);

    // Step 3: Read matrix entries by `(row, column)`: row is the joint/path
    // dimension, column is the query sample index.
    std::cout << "dim = " << path.dim() << "\n";
    std::cout << "q(0, 1) = " << out.q(0, 1) << "\n";
    std::cout << "dq(0, 1) = " << out.dq.value()(0, 1) << "\n";
    std::cout << "ddq(0, 1) = " << out.ddq.value()(0, 1) << "\n";
}
