// Tutorial example: construct a path from a batch third-order evaluator.
//
// Third-order evaluators are required for jerk constraints and TOPP3/COPP3
// workflows. They can also serve lower-order path requests when a TOPP2 solver
// only needs q/dq/ddq.

#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>

#include <copp/copp.hpp>

int main()
{
    constexpr std::size_t dim = 2;

    // Step 1: Fill q, dq, ddq, and dddq for every sample. The callback owns no
    // buffers; all output storage is borrowed from COPP for this call.
    auto path = copp::Path::from_evaluator_3rd(
        dim,
        0.0,
        1.0,
        [](copp::Span<const double> s,
           copp::MatrixRef q,
           copp::MatrixRef dq,
           copp::MatrixRef ddq,
           copp::MatrixRef dddq) {
            for (std::size_t j = 0; j < s.size(); ++j)
            {
                const double x = s[j];
                q(0, j) = x * x * x;
                dq(0, j) = 3.0 * x * x;
                ddq(0, j) = 6.0 * x;
                dddq(0, j) = 6.0;

                q(1, j) = std::sin(x);
                dq(1, j) = std::cos(x);
                ddq(1, j) = -std::sin(x);
                dddq(1, j) = -std::cos(x);
            }
        });

    // Step 2: Query all derivatives. Each result matrix has shape `(2 x 3)`.
    std::vector<double> samples{0.0, 0.5, 1.0};
    auto out = path.evaluate_up_to_3rd(samples);

    // Step 3: Inspect the cubic coordinate at the final sample.
    std::cout << "dim = " << path.dim() << "\n";
    std::cout << "q(0, 2) = " << out.q(0, 2) << "\n";
    std::cout << "dq(0, 2) = " << out.dq.value()(0, 2) << "\n";
    std::cout << "ddq(0, 2) = " << out.ddq.value()(0, 2) << "\n";
    std::cout << "dddq(0, 2) = " << out.dddq.value()(0, 2) << "\n";
}
