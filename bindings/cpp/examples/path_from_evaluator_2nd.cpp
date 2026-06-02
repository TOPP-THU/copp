// Tutorial example: construct a path from a batch second-order evaluator.
//
// Use this form when derivatives come from an external model, table, or custom
// code rather than from `Jet3` automatic differentiation. The callback receives
// all query samples in one batch and fills column-major output buffers.

#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>

#include <copp/copp.hpp>

int main()
{
    constexpr std::size_t dim = 2;

    // Step 1: Declare the path range and a callback that fills q, dq, and ddq.
    // Output matrix shape is `(dim x s.size())`.
    auto path = copp::Path::from_evaluator_2nd(
        dim,
        0.0,
        1.0,
        [](copp::Span<const double> s,
           copp::MatrixRef q,
           copp::MatrixRef dq,
           copp::MatrixRef ddq) {
            for (std::size_t j = 0; j < s.size(); ++j)
            {
                const double x = s[j];
                q(0, j) = x;
                dq(0, j) = 1.0;
                ddq(0, j) = 0.0;

                q(1, j) = std::sin(x);
                dq(1, j) = std::cos(x);
                ddq(1, j) = -std::sin(x);
            }
        });

    // Step 2: Evaluate with a batch of samples. The bridge crosses into C++
    // once for the full batch instead of once per sample.
    std::vector<double> samples{0.0, 0.5, 1.0};
    auto out = path.evaluate_up_to_2nd(samples);

    // Step 3: Read the sine dimension at the middle sample.
    std::cout << "dim = " << path.dim() << "\n";
    std::cout << "q(1, 1) = " << out.q(1, 1) << "\n";
    std::cout << "dq(1, 1) = " << out.dq.value()(1, 1) << "\n";
    std::cout << "ddq(1, 1) = " << out.ddq.value()(1, 1) << "\n";
}
