// Tutorial example: construct a path from a scalar parametric formula.
//
// `copp::Jet3` carries value, first, second, and third derivatives with respect
// to the path parameter `s`, so ordinary-looking math can produce all geometric
// derivatives needed by TOPP2/TOPP3.

#include <cmath>
#include <iostream>
#include <vector>

#include <copp/copp.hpp>

int main()
{
    // Step 1: Return one `Jet3` per path dimension. COPP probes this callback
    // to infer `dim`, then evaluates it with derivative seed `ds/ds = 1`.
    auto path = copp::Path::from_parametric(
        [](copp::Jet3 s) {
            return std::vector<copp::Jet3>{
                copp::sin(s),
                copp::cos(2.0 * s),
            };
        },
        0.0,
        1.0);

    // Step 2: Evaluate all derivatives up to third order at three samples.
    std::vector<double> samples{0.0, 0.5, 1.0};
    auto out = path.evaluate_up_to_3rd(samples);

    // Step 3: Optional derivative matrices are populated according to the
    // evaluation method. Here `dq`, `ddq`, and `dddq` are all present.
    std::cout << "dim = " << path.dim() << "\n";
    std::cout << "q(0, 1) = " << out.q(0, 1) << "\n";
    std::cout << "dq(0, 1) = " << out.dq.value()(0, 1) << "\n";
    std::cout << "ddq(0, 1) = " << out.ddq.value()(0, 1) << "\n";
    std::cout << "dddq(0, 1) = " << out.dddq.value()(0, 1) << "\n";
}
