// Test purpose: verify the no-throw overload convention. Passing
// `copp::no_throw` as the final argument should return `Expected<T>` with
// either a value or a copied `ErrorInfo` payload.

#include <iostream>
#include <vector>

#include <copp/copp.hpp>

int main()
{
    auto path = copp::Path::from_waypoints({{0.0}}, copp::no_throw);
    if (path)
    {
        std::cerr << "expected invalid waypoint path to fail\n";
        return 1;
    }
    if (path.error().status != copp::Status::invalid_input || path.error().message.empty())
    {
        std::cerr << "unexpected no_throw error payload\n";
        return 1;
    }

    const std::vector<double> s{0.0, 0.5, 1.0};
    const std::vector<double> a{1.0, 1.0, 1.0};
    auto time = copp::interpolation::s_to_t_topp2(s, a, copp::no_throw);
    if (!time || time.value().t_s.size() != s.size())
    {
        std::cerr << "unexpected no_throw interpolation result\n";
        return 1;
    }

    return 0;
}
