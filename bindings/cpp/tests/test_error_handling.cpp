// Test purpose: exercise user-facing error paths. The test verifies that Rust
// validation failures and C++ facade checks become `copp::Error` diagnostics
// instead of crashes or silent invalid objects.

#include <iostream>
#include <string>

#include <copp/copp.hpp>

int main()
{
    try
    {
        (void)copp::Path::from_waypoints({{0.0}});
    }
    catch (const copp::Error &error)
    {
        if (error.status() != copp::Status::invalid_input)
        {
            std::cerr << "unexpected error status\n";
            return 1;
        }
        if (std::string(error.what()).empty())
        {
            std::cerr << "empty error message\n";
            return 1;
        }
        return 0;
    }

    std::cerr << "expected copp::Error\n";
    return 1;
}
