// Test purpose: verify that the C++ facade can call into Rust and read the
// crate version string. This is the smallest end-to-end bridge smoke test.

#include <iostream>

#include <copp/copp.hpp>

int main()
{
    if (copp::version().empty())
    {
        std::cerr << "empty C++ facade version\n";
        return 1;
    }

    return 0;
}
