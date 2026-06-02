#include "copp/core.hpp"

// Tiny core facade functions that still need the generated bridge header.
// Most core types are header-only value/view utilities; `version()` crosses
// into Rust so the C++ facade reports the same crate version as other bindings.

#include "rust/cxx.h"
#include "copp/src/ffi/cpp/mod.rs.h"

namespace copp
{

    std::string version()
    {
        const auto value = ::copp::bridge::version();
        return std::string(value.data(), value.size());
    }

} // namespace copp
