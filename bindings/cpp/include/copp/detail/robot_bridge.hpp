#pragma once

#include <cstddef>
#include <mutex>
#include <utility>

#include "rust/cxx.h"
#include "copp/robot.hpp"

namespace copp::bridge
{

// Private owner passed to Rust through `cxx::UniquePtr`.
//
// Rust calls back into this object whenever torque limits or torque objectives
// need inverse dynamics. The mutex keeps user-captured callback state safe when
// Rust evaluates constraints in parallel.
//
// This type is not part of the public C++ API; users provide
// `copp::InverseDynamics`, and `copp::Robot` wraps it in this bridge object.
class CppInverseDynamics final
{
public:
    // `dim` is the expected length of `q`, `dq`, `ddq`, and `tau`.
    CppInverseDynamics(std::size_t dim, copp::InverseDynamics inverse_dynamics);

    std::size_t dim() const noexcept;

    // Evaluate one robot state. The implementation validates all slice lengths
    // before calling the user callback and translates C++ exceptions through cxx.
    void inverse_dynamics(
        rust::Slice<const double> q,
        rust::Slice<const double> dq,
        rust::Slice<const double> ddq,
        rust::Slice<double> tau) const;

private:
    std::size_t dim_ = 0;
    copp::InverseDynamics inverse_dynamics_;
    mutable std::mutex mutex_;
};

} // namespace copp::bridge
