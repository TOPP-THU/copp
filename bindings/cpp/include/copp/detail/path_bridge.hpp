#pragma once

#include <cstddef>
#include <mutex>
#include <utility>

#include "rust/cxx.h"
#include "copp/path.hpp"

namespace copp::bridge
{

// Private owner passed to Rust through `cxx::UniquePtr`.
//
// Rust calls this object through the generated cxx bridge whenever a path built
// from a C++ callback is evaluated. The callback receives all samples at once
// and writes column-major output buffers. A mutex serializes callback execution,
// which makes captured C++ state compatible with Rust's `Send + Sync` path
// trait requirements.
class CppPathEvaluator2nd final
{
public:
    // `dim` is the number of rows in every output matrix.
    CppPathEvaluator2nd(std::size_t dim, copp::PathEvaluator2nd evaluator);

    std::size_t dim() const noexcept;

    // Fill `q`, `dq`, and `ddq`, each with flat shape `(dim, s.size())`.
    void evaluate_up_to_2nd(
        rust::Slice<const double> s,
        rust::Slice<double> q,
        rust::Slice<double> dq,
        rust::Slice<double> ddq) const;

private:
    std::size_t dim_ = 0;
    copp::PathEvaluator2nd evaluator_;
    mutable std::mutex mutex_;
};

// Third-order variant of `CppPathEvaluator2nd`.
//
// It also exposes a second-order entry point because Rust's path abstraction can
// ask a third-order path for lower-order derivatives. The implementation calls
// the same C++ user callback and discards `dddq` when needed.
class CppPathEvaluator3rd final
{
public:
    CppPathEvaluator3rd(std::size_t dim, copp::PathEvaluator3rd evaluator);

    std::size_t dim() const noexcept;

    void evaluate_up_to_2nd(
        rust::Slice<const double> s,
        rust::Slice<double> q,
        rust::Slice<double> dq,
        rust::Slice<double> ddq) const;

    void evaluate_up_to_3rd(
        rust::Slice<const double> s,
        rust::Slice<double> q,
        rust::Slice<double> dq,
        rust::Slice<double> ddq,
        rust::Slice<double> dddq) const;

private:
    std::size_t dim_ = 0;
    copp::PathEvaluator3rd evaluator_;
    mutable std::mutex mutex_;
};

} // namespace copp::bridge
