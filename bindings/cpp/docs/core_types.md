\page cpp_core_types Core Types, Ownership, And Errors

This page describes the low-level C++ types that all higher-level modules use.
They are intentionally small: the C++ facade owns C++ storage at the boundary,
borrows user buffers only for the duration of a call, and hides the private
Rust/C++ bridge behind `.cpp` files.

## Include

```cpp
#include <copp/core.hpp>
```

Most applications can include only:

```cpp
#include <copp/copp.hpp>
```

## `Span<T>`

`copp::Span<T>` is a C++17-compatible subset of `std::span`.  It is a borrowed
view of contiguous memory and does not allocate, copy, or own the data.

```cpp
std::vector<double> s{0.0, 0.5, 1.0};
copp::Span<const double> from_vector{s};
copp::Span<const double> from_pointer{s.data(), s.size()};

double raw[] = {1.0, 2.0, 3.0};
copp::Span<const double> from_array{raw};
```

Input spans must remain alive only for the call that consumes them, unless a
type explicitly documents longer borrowing.

## Matrices

COPP's internal Rust storage is column-major, and the C++ `copp::Matrix` follows
the same rule:

```cpp
copp::Matrix m(2, 3);
m(0, 0) = 0.0;
m(1, 0) = 1.0;
double first_column_second_row = m(1, 0);
```

The physical index is:

@f[
\text{offset}(i, j) = i + j \cdot \text{rows}.
@f]

`copp::MatrixView` borrows input matrices.  Column-major data can usually cross
the bridge without reshaping; row-major data is accepted and copied once into
column-major storage when Rust needs contiguous columns.

```cpp
std::vector<double> row_major{
    0.0, 0.0,
    0.5, 0.25,
    1.0, 1.0,
};

auto view = copp::MatrixView::row_major(row_major.data(), 3, 2);
auto path = copp::Path::from_waypoints(view);
```

`copp::MatrixRef` is a writable borrowed matrix used by C++ callbacks:

```cpp
auto evaluator = [](copp::Span<const double> s,
                    copp::MatrixRef q,
                    copp::MatrixRef dq,
                    copp::MatrixRef ddq) {
    for (std::size_t j = 0; j < s.size(); ++j) {
        q(0, j) = s[j];
        dq(0, j) = 1.0;
        ddq(0, j) = 0.0;
    }
};
```

## Initializer Helpers

`Matrix::from_rows` is convenient for ordinary tabular data:

```cpp
auto table = copp::Matrix::from_rows({
    {0.0, 0.5, 1.0},
    {0.0, 0.25, 1.0},
});
```

`Matrix::from_columns` is convenient for waypoints because each waypoint is one
column in COPP:

```cpp
auto waypoints = copp::Matrix::from_columns({
    {0.0, 0.0},
    {0.5, 0.25},
    {1.0, 1.0},
});
```

## Eigen Adapter

When Eigen support is enabled, `copp/eigen.hpp` provides adapters instead of
changing the core return type.  Solver outputs still return `copp::Matrix`, and
Eigen users can map or view the storage explicitly.

```cpp
#include <copp/eigen.hpp>

Eigen::MatrixXd q = Eigen::MatrixXd::Zero(2, 3);
auto view = copp::eigen::view(q);
```

This keeps the public ABI stable whether Eigen is present or disabled.

## Errors And No-Throw Calls

The primary C++ API throws `copp::Error`:

{% raw %}
```cpp
try {
    auto path = copp::Path::from_waypoints({{0.0}, {1.0}});
} catch (const copp::Error& error) {
    std::cerr << error.what() << "\n";
}
```
{% endraw %}

Selected APIs also provide a no-throw overload with `copp::no_throw` as the
final argument:

{% raw %}
```cpp
auto result = copp::Path::from_waypoints({{0.0}, {1.0}}, copp::no_throw);
if (!result) {
    std::cerr << result.error().message << "\n";
    return;
}

auto path = std::move(result).value();
```
{% endraw %}

The no-throw form uses `copp::Expected<T>`, a small C++17 expected-like type.

## Ownership Summary

- `Path`, `Robot`, `Constraints`, solver `Problem` objects, profiles, matrices,
  and result structs use RAII.
- `ConstraintsRef`, `Span`, `MatrixView`, `MatrixRef`, and `Profile3rdRef` are
  borrowed views.
- `Robot::constraints()` and non-const `Constraints::ref()` return borrowed
  views.  The owning object must outlive the solver call.
- Returned vectors and matrices own their memory.
- The C++ facade calls Rust through a private `cxx` bridge, not through the
  public C ABI.
