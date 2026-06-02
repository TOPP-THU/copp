\page cpp_interpolation Interpolation And Time Profiles

Interpolation helpers convert path-domain profiles into time-domain schedules.
Solvers usually return profiles over station samples `s[k]`; controllers often
need cumulative time `t(s)` or samples of the inverse map `s(t)`.

## Include

```cpp
#include <copp/interpolation.hpp>
```

or:

```cpp
#include <copp/copp.hpp>
```

## TOPP2 Profile

TOPP2/COPP2 solvers return a node profile:

@f[
a_k = a(s_k) = \dot{s}(s_k)^2.
@f]

The segment profile is:

@f[
b_k = \frac{a_{k+1} - a_k}{2(s_{k+1} - s_k)}.
@f]

```cpp
std::vector<double> s{0.0, 0.5, 1.0};
std::vector<double> a{1.0, 1.0, 1.0};

auto b = copp::interpolation::a_to_b_topp2(s, a);
auto time = copp::interpolation::s_to_t_topp2(s, a);

auto s_uniform = copp::interpolation::t_to_s_topp2_uniform(
    s,
    a,
    time.t_s,
    0.1);

std::vector<double> t_query{0.0, 0.25, 0.5, 1.0};
auto s_query = copp::interpolation::t_to_s_topp2_samples(
    s,
    a,
    time.t_s,
    t_query);
```

Input requirements:

- `s.size() >= 2`
- `a.size() == s.size()`
- `s` is strictly increasing
- all values are finite
- every segment has a positive finite speed denominator

## TOPP3 Profile

TOPP3/COPP3 use node-based `a` and `b` arrays:

@f[
a_k = \dot{s}(s_k)^2, \qquad b_k = \ddot{s}(s_k).
@f]

`copp::Profile3rd` owns those arrays:

```cpp
std::vector<double> s{0.0, 0.5, 1.0};
copp::Profile3rd profile{
    {1.0, 1.0, 1.0},
    {0.0, 0.0, 0.0},
};

auto time = copp::interpolation::s_to_t_topp3(s, profile);
auto s_uniform = copp::interpolation::t_to_s_topp3_uniform(
    s,
    profile,
    time.t_s,
    0.1);
```

`copp::Profile3rdRef` is the zero-copy borrowed view.  It is useful for
external storage and for interpolating only part of a profile:

```cpp
auto part = profile.slice(10, 40); // inclusive station indices
auto local_s = copp::Span<const double>(s.data() + 10, 31);
auto local_time = copp::interpolation::s_to_t_topp3(local_s, part);
```

Stationary counts are clipped to the local slice.  A slice must still contain at
least one non-stationary segment after that adjustment.

## Uniform Versus Explicit Time Grids

Uniform sampling is convenient for controllers:

```cpp
copp::interpolation::TimeGridOptions grid;
grid.t0 = 0.0;
grid.include_final = true;

auto s_t = copp::interpolation::t_to_s_topp2_uniform(
    s,
    a,
    time.t_s,
    0.001,
    grid);
```

Explicit samples are better for diagnostics, plotting, or synchronizing with an
external time base:

```cpp
std::vector<double> t_sample{0.0, 0.02, 0.05, time.t_final};
auto s_t = copp::interpolation::t_to_s_topp3_samples(
    s,
    profile,
    time.t_s,
    t_sample);
```

Out-of-range query times are returned as `NaN`, matching Rust/Python.

## Tutorial Sources

- `bindings/cpp/tests/test_topp2_interpolation.cpp`
- `bindings/cpp/tests/test_topp3_interpolation.cpp`
