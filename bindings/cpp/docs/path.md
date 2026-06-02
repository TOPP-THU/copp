\page cpp_path Path Construction And Evaluation

`copp::Path` represents the geometric path `q = q(s)`.  Solvers do not optimize
geometry; they optimize how quickly this already-defined path is traversed.

## Include

```cpp
#include <copp/path.hpp>
```

or:

```cpp
#include <copp/copp.hpp>
```

## Output Shapes

Every batch evaluation uses a station vector `s` of length `N` and returns
column-major matrices with shape `(dim x N)`:

```cpp
std::vector<double> samples{0.0, 0.5, 1.0};
auto out = path.evaluate_up_to_2nd(samples);

double q_axis0_at_mid = out.q(0, 1);
double dq_axis0_at_mid = out.dq.value()(0, 1);
double ddq_axis0_at_mid = out.ddq.value()(0, 1);
```

## Waypoint Splines

Waypoint paths are the easiest entry point.  Each inner initializer-list is one
waypoint vector:

```cpp
auto path = copp::Path::from_waypoints({
    {0.0, 0.0},
    {0.5, 0.25},
    {1.0, 1.0},
});

auto range = path.s_range(); // {0.0, 1.0}
auto dim = path.dim();       // 2
```

If you already have a matrix, use the native COPP shape `(dim x n_points)`:

```cpp
auto waypoints = copp::Matrix::from_columns({
    {0.0, 0.0},
    {0.5, 0.25},
    {1.0, 1.0},
});

auto path = copp::Path::from_waypoints(waypoints.view());
```

The default spline configuration follows Rust/Python:

- order `5`
- range `[0, 1]`
- out-of-range behavior: error
- zero boundary derivative states

Custom range and clamping:

```cpp
copp::SplineConfig config;
config.s_min = -1.0;
config.s_max = 1.0;
config.out_of_range = copp::OutOfRangeMode::Clamp;

auto path = copp::Path::from_waypoints(waypoints.view(), config);
```

## Scalar Parametric Paths

`Path::from_parametric` uses `copp::Jet3` automatic differentiation.  The
callback receives a seeded scalar `s`; ordinary arithmetic and COPP math helpers
propagate derivatives up to third order.

```cpp
auto path = copp::Path::from_parametric(
    [](copp::Jet3 s) {
        return std::vector<copp::Jet3>{
            copp::sin(s),
            copp::cos(2.0 * s),
        };
    },
    0.0,
    1.0);

std::vector<double> samples{0.0, 0.5, 1.0};
auto out = path.evaluate_up_to_3rd(samples);
```

For tighter loops, avoid allocating a vector inside every callback invocation by
using the writer overload:

```cpp
auto path = copp::Path::from_parametric(
    2,
    0.0,
    1.0,
    [](copp::Jet3 s, copp::Span<copp::Jet3> q) {
        q[0] = copp::powi(s, 3);
        q[1] = copp::sin(s);
    });
```

Supported helpers include `sin`, `cos`, `exp`, `log`/`ln`, `sqrt`, and `powi`.

## Batch Evaluator Paths

Use evaluator paths when the path comes from an external model or when you
already know analytic derivatives.  The callback is batch-oriented to avoid
crossing the C++/Rust bridge once per station.

```cpp
auto path = copp::Path::from_evaluator_2nd(
    2,
    0.0,
    1.0,
    [](copp::Span<const double> s,
       copp::MatrixRef q,
       copp::MatrixRef dq,
       copp::MatrixRef ddq) {
        for (std::size_t j = 0; j < s.size(); ++j) {
            const double x = s[j];

            q(0, j) = x;
            dq(0, j) = 1.0;
            ddq(0, j) = 0.0;

            q(1, j) = x * x;
            dq(1, j) = 2.0 * x;
            ddq(1, j) = 2.0;
        }
    });
```

Third-order evaluators add `dddq`:

```cpp
auto path = copp::Path::from_evaluator_3rd(
    1,
    0.0,
    1.0,
    [](copp::Span<const double> s,
       copp::MatrixRef q,
       copp::MatrixRef dq,
       copp::MatrixRef ddq,
       copp::MatrixRef dddq) {
        for (std::size_t j = 0; j < s.size(); ++j) {
            const double x = s[j];
            q(0, j) = x * x * x;
            dq(0, j) = 3.0 * x * x;
            ddq(0, j) = 6.0 * x;
            dddq(0, j) = 6.0;
        }
    });
```

The path owns the callback object.  Throw `copp::Error` or another
`std::exception` from the callback to report evaluator failure.

## Tutorial Sources

- `bindings/cpp/examples/path_from_waypoints.cpp`
- `bindings/cpp/examples/path_from_parametric.cpp`
- `bindings/cpp/examples/path_from_evaluator_2nd.cpp`
- `bindings/cpp/examples/path_from_evaluator_3rd.cpp`
