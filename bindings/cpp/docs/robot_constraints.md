\page cpp_robot_constraints Robot And Constraints

`copp::Robot` is the high-level way to turn path samples and physical limits
into solver constraints.  `copp::Constraints` is the lower-level mathematical
container for users who already know the TOPP/COPP constraint rows they want.

## Include

```cpp
#include <copp/robot.hpp>
```

or:

```cpp
#include <copp/copp.hpp>
```

## Constraint Variables

The second-order state is:

@f[
a(s) = \dot{s}^2, \qquad b(s) = \ddot{s}.
@f]

The third-order control convention is:

@f[
c(s) = \frac{s^{(3)}}{\dot{s}},
\qquad s^{(3)} = \frac{d^3s}{dt^3}.
@f]

Raw constraints follow these sampled forms:

@f[
0 \leq a_k \leq a_{\max,k},
@f]

@f[
R^\mathrm{acc}_{a,k} a_k + R^\mathrm{acc}_{b,k} b_k
\leq R^\mathrm{acc}_{\max,k},
@f]

@f[
\sqrt{a_k}\,
\left(
R^\mathrm{jerk}_{a,k} a_k +
R^\mathrm{jerk}_{b,k} b_k +
R^\mathrm{jerk}_{c,k} c_k +
R^\mathrm{jerk}_{d,k}
\right)
\leq R^\mathrm{jerk}_{\max,k}.
@f]

## Robot Workflow

Use `Robot` when you have physical joint-space limits.  The robot stores the
station grid, sampled path derivatives, and converted constraints.

```cpp
auto path = copp::Path::from_parametric(
    [](copp::Jet3 s) {
        return std::vector<copp::Jet3>{
            copp::powi(s, 3),
            copp::sin(s),
        };
    },
    0.0,
    1.0);

std::vector<double> s{0.0, 0.25, 0.5, 0.75, 1.0};
std::vector<double> upper{3.0, 3.0};
std::vector<double> lower{-3.0, -3.0};

copp::Robot robot(2, s.size());
robot.append_s(s)
    .set_q_from_path_3rd(path, 0, s.size())
    .add_velocity_limits(upper, lower, 0, s.size())
    .add_acceleration_limits(upper, lower, 0, s.size())
    .add_jerk_limits(upper, lower, 0, s.size());
```

Broadcast limit vectors have length `dim`.  Matrix limits have shape
`(dim x n_samples)`:

```cpp
auto upper_matrix = copp::Matrix::from_rows({
    {3.0, 3.0, 3.0},
    {2.0, 2.0, 2.0},
});
auto lower_matrix = copp::Matrix::from_rows({
    {-3.0, -3.0, -3.0},
    {-2.0, -2.0, -2.0},
});

robot.add_velocity_limits(upper_matrix.view(), lower_matrix.view(), 0);
```

`robot.constraints()` returns a borrowed `ConstraintsRef` for solver problems.
The `Robot` must outlive the solver call.

## Inverse Dynamics

Torque limits and torque objectives use the installed inverse-dynamics callback.
Without a callback, COPP uses point-mass dynamics:

@f[
\tau = \ddot{q}.
@f]

```cpp
copp::Robot robot(
    2,
    [](copp::Span<const double> q,
       copp::Span<const double> dq,
       copp::Span<const double> ddq,
       copp::Span<double> tau) {
        tau[0] = 1.5 * ddq[0] + 0.1 * dq[0] + std::sin(q[0]);
        tau[1] = 0.8 * ddq[1] + 0.05 * dq[1] + 0.5 * std::sin(q[1]);
    },
    s.size());

robot.append_s(s)
    .set_q_from_path_2nd(path, 0, s.size());

std::vector<double> tau_upper{5.0, 5.0};
std::vector<double> tau_lower{-5.0, -5.0};
robot.add_torque_limits(tau_upper, tau_lower, 0, s.size());
```

The callback is owned by `Robot`.  If it captures references, those referenced
objects must outlive the robot or at least the torque-related operation.

## Standalone Constraints

Use `Constraints` directly when you already have mathematical rows:

```cpp
copp::Constraints constraints(1, s.size());
constraints.append_s(s)
    .add_constraint_1st(std::vector<double>{1.0, 0.8, 1.0}, 0);

auto acc_a = copp::Matrix::from_rows({
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},
});
auto acc_b = copp::Matrix::from_rows({
    {1.0, 1.0, 1.0},
    {-1.0, -1.0, -1.0},
});
auto acc_max = copp::Matrix::from_rows({
    {2.0, 2.0, 2.0},
    {2.0, 2.0, 2.0},
});

constraints.add_constraint_2nd(acc_a.view(), acc_b.view(), acc_max.view(), 0);
```

This mirrors the Rust/Python split: use `Robot` for physical limits, and use
`Constraints` for direct mathematical control.

## Tutorial Sources

- `bindings/cpp/examples/robot_inverse_dynamics.cpp`
- `bindings/cpp/examples/topp2_ra.cpp`
- `bindings/cpp/examples/topp3_socp.cpp`
