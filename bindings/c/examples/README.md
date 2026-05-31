# COPP C Examples

These examples mirror the open-source algorithm examples in the repository where the C ABI currently exposes the same solver. They use the same analytic Lissajous path, `N = 1001` station grid, symmetric axis limits `[-1, 1]`, solver objectives, and TOPP3/COPP3 setup flow.

The examples use callback-backed paths so their output can match the native parametric-path examples exactly. C users do not need automatic differentiation for ordinary usage: if your path is available as waypoints, use `copp_path_from_waypoints` instead. COPP will build a spline path and the same `copp_robot_sample_path_2nd` / `copp_robot_sample_path_3rd` calls used in these examples will sample the required derivatives from that spline.

Minimal waypoint construction:

```c
double waypoints[dim * num_waypoints]; /* column-major: row + col * dim */
struct CoppPathOptions path_options;
struct CoppPath *path = NULL;

copp_path_default_options(0.0, 1.0, &path_options);
/* path_options.order defaults to 5; override it if your application needs a different odd spline order. */
copp_path_from_waypoints(
    COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(waypoints, dim, num_waypoints),
    path_options,
    &path);
```

| C example | Demonstrates |
| --- | --- |
| `topp2_ra.c` | TOPP2 reachability analysis and `a(s) -> t(s) -> s(t)` interpolation |
| `reach_set2.c` | Second-order reachable interval construction |
| `copp2_socp.c` | COPP2 SOCP with Clarabel options |
| `topp3_lp.c` | TOPP3 LP with Clarabel options |
| `topp3_socp.c` | TOPP3 SOCP with Clarabel options |
| `copp3_socp.c` | COPP3 SOCP with time plus thermal-energy objectives |

Algorithms not listed in the open-source availability table in the [COPP repository README](https://github.com/TOPP-THU/copp#readme) are not documented here as public C examples.
