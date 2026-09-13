//! Regression tests for second-order inverse time interpolation `s(t)`.

use copp::InterpolationMode;
use copp::solver::topp2_ra::{s_to_t_topp2, t_to_s_topp2};

/// A node profile that is constant up to 1-ulp rounding noise must sample to a
/// uniformly spaced `s(t)`: the tiny slope `(a[i+1]-a[i])/ds` must not amplify
/// floating-point cancellation into visible position error.
#[test]
fn t_to_s_topp2_is_stable_for_nearly_constant_profile() {
    let n = 20000usize;
    let ds = 0.01;
    let sdot: f64 = 400.0 / 3.0; // 133.333... mm/s
    let a_base = sdot * sdot;
    let s: Vec<f64> = (0..n).map(|i| i as f64 * ds).collect();
    // Alternate between adjacent representable doubles (1 ulp apart).
    let a: Vec<f64> = (0..n)
        .map(|i| {
            if i % 2 == 0 {
                a_base
            } else {
                f64::from_bits(a_base.to_bits() + 1)
            }
        })
        .collect();
    let (_t_final, t_s) = s_to_t_topp2(&s, &a, 0.0).unwrap();
    let dt = 0.002;
    let s_t = t_to_s_topp2(
        &s,
        &a,
        &t_s,
        InterpolationMode::UniformTimeGrid(0.0, dt, false),
    )
    .unwrap();
    assert!(s_t.len() > 100);
    let expected_step = sdot * dt;
    let mut worst = 0.0f64;
    for pair in s_t.windows(2) {
        worst = worst.max((pair[1] - pair[0] - expected_step).abs());
    }
    assert!(
        worst < 1.0e-9,
        "sampled s(t) spacing deviates from {expected_step} by up to {worst}"
    );
}

/// Sanity: a genuinely accelerating profile still matches the closed-form
/// constant-acceleration solution.
#[test]
fn t_to_s_topp2_matches_constant_acceleration_solution() {
    // s'' = b constant => a(s) = 2*b*s, s(t) = 0.5*b*t^2 (starting from rest is
    // singular, so start from s0 with a0 > 0).
    let b = 3.0;
    let s0 = 1.0;
    let n = 501usize;
    let s: Vec<f64> = (0..n).map(|i| s0 + i as f64 * 0.01).collect();
    let a: Vec<f64> = s.iter().map(|&x| 2.0 * b * x).collect();
    let (_t_final, t_s) = s_to_t_topp2(&s, &a, 0.0).unwrap();
    let dt = 1.0e-3;
    let s_t = t_to_s_topp2(
        &s,
        &a,
        &t_s,
        InterpolationMode::UniformTimeGrid(0.0, dt, false),
    )
    .unwrap();
    let v0 = (2.0 * b * s0).sqrt();
    for (i, &value) in s_t.iter().enumerate() {
        let t = i as f64 * dt;
        let exact = s0 + v0 * t + 0.5 * b * t * t;
        assert!(
            (value - exact).abs() < 1.0e-9,
            "i={i} value={value} exact={exact}"
        );
    }
}
