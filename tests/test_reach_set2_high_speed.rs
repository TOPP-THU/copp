//! Regression tests for TOPP2-RA numerical robustness: squared path speeds
//! `a = sdot^2` far above/below unity (units of mm, um, nm), and fine station
//! grids on large-radius arcs where adjacent LP rows are nearly parallel.

use copp::diag::CoppError;
use copp::path::{Jet3, Path, cos, sin};
use copp::robot::Robot;
use copp::solver::topp2_ra::{
    ReachSet2OptionsBuilder, Topp2ProblemBuilder, s_to_t_topp2, topp2_ra,
};

const ACC: f64 = 3000.0;

fn solve(
    path: &Path,
    dim: usize,
    n: usize,
    v_max: f64,
    feed: Option<f64>,
) -> Result<(Vec<f64>, Vec<f64>), CoppError> {
    solve_with_acc(path, dim, n, v_max, ACC, feed)
}

fn solve_with_acc(
    path: &Path,
    dim: usize,
    n: usize,
    v_max: f64,
    acc: f64,
    feed: Option<f64>,
) -> Result<(Vec<f64>, Vec<f64>), CoppError> {
    let s_max = path.s_range().1;
    let s: Vec<f64> = (0..n).map(|k| s_max * k as f64 / (n - 1) as f64).collect();
    let mut robot = Robot::with_capacity(dim, n);
    robot
        .with_s(s.as_slice())?
        .with_q_from_path_2nd(path, 0, n)?
        .with_axial_velocity(
            (vec![v_max; dim].as_slice(), n),
            (vec![-v_max; dim].as_slice(), n),
            0,
        )?
        .with_axial_acceleration(
            (vec![acc; dim].as_slice(), n),
            (vec![-acc; dim].as_slice(), n),
            0,
        )?;
    if let Some(feed) = feed {
        robot
            .constraints
            .with_constraint_1order(vec![feed * feed; n].as_slice(), 0)?;
    }
    let problem = Topp2ProblemBuilder::new(&robot, (0, n - 1), (0.0, 0.0)).build()?;
    let a = topp2_ra(&problem, &ReachSet2OptionsBuilder::new().build()?)?;
    Ok((s, a))
}

/// A 1-D line with a constant velocity/acceleration box must give the textbook
/// bang-bang profile, whose traversal time is `2 v/A + (L - v^2/A)/v`, for speeds
/// well above 1000 (i.e. `a = sdot^2` well above `1e6`).
#[test]
fn line_reaches_velocity_limit_above_lp_bound() -> Result<(), CoppError> {
    let length = 1000.0;
    let path = Path::from_parametric(|s: Jet3| vec![s], 0.0, length)?;
    for v_max in [1300.0, 1420.0, 1666.667] {
        let (s, a) = solve(&path, 1, 1001, v_max, None)?;
        let peak = a.iter().cloned().fold(0.0, f64::max);
        let (t_final, _) = s_to_t_topp2(&s, &a, 0.0)?;
        let t_theory = 2.0 * v_max / ACC + (length - v_max * v_max / ACC) / v_max;
        assert!(
            (peak - v_max * v_max).abs() <= 1e-6 * v_max * v_max,
            "v_max = {v_max}: peak a = {peak}"
        );
        assert!(
            (t_final - t_theory).abs() <= 1e-6 * t_theory,
            "v_max = {v_max}: t = {t_final}, theory = {t_theory}"
        );
    }
    Ok(())
}

/// A half circle of radius 1000 driven at 1333.33 mm/s (F80000) with a first-order
/// feed constraint must cruise at the feed; no node may collapse to `a = 1e6`.
#[test]
fn arc_has_no_dip_at_lp_bound() -> Result<(), CoppError> {
    let radius = 1000.0;
    let feed = 4000.0 / 3.0;
    let path = Path::from_parametric(
        move |s: Jet3| vec![radius * cos(s / radius), radius * sin(s / radius)],
        0.0,
        std::f64::consts::PI * radius,
    )?;
    let (s, a) = solve(&path, 2, 1001, feed, Some(feed))?;
    let cruise = a
        .iter()
        .filter(|&&x| x >= feed * feed * (1.0 - 1e-6))
        .count();
    let dips = a.iter().filter(|&&x| (x - 1e6).abs() < 1.0).count();
    let (t_final, _) = s_to_t_topp2(&s, &a, 0.0)?;
    // Centripetal load is below the axis limit, so the profile is accelerate / cruise / decelerate.
    let t_theory = 2.0 * feed / ACC + (std::f64::consts::PI * radius - feed * feed / ACC) / feed;
    assert_eq!(dips, 0, "nodes stuck at a = 1e6");
    assert!(
        cruise > a.len() * 3 / 4,
        "cruise nodes = {cruise} of {}",
        a.len()
    );
    assert!(
        (t_final - t_theory).abs() <= 5e-3 * t_theory,
        "t = {t_final}, theory = {t_theory}"
    );
    Ok(())
}

fn half_circle(radius: f64) -> Result<Path, CoppError> {
    Ok(Path::from_parametric(
        move |s: Jet3| vec![radius * cos(s / radius), radius * sin(s / radius)],
        0.0,
        std::f64::consts::PI * radius,
    )?)
}

/// The same line-and-box problem expressed in m, mm, um and nm (`a` spanning 24
/// orders of magnitude) must give the same traversal time: no internal constant
/// may depend on the unit of the path parameter.
#[test]
fn line_is_unit_invariant() -> Result<(), CoppError> {
    let mut times = Vec::new();
    for unit in [1e-3, 1.0, 1e3, 1e6] {
        let length = 1000.0 * unit;
        let v_max = 1666.667 * unit;
        let path = Path::from_parametric(|s: Jet3| vec![s], 0.0, length)?;
        let (s, a) = solve_with_acc(&path, 1, 1001, v_max, ACC * unit, None)?;
        times.push(s_to_t_topp2(&s, &a, 0.0)?.0);
    }
    for &t in &times[1..] {
        assert!((t - times[0]).abs() <= 1e-9 * times[0], "times = {times:?}");
    }
    Ok(())
}

/// A half circle expressed in nm (`a ~ 2e18`) on a 10 000-station grid: the rows of
/// the reachability LP have tiny norms and must still be normalized exactly.
#[test]
fn arc_is_unit_invariant_at_nm_scale() -> Result<(), CoppError> {
    let unit = 1e6;
    let feed = 4000.0 / 3.0 * unit;
    let path = half_circle(1000.0 * unit)?;
    let (s, a) = solve_with_acc(&path, 2, 10001, feed, ACC * unit, Some(feed))?;
    let (t_final, _) = s_to_t_topp2(&s, &a, 0.0)?;
    let t_theory = 2.0 * feed / (ACC * unit)
        + (std::f64::consts::PI * 1000.0 * unit - feed * feed / (ACC * unit)) / feed;
    assert!(
        (t_final - t_theory).abs() <= 5e-3 * t_theory,
        "t = {t_final}, theory = {t_theory}"
    );
    Ok(())
}
