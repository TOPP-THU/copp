//! Public-API integration regressions for tolerance-bounded waypoint smoothing.
//!
//! The default test set covers short corners, smooth/noisy curves, raster and
//! staircase paths, selected-axis fitting, nonuniform parameters, unit scaling,
//! derivative scaling, invalid inputs, and independent dense sampling against
//! the same-parameter reference polyline.
//!
//! Large 100,000--300,000 point campaigns, expanded segment budgets, full
//! tolerance sweeps, and documentation benchmarks are marked `#[ignore]` and
//! are intended to be run explicitly in release mode. Keeping those workloads
//! ignored prevents ordinary test runs from becoming performance campaigns.

/// Deterministic path families shared by integration tests and the plotting example.
pub(crate) mod cases {
    use nalgebra::DMatrix;

    /// Canonical smooth, noisy, scanning, staircase, and sub-tolerance families.
    pub const FAMILIES: [&str; 8] = [
        "smooth",
        "noisy_smooth",
        "raster",
        "noisy_raster",
        "stair",
        "noisy_stair",
        "micro_stair",
        "noisy_micro_stair",
    ];

    /// Return programmed vertices for a 20-row raster or staircase path.
    pub fn vertices(raster: bool) -> Vec<[f64; 3]> {
        let mut points = vec![[0., 0., 0.]];
        for row in 0..20 {
            if raster {
                points.push([if row % 2 == 0 { 10. } else { 0. }, row as f64 * 0.5, 0.]);
                if row < 19 {
                    points.push([points.last().unwrap()[0], (row + 1) as f64 * 0.5, 0.]);
                }
            } else {
                points.push([(row + 1) as f64, row as f64 * 0.2, 0.]);
                points.push([(row + 1) as f64, (row + 1) as f64 * 0.2, 0.]);
            }
        }
        points
    }

    /// Build the sparse programmed path and its normalized chord-length parameter.
    pub fn sparse(raster: bool) -> (DMatrix<f64>, Vec<f64>) {
        let v = vertices(raster);
        let mut s = vec![0.];
        for w in v.windows(2) {
            let ds = (0..3)
                .map(|a| (w[1][a] - w[0][a]).powi(2))
                .sum::<f64>()
                .sqrt();
            s.push(s.last().unwrap() + ds);
        }
        let length = *s.last().unwrap();
        for x in &mut s {
            *x /= length;
        }
        (DMatrix::from_fn(3, v.len(), |a, i| v[i][a]), s)
    }

    /// Generate one deterministic sampled family.
    ///
    /// Returns `(input, parameters, truth)`. Noisy cases perturb only interior
    /// columns with a fixed pseudo-random sequence so failures remain reproducible.
    pub fn case(name: &str, n: usize) -> (DMatrix<f64>, Vec<f64>, DMatrix<f64>) {
        let mut s: Vec<_> = (0..n).map(|i| i as f64 / (n - 1) as f64).collect();
        let mut truth = DMatrix::zeros(3, n);
        if name.contains("smooth") {
            for (i, &u) in s.iter().enumerate() {
                truth[(0, i)] = 10. * u;
                truth[(1, i)] = (std::f64::consts::TAU * u).sin();
                truth[(2, i)] = 0.3 * (2. * std::f64::consts::TAU * u).cos();
            }
        } else if name.contains("micro") {
            for (i, &u) in s.iter().enumerate() {
                let z = u * 200.;
                let k = z.floor();
                let f = z - k;
                truth[(0, i)] = (k + (2. * f).min(1.)) * 0.001;
                truth[(1, i)] = (k + (2. * f - 1.).max(0.)) * 0.001;
            }
        } else {
            let (v, t) = sparse(name.contains("raster"));
            // Keep every programmed corner while increasing the sample density.
            // Otherwise a nominal raster would contain resampling-induced diagonals.
            if n >= t.len() {
                let extra = n - t.len();
                s.clear();
                s.push(0.);
                for i in 0..t.len() - 1 {
                    let count = 1 + ((extra as f64 * t[i + 1]).floor() as usize)
                        - ((extra as f64 * t[i]).floor() as usize);
                    for j in 1..=count {
                        s.push(if j == count {
                            t[i + 1]
                        } else {
                            t[i] + (t[i + 1] - t[i]) * j as f64 / count as f64
                        });
                    }
                }
                assert_eq!(s.len(), n);
            }
            let mut j = 0;
            for (i, &u) in s.iter().enumerate() {
                while j + 2 < t.len() && t[j + 1] < u {
                    j += 1;
                }
                let f = (u - t[j]) / (t[j + 1] - t[j]);
                for a in 0..3 {
                    truth[(a, i)] = v[(a, j)] + f * (v[(a, j + 1)] - v[(a, j)]);
                }
            }
        }
        let mut points = truth.clone();
        let mut rng = 20260908_u64;
        if name.starts_with("noisy") {
            let amplitude = if name.contains("smooth") {
                0.0002
            } else {
                0.0001
            };
            for i in 1..n - 1 {
                for a in 0..3 {
                    rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
                    let r = ((rng >> 11) as f64) / (1_u64 << 53) as f64;
                    points[(a, i)] += (2. * r - 1.) * amplitude;
                }
            }
        }
        (points, s, truth)
    }
}

#[cfg(test)]
mod tests {
    use super::cases;
    use copp::path::{OutOfRangeMode, Path, SmoothingConfig, SmoothingTolerance, SplineConfig};
    use nalgebra::{Const, DMatrix, DMatrixView, Dyn};
    use std::{error::Error, fs, time::Instant};

    /// Independently sample the returned curve and verify its public contract.
    ///
    /// This complements the constructor's Bernstein audit with a 4,003-point
    /// grid, finite third derivatives, report bounds, and endpoint checks within
    /// floating-point roundoff.
    fn check(points: &DMatrix<f64>, s: &[f64], path: &Path, eps: f64) {
        let report = path.smoothing_report().unwrap();
        assert!(report.max_errors.iter().all(|v| v.is_finite() && *v <= eps));
        let samples: Vec<_> = (0..4003)
            .map(|i| s[0] + (s[s.len() - 1] - s[0]) * i as f64 / 4002.)
            .collect();
        let out = path.evaluate_up_to_3rd(&samples).unwrap();
        assert!(out.q.iter().all(|v| v.is_finite()));
        assert!(out.dddq.as_ref().unwrap().iter().all(|v| v.is_finite()));
        let mut j = 0;
        for (i, &x) in samples.iter().enumerate() {
            while j + 2 < s.len() && s[j + 1] < x {
                j += 1;
            }
            let f = (x - s[j]) / (s[j + 1] - s[j]);
            for &a in &report.axes {
                let r = points[(a, j)] + f * (points[(a, j + 1)] - points[(a, j)]);
                assert!((out.q[(a, i)] - r).abs() <= eps + 1e-10, "axis {a} x={x}");
            }
        }
        let ends = path.evaluate_q(&[s[0], s[s.len() - 1]]).unwrap();
        for a in 0..points.nrows() {
            assert!((ends.q[(a, 0)] - points[(a, 0)]).abs() < 1e-9);
            assert!((ends.q[(a, 1)] - points[(a, points.ncols() - 1)]).abs() < 1e-9);
        }
    }

    #[test]
    fn few_points_and_sparse_corners() {
        for n in 2..=5 {
            let p = DMatrix::from_fn(2, n, |a, i| {
                if a == 0 {
                    (i / 2) as f64
                } else {
                    i.div_ceil(2) as f64
                }
            });
            let s: Vec<_> = (0..n).map(|i| i as f64 / (n - 1) as f64).collect();
            let path = Path::from_waypoints_fitting(&p, SmoothingConfig::default()).unwrap();
            check(&p, &s, &path, 0.001);
        }
        for raster in [true, false] {
            let (p, s) = cases::sparse(raster);
            let path = Path::from_waypoints_fitting(
                &p,
                SmoothingConfig {
                    parameters: Some(s.clone()),
                    ..Default::default()
                },
            )
            .unwrap();
            check(&p, &s, &path, 0.001);
        }
    }

    #[test]
    fn all_families_across_small_and_medium_scales() {
        for name in cases::FAMILIES {
            for n in [64, 1000, 10000] {
                let (p, s, _) = cases::case(name, n);
                let path = Path::from_waypoints_fitting(
                    &p,
                    SmoothingConfig {
                        parameters: Some(s.clone()),
                        ..Default::default()
                    },
                )
                .unwrap_or_else(|e| panic!("{name}/{n}: {e}"));
                check(&p, &s, &path, 0.001);
            }
        }
    }

    #[test]
    fn fitting_view_matches_owned_for_padded_column_major() {
        const DIM: usize = 2;
        const N: usize = 5;
        const LEADING_DIM: usize = 3;
        let owned =
            DMatrix::from_row_slice(DIM, N, &[0.0, 0.0, 1.0, 1.0, 2.0, 0.0, 1.0, 1.0, 2.0, 2.0]);
        let from_view = {
            let data = [
                0.0,
                0.0,
                f64::NAN,
                0.0,
                1.0,
                f64::NAN,
                1.0,
                1.0,
                f64::NAN,
                1.0,
                2.0,
                f64::NAN,
                2.0,
                2.0,
                f64::NAN,
            ];
            let view = DMatrixView::from_slice_with_strides_generic(
                &data,
                Dyn(DIM),
                Dyn(N),
                Const::<1>,
                Dyn(LEADING_DIM),
            );
            Path::from_waypoints_fitting_view(
                view,
                SmoothingConfig {
                    axes: Some(vec![0]),
                    ..Default::default()
                },
            )
            .unwrap()
        };
        let from_owned = Path::from_waypoints_fitting(
            &owned,
            SmoothingConfig {
                axes: Some(vec![0]),
                ..Default::default()
            },
        )
        .unwrap();
        let parameters = [0.0, 0.25, 0.5, 0.75, 1.0];
        check(&owned, &parameters, &from_view, 0.001);

        let query: Vec<_> = (0..65).map(|i| i as f64 / 64.0).collect();
        let view_values = from_view.evaluate_up_to_3rd(&query).unwrap();
        let owned_values = from_owned.evaluate_up_to_3rd(&query).unwrap();
        assert!((&view_values.q - &owned_values.q).amax() < 1e-12);
        assert!((view_values.dq.unwrap() - owned_values.dq.unwrap()).amax() < 1e-12);
        assert!((view_values.ddq.unwrap() - owned_values.ddq.unwrap()).amax() < 1e-12);
        assert!((view_values.dddq.unwrap() - owned_values.dddq.unwrap()).amax() < 1e-12);
    }

    #[test]
    #[ignore = "large-scale release validation: cargo test --release --test test_path_smoothing -- --ignored"]
    fn hundred_thousand_to_three_hundred_thousand_points() {
        for name in cases::FAMILIES {
            for n in [100000, 300000] {
                let (p, s, _) = cases::case(name, n);
                let path = Path::from_waypoints_fitting(
                    &p,
                    SmoothingConfig {
                        parameters: Some(s.clone()),
                        ..Default::default()
                    },
                )
                .unwrap_or_else(|e| panic!("{name}/{n}: {e}"));
                check(&p, &s, &path, 0.001);
            }
        }
    }

    #[test]
    fn selected_axes_and_unchanged_interpolation() {
        let p = DMatrix::from_fn(5, 20, |a, i| ((i as f64) / 19. * (a + 1) as f64).sin());
        let cfg = SmoothingConfig {
            axes: Some(vec![4, 0]),
            tolerance: SmoothingTolerance::PerAxis(vec![0.002, 0.003]),
            ..Default::default()
        };
        let smoothed = Path::from_waypoints_fitting(&p, cfg).unwrap();
        let original = Path::from_waypoints_interpolating(&p, SplineConfig::default()).unwrap();
        let s: Vec<_> = (0..113).map(|i| i as f64 / 112.).collect();
        let a = smoothed.evaluate_up_to_3rd(&s).unwrap();
        let b = original.evaluate_up_to_3rd(&s).unwrap();
        for axis in [1, 2, 3] {
            for i in 0..s.len() {
                assert!((a.q[(axis, i)] - b.q[(axis, i)]).abs() < 1e-10);
                assert!(
                    (a.dddq.as_ref().unwrap()[(axis, i)] - b.dddq.as_ref().unwrap()[(axis, i)])
                        .abs()
                        < 1e-5
                );
            }
        }
        assert_eq!(smoothed.smoothing_report().unwrap().axes, vec![4, 0]);
    }

    #[test]
    fn nonuniform_parameters_units_and_derivatives() {
        let s: Vec<_> = (0..121).map(|i| (i as f64 / 120.).powf(1.6)).collect();
        let p = DMatrix::from_fn(3, s.len(), |a, i| (s[i] * (a + 1) as f64).sin());
        let a = Path::from_waypoints_fitting(
            &p,
            SmoothingConfig {
                parameters: Some(s.clone()),
                ..Default::default()
            },
        )
        .unwrap();
        let units = [1000., 0.01, 180. / std::f64::consts::PI];
        let q = DMatrix::from_fn(3, s.len(), |r, i| p[(r, i)] * units[r]);
        let b = Path::from_waypoints_fitting(
            &q,
            SmoothingConfig {
                parameters: Some(s.iter().map(|v| 7. + 12. * v).collect()),
                tolerance: SmoothingTolerance::PerAxis(units.iter().map(|v| 0.001 * v).collect()),
                ..Default::default()
            },
        )
        .unwrap();
        let x = [0.1, 0.3, 0.7, 0.9];
        let av = a.evaluate_up_to_3rd(&x).unwrap();
        let bv = b.evaluate_up_to_3rd(&x.map(|v| 7. + 12. * v)).unwrap();
        for (r, &unit) in units.iter().enumerate() {
            for i in 0..4 {
                assert!((av.q[(r, i)] * unit - bv.q[(r, i)]).abs() < 1e-6);
                assert!(
                    (av.dddq.as_ref().unwrap()[(r, i)] * unit / 12_f64.powi(3)
                        - bv.dddq.as_ref().unwrap()[(r, i)])
                        .abs()
                        < 1e-5
                );
            }
        }
        check(&p, &s, &a, 0.001);
    }

    #[test]
    fn invalid_input_and_budgets_never_return_unchecked_curve() {
        let p = DMatrix::from_row_slice(2, 3, &[0., 1., 1., 0., 0., 1.]);
        for cfg in [
            SmoothingConfig {
                tolerance: 0_f64.into(),
                ..Default::default()
            },
            SmoothingConfig {
                axes: Some(vec![0, 0]),
                ..Default::default()
            },
            SmoothingConfig {
                parameters: Some(vec![0., 0., 1.]),
                ..Default::default()
            },
            SmoothingConfig {
                max_segments: 1,
                ..Default::default()
            },
        ] {
            assert!(Path::from_waypoints_fitting(&p, cfg).is_err());
        }
        let path = Path::from_waypoints_fitting(&p, SmoothingConfig::default()).unwrap();
        assert!(path.evaluate_q(&[f64::NAN]).is_err());
        assert!(path.evaluate_q(&[f64::INFINITY]).is_err());
        assert!(path.evaluate_q(&[-1.]).is_err());
        let clamp = Path::from_waypoints_fitting(
            &p,
            SmoothingConfig {
                out_of_range_mode: OutOfRangeMode::Clamp,
                ..Default::default()
            },
        )
        .unwrap();
        assert_eq!(
            clamp.evaluate_q(&[-1.]).unwrap().q,
            clamp.evaluate_q(&[0.]).unwrap().q
        );
    }

    #[test]
    fn stationary_paths_and_nonuniform_unselected_axes() {
        for n in [2, 5, 81] {
            let s: Vec<_> = (0..n)
                .map(|i| (i as f64 / (n - 1) as f64).powf(1.5))
                .collect();
            let stationary = DMatrix::from_element(7, n, 3.25);
            let path =
                Path::from_waypoints_fitting(&stationary, SmoothingConfig::default()).unwrap();
            assert_eq!(path.smoothing_report().unwrap().segments, 1);
            let out = path.evaluate_up_to_3rd(&[0., 0.123, 0.73, 1.]).unwrap();
            assert!(out.q.iter().all(|v| (v - 3.25).abs() < 1e-12));
            let p = DMatrix::from_fn(3, n, |a, i| {
                if a == 0 {
                    s[i]
                } else {
                    (s[i] * (a + 1) as f64).sin()
                }
            });
            let path = Path::from_waypoints_fitting(
                &p,
                SmoothingConfig {
                    axes: Some(vec![0]),
                    parameters: Some(s.clone()),
                    ..Default::default()
                },
            )
            .unwrap();
            let result = path.evaluate_q(&s).unwrap();
            assert!((&result.q - &p).amax() < 1e-10);
            let endpoints = path.evaluate_up_to_2nd(&[0., 1.]).unwrap();
            for a in [1, 2] {
                for i in 0..2 {
                    assert!(endpoints.dq.as_ref().unwrap()[(a, i)].abs() < 1e-9);
                    assert!(endpoints.ddq.as_ref().unwrap()[(a, i)].abs() < 1e-7);
                }
            }
        }
    }

    #[test]
    fn tolerance_sweep_short_corner() {
        let points = DMatrix::from_row_slice(2, 3, &[0., 1., 1., 0., 0., 1.]);
        for tolerance in [1e-10, 1e-8, 1e-6, 1e-4, 1e-3, 1e-2, 0.1, 1., 10., 100.] {
            let path = Path::from_waypoints_fitting(
                &points,
                SmoothingConfig {
                    tolerance: tolerance.into(),
                    ..Default::default()
                },
            )
            .unwrap();
            check(&points, &[0., 0.5, 1.], &path, tolerance);
            // Strict independent sampling without the general helper's roundoff allowance.
            let s: Vec<_> = (0..4097).map(|i| i as f64 / 4096.).collect();
            let out = path.evaluate_q(&s).unwrap();
            for (i, &x) in s.iter().enumerate() {
                let reference = if x <= 0.5 {
                    [2. * x, 0.]
                } else {
                    [1., 2. * x - 1.]
                };
                for (a, &reference) in reference.iter().enumerate() {
                    assert!((out.q[(a, i)] - reference).abs() <= tolerance);
                }
            }
        }
    }

    #[test]
    #[ignore = "budget expansion stress case; run in release with --include-ignored"]
    fn tolerance_budget_expansion() {
        let (points, s, _) = cases::case("noisy_smooth", 10000);
        let tolerance = 1e-4;
        let default = Path::from_waypoints_fitting(
            &points,
            SmoothingConfig {
                tolerance: tolerance.into(),
                parameters: Some(s.clone()),
                ..Default::default()
            },
        );
        match default {
            Ok(path) => check(&points, &s, &path, tolerance),
            Err(error) => assert!(error.to_string().contains("budget")),
        }
        let expanded = Path::from_waypoints_fitting(
            &points,
            SmoothingConfig {
                tolerance: tolerance.into(),
                parameters: Some(s.clone()),
                max_segments: 60000,
                ..Default::default()
            },
        )
        .unwrap();
        check(&points, &s, &expanded, tolerance);
    }

    const TOLERANCES: [f64; 13] = [
        1e-14, 1e-12, 1e-10, 1e-8, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 0.1, 1., 10., 100.,
    ];

    fn write_tolerance_case(
        name: &str,
        p: &DMatrix<f64>,
        s: &[f64],
        writer: &mut csv::Writer<fs::File>,
    ) -> Result<(), Box<dyn Error>> {
        write_tolerance_case_with_budget(name, p, s, writer, &TOLERANCES, 20_000)
    }

    fn write_tolerance_case_with_budget(
        name: &str,
        p: &DMatrix<f64>,
        s: &[f64],
        writer: &mut csv::Writer<fs::File>,
        tolerances: &[f64],
        budget: usize,
    ) -> Result<(), Box<dyn Error>> {
        let mut violation = false;
        for &eps in tolerances {
            let tick = Instant::now();
            let result = Path::from_waypoints_fitting(
                p,
                SmoothingConfig {
                    tolerance: eps.into(),
                    max_segments: budget,
                    parameters: Some(s.to_vec()),
                    ..Default::default()
                },
            );
            let ms = tick.elapsed().as_secs_f64() * 1000.;
            match result {
                Err(e) => {
                    let message = e.to_string();
                    let status = if message.contains("budget") || message.contains("limit reached")
                    {
                        "budget"
                    } else {
                        "numerical"
                    };
                    writer.write_record([
                        name.into(),
                        p.ncols().to_string(),
                        format!("{eps:e}"),
                        status.into(),
                        String::new(),
                        String::new(),
                        format!("{ms:.6}"),
                        String::new(),
                        String::new(),
                        message,
                    ])?;
                }
                Ok(path) => {
                    let report = path.smoothing_report().unwrap();
                    let bound = report.max_errors.iter().copied().fold(0., f64::max);
                    // Include every input node, interval midpoints, and an independent grid.
                    let mut query = Vec::with_capacity(2 * s.len() + 4097);
                    query.extend_from_slice(s);
                    query.extend(s.windows(2).map(|v| (v[0] + v[1]) / 2.));
                    query.extend((0..4097).map(|i| i as f64 / 4096.));
                    query.sort_by(f64::total_cmp);
                    query.dedup();
                    let mut max_error = 0_f64;
                    let mut finite = true;
                    let mut interval = 0;
                    for chunk in query.chunks(8192) {
                        let q = path.evaluate_up_to_3rd(chunk)?;
                        finite &=
                            q.q.iter()
                                .chain(q.dq.as_ref().unwrap().iter())
                                .chain(q.ddq.as_ref().unwrap().iter())
                                .chain(q.dddq.as_ref().unwrap().iter())
                                .all(|v| v.is_finite());
                        for (i, &x) in chunk.iter().enumerate() {
                            while interval + 2 < s.len() && s[interval + 1] < x {
                                interval += 1;
                            }
                            let f = (x - s[interval]) / (s[interval + 1] - s[interval]);
                            for a in 0..p.nrows() {
                                let reference = p[(a, interval)]
                                    + f * (p[(a, interval + 1)] - p[(a, interval)]);
                                max_error = max_error.max((q.q[(a, i)] - reference).abs());
                            }
                        }
                    }
                    let status = if !finite || max_error > eps || bound > eps {
                        "violation"
                    } else {
                        "ok"
                    };
                    violation |= status == "violation";
                    writer.write_record([
                        name.into(),
                        p.ncols().to_string(),
                        format!("{eps:e}"),
                        status.into(),
                        report.segments.to_string(),
                        report.refinements.to_string(),
                        format!("{ms:.6}"),
                        format!("{bound:e}"),
                        format!("{max_error:e}"),
                        String::new(),
                    ])?;
                }
            }
            writer.flush()?;
            println!("{name} n={} tolerance={eps:e} build={ms:.3}ms", p.ncols());
        }
        if violation {
            return Err("a successful curve failed the independent tolerance audit".into());
        }
        Ok(())
    }

    fn tolerance_writer(name: &str) -> Result<csv::Writer<fs::File>, Box<dyn Error>> {
        let out =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("data/path_smoothing_tolerance");
        fs::create_dir_all(&out)?;
        let mut writer = csv::Writer::from_path(out.join(name))?;
        writer.write_record([
            "case",
            "n",
            "tolerance",
            "status",
            "segments",
            "refinements",
            "build_ms",
            "bound",
            "sampled_error",
            "error",
        ])?;
        Ok(writer)
    }

    #[test]
    #[ignore = "full tolerance/scale sweep; run explicitly in release"]
    fn tolerance_sweep_all_scales() -> Result<(), Box<dyn Error>> {
        let mut writer = tolerance_writer("sweep.csv")?;
        for n in 2..=5 {
            let p = DMatrix::from_fn(3, n, |a, i| match a {
                0 => (i / 2) as f64,
                1 => i.div_ceil(2) as f64,
                _ => 0.,
            });
            let s: Vec<_> = (0..n).map(|i| i as f64 / (n - 1) as f64).collect();
            write_tolerance_case("short_corner", &p, &s, &mut writer)?;
        }
        for raster in [true, false] {
            let (p, s) = cases::sparse(raster);
            write_tolerance_case(
                if raster {
                    "sparse_raster"
                } else {
                    "sparse_stair"
                },
                &p,
                &s,
                &mut writer,
            )?;
        }
        for n in [64, 1000, 10000, 100000, 300000] {
            for name in cases::FAMILIES {
                let (p, s, _) = cases::case(name, n);
                write_tolerance_case(name, &p, &s, &mut writer)?;
            }
        }
        Ok(())
    }

    #[test]
    #[ignore = "expanded-budget tolerance sweep; run explicitly in release"]
    fn tolerance_sweep_expanded_budget() -> Result<(), Box<dyn Error>> {
        let mut writer = tolerance_writer("budget_probe.csv")?;
        for name in ["noisy_smooth", "noisy_raster"] {
            let (p, s, _) = cases::case(name, 10_000);
            write_tolerance_case_with_budget(name, &p, &s, &mut writer, &[1e-6, 1e-4], 60_000)?;
        }
        Ok(())
    }

    /// Comparable public-API timings for the algorithm documentation.
    #[test]
    #[ignore = "documentation benchmark; run in release with one test thread"]
    fn benchmark_path_algorithms() -> Result<(), Box<dyn Error>> {
        use std::hint::black_box;
        let out =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("data/path_algorithm_benchmark");
        fs::create_dir_all(&out)?;
        let mut csv = csv::Writer::from_path(out.join("timings.csv"))?;
        csv.write_record([
            "case",
            "n",
            "dim",
            "method",
            "segments",
            "build_min_ms",
            "build_median_ms",
            "build_max_ms",
            "eval_min_ms",
            "eval_median_ms",
            "eval_max_ms",
            "node_error",
            "fit_error_bound",
        ])?;
        let query: Vec<_> = (0..10001).map(|i| i as f64 / 10000.).collect();
        for n in [2, 5, 100, 1000, 10000, 100000, 300000] {
            for name in cases::FAMILIES {
                if n < 40 && name != "smooth" {
                    continue;
                }
                let (points, _, _) = cases::case(name, n);
                // The interpolation API currently supports uniform parameters only.
                // Both methods use the same points and the same index-based [0,1] parameter.
                for fitting in [false, true] {
                    let build = || {
                        if fitting {
                            Path::from_waypoints_fitting(&points, SmoothingConfig::default())
                        } else {
                            Path::from_waypoints_interpolating(&points, SplineConfig::default())
                        }
                    };
                    let warm = build()?;
                    black_box(warm.evaluate_up_to_3rd(&query)?);
                    drop(warm);
                    let mut build_ms = Vec::new();
                    let mut path = None;
                    for _ in 0..5 {
                        let start = Instant::now();
                        let current = black_box(build()?);
                        build_ms.push(start.elapsed().as_secs_f64() * 1000.);
                        // Previous-path destruction is outside the measured interval.
                        path = Some(current);
                    }
                    let path = path.unwrap();
                    let mut eval_ms = Vec::new();
                    for _ in 0..5 {
                        let start = Instant::now();
                        let result = black_box(path.evaluate_up_to_3rd(&query)?);
                        eval_ms.push(start.elapsed().as_secs_f64() * 1000.);
                        assert!(
                            result
                                .q
                                .iter()
                                .chain(result.dq.as_ref().unwrap().iter())
                                .chain(result.ddq.as_ref().unwrap().iter())
                                .chain(result.dddq.as_ref().unwrap().iter())
                                .all(|x| x.is_finite())
                        );
                    }
                    build_ms.sort_by(f64::total_cmp);
                    eval_ms.sort_by(f64::total_cmp);
                    let mut node_error = 0_f64;
                    for begin in (0..n).step_by(8192) {
                        let end = (begin + 8192).min(n);
                        let s: Vec<_> = (begin..end).map(|i| i as f64 / (n - 1) as f64).collect();
                        let q = path.evaluate_q(&s)?.q;
                        for i in begin..end {
                            for a in 0..3 {
                                node_error =
                                    node_error.max((q[(a, i - begin)] - points[(a, i)]).abs());
                            }
                        }
                    }
                    let (segments, bound) = if let Some(report) = path.smoothing_report() {
                        let b = report.max_errors.iter().copied().fold(0., f64::max);
                        assert!(b <= 0.001 && node_error <= 0.001);
                        (report.segments, format!("{b:e}"))
                    } else {
                        assert!(node_error < 1e-9);
                        (n - 1, String::new())
                    };
                    let method = if fitting { "fitting" } else { "interpolation" };
                    csv.write_record([
                        name.to_string(),
                        n.to_string(),
                        "3".into(),
                        method.into(),
                        segments.to_string(),
                        format!("{:.6}", build_ms[0]),
                        format!("{:.6}", build_ms[2]),
                        format!("{:.6}", build_ms[4]),
                        format!("{:.6}", eval_ms[0]),
                        format!("{:.6}", eval_ms[2]),
                        format!("{:.6}", eval_ms[4]),
                        format!("{node_error:e}"),
                        bound,
                    ])?;
                    csv.flush()?;
                    println!(
                        "{name:20} N={n:7} {method:14} build={:.3} ms eval={:.3} ms segments={segments}",
                        build_ms[2], eval_ms[2]
                    );
                }
            }
        }
        Ok(())
    }
}
