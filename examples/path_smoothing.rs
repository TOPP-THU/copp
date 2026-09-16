//! This example uses [`Path::from_waypoints_fitting`] to fit representative
//! waypoint paths, benchmark construction and evaluation, and plot the resulting
//! paths, first three parameter derivatives, and fitting errors.

#[path = "../tests/test_path_smoothing.rs"]
mod smoothing_tests;
use copp::path::{Path, SmoothingConfig};
use nalgebra::DMatrix;
use plotters::coord::Shift;
use plotters::prelude::*;
use smoothing_tests::cases;
use std::{error::Error, fs, path::PathBuf, time::Instant};

/// Dense curve/reference data needed by the shared path and error plotter.
struct PlotCase {
    name: String,
    raw: DMatrix<f64>,
    curve: DMatrix<f64>,
    errors: DMatrix<f64>,
    segments: usize,
}

/// Measure three constructions plus one 10,001-point third-order evaluation.
///
/// The median/minimum/maximum construction times and selected smoothing-report
/// metrics are appended to `benchmark.csv` by the caller.
fn measure(
    name: &str,
    p: &DMatrix<f64>,
    s: &[f64],
    out: &mut csv::Writer<std::fs::File>,
) -> Result<(), Box<dyn Error>> {
    let mut times = Vec::new();
    let mut result = None;
    for _ in 0..3 {
        let tick = Instant::now();
        let path = Path::from_waypoints_fitting(
            p,
            SmoothingConfig {
                parameters: Some(s.to_vec()),
                ..Default::default()
            },
        )?;
        times.push(tick.elapsed().as_secs_f64() * 1000.);
        result = Some(path);
    }
    let path = result.unwrap();
    let r = path.smoothing_report().unwrap();
    let samples: Vec<_> = (0..10001).map(|i| i as f64 / 10000.).collect();
    let tick = Instant::now();
    let eval = path.evaluate_up_to_3rd(&samples)?;
    let eval_ms = tick.elapsed().as_secs_f64() * 1000.;
    assert!(eval.dddq.as_ref().unwrap().iter().all(|v| v.is_finite()));
    assert!(r.max_errors.iter().all(|v| *v <= 0.001));
    times.sort_by(f64::total_cmp);
    out.write_record([
        name.to_owned(),
        p.ncols().to_string(),
        r.segments.to_string(),
        r.refinements.to_string(),
        format!("{:.6}", times[1]),
        format!("{:.6}", times[0]),
        format!("{:.6}", times[2]),
        format!("{eval_ms:.6}"),
        format!("{:.12}", r.max_errors.iter().copied().fold(0., f64::max)),
        r.fitting_rows.to_string(),
        r.checked_intervals.to_string(),
    ])?;
    out.flush()?;
    println!(
        "{name:20} N={:7} spans={:5} build={:9.3} ms audit_max={:.8}",
        p.ncols(),
        r.segments,
        times[1],
        r.max_errors.iter().copied().fold(0., f64::max)
    );
    Ok(())
}

/// Draw either fitted paths or per-axis errors for a collection of input families.
fn draw<DB: DrawingBackend>(
    root: DrawingArea<DB, Shift>,
    plots: &[PlotCase],
    errors: bool,
) -> Result<(), Box<dyn Error>>
where
    DB::ErrorType: 'static,
{
    root.fill(&WHITE)?;
    let title = if errors {
        "Per-axis error against the input polyline (tolerance +/-0.001)"
    } else {
        "Rust Path smoothing: one constructor, default tolerance 0.001"
    };
    let root = root.titled(title, ("sans-serif", 28))?;
    for (area, p) in root
        .split_evenly((plots.len().div_ceil(2), 2))
        .into_iter()
        .zip(plots)
    {
        if errors {
            let mut chart = ChartBuilder::on(&area)
                .margin(12)
                .caption(&p.name, ("sans-serif", 20))
                .x_label_area_size(35)
                .y_label_area_size(75)
                .build_cartesian_2d(0_f64..1_f64, -0.0011..0.0011)?;
            chart
                .configure_mesh()
                .disable_mesh()
                .x_desc("s")
                .y_desc("axis error (input units)")
                .draw()?;
            for (a, color) in [BLUE, RED, GREEN].iter().enumerate() {
                chart.draw_series(LineSeries::new(
                    (0..p.errors.ncols())
                        .map(|i| (i as f64 / (p.errors.ncols() - 1) as f64, p.errors[(a, i)])),
                    color,
                ))?;
            }
            for y in [-0.001, 0.001] {
                chart.draw_series(LineSeries::new([(0., y), (1., y)], &BLACK.mix(0.3)))?;
            }
        } else {
            let mut x0 = f64::INFINITY;
            let mut x1 = f64::NEG_INFINITY;
            let mut y0 = f64::INFINITY;
            let mut y1 = f64::NEG_INFINITY;
            for v in [&p.raw, &p.curve] {
                for i in 0..v.ncols() {
                    x0 = x0.min(v[(0, i)]);
                    x1 = x1.max(v[(0, i)]);
                    y0 = y0.min(v[(1, i)]);
                    y1 = y1.max(v[(1, i)]);
                }
            }
            let px = ((x1 - x0) * 0.04).max(0.001);
            let py = ((y1 - y0) * 0.08).max(0.001);
            let mut chart = ChartBuilder::on(&area)
                .margin(12)
                .caption(
                    format!(
                        "{}: {} points -> {} spans",
                        p.name,
                        p.raw.ncols(),
                        p.segments
                    ),
                    ("sans-serif", 20),
                )
                .x_label_area_size(35)
                .y_label_area_size(65)
                .build_cartesian_2d(x0 - px..x1 + px, y0 - py..y1 + py)?;
            chart
                .configure_mesh()
                .disable_mesh()
                .x_desc("X (input units)")
                .y_desc("Y (input units)")
                .draw()?;
            chart
                .draw_series(LineSeries::new(
                    (0..p.raw.ncols()).map(|i| (p.raw[(0, i)], p.raw[(1, i)])),
                    &RGBColor(155, 160, 170),
                ))?
                .label("input")
                .legend(|(x, y)| PathElement::new([(x, y), (x + 18, y)], RGBColor(155, 160, 170)));
            chart
                .draw_series(LineSeries::new(
                    (0..p.curve.ncols()).map(|i| (p.curve[(0, i)], p.curve[(1, i)])),
                    BLUE.stroke_width(2),
                ))?
                .label("C4 quintic")
                .legend(|(x, y)| PathElement::new([(x, y), (x + 18, y)], BLUE));
            chart
                .configure_series_labels()
                .background_style(WHITE.mix(0.8))
                .label_font(("sans-serif", 14))
                .draw()?;
        }
    }
    root.present()?;
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let out = PathBuf::from(
        std::env::args()
            .nth(1)
            .unwrap_or_else(|| "data/path_smoothing".into()),
    );
    fs::create_dir_all(&out)?;
    if !std::env::args().any(|a| a == "--plots-only") {
        let mut csv = csv::Writer::from_path(out.join("benchmark.csv"))?;
        csv.write_record([
            "case",
            "n",
            "segments",
            "refinements",
            "build_median_ms",
            "build_min_ms",
            "build_max_ms",
            "eval_10001_up_to_3rd_ms",
            "max_error_bound",
            "fitting_rows",
            "checked_intervals",
        ])?;
        for n in 2..=5 {
            let p = DMatrix::from_fn(3, n, |a, i| match a {
                0 => (i / 2) as f64,
                1 => i.div_ceil(2) as f64,
                _ => 0.,
            });
            let s: Vec<_> = (0..n).map(|i| i as f64 / (n - 1) as f64).collect();
            measure("short_corner", &p, &s, &mut csv)?;
        }
        for raster in [true, false] {
            let (p, s) = cases::sparse(raster);
            measure(
                if raster {
                    "sparse_raster"
                } else {
                    "sparse_stair"
                },
                &p,
                &s,
                &mut csv,
            )?;
        }
        for n in [64, 1000, 10000, 100000, 300000] {
            for name in cases::FAMILIES {
                let (p, s, _) = cases::case(name, n);
                measure(name, &p, &s, &mut csv)?;
            }
        }
    }
    let mut plots = Vec::new();
    for name in cases::FAMILIES {
        let (raw, s, _) = cases::case(name, 10000);
        let path = Path::from_waypoints_fitting(
            &raw,
            SmoothingConfig {
                parameters: Some(s.clone()),
                ..Default::default()
            },
        )?;
        let samples: Vec<_> = (0..20001).map(|i| i as f64 / 20000.).collect();
        let curve = path.evaluate_q(&samples)?.q;
        let mut errors = curve.clone();
        let mut j = 0;
        for (i, &x) in samples.iter().enumerate() {
            while j + 2 < s.len() && s[j + 1] < x {
                j += 1;
            }
            let f = (x - s[j]) / (s[j + 1] - s[j]);
            for a in 0..3 {
                errors[(a, i)] -= raw[(a, j)] + f * (raw[(a, j + 1)] - raw[(a, j)]);
            }
        }
        // Export raw inputs and dense Rust outputs for independent numerical audits.
        let mut original = csv::Writer::from_path(out.join(format!("{name}_input.csv")))?;
        original.write_record(["s", "x", "y", "z"])?;
        for (i, x) in s.iter().enumerate() {
            original.write_record([
                x.to_string(),
                raw[(0, i)].to_string(),
                raw[(1, i)].to_string(),
                raw[(2, i)].to_string(),
            ])?;
        }
        original.flush()?;
        let mut sampled = csv::Writer::from_path(out.join(format!("{name}_curve.csv")))?;
        sampled.write_record(["s", "x", "y", "z"])?;
        for (i, x) in samples.iter().enumerate() {
            sampled.write_record([
                x.to_string(),
                curve[(0, i)].to_string(),
                curve[(1, i)].to_string(),
                curve[(2, i)].to_string(),
            ])?;
        }
        sampled.flush()?;
        let (lo, hi) = if name.contains("smooth") {
            (0.24, 0.25)
        } else if name.contains("micro") {
            (0.245, 0.255)
        } else if name.contains("raster") {
            ((10. - 0.04) / 209.5, (10. + 0.04) / 209.5)
        } else {
            ((1. - 0.04) / 24., (1. + 0.04) / 24.)
        };
        let detail_s: Vec<_> = (0..4001)
            .map(|i| lo + (hi - lo) * i as f64 / 4000.)
            .collect();
        let detailed = path.evaluate_up_to_3rd(&detail_s)?;
        let mut detail = csv::Writer::from_path(out.join(format!("{name}_detail.csv")))?;
        detail.write_record([
            "s", "raw_x", "raw_y", "raw_z", "x", "y", "z", "dx", "dy", "dz", "ddx", "ddy", "ddz",
            "dddx", "dddy", "dddz",
        ])?;
        let mut cursor = 0;
        for (i, x) in detail_s.iter().enumerate() {
            while cursor + 2 < s.len() && s[cursor + 1] < *x {
                cursor += 1;
            }
            let f = (x - s[cursor]) / (s[cursor + 1] - s[cursor]);
            let mut row = vec![x.to_string()];
            for a in 0..3 {
                row.push(
                    (raw[(a, cursor)] + f * (raw[(a, cursor + 1)] - raw[(a, cursor)])).to_string(),
                );
            }
            for matrix in [
                &detailed.q,
                detailed.dq.as_ref().unwrap(),
                detailed.ddq.as_ref().unwrap(),
                detailed.dddq.as_ref().unwrap(),
            ] {
                for a in 0..3 {
                    row.push(matrix[(a, i)].to_string());
                }
            }
            detail.write_record(row)?;
        }
        detail.flush()?;
        plots.push(PlotCase {
            name: name.into(),
            raw,
            curve,
            errors,
            segments: path.smoothing_report().unwrap().segments,
        });
    }
    for errors in [false, true] {
        let name = if errors { "errors" } else { "paths" };
        let svg = out.join(format!("{name}.svg"));
        draw(
            SVGBackend::new(&svg, (1800, 2000)).into_drawing_area(),
            &plots,
            errors,
        )?;
        let png = out.join(format!("{name}.png"));
        draw(
            BitMapBackend::new(&png, (1800, 2000)).into_drawing_area(),
            &plots,
            errors,
        )?;
    }
    let mut small = Vec::new();
    let mut inputs = Vec::new();
    for n in 2..=5 {
        let p = DMatrix::from_fn(3, n, |a, i| match a {
            0 => (i / 2) as f64,
            1 => i.div_ceil(2) as f64,
            _ => 0.,
        });
        let s: Vec<_> = (0..n).map(|i| i as f64 / (n - 1) as f64).collect();
        inputs.push((format!("{n}-point corners"), p, s));
    }
    for raster in [true, false] {
        let (p, s) = cases::sparse(raster);
        inputs.push((
            if raster {
                "sparse raster"
            } else {
                "sparse staircase"
            }
            .into(),
            p,
            s,
        ));
    }
    for (name, raw, s) in inputs {
        let path = Path::from_waypoints_fitting(
            &raw,
            SmoothingConfig {
                parameters: Some(s.clone()),
                ..Default::default()
            },
        )?;
        let sample: Vec<_> = (0..10001).map(|i| i as f64 / 10000.).collect();
        let curve = path.evaluate_q(&sample)?.q;
        let mut errors = curve.clone();
        let mut j = 0;
        for (i, &x) in sample.iter().enumerate() {
            while j + 2 < s.len() && s[j + 1] < x {
                j += 1;
            }
            let f = (x - s[j]) / (s[j + 1] - s[j]);
            for a in 0..3 {
                errors[(a, i)] -= raw[(a, j)] + f * (raw[(a, j + 1)] - raw[(a, j)]);
            }
        }
        small.push(PlotCase {
            name,
            raw,
            curve,
            errors,
            segments: path.smoothing_report().unwrap().segments,
        });
    }
    let png = out.join("small_paths.png");
    draw(
        BitMapBackend::new(&png, (1800, 1500)).into_drawing_area(),
        &small,
        false,
    )?;
    let svg = out.join("small_paths.svg");
    draw(
        SVGBackend::new(&svg, (1800, 1500)).into_drawing_area(),
        &small,
        false,
    )?;
    Ok(())
}
