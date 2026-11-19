// SPDX-License-Identifier: AGPL-3.0-only
//! Benchmark records, baselines and the regression gate.
//!
//! Every algorithmic change is scored on (a) DRC pass rate, (b) completion, (c) total
//! wirelength, (d) via count, (e) component displacement and (f) runtime. The gate
//! fails on any DRC regression, any completion drop, or quality regressions beyond the
//! configured tolerances. Runtime is reported but only gated with a generous factor
//! because CI machines vary.

use crate::Metrics;
use serde::{Deserialize, Serialize};

/// Result of one benchmark board.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BenchRecord {
    pub board: String,
    pub seed: u64,
    pub metrics: Metrics,
}

/// A stored baseline (committed as `bench/baseline.json`).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Baseline {
    pub ferroute_version: String,
    pub records: Vec<BenchRecord>,
}

/// Allowed regressions relative to the baseline.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Tolerances {
    /// Maximum allowed absolute completion drop (0 = none).
    pub completion_drop: f64,
    /// Relative wirelength increase.
    pub wirelength_rel: f64,
    /// Relative + absolute via increase.
    pub vias_rel: f64,
    pub vias_abs: usize,
    pub displacement_rel: f64,
    /// Runtime factor (informational unless exceeded by a wide margin).
    pub runtime_factor: f64,
}

impl Default for Tolerances {
    fn default() -> Self {
        Self {
            completion_drop: 1e-9,
            wirelength_rel: 0.03,
            vias_rel: 0.05,
            vias_abs: 2,
            displacement_rel: 0.15,
            runtime_factor: 5.0,
        }
    }
}

/// A detected regression.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Regression {
    pub board: String,
    pub metric: String,
    pub baseline: f64,
    pub current: f64,
}

/// Compares `current` with `baseline`. Boards missing from the baseline are not
/// regressions (new benchmarks); DRC failures are always regressions.
#[must_use]
pub fn compare(current: &[BenchRecord], baseline: &Baseline, tol: &Tolerances) -> Vec<Regression> {
    let mut out = Vec::new();
    for rec in current {
        let m = &rec.metrics;
        if !m.drc_clean {
            out.push(Regression {
                board: rec.board.clone(),
                metric: "drc_clean".into(),
                baseline: 1.0,
                current: 0.0,
            });
        }
        let Some(base) = baseline.records.iter().find(|b| b.board == rec.board) else { continue };
        let b = &base.metrics;
        let mut push = |metric: &str, bv: f64, cv: f64| {
            out.push(Regression {
                board: rec.board.clone(),
                metric: metric.into(),
                baseline: bv,
                current: cv,
            });
        };
        if m.completion + tol.completion_drop < b.completion {
            push("completion", b.completion, m.completion);
        }
        if m.wirelength_mm > b.wirelength_mm * (1.0 + tol.wirelength_rel) + 1e-9 {
            push("wirelength_mm", b.wirelength_mm, m.wirelength_mm);
        }
        #[allow(clippy::cast_precision_loss)]
        if m.vias as f64 > b.vias as f64 * (1.0 + tol.vias_rel) + tol.vias_abs as f64 {
            #[allow(clippy::cast_precision_loss)]
            push("vias", b.vias as f64, m.vias as f64);
        }
        if m.displacement_mm > b.displacement_mm * (1.0 + tol.displacement_rel) + 0.5 {
            push("displacement_mm", b.displacement_mm, m.displacement_mm);
        }
        if b.runtime_ms > 0.0
            && m.runtime_ms > (b.runtime_ms * tol.runtime_factor).max(b.runtime_ms + 2_000.0)
        {
            push("runtime_ms", b.runtime_ms, m.runtime_ms);
        }
    }
    out
}

/// Markdown summary table (used in CI job summaries).
#[must_use]
pub fn markdown(records: &[BenchRecord]) -> String {
    let mut s = String::from(
        "| board | DRC | completion | wirelength (mm) | vias | moved | displacement (mm) | runtime (ms) |\n|---|---|---|---|---|---|---|---|\n",
    );
    for r in records {
        let m = &r.metrics;
        s.push_str(&format!(
            "| {} | {} | {:.1}% | {:.1} | {} | {} | {:.2} | {:.0} |\n",
            r.board,
            if m.drc_clean { "✅" } else { "❌" },
            m.completion * 100.0,
            m.wirelength_mm,
            m.vias,
            m.moved_footprints,
            m.displacement_mm,
            m.runtime_ms
        ));
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rec(name: &str, completion: f64, wl: f64, vias: usize) -> BenchRecord {
        BenchRecord {
            board: name.into(),
            seed: 1,
            metrics: Metrics {
                drc_clean: true,
                completion,
                wirelength_mm: wl,
                vias,
                ..Metrics::default()
            },
        }
    }

    #[test]
    fn gate_detects_regressions() {
        let base =
            Baseline { ferroute_version: "0".into(), records: vec![rec("a", 1.0, 100.0, 10)] };
        assert!(compare(&[rec("a", 1.0, 102.0, 11)], &base, &Tolerances::default()).is_empty());
        let r = compare(&[rec("a", 0.9, 110.0, 20)], &base, &Tolerances::default());
        let metrics: Vec<_> = r.iter().map(|x| x.metric.as_str()).collect();
        assert_eq!(metrics, vec!["completion", "wirelength_mm", "vias"]);
        let mut bad = rec("new", 1.0, 1.0, 0);
        bad.metrics.drc_clean = false;
        assert_eq!(compare(&[bad], &base, &Tolerances::default()).len(), 1);
    }
}
