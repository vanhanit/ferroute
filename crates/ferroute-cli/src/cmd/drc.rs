// SPDX-License-Identifier: AGPL-3.0-only
//! `ferroute drc`.

use super::{item_label, load, out};
use anyhow::Result;
use ferroute_drc::{DrcConfig, check, summary};
use std::path::PathBuf;
use std::process::ExitCode;

#[derive(clap::Args, Debug)]
pub struct DrcArgs {
    /// Board to check (.kicad_pcb).
    pub board: PathBuf,
    /// Write the full report as JSON.
    #[arg(long)]
    pub json: Option<PathBuf>,
    /// Exit with status 1 on any violation.
    #[arg(long)]
    pub fail_on_violation: bool,
    /// Also exit with status 1 when connections are unrouted.
    #[arg(long)]
    pub fail_on_unconnected: bool,
    /// Include pad-to-pad clearance inside a footprint (library geometry).
    #[arg(long)]
    pub intra_footprint: bool,
}

/// Executes `ferroute drc`.
pub fn run(a: &DrcArgs) -> Result<ExitCode> {
    let kb = load(&a.board)?;
    let rep = check(
        &kb.board,
        &DrcConfig { skip_intra_footprint_pads: !a.intra_footprint, ..DrcConfig::default() },
    );
    let mut kinds: Vec<_> = summary(&rep).into_iter().collect();
    kinds.sort();
    out(&format!(
        "{}: {} violation(s), {} unrouted connection(s), completion {:.1}%",
        a.board.display(),
        rep.violations.len(),
        rep.connectivity.connections_missing,
        rep.connectivity.completion() * 100.0
    ));
    for (k, n) in kinds {
        out(&format!("  {k:?}: {n}"));
    }
    for v in rep.violations.iter().take(50) {
        out(&format!(
            "  - {:?} {} ↔ {} at ({:.3}, {:.3}) mm: {:.4} mm < {:.4} mm",
            v.kind,
            item_label(&kb.board, v.a),
            v.b.map_or_else(String::new, |b| item_label(&kb.board, b)),
            ferroute_geom::units::nm_to_mm(v.at.x),
            ferroute_geom::units::nm_to_mm(v.at.y),
            v.actual / 1e6,
            ferroute_geom::units::nm_to_mm(v.required)
        ));
    }
    if let Some(p) = &a.json {
        std::fs::write(p, serde_json::to_string_pretty(&rep)?)?;
    }
    let fail = (a.fail_on_violation && !rep.violations.is_empty())
        || (a.fail_on_unconnected && rep.connectivity.connections_missing > 0);
    Ok(if fail { ExitCode::from(1) } else { ExitCode::SUCCESS })
}
