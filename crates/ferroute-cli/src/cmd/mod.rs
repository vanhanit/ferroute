// SPDX-License-Identifier: AGPL-3.0-only
//! Subcommand implementations.

pub mod bench;
pub mod drc;
pub mod ipc;
pub mod render;
pub mod route;

use anyhow::{Context, Result};
use ferroute_board::Board;
use ferroute_drc::{DrcReport, ItemRef};
use ferroute_kicad::{KicadBoard, read_board_file};
use ferroute_render::{Marker, RenderOptions};
use std::io::Write;
use std::path::Path;
use std::process::ExitCode;

/// Loads a board file (and its sibling `.kicad_pro`), printing reader warnings.
pub fn load(path: &Path) -> Result<KicadBoard> {
    let kb = read_board_file(path).with_context(|| format!("reading {}", path.display()))?;
    for w in &kb.warnings {
        tracing::warn!("{}: {w}", path.display());
    }
    for w in &kb.rules_warnings {
        eprintln!("rules warning: {w}");
    }
    Ok(kb)
}

/// Writes `text` to stdout (the CLI's only stdout channel).
pub fn out(text: &str) {
    let mut o = std::io::stdout().lock();
    let _ = writeln!(o, "{text}");
}

/// Human-readable label of a DRC item.
#[must_use]
pub fn item_label(board: &Board, it: ItemRef) -> String {
    match it {
        ItemRef::Pad(p) => {
            let pad = board.pad(p);
            format!("{}.{}", board.footprint(pad.footprint).reference, pad.number)
        }
        ItemRef::Track(t) => format!("track#{t}"),
        ItemRef::Arc(a) => format!("arc#{a}"),
        ItemRef::Via(v) => format!("via#{v}"),
        ItemRef::Footprint(f) => board.footprint(f).reference.clone(),
        ItemRef::Keepout(k) => format!("keepout#{k}"),
        ItemRef::BoardEdge => "board-edge".into(),
    }
}

/// Render options with DRC markers and ratsnest from a report.
#[must_use]
pub fn render_opts(board: &Board, drc: &DrcReport, title: String) -> RenderOptions {
    RenderOptions {
        markers: drc
            .violations
            .iter()
            .map(|v| Marker {
                at: v.at,
                label: format!(
                    "{:?}: {} ↔ {}",
                    v.kind,
                    item_label(board, v.a),
                    v.b.map_or_else(String::new, |b| item_label(board, b))
                ),
            })
            .collect(),
        ratsnest: drc.connectivity.unconnected.iter().map(|u| (u.a, u.b)).collect(),
        moves: board
            .footprints
            .iter()
            .filter(|f| f.placement != f.original)
            .map(|f| (f.original.pos, f.placement.pos))
            .collect(),
        title: Some(title),
        highlight_routed: true,
        ..RenderOptions::default()
    }
}

/// `ferroute info`.
pub fn info() -> Result<ExitCode> {
    out(&format!(
        "ferroute {} (AGPL-3.0-only; commercial licences: COMMERCIAL.md)",
        env!("CARGO_PKG_VERSION")
    ));
    out(&format!("cpu threads: {}", rayon::current_num_threads()));
    #[cfg(feature = "gpu")]
    out(&format!(
        "gpu feature: enabled; adapter: {}",
        ferroute_gpu::adapter_info().unwrap_or_else(|| "none found".into())
    ));
    #[cfg(not(feature = "gpu"))]
    out("gpu feature: disabled at build time (rebuild with --features gpu)");
    match ferroute_gpu::select_backend(ferroute_gpu::BackendPreference::Auto) {
        Ok(b) => out(&format!("accelerator (auto): {}", b.name())),
        Err(e) => out(&format!("accelerator (auto): unavailable: {e}")),
    }
    Ok(ExitCode::SUCCESS)
}
