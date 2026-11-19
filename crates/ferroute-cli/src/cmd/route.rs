// SPDX-License-Identifier: AGPL-3.0-only
//! `ferroute route`.

use super::{load, out, render_opts};
use crate::length::parse_length;
use anyhow::{Context, Result, bail};
use ferroute_board::LayerSet;
use ferroute_engine::{PipelineConfig, run as run_pipeline};
use ferroute_geom::Coord;
use ferroute_place::{MovePolicy, PlaceConfig};
use ferroute_router::{Accel, RouterConfig};
use serde_json::json;
use std::path::PathBuf;
use std::process::ExitCode;

#[derive(clap::ValueEnum, Clone, Copy, Debug, PartialEq, Eq)]
pub enum AccelArg {
    Auto,
    Cpu,
    Gpu,
    None,
}

impl From<AccelArg> for Accel {
    fn from(a: AccelArg) -> Self {
        match a {
            AccelArg::Auto => Self::Auto,
            AccelArg::Cpu => Self::Cpu,
            AccelArg::Gpu => Self::Gpu,
            AccelArg::None => Self::None,
        }
    }
}

#[derive(clap::Args, Debug, Clone)]
pub struct RouteArgs {
    /// Input board (.kicad_pcb). A sibling .kicad_pro supplies net classes / rules.
    pub input: PathBuf,
    /// Output board path (may equal the input for in-place routing).
    #[arg(short, long)]
    pub output: PathBuf,
    /// Seed for every stochastic decision (replays are bit-identical).
    #[arg(long, default_value_t = 1)]
    pub seed: u64,
    /// Allow nudging unlocked footprints that match --movable.
    #[arg(long)]
    pub allow_move: bool,
    /// Reference globs of movable footprints (default: R*,C*,L*,FB*).
    #[arg(long, value_delimiter = ',')]
    pub movable: Vec<String>,
    /// Reference globs that must never move.
    #[arg(long, value_delimiter = ',')]
    pub exclude: Vec<String>,
    /// Maximum displacement of a movable footprint.
    #[arg(long, default_value = "2mm", value_parser = parse_length)]
    pub max_move: Coord,
    /// Placement grid for nudged footprints: moved anchors land on multiples of this step.
    #[arg(long, default_value = "0.1mm", value_parser = parse_length)]
    pub move_grid: Coord,
    /// Forbid 90° rotations of movable footprints.
    #[arg(long)]
    pub no_rotate: bool,
    /// Routing lattice pitch (default: default-class track width + clearance).
    #[arg(long, value_parser = parse_length)]
    pub pitch: Option<Coord>,
    /// Restrict routing to these copper layers (e.g. F.Cu,B.Cu).
    #[arg(long, value_delimiter = ',')]
    pub layers: Vec<String>,
    /// Clearance-field accelerator.
    #[arg(long, value_enum, default_value_t = AccelArg::Auto)]
    pub accel: AccelArg,
    /// PathFinder negotiation iterations.
    #[arg(long, default_value_t = 50)]
    pub iterations: u32,
    /// Nudge → re-route rounds for nets that failed.
    #[arg(long, default_value_t = 2)]
    pub lns_rounds: u32,
    /// Write a JSON report.
    #[arg(long)]
    pub report: Option<PathBuf>,
    /// Write an SVG rendering of the result.
    #[arg(long)]
    pub svg: Option<PathBuf>,
    /// Exit with status 1 if any connection remains unrouted.
    #[arg(long)]
    pub fail_on_incomplete: bool,
}

/// Executes `ferroute route`.
pub fn run(a: &RouteArgs) -> Result<ExitCode> {
    let kb = load(&a.input)?;
    let mut doc = kb.doc;
    let board = kb.board;
    let layers = if a.layers.is_empty() {
        None
    } else {
        let mut s = LayerSet::EMPTY;
        for name in &a.layers {
            let Some(i) = board.copper_layers.iter().position(|l| &l.name == name) else {
                bail!("unknown copper layer {name:?}");
            };
            s = s.with(ferroute_board::LayerId(u8::try_from(i)?));
        }
        Some(s)
    };
    let policy = MovePolicy {
        include: if a.movable.is_empty() {
            MovePolicy::default().include
        } else {
            a.movable.clone()
        },
        exclude: a.exclude.clone(),
        max_displacement: a.max_move,
        allow_rotation: !a.no_rotate,
    };
    let cfg = PipelineConfig {
        seed: a.seed,
        router: RouterConfig {
            pitch: a.pitch,
            layers,
            max_iterations: a.iterations,
            accel: a.accel.into(),
            ..RouterConfig::default()
        },
        nudge: a.allow_move.then(|| {
            (
                policy,
                PlaceConfig { seed: a.seed, grid: a.move_grid.max(1), ..PlaceConfig::default() },
            )
        }),
        lns_rounds: a.lns_rounds,
        ..PipelineConfig::default()
    };
    let res = run_pipeline(&board, &cfg).context("routing")?;
    if !res.metrics.drc_clean {
        bail!(
            "internal error: DRC gate reported {} introduced violations; refusing to write output",
            res.metrics.introduced_violations
        );
    }
    doc.apply_edit(&res.policy_board, &res.edit, a.seed)
        .context("applying edit to the KiCad document")?;
    if a.output == a.input {
        let bak = a.input.with_extension("kicad_pcb.ferroute-bak");
        std::fs::copy(&a.input, &bak)
            .with_context(|| format!("backing up to {}", bak.display()))?;
    }
    if let Some(dir) = a.output.parent()
        && !dir.as_os_str().is_empty()
    {
        std::fs::create_dir_all(dir)?;
    }
    std::fs::write(&a.output, doc.to_kicad_string())
        .with_context(|| format!("writing {}", a.output.display()))?;
    // Keep the rules next to the output so KiCad / kicad-cli DRC see the same classes.
    let pro_in = a.input.with_extension("kicad_pro");
    let pro_out = a.output.with_extension("kicad_pro");
    if pro_in.exists() && pro_in != pro_out {
        std::fs::copy(&pro_in, &pro_out)?;
    }
    let m = &res.metrics;
    let failed: Vec<&str> =
        res.failed_nets.iter().map(|n| res.board_after.nets[n.idx()].name.as_str()).collect();
    out(&format!(
        "{}: completion {:.1}% ({}/{} connections), {} tracks, {} vias, wirelength {:.1} mm, {} part(s) moved ({:.2} mm), DRC clean, {:.0} ms [{}]",
        a.output.display(),
        m.completion * 100.0,
        m.connections_total - m.connections_missing,
        m.connections_total,
        res.edit.new_tracks.len(),
        m.vias,
        m.wirelength_mm,
        m.moved_footprints,
        m.displacement_mm,
        m.runtime_ms,
        res.route.accel_backend
    ));
    if !failed.is_empty() {
        out(&format!("unrouted nets: {}", failed.join(", ")));
    }
    if let Some(p) = &a.report {
        let report = json!({
            "ferroute_version": env!("CARGO_PKG_VERSION"),
            "input": a.input, "output": a.output, "seed": a.seed,
            "metrics": m, "route": res.route, "place": res.place, "failed_nets": failed,
            "lns_rounds_run": res.rounds,
            "drc": { "violations": res.drc.violations.len(), "connectivity": res.drc.connectivity },
        });
        std::fs::write(p, serde_json::to_string_pretty(&report)?)?;
    }
    if let Some(p) = &a.svg {
        let title = format!(
            "ferroute {} — {} (seed {})",
            env!("CARGO_PKG_VERSION"),
            a.input.display(),
            a.seed
        );
        ferroute_render::write_svg(
            p,
            &res.board_after,
            &render_opts(&res.board_after, &res.drc, title),
        )?;
    }
    Ok(if a.fail_on_incomplete && m.connections_missing > 0 {
        ExitCode::from(1)
    } else {
        ExitCode::SUCCESS
    })
}
