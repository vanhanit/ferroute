// SPDX-License-Identifier: AGPL-3.0-only
//! `ferroute gen` and `ferroute bench`.

use super::{load, out, render_opts};
use anyhow::{Context, Result};
use ferroute_engine::score::{Baseline, BenchRecord, Tolerances, compare, markdown};
use ferroute_engine::{PipelineConfig, run as run_pipeline, synth};
use ferroute_place::{MovePolicy, PlaceConfig};
use std::path::PathBuf;
use std::process::ExitCode;

#[derive(clap::Args, Debug)]
pub struct GenArgs {
    /// Synthetic suite to generate (`ci`, `full`, `dense`, or `fixtures` for the hand-made testdata boards).
    #[arg(long, default_value = "ci")]
    pub suite: String,
    /// Output directory.
    #[arg(short, long)]
    pub out: PathBuf,
}

/// Executes `ferroute gen`.
pub fn gen_run(a: &GenArgs) -> Result<ExitCode> {
    std::fs::create_dir_all(&a.out)?;
    if a.suite == "fixtures" {
        let fixtures = [
            ("bus_match", synth::bus_match()),
            ("diff_pair", synth::diff_pair()),
            ("bga_planes", synth::bga_planes()),
        ];
        for (name, board) in fixtures {
            let pcb = a.out.join(format!("{name}.kicad_pcb"));
            std::fs::write(&pcb, ferroute_kicad::write_new_board(&board, 1))?;
            std::fs::write(
                pcb.with_extension("kicad_pro"),
                ferroute_kicad::project::write_project_json(&board),
            )?;
            out(&format!("wrote {}", pcb.display()));
        }
        return Ok(ExitCode::SUCCESS);
    }
    for sb in synth::suite_boards(&a.suite) {
        let pcb = a.out.join(format!("{}.kicad_pcb", sb.name));
        std::fs::write(&pcb, ferroute_kicad::write_new_board(&sb.board, sb.seed))?;
        std::fs::write(
            pcb.with_extension("kicad_pro"),
            ferroute_kicad::project::write_project_json(&sb.board),
        )?;
        if let Some(rules) = &sb.rules_toml {
            std::fs::write(pcb.with_extension("toml"), rules)?;
        }
        out(&format!("wrote {}", pcb.display()));
    }
    Ok(ExitCode::SUCCESS)
}

#[derive(clap::Args, Debug)]
pub struct BenchArgs {
    /// Synthetic suite (`ci`, `full` or `dense`; `dense` skips the fixtures and the gate).
    #[arg(long, default_value = "ci")]
    pub suite: String,
    /// Also benchmark every .kicad_pcb in this directory.
    #[arg(long, default_value = "testdata/boards")]
    pub fixtures: PathBuf,
    /// Baseline to gate against.
    #[arg(long)]
    pub baseline: Option<PathBuf>,
    /// Rewrite the baseline with the current results instead of gating.
    #[arg(long)]
    pub update_baseline: bool,
    /// Output directory for per-board reports and SVGs.
    #[arg(long, default_value = "out/bench")]
    pub out: PathBuf,
    #[arg(long, default_value_t = 1)]
    pub seed: u64,
    /// Run only the synthetic boards whose name contains this text (the fixtures are
    /// skipped).
    #[arg(long)]
    pub only: Option<String>,
    /// PathFinder iterations per route (default: the router's own; lower it for the
    /// largest dense boards).
    #[arg(long)]
    pub iterations: Option<u32>,
    /// Nudge → re-route rounds for failed nets (default: the pipeline's own).
    #[arg(long)]
    pub lns_rounds: Option<u32>,
    /// Also write each routed board as `<name>.kicad_pcb` into `--out` (byte-identity checks).
    #[arg(long)]
    pub write_boards: bool,
}

/// Executes `ferroute bench`.
pub fn run(a: &BenchArgs) -> Result<ExitCode> {
    std::fs::create_dir_all(&a.out)?;
    let mut boards = Vec::new();
    for sb in synth::suite_boards(&a.suite) {
        boards.push((sb.name, sb.board));
    }
    if let Some(only) = &a.only {
        boards.retain(|(name, _)| name.contains(only.as_str()));
    }
    if a.suite != "dense" && a.only.is_none() && a.fixtures.is_dir() {
        let mut files: Vec<_> = std::fs::read_dir(&a.fixtures)?
            .filter_map(Result::ok)
            .map(|e| e.path())
            .filter(|p| p.extension().is_some_and(|x| x == "kicad_pcb"))
            .collect();
        files.sort();
        for f in files {
            let name = f.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
            let mut board = load(&f)?.board;
            // Fixtures whose rules file declares differential pairs run with it (pairs
            // exist only through rules); every other fixture runs rule-free as before.
            let rules = a.fixtures.with_file_name("rules").join(format!("{name}.toml"));
            if let Ok(text) = std::fs::read_to_string(&rules) {
                let rs = super::route::parse_rules(&text)?;
                if !rs.pairs.is_empty() {
                    rs.apply(&mut board)?;
                }
            }
            // Fixtures run twice: with nudging and with placement fixed (what CI's KiCad
            // lane does), so a completion drop in either mode fails the gate.
            boards.push((format!("{name}-fixed"), board.clone()));
            boards.push((name, board));
        }
    }
    let mut cfg = PipelineConfig {
        seed: a.seed,
        nudge: Some((
            MovePolicy::default(),
            PlaceConfig { seed: a.seed, ..PlaceConfig::default() },
        )),
        ..PipelineConfig::default()
    };
    if let Some(n) = a.iterations {
        cfg.router.max_iterations = n;
    }
    if let Some(n) = a.lns_rounds {
        cfg.lns_rounds = n;
    }
    let mut records = Vec::new();
    let fixed_cfg = PipelineConfig { nudge: None, ..cfg.clone() };
    for (name, board) in &boards {
        let c = if name.ends_with("-fixed") { &fixed_cfg } else { &cfg };
        let res = run_pipeline(board, c).with_context(|| format!("benchmark {name}"))?;
        let title = format!("{name} — ferroute {} (seed {})", env!("CARGO_PKG_VERSION"), a.seed);
        ferroute_render::write_svg(
            &a.out.join(format!("{name}.svg")),
            &res.board_after,
            &render_opts(&res.board_after, &res.drc, title),
        )?;
        if a.write_boards {
            std::fs::write(
                a.out.join(format!("{name}.kicad_pcb")),
                ferroute_kicad::write_new_board(&res.board_after, a.seed),
            )?;
        }
        if a.suite == "dense" {
            out(&dense_detail(name, &res));
        }
        let rec = BenchRecord { board: name.clone(), seed: a.seed, metrics: res.metrics };
        std::fs::write(a.out.join(format!("{name}.json")), serde_json::to_string_pretty(&rec)?)?;
        records.push(rec);
    }
    let md = markdown(&records);
    std::fs::write(a.out.join("summary.md"), &md)?;
    std::fs::write(a.out.join("records.json"), serde_json::to_string_pretty(&records)?)?;
    out(&md);
    let Some(path) = &a.baseline else { return Ok(ExitCode::SUCCESS) };
    if a.update_baseline {
        let mut base = Baseline {
            ferroute_version: env!("CARGO_PKG_VERSION").into(),
            records: records.clone(),
        };
        for r in &mut base.records {
            // Runtime is machine-dependent; store a rounded, informational value.
            r.metrics.runtime_ms = (r.metrics.runtime_ms / 100.0).ceil() * 100.0;
        }
        if let Some(d) = path.parent() {
            std::fs::create_dir_all(d)?;
        }
        std::fs::write(path, serde_json::to_string_pretty(&base)? + "\n")?;
        out(&format!("baseline updated: {}", path.display()));
        return Ok(ExitCode::SUCCESS);
    }
    let base: Baseline = serde_json::from_str(
        &std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?,
    )?;
    let regs = compare(&records, &base, &Tolerances::default());
    if regs.is_empty() {
        out("benchmark gate: PASS");
        Ok(ExitCode::SUCCESS)
    } else {
        for r in &regs {
            out(&format!("REGRESSION {}: {} {} → {}", r.board, r.metric, r.baseline, r.current));
        }
        Ok(ExitCode::from(1))
    }
}

/// One-board diagnostic for the dense suite: where the router fails.
fn dense_detail(name: &str, res: &ferroute_engine::PipelineResult) -> String {
    let b = &res.board_after;
    let c = &res.drc.connectivity;
    let mut s = format!(
        "{name}: connections {}/{} missing {}, nets failed {}, gate rips {}, introduced {}, constraint issues {}\n",
        c.connections_total - c.connections_missing,
        c.connections_total,
        c.connections_missing,
        res.failed_nets.len(),
        res.route.drc_gate_rips,
        res.metrics.introduced_violations,
        res.metrics.constraint_issues,
    );
    let names: Vec<&str> =
        res.failed_nets.iter().take(40).map(|n| b.nets[n.idx()].name.as_str()).collect();
    s.push_str(&format!("  failed nets: {}\n", names.join(" ")));
    s
}
