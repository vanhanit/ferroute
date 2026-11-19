// SPDX-License-Identifier: AGPL-3.0-only
//! `ferroute render`.

use super::{load, render_opts};
use anyhow::Result;
use ferroute_drc::{DrcConfig, check};
use std::path::PathBuf;
use std::process::ExitCode;

#[derive(clap::Args, Debug)]
pub struct RenderArgs {
    /// Board to render (.kicad_pcb).
    pub board: PathBuf,
    /// Output SVG.
    #[arg(short, long)]
    pub output: PathBuf,
    /// Pixels per millimetre.
    #[arg(long, default_value_t = 20.0)]
    pub scale: f64,
}

/// Executes `ferroute render` (with DRC markers and ratsnest overlays).
pub fn run(a: &RenderArgs) -> Result<ExitCode> {
    let kb = load(&a.board)?;
    let rep = check(&kb.board, &DrcConfig::default());
    let mut opts = render_opts(&kb.board, &rep, a.board.display().to_string());
    opts.px_per_mm = a.scale;
    ferroute_render::write_svg(&a.output, &kb.board, &opts)?;
    Ok(ExitCode::SUCCESS)
}
