// SPDX-License-Identifier: AGPL-3.0-only
//! `ferroute dsn`: Specctra DSN export and SES import (Freerouting interchange).

use super::{load, out};
use anyhow::{Context, Result};
use std::path::PathBuf;
use std::process::ExitCode;

#[derive(clap::Subcommand, Debug)]
pub enum DsnCommand {
    /// Export a .kicad_pcb as a Specctra DSN design file.
    Export {
        /// Board to export (.kicad_pcb).
        board: PathBuf,
        /// Output DSN file.
        #[arg(short, long)]
        output: PathBuf,
    },
    /// Apply a Specctra SES session (e.g. from Freerouting) to a .kicad_pcb.
    ImportSes {
        /// The board the DSN was exported from (.kicad_pcb).
        board: PathBuf,
        /// Specctra session file.
        ses: PathBuf,
        /// Output board.
        #[arg(short, long)]
        output: PathBuf,
        /// Seed (identifiers of the new copper are derived from it).
        #[arg(long, default_value_t = 0)]
        seed: u64,
    },
}

/// Executes a `ferroute dsn` subcommand.
pub fn run(c: &DsnCommand) -> Result<ExitCode> {
    match c {
        DsnCommand::Export { board, output } => {
            let kb = load(board)?;
            let mut buf = Vec::new();
            ferroute_dsn::write_dsn(&kb.board, &mut buf).context("writing DSN")?;
            std::fs::write(output, buf).with_context(|| format!("writing {}", output.display()))?;
            out(&format!("wrote {}", output.display()));
        }
        DsnCommand::ImportSes { board, ses, output, seed } => {
            let mut kb = load(board)?;
            let text = std::fs::read_to_string(ses)
                .with_context(|| format!("reading {}", ses.display()))?;
            let edit = ferroute_dsn::read_ses(&text, &kb.board).context("reading SES")?;
            kb.doc
                .apply_edit(&kb.board, &edit, *seed)
                .context("applying SES routes to the KiCad document")?;
            std::fs::write(output, kb.doc.to_kicad_string())
                .with_context(|| format!("writing {}", output.display()))?;
            out(&format!(
                "imported {} tracks and {} vias into {}",
                edit.new_tracks.len(),
                edit.new_vias.len(),
                output.display()
            ));
        }
    }
    Ok(ExitCode::SUCCESS)
}
