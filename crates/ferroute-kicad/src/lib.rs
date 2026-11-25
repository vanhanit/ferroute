// SPDX-License-Identifier: AGPL-3.0-only
//! # ferroute-kicad
//!
//! Lossless KiCad `.kicad_pcb` reader/writer for ferroute.
//!
//! * [`sexpr`]: lossless S-expression tree and a KiCad-style pretty printer.
//! * [`read_board`] / [`read_board_file`]: tree to [`ferroute_board::Board`] plus a
//!   [`KicadDocument`] that remembers the tree.
//! * [`KicadDocument::apply_edit`]: applies a [`ferroute_board::BoardEdit`] by touching only
//!   the footprints that moved and appending new tracks/vias.
//! * [`write_new_board`]: complete KiCad 10 file from scratch.
//! * [`project`]: net classes and rules from the sibling `.kicad_pro`.
//!
//! ## Facts about the format (verified against KiCad 10 sources)
//!
//! * KiCad 10 writes `(version 20260206)` and stopped writing net codes at file version
//!   `20251028`: pads, tracks, vias and zones carry `(net "NAME")` and there is no top-level
//!   net table. Older files have `(net N "NAME")` in pads, `(net N)` on tracks and a
//!   `(net N "NAME")` table. The parser decides per token (number vs string), not per
//!   version; this crate does the same and remembers which dialect a document uses.
//! * A pad's `(at x y a)`: `x y` are relative to the footprint anchor in the footprint's
//!   *unrotated* frame, `a` is the pad's *absolute* angle. Footprint graphics, courtyards and
//!   text positions are footprint-local and unrotated; text angles are absolute.
//! * Rotating a footprint adds the same delta to the angle of every pad and text and leaves
//!   all local coordinates unchanged. Footprint, pad and text angles are written normalised
//!   to `(-180, 180]`; footprint and pad angles are omitted when zero.

pub mod dru;
pub mod project;
pub mod sexpr;

mod doc;
mod error;
mod layers;
mod outline;
mod pad;
mod read;
mod teardrop;
mod util;
mod write;

pub use doc::{KicadDocument, Removal};
pub use error::KicadError;
pub use write::{KICAD10_VERSION, write_new_board};

use ferroute_board::Board;
use std::path::Path;

/// Options for [`read_board`].
#[derive(Clone, Debug, Default)]
pub struct ReadOptions {
    /// Contents of the sibling `.kicad_pro`, if any. Without it the board gets KiCad's
    /// default design rules.
    pub project_json: Option<String>,
    /// Contents of the sibling `.kicad_dru` (custom design rules), if any.
    pub dru: Option<String>,
}

/// A board read from a KiCad file.
#[derive(Debug)]
pub struct KicadBoard {
    /// The transport-agnostic model.
    pub board: Board,
    /// The lossless tree, used to write edits back.
    pub doc: KicadDocument,
    /// Things that were ignored or approximated (copper pours, per-layer padstacks, ...).
    pub warnings: Vec<String>,
    /// Warnings of the custom-rules file (unsupported conditions / constraints).
    pub rules_warnings: Vec<String>,
}

/// Parses `.kicad_pcb` text into a [`KicadBoard`].
///
/// # Errors
/// [`KicadError::Parse`] for lexical errors, [`KicadError::Format`] when the file is not a
/// usable board (no copper layers, no closed Edge.Cuts outline, invalid numbers, ...),
/// [`KicadError::Json`] for a malformed project file.
pub fn read_board(src: &str, opts: &ReadOptions) -> Result<KicadBoard, KicadError> {
    let root = sexpr::parse(src)?;
    let parsed = read::parse_board(&root)?;
    let mut board = parsed.board;
    let mut warnings = parsed.warnings;
    match &opts.project_json {
        Some(json) => project::read_project_rules(json, &mut board)?,
        None => warnings.push("no .kicad_pro supplied: using KiCad default design rules".into()),
    }
    let mut rules_warnings = Vec::new();
    if let Some(dru) = &opts.dru {
        let parsed = dru::parse_dru(dru)?;
        board.rules.custom = parsed.rules;
        rules_warnings = parsed.warnings;
    }
    let doc = KicadDocument::new(
        root,
        parsed.version,
        parsed.dialect,
        doc::SourceNodes {
            footprints: parsed.fp_nodes,
            tracks: parsed.track_nodes,
            vias: parsed.via_nodes,
        },
    );
    Ok(KicadBoard { board, doc, warnings, rules_warnings })
}

/// Reads a `.kicad_pcb` file and its sibling `.kicad_pro` (same stem) when present.
///
/// # Errors
/// As [`read_board`], plus [`KicadError::Io`].
pub fn read_board_file(path: &Path) -> Result<KicadBoard, KicadError> {
    let src = std::fs::read_to_string(path)?;
    let pro = path.with_extension("kicad_pro");
    let read_opt = |p: &Path| match std::fs::read_to_string(p) {
        Ok(s) => Ok(Some(s)),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(e) => Err(KicadError::from(e)),
    };
    let project_json = read_opt(&pro)?;
    let dru = read_opt(&path.with_extension("kicad_dru"))?;
    read_board(&src, &ReadOptions { project_json, dru })
}
