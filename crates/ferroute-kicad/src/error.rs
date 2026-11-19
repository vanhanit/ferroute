// SPDX-License-Identifier: AGPL-3.0-only
//! Error type shared by every part of the crate.

/// Everything that can go wrong while reading or writing KiCad files.
#[derive(Debug, thiserror::Error)]
pub enum KicadError {
    /// Lexical / structural S-expression error at a 1-based line and column.
    #[error("parse error at {line}:{col}: {msg}")]
    Parse { line: usize, col: usize, msg: String },
    /// The S-expression is well formed but is not a usable KiCad board
    /// (missing sections, malformed numbers, dangling references, ...).
    #[error("invalid KiCad board: {0}")]
    Format(String),
    /// File-system error.
    #[error(transparent)]
    Io(#[from] std::io::Error),
    /// The `.kicad_pro` project file is not valid JSON or has unusable values.
    #[error("invalid project file: {0}")]
    Json(String),
}
