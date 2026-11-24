// SPDX-License-Identifier: AGPL-3.0-only
//! Error type.

/// Errors from DSN export and SES import.
#[derive(Debug, thiserror::Error)]
pub enum DsnError {
    /// Writing the output failed.
    #[error("i/o error: {0}")]
    Io(#[from] std::io::Error),
    /// The S-expression text is malformed.
    #[error("syntax error: {0}")]
    Syntax(String),
    /// The SES is well-formed but unusable for this board (unknown net/layer, bad number).
    #[error("session error: {0}")]
    Session(String),
    /// A name cannot be written in DSN (contains the quote character).
    #[error("cannot represent {0:?} in a DSN file")]
    Unrepresentable(String),
}
