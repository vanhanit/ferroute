// SPDX-License-Identifier: AGPL-3.0-only
//! Prints the Freerouting settings for a board, one argument per line.
//!
//! `cargo run -p ferroute-dsn --example freerouting_args -- board.kicad_pcb`

use std::io::Write as _;
use std::process::ExitCode;

fn main() -> ExitCode {
    let Some(path) = std::env::args().nth(1) else {
        eprintln!("usage: freerouting_args <board.kicad_pcb>");
        return ExitCode::from(2);
    };
    match ferroute_kicad::read_board_file(std::path::Path::new(&path)) {
        Ok(kb) => {
            let mut out = std::io::stdout().lock();
            for a in ferroute_dsn::freerouting_settings(&kb.board).args() {
                if writeln!(out, "{a}").is_err() {
                    return ExitCode::FAILURE;
                }
            }
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("{path}: {e}");
            ExitCode::FAILURE
        }
    }
}
