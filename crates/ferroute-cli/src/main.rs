// SPDX-License-Identifier: AGPL-3.0-only
//! `ferroute` — placement-aware, GPU/multi-core accelerated PCB autorouter for KiCad.
//!
//! Copyright (C) 2026 Thomas Vanhaniemi and ferroute contributors.
//! Licensed under the GNU AGPL-3.0-only; commercial licences available (COMMERCIAL.md).

mod cmd;
mod length;

use clap::{Parser, Subcommand};
use std::process::ExitCode;

#[derive(Parser, Debug)]
#[command(name = "ferroute", version, about, long_about = None, propagate_version = true)]
struct Cli {
    /// Worker threads (default: all cores).
    #[arg(long, global = true)]
    threads: Option<usize>,
    /// Log level filter (e.g. `info`, `ferroute_router=debug`).
    #[arg(long, global = true, default_value = "warn")]
    log: String,
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand, Debug)]
enum Command {
    /// Route a .kicad_pcb (optionally nudging unlocked parts) and write the result.
    Route(cmd::route::RouteArgs),
    /// Run the deterministic DRC oracle on a board.
    Drc(cmd::drc::DrcArgs),
    /// Render a board to SVG.
    Render(cmd::render::RenderArgs),
    /// Generate synthetic benchmark boards (.kicad_pcb + .kicad_pro).
    Gen(cmd::bench::GenArgs),
    /// Run the benchmark suite and gate against a baseline.
    Bench(cmd::bench::BenchArgs),
    /// KiCad IPC API (live KiCad 10 GUI / KiCad 11 headless `kicad-cli api-server`).
    #[command(subcommand)]
    Ipc(cmd::ipc::IpcCommand),
    /// Specctra DSN export / SES import (Freerouting interchange).
    #[command(subcommand)]
    Dsn(cmd::dsn::DsnCommand),
    /// Show compute backends and build information.
    Info,
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    let filter = tracing_subscriber::EnvFilter::try_new(&cli.log)
        .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("warn"));
    tracing_subscriber::fmt().with_env_filter(filter).with_writer(std::io::stderr).init();
    if let Some(n) = cli.threads
        && let Err(e) = rayon::ThreadPoolBuilder::new().num_threads(n).build_global()
    {
        eprintln!("warning: could not size thread pool: {e}");
    }
    let res = match cli.command {
        Command::Route(a) => cmd::route::run(&a),
        Command::Drc(a) => cmd::drc::run(&a),
        Command::Render(a) => cmd::render::run(&a),
        Command::Gen(a) => cmd::bench::gen_run(&a),
        Command::Bench(a) => cmd::bench::run(&a),
        Command::Ipc(c) => cmd::ipc::run(&c),
        Command::Dsn(c) => cmd::dsn::run(&c),
        Command::Info => cmd::info(),
    };
    match res {
        Ok(code) => code,
        Err(e) => {
            eprintln!("error: {e:#}");
            ExitCode::from(2)
        }
    }
}
