// SPDX-License-Identifier: AGPL-3.0-only
//! `ferroute ipc …` — KiCad IPC API integration.
//!
//! * `probe` — connect, negotiate capabilities (version *and* feature probes), list
//!   open documents. Works with the KiCad 10 GUI and the KiCad 11 headless server.
//! * `route` — KiCad 10 "file bridge" live mode: save the open board, route the file,
//!   then ask KiCad to revert (reload) it so the GUI shows the result. Domain logic is
//!   identical to `ferroute route`; only the transport differs.
//! * `export-svg` — KiCad 11+: plot the board through the IPC job API.

use super::out;
use super::route::{RouteArgs, run as route_run};
use anyhow::{Context, Result, bail};
use ferroute_ipc::proto::{BoardLayer, DocumentSpecifier, DocumentType};
use ferroute_ipc::{Endpoint, KiCadClient, NngIpcTransport, negotiate};
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::Duration;

const CLIENT: &str = "io.github.vanhanit.ferroute";

/// Printed before every file-bridge run: until native IPC commits land (task board #6),
/// the GUI board is saved, routed on disk and reloaded, which discards KiCad's undo
/// history and any edit made in the GUI while routing runs.
pub const FILE_BRIDGE_WARNING: &str = "warning: KiCad 10 file bridge — the open board is saved, \
routed on disk and reloaded (RevertDocument). Undo history is lost and any edit made in KiCad \
while routing runs is discarded. A backup is written next to the board (*.ferroute-bak).";

#[derive(clap::Subcommand, Debug)]
pub enum IpcCommand {
    /// Negotiate capabilities with a running KiCad (GUI or `kicad-cli api-server`).
    Probe {
        /// Socket (`ipc:///tmp/kicad/api.sock` or a path). Default: $KICAD_API_SOCKET or platform default.
        #[arg(long)]
        socket: Option<String>,
        /// Print JSON.
        #[arg(long)]
        json: bool,
        #[arg(long, default_value_t = 5000)]
        timeout_ms: u64,
    },
    /// Route the board open in KiCad (save → route → revert/reload).
    Route {
        #[arg(long)]
        socket: Option<String>,
        #[arg(long, default_value_t = 1)]
        seed: u64,
        #[arg(long)]
        allow_move: bool,
        /// Per-net electrical rules (TOML), as for `ferroute route --rules`.
        #[arg(long)]
        rules: Option<PathBuf>,
        /// Allow vias inside SMD pads (needs filled and capped vias: extra fabrication cost).
        #[arg(long)]
        allow_via_in_pad: bool,
    },
    /// KiCad 11+: export the open (or given) board to SVG via the IPC job API.
    ExportSvg {
        #[arg(long)]
        socket: Option<String>,
        /// Output directory.
        #[arg(long)]
        out: PathBuf,
        /// Board to open first (headless server only).
        #[arg(long)]
        board: Option<PathBuf>,
    },
}

fn endpoint(socket: Option<&String>) -> Result<Endpoint> {
    Ok(match socket {
        Some(s) => Endpoint::parse(s).context("parsing --socket")?,
        None => Endpoint::discover(),
    })
}

fn connect(socket: Option<&String>, timeout: Duration) -> Result<KiCadClient<NngIpcTransport>> {
    let ep = endpoint(socket)?;
    let t = NngIpcTransport::connect(&ep).with_context(|| format!("connecting to {}", ep.url))?;
    Ok(KiCadClient::new(t, CLIENT, ep.token).with_timeout(timeout))
}

fn board_path(doc: &DocumentSpecifier) -> Option<PathBuf> {
    let f = doc.board_filename()?;
    let p = Path::new(f);
    if p.is_absolute() {
        return Some(p.to_path_buf());
    }
    Some(doc.project.as_ref().map_or_else(|| p.to_path_buf(), |pr| Path::new(&pr.path).join(p)))
}

/// Executes `ferroute ipc …`.
pub fn run(c: &IpcCommand) -> Result<ExitCode> {
    match c {
        IpcCommand::Probe { socket, json, timeout_ms } => {
            let ep = endpoint(socket.as_ref())?;
            let rep =
                ferroute_ipc::caps::probe_endpoint(&ep, CLIENT, Duration::from_millis(*timeout_ms))
                    .with_context(|| format!("probing {}", ep.url))?;
            if *json {
                out(&serde_json::to_string_pretty(&rep)?);
            } else {
                let c = &rep.capabilities;
                out(&format!("{} → KiCad {} ({:?})", rep.endpoint, c.version.full, c.generation));
                out(&format!(
                    "  live GUI: {}  headless server: {}  job export: {}  document commands: {}",
                    c.live_gui, c.headless_server, c.jobs_export, c.document_commands
                ));
                for d in &rep.open_documents {
                    out(&format!("  open: {d}"));
                }
            }
            Ok(ExitCode::SUCCESS)
        }
        IpcCommand::Route { socket, seed, allow_move, rules, allow_via_in_pad } => {
            let mut client = connect(socket.as_ref(), Duration::from_secs(30))?;
            let caps = negotiate(&mut client)?;
            let docs = client.get_open_documents(DocumentType::Pcb)?;
            let Some(doc) = docs.first() else { bail!("no board is open in KiCad") };
            let Some(path) = board_path(doc) else { bail!("open document has no board file name") };
            tracing::info!(?caps, path = %path.display(), "routing open board via file bridge");
            out(FILE_BRIDGE_WARNING);
            client.save_document(doc).context("SaveDocument")?;
            let args = RouteArgs {
                input: path.clone(),
                output: path,
                seed: *seed,
                allow_move: *allow_move,
                movable: Vec::new(),
                exclude: Vec::new(),
                max_move: 2_000_000,
                move_grid: 100_000,
                no_rotate: false,
                pitch: None,
                layers: Vec::new(),
                accel: super::route::AccelArg::Auto,
                iterations: 50,
                lns_rounds: 2,
                report: None,
                svg: None,
                fail_on_incomplete: false,
                rules: rules.clone(),
                strict_rules: false,
                allow_via_in_pad: *allow_via_in_pad,
            };
            let code = route_run(&args)?;
            client.revert_document(doc).context("RevertDocument (reload)")?;
            Ok(code)
        }
        IpcCommand::ExportSvg { socket, out: dir, board } => {
            let mut client = connect(socket.as_ref(), Duration::from_secs(120))?;
            let caps = negotiate(&mut client)?;
            if !caps.jobs_export {
                bail!(
                    "KiCad {} does not support IPC plotting/export (needs KiCad 11+)",
                    caps.version.full
                );
            }
            let doc = match board {
                Some(b) if caps.document_commands => client.open_document(&b.to_string_lossy())?,
                _ => client
                    .get_open_documents(DocumentType::Pcb)?
                    .into_iter()
                    .next()
                    .context("no board open")?,
            };
            std::fs::create_dir_all(dir)?;
            let target = std::fs::canonicalize(dir)?.join("board.svg");
            let r = client.export_svg(
                &doc,
                &target.to_string_lossy(),
                &[BoardLayer::BL_F_Cu, BoardLayer::BL_B_Cu, BoardLayer::BL_Edge_Cuts],
            )?;
            if !r.is_ok() {
                bail!("export failed: {}", r.message);
            }
            for p in &r.output_path {
                out(&format!("exported {p}"));
            }
            Ok(ExitCode::SUCCESS)
        }
    }
}
