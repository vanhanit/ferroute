// SPDX-License-Identifier: AGPL-3.0-only
//! Wire-compat test against a real NNG REP0 server (pynng + KiCad's generated protobufs).
//! Runs only when `FERROUTE_PY` points to a Python with `pynng` and `kipy` installed.
#![cfg(unix)]

use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::time::Duration;

use ferroute_ipc::caps::{ApiGeneration, negotiate};
use ferroute_ipc::client::KiCadClient;
use ferroute_ipc::endpoint::Endpoint;
use ferroute_ipc::headless::HeadlessServer;
use ferroute_ipc::proto::DocumentType;
use ferroute_ipc::transport::NngIpcTransport;

struct Fake(Child);
impl Drop for Fake {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

fn server_script() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/fake_kicad_server.py")
}

fn python() -> Option<String> {
    let py = std::env::var("FERROUTE_PY").ok().filter(|s| !s.is_empty());
    if py.is_none() {
        eprintln!("skipped: FERROUTE_PY not set");
    }
    py
}

fn start(py: &str, version: &str, sock: &Path) -> Fake {
    let mut child = Command::new(py)
        .arg("-I")
        .arg(server_script())
        .arg(version)
        .arg(format!("ipc://{}", sock.display()))
        .arg("/work/board.kicad_pcb")
        .stdout(Stdio::piped())
        .spawn()
        .expect("spawn python");
    let mut line = String::new();
    BufReader::new(child.stdout.take().unwrap()).read_line(&mut line).unwrap();
    assert_eq!(line.trim(), "READY");
    Fake(child)
}

fn run(py: &str, version: &str) -> (ferroute_ipc::Capabilities, Vec<String>) {
    let dir = tempfile::tempdir().unwrap();
    let sock = dir.path().join("api.sock");
    let _srv = start(py, version, &sock);
    let ep = Endpoint::parse(&sock.to_string_lossy()).unwrap();
    let mut c = KiCadClient::new(NngIpcTransport::connect(&ep).unwrap(), "ferroute-test", None)
        .with_timeout(Duration::from_secs(10));
    c.ping().unwrap();
    let caps = negotiate(&mut c).unwrap();
    let docs = c.get_open_documents(DocumentType::Pcb).unwrap();
    assert!(c.get_open_documents(DocumentType::Schematic).unwrap().is_empty());
    (caps, docs.iter().map(|d| d.board_filename().unwrap().to_owned()).collect())
}

#[test]
fn kicad_10_0_8_gui_like() {
    let Some(py) = python() else { return };
    let (caps, docs) = run(&py, "10.0.8");
    assert_eq!(caps.generation, ApiGeneration::KiCad10);
    assert_eq!((caps.version.major, caps.version.minor, caps.version.patch), (10, 0, 8));
    assert!(caps.live_gui && !caps.headless_server && !caps.jobs_export && !caps.document_commands);
    assert_eq!(docs, ["/work/board.kicad_pcb"]);
}

#[test]
fn kicad_10_99_headless_like() {
    let Some(py) = python() else { return };
    let (caps, docs) = run(&py, "10.99.0");
    assert_eq!(caps.generation, ApiGeneration::KiCad11Plus);
    assert!(caps.headless_server && !caps.live_gui && caps.jobs_export && caps.document_commands);
    assert_eq!(docs, ["/work/board.kicad_pcb"]);
}

/// `HeadlessServer::spawn` end to end, with a shell wrapper standing in for `kicad-cli`.
#[test]
fn headless_spawn_with_wrapper() {
    use std::os::unix::fs::PermissionsExt;
    let Some(py) = python() else { return };
    let dir = tempfile::tempdir().unwrap();
    let cli = dir.path().join("kicad-cli");
    // Called as: kicad-cli api-server --socket <path> [file]
    let script = format!(
        "#!/bin/sh\n[ \"$1\" = api-server ] && [ \"$2\" = --socket ] || exit 9\nexec '{py}' -I '{}' 10.99.0 \"ipc://$3\" \"$4\"\n",
        server_script().display()
    );
    std::fs::write(&cli, script).unwrap();
    std::fs::set_permissions(&cli, std::fs::Permissions::from_mode(0o755)).unwrap();
    let board = dir.path().join("b.kicad_pcb");
    let sock = dir.path().join("api.sock");
    let srv = HeadlessServer::spawn(&cli, Some(&board), &sock, Duration::from_secs(30)).unwrap();
    let mut c = KiCadClient::new(NngIpcTransport::connect(srv.endpoint()).unwrap(), "t", None);
    let docs = c.get_open_documents(DocumentType::Pcb).unwrap();
    assert_eq!(docs[0].board_filename(), Some(board.to_str().unwrap()));
}
