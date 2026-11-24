// SPDX-License-Identifier: AGPL-3.0-only
//! `ferroute ipc …` against an in-process REP0 mock of KiCad (no KiCad needed).
#![cfg(unix)]

use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};
use std::process::{Command, Output};
use std::sync::{Arc, Mutex};
use std::thread;

use ferroute_ipc::nng::{self, PROTO_REP0, PROTO_REQ0};
use ferroute_ipc::proto::*;
use prost::Message;

#[derive(Clone)]
struct Mock {
    major: u32,
    minor: u32,
    /// Handles `OpenDocument` and `RunBoardJobExportSvg` (KiCad 11 headless).
    kicad11: bool,
    board: Option<String>,
    job_ok: bool,
}

type Log = Arc<Mutex<Vec<String>>>;

fn status(code: ApiStatusCode, msg: &str, m: Option<Any>) -> Vec<u8> {
    ApiResponse {
        header: None,
        status: Some(ApiResponseStatus { status: code as i32, error_message: msg.into() }),
        message: m,
    }
    .encode_to_vec()
}

fn answer(mock: &Mock, log: &Log, req: &[u8]) -> Vec<u8> {
    let any = ApiRequest::decode(req).expect("request").message.expect("message");
    let name = any.type_url.rsplit('.').next().expect("name").to_owned();
    log.lock().expect("log").push(name.clone());
    let ok = |m: Option<Any>| status(ApiStatusCode::Ok, "", m);
    match name.as_str() {
        "GetVersion" => ok(Some(Any::pack(&GetVersionResponse {
            version: Some(KiCadVersion {
                major: mock.major,
                minor: mock.minor,
                patch: 0,
                full_version: format!("{}.{}.0", mock.major, mock.minor),
            }),
        }))),
        "GetOpenDocuments" => {
            let ty = GetOpenDocuments::decode(&any.value[..]).expect("decode").r#type;
            let documents = match (&mock.board, ty == DocumentType::Pcb as i32) {
                (Some(b), true) => vec![DocumentSpecifier::pcb(b.clone())],
                _ => vec![],
            };
            ok(Some(Any::pack(&GetOpenDocumentsResponse { documents })))
        }
        "SaveDocument" | "RevertDocument" => ok(Some(Any::pack(&Empty {}))),
        "OpenDocument" if mock.kicad11 => {
            let r = OpenDocument::decode(&any.value[..]).expect("decode");
            if r.path.is_empty() {
                status(ApiStatusCode::BadRequest, "empty path", None)
            } else {
                ok(Some(Any::pack(&OpenDocumentResponse {
                    document: Some(DocumentSpecifier::pcb(r.path)),
                })))
            }
        }
        "RunBoardJobExportSvg" if mock.kicad11 => {
            let req = RunBoardJobExportSvg::decode(&any.value[..]).expect("decode");
            let target = req.job_settings.map(|j| j.output_path).unwrap_or_default();
            if target.is_empty() {
                status(ApiStatusCode::BadRequest, "no path", None)
            } else if mock.job_ok {
                ok(Some(Any::pack(&RunJobResponse {
                    status: JobStatus::Success as i32,
                    output_path: vec![target],
                    message: String::new(),
                })))
            } else {
                ok(Some(Any::pack(&RunJobResponse {
                    status: JobStatus::Error as i32,
                    output_path: vec![],
                    message: "plot exploded".into(),
                })))
            }
        }
        _ => status(ApiStatusCode::Unhandled, "unhandled", None),
    }
}

fn serve(mut s: UnixStream, mock: &Mock, log: &Log) {
    nng::handshake(&mut s, PROTO_REP0, PROTO_REQ0).expect("handshake");
    while let Ok(msg) = nng::read_msg(&mut s) {
        let mut reply = msg[..4].to_vec();
        reply.extend(answer(mock, log, &msg[4..]));
        if nng::write_msg(&mut s, &reply).is_err() {
            break;
        }
    }
}

struct Server {
    _dir: tempfile::TempDir,
    sock: PathBuf,
    log: Log,
}

fn spawn(mock: Mock) -> Server {
    let dir = tempfile::tempdir().expect("tmp");
    let sock = dir.path().join("api.sock");
    let l = UnixListener::bind(&sock).expect("bind");
    let log: Log = Arc::default();
    let log2 = log.clone();
    thread::spawn(move || {
        for conn in l.incoming().flatten() {
            let (m, lg) = (mock.clone(), log2.clone());
            thread::spawn(move || serve(conn, &m, &lg));
        }
    });
    Server { _dir: dir, sock, log }
}

impl Server {
    fn sock(&self) -> &str {
        self.sock.to_str().expect("utf-8")
    }
    fn calls(&self) -> Vec<String> {
        self.log.lock().expect("log").clone()
    }
}

fn mock10(board: Option<&str>) -> Mock {
    Mock { major: 10, minor: 0, kicad11: false, board: board.map(Into::into), job_ok: true }
}

fn mock11(job_ok: bool) -> Mock {
    Mock { major: 11, minor: 0, kicad11: true, board: Some("b.kicad_pcb".into()), job_ok }
}

fn ferroute(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_ferroute"))
        .args(args)
        .current_dir(Path::new(env!("CARGO_MANIFEST_DIR")).join("../.."))
        .output()
        .expect("run ferroute")
}

fn text(b: &[u8]) -> String {
    String::from_utf8_lossy(b).into_owned()
}

fn p(path: &Path) -> &str {
    path.to_str().expect("utf-8 path")
}

#[test]
fn probe_reports_a_kicad10_gui() {
    let srv = spawn(mock10(Some("x.kicad_pcb")));
    let o = ferroute(&["ipc", "probe", "--socket", srv.sock()]);
    assert!(o.status.success(), "{}", text(&o.stderr));
    let out = text(&o.stdout);
    assert!(out.contains("KiCad 10.0.0"), "{out}");
    assert!(out.contains("live GUI: true  headless server: false"), "{out}");
    assert!(out.contains("open: x.kicad_pcb"), "{out}");
}

#[test]
fn probe_json_reports_a_kicad11_headless_server() {
    let srv = spawn(mock11(true));
    let o = ferroute(&["ipc", "probe", "--socket", srv.sock(), "--json"]);
    assert!(o.status.success(), "{}", text(&o.stderr));
    let v: serde_json::Value = serde_json::from_slice(&o.stdout).expect("json");
    assert_eq!(v["capabilities"]["headless_server"], true);
    assert_eq!(v["capabilities"]["jobs_export"], true);
    assert_eq!(v["open_documents"][0], "b.kicad_pcb");
}

#[test]
fn route_saves_routes_in_place_and_reverts() {
    let dir = tempfile::tempdir().expect("tmp");
    let board = dir.path().join("demo.kicad_pcb");
    let src = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/boards");
    std::fs::copy(src.join("demo_2layer.kicad_pcb"), &board).expect("copy");
    let before = std::fs::read(&board).expect("read");
    let srv = spawn(mock10(Some(p(&board))));
    let o = ferroute(&["ipc", "route", "--socket", srv.sock(), "--seed", "3"]);
    assert!(o.status.success(), "{}{}", text(&o.stdout), text(&o.stderr));
    assert!(text(&o.stdout).contains("KiCad 10 file bridge"), "{}", text(&o.stdout));
    let calls = srv.calls();
    let pos = |n: &str| calls.iter().position(|c| c == n).unwrap_or_else(|| panic!("{n}"));
    assert!(pos("SaveDocument") < pos("RevertDocument"), "{calls:?}");
    assert_ne!(std::fs::read(&board).expect("read"), before, "board was routed in place");
}

#[test]
fn route_without_an_open_board_fails() {
    let srv = spawn(mock10(None));
    let o = ferroute(&["ipc", "route", "--socket", srv.sock()]);
    assert_eq!(o.status.code(), Some(2));
    assert!(text(&o.stderr).contains("no board is open"), "{}", text(&o.stderr));
    assert!(!srv.calls().iter().any(|c| c == "SaveDocument"));
}

#[test]
fn export_svg_needs_kicad11() {
    let srv = spawn(mock10(Some("x.kicad_pcb")));
    let dir = tempfile::tempdir().expect("tmp");
    let o = ferroute(&["ipc", "export-svg", "--socket", srv.sock(), "--out", p(dir.path())]);
    assert_eq!(o.status.code(), Some(2));
    assert!(text(&o.stderr).contains("needs KiCad 11+"), "{}", text(&o.stderr));
}

#[test]
fn export_svg_opens_the_given_board_and_lists_outputs() {
    let srv = spawn(mock11(true));
    let dir = tempfile::tempdir().expect("tmp");
    let out = dir.path().join("plots");
    let o = ferroute(&[
        "ipc",
        "export-svg",
        "--socket",
        srv.sock(),
        "--out",
        p(&out),
        "--board",
        "/tmp/some.kicad_pcb",
    ]);
    assert!(o.status.success(), "{}", text(&o.stderr));
    assert!(text(&o.stdout).contains("board.svg"), "{}", text(&o.stdout));
    assert!(out.is_dir(), "output directory created");
    assert!(srv.calls().iter().any(|c| c == "RunBoardJobExportSvg"));
}

#[test]
fn export_svg_without_board_uses_the_open_one_and_reports_job_errors() {
    let srv = spawn(mock11(false));
    let dir = tempfile::tempdir().expect("tmp");
    let o = ferroute(&["ipc", "export-svg", "--socket", srv.sock(), "--out", p(dir.path())]);
    assert_eq!(o.status.code(), Some(2));
    assert!(text(&o.stderr).contains("export failed: plot exploded"), "{}", text(&o.stderr));
}
