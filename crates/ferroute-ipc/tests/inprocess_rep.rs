// SPDX-License-Identifier: AGPL-3.0-only
//! Socket-path test against a pure-Rust in-process REP0 server (no Python needed).
#![cfg(unix)]

use std::os::unix::net::{UnixListener, UnixStream};
use std::thread;
use std::time::Duration;

use ferroute_ipc::caps::{ApiGeneration, negotiate};
use ferroute_ipc::client::KiCadClient;
use ferroute_ipc::endpoint::Endpoint;
use ferroute_ipc::error::IpcError;
use ferroute_ipc::nng::{self, PROTO_REP0, PROTO_REQ0};
use ferroute_ipc::proto::*;
use ferroute_ipc::transport::{NngIpcTransport, Transport};
use prost::Message;

fn answer(req: &[u8]) -> Vec<u8> {
    let any = ApiRequest::decode(req).unwrap().message.unwrap();
    let ok = |m: Option<Any>| ApiResponse {
        header: None,
        status: Some(ApiResponseStatus {
            status: ApiStatusCode::Ok as i32,
            error_message: String::new(),
        }),
        message: m,
    };
    let resp = match any.type_url.rsplit('/').next().unwrap() {
        "kiapi.common.commands.GetVersion" => ok(Some(Any::pack(&GetVersionResponse {
            version: Some(KiCadVersion {
                major: 10,
                minor: 0,
                patch: 8,
                full_version: "10.0.8".into(),
            }),
        }))),
        "kiapi.common.commands.Ping" => ok(Some(Any::pack(&Empty {}))),
        "kiapi.common.commands.GetOpenDocuments" => {
            ok(Some(Any::pack(&GetOpenDocumentsResponse {
                documents: vec![DocumentSpecifier::pcb("x.kicad_pcb")],
            })))
        }
        _ => ApiResponse {
            header: None,
            status: Some(ApiResponseStatus {
                status: ApiStatusCode::Unhandled as i32,
                error_message: "unhandled".into(),
            }),
            message: None,
        },
    };
    resp.encode_to_vec()
}

/// REP side: handshake, then answer each request, optionally preceded by a stale reply.
fn serve(mut s: UnixStream, stale_first: bool) {
    nng::handshake(&mut s, PROTO_REP0, PROTO_REQ0).unwrap();
    while let Ok(msg) = nng::read_msg(&mut s) {
        let hdr = &msg[..4];
        assert_eq!(hdr[0] & 0x80, 0x80, "request id high bit");
        if stale_first {
            let mut stale = vec![0x80, 0, 0xff, 0xff];
            stale.extend_from_slice(b"garbage");
            nng::write_msg(&mut s, &stale).unwrap();
        }
        let mut reply = hdr.to_vec();
        reply.extend(answer(&msg[4..]));
        nng::write_msg(&mut s, &reply).unwrap();
    }
}

fn spawn_server(stale: bool) -> (tempfile::TempDir, Endpoint) {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("api.sock");
    let l = UnixListener::bind(&path).unwrap();
    thread::spawn(move || {
        if let Ok((s, _)) = l.accept() {
            serve(s, stale);
        }
    });
    let ep = Endpoint::parse(&path.to_string_lossy()).unwrap();
    (dir, ep)
}

#[test]
fn negotiate_over_real_socket() {
    let (_d, ep) = spawn_server(false);
    let mut c = KiCadClient::new(NngIpcTransport::connect(&ep).unwrap(), "t", None);
    c.ping().unwrap();
    let caps = negotiate(&mut c).unwrap();
    assert_eq!(caps.generation, ApiGeneration::KiCad10);
    assert!(!caps.jobs_export && !caps.document_commands);
    assert_eq!(
        c.get_open_documents(DocumentType::Pcb).unwrap()[0].board_filename(),
        Some("x.kicad_pcb")
    );
}

#[test]
fn stale_replies_are_skipped_over_socket() {
    let (_d, ep) = spawn_server(true);
    let mut c = KiCadClient::new(NngIpcTransport::connect(&ep).unwrap(), "t", None);
    assert_eq!(c.get_version().unwrap().major, 10);
    assert_eq!(c.get_version().unwrap().minor, 0);
}

#[test]
fn bad_peer_protocol_rejected() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("x.sock");
    let l = UnixListener::bind(&path).unwrap();
    thread::spawn(move || {
        if let Ok((mut s, _)) = l.accept() {
            // Pretend to be a REQ0 peer (wrong: we need REP0).
            let _ = nng::handshake(&mut s, PROTO_REQ0, PROTO_REQ0);
        }
    });
    let ep = Endpoint::parse(&path.to_string_lossy()).unwrap();
    assert!(matches!(NngIpcTransport::connect(&ep), Err(IpcError::Protocol(_))));
}

#[test]
fn read_timeout_is_enforced() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("x.sock");
    let l = UnixListener::bind(&path).unwrap();
    thread::spawn(move || {
        if let Ok((mut s, _)) = l.accept() {
            nng::handshake(&mut s, PROTO_REP0, PROTO_REQ0).unwrap();
            let _ = nng::read_msg(&mut s);
            thread::sleep(Duration::from_secs(3)); // never reply in time
        }
    });
    let ep = Endpoint::parse(&path.to_string_lossy()).unwrap();
    let mut t = NngIpcTransport::connect(&ep).unwrap();
    let err = t.roundtrip(b"hi", Duration::from_millis(200)).unwrap_err();
    assert!(matches!(err, IpcError::Timeout), "{err:?}");
}
