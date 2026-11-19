// SPDX-License-Identifier: AGPL-3.0-only
//! Capability negotiation: version-derived generation plus live feature probes.

use std::time::Duration;

use serde::Serialize;

use crate::client::KiCadClient;
use crate::endpoint::Endpoint;
use crate::error::IpcError;
use crate::proto::{
    ApiMessage, DocumentType, OpenDocument, OpenDocumentResponse, RunBoardJobExportSvg,
    RunJobResponse,
};
use crate::transport::{NngIpcTransport, Transport};

/// Coarse KiCad API generation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum ApiGeneration {
    /// Older than KiCad 9 (no IPC API).
    Unsupported,
    /// KiCad 9.x.
    KiCad9,
    /// KiCad 10.x stable.
    KiCad10,
    /// KiCad 11 and 10.99 nightlies.
    KiCad11Plus,
}

/// Map a reported version to a generation. `10.99.x` nightlies are KiCad 11.
#[must_use]
pub fn generation_for(major: u32, minor: u32) -> ApiGeneration {
    match (major, minor) {
        (0..=8, _) => ApiGeneration::Unsupported,
        (9, _) => ApiGeneration::KiCad9,
        (10, 0..=98) => ApiGeneration::KiCad10,
        _ => ApiGeneration::KiCad11Plus,
    }
}

/// Version triple plus KiCad's full version string.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct KiCadVersionInfo {
    pub major: u32,
    pub minor: u32,
    pub patch: u32,
    pub full: String,
}

/// What the connected server can do.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct Capabilities {
    pub version: KiCadVersionInfo,
    pub generation: ApiGeneration,
    /// A GUI instance (heuristic: not a headless api-server).
    pub live_gui: bool,
    /// `kicad-cli api-server` (heuristic: it handles `OpenDocument`, documented as CLI-only).
    pub headless_server: bool,
    /// `RunBoardJobExportSvg` is handled.
    pub jobs_export: bool,
    /// `OpenDocument` / `CloseAllDocuments` are handled.
    pub document_commands: bool,
}

/// Output of [`probe_endpoint`], printed as JSON by the CLI.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProbeReport {
    pub endpoint: String,
    pub capabilities: Capabilities,
    pub open_documents: Vec<String>,
}

/// Send `req` and report whether the server handles it. Any reply other than
/// "unhandled / unimplemented / unknown type" counts as present (a validation
/// error for our deliberately empty request proves the handler exists).
fn probe<T: Transport, Req: ApiMessage, Resp: ApiMessage>(
    client: &mut KiCadClient<T>,
    req: &Req,
) -> Result<bool, IpcError> {
    match client.call::<Req, Resp>(req) {
        Ok(_) => Ok(true),
        Err(e) if e.is_unsupported_command() => Ok(false),
        Err(IpcError::Api { .. } | IpcError::UnexpectedType { .. } | IpcError::Decode(_)) => {
            Ok(true)
        }
        Err(e) => Err(e),
    }
}

/// `GetVersion`, then probe optional commands with harmless, deliberately invalid requests.
///
/// # Errors
/// Transport failures and a failing `GetVersion`.
pub fn negotiate<T: Transport>(client: &mut KiCadClient<T>) -> Result<Capabilities, IpcError> {
    let v = client.get_version()?;
    let generation = generation_for(v.major, v.minor);
    // Empty path + unknown type: a handler rejects it without touching any state.
    let document_commands = probe::<_, _, OpenDocumentResponse>(
        client,
        &OpenDocument { r#type: DocumentType::Unknown as i32, path: String::new() },
    )?;
    let jobs_export = probe::<_, _, RunJobResponse>(client, &RunBoardJobExportSvg::default())?;
    let headless_server = document_commands;
    Ok(Capabilities {
        version: KiCadVersionInfo {
            major: v.major,
            minor: v.minor,
            patch: v.patch,
            full: v.full_version,
        },
        generation,
        live_gui: !headless_server,
        headless_server,
        jobs_export,
        document_commands,
    })
}

/// Connect to `endpoint`, negotiate, and list open documents (best effort).
///
/// # Errors
/// Connection and negotiation failures.
pub fn probe_endpoint(
    endpoint: &Endpoint,
    client_name: &str,
    timeout: Duration,
) -> Result<ProbeReport, IpcError> {
    let transport = NngIpcTransport::connect(endpoint)?;
    let mut client =
        KiCadClient::new(transport, client_name, endpoint.token.clone()).with_timeout(timeout);
    let capabilities = negotiate(&mut client)?;
    let mut open_documents = Vec::new();
    for ty in [DocumentType::Pcb, DocumentType::Schematic] {
        if let Ok(docs) = client.get_open_documents(ty) {
            open_documents.extend(docs.iter().map(crate::proto::DocumentSpecifier::describe));
        }
    }
    Ok(ProbeReport { endpoint: endpoint.url.clone(), capabilities, open_documents })
}

#[cfg(test)]
mod tests {
    use prost::Message;

    use super::*;
    use crate::proto::*;
    use crate::transport::MockTransport;

    #[test]
    fn generation_table() {
        use ApiGeneration::*;
        for (maj, min, want) in [
            (8, 2, Unsupported),
            (9, 0, KiCad9),
            (9, 99, KiCad9),
            (10, 0, KiCad10),
            (10, 8, KiCad10),
            (10, 98, KiCad10),
            (10, 99, KiCad11Plus),
            (11, 0, KiCad11Plus),
            (12, 3, KiCad11Plus),
        ] {
            assert_eq!(generation_for(maj, min), want, "{maj}.{min}");
        }
    }

    fn server(major: u32, minor: u32, handles_new: bool) -> MockTransport {
        MockTransport::new(move |req| {
            let any = ApiRequest::decode(req).unwrap().message.unwrap();
            let (status, msg) = if any.type_url.ends_with(".GetVersion") {
                (
                    ApiStatusCode::Ok,
                    Some(Any::pack(&GetVersionResponse {
                        version: Some(KiCadVersion {
                            major,
                            minor,
                            patch: 0,
                            full_version: format!("{major}.{minor}.0"),
                        }),
                    })),
                )
            } else if handles_new {
                (ApiStatusCode::BadRequest, None)
            } else {
                (ApiStatusCode::Unhandled, None)
            };
            Ok(ApiResponse {
                header: None,
                status: Some(ApiResponseStatus {
                    status: status as i32,
                    error_message: "x".into(),
                }),
                message: msg,
            }
            .encode_to_vec())
        })
    }

    #[test]
    fn negotiate_gui_10() {
        let mut c = KiCadClient::new(server(10, 0, false), "t", None);
        let caps = negotiate(&mut c).unwrap();
        assert_eq!(caps.generation, ApiGeneration::KiCad10);
        assert!(
            caps.live_gui && !caps.headless_server && !caps.jobs_export && !caps.document_commands
        );
    }

    #[test]
    fn negotiate_headless_1099() {
        let mut c = KiCadClient::new(server(10, 99, true), "t", None);
        let caps = negotiate(&mut c).unwrap();
        assert_eq!(caps.generation, ApiGeneration::KiCad11Plus);
        assert!(
            caps.headless_server && caps.jobs_export && caps.document_commands && !caps.live_gui
        );
        let json = serde_json::to_string(&caps).unwrap();
        assert!(json.contains("KiCad11Plus"));
    }

    #[test]
    fn version_lie_is_not_trusted() {
        // Claims 11.0 but handles neither new command.
        let mut c = KiCadClient::new(server(11, 0, false), "t", None);
        let caps = negotiate(&mut c).unwrap();
        assert_eq!(caps.generation, ApiGeneration::KiCad11Plus);
        assert!(!caps.jobs_export && !caps.document_commands);
    }
}
