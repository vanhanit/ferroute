// SPDX-License-Identifier: AGPL-3.0-only
//! Typed KiCad API client on top of a [`Transport`].

use std::time::Duration;

use prost::Message;

use crate::error::IpcError;
use crate::proto::*;
use crate::transport::Transport;

/// Default per-request timeout.
pub const DEFAULT_TIMEOUT: Duration = Duration::from_secs(10);

/// KiCad IPC API client.
#[derive(Debug)]
pub struct KiCadClient<T: Transport> {
    transport: T,
    client_name: String,
    token: Option<String>,
    timeout: Duration,
}

impl<T: Transport> KiCadClient<T> {
    /// Create a client. `token` is sent as `kicad_token` when non-empty.
    pub fn new(transport: T, client_name: impl Into<String>, token: Option<String>) -> Self {
        Self { transport, client_name: client_name.into(), token, timeout: DEFAULT_TIMEOUT }
    }

    /// Set the per-request timeout.
    #[must_use]
    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }

    /// Set the per-request timeout in place.
    pub fn set_timeout(&mut self, timeout: Duration) {
        self.timeout = timeout;
    }

    /// Consume the client, returning its transport.
    pub fn into_transport(self) -> T {
        self.transport
    }

    /// Send `req` and decode the typed reply.
    ///
    /// # Errors
    /// Transport errors, [`IpcError::Api`] for non-`AS_OK`, [`IpcError::UnexpectedType`] when
    /// the reply `Any` is not `Resp`. For `Resp = Empty` the reply payload is not inspected.
    pub fn call<Req: ApiMessage, Resp: ApiMessage>(&mut self, req: &Req) -> Result<Resp, IpcError> {
        let envelope = ApiRequest {
            header: Some(ApiRequestHeader {
                kicad_token: self.token.clone().unwrap_or_default(),
                client_name: self.client_name.clone(),
            }),
            message: Some(Any::pack(req)),
        };
        let reply = self.transport.roundtrip(&envelope.encode_to_vec(), self.timeout)?;
        let resp = ApiResponse::decode(reply.as_slice())?;
        let (code, message) =
            resp.status.as_ref().map_or((ApiStatusCode::Unknown, String::new()), |s| {
                (
                    ApiStatusCode::try_from(s.status).unwrap_or(ApiStatusCode::Unknown),
                    s.error_message.clone(),
                )
            });
        if code != ApiStatusCode::Ok {
            return Err(IpcError::Api { code, message });
        }
        if Resp::FULL_NAME == Empty::FULL_NAME {
            return Ok(Resp::default());
        }
        match resp.message {
            Some(any) => any.unpack::<Resp>(),
            None => Err(IpcError::UnexpectedType {
                expected: Resp::TYPE_URL.into(),
                got: String::new(),
            }),
        }
    }

    /// `GetVersion`.
    ///
    /// # Errors
    /// See [`Self::call`]; also [`IpcError::Protocol`] if the version is missing.
    pub fn get_version(&mut self) -> Result<KiCadVersion, IpcError> {
        let r: GetVersionResponse = self.call(&GetVersion {})?;
        r.version.ok_or_else(|| IpcError::Protocol("GetVersionResponse without version".into()))
    }

    /// `Ping`.
    ///
    /// # Errors
    /// See [`Self::call`].
    pub fn ping(&mut self) -> Result<(), IpcError> {
        self.call::<_, Empty>(&Ping {}).map(drop)
    }

    /// `GetKiCadBinaryPath`.
    ///
    /// # Errors
    /// See [`Self::call`].
    pub fn get_binary_path(&mut self, name: &str) -> Result<String, IpcError> {
        let r: PathResponse = self.call(&GetKiCadBinaryPath { binary_name: name.to_owned() })?;
        Ok(r.path)
    }

    /// `GetOpenDocuments`.
    ///
    /// # Errors
    /// See [`Self::call`].
    pub fn get_open_documents(
        &mut self,
        ty: DocumentType,
    ) -> Result<Vec<DocumentSpecifier>, IpcError> {
        let r: GetOpenDocumentsResponse = self.call(&GetOpenDocuments { r#type: ty as i32 })?;
        Ok(r.documents)
    }

    /// `SaveDocument`.
    ///
    /// # Errors
    /// See [`Self::call`].
    pub fn save_document(&mut self, doc: &DocumentSpecifier) -> Result<(), IpcError> {
        self.call::<_, Empty>(&SaveDocument { document: Some(doc.clone()) }).map(drop)
    }

    /// `RevertDocument`.
    ///
    /// # Errors
    /// See [`Self::call`].
    pub fn revert_document(&mut self, doc: &DocumentSpecifier) -> Result<(), IpcError> {
        self.call::<_, Empty>(&RevertDocument { document: Some(doc.clone()) }).map(drop)
    }

    /// `RefreshEditor`.
    ///
    /// # Errors
    /// See [`Self::call`].
    pub fn refresh_editor(&mut self, frame: FrameType) -> Result<(), IpcError> {
        self.call::<_, Empty>(&RefreshEditor { frame: frame as i32 }).map(drop)
    }

    /// `BeginCommit` on `doc`, returning the commit id.
    ///
    /// # Errors
    /// See [`Self::call`].
    pub fn begin_commit(&mut self, doc: &DocumentSpecifier) -> Result<Kiid, IpcError> {
        let header = ItemHeader { document: Some(doc.clone()), ..ItemHeader::default() };
        let r: BeginCommitResponse = self.call(&BeginCommit { header: Some(header) })?;
        r.id.ok_or_else(|| IpcError::Protocol("BeginCommitResponse without id".into()))
    }

    /// `EndCommit`.
    ///
    /// # Errors
    /// See [`Self::call`].
    pub fn end_commit(
        &mut self,
        id: Kiid,
        action: CommitAction,
        message: &str,
    ) -> Result<(), IpcError> {
        let req = EndCommit {
            id: Some(id),
            action: action as i32,
            message: message.to_owned(),
            header: None,
        };
        self.call::<_, EndCommitResponse>(&req).map(drop)
    }

    /// KiCad 11: `OpenDocument`, inferring the document type from the file extension
    /// (`.kicad_pcb`, `.kicad_sch`, `.kicad_pro`).
    ///
    /// # Errors
    /// [`IpcError::Unsupported`] for unknown extensions; see [`Self::call`].
    pub fn open_document(&mut self, path: &str) -> Result<DocumentSpecifier, IpcError> {
        let ty = if path.ends_with(".kicad_pcb") {
            DocumentType::Pcb
        } else if path.ends_with(".kicad_sch") {
            DocumentType::Schematic
        } else if path.ends_with(".kicad_pro") {
            DocumentType::Project
        } else {
            return Err(IpcError::Unsupported(format!("cannot infer document type of {path}")));
        };
        self.open_document_typed(ty, path)
    }

    /// KiCad 11: `OpenDocument` with an explicit type.
    ///
    /// # Errors
    /// See [`Self::call`].
    pub fn open_document_typed(
        &mut self,
        ty: DocumentType,
        path: &str,
    ) -> Result<DocumentSpecifier, IpcError> {
        let r: OpenDocumentResponse =
            self.call(&OpenDocument { r#type: ty as i32, path: path.to_owned() })?;
        r.document.ok_or_else(|| IpcError::Protocol("OpenDocumentResponse without document".into()))
    }

    /// KiCad 11: `CloseAllDocuments`.
    ///
    /// # Errors
    /// See [`Self::call`].
    pub fn close_all_documents(&mut self, force: bool) -> Result<(), IpcError> {
        self.call::<_, Empty>(&CloseAllDocuments { force }).map(drop)
    }

    /// KiCad 11: `RunBoardJobExportSvg` for `doc`, writing to `output_path` (one page
    /// containing all `layers`). Check [`RunJobResponse::is_ok`] on the result.
    ///
    /// # Errors
    /// See [`Self::call`].
    pub fn export_svg(
        &mut self,
        doc: &DocumentSpecifier,
        output_path: &str,
        layers: &[BoardLayer],
    ) -> Result<RunJobResponse, IpcError> {
        let req = RunBoardJobExportSvg {
            job_settings: Some(RunJobSettings {
                document: Some(doc.clone()),
                output_path: output_path.to_owned(),
            }),
            plot_settings: Some(BoardPlotSettings {
                layers: layers.iter().map(|l| *l as i32).collect(),
                scale: 1.0,
                ..BoardPlotSettings::default()
            }),
            fit_page_to_board: true,
            precision: 4,
            page_mode: BoardJobPaginationMode::AllLayersOnePage as i32,
        };
        self.call(&req)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::MockTransport;

    fn ok_reply<M: ApiMessage>(m: &M) -> Vec<u8> {
        ApiResponse {
            header: Some(ApiResponseHeader { kicad_token: "t".into() }),
            status: Some(ApiResponseStatus {
                status: ApiStatusCode::Ok as i32,
                error_message: String::new(),
            }),
            message: Some(Any::pack(m)),
        }
        .encode_to_vec()
    }

    fn err_reply(code: ApiStatusCode, msg: &str) -> Vec<u8> {
        ApiResponse {
            header: None,
            status: Some(ApiResponseStatus { status: code as i32, error_message: msg.into() }),
            message: None,
        }
        .encode_to_vec()
    }

    #[test]
    fn get_version_and_envelope() {
        let t = MockTransport::new(|req| {
            let r = ApiRequest::decode(req).unwrap();
            let h = r.header.unwrap();
            assert_eq!(h.client_name, "me");
            assert_eq!(h.kicad_token, "tok");
            assert_eq!(
                r.message.unwrap().type_url,
                "type.googleapis.com/kiapi.common.commands.GetVersion"
            );
            Ok(ok_reply(&GetVersionResponse {
                version: Some(KiCadVersion {
                    major: 10,
                    minor: 0,
                    patch: 8,
                    full_version: "10.0.8".into(),
                }),
            }))
        });
        let mut c = KiCadClient::new(t, "me", Some("tok".into()));
        let v = c.get_version().unwrap();
        assert_eq!((v.major, v.minor, v.patch), (10, 0, 8));
    }

    #[test]
    fn status_maps_to_error() {
        let t = MockTransport::new(|_| Ok(err_reply(ApiStatusCode::Unhandled, "nope")));
        let mut c = KiCadClient::new(t, "me", None);
        match c.ping() {
            Err(IpcError::Api { code: ApiStatusCode::Unhandled, message }) => {
                assert_eq!(message, "nope");
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn type_mismatch_detected() {
        let t = MockTransport::new(|_| Ok(ok_reply(&PathResponse { path: "x".into() })));
        let mut c = KiCadClient::new(t, "me", None);
        assert!(matches!(c.get_version(), Err(IpcError::UnexpectedType { .. })));
    }

    #[test]
    fn ping_accepts_empty_any() {
        let t = MockTransport::new(|_| Ok(ok_reply(&Empty {})));
        assert!(KiCadClient::new(t, "me", None).ping().is_ok());
    }

    #[test]
    fn open_documents_and_open_document_inference() {
        let t = MockTransport::new(|req| {
            let any = ApiRequest::decode(req).unwrap().message.unwrap();
            if any.type_url.ends_with("GetOpenDocuments") {
                let g: GetOpenDocuments = any.unpack().unwrap();
                assert_eq!(g.r#type, DocumentType::Pcb as i32);
                Ok(ok_reply(&GetOpenDocumentsResponse {
                    documents: vec![DocumentSpecifier::pcb("a.kicad_pcb")],
                }))
            } else {
                let o: OpenDocument = any.unpack().unwrap();
                assert_eq!(o.r#type, DocumentType::Pcb as i32);
                Ok(ok_reply(&OpenDocumentResponse {
                    document: Some(DocumentSpecifier::pcb(o.path)),
                }))
            }
        });
        let mut c = KiCadClient::new(t, "me", None);
        let docs = c.get_open_documents(DocumentType::Pcb).unwrap();
        assert_eq!(docs[0].board_filename(), Some("a.kicad_pcb"));
        assert_eq!(
            c.open_document("/x/b.kicad_pcb").unwrap().board_filename(),
            Some("/x/b.kicad_pcb")
        );
        assert!(matches!(c.open_document("/x/b.txt"), Err(IpcError::Unsupported(_))));
    }
}
