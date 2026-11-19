// SPDX-License-Identifier: AGPL-3.0-only
//! Hand-written prost mirrors of the subset of KiCad's IPC API protobufs we use.
//!
//! Field numbers and types follow KiCad `api/proto/**` (10.0 branch and master).
//! Source file per message is given in the doc comments. Enum values keep KiCad's
//! numbering; Rust variant names are CamelCase versions of the proto names.

#![allow(missing_docs)]

use prost::Message;

use crate::error::IpcError;

/// A protobuf message that can travel inside `google.protobuf.Any`.
pub trait ApiMessage: Message + Default + Sized {
    /// Fully-qualified proto name, e.g. `kiapi.common.commands.GetVersion`.
    const FULL_NAME: &'static str;
    /// `type.googleapis.com/<FULL_NAME>`.
    const TYPE_URL: &'static str;

    /// Fully-qualified proto name.
    #[must_use]
    fn full_name() -> &'static str {
        Self::FULL_NAME
    }
}

macro_rules! api_message {
    ($t:ty, $name:literal) => {
        impl ApiMessage for $t {
            const FULL_NAME: &'static str = $name;
            const TYPE_URL: &'static str = concat!("type.googleapis.com/", $name);
        }
    };
}

// ---------------------------------------------------------------- google.protobuf

/// `google.protobuf.Any`.
#[derive(Clone, PartialEq, Message)]
pub struct Any {
    #[prost(string, tag = "1")]
    pub type_url: String,
    #[prost(bytes = "vec", tag = "2")]
    pub value: Vec<u8>,
}

impl Any {
    /// Pack a message.
    #[must_use]
    pub fn pack<M: ApiMessage>(msg: &M) -> Self {
        Self { type_url: M::TYPE_URL.to_owned(), value: msg.encode_to_vec() }
    }

    /// Unpack into `M`, verifying the type URL (the part after the last `/`).
    ///
    /// # Errors
    /// [`IpcError::UnexpectedType`] on mismatch, [`IpcError::Decode`] on bad bytes.
    pub fn unpack<M: ApiMessage>(&self) -> Result<M, IpcError> {
        let name = self.type_url.rsplit('/').next().unwrap_or("");
        if name != M::FULL_NAME {
            return Err(IpcError::UnexpectedType {
                expected: M::TYPE_URL.to_owned(),
                got: self.type_url.clone(),
            });
        }
        Ok(M::decode(self.value.as_slice())?)
    }
}

/// `google.protobuf.Empty`.
#[derive(Clone, Copy, PartialEq, Message)]
pub struct Empty {}
api_message!(Empty, "google.protobuf.Empty");

/// `google.protobuf.FieldMask`.
#[derive(Clone, PartialEq, Message)]
pub struct FieldMask {
    #[prost(string, repeated, tag = "1")]
    pub paths: Vec<String>,
}

// ---------------------------------------------------------------- common/envelope.proto

/// `kiapi.common.ApiStatusCode`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, prost::Enumeration)]
#[repr(i32)]
pub enum ApiStatusCode {
    /// AS_UNKNOWN
    Unknown = 0,
    /// AS_OK
    Ok = 1,
    /// AS_TIMEOUT
    Timeout = 2,
    /// AS_BAD_REQUEST
    BadRequest = 3,
    /// AS_NOT_READY
    NotReady = 4,
    /// AS_UNHANDLED
    Unhandled = 5,
    /// AS_TOKEN_MISMATCH
    TokenMismatch = 6,
    /// AS_BUSY
    Busy = 7,
    /// AS_UNIMPLEMENTED
    Unimplemented = 8,
    /// AS_INTERNAL_ERROR
    InternalError = 9,
}

/// `kiapi.common.ApiRequestHeader`.
#[derive(Clone, PartialEq, Message)]
pub struct ApiRequestHeader {
    #[prost(string, tag = "1")]
    pub kicad_token: String,
    #[prost(string, tag = "2")]
    pub client_name: String,
}

/// `kiapi.common.ApiRequest`.
#[derive(Clone, PartialEq, Message)]
pub struct ApiRequest {
    #[prost(message, optional, tag = "1")]
    pub header: Option<ApiRequestHeader>,
    #[prost(message, optional, tag = "2")]
    pub message: Option<Any>,
}

/// `kiapi.common.ApiResponseHeader`.
#[derive(Clone, PartialEq, Message)]
pub struct ApiResponseHeader {
    #[prost(string, tag = "1")]
    pub kicad_token: String,
}

/// `kiapi.common.ApiResponseStatus`.
#[derive(Clone, PartialEq, Message)]
pub struct ApiResponseStatus {
    #[prost(enumeration = "ApiStatusCode", tag = "1")]
    pub status: i32,
    #[prost(string, tag = "2")]
    pub error_message: String,
}

/// `kiapi.common.ApiResponse`.
#[derive(Clone, PartialEq, Message)]
pub struct ApiResponse {
    #[prost(message, optional, tag = "1")]
    pub header: Option<ApiResponseHeader>,
    #[prost(message, optional, tag = "2")]
    pub status: Option<ApiResponseStatus>,
    #[prost(message, optional, tag = "3")]
    pub message: Option<Any>,
}

// ---------------------------------------------------------------- common/types/base_types.proto

/// `kiapi.common.types.KiCadVersion`.
#[derive(Clone, PartialEq, Message)]
pub struct KiCadVersion {
    #[prost(uint32, tag = "1")]
    pub major: u32,
    #[prost(uint32, tag = "2")]
    pub minor: u32,
    #[prost(uint32, tag = "3")]
    pub patch: u32,
    #[prost(string, tag = "4")]
    pub full_version: String,
}

/// `kiapi.common.types.FrameType`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, prost::Enumeration)]
#[repr(i32)]
pub enum FrameType {
    Unknown = 0,
    ProjectManager = 1,
    SchematicEditor = 2,
    PcbEditor = 3,
    SpiceSimulator = 4,
    SymbolEditor = 5,
    FootprintEditor = 6,
    DrawingSheetEditor = 7,
}

/// `kiapi.common.types.KIID`.
#[derive(Clone, PartialEq, Message)]
pub struct Kiid {
    #[prost(string, tag = "1")]
    pub value: String,
}

/// `kiapi.common.types.DocumentType`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, prost::Enumeration)]
#[repr(i32)]
pub enum DocumentType {
    /// DOCTYPE_UNKNOWN
    Unknown = 0,
    /// DOCTYPE_SCHEMATIC
    Schematic = 1,
    /// DOCTYPE_SYMBOL
    Symbol = 2,
    /// DOCTYPE_PCB
    Pcb = 3,
    /// DOCTYPE_FOOTPRINT
    Footprint = 4,
    /// DOCTYPE_DRAWING_SHEET
    DrawingSheet = 5,
    /// DOCTYPE_PROJECT
    Project = 6,
}

/// `kiapi.common.types.LibraryIdentifier`.
#[derive(Clone, PartialEq, Message)]
pub struct LibraryIdentifier {
    #[prost(string, tag = "1")]
    pub library_nickname: String,
    #[prost(string, tag = "2")]
    pub entry_name: String,
}

/// `kiapi.common.types.SheetPath`.
#[derive(Clone, PartialEq, Message)]
pub struct SheetPath {
    #[prost(message, repeated, tag = "1")]
    pub path: Vec<Kiid>,
    #[prost(string, tag = "2")]
    pub path_human_readable: String,
}

/// `kiapi.common.types.ProjectSpecifier`.
#[derive(Clone, PartialEq, Message)]
pub struct ProjectSpecifier {
    #[prost(string, tag = "1")]
    pub name: String,
    #[prost(string, tag = "2")]
    pub path: String,
}

/// `kiapi.common.types.DocumentSpecifier`.
#[derive(Clone, PartialEq, Message)]
pub struct DocumentSpecifier {
    #[prost(enumeration = "DocumentType", tag = "1")]
    pub r#type: i32,
    #[prost(oneof = "document_specifier::Identifier", tags = "2, 3, 4")]
    pub identifier: Option<document_specifier::Identifier>,
    #[prost(message, optional, tag = "5")]
    pub project: Option<ProjectSpecifier>,
}

/// Oneof for [`DocumentSpecifier`].
pub mod document_specifier {
    use super::{LibraryIdentifier, SheetPath};

    /// `DocumentSpecifier.identifier`.
    #[derive(Clone, PartialEq, prost::Oneof)]
    pub enum Identifier {
        #[prost(message, tag = "2")]
        LibId(LibraryIdentifier),
        #[prost(message, tag = "3")]
        SheetPath(SheetPath),
        #[prost(string, tag = "4")]
        BoardFilename(String),
    }
}

impl DocumentSpecifier {
    /// Specifier for a board file.
    #[must_use]
    pub fn pcb(board_filename: impl Into<String>) -> Self {
        Self {
            r#type: DocumentType::Pcb as i32,
            identifier: Some(document_specifier::Identifier::BoardFilename(board_filename.into())),
            project: None,
        }
    }

    /// The board filename, if the identifier is one.
    #[must_use]
    pub fn board_filename(&self) -> Option<&str> {
        match &self.identifier {
            Some(document_specifier::Identifier::BoardFilename(f)) => Some(f),
            _ => None,
        }
    }

    /// Short human readable description.
    #[must_use]
    pub fn describe(&self) -> String {
        match &self.identifier {
            Some(document_specifier::Identifier::BoardFilename(f)) => f.clone(),
            Some(document_specifier::Identifier::SheetPath(s)) => s.path_human_readable.clone(),
            Some(document_specifier::Identifier::LibId(l)) => {
                format!("{}:{}", l.library_nickname, l.entry_name)
            }
            None => format!(
                "{:?}",
                DocumentType::try_from(self.r#type).unwrap_or(DocumentType::Unknown)
            ),
        }
    }
}

/// `kiapi.common.types.ItemHeader`.
#[derive(Clone, PartialEq, Message)]
pub struct ItemHeader {
    #[prost(message, optional, tag = "1")]
    pub document: Option<DocumentSpecifier>,
    #[prost(message, optional, tag = "2")]
    pub container: Option<Kiid>,
    #[prost(message, optional, tag = "3")]
    pub field_mask: Option<FieldMask>,
}

// ---------------------------------------------------------------- common/commands/base_commands.proto

/// `kiapi.common.commands.GetVersion`.
#[derive(Clone, PartialEq, Message)]
pub struct GetVersion {}
api_message!(GetVersion, "kiapi.common.commands.GetVersion");

/// `kiapi.common.commands.GetVersionResponse`.
#[derive(Clone, PartialEq, Message)]
pub struct GetVersionResponse {
    #[prost(message, optional, tag = "1")]
    pub version: Option<KiCadVersion>,
}
api_message!(GetVersionResponse, "kiapi.common.commands.GetVersionResponse");

/// `kiapi.common.commands.Ping`.
#[derive(Clone, PartialEq, Message)]
pub struct Ping {}
api_message!(Ping, "kiapi.common.commands.Ping");

/// `kiapi.common.commands.GetKiCadBinaryPath`.
#[derive(Clone, PartialEq, Message)]
pub struct GetKiCadBinaryPath {
    #[prost(string, tag = "1")]
    pub binary_name: String,
}
api_message!(GetKiCadBinaryPath, "kiapi.common.commands.GetKiCadBinaryPath");

/// `kiapi.common.commands.PathResponse`.
#[derive(Clone, PartialEq, Message)]
pub struct PathResponse {
    #[prost(string, tag = "1")]
    pub path: String,
}
api_message!(PathResponse, "kiapi.common.commands.PathResponse");

// ---------------------------------------------------------------- common/commands/editor_commands.proto

/// `kiapi.common.commands.RefreshEditor`.
#[derive(Clone, PartialEq, Message)]
pub struct RefreshEditor {
    #[prost(enumeration = "FrameType", tag = "1")]
    pub frame: i32,
}
api_message!(RefreshEditor, "kiapi.common.commands.RefreshEditor");

/// `kiapi.common.commands.GetOpenDocuments`.
#[derive(Clone, PartialEq, Message)]
pub struct GetOpenDocuments {
    #[prost(enumeration = "DocumentType", tag = "1")]
    pub r#type: i32,
}
api_message!(GetOpenDocuments, "kiapi.common.commands.GetOpenDocuments");

/// `kiapi.common.commands.GetOpenDocumentsResponse`.
#[derive(Clone, PartialEq, Message)]
pub struct GetOpenDocumentsResponse {
    #[prost(message, repeated, tag = "1")]
    pub documents: Vec<DocumentSpecifier>,
}
api_message!(GetOpenDocumentsResponse, "kiapi.common.commands.GetOpenDocumentsResponse");

/// `kiapi.common.commands.SaveDocument` (editor_commands.proto in 10.0, project_commands.proto in master).
#[derive(Clone, PartialEq, Message)]
pub struct SaveDocument {
    #[prost(message, optional, tag = "1")]
    pub document: Option<DocumentSpecifier>,
}
api_message!(SaveDocument, "kiapi.common.commands.SaveDocument");

/// `kiapi.common.commands.RevertDocument`.
#[derive(Clone, PartialEq, Message)]
pub struct RevertDocument {
    #[prost(message, optional, tag = "1")]
    pub document: Option<DocumentSpecifier>,
}
api_message!(RevertDocument, "kiapi.common.commands.RevertDocument");

/// `kiapi.common.commands.BeginCommit`.
#[derive(Clone, PartialEq, Message)]
pub struct BeginCommit {
    #[prost(message, optional, tag = "1")]
    pub header: Option<ItemHeader>,
}
api_message!(BeginCommit, "kiapi.common.commands.BeginCommit");

/// `kiapi.common.commands.BeginCommitResponse`.
#[derive(Clone, PartialEq, Message)]
pub struct BeginCommitResponse {
    #[prost(message, optional, tag = "1")]
    pub id: Option<Kiid>,
}
api_message!(BeginCommitResponse, "kiapi.common.commands.BeginCommitResponse");

/// `kiapi.common.commands.CommitAction`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, prost::Enumeration)]
#[repr(i32)]
pub enum CommitAction {
    /// CMA_UNKNOWN
    Unknown = 0,
    /// CMA_COMMIT
    Commit = 1,
    /// CMA_DROP
    Drop = 2,
}

/// `kiapi.common.commands.EndCommit`.
#[derive(Clone, PartialEq, Message)]
pub struct EndCommit {
    #[prost(message, optional, tag = "1")]
    pub id: Option<Kiid>,
    #[prost(enumeration = "CommitAction", tag = "2")]
    pub action: i32,
    #[prost(string, tag = "3")]
    pub message: String,
    #[prost(message, optional, tag = "4")]
    pub header: Option<ItemHeader>,
}
api_message!(EndCommit, "kiapi.common.commands.EndCommit");

/// `kiapi.common.commands.EndCommitResponse`.
#[derive(Clone, PartialEq, Message)]
pub struct EndCommitResponse {}
api_message!(EndCommitResponse, "kiapi.common.commands.EndCommitResponse");

// ---------------------------------------------------------------- KiCad 11: common/commands/project_commands.proto

/// `kiapi.common.commands.OpenDocument` (KiCad 11; CLI api-server mode only).
#[derive(Clone, PartialEq, Message)]
pub struct OpenDocument {
    #[prost(enumeration = "DocumentType", tag = "1")]
    pub r#type: i32,
    #[prost(string, tag = "2")]
    pub path: String,
}
api_message!(OpenDocument, "kiapi.common.commands.OpenDocument");

/// `kiapi.common.commands.OpenDocumentResponse`.
#[derive(Clone, PartialEq, Message)]
pub struct OpenDocumentResponse {
    #[prost(message, optional, tag = "1")]
    pub document: Option<DocumentSpecifier>,
}
api_message!(OpenDocumentResponse, "kiapi.common.commands.OpenDocumentResponse");

/// `kiapi.common.commands.CloseDocument` (KiCad 11).
#[derive(Clone, PartialEq, Message)]
pub struct CloseDocument {
    #[prost(message, optional, tag = "1")]
    pub document: Option<DocumentSpecifier>,
}
api_message!(CloseDocument, "kiapi.common.commands.CloseDocument");

/// `kiapi.common.commands.CloseAllDocuments` (KiCad 11; CLI api-server mode only).
#[derive(Clone, PartialEq, Message)]
pub struct CloseAllDocuments {
    #[prost(bool, tag = "1")]
    pub force: bool,
}
api_message!(CloseAllDocuments, "kiapi.common.commands.CloseAllDocuments");

// ---------------------------------------------------------------- KiCad 11: common/types/jobs.proto

/// `kiapi.common.types.JobStatus`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, prost::Enumeration)]
#[repr(i32)]
pub enum JobStatus {
    /// JS_UNSPECIFIED
    Unspecified = 0,
    /// JS_SUCCESS
    Success = 1,
    /// JS_WARNING
    Warning = 2,
    /// JS_ERROR
    Error = 3,
}

/// `kiapi.common.types.RunJobResponse`.
#[derive(Clone, PartialEq, Message)]
pub struct RunJobResponse {
    #[prost(enumeration = "JobStatus", tag = "1")]
    pub status: i32,
    #[prost(string, repeated, tag = "2")]
    pub output_path: Vec<String>,
    #[prost(string, tag = "3")]
    pub message: String,
}
api_message!(RunJobResponse, "kiapi.common.types.RunJobResponse");

impl RunJobResponse {
    /// Job finished with success or warnings.
    #[must_use]
    pub fn is_ok(&self) -> bool {
        matches!(JobStatus::try_from(self.status), Ok(JobStatus::Success | JobStatus::Warning))
    }
}

/// `kiapi.common.types.RunJobSettings`.
#[derive(Clone, PartialEq, Message)]
pub struct RunJobSettings {
    #[prost(message, optional, tag = "1")]
    pub document: Option<DocumentSpecifier>,
    #[prost(string, tag = "2")]
    pub output_path: String,
}

// ---------------------------------------------------------------- KiCad 11: board/board_jobs.proto

/// `kiapi.board.jobs.PlotDrillMarks`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, prost::Enumeration)]
#[repr(i32)]
pub enum PlotDrillMarks {
    /// PDM_UNKNOWN
    Unknown = 0,
    /// PDM_NONE
    None = 1,
    /// PDM_SMALL
    Small = 2,
    /// PDM_FULL
    Full = 3,
}

/// `kiapi.board.jobs.BoardJobPaginationMode`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, prost::Enumeration)]
#[repr(i32)]
pub enum BoardJobPaginationMode {
    /// BJPM_UNKNOWN
    Unknown = 0,
    /// BJPM_ALL_LAYERS_ONE_PAGE
    AllLayersOnePage = 1,
    /// BJPM_EACH_LAYER_OWN_PAGE
    EachLayerOwnPage = 2,
    /// BJPM_EACH_LAYER_OWN_FILE
    EachLayerOwnFile = 3,
}

/// `kiapi.board.jobs.BoardPlotSettings` (all 21 fields).
#[derive(Clone, PartialEq, Message)]
pub struct BoardPlotSettings {
    #[prost(enumeration = "BoardLayer", repeated, packed = "true", tag = "1")]
    pub layers: Vec<i32>,
    #[prost(enumeration = "BoardLayer", repeated, packed = "true", tag = "2")]
    pub common_layers: Vec<i32>,
    #[prost(string, tag = "3")]
    pub color_theme: String,
    #[prost(string, tag = "4")]
    pub drawing_sheet: String,
    #[prost(string, tag = "5")]
    pub variant: String,
    #[prost(bool, tag = "6")]
    pub mirror: bool,
    #[prost(bool, tag = "7")]
    pub black_and_white: bool,
    #[prost(bool, tag = "8")]
    pub negative: bool,
    #[prost(double, tag = "9")]
    pub scale: f64,
    #[prost(bool, tag = "10")]
    pub sketch_pads_on_fab_layers: bool,
    #[prost(bool, tag = "11")]
    pub hide_dnp_footprints_on_fab_layers: bool,
    #[prost(bool, tag = "12")]
    pub sketch_dnp_footprints_on_fab_layers: bool,
    #[prost(bool, tag = "13")]
    pub crossout_dnp_footprints_on_fab_layers: bool,
    #[prost(bool, tag = "14")]
    pub plot_footprint_values: bool,
    #[prost(bool, tag = "15")]
    pub plot_reference_designators: bool,
    #[prost(bool, tag = "16")]
    pub plot_drawing_sheet: bool,
    #[prost(bool, tag = "17")]
    pub subtract_solder_mask_from_silk: bool,
    #[prost(bool, tag = "18")]
    pub plot_pad_numbers: bool,
    #[prost(enumeration = "PlotDrillMarks", tag = "19")]
    pub drill_marks: i32,
    #[prost(bool, tag = "20")]
    pub use_drill_origin: bool,
    #[prost(bool, tag = "21")]
    pub check_zones_before_plot: bool,
}

/// `kiapi.board.jobs.RunBoardJobExportSvg`; returns [`RunJobResponse`].
#[derive(Clone, PartialEq, Message)]
pub struct RunBoardJobExportSvg {
    #[prost(message, optional, tag = "1")]
    pub job_settings: Option<RunJobSettings>,
    #[prost(message, optional, tag = "2")]
    pub plot_settings: Option<BoardPlotSettings>,
    #[prost(bool, tag = "3")]
    pub fit_page_to_board: bool,
    #[prost(uint32, tag = "4")]
    pub precision: u32,
    #[prost(enumeration = "BoardJobPaginationMode", tag = "5")]
    pub page_mode: i32,
}
api_message!(RunBoardJobExportSvg, "kiapi.board.jobs.RunBoardJobExportSvg");

/// `kiapi.board.types.BoardLayer` (board/board_types.proto), values 0..=62 (`BL_Rescue`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, prost::Enumeration)]
#[repr(i32)]
#[allow(non_camel_case_types, missing_docs)]
pub enum BoardLayer {
    BL_UNKNOWN = 0,
    BL_UNDEFINED = 1,
    BL_UNSELECTED = 2,
    BL_F_Cu = 3,
    BL_In1_Cu = 4,
    BL_In2_Cu = 5,
    BL_In3_Cu = 6,
    BL_In4_Cu = 7,
    BL_In5_Cu = 8,
    BL_In6_Cu = 9,
    BL_In7_Cu = 10,
    BL_In8_Cu = 11,
    BL_In9_Cu = 12,
    BL_In10_Cu = 13,
    BL_In11_Cu = 14,
    BL_In12_Cu = 15,
    BL_In13_Cu = 16,
    BL_In14_Cu = 17,
    BL_In15_Cu = 18,
    BL_In16_Cu = 19,
    BL_In17_Cu = 20,
    BL_In18_Cu = 21,
    BL_In19_Cu = 22,
    BL_In20_Cu = 23,
    BL_In21_Cu = 24,
    BL_In22_Cu = 25,
    BL_In23_Cu = 26,
    BL_In24_Cu = 27,
    BL_In25_Cu = 28,
    BL_In26_Cu = 29,
    BL_In27_Cu = 30,
    BL_In28_Cu = 31,
    BL_In29_Cu = 32,
    BL_In30_Cu = 33,
    BL_B_Cu = 34,
    BL_B_Adhes = 35,
    BL_F_Adhes = 36,
    BL_B_Paste = 37,
    BL_F_Paste = 38,
    BL_B_SilkS = 39,
    BL_F_SilkS = 40,
    BL_B_Mask = 41,
    BL_F_Mask = 42,
    BL_Dwgs_User = 43,
    BL_Cmts_User = 44,
    BL_Eco1_User = 45,
    BL_Eco2_User = 46,
    BL_Edge_Cuts = 47,
    BL_Margin = 48,
    BL_B_CrtYd = 49,
    BL_F_CrtYd = 50,
    BL_B_Fab = 51,
    BL_F_Fab = 52,
    BL_User_1 = 53,
    BL_User_2 = 54,
    BL_User_3 = 55,
    BL_User_4 = 56,
    BL_User_5 = 57,
    BL_User_6 = 58,
    BL_User_7 = 59,
    BL_User_8 = 60,
    BL_User_9 = 61,
    BL_Rescue = 62,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rt<M: ApiMessage + PartialEq + std::fmt::Debug>(m: &M) {
        let bytes = m.encode_to_vec();
        assert_eq!(&M::decode(bytes.as_slice()).unwrap(), m);
        let any = Any::pack(m);
        assert_eq!(any.type_url, format!("type.googleapis.com/{}", M::full_name()));
        assert_eq!(&any.unpack::<M>().unwrap(), m);
    }

    fn doc() -> DocumentSpecifier {
        DocumentSpecifier {
            project: Some(ProjectSpecifier { name: "p".into(), path: "/p".into() }),
            ..DocumentSpecifier::pcb("/p/b.kicad_pcb")
        }
    }

    #[test]
    fn every_message_roundtrips() {
        rt(&Empty {});
        rt(&GetVersion {});
        rt(&GetVersionResponse {
            version: Some(KiCadVersion {
                major: 10,
                minor: 99,
                patch: 1,
                full_version: "x".into(),
            }),
        });
        rt(&Ping {});
        rt(&GetKiCadBinaryPath { binary_name: "kicad-cli".into() });
        rt(&PathResponse { path: "/usr/bin/kicad-cli".into() });
        rt(&RefreshEditor { frame: FrameType::PcbEditor as i32 });
        rt(&GetOpenDocuments { r#type: DocumentType::Pcb as i32 });
        rt(&GetOpenDocumentsResponse { documents: vec![doc(), DocumentSpecifier::default()] });
        rt(&SaveDocument { document: Some(doc()) });
        rt(&RevertDocument { document: Some(doc()) });
        rt(&BeginCommit {
            header: Some(ItemHeader {
                document: Some(doc()),
                container: Some(Kiid { value: "k".into() }),
                field_mask: Some(FieldMask { paths: vec!["a".into()] }),
            }),
        });
        rt(&BeginCommitResponse { id: Some(Kiid { value: "abc".into() }) });
        rt(&EndCommit {
            id: Some(Kiid { value: "abc".into() }),
            action: CommitAction::Drop as i32,
            message: "m".into(),
            header: None,
        });
        rt(&EndCommitResponse {});
        rt(&OpenDocument { r#type: DocumentType::Pcb as i32, path: "/x".into() });
        rt(&OpenDocumentResponse { document: Some(doc()) });
        rt(&CloseDocument { document: Some(doc()) });
        rt(&CloseAllDocuments { force: true });
        rt(&RunJobResponse {
            status: JobStatus::Warning as i32,
            output_path: vec!["a.svg".into(), "b.svg".into()],
            message: "w".into(),
        });
        rt(&RunBoardJobExportSvg {
            job_settings: Some(RunJobSettings {
                document: Some(doc()),
                output_path: "o.svg".into(),
            }),
            plot_settings: Some(BoardPlotSettings {
                layers: vec![BoardLayer::BL_F_Cu as i32, BoardLayer::BL_Edge_Cuts as i32],
                scale: 1.0,
                drill_marks: PlotDrillMarks::Small as i32,
                ..BoardPlotSettings::default()
            }),
            fit_page_to_board: true,
            precision: 4,
            page_mode: BoardJobPaginationMode::EachLayerOwnFile as i32,
        });
        let env = ApiRequest {
            header: Some(ApiRequestHeader { kicad_token: "t".into(), client_name: "c".into() }),
            message: Some(Any::pack(&Ping {})),
        };
        assert_eq!(ApiRequest::decode(env.encode_to_vec().as_slice()).unwrap(), env);
        let resp = ApiResponse {
            header: Some(ApiResponseHeader { kicad_token: "t".into() }),
            status: Some(ApiResponseStatus {
                status: ApiStatusCode::Busy as i32,
                error_message: "b".into(),
            }),
            message: None,
        };
        assert_eq!(ApiResponse::decode(resp.encode_to_vec().as_slice()).unwrap(), resp);
    }

    #[test]
    fn wire_bytes_match_kicad_numbering() {
        // GetVersionResponse{version{major:10, minor:0, patch:8, full:"10.0.8"}}
        let m = GetVersionResponse {
            version: Some(KiCadVersion {
                major: 10,
                minor: 0,
                patch: 8,
                full_version: "10.0.8".into(),
            }),
        };
        assert_eq!(
            m.encode_to_vec(),
            [0x0A, 0x0C, 0x08, 10, 0x18, 8, 0x22, 6, b'1', b'0', b'.', b'0', b'.', b'8']
        );
        // DocumentSpecifier{type: PCB, board_filename: "a"} -> field1 varint 3, field 4 string.
        assert_eq!(DocumentSpecifier::pcb("a").encode_to_vec(), [0x08, 3, 0x22, 1, b'a']);
        // EndCommit: id=1, action=2, message=3, header=4.
        let e = EndCommit {
            id: Some(Kiid::default()),
            action: 1,
            message: "m".into(),
            header: Some(ItemHeader::default()),
        };
        assert_eq!(e.encode_to_vec(), [0x0A, 0, 0x10, 1, 0x1A, 1, b'm', 0x22, 0]);
        // Packed repeated enum layers = field 1.
        let p = BoardPlotSettings { layers: vec![3, 47], ..BoardPlotSettings::default() };
        assert_eq!(p.encode_to_vec(), [0x0A, 2, 3, 47]);
    }

    #[test]
    fn unpack_checks_type() {
        let any = Any::pack(&Ping {});
        assert!(matches!(any.unpack::<GetVersion>(), Err(IpcError::UnexpectedType { .. })));
    }

    #[test]
    fn full_names() {
        assert_eq!(GetVersion::TYPE_URL, "type.googleapis.com/kiapi.common.commands.GetVersion");
        assert_eq!(RunBoardJobExportSvg::full_name(), "kiapi.board.jobs.RunBoardJobExportSvg");
        assert_eq!(RunJobResponse::full_name(), "kiapi.common.types.RunJobResponse");
    }
}
