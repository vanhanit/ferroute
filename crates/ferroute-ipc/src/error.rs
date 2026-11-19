// SPDX-License-Identifier: AGPL-3.0-only
//! Error type for the KiCad IPC client.

use crate::proto::ApiStatusCode;

/// Everything that can go wrong talking to KiCad.
#[derive(Debug, thiserror::Error)]
pub enum IpcError {
    /// Underlying socket / process I/O failure.
    #[error("I/O error: {0}")]
    Io(std::io::Error),
    /// The peer violated the NNG SP wire protocol (bad header, bad framing, ...).
    #[error("protocol error: {0}")]
    Protocol(String),
    /// A read or write on the socket timed out.
    #[error("timed out waiting for KiCad")]
    Timeout,
    /// KiCad answered with a non-`AS_OK` status.
    #[error("KiCad API error {code:?}: {message}")]
    Api {
        /// Status code returned by KiCad.
        code: ApiStatusCode,
        /// Human readable message returned by KiCad.
        message: String,
    },
    /// A protobuf payload could not be decoded.
    #[error("protobuf decode error: {0}")]
    Decode(#[from] prost::DecodeError),
    /// The `Any` in a response carried a different type than requested.
    #[error("unexpected response type: expected {expected}, got {got}")]
    UnexpectedType {
        /// Type URL we expected.
        expected: String,
        /// Type URL we received.
        got: String,
    },
    /// The requested feature is not available (client side or server side).
    #[error("unsupported: {0}")]
    Unsupported(String),
}

impl From<std::io::Error> for IpcError {
    fn from(e: std::io::Error) -> Self {
        match e.kind() {
            std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut => Self::Timeout,
            _ => Self::Io(e),
        }
    }
}

impl IpcError {
    /// True when this error means "the server does not know this command".
    ///
    /// KiCad answers unknown request types with `AS_UNHANDLED`; not-yet-implemented
    /// commands with `AS_UNIMPLEMENTED`; and some builds with `AS_BAD_REQUEST` plus a
    /// message about an unknown / unsupported type.
    #[must_use]
    pub fn is_unsupported_command(&self) -> bool {
        match self {
            Self::Unsupported(_) => true,
            Self::Api { code, message } => match code {
                ApiStatusCode::Unhandled | ApiStatusCode::Unimplemented => true,
                ApiStatusCode::BadRequest => {
                    let m = message.to_ascii_lowercase();
                    [
                        "unknown",
                        "unsupported",
                        "not supported",
                        "no handler",
                        "unhandled",
                        "unrecognized",
                    ]
                    .iter()
                    .any(|k| m.contains(k))
                }
                _ => false,
            },
            _ => false,
        }
    }
}
