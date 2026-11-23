// SPDX-License-Identifier: AGPL-3.0-only
//! Request/response transports.

use std::fmt;
use std::time::Duration;

use crate::endpoint::Endpoint;
use crate::error::IpcError;
use crate::nng::ReqSocket;

/// One blocking request/response exchange of raw `ApiRequest` / `ApiResponse` bytes.
pub trait Transport: Send {
    /// Send `request`, wait up to `timeout` for the reply.
    ///
    /// # Errors
    /// [`IpcError::Timeout`], I/O or protocol errors.
    fn roundtrip(&mut self, request: &[u8], timeout: Duration) -> Result<Vec<u8>, IpcError>;
}

impl<T: Transport + ?Sized> Transport for Box<T> {
    fn roundtrip(&mut self, request: &[u8], timeout: Duration) -> Result<Vec<u8>, IpcError> {
        (**self).roundtrip(request, timeout)
    }
}

#[cfg(unix)]
type PlatformStream = std::os::unix::net::UnixStream;
#[cfg(windows)]
type PlatformStream = std::fs::File;

/// NNG REQ0 over `ipc://` (Unix domain socket; Windows named pipe).
#[derive(Debug)]
pub struct NngIpcTransport {
    sock: ReqSocket<PlatformStream>,
}

impl NngIpcTransport {
    /// Connect and perform the SP handshake.
    ///
    /// # Errors
    /// I/O errors (socket missing / refused) or protocol errors.
    #[cfg(unix)]
    pub fn connect(endpoint: &Endpoint) -> Result<Self, IpcError> {
        let stream = std::os::unix::net::UnixStream::connect(endpoint.socket_path())?;
        // Bound the handshake too.
        stream.set_read_timeout(Some(Duration::from_secs(5)))?;
        stream.set_write_timeout(Some(Duration::from_secs(5)))?;
        Ok(Self { sock: ReqSocket::connect(stream)? })
    }

    /// Connect and perform the SP handshake (Windows named pipe `\\.\pipe\<path>`).
    ///
    /// Note: plain `File` pipe handles have no read timeout; `timeout` is not enforced here.
    ///
    /// # Errors
    /// I/O errors or protocol errors.
    #[cfg(windows)]
    pub fn connect(endpoint: &Endpoint) -> Result<Self, IpcError> {
        let name = format!(r"\\.\pipe\{}", endpoint.socket_path().display());
        let f = std::fs::OpenOptions::new().read(true).write(true).open(name)?;
        Ok(Self { sock: ReqSocket::connect(f)? })
    }
}

impl Transport for NngIpcTransport {
    fn roundtrip(&mut self, request: &[u8], timeout: Duration) -> Result<Vec<u8>, IpcError> {
        #[cfg(unix)]
        {
            let s = self.sock.get_mut();
            s.set_read_timeout(Some(timeout))?;
            s.set_write_timeout(Some(timeout))?;
        }
        #[cfg(windows)]
        let _ = timeout;
        self.sock.request(request)
    }
}

type MockFn = dyn FnMut(&[u8]) -> Result<Vec<u8>, IpcError> + Send;

/// Closure-backed transport for tests.
pub struct MockTransport {
    handler: Box<MockFn>,
}

impl MockTransport {
    /// Build from a closure mapping request bytes to reply bytes.
    pub fn new(handler: impl FnMut(&[u8]) -> Result<Vec<u8>, IpcError> + Send + 'static) -> Self {
        Self { handler: Box::new(handler) }
    }
}

impl fmt::Debug for MockTransport {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MockTransport").finish_non_exhaustive()
    }
}

impl Transport for MockTransport {
    fn roundtrip(&mut self, request: &[u8], _timeout: Duration) -> Result<Vec<u8>, IpcError> {
        (self.handler)(request)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mock_transport_passes_bytes_and_errors() {
        let mut t = MockTransport::new(|req| {
            if req.is_empty() {
                Err(IpcError::Timeout)
            } else {
                Ok(req.iter().rev().copied().collect())
            }
        });
        assert_eq!(t.roundtrip(b"abc", Duration::from_secs(1)).unwrap(), b"cba");
        assert!(matches!(t.roundtrip(b"", Duration::ZERO), Err(IpcError::Timeout)));
        assert!(format!("{t:?}").starts_with("MockTransport"));
    }

    #[test]
    fn mock_transport_is_stateful() {
        let mut n = 0u8;
        let mut t = MockTransport::new(move |_| {
            n += 1;
            Ok(vec![n])
        });
        assert_eq!(t.roundtrip(b"", Duration::ZERO).unwrap(), [1]);
        assert_eq!(t.roundtrip(b"", Duration::ZERO).unwrap(), [2]);
    }

    #[test]
    fn boxed_dyn_transport_delegates() {
        let mut t: Box<dyn Transport> = Box::new(MockTransport::new(|r| Ok(r.to_vec())));
        assert_eq!(t.roundtrip(b"xy", Duration::ZERO).unwrap(), b"xy");
    }

    #[cfg(unix)]
    #[test]
    fn connect_to_missing_socket_is_io_error() {
        let d = tempfile::tempdir().unwrap();
        let ep = Endpoint::parse(&d.path().join("none.sock").to_string_lossy()).unwrap();
        assert!(matches!(NngIpcTransport::connect(&ep), Err(IpcError::Io(_))));
    }

    #[cfg(unix)]
    #[test]
    fn connect_to_non_socket_file_fails() {
        let d = tempfile::tempdir().unwrap();
        let p = d.path().join("plain");
        std::fs::write(&p, b"x").unwrap();
        let ep = Endpoint::parse(&p.to_string_lossy()).unwrap();
        assert!(NngIpcTransport::connect(&ep).is_err());
    }

    #[cfg(unix)]
    #[test]
    fn peer_closing_after_handshake_fails_roundtrip() {
        use crate::nng::{self, PROTO_REP0, PROTO_REQ0};
        let d = tempfile::tempdir().unwrap();
        let path = d.path().join("c.sock");
        let l = std::os::unix::net::UnixListener::bind(&path).unwrap();
        let h = std::thread::spawn(move || {
            let (mut s, _) = l.accept().unwrap();
            nng::handshake(&mut s, PROTO_REP0, PROTO_REQ0).unwrap();
            let _ = nng::read_msg(&mut s);
            // drop: close without replying
        });
        let ep = Endpoint::parse(&path.to_string_lossy()).unwrap();
        let mut t = NngIpcTransport::connect(&ep).unwrap();
        assert!(t.roundtrip(b"hi", Duration::from_secs(2)).is_err());
        h.join().unwrap();
    }
}
