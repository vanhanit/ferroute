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
