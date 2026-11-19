// SPDX-License-Identifier: AGPL-3.0-only
//! Minimal NNG scalability-protocols (SP) over a byte stream (`ipc://`), REQ0 side.
//!
//! Wire format (verified against NNG / pynng, see `tests/pynng_compat.rs`):
//! * connection header, both directions: `00 'S' 'P' 00`, u16 BE protocol id, u16 reserved = 0;
//! * message: `0x01`, u64 BE length, payload;
//! * REQ0/REP0 payload: u32 BE request id (high bit set) followed by the body.

use std::io::{Read, Write};

use crate::error::IpcError;

/// SP protocol id of REQ0.
pub const PROTO_REQ0: u16 = 0x0030;
/// SP protocol id of REP0.
pub const PROTO_REP0: u16 = 0x0031;
/// Message-type byte that precedes each framed message on an IPC pipe.
pub const MSG_TYPE: u8 = 0x01;
/// Upper bound on accepted message sizes (guards against hostile length fields).
pub const MAX_MSG_LEN: u64 = 256 * 1024 * 1024;
/// Replies skipped as stale before giving up.
const MAX_STALE_REPLIES: usize = 64;

/// Build the 8-byte SP connection header for protocol `proto`.
#[must_use]
pub fn header(proto: u16) -> [u8; 8] {
    let p = proto.to_be_bytes();
    [0, b'S', b'P', 0, p[0], p[1], 0, 0]
}

/// Validate a peer's SP header and require it to speak `expected_peer`.
///
/// # Errors
/// [`IpcError::Protocol`] on bad magic, non-zero reserved bytes or wrong protocol id.
pub fn validate_header(buf: &[u8; 8], expected_peer: u16) -> Result<(), IpcError> {
    if buf[..4] != [0, b'S', b'P', 0] {
        return Err(IpcError::Protocol(format!("bad SP magic {:02x?}", &buf[..4])));
    }
    if buf[6..8] != [0, 0] {
        return Err(IpcError::Protocol(format!(
            "non-zero reserved header bytes {:02x?}",
            &buf[6..8]
        )));
    }
    let peer = u16::from_be_bytes([buf[4], buf[5]]);
    if peer != expected_peer {
        return Err(IpcError::Protocol(format!(
            "peer protocol {peer:#06x} is not the expected {expected_peer:#06x}"
        )));
    }
    Ok(())
}

/// Exchange SP headers: send ours (`local`), read and validate the peer's (`peer`).
///
/// # Errors
/// I/O errors, or [`IpcError::Protocol`] when the peer header is invalid.
pub fn handshake<S: Read + Write>(stream: &mut S, local: u16, peer: u16) -> Result<(), IpcError> {
    stream.write_all(&header(local))?;
    stream.flush()?;
    let mut buf = [0u8; 8];
    stream.read_exact(&mut buf)?;
    validate_header(&buf, peer)
}

/// Write one framed message (`0x01`, u64 BE length, payload).
///
/// # Errors
/// I/O errors.
pub fn write_msg<W: Write>(w: &mut W, payload: &[u8]) -> Result<(), IpcError> {
    let mut frame = Vec::with_capacity(9 + payload.len());
    frame.push(MSG_TYPE);
    frame.extend_from_slice(&(payload.len() as u64).to_be_bytes());
    frame.extend_from_slice(payload);
    w.write_all(&frame)?;
    w.flush()?;
    Ok(())
}

/// Read one framed message and return its payload.
///
/// # Errors
/// I/O errors (incl. timeouts), bad message-type byte, or an oversized length.
pub fn read_msg<R: Read>(r: &mut R) -> Result<Vec<u8>, IpcError> {
    let mut head = [0u8; 9];
    r.read_exact(&mut head)?;
    if head[0] != MSG_TYPE {
        return Err(IpcError::Protocol(format!("unexpected message type byte {:#04x}", head[0])));
    }
    let mut len_bytes = [0u8; 8];
    len_bytes.copy_from_slice(&head[1..9]);
    let len = u64::from_be_bytes(len_bytes);
    if len > MAX_MSG_LEN {
        return Err(IpcError::Protocol(format!("message of {len} bytes exceeds limit")));
    }
    let mut buf = Vec::new();
    let got = r.take(len).read_to_end(&mut buf)?;
    if got as u64 != len {
        return Err(IpcError::Protocol(format!("truncated message: {got} of {len} bytes")));
    }
    Ok(buf)
}

/// REQ0 socket over any byte stream: request-id generation and reply matching.
#[derive(Debug)]
pub struct ReqSocket<S> {
    stream: S,
    next_id: u32,
}

impl<S: Read + Write> ReqSocket<S> {
    /// Wrap an already-handshaken stream.
    #[must_use]
    pub fn new(stream: S) -> Self {
        Self { stream, next_id: 1 }
    }

    /// Perform the SP handshake (we are REQ0, peer must be REP0) and wrap the stream.
    ///
    /// # Errors
    /// See [`handshake`].
    pub fn connect(mut stream: S) -> Result<Self, IpcError> {
        handshake(&mut stream, PROTO_REQ0, PROTO_REP0)?;
        Ok(Self::new(stream))
    }

    /// Borrow the underlying stream (e.g. to adjust timeouts).
    pub fn get_mut(&mut self) -> &mut S {
        &mut self.stream
    }

    /// Borrow the underlying stream.
    #[must_use]
    pub fn get_ref(&self) -> &S {
        &self.stream
    }

    fn take_id(&mut self) -> u32 {
        let id = self.next_id;
        self.next_id = match (self.next_id + 1) & 0x7FFF_FFFF {
            0 => 1,
            n => n,
        };
        id
    }

    /// Send `body` as a request and return the matching reply body.
    ///
    /// Replies whose request id does not match are discarded (stale replies from
    /// requests that timed out earlier).
    ///
    /// # Errors
    /// I/O / timeout errors or [`IpcError::Protocol`] for malformed replies.
    pub fn request(&mut self, body: &[u8]) -> Result<Vec<u8>, IpcError> {
        let id = self.take_id() | 0x8000_0000;
        let mut payload = Vec::with_capacity(4 + body.len());
        payload.extend_from_slice(&id.to_be_bytes());
        payload.extend_from_slice(body);
        write_msg(&mut self.stream, &payload)?;
        for _ in 0..MAX_STALE_REPLIES {
            let mut reply = read_msg(&mut self.stream)?;
            if reply.len() < 4 {
                return Err(IpcError::Protocol("reply shorter than SP header".into()));
            }
            let rid = u32::from_be_bytes([reply[0], reply[1], reply[2], reply[3]]);
            if rid == id {
                reply.drain(..4);
                return Ok(reply);
            }
            tracing::debug!(got = rid, want = id, "discarding stale reply");
        }
        Err(IpcError::Protocol("too many stale replies".into()))
    }
}

#[cfg(test)]
mod tests {
    use std::io::{self, Cursor};

    use super::*;

    /// Scripted duplex stream: reads from `input`, records writes.
    struct Duplex {
        input: Cursor<Vec<u8>>,
        out: Vec<u8>,
    }
    impl Read for Duplex {
        fn read(&mut self, b: &mut [u8]) -> io::Result<usize> {
            self.input.read(b)
        }
    }
    impl Write for Duplex {
        fn write(&mut self, b: &[u8]) -> io::Result<usize> {
            self.out.extend_from_slice(b);
            Ok(b.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    fn frame(payload: &[u8]) -> Vec<u8> {
        let mut v = Vec::new();
        write_msg(&mut v, payload).unwrap();
        v
    }

    #[test]
    fn header_bytes() {
        assert_eq!(header(PROTO_REQ0), [0, b'S', b'P', 0, 0x00, 0x30, 0, 0]);
        assert_eq!(header(PROTO_REP0), [0, b'S', b'P', 0, 0x00, 0x31, 0, 0]);
    }

    #[test]
    fn framing_roundtrip() {
        for payload in [&b""[..], b"x", &[7u8; 70_000]] {
            let f = frame(payload);
            assert_eq!(f[0], 1);
            assert_eq!(u64::from_be_bytes(f[1..9].try_into().unwrap()), payload.len() as u64);
            assert_eq!(read_msg(&mut Cursor::new(f)).unwrap(), payload);
        }
    }

    #[test]
    fn read_msg_rejects_bad_frames() {
        assert!(matches!(
            read_msg(&mut Cursor::new(vec![2, 0, 0, 0, 0, 0, 0, 0, 0])),
            Err(IpcError::Protocol(_))
        ));
        let mut huge = vec![1];
        huge.extend_from_slice(&u64::MAX.to_be_bytes());
        assert!(matches!(read_msg(&mut Cursor::new(huge)), Err(IpcError::Protocol(_))));
        let mut short = frame(b"abcdef");
        short.truncate(short.len() - 2);
        assert!(read_msg(&mut Cursor::new(short)).is_err());
    }

    #[test]
    fn handshake_ok_and_rejects() {
        let mut ok = Duplex { input: Cursor::new(header(PROTO_REP0).to_vec()), out: vec![] };
        handshake(&mut ok, PROTO_REQ0, PROTO_REP0).unwrap();
        assert_eq!(ok.out, header(PROTO_REQ0));

        let mut wrong = Duplex { input: Cursor::new(header(PROTO_REQ0).to_vec()), out: vec![] };
        assert!(
            matches!(handshake(&mut wrong, PROTO_REQ0, PROTO_REP0), Err(IpcError::Protocol(m)) if m.contains("0x0030"))
        );

        let mut bad_magic = header(PROTO_REP0);
        bad_magic[1] = b'X';
        let mut s = Duplex { input: Cursor::new(bad_magic.to_vec()), out: vec![] };
        assert!(
            matches!(handshake(&mut s, PROTO_REQ0, PROTO_REP0), Err(IpcError::Protocol(m)) if m.contains("magic"))
        );

        let mut reserved = header(PROTO_REP0);
        reserved[7] = 1;
        let mut s = Duplex { input: Cursor::new(reserved.to_vec()), out: vec![] };
        assert!(matches!(handshake(&mut s, PROTO_REQ0, PROTO_REP0), Err(IpcError::Protocol(_))));

        let mut eof = Duplex { input: Cursor::new(vec![0, b'S']), out: vec![] };
        assert!(matches!(handshake(&mut eof, PROTO_REQ0, PROTO_REP0), Err(IpcError::Io(_))));
    }

    #[test]
    fn request_id_matching_skips_stale() {
        // First request gets id 0x8000_0001; script a stale reply (id ...0000) then the real one.
        let mut input = Vec::new();
        input.extend(frame(&[&0x8000_0000u32.to_be_bytes()[..], b"stale"].concat()));
        input.extend(frame(&[&0x8000_0001u32.to_be_bytes()[..], b"fresh"].concat()));
        // Second request id 0x8000_0002 answered directly.
        input.extend(frame(&[&0x8000_0002u32.to_be_bytes()[..], b"two"].concat()));
        let mut sock = ReqSocket::new(Duplex { input: Cursor::new(input), out: vec![] });
        assert_eq!(sock.request(b"ping").unwrap(), b"fresh");
        assert_eq!(sock.request(b"ping2").unwrap(), b"two");
        let w = &sock.get_ref().out;
        let first = read_msg(&mut Cursor::new(w.clone())).unwrap();
        assert_eq!(&first[..4], &0x8000_0001u32.to_be_bytes());
        assert_eq!(&first[4..], b"ping");
    }

    #[test]
    fn short_reply_is_protocol_error() {
        let mut sock = ReqSocket::new(Duplex { input: Cursor::new(frame(b"ab")), out: vec![] });
        assert!(matches!(sock.request(b"x"), Err(IpcError::Protocol(_))));
    }

    #[test]
    fn id_wraps_within_31_bits() {
        let mut s = ReqSocket::new(Duplex { input: Cursor::new(vec![]), out: vec![] });
        s.next_id = 0x7FFF_FFFF;
        assert_eq!(s.take_id(), 0x7FFF_FFFF);
        assert_eq!(s.take_id(), 1);
    }
}
