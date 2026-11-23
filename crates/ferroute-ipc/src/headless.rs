// SPDX-License-Identifier: AGPL-3.0-only
//! Spawning and supervising `kicad-cli api-server` (KiCad 11+).

use std::ffi::OsString;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

use crate::client::KiCadClient;
use crate::endpoint::Endpoint;
use crate::error::IpcError;
use crate::transport::NngIpcTransport;

/// A running headless KiCad API server; the child process is killed on drop.
#[derive(Debug)]
pub struct HeadlessServer {
    child: Child,
    endpoint: Endpoint,
}

impl HeadlessServer {
    /// Run `kicad-cli api-server --socket <socket_path> [board]` and wait until the
    /// socket accepts a connection and a `Ping` succeeds.
    ///
    /// # Errors
    /// Spawn failure, early child exit, or [`IpcError::Timeout`] after `ready_timeout`.
    pub fn spawn(
        kicad_cli: &Path,
        board: Option<&Path>,
        socket_path: &Path,
        ready_timeout: Duration,
    ) -> Result<Self, IpcError> {
        let mut cmd = Command::new(kicad_cli);
        cmd.arg("api-server").arg("--socket").arg(socket_path);
        if let Some(b) = board {
            cmd.arg(b);
        }
        cmd.stdin(Stdio::null()).stdout(Stdio::null()).stderr(Stdio::null());
        let child = cmd.spawn()?;
        let endpoint = Endpoint::parse(&socket_path.to_string_lossy())?;
        let mut server = Self { child, endpoint };
        server.wait_ready(ready_timeout)?;
        Ok(server)
    }

    fn wait_ready(&mut self, timeout: Duration) -> Result<(), IpcError> {
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(status) = self.child.try_wait()? {
                return Err(IpcError::Protocol(format!(
                    "kicad-cli api-server exited early: {status}"
                )));
            }
            if let Ok(t) = NngIpcTransport::connect(&self.endpoint) {
                let mut c = KiCadClient::new(t, "ferroute-headless-probe", None)
                    .with_timeout(Duration::from_secs(2));
                if c.ping().is_ok() {
                    return Ok(());
                }
            }
            if Instant::now() >= deadline {
                return Err(IpcError::Timeout);
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    }

    /// Endpoint to connect clients to.
    #[must_use]
    pub fn endpoint(&self) -> &Endpoint {
        &self.endpoint
    }

    /// OS process id of the server.
    #[must_use]
    pub fn pid(&self) -> u32 {
        self.child.id()
    }
}

impl Drop for HeadlessServer {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
        let _ = std::fs::remove_file(self.endpoint.socket_path());
    }
}

fn exe_names() -> &'static [&'static str] {
    if cfg!(windows) {
        &["kicad-cli.exe", "kicad-cli-nightly.exe"]
    } else {
        &["kicad-cli", "kicad-cli-nightly"]
    }
}

/// Common install locations (besides `PATH`).
#[must_use]
pub fn common_install_dirs() -> Vec<PathBuf> {
    let mut v: Vec<PathBuf> = Vec::new();
    if cfg!(windows) {
        for ver in ["11.0", "10.99", "10.0", "9.0"] {
            v.push(PathBuf::from(format!(r"C:\Program Files\KiCad\{ver}\bin")));
        }
        v.push(PathBuf::from(r"C:\Program Files\KiCad\bin"));
    } else if cfg!(target_os = "macos") {
        v.push("/Applications/KiCad/KiCad.app/Contents/MacOS".into());
        v.push("/Applications/KiCad-Nightly/KiCad.app/Contents/MacOS".into());
        v.push("/opt/homebrew/bin".into());
    } else {
        for d in [
            "/usr/bin",
            "/usr/local/bin",
            "/usr/lib/kicad-nightly/bin",
            "/opt/kicad/bin",
            "/var/lib/flatpak/exports/bin",
        ] {
            v.push(d.into());
        }
    }
    v
}

/// Search `path_var` (a `PATH`-style list) then `extra_dirs` for `kicad-cli` / `kicad-cli-nightly`.
#[must_use]
pub fn find_kicad_cli_in(path_var: Option<OsString>, extra_dirs: &[PathBuf]) -> Option<PathBuf> {
    let mut dirs: Vec<PathBuf> =
        path_var.map(|p| std::env::split_paths(&p).collect()).unwrap_or_default();
    dirs.extend(extra_dirs.iter().cloned());
    dirs.iter().flat_map(|d| exe_names().iter().map(move |n| d.join(n))).find(|p| p.is_file())
}

/// Find `kicad-cli` on `PATH` or in common install directories.
#[must_use]
pub fn find_kicad_cli() -> Option<PathBuf> {
    find_kicad_cli_in(std::env::var_os("PATH"), &common_install_dirs())
}

#[cfg(all(test, unix))]
mod tests {
    use std::os::unix::fs::PermissionsExt;

    use super::*;

    fn script(dir: &Path, name: &str, body: &str) -> PathBuf {
        let p = dir.join(name);
        std::fs::write(&p, format!("#!/bin/sh\n{body}\n")).unwrap();
        std::fs::set_permissions(&p, std::fs::Permissions::from_mode(0o755)).unwrap();
        p
    }

    #[test]
    fn finds_cli_in_path_and_extra() {
        let d = tempfile::tempdir().unwrap();
        let e = tempfile::tempdir().unwrap();
        assert!(find_kicad_cli_in(Some(d.path().into()), &[]).is_none());
        let p = script(e.path(), "kicad-cli-nightly", "exit 0");
        assert_eq!(find_kicad_cli_in(Some(d.path().into()), &[e.path().to_path_buf()]), Some(p));
        let p2 = script(d.path(), "kicad-cli", "exit 0");
        assert_eq!(find_kicad_cli_in(Some(d.path().into()), &[e.path().to_path_buf()]), Some(p2));
    }

    #[test]
    fn early_exit_is_reported() {
        let d = tempfile::tempdir().unwrap();
        let cli = script(d.path(), "kicad-cli", "exit 3");
        let err =
            HeadlessServer::spawn(&cli, None, &d.path().join("s.sock"), Duration::from_secs(5))
                .unwrap_err();
        assert!(matches!(err, IpcError::Protocol(m) if m.contains("exited early")));
    }

    #[test]
    fn never_ready_times_out_and_kills_child() {
        let d = tempfile::tempdir().unwrap();
        let cli = script(d.path(), "kicad-cli", "sleep 30");
        let err =
            HeadlessServer::spawn(&cli, None, &d.path().join("s.sock"), Duration::from_millis(400))
                .unwrap_err();
        assert!(matches!(err, IpcError::Timeout));
    }

    #[test]
    fn spawn_of_missing_binary_is_io_error() {
        let d = tempfile::tempdir().unwrap();
        let err = HeadlessServer::spawn(
            &d.path().join("no-such-kicad-cli"),
            None,
            &d.path().join("s.sock"),
            Duration::from_secs(1),
        )
        .unwrap_err();
        assert!(matches!(err, IpcError::Io(_)), "{err:?}");
    }

    #[test]
    fn spawn_passes_arguments_and_reports_exit_status() {
        let d = tempfile::tempdir().unwrap();
        let out = d.path().join("args.txt");
        let cli =
            script(d.path(), "kicad-cli", &format!("echo \"$@\" > '{}'\nexit 7", out.display()));
        let sock = d.path().join("s.sock");
        let board = d.path().join("b.kicad_pcb");
        let err =
            HeadlessServer::spawn(&cli, Some(&board), &sock, Duration::from_secs(5)).unwrap_err();
        assert!(matches!(&err, IpcError::Protocol(m) if m.contains("exited early")), "{err:?}");
        let args = std::fs::read_to_string(&out).unwrap();
        assert_eq!(
            args.trim(),
            format!("api-server --socket {} {}", sock.display(), board.display())
        );
    }

    #[test]
    fn ready_server_exposes_endpoint_pid_and_cleans_up_on_drop() {
        use std::os::unix::net::UnixListener;

        use prost::Message;

        use crate::nng::{self, PROTO_REP0, PROTO_REQ0};
        use crate::proto::*;

        let d = tempfile::tempdir().unwrap();
        let cli = script(d.path(), "kicad-cli", "sleep 30");
        let sock = d.path().join("s.sock");
        // Stand in for the api-server: answer the readiness Ping.
        let l = UnixListener::bind(&sock).unwrap();
        let h = std::thread::spawn(move || {
            let (mut s, _) = l.accept().unwrap();
            nng::handshake(&mut s, PROTO_REP0, PROTO_REQ0).unwrap();
            let msg = nng::read_msg(&mut s).unwrap();
            let mut reply = msg[..4].to_vec();
            reply.extend(
                ApiResponse {
                    header: None,
                    status: Some(ApiResponseStatus {
                        status: ApiStatusCode::Ok as i32,
                        error_message: String::new(),
                    }),
                    message: Some(Any::pack(&Empty {})),
                }
                .encode_to_vec(),
            );
            nng::write_msg(&mut s, &reply).unwrap();
            // keep the connection until the client side drops it
            let _ = nng::read_msg(&mut s);
        });
        let server = HeadlessServer::spawn(&cli, None, &sock, Duration::from_secs(10)).unwrap();
        assert_eq!(server.endpoint().socket_path(), sock);
        assert!(server.pid() > 0);
        drop(server);
        assert!(!sock.exists(), "socket file removed on drop");
        h.join().unwrap();
    }

    #[test]
    fn common_install_dirs_are_absolute_and_nonempty() {
        let v = common_install_dirs();
        assert!(!v.is_empty());
        assert!(v.iter().all(|p| p.is_absolute()));
    }

    #[test]
    fn find_kicad_cli_ignores_directories_and_prefers_path_order() {
        let a = tempfile::tempdir().unwrap();
        let b = tempfile::tempdir().unwrap();
        // A directory named like the binary must not match.
        std::fs::create_dir(a.path().join("kicad-cli")).unwrap();
        assert!(find_kicad_cli_in(Some(a.path().into()), &[]).is_none());
        let pb = script(b.path(), "kicad-cli", "exit 0");
        let joined = std::env::join_paths([a.path(), b.path()]).unwrap();
        assert_eq!(find_kicad_cli_in(Some(joined), &[]), Some(pb));
        assert!(find_kicad_cli_in(None, &[]).is_none());
    }

    #[test]
    fn find_kicad_cli_does_not_panic() {
        let _ = find_kicad_cli();
    }
}
