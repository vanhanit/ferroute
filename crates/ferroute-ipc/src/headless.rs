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
}
