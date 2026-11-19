// SPDX-License-Identifier: AGPL-3.0-only
//! Locating a KiCad API server.

use std::path::PathBuf;

use serde::Serialize;

use crate::error::IpcError;

/// Env var KiCad sets for plugins: full `ipc://...` URL of the API socket.
pub const ENV_SOCKET: &str = "KICAD_API_SOCKET";
/// Env var KiCad sets for plugins: instance token.
pub const ENV_TOKEN: &str = "KICAD_API_TOKEN";

/// Where to reach a KiCad API server, plus the optional instance token.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct Endpoint {
    /// NNG URL, always `ipc://...`.
    pub url: String,
    /// Value for `ApiRequestHeader.kicad_token`.
    pub token: Option<String>,
}

impl Endpoint {
    /// Parse `ipc:///tmp/x.sock` or a bare filesystem path `/tmp/x.sock`.
    ///
    /// # Errors
    /// [`IpcError::Unsupported`] for empty input or non-`ipc` URL schemes.
    pub fn parse(s: &str) -> Result<Self, IpcError> {
        let s = s.trim();
        if s.is_empty() {
            return Err(IpcError::Unsupported("empty endpoint".into()));
        }
        if let Some(rest) = s.strip_prefix("ipc://") {
            if rest.is_empty() {
                return Err(IpcError::Unsupported("ipc:// endpoint without a path".into()));
            }
            return Ok(Self { url: s.to_owned(), token: None });
        }
        if s.contains("://") {
            return Err(IpcError::Unsupported(format!("only ipc:// endpoints are supported: {s}")));
        }
        Ok(Self { url: format!("ipc://{s}"), token: None })
    }

    /// Attach a token.
    #[must_use]
    pub fn with_token(mut self, token: Option<String>) -> Self {
        self.token = token.filter(|t| !t.is_empty());
        self
    }

    /// Filesystem path (Unix) / pipe name payload (Windows) of the socket.
    #[must_use]
    pub fn socket_path(&self) -> PathBuf {
        PathBuf::from(self.url.strip_prefix("ipc://").unwrap_or(&self.url))
    }

    /// Platform default endpoint (`/tmp/kicad/api.sock`, `%TEMP%\kicad\api.sock`).
    #[must_use]
    pub fn platform_default(env: &dyn Fn(&str) -> Option<String>) -> Self {
        let path = if cfg!(windows) {
            let temp =
                env("TEMP").or_else(|| env("TMP")).unwrap_or_else(|| "C:\\Windows\\Temp".into());
            format!("{}\\kicad\\api.sock", temp.trim_end_matches(['\\', '/']))
        } else {
            "/tmp/kicad/api.sock".to_owned()
        };
        Self { url: format!("ipc://{path}"), token: None }
    }

    /// Discover from `KICAD_API_SOCKET` / `KICAD_API_TOKEN`, else the platform default.
    /// `env` is injectable for tests.
    #[must_use]
    pub fn discover_with(env: &dyn Fn(&str) -> Option<String>) -> Self {
        let token = env(ENV_TOKEN);
        let base = match env(ENV_SOCKET).map(|s| Self::parse(&s)) {
            Some(Ok(ep)) => ep,
            _ => Self::platform_default(env),
        };
        base.with_token(token)
    }

    /// [`Self::discover_with`] using the process environment.
    #[must_use]
    pub fn discover() -> Self {
        Self::discover_with(&|k| std::env::var(k).ok())
    }

    /// Candidate endpoints: the discovered one first, then any `api-<pid>.sock` files
    /// found next to the default socket (several KiCad instances may be running).
    #[must_use]
    pub fn candidates(env: &dyn Fn(&str) -> Option<String>) -> Vec<Self> {
        let first = Self::discover_with(env);
        let mut out = vec![first.clone()];
        if let Some(dir) = first.socket_path().parent()
            && let Ok(rd) = std::fs::read_dir(dir)
        {
            let mut extra: Vec<Self> = rd
                .flatten()
                .map(|e| e.path())
                .filter(|p| {
                    p.file_name()
                        .and_then(|n| n.to_str())
                        .is_some_and(|n| n.starts_with("api") && n.ends_with(".sock"))
                })
                .filter_map(|p| Self::parse(&p.to_string_lossy()).ok())
                .map(|e| e.with_token(first.token.clone()))
                .filter(|e| e.url != first.url)
                .collect();
            extra.sort_by(|a, b| a.url.cmp(&b.url));
            out.extend(extra);
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn env_of(pairs: &'static [(&'static str, &'static str)]) -> impl Fn(&str) -> Option<String> {
        move |k| pairs.iter().find(|(n, _)| *n == k).map(|(_, v)| (*v).to_owned())
    }

    #[test]
    fn parse_forms() {
        assert_eq!(Endpoint::parse("ipc:///tmp/x.sock").unwrap().url, "ipc:///tmp/x.sock");
        assert_eq!(Endpoint::parse("/tmp/x.sock").unwrap().url, "ipc:///tmp/x.sock");
        assert_eq!(
            Endpoint::parse("ipc:///tmp/x.sock").unwrap().socket_path(),
            PathBuf::from("/tmp/x.sock")
        );
        assert!(Endpoint::parse("").is_err());
        assert!(Endpoint::parse("ipc://").is_err());
        assert!(Endpoint::parse("tcp://127.0.0.1:1").is_err());
    }

    #[test]
    fn discover_from_env() {
        let ep = Endpoint::discover_with(&env_of(&[
            (ENV_SOCKET, "ipc:///run/k.sock"),
            (ENV_TOKEN, "tok"),
        ]));
        assert_eq!(ep.url, "ipc:///run/k.sock");
        assert_eq!(ep.token.as_deref(), Some("tok"));
    }

    #[test]
    fn discover_default_and_bad_env() {
        let ep = Endpoint::discover_with(&env_of(&[]));
        assert!(ep.url.starts_with("ipc://") && ep.url.ends_with("api.sock"));
        assert_eq!(ep.token, None);
        // Unparsable socket value falls back to the default; empty token is dropped.
        let ep = Endpoint::discover_with(&env_of(&[(ENV_SOCKET, "tcp://x:1"), (ENV_TOKEN, "")]));
        assert!(ep.url.ends_with("api.sock"));
        assert_eq!(ep.token, None);
    }

    #[cfg(unix)]
    #[test]
    fn default_is_tmp_kicad() {
        assert_eq!(Endpoint::platform_default(&|_| None).url, "ipc:///tmp/kicad/api.sock");
    }
}
