// SPDX-License-Identifier: AGPL-3.0-only
//! KiCad 10/11 IPC API client (NNG SP over IPC, protobuf) with capability negotiation.
//!
//! Layers: [`nng`] (SP framing) -> [`transport`] -> [`client`] (typed calls over [`proto`])
//! -> [`caps`] (negotiation). [`headless`] spawns `kicad-cli api-server` (KiCad 11+).

pub mod caps;
pub mod client;
pub mod endpoint;
pub mod error;
pub mod headless;
pub mod nng;
pub mod proto;
pub mod transport;

pub use caps::{ApiGeneration, Capabilities, KiCadVersionInfo, ProbeReport, negotiate};
pub use client::KiCadClient;
pub use endpoint::Endpoint;
pub use error::IpcError;
pub use headless::HeadlessServer;
pub use transport::{MockTransport, NngIpcTransport, Transport};
