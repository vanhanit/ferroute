// SPDX-License-Identifier: AGPL-3.0-only
//! Electrical models behind the per-net rules (task #20/#20b): current-carrying width
//! (IPC-2152), edge-coupled microstrip (Kirschning–Jansen / Hammerstad–Jensen static
//! even/odd modes), narrow-strip and offset stripline (Cohn/Wheeler/Wadell) and coplanar
//! waveguide (Wadell). Everything here is deterministic (INV-DET-3): only `+ − × ÷ √` and
//! `ferroute_geom::detmath::{exp, ln, pow}`.
//!
//! The single-line building blocks (`microstrip_hj`, `stripline_z0`, …) stay in
//! [`crate::constraints`]; this module holds the models that sit on top of them.

pub mod coupled;
pub mod cpw;
pub mod current;
pub mod stripline;

#[cfg(test)]
mod validation;

pub use coupled::{CoupledMicrostrip, CoupledModel, coupled_microstrip_kj, microstrip_mask_z0};
pub use cpw::{cpw_width, cpw_z0};
pub use current::{CurrentModel, width_for_current_2152, width_for_current_model};
pub use stripline::{offset_stripline_z0, stripline_z0_narrow};
