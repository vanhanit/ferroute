// SPDX-License-Identifier: AGPL-3.0-only
//! # ferroute-dsn
//!
//! Specctra interchange for comparisons with Freerouting and other external routers.
//!
//! * [`write_dsn`]: [`ferroute_board::Board`] to a Specctra DSN design file.
//! * [`read_ses`]: Specctra SES session (routes) to an additive
//!   [`ferroute_board::BoardEdit`] (INV-EDIT-1) that the KiCad writer can apply.
//!
//! ## Conventions
//!
//! * The DSN uses `(resolution um 1)`: one design unit is 1 um (finer overflows Freerouting
//!   2.5.0, coarser makes its SES writer scale coordinates twice). Positions round to nearest, sizes (widths,
//!   clearances, diameters, pad extents) round up. SES files with any `nm`/`um`/`mm`/`cm`/`inch`/
//!   `mil` resolution are converted with exact integer arithmetic (decimal coordinates
//!   included, rounded half away from zero to the nanometre).
//! * Specctra is y-up, ferroute (KiCad) is y-down: `y` is negated on export and import.
//! * Every footprint becomes its own image placed at rotation 0 on the front side with
//!   its pads at their absolute offsets, so rotation and side are baked into the image.
//!   Oval pads are exact stadium paths; rounded rectangles are exported as their
//!   bounding rectangle (a conservative superset). Arc tracks are exported as their
//!   start-mid-end polyline (protected wiring only).
//! * Existing copper is exported as `(type protect)` wiring; SES wires/vias of type
//!   `protect`/`fix` and exact duplicates of existing copper are not re-imported.

mod error;
mod ses;
mod sexp;
mod units;
mod write;

pub use error::DsnError;
pub use ses::read_ses;
pub use write::write_dsn;
