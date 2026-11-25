// SPDX-License-Identifier: AGPL-3.0-only
//! Freerouting 2.5.0 router settings that a Specctra DSN cannot express.
//!
//! Findings (verified with the real Freerouting 2.5.0 on the dense fixtures, see
//! `docs/FREEROUTING.md`):
//!
//! * **Widths.** The per-class `(rule (width W))` is only the *default* width. Freerouting
//!   narrows traces to 3/4, 3/5 or 1/2 of it near pads (fanout "micro neckdown" and, when
//!   `automatic_neckdown` is on, regular neckdown). No DSN keyword turns this off; it is
//!   controlled by `--router.fanout.enabled=false` and `--router.automatic_neckdown=false`.
//! * **Hole clearance.** The DSN has no drill-to-copper rule that Freerouting reads; it is
//!   `--router.hole_clearance_um`.
//! * **Pours.** `(plane net ...)` makes the pour connectable for its own net, but other
//!   nets still route through it unless `--router.plane_as_obstacle=true`.

use ferroute_board::{Board, NetId};

/// Router settings for one board, rendered as Freerouting command-line arguments.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FreeroutingSettings {
    /// Minimum copper-to-drill clearance in whole micrometres (rounded up).
    pub hole_clearance_um: i64,
    /// Other nets must treat the exported `(plane ...)` pours as obstacles.
    pub plane_as_obstacle: bool,
}

impl FreeroutingSettings {
    /// Command-line arguments (`--router.<key>=<value>`), in a fixed order.
    ///
    /// Neckdown and the fanout stage are always switched off: both narrow traces below the
    /// net's required width, which no DSN rule can prevent.
    #[must_use]
    pub fn args(&self) -> Vec<String> {
        vec![
            "--router.fanout.enabled=false".to_owned(),
            "--router.automatic_neckdown=false".to_owned(),
            format!("--router.hole_clearance_um={}", self.hole_clearance_um),
            format!("--router.plane_as_obstacle={}", self.plane_as_obstacle),
        ]
    }
}

/// Settings that make Freerouting honour `board`'s rules (see the module documentation).
#[must_use]
pub fn freerouting_settings(board: &Board) -> FreeroutingSettings {
    let nm = board.rules.hole_clearance.max(0);
    FreeroutingSettings {
        hole_clearance_um: (nm + 999) / 1000,
        plane_as_obstacle: board.zones.iter().any(|z| z.net != NetId::NONE),
    }
}
