// SPDX-License-Identifier: AGPL-3.0-only
//! Design rules (net classes + board-level constraints), mirroring the subset of KiCad's
//! board setup / net-class settings that routing and DRC depend on.

use crate::{Board, NetId};
use ferroute_geom::Coord;
use serde::{Deserialize, Serialize};

/// Net-class rules (KiCad: Board Setup → Net Classes).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct NetClass {
    pub name: String,
    pub clearance: Coord,
    pub track_width: Coord,
    pub via_diameter: Coord,
    pub via_drill: Coord,
}

impl Default for NetClass {
    /// KiCad's default net class values.
    fn default() -> Self {
        Self {
            name: "Default".into(),
            clearance: 200_000,
            track_width: 250_000,
            via_diameter: 600_000,
            via_drill: 300_000,
        }
    }
}

/// Board-level constraints.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DesignRules {
    /// `classes[0]` is the default class.
    pub classes: Vec<NetClass>,
    /// Copper to board-edge clearance.
    pub edge_clearance: Coord,
    /// Minimum drill-to-drill (hole-to-hole) spacing.
    pub hole_to_hole: Coord,
    /// Minimum copper-to-hole clearance for holes of other nets.
    pub hole_clearance: Coord,
    pub min_track_width: Coord,
    pub min_via_diameter: Coord,
    pub min_via_drill: Coord,
}

impl Default for DesignRules {
    /// KiCad 10 board-setup defaults.
    fn default() -> Self {
        Self {
            classes: vec![NetClass::default()],
            edge_clearance: 500_000,
            hole_to_hole: 250_000,
            hole_clearance: 250_000,
            min_track_width: 0,
            min_via_diameter: 500_000,
            min_via_drill: 300_000,
        }
    }
}

impl DesignRules {
    /// Net class of `net` (falls back to the default class for out-of-range indices).
    #[must_use]
    pub fn class_of<'a>(&'a self, board: &Board, net: NetId) -> &'a NetClass {
        let idx = board.nets.get(net.idx()).map_or(0, |n| usize::from(n.class));
        self.classes.get(idx).unwrap_or(&self.classes[0])
    }

    /// Largest clearance over all classes.
    #[must_use]
    pub fn max_clearance(&self) -> Coord {
        self.classes.iter().map(|c| c.clearance).max().unwrap_or(0)
    }

    /// Largest track width over all classes.
    #[must_use]
    pub fn max_track_width(&self) -> Coord {
        self.classes.iter().map(|c| c.track_width).max().unwrap_or(0)
    }

    /// Largest via diameter over all classes.
    #[must_use]
    pub fn max_via_diameter(&self) -> Coord {
        self.classes.iter().map(|c| c.via_diameter).max().unwrap_or(0)
    }
}
