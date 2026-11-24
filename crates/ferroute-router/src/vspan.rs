// SPDX-License-Identifier: AGPL-3.0-only
//! Via spans: blind, buried and micro vias (opt-in, extra fabrication cost).
//!
//! A layer change of a lattice path between grid layers `lo < hi` at one planar site is a
//! via spanning exactly the copper layers `lo..=hi`. Which [`ViaKind`] it is follows from
//! the span alone ([`ViaModel::span_kind`]):
//!
//! * `0..=nl-1` — through (always available when the policy allows it),
//! * adjacent layers starting at an outer layer — micro (else blind),
//! * one end on an outer layer — blind,
//! * both ends inner — buried.
//!
//! [`ViaModel::resolve`] picks the kind of a concrete layer change deterministically from
//! the static legality tables: the span's own kind when enabled and legal on exactly its
//! layers, else a through via when enabled and legal. The search, the reservation and the
//! emitter all call it, so they always agree. With the default policy (through only) the
//! model is inert and every table, cost and reservation equals the pre-span behaviour.

use crate::grid::Grid;
use crate::legality::{ClassGeom, ClassLegality, allows};
use ferroute_board::{LayerSet, NetId, ViaKind, ViaPolicy};
use ferroute_geom::Coord;

/// Default cost factor of a micro via relative to a through via.
pub const MICRO_FACTOR: f32 = 1.5;
/// Default cost factor of a blind or buried via relative to a through via.
pub const SPAN_FACTOR: f32 = 2.0;

/// Which via kinds the lattice may use and what they cost.
#[derive(Clone, Debug)]
pub struct ViaModel {
    pub policy: ViaPolicy,
    /// Number of grid layers.
    pub nl: u32,
    pub micro_factor: f32,
    pub span_factor: f32,
}

/// One routed via of a lattice route: its site, span and kind.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct ViaSite {
    /// Planar site `j * nx + i`.
    pub planar: u32,
    /// Grid layers spanned, `lo < hi` (through vias are normalised to the full stack).
    pub lo: u32,
    pub hi: u32,
    pub kind: ViaKind,
}

impl ViaModel {
    /// The inert model: through vias only.
    #[must_use]
    pub fn through_only() -> Self {
        Self {
            policy: ViaPolicy::default(),
            nl: 0,
            micro_factor: MICRO_FACTOR,
            span_factor: SPAN_FACTOR,
        }
    }

    /// Model of `policy` on `grid`. Spans need the grid to hold every copper layer of the
    /// board (`board_layers`), otherwise a partial span could not be told from a through
    /// via and the policy falls back to through vias.
    #[must_use]
    pub fn new(policy: ViaPolicy, grid: &Grid, board_layers: usize, micro: f32, span: f32) -> Self {
        let full = grid.layers.len() == board_layers
            && grid.layers.iter().enumerate().all(|(i, l)| usize::from(l.0) == i);
        let policy = if full {
            policy
        } else {
            ViaPolicy { blind: false, buried: false, micro: false, micro_in_pad: false, ..policy }
        };
        Self { policy, nl: grid.nl(), micro_factor: micro, span_factor: span }
    }

    /// `true` when any blind / buried / micro via may be created on this grid.
    #[must_use]
    pub fn spans_enabled(&self) -> bool {
        self.nl > 2 && self.policy.any_span()
    }

    /// Index of span `lo..=hi` into [`ClassLegality::via_span`].
    #[must_use]
    pub fn span_index(&self, lo: u32, hi: u32) -> usize {
        (lo * self.nl + hi) as usize
    }

    /// The non-through kind a layer change `lo..=hi` would be, when the policy allows it.
    #[must_use]
    pub fn span_kind(&self, lo: u32, hi: u32) -> Option<ViaKind> {
        if !self.spans_enabled() || lo >= hi || hi >= self.nl || (lo == 0 && hi == self.nl - 1) {
            return None;
        }
        let outer = lo == 0 || hi == self.nl - 1;
        if outer {
            if hi - lo == 1 && self.policy.micro {
                Some(ViaKind::Micro)
            } else if self.policy.blind {
                Some(ViaKind::Blind)
            } else {
                None
            }
        } else if self.policy.buried {
            Some(ViaKind::Buried)
        } else {
            None
        }
    }

    /// Board copper layers of the span `lo..=hi`.
    #[must_use]
    pub fn span_layers(&self, grid: &Grid, lo: u32, hi: u32) -> LayerSet {
        LayerSet::span(grid.layers[lo as usize], grid.layers[hi as usize])
    }

    /// `(diameter, drill)` of a via of `kind` in class `cg`.
    #[must_use]
    pub fn sizes(&self, kind: ViaKind, cg: &ClassGeom) -> (Coord, Coord) {
        match kind {
            ViaKind::Micro => (cg.micro_diameter, cg.micro_drill),
            _ => (cg.via_diameter, cg.via_drill),
        }
    }

    /// Cost factor of a via of `kind`.
    #[must_use]
    pub fn factor(&self, kind: ViaKind) -> f32 {
        match kind {
            ViaKind::Through => 1.0,
            ViaKind::Micro => self.micro_factor,
            ViaKind::Blind | ViaKind::Buried => self.span_factor,
        }
    }

    /// Kind of the via a path takes between grid layers `a` and `b` at `planar`, or `None`
    /// when no allowed kind is legal there for `net`.
    #[must_use]
    pub fn resolve(
        &self,
        leg: &ClassLegality,
        net: NetId,
        planar: u32,
        a: u32,
        b: u32,
    ) -> Option<ViaKind> {
        let (lo, hi) = (a.min(b), a.max(b));
        if let Some(kind) = self.span_kind(lo, hi)
            && leg
                .via_span
                .get(self.span_index(lo, hi))
                .and_then(|t| t.get(planar as usize))
                .is_some_and(|&s| allows(s, net))
        {
            return Some(kind);
        }
        (self.policy.through && allows(leg.via[planar as usize], net)).then_some(ViaKind::Through)
    }

    /// The [`ViaSite`] of a layer change between `a` and `b` at `planar` (through vias are
    /// normalised to the full stack). With no legal kind (cannot happen for a searched
    /// path) a through via is assumed, which keeps reservations conservative.
    #[must_use]
    pub fn site(&self, leg: &ClassLegality, net: NetId, planar: u32, a: u32, b: u32) -> ViaSite {
        let (lo, hi) = (a.min(b), a.max(b));
        match self.resolve(leg, net, planar, a, b) {
            Some(kind) if kind != ViaKind::Through => ViaSite { planar, lo, hi, kind },
            _ => ViaSite { planar, lo: 0, hi: self.nl.max(1) - 1, kind: ViaKind::Through },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ferroute_board::LayerId;

    fn model(policy: ViaPolicy, nl: u32) -> ViaModel {
        ViaModel { policy, nl, micro_factor: MICRO_FACTOR, span_factor: SPAN_FACTOR }
    }

    fn all() -> ViaPolicy {
        ViaPolicy { through: true, blind: true, buried: true, micro: true, micro_in_pad: false }
    }

    #[test]
    fn default_policy_is_inert() {
        let m = model(ViaPolicy::default(), 6);
        assert!(!m.spans_enabled());
        for lo in 0..6 {
            for hi in lo + 1..6 {
                assert_eq!(m.span_kind(lo, hi), None);
            }
        }
    }

    #[test]
    fn span_kinds_follow_the_stack_position() {
        let m = model(all(), 6);
        assert_eq!(m.span_kind(0, 5), None, "full span is a through via");
        assert_eq!(m.span_kind(0, 1), Some(ViaKind::Micro));
        assert_eq!(m.span_kind(4, 5), Some(ViaKind::Micro));
        assert_eq!(m.span_kind(0, 3), Some(ViaKind::Blind));
        assert_eq!(m.span_kind(2, 5), Some(ViaKind::Blind));
        assert_eq!(m.span_kind(1, 4), Some(ViaKind::Buried));
        assert_eq!(m.span_kind(1, 2), Some(ViaKind::Buried), "adjacent inner layers: buried");
    }

    #[test]
    fn disabled_kinds_have_no_span() {
        let blind_only = ViaPolicy { blind: true, ..ViaPolicy::default() };
        let m = model(blind_only, 4);
        assert_eq!(m.span_kind(0, 1), Some(ViaKind::Blind), "no micro: adjacent is blind");
        assert_eq!(m.span_kind(1, 2), None, "buried is off");
        let micro_only = ViaPolicy { micro: true, ..ViaPolicy::default() };
        let m = model(micro_only, 4);
        assert_eq!(m.span_kind(0, 1), Some(ViaKind::Micro));
        assert_eq!(m.span_kind(0, 2), None, "a micro via spans one layer pair only");
    }

    #[test]
    fn two_layer_boards_have_only_through_vias() {
        assert!(!model(all(), 2).spans_enabled());
    }

    #[test]
    fn partial_grids_fall_back_to_through() {
        let grid = Grid::covering(
            ferroute_geom::Rect::new(
                ferroute_geom::Point::ORIGIN,
                ferroute_geom::Point::new(10, 10),
            ),
            1,
            vec![LayerId(0), LayerId(2)],
        );
        let m = ViaModel::new(all(), &grid, 4, 1.5, 2.0);
        assert!(!m.spans_enabled());
        assert!(m.policy.through);
    }
}
