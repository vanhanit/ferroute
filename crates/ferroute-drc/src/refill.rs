// SPDX-License-Identifier: AGPL-3.0-only
//! Copper-pour refill: recompute stored zone fills around the copper routing added.
//!
//! For every zone polygon that new (or moved) other-net copper reaches:
//!
//! 1. `new = old − ⋃ circumscribed(item ⊕ max(zone clearance, pair clearance))`. The
//!    obstacle polygons *contain* the true keep-out region
//!    ([`ferroute_geom::boolean::circumscribed`]), so the new fill is a subset of the legal
//!    fill (INV-DRC-1 spirit). Same-net copper is never cut.
//! 2. KiCad-like cleanup: a morphological opening by `min_thickness` drops slivers and
//!    thin necks (the result is intersected with the pre-opening fill, so it only ever
//!    shrinks), then islands connected to no same-net pad/via/track are removed according to
//!    the zone's island mode.
//!
//! Polygons no obstacle touches keep their stored vertices byte for byte, and zones with no
//! change are not reported. The function is pure and deterministic.

use crate::connectivity::CONTACT_TOLERANCE;
use crate::{BucketGrid, CopperItem, ItemRef, copper_items};
use ferroute_board::{Board, IslandRemoval, LayerId, NetId, Zone, ZoneFill};
use ferroute_geom::boolean::{
    Region, SAFETY_NM, bridge, circumscribed, difference, intersection, offset, regions_of,
};
use ferroute_geom::{Coord, Polygon, Rect};

/// Result of [`refill_zones`].
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Refill {
    /// `(index into Board::zones, every fill of that zone)` for each zone that changed.
    pub fills: Vec<(u32, Vec<ZoneFill>)>,
    /// Human-readable notes (regions dropped because no hole bridge exists, …).
    pub notes: Vec<String>,
}

/// Recomputes the fills of every zone of `board` (the board *after* routing).
#[must_use]
pub fn refill_zones(board: &Board) -> Refill {
    let mut out = Refill::default();
    if board.zones.iter().all(Zone::is_unfilled) {
        return out;
    }
    let items = copper_items(board);
    let boxes: Vec<Rect> = items.iter().map(|it| it.bbox).collect();
    let grid = BucketGrid::new(&boxes, 2_000_000);
    for (zi, zone) in board.zones.iter().enumerate() {
        if zone.is_unfilled() {
            continue;
        }
        let ctx = Ctx { board, zone, items: &items, boxes: &boxes, grid: &grid };
        let mut changed = false;
        let mut fills = Vec::with_capacity(zone.fills.len());
        for f in &zone.fills {
            let (polys, ch) = ctx.refill_layer(f, &mut out.notes);
            changed |= ch;
            fills.push(ZoneFill { layer: f.layer, polys });
        }
        if changed {
            out.fills.push((u32::try_from(zi).unwrap_or(u32::MAX), fills));
        }
    }
    out
}

struct Ctx<'a> {
    board: &'a Board,
    zone: &'a Zone,
    items: &'a [CopperItem],
    boxes: &'a [Rect],
    grid: &'a BucketGrid,
}

impl Ctx<'_> {
    fn reach(&self) -> Coord {
        self.zone.clearance.max(self.board.rules.max_clearance_for_net(self.board, self.zone.net))
            + 4 * SAFETY_NM
            + 1
    }

    /// Indices of items of other nets on `layer` whose box meets `bb` inflated by the reach.
    fn nearby(&self, bb: Rect, layer: LayerId) -> Vec<usize> {
        let mut v: Vec<usize> = self
            .grid
            .query(bb.inflate(self.reach()), self.boxes)
            .into_iter()
            .map(|k| k as usize)
            .filter(|&k| {
                let it = &self.items[k];
                it.layers.contains(layer)
                    && !(it.net == self.zone.net && self.zone.net != NetId::NONE)
            })
            .collect();
        v.sort_unstable();
        v
    }

    fn obstacles(&self, poly: &Polygon, bb: Rect, layer: LayerId) -> Vec<Polygon> {
        self.nearby(bb, layer)
            .into_iter()
            .filter_map(|k| {
                let it = &self.items[k];
                let req = crate::zones::zone_clearance(
                    self.board,
                    self.zone.net,
                    self.zone.clearance,
                    layer,
                    it.net,
                );
                let c = circumscribed(&it.hull, req);
                (c.bbox()?.intersects(bb) && poly.intersects(&c)).then_some(c)
            })
            .collect()
    }

    /// New polygons of one fill layer and whether anything changed.
    fn refill_layer(&self, fill: &ZoneFill, notes: &mut Vec<String>) -> (Vec<Polygon>, bool) {
        let mut out = Vec::with_capacity(fill.polys.len());
        let mut changed = false;
        for poly in &fill.polys {
            let Some(bb) = poly.bbox() else { continue };
            let obs = self.obstacles(poly, bb, fill.layer);
            if obs.is_empty() {
                out.push(poly.clone());
                continue;
            }
            changed = true;
            let mut regions = difference(&regions_of(std::slice::from_ref(poly)), &obs);
            regions = self.open(regions);
            for r in regions {
                if !self.keeps(&r, fill.layer) {
                    continue;
                }
                match bridge(&r) {
                    Some(p) => out.push(p),
                    None => notes.push(format!(
                        "zone fill region dropped on layer {}: no hole bridge found",
                        fill.layer.0
                    )),
                }
            }
        }
        (out, changed)
    }

    /// Morphological opening by the zone's `min_thickness` (never grows the copper).
    fn open(&self, regions: Vec<Region>) -> Vec<Region> {
        let t = self.zone.min_thickness;
        if t <= 10 || regions.is_empty() {
            return regions;
        }
        let r = (t - 10) / 2;
        let opened = offset(&offset(&regions, -r), r);
        intersection(&opened, &regions)
    }

    /// Island policy: `true` when the region stays.
    fn keeps(&self, r: &Region, layer: LayerId) -> bool {
        match self.zone.islands {
            IslandRemoval::Never => true,
            IslandRemoval::Always => self.connected(r, layer),
            IslandRemoval::BelowArea(a) => r.area2() >= 2 * a || self.connected(r, layer),
        }
    }

    /// `true` when a same-net pad (allowed to join by the zone), track, arc or via on
    /// `layer` touches the region.
    fn connected(&self, r: &Region, layer: LayerId) -> bool {
        if self.zone.net == NetId::NONE {
            return false;
        }
        let Some(bb) = r.bbox() else { return false };
        let outer = Polygon::new(r.outer.clone());
        let holes: Vec<Polygon> = r.holes.iter().map(|h| Polygon::new(h.clone())).collect();
        self.grid.query(bb.inflate(CONTACT_TOLERANCE), self.boxes).into_iter().any(|k| {
            let it = &self.items[k as usize];
            if it.net != self.zone.net || !it.layers.contains(layer) {
                return false;
            }
            if let ItemRef::Pad(p) = it.item
                && !self.zone.pad_connection.joins(self.board.pad(p).drill.is_some())
            {
                return false;
            }
            outer.hull_violates(&it.hull, CONTACT_TOLERANCE)
                && !holes
                    .iter()
                    .any(|h| h.contains_hull_with_clearance(&it.hull, CONTACT_TOLERANCE))
        })
    }
}
