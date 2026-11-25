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
    Region, SAFETY_NM, bridge, circumscribed, difference, intersection, offset, offset_bevel,
    regions_of,
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
        // Bevelled grow: a mitred one would restore the acute tips of a neck between two
        // round obstacles and leave necks KiCad's own fill removes.
        let opened = offset_bevel(&offset(&regions, -r), r);
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

/// A foreign item that walls a pour: within neck reach of two or more separated parts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Cutter {
    pub item: ItemRef,
    pub net: NetId,
    pub layer: LayerId,
    pub bbox: Rect,
    /// `false` for pre-existing copper (it cannot be re-routed).
    pub introduced: bool,
}

/// A pour net whose refill separated pads that the stored fill (plus copper) joined.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PourSplit {
    /// The poured net.
    pub net: NetId,
    /// Number of pad groups the net falls apart into inside the affected old component(s).
    pub groups: usize,
    /// Foreign items within neck reach of two or more groups, in item order.
    pub cutters: Vec<Cutter>,
}

impl PourSplit {
    /// Cutters that routing can move (introduced copper).
    pub fn movable(&self) -> impl Iterator<Item = &Cutter> {
        self.cutters.iter().filter(|c| c.introduced)
    }
}

struct NetCopper<'a> {
    items: &'a [CopperItem],
    ids: Vec<usize>,
    pairs: Vec<(u32, u32)>,
}

/// Union-find over `ids` (positions in `net.ids`) and polygons; returns the root of every
/// `ids` entry and of every polygon.
fn components(
    board: &Board,
    net: &NetCopper<'_>,
    polys: &[(usize, LayerId, &Polygon, Rect)],
) -> (Vec<u32>, Vec<u32>) {
    let n = net.ids.len();
    let mut uf = crate::connectivity::UnionFind::new(n + polys.len());
    for &(a, b) in &net.pairs {
        uf.union(a, b);
    }
    for (pi, &(zi, layer, poly, bb)) in polys.iter().enumerate() {
        let zone = &board.zones[zi];
        let bb = bb.inflate(CONTACT_TOLERANCE);
        for (k, &id) in net.ids.iter().enumerate() {
            let it = &net.items[id];
            if !it.layers.contains(layer) || !it.bbox.intersects(bb) {
                continue;
            }
            if let ItemRef::Pad(p) = it.item
                && !zone.pad_connection.joins(board.pad(p).drill.is_some())
            {
                continue;
            }
            if poly.hull_violates(&it.hull, CONTACT_TOLERANCE) {
                uf.union(u32::try_from(k).unwrap_or(0), u32::try_from(n + pi).unwrap_or(0));
            }
        }
    }
    let item_roots = (0..n).map(|k| uf.find(u32::try_from(k).unwrap_or(0))).collect();
    let poly_roots = (0..polys.len()).map(|p| uf.find(u32::try_from(n + p).unwrap_or(0))).collect();
    (item_roots, poly_roots)
}

/// Diagnoses the pour splits a refill causes. `board` is the board after routing with its
/// stored (old) fills; `refill` is the result of [`refill_zones`] on it. For every poured net
/// whose pads fall into more groups than before, lists the foreign items that wall the pour
/// (within `clearance + min_thickness` of two or more groups). Deterministic.
#[must_use]
pub fn pour_splits(board: &Board, refill: &Refill) -> Vec<PourSplit> {
    let items = copper_items(board);
    let mut nets: Vec<NetId> =
        refill.fills.iter().map(|(zi, _)| board.zones[*zi as usize].net).collect();
    nets.sort_unstable();
    nets.dedup();
    let mut out = Vec::new();
    for net in nets {
        if net == NetId::NONE {
            continue;
        }
        let ids: Vec<usize> = (0..items.len()).filter(|&k| items[k].net == net).collect();
        let nb: Vec<Rect> = ids.iter().map(|&k| items[k].bbox.inflate(CONTACT_TOLERANCE)).collect();
        let ngrid = BucketGrid::new(&nb, 2_000_000);
        let pairs: Vec<(u32, u32)> = ngrid
            .pairs(&nb)
            .into_iter()
            .filter(|&(a, b)| {
                let (x, y) = (&items[ids[a as usize]], &items[ids[b as usize]]);
                x.layers.intersects(y.layers)
                    && x.hull.clearance_violated(&y.hull, CONTACT_TOLERANCE)
            })
            .collect();
        let nc = NetCopper { items: &items, ids, pairs };
        let mut old: Vec<(usize, LayerId, &Polygon, Rect)> = Vec::new();
        let mut new: Vec<(usize, LayerId, &Polygon, Rect)> = Vec::new();
        for (zi, z) in board.zones.iter().enumerate().filter(|(_, z)| z.net == net) {
            let repl = refill.fills.iter().find(|(i, _)| *i as usize == zi).map(|(_, f)| f);
            for f in &z.fills {
                for p in &f.polys {
                    if let Some(bb) = p.bbox() {
                        old.push((zi, f.layer, p, bb));
                    }
                }
            }
            for f in repl.map_or(&z.fills, |f| f) {
                for p in &f.polys {
                    if let Some(bb) = p.bbox() {
                        new.push((zi, f.layer, p, bb));
                    }
                }
            }
        }
        let (old_roots, _) = components(board, &nc, &old);
        let (new_roots, new_poly_roots) = components(board, &nc, &new);
        // Pads grouped by old component; count distinct new roots among them.
        let pad_ks: Vec<usize> = (0..nc.ids.len())
            .filter(|&k| matches!(items[nc.ids[k]].item, ItemRef::Pad(_)))
            .collect();
        let mut by_old: std::collections::BTreeMap<u32, Vec<u32>> =
            std::collections::BTreeMap::new();
        for &k in &pad_ks {
            by_old.entry(old_roots[k]).or_default().push(new_roots[k]);
        }
        let mut split_roots: Vec<u32> = Vec::new();
        let mut groups = 0usize;
        for roots in by_old.values_mut() {
            roots.sort_unstable();
            roots.dedup();
            if roots.len() > 1 {
                groups += roots.len();
                split_roots.extend(roots.iter().copied());
            }
        }
        if groups == 0 {
            continue;
        }
        split_roots.sort_unstable();
        split_roots.dedup();
        let mut cutters: Vec<Cutter> = Vec::new();
        for it in &items {
            if it.net == net {
                continue;
            }
            let mut seen: Vec<(u32, LayerId)> = Vec::new();
            for (pi, &(zi, layer, poly, bb)) in new.iter().enumerate() {
                let root = new_poly_roots[pi];
                if !it.layers.contains(layer) || split_roots.binary_search(&root).is_err() {
                    continue;
                }
                let zone = &board.zones[zi];
                let reach = crate::zones::zone_clearance(board, net, zone.clearance, layer, it.net)
                    + zone.min_thickness;
                if !it.bbox.intersects(bb.inflate(reach)) || !poly.hull_violates(&it.hull, reach) {
                    continue;
                }
                if !seen.iter().any(|&(r, l)| r == root && l == layer) {
                    seen.push((root, layer));
                }
            }
            for layer in seen.iter().map(|&(_, l)| l).collect::<std::collections::BTreeSet<_>>() {
                let distinct = {
                    let mut r: Vec<u32> =
                        seen.iter().filter(|&&(_, l)| l == layer).map(|&(r, _)| r).collect();
                    r.sort_unstable();
                    r.dedup();
                    r.len()
                };
                if distinct >= 2 && !cutters.iter().any(|c| c.item == it.item && c.layer == layer) {
                    cutters.push(Cutter {
                        item: it.item,
                        net: it.net,
                        layer,
                        bbox: it.bbox,
                        introduced: it.introduced,
                    });
                }
            }
        }
        out.push(PourSplit { net, groups, cutters });
    }
    out
}
