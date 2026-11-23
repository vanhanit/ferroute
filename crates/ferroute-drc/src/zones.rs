// SPDX-License-Identifier: AGPL-3.0-only
//! Copper pours: the stored fill polygons of `Board::zones` as copper.
//!
//! Fill polygons are non-convex, so they are tested exactly against item hulls with
//! `Polygon::hull_violates` (no approximation). Only the fill KiCad stored counts;
//! an unfilled zone contributes nothing.

use crate::connectivity::CONTACT_TOLERANCE;
use crate::{BucketGrid, CopperItem, ItemRef, Violation, ViolationKind};
use ferroute_board::{Board, LayerId, NetId};
use ferroute_geom::{Coord, Polygon, Rect};
use rayon::prelude::*;

/// Slack (nm) granted to pre-existing copper against KiCad's polygonised fills.
pub const FILL_TOLERANCE: Coord = 1_000;

/// One stored fill polygon of a zone.
#[derive(Clone, Copy, Debug)]
pub struct FillPoly<'a> {
    /// Index into `Board::zones`.
    pub zone: u32,
    pub net: NetId,
    pub layer: LayerId,
    /// The zone's own clearance to other-net copper.
    pub clearance: Coord,
    pub poly: &'a Polygon,
    pub bbox: Rect,
}

/// Every stored fill polygon of the board in zone order (deterministic).
#[must_use]
pub fn fill_polys(board: &Board) -> Vec<FillPoly<'_>> {
    let mut out = Vec::new();
    for (zi, z) in board.zones.iter().enumerate() {
        for (layer, poly) in z.fill_polys() {
            let Some(bbox) = poly.bbox() else { continue };
            out.push(FillPoly {
                zone: u32::try_from(zi).unwrap_or(u32::MAX),
                net: z.net,
                layer,
                clearance: z.clearance,
                poly,
                bbox,
            });
        }
    }
    out
}

/// Required gap between a fill and an item of net `other`: the larger of the zone's own
/// clearance and the board's pair clearance.
#[must_use]
pub fn required_clearance(board: &Board, f: &FillPoly<'_>, other: NetId) -> Coord {
    f.clearance.max(board.clearance_between(f.net, other))
}

/// Fill-vs-copper clearance and short violations (other-net copper only).
#[must_use]
pub fn check_fills(board: &Board, items: &[CopperItem]) -> Vec<Violation> {
    let fills = fill_polys(board);
    if fills.is_empty() || items.is_empty() {
        return Vec::new();
    }
    let reach =
        fills.iter().map(|f| f.clearance).max().unwrap_or(0).max(board.rules.max_clearance()) + 1;
    let boxes: Vec<Rect> = items.iter().map(|it| it.bbox).collect();
    let grid = BucketGrid::new(&boxes, 2_000_000);
    fills
        .par_iter()
        .flat_map_iter(|f| {
            let mut out = Vec::new();
            for k in grid.query(f.bbox.inflate(reach), &boxes) {
                let it = &items[k as usize];
                if (it.net == f.net && f.net != NetId::NONE) || !it.layers.contains(f.layer) {
                    continue;
                }
                let req = required_clearance(board, f, it.net);
                // KiCad's stored fill is a polygon approximation of the true pour (arc
                // flattening, sub-micron rounding): pre-existing copper gets a small
                // tolerance, anything ferroute adds is held to the exact value.
                let eff = if it.introduced { req } else { (req - FILL_TOLERANCE).max(0) };
                if !f.poly.hull_violates(&it.hull, eff) {
                    continue;
                }
                let overlap = f.poly.hull_violates(&it.hull, 0);
                out.push(Violation {
                    kind: if overlap { ViolationKind::Short } else { ViolationKind::Clearance },
                    a: it.item,
                    b: Some(ItemRef::Zone(f.zone)),
                    layer: Some(f.layer),
                    at: it.bbox.center(),
                    actual: gap(f.poly, it, overlap),
                    required: req,
                    introduced: it.introduced,
                });
            }
            out
        })
        .collect()
}

/// Gap in nm between the polygon and the item (reporting only; 0 on overlap).
fn gap(poly: &Polygon, it: &CopperItem, overlap: bool) -> f64 {
    if overlap {
        return 0.0;
    }
    let mut best = f64::INFINITY;
    for e in poly.edges() {
        let h = ferroute_geom::Hull::capsule(e.a, e.b, 0);
        best = best.min(it.hull.distance(&h));
    }
    best
}

/// Number of fill polygons (nodes appended after the items in connectivity graphs).
#[must_use]
pub fn fill_count(board: &Board) -> usize {
    board.zones.iter().map(|z| z.fill_polys().count()).sum()
}

/// Pairs `(a, b)` to union for connectivity: fill nodes are numbered
/// `items.len() + k` in [`fill_polys`] order. Links fills to same-net items touching them
/// (pads only when the zone's pad connection allows it) and overlapping same-net fills.
#[must_use]
pub fn fill_links(board: &Board, items: &[CopperItem]) -> Vec<(u32, u32)> {
    let fills = fill_polys(board);
    let base = items.len();
    let node = |k: usize| u32::try_from(base + k).unwrap_or(u32::MAX);
    let mut links = Vec::new();
    if fills.is_empty() {
        return links;
    }
    let boxes: Vec<Rect> = items.iter().map(|it| it.bbox).collect();
    let grid = BucketGrid::new(&boxes, 2_000_000);
    for (k, f) in fills.iter().enumerate() {
        if f.net == NetId::NONE {
            continue;
        }
        let zone = &board.zones[f.zone as usize];
        for i in grid.query(f.bbox.inflate(CONTACT_TOLERANCE), &boxes) {
            let it = &items[i as usize];
            if it.net != f.net || !it.layers.contains(f.layer) {
                continue;
            }
            if let ItemRef::Pad(p) = it.item
                && !zone.pad_connection.joins(board.pad(p).drill.is_some())
            {
                continue;
            }
            if f.poly.hull_violates(&it.hull, CONTACT_TOLERANCE) {
                links.push((i, node(k)));
            }
        }
        for (j, g) in fills.iter().enumerate().skip(k + 1) {
            if g.net == f.net
                && g.layer == f.layer
                && f.bbox.intersects(g.bbox)
                && f.poly.intersects(g.poly)
            {
                links.push((node(k), node(j)));
            }
        }
    }
    links
}

/// `true` when copper `hull` of `net` on any of `layers` comes closer than the pair
/// clearance to a fill of another net (exact; for the router's optimisation passes).
#[must_use]
pub fn conflicts(
    board: &Board,
    fills: &[FillPoly<'_>],
    hull: &ferroute_geom::Hull,
    layers: ferroute_board::LayerSet,
    net: NetId,
) -> bool {
    if fills.is_empty() {
        return false;
    }
    let bb = hull.bbox();
    fills.iter().any(|f| {
        (f.net != net || net == NetId::NONE)
            && layers.contains(f.layer)
            && f.bbox.inflate(required_clearance(board, f, net)).intersects(bb)
            && f.poly.hull_violates(hull, required_clearance(board, f, net))
    })
}
