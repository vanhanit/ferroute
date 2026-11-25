// SPDX-License-Identifier: AGPL-3.0-only
//! Starved thermal reliefs (KiCad `starved_thermal`).
//!
//! KiCad 10 (`drc_test_provider_zone_connections.cpp`) checks, per zone layer and per pad of
//! the zone's net that flashes copper on that layer and whose resolved zone connection is
//! *thermal relief*, that at least `min_resolved_spokes` (default 2) spokes reach the fill.
//! A spoke is a pair of crossings between the fill outline and the pad outline grown by
//! half the thermal gap; fill islands that touch nothing but this pad do not count, tracks
//! of the pad's net that leave the pad into the fill count as manual spokes, and a pad with
//! no spoke at all is *not* reported (the connectivity check owns that case).
//!
//! The board model carries neither the thermal gap nor per-pad overrides, so the crossing
//! test is replaced by an equivalent that needs no gap: the fill is intersected with thin
//! rings around the pad (offsets [`RINGS`]); inside the relief void every spoke contributes
//! exactly one ring component. Rings that lie mostly in copper (gap narrower than the ring)
//! are discarded and the smallest count over the valid rings is used, so a spoke is counted
//! only when it certainly reaches the fill (INV-DRC-1: pad hulls are circumscribed supersets,
//! components must have a minimum area, and the fill region must touch the pad).

use crate::connectivity::CONTACT_TOLERANCE;
use crate::{BucketGrid, CopperItem, ItemRef, Severity, Violation, ViolationKind};
use ferroute_board::{Board, LayerId, NetId, PadConnection, Zone};
use ferroute_geom::boolean::{Region, circumscribed, difference, intersection, regions_of};
use ferroute_geom::{Coord, Hull, Point, Polygon, Rect};

/// KiCad's default `min_resolved_spokes` constraint.
pub const MIN_RESOLVED_SPOKES: usize = 2;
/// Probe ring offsets from the pad outline (nm).
pub const RINGS: [Coord; 4] = [40_000, 80_000, 120_000, 160_000];
/// Half width of a probe ring (nm).
pub const HALF_BAND: Coord = 12_000;
/// Fill within this distance of a pad is cut when a refill would leave it starved.
pub const CUT_RADIUS: Coord = RINGS[3] + HALF_BAND + 3_000;

/// Spokes found at one pad on one layer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SpokeCount {
    /// Spokes to fill that reaches other same-net copper, plus manual track spokes.
    pub spokes: usize,
    /// Spokes to an island that touches nothing but the pad.
    pub ignored: usize,
}

impl SpokeCount {
    /// KiCad's final count (isolated spokes only count when nothing else connects).
    #[must_use]
    pub const fn effective(self) -> usize {
        if self.spokes == 0 { self.ignored } else { self.spokes }
    }

    /// `true` when KiCad would report the pad as starved.
    #[must_use]
    pub const fn starved(self) -> bool {
        let n = self.effective();
        n >= 1 && n < MIN_RESOLVED_SPOKES
    }
}

/// `true` when the zone connects pad `item` through thermal reliefs.
#[must_use]
pub fn is_thermal(board: &Board, zone: &Zone, item: &CopperItem) -> bool {
    let ItemRef::Pad(p) = item.item else { return false };
    match zone.pad_connection {
        PadConnection::ThermalRelief => true,
        PadConnection::ThruHoleOnly => board.pad(p).drill.is_some(),
        PadConnection::Solid | PadConnection::None => false,
    }
}

/// Shared lookup structures over the board's copper items.
pub(crate) struct Lookup<'a> {
    board: &'a Board,
    items: &'a [CopperItem],
    boxes: Vec<Rect>,
    grid: BucketGrid,
}

/// `true` when the region touches `hull` (outer ring, not buried in a hole).
fn touches(r: &Region, hull: &Hull) -> bool {
    Polygon::new(r.outer.clone()).hull_violates(hull, CONTACT_TOLERANCE)
        && !r
            .holes
            .iter()
            .any(|h| Polygon::new(h.clone()).contains_hull_with_clearance(hull, CONTACT_TOLERANCE))
}

impl<'a> Lookup<'a> {
    #[must_use]
    pub(crate) fn new(board: &'a Board, items: &'a [CopperItem]) -> Self {
        let boxes: Vec<Rect> = items.iter().map(|it| it.bbox).collect();
        let grid = BucketGrid::new(&boxes, 2_000_000);
        Self { board, items, boxes, grid }
    }

    fn near(&self, bb: Rect) -> impl Iterator<Item = &CopperItem> + '_ {
        self.grid.query(bb, &self.boxes).into_iter().map(|k| &self.items[k as usize])
    }

    /// `true` when `r` touches same-net copper on `layer` other than `pad`.
    fn touches_other(&self, zone: &Zone, r: &Region, layer: LayerId, pad: &CopperItem) -> bool {
        let Some(bb) = r.bbox() else { return false };
        self.near(bb.inflate(CONTACT_TOLERANCE)).any(|it| {
            it.item != pad.item
                && it.net == zone.net
                && it.layers.contains(layer)
                && match it.item {
                    ItemRef::Pad(p) => zone.pad_connection.joins(self.board.pad(p).drill.is_some()),
                    _ => true,
                }
                && touches(r, &it.hull)
        })
    }

    /// Manual spokes: same-net tracks with one end on the pad and the other inside the fill.
    fn manual_spokes(&self, pad: &CopperItem, layer: LayerId, fill: &[Region]) -> usize {
        let core = circumscribed(&pad.hull, 0);
        let in_fill = |p: Point| {
            fill.iter().any(|r| r.bbox().is_some_and(|b| b.contains(p)) && r.contains(p))
        };
        self.near(pad.bbox.inflate(CONTACT_TOLERANCE))
            .filter(|it| it.net == pad.net && it.layers.contains(layer))
            .filter_map(|it| match it.item {
                ItemRef::Track(i) => {
                    let t = &self.board.tracks[i as usize];
                    Some((t.start, t.end))
                }
                ItemRef::Arc(i) => {
                    let a = &self.board.arcs[i as usize];
                    Some((a.start, a.end))
                }
                _ => None,
            })
            .filter(|&(s, e)| {
                (core.contains(s) && !core.contains(e) && in_fill(e))
                    || (core.contains(e) && !core.contains(s) && in_fill(s))
            })
            .count()
    }

    /// Spokes of `pad` on `layer` into `fill` (connected regions of the zone's fill on that
    /// layer, or of the part under test). `None` when KiCad skips the pad: no probe ring
    /// lies in the relief void, or the fill does not touch the pad at all.
    #[must_use]
    pub(crate) fn spokes(
        &self,
        zone: &Zone,
        layer: LayerId,
        pad: &CopperItem,
        fill: &[Region],
    ) -> Option<SpokeCount> {
        let bb = pad.bbox.inflate(CUT_RADIUS + 20_000);
        let near: Vec<Region> =
            fill.iter().filter(|r| r.bbox().is_some_and(|b| b.intersects(bb))).cloned().collect();
        let pad_regions: Vec<bool> = near.iter().map(|r| touches(r, &pad.hull)).collect();
        if !pad_regions.iter().any(|&t| t) {
            return None;
        }
        let min_area2 = 2 * i128::from(HALF_BAND) * i128::from(zone.min_thickness);
        let mut best: Option<(usize, usize)> = None;
        for d in RINGS {
            let outer = regions_of(&[circumscribed(&pad.hull, d + HALF_BAND)]);
            let ring = difference(&outer, &[circumscribed(&pad.hull, d - HALF_BAND)]);
            let ring_area: i128 = ring.iter().map(Region::area2).sum();
            let comps = intersection(&near, &ring);
            let filled: i128 = comps.iter().map(Region::area2).sum();
            if ring_area <= 0 || 2 * filled >= ring_area {
                continue; // ring lies in copper: the relief gap is narrower than this ring
            }
            let (mut spokes, mut ignored) = (0usize, 0usize);
            for c in comps.iter().filter(|c| c.area2() >= min_area2) {
                // The vertex mean is interior (vertices sit on spoke edges = hole boundary).
                let Some(p) = mean(&c.outer) else { continue };
                let Some(pi) = near.iter().position(|r| r.contains(p)) else { continue };
                if !pad_regions[pi] {
                    continue;
                }
                if self.touches_other(zone, &near[pi], layer, pad) {
                    spokes += 1;
                } else {
                    ignored += 1;
                }
            }
            if best.is_none_or(|b| spokes < b.0) {
                best = Some((spokes, ignored));
            }
        }
        let (mut spokes, ignored) = best?;
        if spokes == 0 && ignored == 0 {
            return None;
        }
        if spokes < MIN_RESOLVED_SPOKES {
            spokes += self.manual_spokes(pad, layer, fill);
        }
        Some(SpokeCount { spokes, ignored })
    }
}

/// Mean of the vertices (integer, rounded toward zero).
fn mean(pts: &[Point]) -> Option<Point> {
    let n = i64::try_from(pts.len()).ok().filter(|&n| n > 0)?;
    let (sx, sy) = pts.iter().fold((0i64, 0i64), |a, p| (a.0 + p.x, a.1 + p.y));
    Some(Point::new(sx / n, sy / n))
}

/// Starved thermal reliefs of every non-yielding zone's stored fill.
///
/// A violation is `introduced` when ferroute-added copper lies within the relief
/// neighbourhood of the pad (zone clearance + 1.5 mm); a starved relief KiCad itself left
/// behind is reported as pre-existing.
#[must_use]
pub fn check_starved_thermals(board: &Board, items: &[CopperItem]) -> Vec<Violation> {
    let mut out = Vec::new();
    if board.zones.iter().all(|z| z.yields || z.is_unfilled()) {
        return out;
    }
    let lk = Lookup::new(board, items);
    for (zi, zone) in board.zones.iter().enumerate() {
        if zone.yields || zone.net == NetId::NONE {
            continue;
        }
        for f in &zone.fills {
            let Some(fb) = f.polys.iter().filter_map(Polygon::bbox).reduce(|a, b| a.union(b))
            else {
                continue;
            };
            let regions = regions_of(&f.polys);
            for pad in items {
                if !matches!(pad.item, ItemRef::Pad(_))
                    || pad.net != zone.net
                    || !pad.layers.contains(f.layer)
                    || !pad.bbox.intersects(fb)
                    || !is_thermal(board, zone, pad)
                {
                    continue;
                }
                let Some(n) = lk.spokes(zone, f.layer, pad, &regions) else { continue };
                if !n.starved() {
                    continue;
                }
                let reach = pad.bbox.inflate(zone.clearance + 1_500_000);
                let introduced =
                    lk.near(reach).any(|it| it.introduced && it.layers.contains(f.layer));
                #[allow(clippy::cast_precision_loss)]
                out.push(Violation {
                    kind: ViolationKind::StarvedThermal,
                    a: pad.item,
                    b: Some(ItemRef::Zone(u32::try_from(zi).unwrap_or(u32::MAX))),
                    layer: Some(f.layer),
                    at: pad.bbox.center(),
                    actual: n.effective() as f64,
                    required: Coord::try_from(MIN_RESOLVED_SPOKES).unwrap_or(2),
                    introduced,
                    severity: Severity::Error,
                });
            }
        }
    }
    out
}
