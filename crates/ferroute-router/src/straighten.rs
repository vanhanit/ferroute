// SPDX-License-Identifier: AGPL-3.0-only
//! Gridless post-optimiser.
//!
//! The lattice router leaves short break-out dog-legs next to pads (pad centres are
//! usually off the lattice rows/columns) and follows lattice lines for the long run. This
//! pass works on the *emitted* copper: every maximal chain of same-net tracks between
//! anchors (junctions, vias, pads, width changes, touching existing copper) is greedily
//! shortcut by 1- or 2-segment octilinear connections between non-adjacent vertices.
//!
//! A shortcut is accepted only when it is strictly better by
//! `(slivers, avoidable bends next to pads, bends, 90° bends, length)`
//! and *exactly* legal: pad entry axes (INV-ROUTE-2), no turn sharper than 90°
//! (INV-ROUTE-3), net-pair clearance, board edge, keep-outs and hole clearance against all
//! other copper including the other nets' new tracks, and no overlap with the net's own
//! other copper. Everything uses the exact integer predicates of `ferroute-geom`. A final
//! oracle run guards the whole pass: if the result is worse than the input in any
//! measured way, the input is returned unchanged.

use crate::access::{entry_dirs, octilinear_paths};
use ferroute_board::{Board, BoardEdit, LayerId, LayerSet, NetId, PadId, Track};
use ferroute_drc::quality::{avoidable_pad_bend, is_sliver};
use ferroute_drc::{BucketGrid, CopperItem, DrcConfig, ItemRef, check, copper_items};
use ferroute_geom::{Coord, Hull, Point, Rect, orient};
use rustc_hash::FxHashMap;
use std::collections::{BTreeMap, BTreeSet};

/// Bucket size of the spatial indices.
const CELL: Coord = 4_000_000;
/// Long segments are queried piecewise so diagonals do not scan their whole bounding box.
const PIECE: Coord = 6_000_000;
const MAX_PASSES: usize = 4;
/// Longest window (in vertices) considered for one shortcut.
const MAX_WINDOW: usize = 96;

/// `(slivers, avoidable bends next to pads, bends, bends of 90° or more, rounded length in nm)`, compared lexicographically.
type Score = (u32, u32, u32, u32, u64);

pub(crate) fn unit(v: Point) -> (i64, i64) {
    (v.x.signum(), v.y.signum())
}

fn same_net(a: NetId, b: NetId) -> bool {
    a == b && a != NetId::NONE
}

/// Does copper `h` cover the point `p`?
pub(crate) fn covers(h: &Hull, p: Point) -> bool {
    h.clearance_violated(&Hull::disk(p, 0), 0)
}

/// Is `q` strictly inside the segment `a`–`b` (collinear, not an endpoint)?
pub(crate) fn on_interior(a: Point, b: Point, q: Point) -> bool {
    q != a && q != b && orient(a, b, q) == 0 && Rect::new(a, b).contains(q)
}

/// Score of the polyline `pts` (with its neighbours) of `ch`; lower is better.
pub(crate) fn score(
    base: &Board,
    ch: &Chain,
    prev: Option<Point>,
    pts: &[Point],
    next: Option<Point>,
    slivers: u32,
) -> Score {
    let mut ext: Vec<Point> = Vec::with_capacity(pts.len() + 2);
    ext.extend(prev);
    ext.extend_from_slice(pts);
    ext.extend(next);
    let (mut near, mut bends, mut right) = (0, 0, 0);
    for w in ext.windows(3) {
        let (u, v) = (unit(w[1] - w[0]), unit(w[2] - w[1]));
        if u != v {
            bends += 1;
            if u.0 * v.0 + u.1 * v.1 <= 0 {
                right += 1;
            }
            if avoidable_pad_bend(base, (ch.net, ch.layer, ch.template.width), (w[0], w[1], w[2])) {
                near += 1;
            }
        }
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let len = pts.windows(2).map(|w| w[0].dist(w[1])).sum::<f64>().round() as u64;
    (slivers, near, bends, right, len)
}

/// Query rectangles covering the segment `a`–`b`, each inflated by `margin`.
fn pieces(a: Point, b: Point, margin: Coord) -> Vec<Rect> {
    #[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
    let n = ((a.dist(b) / PIECE as f64).ceil() as i64).max(1);
    let lerp = |t: i64| {
        let f = |p: Coord, q: Coord| {
            p + Coord::try_from(i128::from(q - p) * i128::from(t) / i128::from(n)).unwrap_or(0)
        };
        Point::new(f(a.x, b.x), f(a.y, b.y))
    };
    (0..n).map(|t| Rect::new(lerp(t), lerp(t + 1)).inflate(margin)).collect()
}

/// A track of the edit under optimisation.
pub(crate) struct Seg {
    pub(crate) a: Point,
    pub(crate) b: Point,
    pub(crate) width: Coord,
    pub(crate) net: NetId,
    pub(crate) layer: LayerId,
    pub(crate) alive: bool,
}

impl Seg {
    pub(crate) fn hull(&self) -> Hull {
        Hull::capsule(self.a, self.b, self.width / 2)
    }
}

/// Dynamic bucket index over the edit's tracks (they change while chains are rewritten).
#[derive(Default)]
pub(crate) struct Dynamic {
    pub(crate) segs: Vec<Seg>,
    pub(crate) cells: FxHashMap<(i64, i64), Vec<u32>>,
}

impl Dynamic {
    /// An index holding `tracks`; segment ids equal track indices.
    pub(crate) fn of(tracks: &[Track]) -> Self {
        let mut d = Self::default();
        for t in tracks {
            d.insert(Seg {
                a: t.start,
                b: t.end,
                width: t.width,
                net: t.net,
                layer: t.layer,
                alive: true,
            });
        }
        d
    }

    pub(crate) fn insert(&mut self, s: Seg) -> u32 {
        let id = u32::try_from(self.segs.len()).unwrap_or(u32::MAX);
        let r = s.hull().bbox();
        for cy in r.min.y.div_euclid(CELL)..=r.max.y.div_euclid(CELL) {
            for cx in r.min.x.div_euclid(CELL)..=r.max.x.div_euclid(CELL) {
                self.cells.entry((cx, cy)).or_default().push(id);
            }
        }
        self.segs.push(s);
        id
    }

    pub(crate) fn remove(&mut self, id: u32) {
        self.segs[id as usize].alive = false;
    }

    pub(crate) fn query(&self, r: Rect, out: &mut Vec<u32>) {
        for cy in r.min.y.div_euclid(CELL)..=r.max.y.div_euclid(CELL) {
            for cx in r.min.x.div_euclid(CELL)..=r.max.x.div_euclid(CELL) {
                for &id in self.cells.get(&(cx, cy)).into_iter().flatten() {
                    let s = &self.segs[id as usize];
                    if s.alive && s.hull().bbox().intersects(r) {
                        out.push(id);
                    }
                }
            }
        }
    }

    /// Ids of live segments near `a`–`b` (sorted, deduplicated).
    fn near(&self, a: Point, b: Point, margin: Coord) -> Vec<u32> {
        let mut v = Vec::new();
        for r in pieces(a, b, margin) {
            self.query(r, &mut v);
        }
        v.sort_unstable();
        v.dedup();
        v
    }
}

pub(crate) struct Hole {
    pub(crate) hull: Hull,
    pub(crate) net: NetId,
}

/// Static obstacles: everything of the board plus the edit's vias and arcs.
pub(crate) struct Statics<'a> {
    pub(crate) base: &'a Board,
    pub(crate) items: Vec<CopperItem>,
    pub(crate) boxes: Vec<Rect>,
    pub(crate) grid: BucketGrid,
    pub(crate) holes: Vec<Hole>,
    pub(crate) hole_boxes: Vec<Rect>,
    pub(crate) hole_grid: BucketGrid,
    pub(crate) pads_at: BTreeMap<Point, Vec<PadId>>,
    pub(crate) reach: Coord,
    /// Stored copper-pour fills (other-net fills are obstacles).
    pub(crate) fills: Vec<ferroute_drc::zones::FillPoly<'a>>,
}

/// The segment window being replaced and what is exempt from the same-net checks.
struct Window<'a> {
    net: NetId,
    layer: LayerId,
    vi: Point,
    vj: Point,
    /// Segments replaced by the candidate.
    replaced: &'a [u32],
    /// Neighbouring segments (touch each other at the window ends by construction).
    adjacent: [Option<u32>; 2],
}

/// A maximal run of degree-2 vertices between anchors.
pub(crate) struct Chain {
    pub(crate) net: NetId,
    pub(crate) layer: LayerId,
    pub(crate) template: Track,
    pub(crate) pts: Vec<Point>,
    pub(crate) ids: Vec<u32>,
    pub(crate) orig: Vec<usize>,
    pub(crate) changed: bool,
}

impl Chain {
    /// Replaces the vertices `i..=j` by the polyline `cand`, keeping `dynamic` in sync.
    pub(crate) fn splice(&mut self, dynamic: &mut Dynamic, i: usize, j: usize, cand: Vec<Point>) {
        for &id in &self.ids[i..j] {
            dynamic.remove(id);
        }
        let new_ids: Vec<u32> = cand
            .windows(2)
            .map(|s| {
                dynamic.insert(Seg {
                    a: s[0],
                    b: s[1],
                    width: self.template.width,
                    net: self.net,
                    layer: self.layer,
                    alive: true,
                })
            })
            .collect();
        self.ids.splice(i..j, new_ids);
        self.pts.splice(i..=j, cand);
        self.changed = true;
    }
}

impl<'a> Statics<'a> {
    pub(crate) fn new(base: &'a Board, edit: &BoardEdit) -> Self {
        let mut items = copper_items(base);
        let nv = base.vias.len();
        for (k, v) in edit.new_vias.iter().enumerate() {
            let hull = v.hull();
            items.push(CopperItem {
                item: ItemRef::Via(u32::try_from(nv + k).unwrap_or(u32::MAX)),
                bbox: hull.bbox(),
                hull,
                layers: v.layers,
                net: v.net,
                footprint: None,
                introduced: true,
            });
        }
        for a in &edit.new_arcs {
            for hull in a.hulls() {
                items.push(CopperItem {
                    item: ItemRef::Track(u32::MAX),
                    bbox: hull.bbox(),
                    hull,
                    layers: LayerSet::single(a.layer),
                    net: a.net,
                    footprint: None,
                    introduced: true,
                });
            }
        }
        let boxes: Vec<Rect> = items.iter().map(|i| i.bbox).collect();
        let grid = BucketGrid::new(&boxes, CELL);
        let mut holes = Vec::new();
        let mut pads_at: BTreeMap<Point, Vec<PadId>> = BTreeMap::new();
        for id in base.pad_ids() {
            let pad = base.pad(id);
            if let Some(d) = pad.drill {
                holes.push(Hole {
                    hull: Hull::disk(base.pad_position(id), d.diameter / 2),
                    net: pad.net,
                });
            }
            if !pad.layers.is_empty() && pad.net != NetId::NONE {
                pads_at.entry(base.pad_position(id)).or_default().push(id);
            }
        }
        for v in base.vias.iter().chain(&edit.new_vias) {
            holes.push(Hole { hull: Hull::disk(v.pos, v.drill / 2), net: v.net });
        }
        let hole_boxes: Vec<Rect> = holes.iter().map(|h| h.hull.bbox()).collect();
        let hole_grid = BucketGrid::new(&hole_boxes, CELL);
        let reach = base.rules.max_clearance().max(base.rules.hole_clearance);
        let fills = ferroute_drc::zones::fill_polys(base);
        Self { base, items, boxes, grid, holes, hole_boxes, hole_grid, pads_at, reach, fills }
    }

    pub(crate) fn static_near(&self, a: Point, b: Point, margin: Coord) -> Vec<u32> {
        let mut v = Vec::new();
        for r in pieces(a, b, margin) {
            v.extend(self.grid.query(r, &self.boxes));
        }
        v.sort_unstable();
        v.dedup();
        v
    }

    /// Is `p` a point the optimiser must keep (touched by existing same-net copper)?
    pub(crate) fn static_anchor(&self, net: NetId, layer: LayerId, p: Point) -> bool {
        if self.pads_at.get(&p).is_some_and(|ps| {
            ps.iter().any(|&id| {
                let pad = self.base.pad(id);
                pad.net == net && pad.layers.contains(layer)
            })
        }) {
            return true;
        }
        self.grid.query(Rect::new(p, p).inflate(1), &self.boxes).into_iter().any(|k| {
            let it = &self.items[k as usize];
            same_net(it.net, net)
                && it.layers.contains(layer)
                && !matches!(it.item, ItemRef::Pad(_))
                && covers(&it.hull, p)
        })
    }

    /// Every pad centred on `p` (of this net, on this layer) accepts entry along `d`.
    fn entry_ok(&self, net: NetId, layer: LayerId, p: Point, d: (i64, i64)) -> bool {
        self.pads_at.get(&p).is_none_or(|ps| {
            ps.iter().all(|&id| {
                let pad = self.base.pad(id);
                pad.net != net
                    || !pad.layers.contains(layer)
                    || entry_dirs(self.base, id).contains(&d)
            })
        })
    }

    /// Vectors from `p` along same-net, same-layer tracks outside the replaced window.
    fn incident(&self, dynamic: &Dynamic, w: &Window<'_>, p: Point) -> Vec<Point> {
        let mut out = Vec::new();
        let at = Rect::new(p, p).inflate(1);
        for k in self.grid.query(at, &self.boxes) {
            let it = &self.items[k as usize];
            if matches!(it.item, ItemRef::Track(_))
                && same_net(it.net, w.net)
                && it.layers.contains(w.layer)
                && it.hull.pts.len() == 2
            {
                if it.hull.pts[0] == p {
                    out.push(it.hull.pts[1] - p);
                } else if it.hull.pts[1] == p {
                    out.push(it.hull.pts[0] - p);
                } else if on_interior(it.hull.pts[0], it.hull.pts[1], p) {
                    out.push(it.hull.pts[0] - p);
                    out.push(it.hull.pts[1] - p);
                }
            }
        }
        let mut ids = Vec::new();
        dynamic.query(at, &mut ids);
        for id in ids {
            let s = &dynamic.segs[id as usize];
            if s.layer != w.layer || !same_net(s.net, w.net) || w.replaced.contains(&id) {
                continue;
            }
            if s.a == p && s.b != p {
                out.push(s.b - p);
            } else if s.b == p && s.a != p {
                out.push(s.a - p);
            } else if on_interior(s.a, s.b, p) {
                out.push(s.a - p);
                out.push(s.b - p);
            }
        }
        out
    }

    /// Would a track `a`–`b` of `width` conflict with anything? With `only_same_net` just
    /// the same-net overlap rule is evaluated (used to vet the copper being replaced).
    fn scan(
        &self,
        dynamic: &Dynamic,
        w: &Window<'_>,
        (a, b): (Point, Point),
        width: Coord,
        only_same_net: bool,
    ) -> bool {
        let base = self.base;
        let hull = Hull::capsule(a, b, width / 2);
        let margin = width / 2 + self.reach + 1;
        let (net, layer) = (w.net, w.layer);
        // Same-net copper may be touched only where it covers a window end.
        let touches = |other: &Hull| {
            hull.clearance_violated(other, 1) && !covers(other, w.vi) && !covers(other, w.vj)
        };
        if !only_same_net {
            if !base.outline.admits(&hull, base.rules.edge_clearance) {
                return true;
            }
            if base
                .keepouts
                .iter()
                .any(|k| k.no_tracks && k.layers.contains(layer) && k.area.hull_violates(&hull, 0))
            {
                return true;
            }
            if ferroute_drc::zones::conflicts(
                base,
                &self.fills,
                &hull,
                LayerSet::single(layer),
                net,
            ) {
                return true;
            }
            let hc = base.rules.hole_clearance;
            let mut hv = Vec::new();
            for r in pieces(a, b, width / 2 + hc + 1) {
                hv.extend(self.hole_grid.query(r, &self.hole_boxes));
            }
            hv.sort_unstable();
            hv.dedup();
            if hv.iter().any(|&k| {
                let h = &self.holes[k as usize];
                !same_net(h.net, net) && hull.clearance_violated(&h.hull, hc)
            }) {
                return true;
            }
        }
        for k in self.static_near(a, b, margin) {
            let it = &self.items[k as usize];
            if !it.layers.contains(layer) {
                continue;
            }
            if same_net(it.net, net) {
                if touches(&it.hull) {
                    return true;
                }
            } else if !only_same_net
                && hull.clearance_violated(&it.hull, base.clearance_between(net, it.net))
            {
                return true;
            }
        }
        for id in dynamic.near(a, b, margin) {
            let s = &dynamic.segs[id as usize];
            if s.layer != layer {
                continue;
            }
            if same_net(s.net, net) {
                if !w.replaced.contains(&id)
                    && !w.adjacent.contains(&Some(id))
                    && touches(&s.hull())
                {
                    return true;
                }
            } else if !only_same_net
                && hull.clearance_violated(&s.hull(), base.clearance_between(net, s.net))
            {
                return true;
            }
        }
        false
    }

    /// Can `cand` (a polyline from vertex `i` to vertex `j`) replace that part of `ch`?
    pub(crate) fn admissible(
        &self,
        dynamic: &Dynamic,
        ch: &Chain,
        i: usize,
        j: usize,
        cand: &[Point],
    ) -> bool {
        let n = ch.pts.len() - 1;
        let width = ch.template.width;
        // The window ends are the candidate's ends (`vj` may differ from `ch.pts[j]` when
        // the far end is a via being relocated).
        let last = cand.len() - 1;
        let (vi, vj) = (cand[0], cand[last]);
        if cand.len() == 3 {
            let min2 = u128::try_from(4 * i128::from(width) * i128::from(width)).unwrap_or(0);
            if (cand[1] - cand[0]).norm2() < min2 || (cand[2] - cand[1]).norm2() < min2 {
                return false;
            }
        }
        let (first, back) = (cand[1] - cand[0], cand[last - 1] - cand[last]);
        if !self.entry_ok(ch.net, ch.layer, vi, unit(first))
            || !self.entry_ok(ch.net, ch.layer, vj, unit(back))
        {
            return false;
        }
        let win = Window {
            net: ch.net,
            layer: ch.layer,
            vi,
            vj,
            replaced: &ch.ids[i..j],
            adjacent: [i.checked_sub(1).map(|k| ch.ids[k]), (j < n).then(|| ch.ids[j])],
        };
        // Same-net junction angles at both ends (no turn sharper than 90°).
        if self.incident(dynamic, &win, vi).iter().any(|&o| first.dot(o) > 0)
            || self.incident(dynamic, &win, vj).iter().any(|&o| back.dot(o) > 0)
        {
            return false;
        }
        if cand.windows(2).any(|s| self.scan(dynamic, &win, (s[0], s[1]), width, false)) {
            return false;
        }
        // The copper being replaced must not be what connects some other same-net item.
        !ch.pts[i..=j].windows(2).any(|s| self.scan(dynamic, &win, (s[0], s[1]), width, true))
    }

    /// Same-net sliver pairs between the polyline `pts` and other copper, ignoring the
    /// chain's own window `i..j` (which the polyline replaces).
    pub(crate) fn slivers(
        &self,
        dynamic: &Dynamic,
        ch: &Chain,
        i: usize,
        j: usize,
        pts: &[Point],
    ) -> u32 {
        let base = self.base;
        let clr = base.clearance_between(ch.net, ch.net);
        let margin = ch.template.width / 2 + clr + 1;
        let mut n = 0;
        for s in pts.windows(2) {
            let hull = Hull::capsule(s[0], s[1], ch.template.width / 2);
            for k in self.static_near(s[0], s[1], margin) {
                let it = &self.items[k as usize];
                if it.layers.contains(ch.layer)
                    && same_net(it.net, ch.net)
                    && is_sliver(&hull, &it.hull, clr)
                {
                    n += 1;
                }
            }
            for id in dynamic.near(s[0], s[1], margin) {
                let o = &dynamic.segs[id as usize];
                if o.layer == ch.layer
                    && same_net(o.net, ch.net)
                    && !ch.ids[i..j].contains(&id)
                    && is_sliver(&hull, &o.hull(), clr)
                {
                    n += 1;
                }
            }
        }
        n
    }

    /// Greedy shortcutting of one chain. Returns `true` if anything changed.
    fn improve(&self, dynamic: &mut Dynamic, ch: &mut Chain) -> bool {
        let mut any = false;
        for _ in 0..MAX_PASSES {
            let mut changed = false;
            let mut i = 0;
            while i + 2 < ch.pts.len() {
                let n = ch.pts.len() - 1;
                let prev = i.checked_sub(1).map(|k| ch.pts[k]);
                let mut chosen: Option<(usize, Vec<Point>)> = None;
                for j in (i + 2..=n.min(i + MAX_WINDOW)).rev() {
                    let (vi, vj) = (ch.pts[i], ch.pts[j]);
                    if vi == vj {
                        continue;
                    }
                    let next = (j < n).then(|| ch.pts[j + 1]);
                    let old = score(
                        self.base,
                        ch,
                        prev,
                        &ch.pts[i..=j],
                        next,
                        self.slivers(dynamic, ch, i, j, &ch.pts[i..=j]),
                    );
                    let mut best: Option<(Score, Vec<Point>)> = None;
                    // `octilinear_paths` lists the diagonal-first dog-leg first; prefer the
                    // straight-first one on ties ("straight out of the pad, one 45°").
                    for cand in octilinear_paths(vi, vj).into_iter().rev() {
                        let s = score(
                            self.base,
                            ch,
                            prev,
                            &cand,
                            next,
                            self.slivers(dynamic, ch, i, j, &cand),
                        );
                        if s >= old || best.as_ref().is_some_and(|b| s >= b.0) {
                            continue;
                        }
                        if self.admissible(dynamic, ch, i, j, &cand) {
                            best = Some((s, cand));
                        }
                    }
                    if let Some((_, cand)) = best {
                        chosen = Some((j, cand));
                        break;
                    }
                }
                if let Some((j, cand)) = chosen {
                    ch.splice(dynamic, i, j, cand);
                    changed = true;
                }
                i += 1;
            }
            any |= changed;
            if !changed {
                break;
            }
        }
        any
    }

    /// Extracts every optimisable chain of the edit's tracks, in a deterministic order.
    pub(crate) fn chains(&self, tracks: &[Track]) -> Vec<Chain> {
        self.chains_min(tracks, 2)
    }

    /// Like [`Self::chains`] but keeps chains of at least `min_segments` tracks.
    pub(crate) fn chains_min(&self, tracks: &[Track], min_segments: usize) -> Vec<Chain> {
        let mut groups: BTreeMap<(u32, u8), Vec<usize>> = BTreeMap::new();
        for (k, t) in tracks.iter().enumerate() {
            if t.net != NetId::NONE && t.start != t.end {
                groups.entry((t.net.0, t.layer.0)).or_default().push(k);
            }
        }
        let mut out = Vec::new();
        for ((net, layer), idxs) in groups {
            let (net, layer) = (NetId(net), LayerId(layer));
            let mut nodes: BTreeMap<Point, Vec<usize>> = BTreeMap::new();
            for &k in &idxs {
                nodes.entry(tracks[k].start).or_default().push(k);
                nodes.entry(tracks[k].end).or_default().push(k);
            }
            // Tracks with a foreign endpoint in their interior cannot be split safely.
            let mut frozen: BTreeSet<usize> = BTreeSet::new();
            for &k in &idxs {
                let (a, b) = (tracks[k].start, tracks[k].end);
                let bb = Rect::new(a, b);
                let lo = Point::new(bb.min.x, Coord::MIN);
                let hi = Point::new(bb.max.x, Coord::MAX);
                if nodes.range(lo..=hi).any(|(&q, _)| on_interior(a, b, q)) {
                    frozen.insert(k);
                    continue;
                }
                let hit = self.grid.query(bb.inflate(1), &self.boxes).into_iter().any(|m| {
                    let it = &self.items[m as usize];
                    same_net(it.net, net)
                        && it.layers.contains(layer)
                        && matches!(it.item, ItemRef::Track(_))
                        && it.hull.pts.iter().any(|&q| on_interior(a, b, q))
                });
                if hit {
                    frozen.insert(k);
                }
            }
            let anchor: BTreeMap<Point, bool> = nodes
                .iter()
                .map(|(&p, inc)| {
                    let a = inc.len() != 2
                        || {
                            let (t0, t1) = (&tracks[inc[0]], &tracks[inc[1]]);
                            t0.width != t1.width || t0.locked != t1.locked || t0.origin != t1.origin
                        }
                        || frozen.contains(&inc[0])
                        || frozen.contains(&inc[1])
                        || self.static_anchor(net, layer, p);
                    (p, a)
                })
                .collect();
            let mut used: BTreeSet<usize> = BTreeSet::new();
            for (&p0, inc) in &nodes {
                if !anchor[&p0] {
                    continue;
                }
                for &k0 in inc {
                    if frozen.contains(&k0) || used.contains(&k0) {
                        continue;
                    }
                    let (mut pts, mut orig) = (vec![p0], Vec::new());
                    let (mut cur, mut k) = (p0, k0);
                    let complete = loop {
                        if !used.insert(k) {
                            break false;
                        }
                        orig.push(k);
                        let t = &tracks[k];
                        let nxt = if t.start == cur { t.end } else { t.start };
                        pts.push(nxt);
                        if anchor[&nxt] {
                            break true;
                        }
                        let inc2 = &nodes[&nxt];
                        k = if inc2[0] == k { inc2[1] } else { inc2[0] };
                        cur = nxt;
                    };
                    if !complete || orig.len() < min_segments {
                        continue;
                    }
                    if pts[0] > pts[pts.len() - 1] {
                        pts.reverse();
                        orig.reverse();
                    }
                    out.push(Chain {
                        net,
                        layer,
                        template: tracks[orig[0]].clone(),
                        ids: orig.iter().map(|&k| u32::try_from(k).unwrap_or(u32::MAX)).collect(),
                        pts,
                        orig,
                        changed: false,
                    });
                }
            }
        }
        out.sort_by_key(|c| {
            let (a, b) = (c.pts[0], c.pts[c.pts.len() - 1]);
            (c.net.0, c.layer.0, a.min(b), a.max(b), c.orig[0])
        });
        out
    }
}

/// `(introduced violations, acid traps, missing connections)` of `base + edit`.
pub(crate) fn quality(base: &Board, edit: &BoardEdit) -> (usize, usize, usize) {
    let mut after = base.clone();
    after.apply(edit);
    let rep = check(&after, &DrcConfig { courtyards: false, ..DrcConfig::default() });
    (
        rep.introduced().count(),
        ferroute_drc::quality::acute_angles(&after).len(),
        rep.connectivity.connections_missing,
    )
}

/// `base` with the edit's footprint moves applied (borrowed when there are none).
pub(crate) fn moved_board<'a>(base: &'a Board, edit: &BoardEdit) -> std::borrow::Cow<'a, Board> {
    if edit.moves.is_empty() {
        return std::borrow::Cow::Borrowed(base);
    }
    let mut b = base.clone();
    for m in &edit.moves {
        b.footprints[m.footprint.idx()].placement = m.to;
    }
    std::borrow::Cow::Owned(b)
}

/// The edit with every changed chain's tracks replaced by its current polyline.
pub(crate) fn rebuild(edit: &BoardEdit, chains: &[Chain]) -> BoardEdit {
    let mut drop = vec![false; edit.new_tracks.len()];
    let mut insert: BTreeMap<usize, Vec<Track>> = BTreeMap::new();
    for ch in chains.iter().filter(|c| c.changed) {
        for &k in &ch.orig {
            drop[k] = true;
        }
        let first = ch.orig.iter().copied().min().unwrap_or(0);
        insert.insert(
            first,
            ch.pts
                .windows(2)
                .map(|s| Track { start: s[0], end: s[1], ..ch.template.clone() })
                .collect(),
        );
    }
    let mut new_tracks = Vec::with_capacity(edit.new_tracks.len());
    for (k, t) in edit.new_tracks.iter().enumerate() {
        if let Some(v) = insert.remove(&k) {
            new_tracks.extend(v);
        }
        if !drop[k] {
            new_tracks.push(t.clone());
        }
    }
    BoardEdit { new_tracks, ..edit.clone() }
}

/// Straightens routed connections off-lattice. `base` is the board BEFORE `edit`
/// (pads, existing copper, rules, outline, keepouts). Returns an edit that differs only in
/// `new_tracks`; vias, moves and all `Origin::Existing` copper are untouched.
#[must_use]
pub fn straighten(base: &Board, edit: &BoardEdit) -> BoardEdit {
    let moved = moved_board(base, edit);
    let board = moved.as_ref();
    let st = Statics::new(board, edit);
    let mut dynamic = Dynamic::of(&edit.new_tracks);
    let mut chains = st.chains(&edit.new_tracks);
    for ch in &mut chains {
        st.improve(&mut dynamic, ch);
    }
    if !chains.iter().any(|c| c.changed) {
        return edit.clone();
    }
    let out = rebuild(edit, &chains);
    // Safety net: never return anything the oracle likes less than the input.
    let (before, after) = (quality(base, edit), quality(base, &out));
    if after.0 > before.0 || after.1 > before.1 || after.2 > before.2 {
        tracing::error!(?before, ?after, "straighten made the result worse; keeping the input");
        return edit.clone();
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use ferroute_board::builder::{BoardBuilder, mm};
    use ferroute_board::{LayerId, NetId, Origin, Track, Via};
    use ferroute_drc::{DrcConfig, check, quality::acute_angles};
    use ferroute_geom::Point;

    const W: i64 = 250_000;

    #[allow(clippy::cast_possible_truncation)]
    fn p(x: f64, y: f64) -> Point {
        Point::new((x * 1e6).round() as i64, (y * 1e6).round() as i64)
    }

    fn trk(net: NetId, layer: u8, pts: &[Point]) -> Vec<Track> {
        pts.windows(2)
            .map(|w| Track {
                start: w[0],
                end: w[1],
                width: W,
                layer: ferroute_board::LayerId(layer),
                net,
                locked: false,
                origin: Origin::Routed,
            })
            .collect()
    }

    fn existing(tracks: Vec<Track>) -> Vec<Track> {
        tracks.into_iter().map(|t| Track { origin: Origin::Existing, ..t }).collect()
    }

    /// R1 (rotated, pad 1 on its right) at pad A = (5, 5.13); R2 pad 1 = B = (20, 7.13).
    fn two_pad_board() -> (ferroute_board::Board, NetId, NetId, Point, Point) {
        let mut b = BoardBuilder::new(mm(30), mm(20), 2);
        let n = b.net("N");
        let m = b.net("M");
        b.smd2("R1", p(4.2, 5.13), 180.0, n, NetId::NONE);
        b.smd2("R2", p(20.8, 7.13), 0.0, n, NetId::NONE);
        (b.build(), n, m, p(5.0, 5.13), p(20.0, 7.13))
    }

    fn dog_leg_chain(a: Point, b: Point) -> Vec<Point> {
        vec![a, p(5.6, 5.13), p(5.87, 5.40), p(17.0, 5.40), p(18.73, 7.13), b]
    }

    /// Number of direction changes at vertices shared by exactly two same-net tracks.
    fn bends(tracks: &[Track]) -> usize {
        let mut n = 0;
        for (i, t) in tracks.iter().enumerate() {
            for u in &tracks[i + 1..] {
                if t.net != u.net || t.layer != u.layer {
                    continue;
                }
                let (pt, a, b) = if t.end == u.start {
                    (t.end, t.start, u.end)
                } else if t.end == u.end {
                    (t.end, t.start, u.start)
                } else if t.start == u.start {
                    (t.start, t.end, u.end)
                } else if t.start == u.end {
                    (t.start, t.end, u.start)
                } else {
                    continue;
                };
                let (d1, d2) = (a - pt, b - pt);
                if (d1.x.signum(), d1.y.signum()) != (-d2.x.signum(), -d2.y.signum()) {
                    n += 1;
                }
            }
        }
        n
    }

    fn assert_clean(base: &ferroute_board::Board, edit: &BoardEdit) {
        let mut after = base.clone();
        after.apply(edit);
        let rep = check(&after, &DrcConfig { courtyards: false, ..DrcConfig::default() });
        let intro: Vec<_> = rep.introduced().collect();
        assert!(intro.is_empty(), "introduced violations: {intro:#?}");
        assert_eq!(rep.connectivity.connections_missing, 0, "{:?}", rep.connectivity);
        assert!(acute_angles(&after).is_empty(), "acid traps");
    }

    fn length(tracks: &[Track]) -> f64 {
        tracks.iter().map(|t| t.start.dist(t.end)).sum()
    }

    #[test]
    fn dog_leg_next_to_pad_is_removed() {
        let (base, n, _m, a, b) = two_pad_board();
        let edit =
            BoardEdit { new_tracks: trk(n, 0, &dog_leg_chain(a, b)), ..BoardEdit::default() };
        assert_clean(&base, &edit);
        let out = straighten(&base, &edit);
        assert_clean(&base, &out);
        assert!(out.new_tracks.len() <= 3, "{:#?}", out.new_tracks);
        let first =
            out.new_tracks.iter().find(|t| t.start == a || t.end == a).expect("track at pad A");
        let other = if first.start == a { first.end } else { first.start };
        assert!(other.x > a.x && other.y == a.y, "leaves A along +x, got {other:?}");
        assert!(bends(&out.new_tracks) < bends(&edit.new_tracks));
        assert!(length(&out.new_tracks) <= length(&edit.new_tracks) + 1.0);
        assert!(out.new_tracks.iter().any(|t| t.start == b || t.end == b));
        assert_eq!(out.new_vias, edit.new_vias);
        assert!(out.moves.is_empty());
    }

    #[test]
    fn obstacle_on_the_straight_candidate_keeps_the_chain_valid() {
        let (mut base, n, m, a, b) = two_pad_board();
        // A short stub of another net sitting on the straight-first line y = 5.13.
        base.tracks.extend(existing(trk(m, 0, &[p(10.0, 3.0), p(10.0, 4.9)])));
        let edit =
            BoardEdit { new_tracks: trk(n, 0, &dog_leg_chain(a, b)), ..BoardEdit::default() };
        assert_clean(&base, &edit);
        let out = straighten(&base, &edit);
        assert_clean(&base, &out);
        assert!(bends(&out.new_tracks) <= bends(&edit.new_tracks));
        let obstacle = ferroute_geom::Hull::capsule(p(10.0, 3.0), p(10.0, 4.9), W / 2);
        let clr = base.rules.class_of(&base, n).clearance;
        for t in &out.new_tracks {
            assert!(!t.hull().clearance_violated(&obstacle, clr), "{t:?} too close");
        }
    }

    #[test]
    fn blocked_everywhere_leaves_chain_unchanged() {
        let (mut base, n, m, a, b) = two_pad_board();
        for x in [8.0, 11.0, 14.0] {
            base.tracks.extend(existing(trk(m, 0, &[p(x, 3.0), p(x, 4.9)])));
            base.tracks.extend(existing(trk(m, 0, &[p(x, 6.0), p(x, 8.0)])));
        }
        // Original run: stub, jog, long run on y = 5.4, then down at x = 17.
        let chain = vec![a, p(5.6, 5.13), p(5.87, 5.40), p(17.0, 5.40), p(17.0, 7.13), b];
        let edit = BoardEdit { new_tracks: trk(n, 0, &chain), ..BoardEdit::default() };
        assert_clean(&base, &edit);
        let out = straighten(&base, &edit);
        assert_clean(&base, &out);
        assert!(bends(&out.new_tracks) <= bends(&edit.new_tracks));
    }

    #[test]
    fn pad_entry_axes_are_respected() {
        let (base, n, _m, a, b) = two_pad_board();
        // A shortcut from A to (7, 7.13) would be a pure 45° diagonal leaving the pad.
        let chain =
            vec![a, p(5.6, 5.13), p(5.6, 6.53), p(6.2, 7.13), p(7.0, 7.13), p(15.0, 7.13), b];
        let edit = BoardEdit { new_tracks: trk(n, 0, &chain), ..BoardEdit::default() };
        assert_clean(&base, &edit);
        let out = straighten(&base, &edit);
        assert_clean(&base, &out);
        for pid in base.pad_ids() {
            let c = base.pad_position(pid);
            let dirs = crate::access::entry_dirs(&base, pid);
            for t in out.new_tracks.iter().filter(|t| t.start == c || t.end == c) {
                let o = if t.start == c { t.end } else { t.start } - c;
                assert!(
                    dirs.contains(&(o.x.signum(), o.y.signum())),
                    "{pid:?} entered along {o:?}"
                );
            }
        }
        assert!(bends(&out.new_tracks) < bends(&edit.new_tracks));
    }

    #[test]
    fn vias_junctions_and_other_nets_are_untouched() {
        let (base, n, m, a, b) = two_pad_board();
        let via = p(17.0, 5.40);
        let mut tracks = trk(n, 0, &[a, p(5.6, 5.13), p(5.87, 5.40), via]);
        tracks.extend(trk(n, 1, &[via, p(18.73, 7.13), b]));
        // T junction (degree 3) at (5.87, 5.40) with a stub up.
        tracks.extend(trk(n, 0, &[p(5.87, 5.40), p(5.87, 3.0)]));
        let other = trk(m, 1, &[p(8.0, 12.0), p(12.0, 12.0)]);
        tracks.extend(other.clone());
        let vias = vec![Via {
            pos: via,
            diameter: 600_000,
            drill: 300_000,
            layers: base.all_layers(),
            net: n,
            locked: false,
            origin: Origin::Routed,
        }];
        let edit = BoardEdit { new_tracks: tracks, new_vias: vias, ..BoardEdit::default() };
        let out = straighten(&base, &edit);
        assert_eq!(out.new_vias, edit.new_vias);
        for t in &other {
            assert!(out.new_tracks.contains(t), "other-net track changed");
        }
        let has_endpoint = |q: Point| out.new_tracks.iter().any(|t| t.start == q || t.end == q);
        assert!(has_endpoint(via) && has_endpoint(p(5.87, 5.40)), "anchor vertex vanished");
        assert!(out.new_tracks.contains(&trk(n, 0, &[p(5.87, 5.40), p(5.87, 3.0)])[0]));
        assert_eq!(out, straighten(&base, &edit), "deterministic");
    }

    #[test]
    fn deterministic_and_idempotent() {
        let (base, n, _m, a, b) = two_pad_board();
        let edit =
            BoardEdit { new_tracks: trk(n, 0, &dog_leg_chain(a, b)), ..BoardEdit::default() };
        let o1 = straighten(&base, &edit);
        assert_eq!(o1, straighten(&base, &edit));
        let o2 = straighten(&base, &o1);
        assert!(bends(&o2.new_tracks) <= bends(&o1.new_tracks));
        assert_clean(&base, &o2);
    }

    #[test]
    fn tracks_passing_through_an_anchor_count_as_incident_copper() {
        // Regression: a branch leaving the interior of a long track at 45° is an acid trap.
        let (mut base, n, _m, _a, _b) = two_pad_board();
        base.tracks.extend(existing(trk(n, 0, &[p(10.0, 12.0), p(25.0, 12.0)])));
        let at = p(20.0, 12.0);
        let edit =
            BoardEdit { new_tracks: trk(n, 0, &[at, p(20.0, 14.0)]), ..BoardEdit::default() };
        let st = Statics::new(&base, &edit);
        let dynamic = Dynamic::default();
        let win = Window {
            net: n,
            layer: LayerId(0),
            vi: at,
            vj: at,
            replaced: &[],
            adjacent: [None; 2],
        };
        let mut dirs = st.incident(&dynamic, &win, at);
        dirs.sort();
        assert_eq!(dirs, vec![p(-10.0, 0.0), p(5.0, 0.0)]);
    }

    #[test]
    fn round_pad_is_left_diagonally_instead_of_bending_at_its_edge() {
        let mut bb = BoardBuilder::new(mm(30), mm(20), 2);
        let n = bb.net("N");
        bb.header("J1", p(10.0, 10.0), 0.0, &[n, n]);
        let base = bb.build();
        // Pin 2 (round) at (10, 12.54): out horizontally, 45° bend at its edge, then on.
        let a = p(10.0, 12.54);
        let pts = [a, p(11.1, 12.54), p(12.1, 13.54), p(20.0, 13.54), p(22.0, 15.54)];
        let edit = BoardEdit { new_tracks: trk(n, 0, &pts), ..BoardEdit::default() };
        let mut before = base.clone();
        before.apply(&edit);
        assert_eq!(ferroute_drc::quality::near_pad_bends(&before).len(), 1);
        let out = straighten(&base, &edit);
        let mut after = base;
        after.apply(&out);
        assert!(ferroute_drc::quality::near_pad_bends(&after).is_empty());
        let rep = check(&after, &DrcConfig { courtyards: false, ..DrcConfig::default() });
        assert_eq!(rep.introduced().count(), 0);
    }
}
