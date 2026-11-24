// SPDX-License-Identifier: AGPL-3.0-only
//! Area-array (BGA/LGA) escape planning, a pre-routing stage.
//!
//! A lattice search alone breaks out a ball grid array poorly: the channels between balls
//! hold one or two tracks, so the order in which balls claim them matters, and the vias
//! that dog-bone deep balls sit in the *interstices* between four balls, which are off the
//! lattice at fine pitches. This stage plans the break-out the way an expert does it,
//! deterministically and exactly:
//!
//! 1. [`detect_arrays`] finds footprints whose pads form a uniform grid (≥ 4 × 4).
//! 2. [`plan`] visits balls ring by ring (outermost first). Each eligible ball gets the
//!    first legal escape of an ordered candidate list: no-via candidates before via
//!    candidates (the shallow rings keep to the top layer, deep rings dog-bone first), the
//!    nearest array side first. A candidate is an octilinear polyline that leaves the ball
//!    centre on an entry axis (straight, or along a 45° diagonal for `d` and then straight
//!    outward), optionally changes layer with a dog-bone via at that corner (the
//!    interstice centre for `d = pitch / 2`), runs straight past the array boundary and
//!    finishes with a **45° departure stub** that lands on a lattice node. Channel
//!    capacity needs no formula: a candidate is accepted only if it is exactly legal
//!    against every fixed obstacle and every escape accepted before it.
//! 3. The departure stub makes the follow-on route continue straight or at 45° (a turn
//!    to the axis, never a 90° corner), and the stub ends of neighbouring escapes are
//!    staggered (alternating front rows) so their continuations do not run side by side
//!    into facing slivers.
//! 4. Via kinds follow the [`ViaModel`]: the cheapest legal kind per layer change
//!    (through, then micro, then blind), exactly as the lattice search prices them.
//!
//! The escape copper becomes fixed copper of its net; the PathFinder then routes from each
//! escape end (a terminal with a heading) to the rest of the net. Escapes the oracle gate
//! rejects are ripped like any other routed copper.

use crate::grid::Grid;
use crate::legality::{ClassGeom, Obstacles, allows};
use crate::vspan::ViaModel;
use ferroute_board::custom::{ItemCtx, ItemType};
use ferroute_board::{Board, LayerId, LayerSet, NetId, Origin, PadId, Track, Via, ViaKind};
use ferroute_drc::zones::FillPoly;
use ferroute_geom::{Coord, Dir8, Hull, Point, Rect};

/// How far past the array boundary the escape lines end, in lattice pitches.
pub const FRONT_MARGIN_STEPS: i64 = 3;
/// Array sides tried per ball (nearest first).
const MAX_SIDES: usize = 2;
/// Smallest area array: balls per row and column.
const MIN_DIM: usize = 4;
/// First ring that prefers a dog-bone via to a free layer over a channel on the top layer.
const DEEP_RING: usize = 2;
/// Outermost ring planned by default ([`crate::RouterConfig::escape_rings`]).
///
/// Measured on the dense BGA suite: the outer ring's straight channel-free break-outs
/// raise completion, while pre-planning inner rings (channel and dog-bone escapes chosen
/// first-fit) takes capacity from neighbours that the negotiation would have shared better
/// (bga_256_4l fixed placement: ring 0 only 94.6 %, rings 0-1 89.5 %, rings 0-2 85.7 %,
/// no escapes 89.0 %).
pub const DEFAULT_MAX_RING: usize = 0;
/// Deepest nesting level of a fan-out (levels beyond share the last row).
const MAX_LEVEL: usize = 12;
/// Longest departure stub, in lattice pitches of lateral shift.
const MAX_JOG_STEPS: i64 = 2;

/// A ball of an [`Array`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ArrayPad {
    pub pad: PadId,
    pub centre: Point,
    /// Column and row index in the array grid (0 = left / top).
    pub col: usize,
    pub row: usize,
    /// Distance in balls to the nearest array side (0 = outermost ring).
    pub ring: usize,
}

/// A footprint whose pads form a uniform grid.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Array {
    pub pads: Vec<ArrayPad>,
    pub pitch: Coord,
    pub cols: usize,
    pub rows: usize,
    /// Bounding box of the ball centres.
    pub bbox: Rect,
}

/// Where an escape ends: a lattice node from which the router continues.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EscapeEnd {
    pub point: Point,
    pub layer: LayerId,
    /// Direction from the end towards the ball (the last leg, reversed). Always a 45°
    /// diagonal: the continuation can go straight on or turn 45° onto an axis, but never
    /// meets the escape at a right angle.
    pub heading: Dir8,
}

/// Planned break-out copper of one ball.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Escape {
    pub pad: PadId,
    pub net: NetId,
    pub tracks: Vec<Track>,
    pub vias: Vec<Via>,
    pub end: EscapeEnd,
}

/// Finds the area arrays of `board` (deterministic, footprint order).
#[must_use]
pub fn detect_arrays(board: &Board) -> Vec<Array> {
    let mut out = Vec::new();
    for fp in &board.footprints {
        let pads: Vec<(PadId, Point)> = fp
            .pads
            .iter()
            .copied()
            .filter(|&id| {
                let p = board.pad(id);
                p.drill.is_none() && !p.layers.is_empty()
            })
            .map(|id| (id, board.pad_position(id)))
            .collect();
        if pads.len() < MIN_DIM * MIN_DIM {
            continue;
        }
        let axis = |f: &dyn Fn(Point) -> Coord| {
            let mut v: Vec<Coord> = pads.iter().map(|&(_, p)| f(p)).collect();
            v.sort_unstable();
            v.dedup();
            v
        };
        let (xs, ys) = (axis(&|p| p.x), axis(&|p| p.y));
        let step = |v: &[Coord]| v.windows(2).map(|w| w[1] - w[0]).min().unwrap_or(0);
        let pitch = step(&xs);
        if xs.len() < MIN_DIM
            || ys.len() < MIN_DIM
            || pitch <= 0
            || step(&ys) != pitch
            || xs.iter().any(|x| (x - xs[0]) % pitch != 0)
            || ys.iter().any(|y| (y - ys[0]) % pitch != 0)
        {
            continue;
        }
        let cols = usize::try_from((xs[xs.len() - 1] - xs[0]) / pitch).unwrap_or(0) + 1;
        let rows = usize::try_from((ys[ys.len() - 1] - ys[0]) / pitch).unwrap_or(0) + 1;
        // A real array fills at least half of its grid (rejects QFN-like rings of pads).
        if pads.len() * 2 < cols * rows {
            continue;
        }
        let mut seen = std::collections::BTreeSet::new();
        let mut ap: Vec<ArrayPad> = Vec::with_capacity(pads.len());
        for &(pad, centre) in &pads {
            let col = usize::try_from((centre.x - xs[0]) / pitch).unwrap_or(0);
            let row = usize::try_from((centre.y - ys[0]) / pitch).unwrap_or(0);
            if !seen.insert((col, row)) {
                continue;
            }
            let ring = col.min(row).min(cols - 1 - col).min(rows - 1 - row);
            ap.push(ArrayPad { pad, centre, col, row, ring });
        }
        if ap.len() != pads.len() {
            continue;
        }
        out.push(Array {
            pads: ap,
            pitch,
            cols,
            rows,
            bbox: Rect::new(
                Point::new(xs[0], ys[0]),
                Point::new(xs[xs.len() - 1], ys[ys.len() - 1]),
            ),
        });
    }
    out
}

#[derive(Clone, Debug)]
struct Leg {
    a: Point,
    b: Point,
    layer: LayerId,
}

/// A planned via: position, kind, the copper layers it spans and its sizes.
#[derive(Clone, Copy, Debug)]
struct ViaSpec {
    pos: Point,
    kind: ViaKind,
    layers: LayerSet,
    diameter: Coord,
    drill: Coord,
}

#[derive(Clone, Debug)]
struct Cand {
    legs: Vec<Leg>,
    via: Option<ViaSpec>,
    end: EscapeEnd,
}

/// A layer a candidate may change to, with the via kinds that can do it (cheapest first).
#[derive(Clone, Debug)]
struct ViaOpt {
    layer: LayerId,
    kinds: Vec<ViaKind>,
}

/// Copper accepted so far (hulls with the layer and net they belong to).
struct Accepted {
    net: NetId,
    legs: Vec<(Hull, LayerId)>,
    via: Option<(Hull, Hull, LayerSet)>,
}

struct Planner<'a> {
    board: &'a Board,
    obs: &'a Obstacles,
    geoms: &'a [ClassGeom],
    grid: &'a Grid,
    vm: &'a ViaModel,
    /// Stored pours (including those that yield to routed copper).
    fills: Vec<FillPoly<'a>>,
    /// Layers that carry a pour.
    fill_layers: LayerSet,
    /// Smallest edge-to-edge gap between two vias that keeps a pour between them alive
    /// (two anti-pad clearances plus the pour's minimum thickness); 0 without pours.
    plane_gap: Coord,
    accepted: Vec<Accepted>,
    clr: Vec<Option<Coord>>,
}

/// Outward unit vectors in the fixed tie-break order N, S, W, E (KiCad y points down).
const SIDES: [(i64, i64); 4] = [(0, -1), (0, 1), (-1, 0), (1, 0)];

impl Planner<'_> {
    fn clearance(&mut self, net: NetId) -> Coord {
        let i = net.idx();
        if let Some(c) = self.clr[i] {
            return c;
        }
        let c = self.board.rules.max_clearance_for_net(self.board, net);
        self.clr[i] = Some(c);
        c
    }

    /// Drill-to-drill distance two escape vias of `a` and `b` need (custom per-pair rule,
    /// else the board-wide value).
    fn hole_to_hole(&self, a: NetId, b: NetId) -> Coord {
        let r = &self.board.rules;
        if r.custom.is_empty() {
            r.hole_to_hole
        } else {
            r.hole_to_hole_for(
                self.board,
                &ItemCtx::new(a, ItemType::Via),
                &ItemCtx::new(b, ItemType::Via),
            )
        }
    }

    fn geom(&self, net: NetId) -> ClassGeom {
        self.geoms[usize::from(self.board.nets[net.idx()].class).min(self.geoms.len() - 1)]
    }

    /// Exact legality of `c` for `net` against fixed obstacles and accepted escapes; on
    /// success the candidate's copper is recorded as accepted.
    fn legal(&mut self, c: &Cand, net: NetId) -> bool {
        let cg = self.geom(net);
        let my = self.clearance(net);
        let legs: Vec<(Hull, LayerId)> =
            c.legs.iter().map(|l| (Hull::capsule(l.a, l.b, cg.track_radius), l.layer)).collect();
        for (h, l) in &legs {
            if !allows(self.obs.classify_track(h, *l, &cg), net) {
                return false;
            }
        }
        let via = c
            .via
            .map(|v| (Hull::disk(v.pos, v.diameter / 2), Hull::disk(v.pos, v.drill / 2), v.layers));
        if let Some(v) = &c.via {
            let state = if v.kind == ViaKind::Through {
                self.obs.classify_via(v.pos, &cg)
            } else {
                self.obs.classify_span_via(v.pos, &cg, v.diameter, v.drill, v.layers)
            };
            if !allows(state, net) {
                return false;
            }
        }
        let (plane_gap, fill_layers) = (self.plane_gap, self.fill_layers);
        for k in 0..self.accepted.len() {
            let other = self.accepted[k].net;
            let clr = my.max(self.clearance(other));
            let a = &self.accepted[k];
            for (h, l) in &legs {
                if a.legs.iter().any(|(g, gl)| gl == l && h.clearance_violated(g, clr))
                    || a.via
                        .as_ref()
                        .is_some_and(|(vc, _, vl)| vl.contains(*l) && h.clearance_violated(vc, clr))
                {
                    return false;
                }
            }
            if let Some((vc, vd, vl)) = &via {
                if a.legs.iter().any(|(g, gl)| vl.contains(*gl) && vc.clearance_violated(g, clr)) {
                    return false;
                }
                if let Some((oc, od, ol)) = &a.via {
                    let shared = LayerSet(vl.0 & ol.0);
                    let gap = if shared.intersects(fill_layers) { clr.max(plane_gap) } else { clr };
                    let hole = self.hole_to_hole(net, other);
                    if (!shared.is_empty() && vc.clearance_violated(oc, gap))
                        || vd.clearance_violated(od, hole)
                    {
                        return false;
                    }
                }
            }
        }
        self.accepted.push(Accepted { net, legs, via });
        true
    }
}

fn mk(ux: i64, lat: Coord, along: Coord, uy: i64) -> Point {
    if ux == 0 { Point::new(lat, along * uy) } else { Point::new(along * ux, lat) }
}

fn lat_of(ux: i64, p: Point) -> Coord {
    if ux == 0 { p.x } else { p.y }
}

fn along_of(ux: i64, uy: i64, p: Point) -> Coord {
    if ux == 0 { p.y * uy } else { p.x * ux }
}

/// Parameters of one candidate polyline.
#[derive(Clone, Copy, Debug)]
struct Shape {
    /// Outward axis of the side (unit vector).
    u: (i64, i64),
    /// Side of the first diagonal (±1; ignored for `d == 0`).
    s: i64,
    /// Extent of the first diagonal per axis (`0` = none).
    d: Coord,
    /// `along` coordinate (metric of `u`) of the lattice row the stub ends on.
    front: Coord,
    /// Signed lateral shift of the departure stub (never 0).
    delta: Coord,
}

/// Builds one candidate: ball centre `p0` → diagonal run of `d` per axis towards side `u`
/// (`d = 0`: none) → [via] → straight outward → 45° departure stub of lateral shift
/// `|delta|` ending on the lattice row `front`.
fn build(
    p0: Point,
    sh: Shape,
    layer0: LayerId,
    via: Option<(LayerId, ViaSpecKind)>,
) -> Option<Cand> {
    let Shape { u: (ux, uy), s, d, front, delta } = sh;
    let w = if ux == 0 { (s, 0) } else { (0, s) };
    let corner = if d > 0 { Point::new(p0.x + d * (ux + w.0), p0.y + d * (uy + w.1)) } else { p0 };
    if d == 0 && via.is_some() {
        return None;
    }
    let (c_lat, c_along) = (lat_of(ux, corner), along_of(ux, uy, corner));
    let e = delta.abs();
    let j_along = front - e;
    if e == 0 || j_along < c_along {
        return None;
    }
    let end = mk(ux, c_lat + delta, front, uy);
    let j = mk(ux, c_lat, j_along, uy);
    let layer1 = via.map_or(layer0, |(l, _)| l);
    let mut legs = Vec::new();
    if d > 0 {
        legs.push(Leg { a: p0, b: corner, layer: layer0 });
    }
    if j != corner {
        legs.push(Leg { a: corner, b: j, layer: layer1 });
    }
    legs.push(Leg { a: j, b: end, layer: layer1 });
    let last = legs.last()?;
    let heading = Dir8::from_vector(Point::new(last.a.x - last.b.x, last.a.y - last.b.y))?;
    Some(Cand {
        legs,
        via: via.map(|(_, k)| ViaSpec {
            pos: corner,
            kind: k.kind,
            layers: k.layers,
            diameter: k.diameter,
            drill: k.drill,
        }),
        end: EscapeEnd { point: end, layer: layer1, heading },
    })
}

/// The kind-dependent part of a via, resolved by the caller.
#[derive(Clone, Copy, Debug)]
struct ViaSpecKind {
    kind: ViaKind,
    layers: LayerSet,
    diameter: Coord,
    drill: Coord,
}

/// Escape plan for the eligible balls of the rings `0..=max_ring` of every array.
/// `eligible` decides per pad whether it still needs copper (the caller knows islands,
/// direct links and planes).
#[must_use]
#[allow(clippy::too_many_arguments)]
pub fn plan(
    board: &Board,
    obs: &Obstacles,
    geoms: &[ClassGeom],
    grid: &Grid,
    vm: &ViaModel,
    arrays: &[Array],
    max_ring: usize,
    eligible: &dyn Fn(PadId) -> bool,
) -> Vec<Escape> {
    let fills = ferroute_drc::zones::fill_polys(board);
    let plane_gap = fills
        .iter()
        .map(|f| 2 * f.clearance + board.zones[f.zone as usize].min_thickness)
        .max()
        .unwrap_or(0);
    let fill_layers = fills.iter().fold(LayerSet::EMPTY, |s, f| s.with(f.layer));
    let mut pl = Planner {
        board,
        obs,
        geoms,
        grid,
        vm,
        fills,
        fill_layers,
        plane_gap,
        accepted: Vec::new(),
        clr: vec![None; board.nets.len()],
    };
    let pads_by_net = board.pads_by_net();
    let mut out: Vec<Escape> = Vec::new();
    for arr in arrays {
        let mut order: Vec<&ArrayPad> = arr
            .pads
            .iter()
            .filter(|b| b.ring <= max_ring)
            .filter(|b| board.pad(b.pad).net != NetId::NONE && eligible(b.pad))
            .collect();
        order.sort_by_key(|b| (b.ring, b.row, b.col));
        let fan = fan_levels(board, arr, &order, &pads_by_net);
        for b in order {
            let net = board.pad(b.pad).net;
            let cg = pl.geom(net);
            let Some(layer0) =
                grid.layers.iter().copied().find(|&l| board.pad(b.pad).layers.contains(l))
            else {
                continue;
            };
            let aligned = |v: Coord, o: Coord| (v - o).rem_euclid(grid.pitch) == 0;
            if !(aligned(b.centre.x, grid.origin.x)
                && aligned(b.centre.y, grid.origin.y)
                && pads_by_net[net.idx()].len() >= 2)
            {
                continue;
            }
            let Some(c) =
                signal_escape(&mut pl, arr, b, net, layer0, &pads_by_net[net.idx()], &fan)
            else {
                continue;
            };
            let track = |l: &Leg| Track {
                start: l.a,
                end: l.b,
                width: cg.track_width,
                layer: l.layer,
                net,
                locked: false,
                origin: Origin::Routed,
            };
            out.push(Escape {
                pad: b.pad,
                net,
                tracks: c.legs.iter().map(track).collect(),
                vias: c
                    .via
                    .iter()
                    .map(|v| Via {
                        pos: v.pos,
                        diameter: v.diameter,
                        drill: v.drill,
                        layers: v.layers,
                        kind: v.kind,
                        net,
                        locked: false,
                        origin: Origin::Routed,
                    })
                    .collect(),
                end: c.end,
            });
        }
    }
    out
}

/// Distance in balls from `b` to array side `u`.
fn side_dist(arr: &Array, b: &ArrayPad, u: (i64, i64)) -> usize {
    match u {
        (0, -1) => b.row,
        (0, 1) => arr.rows - 1 - b.row,
        (-1, 0) => b.col,
        _ => arr.cols - 1 - b.col,
    }
}

/// Lateral shifts of the departure stub that land on a lattice line, nearest first. The
/// stub is at least half a pitch long (no tiny kinks) and `prefer` (±1) breaks ties.
fn stub_shifts(grid: &Grid, c_lat: Coord, olat: Coord, prefer: i64) -> Vec<Coord> {
    let p = grid.pitch;
    let mut v: Vec<Coord> = Vec::new();
    // Candidates are `lattice line - c_lat`: enumerate the lines.
    let first = olat + (c_lat - olat - MAX_JOG_STEPS * p).div_euclid(p) * p;
    let mut line = first;
    while line <= c_lat + MAX_JOG_STEPS * p {
        let delta = line - c_lat;
        if delta != 0 && 2 * delta.abs() >= p && delta.abs() <= MAX_JOG_STEPS * p {
            v.push(delta);
        }
        line += p;
    }
    v.sort_by_key(|&d| (-(d.signum() * prefer), d.abs()));
    v
}

/// Centroid of the other pads of `pad`'s net (where the connection has to go).
fn partner(board: &Board, net_pads: &[PadId], pad: PadId, own: Point) -> Point {
    let others: Vec<Point> =
        net_pads.iter().filter(|&&q| q != pad).map(|&q| board.pad_position(q)).collect();
    let (sx, sy) =
        others.iter().fold((0i128, 0i128), |a, q| (a.0 + i128::from(q.x), a.1 + i128::from(q.y)));
    let n = i128::try_from(others.len().max(1)).unwrap_or(1);
    Point::new(Coord::try_from(sx / n).unwrap_or(own.x), Coord::try_from(sy / n).unwrap_or(own.y))
}

/// Per `(pad index, outward side)`: the lateral turn direction of the connection
/// (`-1`, `0`, `+1` along the side) and the nesting level of its escape.
pub(crate) type Fan = std::collections::BTreeMap<(usize, (i64, i64)), (i64, i64)>;

/// Nested fan-out order of the balls on each side of `arr`. Balls whose connection turns
/// to the same lateral direction nest like the layers of an onion: the ball farthest from
/// the turn escapes farthest, so no connection has to cross another's escape. Levels are
/// capped; connections that run straight on stay on level 0.
pub(crate) fn fan_levels(
    board: &Board,
    arr: &Array,
    balls: &[&ArrayPad],
    pads_by_net: &[Vec<PadId>],
) -> Fan {
    let mut fan = Fan::new();
    for u in SIDES {
        let lat = |p: Point| if u.0 == 0 { p.x } else { p.y };
        let mut group: Vec<(&ArrayPad, i64)> = balls
            .iter()
            .copied()
            .filter(|b| side_dist(arr, b, u) == 0)
            .map(|b| {
                let net = board.pad(b.pad).net;
                let t = partner(board, &pads_by_net[net.idx()], b.pad, b.centre);
                let d = lat(t) - lat(b.centre);
                (b, if d.abs() < arr.pitch { 0 } else { d.signum() })
            })
            .collect();
        group.sort_by_key(|(b, _)| lat(b.centre));
        for (i, &(b, tau)) in group.iter().enumerate() {
            let level = match tau {
                1 => group[i + 1..].iter().filter(|(_, t)| *t == 1).count(),
                -1 => group[..i].iter().filter(|(_, t)| *t == -1).count(),
                _ => 0,
            };
            let level = i64::try_from(level.min(MAX_LEVEL)).unwrap_or(0);
            fan.insert((b.pad.idx(), u), (tau, level));
        }
    }
    fan
}

fn signal_escape(
    pl: &mut Planner<'_>,
    arr: &Array,
    b: &ArrayPad,
    net: NetId,
    layer0: LayerId,
    net_pads: &[PadId],
    fan: &Fan,
) -> Option<Cand> {
    let grid = pl.grid;
    let board = pl.board;
    let p = grid.pitch;
    let cg = pl.geom(net);
    // Ties between equally near sides go to the side facing the rest of the net.
    let target = partner(board, net_pads, b.pad, b.centre);
    let facing = |u: (i64, i64)| -> i128 {
        // Larger = target lies further along the outward direction.
        i128::from((target.x - b.centre.x) * u.0 + (target.y - b.centre.y) * u.1)
    };
    let entry = board.pad_entry_dirs(b.pad);
    let can_leave = |dx: i64, dy: i64| entry.contains(&(dx, dy));
    let mut sides: Vec<(i64, i64)> = SIDES.to_vec();
    sides.sort_by_key(|&u| (side_dist(arr, b, u), -facing(u)));
    sides.truncate(MAX_SIDES);
    // Diagonal extents: lattice multiples up to one ball pitch, plus the interstice centre.
    let mut ds: Vec<Coord> = (1..=arr.pitch / p).map(|k| k * p).collect();
    ds.push(arr.pitch / 2);
    ds.sort_unstable();
    ds.dedup();
    let l0 = grid.grid_layer(layer0)?;
    // Via options: other layers without a pour at the ball, each with its legal kinds
    // ordered by the router's own price list.
    let mut via_opts: Vec<Option<ViaOpt>> = vec![None];
    for (gl, &l) in grid.layers.iter().enumerate() {
        let gl = u32::try_from(gl).unwrap_or(u32::MAX);
        if l == layer0 || pl.fills.iter().any(|f| f.layer == l && f.poly.contains(b.centre)) {
            continue;
        }
        let (lo, hi) = (l0.min(gl), l0.max(gl));
        let mut kinds: Vec<ViaKind> = Vec::new();
        if pl.vm.policy.through {
            kinds.push(ViaKind::Through);
        }
        if let Some(k) = pl.vm.span_kind(lo, hi) {
            kinds.push(k);
        }
        kinds.sort_by(|&a, &c| pl.vm.factor(a).total_cmp(&pl.vm.factor(c)));
        if !kinds.is_empty() {
            via_opts.push(Some(ViaOpt { layer: l, kinds }));
        }
    }
    // Deep balls dog-bone to a free layer first; the top layer keeps its channels for the
    // outer rings.
    if b.ring >= DEEP_RING && via_opts.len() > 1 {
        via_opts.rotate_left(1);
    }
    let all = board.all_layers();
    for opt in &via_opts {
        // Via variants of this option: one per kind (cheapest first), or the no-via shape.
        let variants: Vec<Option<(LayerId, ViaSpecKind)>> = match opt {
            None => vec![None],
            Some(o) => {
                let gl = grid.grid_layer(o.layer)?;
                let (lo, hi) = (l0.min(gl), l0.max(gl));
                o.kinds
                    .iter()
                    .map(|&k| {
                        let (diameter, drill) = pl.vm.sizes(k, &cg);
                        let layers = if k == ViaKind::Through {
                            all
                        } else {
                            pl.vm.span_layers(grid, lo, hi)
                        };
                        Some((o.layer, ViaSpecKind { kind: k, layers, diameter, drill }))
                    })
                    .collect()
            }
        };
        for &u in &sides {
            let front0 = front_for(grid, arr, u);
            // The connection turns towards `tau`; its escape nests at `level` (see
            // [`fan_levels`]), and its departure stub jogs the same way.
            let (tau, level) = fan.get(&(b.pad.idx(), u)).copied().unwrap_or((0, 0));
            let par =
                i64::from(u8::try_from((if u.0 == 0 { b.col } else { b.row }) % 2).unwrap_or(0));
            for &d in std::iter::once(&0).chain(ds.iter()) {
                if d == 0 && opt.is_some() {
                    continue;
                }
                for s in [1i64, -1] {
                    if d == 0 && s == -1 {
                        continue;
                    }
                    let dir = if d == 0 {
                        u
                    } else {
                        let w = if u.0 == 0 { (s, 0) } else { (0, s) };
                        ((u.0 + w.0).signum(), (u.1 + w.1).signum())
                    };
                    if !can_leave(dir.0, dir.1) {
                        continue;
                    }
                    let w = if u.0 == 0 { (s, 0) } else { (0, s) };
                    let corner = if d > 0 {
                        Point::new(b.centre.x + d * (u.0 + w.0), b.centre.y + d * (u.1 + w.1))
                    } else {
                        b.centre
                    };
                    let (c_lat, olat) = if u.0 == 0 {
                        (corner.x, grid.origin.x)
                    } else {
                        (corner.y, grid.origin.y)
                    };
                    let prefer = if tau != 0 {
                        tau
                    } else if par == 0 {
                        1
                    } else {
                        -1
                    };
                    let shifts = stub_shifts(grid, c_lat, olat, prefer);
                    for extra in [0, 1] {
                        let front = front0 + (level + extra) * p;
                        for &delta in &shifts {
                            for v in &variants {
                                let sh = Shape { u, s, d, front, delta };
                                let Some(c) = build(b.centre, sh, layer0, *v) else { continue };
                                if pl.legal(&c, net) {
                                    return Some(c);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    None
}

/// `along` coordinate (metric of outward axis `u`) of the lattice row `FRONT_MARGIN_STEPS`
/// pitches past the array boundary, rounded outward.
fn front_for(grid: &Grid, arr: &Array, u: (i64, i64)) -> Coord {
    let p = grid.pitch;
    let reach = arr.pitch / 2 + FRONT_MARGIN_STEPS * p;
    let edge = match u {
        (0, -1) => -arr.bbox.min.y,
        (0, 1) => arr.bbox.max.y,
        (-1, 0) => -arr.bbox.min.x,
        _ => arr.bbox.max.x,
    } + reach;
    let o = match u {
        (0, -1) => -grid.origin.y,
        (0, 1) => grid.origin.y,
        (-1, 0) => -grid.origin.x,
        _ => grid.origin.x,
    };
    // Lattice lines are at o + k * p in the `along` metric (sign-adjusted origin).
    let k = (edge - o + p - 1).div_euclid(p);
    o + k * p
}

/// Lattice nodes and canonical forward edges of the routing lattice that the copper of
/// `escapes` (all of one net; track radius `r`) would overlap: the net's own lattice routes
/// must neither run over nor cross the planned break-outs. The end node of each escape is
/// excluded (the route starts there, governed by its heading).
#[must_use]
pub fn keepout(
    grid: &Grid,
    escapes: &[&Escape],
    r: Coord,
) -> (Vec<crate::grid::NodeIdx>, Vec<(crate::grid::NodeIdx, crate::grid::NodeIdx)>) {
    use ferroute_geom::Segment;
    let mut nodes = Vec::new();
    let mut edges = Vec::new();
    for e in escapes {
        let end_node = |l: u32| {
            let (i, j) = (
                u32::try_from((e.end.point.x - grid.origin.x) / grid.pitch).ok()?,
                u32::try_from((e.end.point.y - grid.origin.y) / grid.pitch).ok()?,
            );
            Some(grid.idx(i, j, l))
        };
        for (gl, &layer) in grid.layers.iter().enumerate() {
            let gl = u32::try_from(gl).unwrap_or(u32::MAX);
            let segs: Vec<Segment> = e
                .tracks
                .iter()
                .filter(|t| t.layer == layer)
                .map(|t| Segment::new(t.start, t.end))
                .collect();
            let q = if layer == e.end.layer { end_node(gl) } else { None };
            // Node ids are never `u32::MAX`, so this excludes nothing off the end layer.
            let q = q.unwrap_or(u32::MAX);
            let (n, ed) = crate::access::stub_keepout(grid, gl, q, &segs, r);
            nodes.extend(n);
            edges.extend(ed);
            for v in e.vias.iter().filter(|v| v.layers.contains(layer)) {
                let pt = [Segment::new(v.pos, v.pos)];
                let (n, ed) =
                    crate::access::stub_keepout(grid, gl, u32::MAX, &pt, (v.diameter / 2 + r) / 2);
                nodes.extend(n);
                edges.extend(ed);
            }
        }
    }
    nodes.sort_unstable();
    nodes.dedup();
    edges.sort_unstable();
    edges.dedup();
    (nodes, edges)
}
