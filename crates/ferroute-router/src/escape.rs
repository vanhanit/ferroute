// SPDX-License-Identifier: AGPL-3.0-only
//! Area-array (BGA/LGA) escape planning, a pre-routing stage.
//!
//! A ball grid array cannot be fanned out by a lattice search alone: the vias that dog-bone
//! deep balls sit in the *interstices* between four balls, which are off the lattice at
//! fine pitches, and the channels between balls hold only one or two tracks, so the order
//! in which balls claim them matters. This stage plans the break-out the way an expert
//! does it, deterministically and exactly:
//!
//! 1. [`detect_arrays`] finds footprints whose pads form a uniform grid (≥ 4 × 4).
//! 2. [`plan`] visits balls ring by ring (outermost first). Each signal ball gets the
//!    first legal escape of an ordered candidate list: no-via candidates before via
//!    candidates, the nearest array side first. A candidate is an octilinear polyline that
//!    leaves the ball centre on an entry axis (straight, or along a 45° diagonal for `d`
//!    and then straight outward), optionally changes layer with a dog-bone via at the
//!    corner (the interstice centre for `d = pitch / 2`), runs straight past the array
//!    boundary and ends on a lattice node (an optional 45° jog brings an off-lattice
//!    line onto the lattice). Channel capacity needs no formula: a candidate is accepted
//!    only if it is exactly legal against every fixed obstacle and every escape accepted
//!    before it.
//! 3. Balls of nets with a pour of their own get a dog-bone via into the interstice when
//!    that site is off the lattice (the lattice plane fan-out cannot reach it).
//!
//! The escape copper becomes fixed copper of its net; the PathFinder then routes from each
//! escape end (a stub-less terminal with a heading) to the rest of the net. Escapes the
//! oracle gate rejects are ripped like any other routed copper.

use crate::grid::Grid;
use crate::legality::{ClassGeom, Obstacles, allows};
use ferroute_board::{Board, LayerId, NetId, Origin, PadId, Track, Via};
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
    /// Direction from the end towards the ball (the last leg, reversed).
    pub heading: Dir8,
}

/// Planned break-out copper of one ball.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Escape {
    pub pad: PadId,
    pub net: NetId,
    pub tracks: Vec<Track>,
    pub vias: Vec<Via>,
    /// `None` for a via into the net's own pour (the ball is connected already).
    pub end: Option<EscapeEnd>,
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

#[derive(Clone, Debug)]
struct Cand {
    legs: Vec<Leg>,
    vias: Vec<Point>,
    end: Option<EscapeEnd>,
}

/// Copper accepted so far (hulls with the layer and net they belong to).
struct Accepted {
    net: NetId,
    legs: Vec<(Hull, LayerId)>,
    vias: Vec<(Hull, Hull)>,
}

struct Planner<'a> {
    board: &'a Board,
    obs: &'a Obstacles,
    geoms: &'a [ClassGeom],
    grid: &'a Grid,
    /// Stored pours (including those that yield to routed copper).
    fills: Vec<FillPoly<'a>>,
    /// Smallest edge-to-edge gap between two vias that keeps a pour between them alive
    /// (two anti-pad clearances plus the pour's minimum thickness); 0 without pours.
    plane_gap: Coord,
    accepted: Vec<Accepted>,
    clr: Vec<Option<Coord>>,
}

/// Outward unit vectors in the fixed tie-break order N, S, W, E (KiCad y points down).
const SIDES: [(i64, i64); 4] = [(0, -1), (0, 1), (-1, 0), (1, 0)];

fn sign(v: i64) -> i64 {
    v.signum()
}

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

    fn geom(&self, net: NetId) -> ClassGeom {
        self.geoms[usize::from(self.board.nets[net.idx()].class).min(self.geoms.len() - 1)]
    }

    /// Exact legality of `c` for `net` against fixed obstacles and accepted escapes.
    fn legal(&mut self, c: &Cand, net: NetId) -> bool {
        let cg = self.geom(net);
        let my = self.clearance(net);
        let legs: Vec<(Hull, LayerId)> =
            c.legs.iter().map(|l| (Hull::capsule(l.a, l.b, cg.track_radius), l.layer)).collect();
        let vias: Vec<(Hull, Hull)> = c
            .vias
            .iter()
            .map(|&p| (Hull::disk(p, cg.via_radius), Hull::disk(p, cg.via_drill / 2)))
            .collect();
        for (h, l) in &legs {
            if !allows(self.obs.classify_track(h, *l, &cg), net) {
                return false;
            }
        }
        for &p in &c.vias {
            if !allows(self.obs.classify_via(p, &cg), net) {
                return false;
            }
        }
        let hole = self.board.rules.hole_to_hole;
        let plane_gap = self.plane_gap;
        for k in 0..self.accepted.len() {
            let other = self.accepted[k].net;
            let clr = my.max(self.clearance(other));
            let a = &self.accepted[k];
            for (h, l) in &legs {
                if a.legs.iter().any(|(g, gl)| gl == l && h.clearance_violated(g, clr))
                    || a.vias.iter().any(|(vc, _)| h.clearance_violated(vc, clr))
                {
                    return false;
                }
            }
            for (vc, vd) in &vias {
                if a.legs.iter().any(|(g, _)| vc.clearance_violated(g, clr))
                    || a.vias.iter().any(|(oc, od)| {
                        vc.clearance_violated(oc, clr.max(plane_gap))
                            || vd.clearance_violated(od, hole)
                    })
                {
                    return false;
                }
            }
        }
        self.accepted.push(Accepted { net, legs, vias });
        true
    }
}

fn mk(ux: i64, uy: i64, lat: Coord, along: Coord) -> Point {
    if ux == 0 { Point::new(lat, along * uy) } else { Point::new(along * ux, lat) }
}

fn lat_of(ux: i64, p: Point) -> Coord {
    if ux == 0 { p.x } else { p.y }
}

fn along_of(ux: i64, uy: i64, p: Point) -> Coord {
    if ux == 0 { p.y * uy } else { p.x * ux }
}

/// Builds one candidate: ball centre `p0` → diagonal run of `d` per axis towards side `s`
/// (`d = 0`: none) → [via] → straight outward to the lattice row at `front` (an `along`
/// coordinate) → optional ≤ 45° jog onto lattice line `target_lat`.
#[allow(clippy::too_many_arguments)]
fn build(
    grid: &Grid,
    p0: Point,
    u: (i64, i64),
    s: i64,
    d: Coord,
    layer0: LayerId,
    via_layer: Option<LayerId>,
    front: Coord,
    lat_choice: usize,
) -> Option<Cand> {
    let (ux, uy) = u;
    let w = if ux == 0 { (s, 0) } else { (0, s) };
    let corner = if d > 0 { Point::new(p0.x + d * (ux + w.0), p0.y + d * (uy + w.1)) } else { p0 };
    let (olat, p) = (if ux == 0 { grid.origin.x } else { grid.origin.y }, grid.pitch);
    let c_lat = lat_of(ux, corner);
    let off = (c_lat - olat).rem_euclid(p);
    let (e_lat, e) = if off == 0 {
        if lat_choice != 0 {
            return None;
        }
        (c_lat, 0)
    } else if lat_choice == 0 {
        (c_lat - off, off)
    } else {
        (c_lat + p - off, p - off)
    };
    let c_along = along_of(ux, uy, corner);
    let j_along = front - e;
    if j_along < c_along {
        return None;
    }
    let end = mk(ux, uy, e_lat, front);
    let j = mk(ux, uy, c_lat, j_along);
    let layer1 = via_layer.unwrap_or(layer0);
    let mut legs = Vec::new();
    if d > 0 {
        legs.push(Leg { a: p0, b: corner, layer: layer0 });
    }
    if j != corner {
        legs.push(Leg { a: corner, b: j, layer: layer1 });
    }
    if e > 0 {
        legs.push(Leg { a: j, b: end, layer: layer1 });
    }
    let last = legs.last()?;
    let back = Point::new(last.a.x - last.b.x, last.a.y - last.b.y);
    let heading = Dir8::from_vector(back)?;
    Some(Cand {
        legs,
        vias: if via_layer.is_some() { vec![corner] } else { Vec::new() },
        end: Some(EscapeEnd { point: end, layer: layer1, heading }),
    })
}

/// Escape plan for every eligible ball of every array. `eligible` decides per pad whether
/// it still needs copper (the caller knows islands, direct links and planes): `Some(key)`
/// names the pad's island, of which a pour net needs only one escape via.
#[must_use]
pub fn plan(
    board: &Board,
    obs: &Obstacles,
    geoms: &[ClassGeom],
    grid: &Grid,
    arrays: &[Array],
    eligible: &dyn Fn(PadId) -> Option<usize>,
) -> Vec<Escape> {
    let fills = ferroute_drc::zones::fill_polys(board);
    let plane_gap = fills
        .iter()
        .map(|f| 2 * f.clearance + board.zones[f.zone as usize].min_thickness)
        .max()
        .unwrap_or(0);
    let mut pl = Planner {
        board,
        obs,
        geoms,
        grid,
        fills,
        plane_gap,
        accepted: Vec::new(),
        clr: vec![None; board.nets.len()],
    };
    let pads_by_net = board.pads_by_net();
    let mut out: Vec<Escape> = Vec::new();
    for arr in arrays {
        let mut order: Vec<(&ArrayPad, usize)> = arr
            .pads
            .iter()
            .filter(|b| board.pad(b.pad).net != NetId::NONE)
            .filter_map(|b| eligible(b.pad).map(|k| (b, k)))
            .collect();
        let pour_nets: Vec<NetId> = pl.fills.iter().map(|f| f.net).collect();
        let has_pour = |net: NetId| pour_nets.contains(&net);
        order
            .sort_by_key(|(b, _)| (b.ring, u8::from(has_pour(board.pad(b.pad).net)), b.row, b.col));
        let mut served: Vec<usize> = Vec::new();
        for (b, key) in order {
            let net = board.pad(b.pad).net;
            if served.contains(&key) {
                continue;
            }
            let cg = pl.geom(net);
            let Some(layer0) =
                grid.layers.iter().copied().find(|&l| board.pad(b.pad).layers.contains(l))
            else {
                continue;
            };
            let aligned = |v: Coord, o: Coord| (v - o).rem_euclid(grid.pitch) == 0;
            let res = if has_pour(net) {
                plane_escape(&mut pl, arr, b, net, layer0)
            } else if aligned(b.centre.x, grid.origin.x)
                && aligned(b.centre.y, grid.origin.y)
                && pads_by_net[net.idx()].len() >= 2
            {
                signal_escape(&mut pl, arr, b, net, layer0, &pads_by_net[net.idx()])
            } else {
                None
            };
            let Some(c) = res else { continue };
            if has_pour(net) {
                served.push(key);
            }
            let track = |l: &Leg| Track {
                start: l.a,
                end: l.b,
                width: cg.track_width,
                layer: l.layer,
                net,
                locked: false,
                origin: Origin::Routed,
            };
            let all = board.all_layers();
            out.push(Escape {
                pad: b.pad,
                net,
                tracks: c.legs.iter().map(track).collect(),
                vias: c
                    .vias
                    .iter()
                    .map(|&pos| Via {
                        pos,
                        diameter: cg.via_diameter,
                        drill: cg.via_drill,
                        layers: all,
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

fn signal_escape(
    pl: &mut Planner<'_>,
    arr: &Array,
    b: &ArrayPad,
    net: NetId,
    layer0: LayerId,
    net_pads: &[PadId],
) -> Option<Cand> {
    let grid = pl.grid;
    let board = pl.board;
    let p = grid.pitch;
    // Centroid of the net's other pads: ties between equally near sides go to the side
    // facing the rest of the net.
    let others: Vec<Point> =
        net_pads.iter().filter(|&&q| q != b.pad).map(|&q| board.pad_position(q)).collect();
    let (sx, sy) =
        others.iter().fold((0i128, 0i128), |a, q| (a.0 + i128::from(q.x), a.1 + i128::from(q.y)));
    let n = i128::try_from(others.len().max(1)).unwrap_or(1);
    let target = Point::new(
        Coord::try_from(sx / n).unwrap_or(b.centre.x),
        Coord::try_from(sy / n).unwrap_or(b.centre.y),
    );
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
    let other_layers: Vec<LayerId> = grid
        .layers
        .iter()
        .copied()
        .filter(|&l| {
            l != layer0 && !pl.fills.iter().any(|f| f.layer == l && f.poly.contains(b.centre))
        })
        .collect();
    let front_for = |u: (i64, i64)| -> Coord {
        let m = FRONT_MARGIN_STEPS * p;
        let reach = arr.pitch / 2 + m;
        // `along` coordinate of the array boundary + reach, rounded outward onto the lattice.
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
    };
    let mut via_opts: Vec<Option<LayerId>> = vec![None];
    via_opts.extend(other_layers.iter().map(|&l| Some(l)));
    // Deep balls dog-bone to a free layer first; the top layer keeps its channels for the
    // outer rings.
    if b.ring >= DEEP_RING && via_opts.len() > 1 {
        via_opts.rotate_left(1);
    }
    for via in via_opts {
        for &u in &sides {
            let front = front_for(u);
            for &d in std::iter::once(&0).chain(ds.iter()) {
                if d == 0 && via.is_some() {
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
                        (sign(u.0 + w.0), sign(u.1 + w.1))
                    };
                    if !can_leave(dir.0, dir.1) {
                        continue;
                    }
                    for lat_choice in 0..2 {
                        let Some(c) =
                            build(grid, b.centre, u, s, d, layer0, via, front, lat_choice)
                        else {
                            continue;
                        };
                        if pl.legal(&c, net) {
                            return Some(c);
                        }
                    }
                }
            }
        }
    }
    None
}

/// Dog-bone via into the interstice, only where the lattice cannot place that via itself.
fn plane_escape(
    pl: &mut Planner<'_>,
    arr: &Array,
    b: &ArrayPad,
    net: NetId,
    layer0: LayerId,
) -> Option<Cand> {
    let grid = pl.grid;
    let cg = pl.geom(net);
    let h = arr.pitch / 2;
    let entry = pl.board.pad_entry_dirs(b.pad);
    let mut diags: Vec<(i64, i64)> = vec![(-1, -1), (1, -1), (-1, 1), (1, 1)];
    let toward =
        |sx: i64, sy: i64| -> usize { side_dist(arr, b, (sx, 0)) + side_dist(arr, b, (0, sy)) };
    diags.sort_by_key(|&(sx, sy)| toward(sx, sy));
    for (sx, sy) in diags {
        if !entry.contains(&(sx, sy)) {
            continue;
        }
        let corner = Point::new(b.centre.x + sx * h, b.centre.y + sy * h);
        let on_lattice = (corner.x - grid.origin.x).rem_euclid(grid.pitch) == 0
            && (corner.y - grid.origin.y).rem_euclid(grid.pitch) == 0;
        if on_lattice {
            return None;
        }
        let in_pour = pl.fills.iter().any(|f| {
            f.net == net
                && grid.layers.contains(&f.layer)
                && f.poly.contains_disk(corner, cg.via_radius)
        });
        if !in_pour {
            continue;
        }
        let c = Cand {
            legs: vec![Leg { a: b.centre, b: corner, layer: layer0 }],
            vias: vec![corner],
            end: None,
        };
        if pl.legal(&c, net) {
            return Some(c);
        }
    }
    None
}
