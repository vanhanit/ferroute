// SPDX-License-Identifier: AGPL-3.0-only
//! Differential-pair routing: each pair is routed *before* the single nets as one coupled
//! unit.
//!
//! 1. A **centre-line** is searched on a private octilinear lattice anchored at the pair's
//!    start (A*, ≤ 45° turns, fat-track legality: the capsule that covers both traces plus
//!    the mitre overshoot, checked exactly against every fixed obstacle).
//! 2. The centre-line is split into two **exact integer offsets** ([`pairgeom`]) joined to
//!    their pads by symmetric **fans** (straight lead along the pad axis, then a 45° jog).
//! 3. Every candidate is verified by the exact DRC oracle (and the routing-quality
//!    predicates) before it is accepted; the cheapest passing candidate wins.
//!
//! Pairs stay on one layer (no vias) and need two single-pad terminals per net, pads that
//! sit side by side across a shared axis, and the same polarity side at both ends. A pair
//! that cannot be coupled is left to the ordinary router (two independent nets) and reported.

use crate::access::entry_dirs;
use crate::legality::{ClassGeom, Obstacles, allows};
use crate::pairgeom::{Fan, fan, offset_polyline, simplify};
use ferroute_board::pairs::{DiffPair, pair_width};
use ferroute_board::{Board, LayerId, NetId, Origin, PadId, Track};
use ferroute_drc::{DrcConfig, quality};
use ferroute_geom::{Coord, Dir8, Hull, Point, Rect};
use std::cmp::Reverse;
use std::collections::BinaryHeap;

/// Lattice steps of margin around the two ends in which the centre-line may wander.
const MARGIN_STEPS: i64 = 10;
/// Most lattice states a pair search may allocate.
const MAX_STATES: usize = 6_000_000;
/// Most candidates verified by the oracle per pair.
const MAX_VERIFIED: usize = 8;
/// Cost units: a lattice step is `100` (axis) or `141` (diagonal); a 45° bend costs this.
const BEND_COST: u32 = 60;
const TERMINAL: u32 = 1 << 31;

/// A coupled pair accepted by the router.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CoupledPair {
    /// Index into `Board::diff_pairs`.
    pub pair: usize,
    /// Copper of both nets (`Origin::Routed`).
    pub tracks: Vec<Track>,
}

/// Result of the pair stage.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PairStage {
    pub coupled: Vec<CoupledPair>,
    /// `(pair index, reason)` for pairs left to the ordinary router.
    pub uncoupled: Vec<(usize, String)>,
}

impl PairStage {
    /// All coupled copper.
    #[must_use]
    pub fn tracks(&self) -> Vec<Track> {
        self.coupled.iter().flat_map(|c| c.tracks.iter().cloned()).collect()
    }

    /// Nets whose copper is owned by the pair stage.
    #[must_use]
    pub fn nets(&self, board: &Board) -> Vec<NetId> {
        self.coupled
            .iter()
            .flat_map(|c| [board.diff_pairs[c.pair].p, board.diff_pairs[c.pair].n])
            .collect()
    }
}

/// Exact legality of copper of the pair on one layer against fixed obstacles.
struct Probe<'a> {
    obs: &'a Obstacles,
    cg: ClassGeom,
    layer: LayerId,
    nets: [NetId; 2],
}

impl Probe<'_> {
    fn ok(&self, a: Point, b: Point) -> bool {
        let h = Hull::capsule(a, b, self.cg.track_radius);
        let st = self.obs.classify_track(&h, self.layer, &self.cg);
        allows(st, self.nets[0]) || allows(st, self.nets[1])
    }
}

fn vec_of(d: Dir8) -> Point {
    let (x, y) = d.delta();
    Point::new(i64::from(x), i64::from(y))
}

/// Cost of a straight run of `len` nm along an axis (`diag = false`) or a diagonal.
fn run_cost(len: Coord, diag: bool, pitch: Coord) -> u32 {
    let unit = if diag { 141 } else { 100 };
    u32::try_from(len.max(0) * unit / pitch.max(1)).unwrap_or(u32::MAX / 4)
}

/// Octile lower bound of the remaining cost from `q` to `goal`.
fn heuristic(q: Point, goal: Point, pitch: Coord) -> u32 {
    let (dx, dy) = ((goal.x - q.x).abs(), (goal.y - q.y).abs());
    run_cost(dx.min(dy), true, pitch) + run_cost(dx.max(dy) - dx.min(dy), false, pitch)
}

/// What the centre-line search needs about the two ends.
struct Ends {
    /// Start of the coupled section at end A and the outward direction there.
    start: (Point, Dir8),
    /// End of the coupled section at end B and the direction of travel arriving there.
    goal: (Point, Dir8),
    pitch: Coord,
    /// Shortest straight run into `goal`.
    lmin: Coord,
    window: Rect,
}

/// Bridge from lattice node `q` (arrived with heading `u`) to the goal: an optional
/// diagonal that absorbs the lateral offset, then a straight run of at least `lmin`.
fn finish(probe: &Probe<'_>, e: &Ends, q: Point, u: Dir8) -> Option<(u32, Vec<Point>)> {
    let (mb, tb) = e.goal;
    let t = vec_of(tb);
    let n = Point::new(-t.y, t.x);
    let rel = mb - q;
    let (w, v) = (Coord::try_from(rel.dot(t)).ok()?, Coord::try_from(rel.dot(n)).ok()?);
    // The first run out of the fan is straight, so no bend may sit at the start itself.
    if w < e.lmin || (v == 0 && u.turn(tb) > 1) || (v != 0 && q == e.start.0) {
        return None;
    }
    if v == 0 {
        let bend = if u == tb { 0 } else { BEND_COST };
        return probe.ok(q, mb).then(|| (run_cost(w, false, e.pitch) + bend, vec![mb]));
    }
    let av = v.abs();
    if av > w - e.lmin {
        return None;
    }
    let g = Dir8::from_vector(t + n * v.signum())?;
    if u.turn(g) > 1 {
        return None;
    }
    let r = q + t * av + n * v;
    if !probe.ok(q, r) || !probe.ok(r, mb) {
        return None;
    }
    let bends = BEND_COST * (u32::from(u != g) + 1);
    let cost = run_cost(av, true, e.pitch) + run_cost(w - av, false, e.pitch) + bends;
    Some((cost, vec![r, mb]))
}

/// A* centre-line from the start to the goal of `e`; `None` if no legal route exists.
fn centre_line(probe: &Probe<'_>, e: &Ends) -> Option<(u32, Vec<Point>)> {
    let (ma, da) = e.start;
    let p = e.pitch;
    let lo = |v: Coord, o: Coord| (v - o).div_euclid(p) - 1;
    let hi = |v: Coord, o: Coord| (v - o).div_euclid(p) + 2;
    let (i0, i1) = (lo(e.window.min.x, ma.x), hi(e.window.max.x, ma.x));
    let (j0, j1) = (lo(e.window.min.y, ma.y), hi(e.window.max.y, ma.y));
    let (w, h) = (usize::try_from(i1 - i0 + 1).ok()?, usize::try_from(j1 - j0 + 1).ok()?);
    let nodes = w.checked_mul(h)?;
    if nodes.checked_mul(8)? > MAX_STATES {
        return None;
    }
    let pos = |id: usize| {
        let (i, j) =
            (i64::try_from(id % w).unwrap_or(0) + i0, i64::try_from(id / w).unwrap_or(0) + j0);
        ma + Point::new(i * p, j * p)
    };
    let node_of = |i: i64, j: i64| -> Option<usize> {
        ((i0..=i1).contains(&i) && (j0..=j1).contains(&j)).then(|| {
            usize::try_from((j - j0) * i64::try_from(w).unwrap_or(0) + (i - i0)).unwrap_or(0)
        })
    };
    let start_node = node_of(0, 0)?;
    let start = u32::try_from(start_node * 8 + da.index()).ok()?;
    let mut dist = vec![u32::MAX; nodes * 8];
    let mut prev = vec![u32::MAX; nodes * 8];
    let mut edge: Vec<u8> = vec![0; nodes * 8];
    let mut heap: BinaryHeap<Reverse<(u32, u32, u32)>> = BinaryHeap::new();
    dist[start as usize] = 0;
    heap.push(Reverse((heuristic(ma, e.goal.0, p), 0, start)));
    while let Some(Reverse((_, g, s))) = heap.pop() {
        if s & TERMINAL != 0 {
            let s = s & !TERMINAL;
            let (node, dir) = ((s / 8) as usize, Dir8::ALL[(s % 8) as usize]);
            let (_, tail) = finish(probe, e, pos(node), dir)?;
            let mut chain = vec![s];
            while prev[*chain.last()? as usize] != u32::MAX {
                chain.push(prev[*chain.last()? as usize]);
            }
            let mut pts: Vec<Point> = chain.iter().rev().map(|&c| pos((c / 8) as usize)).collect();
            pts.extend(tail);
            return Some((g, simplify(&pts)));
        }
        if g > dist[s as usize] {
            continue;
        }
        let (node, dir) = ((s / 8) as usize, Dir8::ALL[(s % 8) as usize]);
        let q = pos(node);
        if let Some((fc, _)) = finish(probe, e, q, dir) {
            heap.push(Reverse((g + fc, g + fc, s | TERMINAL)));
        }
        let forced = [dir];
        let turns = [Dir8::ALL[(dir.index() + 7) % 8], dir, Dir8::ALL[(dir.index() + 1) % 8]];
        let dirs: &[Dir8] = if s == start { &forced } else { &turns };
        let (qi, qj) =
            (i64::try_from(node % w).unwrap_or(0) + i0, i64::try_from(node / w).unwrap_or(0) + j0);
        for &nd in dirs {
            let (di, dj) = nd.delta();
            let Some(next) = node_of(qi + i64::from(di), qj + i64::from(dj)) else { continue };
            let slot = node * 8 + nd.index();
            if edge[slot] == 0 {
                edge[slot] = if probe.ok(q, pos(next)) { 1 } else { 2 };
            }
            if edge[slot] != 1 {
                continue;
            }
            let step = if nd.is_diagonal() { 141 } else { 100 };
            let cost = g + step + if nd == dir { 0 } else { BEND_COST };
            let ns = next * 8 + nd.index();
            if cost < dist[ns] {
                dist[ns] = cost;
                prev[ns] = s;
                let f = cost + heuristic(pos(next), e.goal.0, p);
                heap.push(Reverse((f, cost, u32::try_from(ns).ok()?)));
            }
        }
    }
    None
}

/// One pad end of a pair.
#[derive(Clone, Copy)]
struct PadEnd {
    p: PadId,
    n: PadId,
}

const AXES: [(i64, i64); 4] = [(1, 0), (0, 1), (-1, 0), (0, -1)];

/// Shortest lead along `d` that puts the first bend outside both pads' bend zone.
fn lead_min(board: &Board, e: PadEnd, d: Point, w: Coord, clearance: Coord) -> Coord {
    let ext = |pad: PadId| {
        let c = board.pad_position(pad);
        let b = board.pad_hull(pad).bbox();
        (b.max.x - c.x) * d.x.max(0)
            + (c.x - b.min.x) * (-d.x).max(0)
            + (b.max.y - c.y) * d.y.max(0)
            + (c.y - b.min.y) * (-d.y).max(0)
    };
    let raw = ext(e.p).max(ext(e.n)) + w + clearance + 10_000;
    (raw + 9_999) / 10_000 * 10_000
}

/// Axis directions (outward) along which both pads of `e` may be entered and which cross
/// the pair side by side.
fn end_dirs(board: &Board, e: PadEnd) -> Vec<Point> {
    let (dp, dn) = (entry_dirs(board, e.p), entry_dirs(board, e.n));
    let (pp, pn) = (board.pad_position(e.p), board.pad_position(e.n));
    AXES.iter()
        .filter(|a| dp.contains(a) && dn.contains(a))
        .map(|&(x, y)| Point::new(x, y))
        .filter(|&d| pp.dot(d) == pn.dot(d))
        .collect()
}

/// Splits the pair's two single-pad terminals per net into the two ends.
fn pad_ends(
    board: &Board,
    islands: &[Vec<Vec<PadId>>],
    pair: &DiffPair,
) -> Result<[PadEnd; 2], String> {
    let single = |net: NetId| -> Option<[PadId; 2]> {
        match islands.get(net.idx())?.as_slice() {
            [a, b] if a.len() == 1 && b.len() == 1 => Some([a[0], b[0]]),
            _ => None,
        }
    };
    let (Some([p0, p1]), Some([n0, n1])) = (single(pair.p), single(pair.n)) else {
        return Err("each net needs exactly two single-pad terminals".into());
    };
    let d = |a: PadId, b: PadId| board.pad_position(a).dist(board.pad_position(b));
    Ok(if d(p0, n0) + d(p1, n1) <= d(p0, n1) + d(p1, n0) {
        [PadEnd { p: p0, n: n0 }, PadEnd { p: p1, n: n1 }]
    } else {
        [PadEnd { p: p0, n: n1 }, PadEnd { p: p1, n: n0 }]
    })
}

/// A fully built candidate, not yet verified.
struct Candidate {
    cost: u32,
    layer: LayerId,
    p: Vec<Point>,
    n: Vec<Point>,
}

fn dir8(v: Point) -> Option<Dir8> {
    Dir8::from_vector(v)
}

#[allow(clippy::too_many_arguments)]
fn candidates(
    board: &Board,
    obs: &Obstacles,
    pair: &DiffPair,
    ends: [PadEnd; 2],
    layers: &[LayerId],
    pitch: Coord,
    w: Coord,
    spacing: Coord,
) -> Result<Vec<Candidate>, String> {
    let hs = spacing / 2;
    let clearance = board
        .rules
        .class_of(board, pair.p)
        .clearance
        .max(board.rules.class_of(board, pair.n).clearance)
        .max(1);
    let fat_r = hs + w / 2 + hs / 10 + 4;
    let class = board.rules.class_of(board, pair.p);
    let cg = ClassGeom {
        track_width: 2 * fat_r,
        track_radius: fat_r,
        clearance,
        via_diameter: class.via_diameter,
        via_radius: class.via_diameter / 2,
        via_drill: class.via_drill,
    };
    let (da, db) = (end_dirs(board, ends[0]), end_dirs(board, ends[1]));
    if da.is_empty() || db.is_empty() {
        return Err("pads are not side by side across a shared entry axis".into());
    }
    let pos = |pad: PadId| board.pad_position(pad);
    let bbox = board.outline.bbox().ok_or("no outline")?;
    let mut out = Vec::new();
    for &layer in layers {
        let probe = Probe { obs, cg, layer, nets: [pair.p, pair.n] };
        for &d_a in &da {
            for &d_b in &db {
                let (la, lb) = (
                    lead_min(board, ends[0], d_a, w, clearance),
                    lead_min(board, ends[1], d_b, w, clearance),
                );
                for extra in [0, 300_000, 800_000, 1_600_000] {
                    let fa = fan(pos(ends[0].p), pos(ends[0].n), d_a, la + extra, spacing);
                    let fb = fan(pos(ends[1].p), pos(ends[1].n), d_b, lb + extra, spacing);
                    let (Some(fa), Some(fb)) = (fa, fb) else { continue };
                    // Polarity side: P must be on the same side of the direction of travel.
                    if fa.p_left != !fb.p_left {
                        continue;
                    }
                    let (Some(sa), Some(tb)) = (dir8(d_a), dir8(-d_b)) else { continue };
                    let region = Rect::from_points(&[fa.merge, fb.merge]).unwrap_or(bbox);
                    let ends_s = Ends {
                        start: (fa.merge, sa),
                        goal: (fb.merge, tb),
                        pitch,
                        lmin: hs.max(pitch / 2),
                        window: region.inflate(MARGIN_STEPS * pitch),
                    };
                    let Some((cost, centre)) = centre_line(&probe, &ends_s) else { continue };
                    if let Some(c) = build(&fa, &fb, &centre, hs, cost, layer) {
                        out.push(c);
                    }
                }
            }
        }
    }
    Ok(out)
}

/// Joins the two fans and the two offsets of `centre` into the P and N polylines.
fn build(
    fa: &Fan,
    fb: &Fan,
    centre: &[Point],
    hs: Coord,
    cost: u32,
    layer: LayerId,
) -> Option<Candidate> {
    let off_p = offset_polyline(centre, hs, fa.p_left)?;
    let off_n = offset_polyline(centre, hs, !fa.p_left)?;
    let join = |head: &[Point], off: &[Point], tail: &[Point]| -> Option<Vec<Point>> {
        // The fans end exactly where the offsets begin / end.
        if head.last() != off.first() || tail.last() != off.last() {
            return None;
        }
        let mut v = head.to_vec();
        v.extend_from_slice(&off[1..]);
        v.extend(tail.iter().rev().skip(1));
        Some(simplify(&v))
    };
    Some(Candidate { cost, layer, p: join(&fa.p, &off_p, &fb.p)?, n: join(&fa.n, &off_n, &fb.n)? })
}

fn to_tracks(
    pts: &[Point],
    width: Coord,
    layer: LayerId,
    net: NetId,
) -> impl Iterator<Item = Track> + '_ {
    pts.windows(2).map(move |s| Track {
        start: s[0],
        end: s[1],
        width,
        layer,
        net,
        locked: false,
        origin: Origin::Routed,
    })
}

/// Quality-defect count of `board` (acid traps, slivers, right angles, avoidable pad bends).
fn defects(board: &Board) -> usize {
    quality::acute_angles(board).len()
        + quality::slivers(board).len()
        + quality::right_angles(board).len()
        + quality::near_pad_bends(board).len()
}

/// Routes the pairs of `board` that can be coupled. `layers` are the routable layers,
/// `pitch` the router's lattice pitch, `only` restricts to pairs whose two nets are listed.
/// Earlier pairs are fixed copper for later ones.
#[must_use]
pub fn route_pairs(
    board: &Board,
    layers: &[LayerId],
    pitch: Coord,
    only: Option<&[NetId]>,
) -> PairStage {
    let mut stage = PairStage::default();
    let mut work = board.clone();
    for (k, pair) in board.diff_pairs.iter().enumerate() {
        if only.is_some_and(|v| !(v.contains(&pair.p) && v.contains(&pair.n))) {
            continue;
        }
        let islands = crate::pad_islands(&work);
        if islands[pair.p.idx()].len() < 2 && islands[pair.n.idx()].len() < 2 {
            continue; // already connected (e.g. by the designer's copper)
        }
        match route_pair(&work, &islands, pair, layers, pitch) {
            Ok(tracks) => {
                work.tracks
                    .extend(tracks.iter().map(|t| Track { origin: Origin::Existing, ..t.clone() }));
                stage.coupled.push(CoupledPair { pair: k, tracks });
            }
            Err(why) => stage.uncoupled.push((k, why)),
        }
    }
    stage
}

fn route_pair(
    work: &Board,
    islands: &[Vec<Vec<PadId>>],
    pair: &DiffPair,
    layers: &[LayerId],
    pitch: Coord,
) -> Result<Vec<Track>, String> {
    let ends = pad_ends(work, islands, pair)?;
    let w = pair_width(work, pair);
    let clearance = work
        .rules
        .class_of(work, pair.p)
        .clearance
        .max(work.rules.class_of(work, pair.n).clearance);
    let gap = pair.gap.max(clearance);
    let spacing = (gap + w + 1) / 2 * 2;
    let on: Vec<LayerId> = layers
        .iter()
        .copied()
        .filter(|&l| {
            [ends[0].p, ends[0].n, ends[1].p, ends[1].n]
                .iter()
                .all(|&p| work.pad(p).layers.contains(l))
        })
        .collect();
    if on.is_empty() {
        return Err("the pads share no routable copper layer".into());
    }
    let obs = Obstacles::from_board(work);
    let mut cands = candidates(work, &obs, pair, ends, &on, pitch, w, spacing)?;
    if cands.is_empty() {
        return Err("no legal coupled route".into());
    }
    cands.sort_by_key(|c| c.cost);
    let base = defects(work);
    let mut fallback: Option<Vec<Track>> = None;
    for c in cands.iter().take(MAX_VERIFIED) {
        let tracks: Vec<Track> = to_tracks(&c.p, w, c.layer, pair.p)
            .chain(to_tracks(&c.n, w, c.layer, pair.n))
            .collect();
        let mut after = work.clone();
        after.tracks.extend(tracks.iter().cloned());
        let rep = ferroute_drc::check(
            &after,
            &DrcConfig { courtyards: false, connectivity: false, ..DrcConfig::default() },
        );
        if rep.introduced().next().is_some() || defects(&after) > base {
            continue;
        }
        let meets = pair.max_uncoupled.is_none_or(|max| {
            ferroute_drc::pairs::pair_metrics(&after, pair).is_some_and(|m| m.uncoupled() <= max)
        });
        if meets {
            return Ok(tracks);
        }
        fallback.get_or_insert(tracks);
    }
    fallback.ok_or_else(|| "no coupled route passes the exact DRC and quality checks".into())
}

#[cfg(test)]
mod tests;
