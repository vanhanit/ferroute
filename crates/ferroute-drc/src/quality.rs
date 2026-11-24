// SPDX-License-Identifier: AGPL-3.0-only
//! Manufacturing-quality checks beyond electrical DRC.
//!
//! **Acid traps:** two copper segments of the same net meeting at an acute angle (< 90°)
//! form a narrow wedge where etchant pools and over-etches. A point is reported when two
//! track directions leaving it have a positive dot product. Junctions inside pad or via
//! copper are solid copper and are not reported.

use crate::{BucketGrid, ItemRef, copper_items};
use ferroute_board::{Board, LayerId, NetId};
use ferroute_geom::{Coord, Hull, Point, Rect, Segment, orient, segment_segment_dist_lt};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// An acute same-net copper junction.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AcidTrap {
    pub at: Point,
    pub layer: LayerId,
    pub net: NetId,
}

fn on_interior(s: Segment, p: Point) -> bool {
    p != s.a && p != s.b && orient(s.a, s.b, p) == 0 && s.bbox().contains(p)
}

/// A copper segment of a net on a layer; `arc` marks chords of a flattened arc.
#[derive(Clone, Copy)]
struct Piece {
    seg: Segment,
    arc: bool,
    width: Coord,
}

/// Copper of every (layer, net), tracks and flattened arcs, in deterministic order.
fn groups(board: &Board) -> BTreeMap<(u8, u32), Vec<Piece>> {
    let mut groups: BTreeMap<(u8, u32), Vec<Piece>> = BTreeMap::new();
    for t in &board.tracks {
        if t.start != t.end && t.net != NetId::NONE {
            groups.entry((t.layer.0, t.net.0)).or_default().push(Piece {
                seg: Segment::new(t.start, t.end),
                arc: false,
                width: t.width,
            });
        }
    }
    for a in &board.arcs {
        if a.net == NetId::NONE {
            continue;
        }
        let pts = a.geom().flatten(ferroute_board::ARC_MAX_SAG).0;
        let entry = groups.entry((a.layer.0, a.net.0)).or_default();
        entry.extend(pts.windows(2).filter(|w| w[0] != w[1]).map(|w| Piece {
            seg: Segment::new(w[0], w[1]),
            arc: true,
            width: a.width,
        }));
    }
    groups
}

/// Is `p` inside solid (pad or via) copper of `net` on `layer`?
fn solid(board: &Board, layer: LayerId, net: NetId, p: Point) -> bool {
    board.pad_ids().any(|id| {
        let pad = board.pad(id);
        pad.net == net
            && pad.layers.contains(layer)
            && board.pad_hull(id).clearance_violated(&Hull::disk(p, 0), 0)
    }) || board.vias.iter().any(|v| {
        v.net == net
            && v.layers.contains(layer)
            && v.hull().clearance_violated(&Hull::disk(p, 0), 0)
    })
}

/// Every junction point of a group with the directions of copper leaving it (both ways
/// for segments passing through), flagged `true` for arc chords.
/// A junction point, the directions of copper leaving it (flagged for arcs), the widest track there.
type Junction = (Point, Vec<(Point, bool)>, Coord);

fn junctions(pieces: &[Piece]) -> Vec<Junction> {
    let mut points: Vec<Point> = pieces.iter().flat_map(|c| [c.seg.a, c.seg.b]).collect();
    points.sort_unstable();
    points.dedup();
    points
        .into_iter()
        .map(|p| {
            let mut dirs = Vec::new();
            let mut width = 0;
            for c in pieces {
                let s = c.seg;
                if s.a == p || s.b == p || on_interior(s, p) {
                    width = width.max(c.width);
                }
                if s.a == p {
                    dirs.push((s.b - p, c.arc));
                } else if s.b == p {
                    dirs.push((s.a - p, c.arc));
                } else if on_interior(s, p) {
                    dirs.push((s.a - p, c.arc));
                    dirs.push((s.b - p, c.arc));
                }
            }
            (p, dirs, width)
        })
        .collect()
}

/// Reports every acute (< 90°) junction between same-net tracks on the same layer,
/// excluding junctions inside pad or via copper. Exact integer arithmetic.
#[must_use]
pub fn acute_angles(board: &Board) -> Vec<AcidTrap> {
    let mut out = Vec::new();
    for (&(layer, net), pieces) in &groups(board) {
        let (layer, net) = (LayerId(layer), NetId(net));
        for (p, dirs, _) in junctions(pieces) {
            let acute = dirs
                .iter()
                .enumerate()
                .any(|(i, u)| dirs[i + 1..].iter().any(|v| u.0.dot(v.0) > 0));
            if acute && !solid(board, layer, net, p) {
                out.push(AcidTrap { at: p, layer, net });
            }
        }
    }
    out
}

/// A bend close to a same-net pad that could have been avoided.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct NearPadBend {
    pub at: Point,
    pub layer: LayerId,
    pub net: NetId,
}

fn unit(v: Point) -> (i64, i64) {
    (v.x.signum(), v.y.signum())
}

/// Is the bend `prev → v → next` of a `net` track of `width` on `layer` an avoidable bend
/// next to a pad? It is when `v` lies within a same-net pad's hull inflated by the track
/// width plus clearance and the pad accepts leaving its centre directly along the
/// post-bend direction (`Board::pad_entry_dirs`). The pad-side leg is the one whose far
/// end is nearer the pad centre.
#[must_use]
pub fn avoidable_pad_bend(
    board: &Board,
    (net, layer, width): (NetId, LayerId, Coord),
    (prev, v, next): (Point, Point, Point),
) -> bool {
    let (d0, d1) = (prev - v, next - v);
    if d0.cross(d1) == 0 && d0.dot(d1) < 0 {
        return false; // straight through
    }
    let zone = width + board.clearance_between(net, net);
    board.pad_ids().any(|id| {
        let pad = board.pad(id);
        if pad.net != net || !pad.layers.contains(layer) {
            return false;
        }
        let hull = board.pad_hull(id);
        if !hull.clearance_violated(&Hull::disk(v, 0), zone) {
            return false;
        }
        let c = board.pad_position(id);
        let post = if prev.dist(c) <= next.dist(c) { next - v } else { prev - v };
        board.pad_entry_dirs(id).contains(&unit(post))
    })
}

/// Degree-2 same-net track bends (any angle) that [`avoidable_pad_bend`] flags.
#[must_use]
pub fn near_pad_bends(board: &Board) -> Vec<NearPadBend> {
    let mut out = Vec::new();
    for (&(layer, net), pieces) in &groups(board) {
        let (layer, net) = (LayerId(layer), NetId(net));
        for (p, dirs, width) in junctions(pieces) {
            if let [(u, false), (v, false)] = dirs[..]
                && avoidable_pad_bend(board, (net, layer, width), (p + u, p, p + v))
            {
                out.push(NearPadBend { at: p, layer, net });
            }
        }
    }
    out
}

/// A routed connection that wraps around its target pad's footprint instead of entering
/// the pad directly (see [`pad_wraps`]).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PadWrap {
    /// The pad the connection ends at.
    pub pad: ferroute_board::PadId,
    pub layer: LayerId,
    pub net: NetId,
    /// Where the wrap is detected: the turn or the pad it passes.
    pub at: Point,
}

/// Turn from direction `u` to `v` in whole 45° steps, signed (left positive), −4..=4.
/// Exact integer comparisons against tan 22.5° ≈ 5/12 and tan 67.5° ≈ 29/12.
fn turn_units(u: Point, v: Point) -> i32 {
    let (c, d) = (u.cross(v), u.dot(v));
    let (a, b) = (c.abs() * 12, d.abs());
    let mag = if d == 0 || a >= 29 * b {
        2
    } else if a < 5 * b {
        if d > 0 { 0 } else { 4 }
    } else if d > 0 {
        1
    } else {
        3
    };
    if c < 0 { -mag } else { mag }
}

/// Connections that wrap around their target pad instead of entering it directly. Starting
/// at every pad centre, the same-net chain of straight tracks is followed (through
/// degree-2 joints, stopping at junctions, arcs and vias) while it stays within
/// `2 × (pad size + clearance)` of the pad centre (the first vertex beyond that radius is
/// still examined). It is a wrap when
///
/// * it turns by a cumulative signed angle of at least 180° (it circles the pad), or
/// * it doubles back: after leaving the pad copper along its stub axis it returns across
///   the pad's edge line (the line through the exit point perpendicular to the stub), so
///   the connection reaches the pad from the far side of where it heads, passing beside
///   the pad's own footprint.
///
/// Exact integer arithmetic, deterministic order.
#[must_use]
pub fn pad_wraps(board: &Board) -> Vec<PadWrap> {
    let groups = groups(board);
    let mut out = Vec::new();
    for id in board.pad_ids() {
        let pad = board.pad(id);
        if pad.net == NetId::NONE {
            continue;
        }
        let c = board.pad_position(id);
        let radius = wrap_radius(board, id);
        for layer in pad.layers.iter() {
            let Some(pieces) = groups.get(&(layer.0, pad.net.0)) else { continue };
            let touching = |p: Point, skip: Option<usize>| -> Vec<usize> {
                (0..pieces.len())
                    .filter(|&i| Some(i) != skip && (pieces[i].seg.a == p || pieces[i].seg.b == p))
                    .collect()
            };
            for first in touching(c, None) {
                if pieces[first].arc {
                    continue;
                }
                // The vertices of the plain degree-2 chain leaving the pad, up to and
                // including the first one beyond `radius`.
                let mut pts = vec![c];
                let (mut cur, mut piece) = (c, first);
                loop {
                    let s = pieces[piece].seg;
                    let next = if s.a == cur { s.b } else { s.a };
                    pts.push(next);
                    if !within(next, c, radius) {
                        break;
                    }
                    // Continue through a plain degree-2 joint only.
                    if board.vias.iter().any(|v| v.net == pad.net && v.pos == next) {
                        break;
                    }
                    let cont = touching(next, Some(piece));
                    let [n] = cont[..] else { break };
                    if pieces[n].arc {
                        break;
                    }
                    (cur, piece) = (next, n);
                }
                if let Some(at) = wrap_hit(board, id, &pts) {
                    out.push(PadWrap { pad: id, layer, net: pad.net, at });
                }
            }
        }
    }
    out
}

/// Distance from a pad centre within which a connection is examined for wrapping.
fn wrap_radius(board: &Board, id: ferroute_board::PadId) -> Coord {
    let pad = board.pad(id);
    let bb = board.pad_hull(id).bbox();
    2 * (bb.width().max(bb.height()) + board.clearance_between(pad.net, pad.net))
}

/// Does the polyline `pts` (starting at the centre of pad `id`) wrap around the pad
/// (see [`pad_wraps`])? Returns where the wrap is detected. Vertices beyond the examined
/// radius are ignored, so `pts` may be longer than needed.
#[must_use]
pub fn wrap_hit(board: &Board, id: ferroute_board::PadId, pts: &[Point]) -> Option<Point> {
    let c = board.pad_position(id);
    let hull = board.pad_hull(id);
    let radius = wrap_radius(board, id);
    let inside = |q: Point| hull.clearance_violated(&Hull::disk(q, 0), 0);
    let u0 = *pts.get(1)? - c;
    // Axis coordinate of the point where the chain leaves the pad copper.
    let mut edge: Option<i128> = None;
    let (mut turn, mut prev_dir) = (0i32, None::<Point>);
    for w in pts.windows(2) {
        let (cur, next) = (w[0], w[1]);
        let dir = next - cur;
        if let Some(u) = prev_dir {
            turn += turn_units(u, dir);
            if turn.abs() >= 4 {
                return Some(cur);
            }
        }
        if let Some(e) = edge {
            if (next - c).dot(u0) < e {
                return Some(next);
            }
        } else if !inside(next) {
            // Where this leg leaves the pad copper (the pad is convex).
            let at = |t: i128| {
                let d = next - cur;
                cur + Point::new(
                    i64::try_from((i128::from(d.x) * t) >> 20).unwrap_or(0),
                    i64::try_from((i128::from(d.y) * t) >> 20).unwrap_or(0),
                )
            };
            let (mut lo, mut hi) = (0i128, 1i128 << 20);
            while hi - lo > 1 {
                let mid = (lo + hi) / 2;
                if inside(at(mid)) { lo = mid } else { hi = mid }
            }
            edge = Some((at(hi) - c).dot(u0));
        }
        if !within(next, c, radius) {
            break;
        }
        prev_dir = Some(dir);
    }
    None
}

/// Two same-net copper items that do not touch but are closer than the net clearance: a
/// narrow sliver of bare laminate between them (an acid trap).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Sliver {
    pub at: Point,
    pub layer: LayerId,
    pub net: NetId,
}

/// Do the same-net hulls `a` and `b` form a sliver for `clearance`? They must not touch
/// (gap of at least 1 nm) yet be closer than `clearance`.
#[must_use]
pub fn is_sliver(a: &Hull, b: &Hull, clearance: Coord) -> bool {
    if !a.clearance_violated(b, clearance) || a.clearance_violated(b, 1) {
        return false;
    }
    // A sliver is a narrow *channel*: two boundary pieces that run alongside each other
    // (not perpendicular, with overlapping extents) closer than the clearance. Pieces that
    // only meet end-to-end or at ≥ 90° (e.g. the legs of a chamfer, collinear tracks joined
    // through a pad) leave no channel.
    let reach = clearance + a.radius + b.radius;
    let (ea, eb) = (edges(a), edges(b));
    ea.iter().any(|&e| eb.iter().any(|&f| facing(e, f) && segment_segment_dist_lt(e, f, reach)))
}

/// Boundary pieces of a hull's core: the point of a disc, the segment of a capsule, the
/// edges of a polygon.
fn edges(h: &Hull) -> Vec<Segment> {
    match h.pts.len() {
        0 => Vec::new(),
        1 => vec![Segment::new(h.pts[0], h.pts[0])],
        2 => vec![Segment::new(h.pts[0], h.pts[1])],
        n => (0..n).map(|i| Segment::new(h.pts[i], h.pts[(i + 1) % n])).collect(),
    }
}

/// `true` if `e` and `f` run alongside each other: not perpendicular, and the projection of
/// one onto the other overlaps with positive length. A point (disc) faces a segment whose
/// interior its foot lands on. Exact integer arithmetic.
fn facing(e: Segment, f: Segment) -> bool {
    let (de, df) = (e.b - e.a, f.b - f.a);
    let zero = |d: Point| d.x == 0 && d.y == 0;
    let dot =
        |u: Point, v: Point| i128::from(u.x) * i128::from(v.x) + i128::from(u.y) * i128::from(v.y);
    // Projection of the segment `s` onto line `l`: interval of dot products along `dl`.
    let overlaps = |l: Segment, dl: Point, s: Segment| {
        let (t0, t1) = (dot(s.a - l.a, dl), dot(s.b - l.a, dl));
        let (lo, hi) = (t0.min(t1), t0.max(t1));
        let len = dot(dl, dl);
        if s.a == s.b { lo > 0 && lo < len } else { lo < len && hi > 0 }
    };
    match (zero(de), zero(df)) {
        (true, true) => false,
        (false, true) => overlaps(e, de, f),
        (true, false) => overlaps(f, df, e),
        (false, false) => dot(de, df) != 0 && overlaps(e, de, f) && overlaps(f, df, e),
    }
}

const fn is_copper(i: ItemRef) -> bool {
    matches!(i, ItemRef::Pad(_) | ItemRef::Track(_) | ItemRef::Arc(_) | ItemRef::Via(_))
}

/// Same-net copper pairs on a layer closer than the net's clearance without touching.
/// Pad/pad pairs are library geometry and are ignored. Deterministic order.
#[must_use]
pub fn slivers(board: &Board) -> Vec<Sliver> {
    let items = copper_items(board);
    let reach = board.rules.max_clearance();
    let boxes: Vec<Rect> = items.iter().map(|i| i.bbox).collect();
    let grid = BucketGrid::new(&boxes, 4_000_000);
    let mut out = Vec::new();
    for (i, a) in items.iter().enumerate() {
        if a.net == NetId::NONE || !is_copper(a.item) {
            continue;
        }
        let clearance = board.clearance_between(a.net, a.net);
        let mut near = grid.query(a.bbox.inflate(reach.max(clearance) + 1), &boxes);
        near.sort_unstable();
        for j in near {
            let j = j as usize;
            let b = &items[j];
            if j <= i
                || b.net != a.net
                || !is_copper(b.item)
                || matches!((a.item, b.item), (ItemRef::Pad(_), ItemRef::Pad(_)))
            {
                continue;
            }
            if let Some(layer) = a
                .layers
                .iter()
                .find(|&l| b.layers.contains(l) && is_sliver(&a.hull, &b.hull, clearance))
            {
                out.push(Sliver { at: a.bbox.min, layer, net: a.net });
            }
        }
    }
    out
}

/// A 90° bend between two straight same-net tracks.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RightAngle {
    pub at: Point,
    pub layer: LayerId,
    pub net: NetId,
}

/// Same-net straight tracks meeting at exactly 90° at a point where exactly two tracks of
/// that net/layer meet (a bend, not a T/X junction), outside pad and via copper. Arcs
/// never count. Exact integer arithmetic, deterministic order.
#[must_use]
pub fn right_angles(board: &Board) -> Vec<RightAngle> {
    let mut out = Vec::new();
    for (&(layer, net), pieces) in &groups(board) {
        let (layer, net) = (LayerId(layer), NetId(net));
        for (p, dirs, _) in junctions(pieces) {
            if let [(u, false), (v, false)] = dirs[..]
                && u.dot(v) == 0
                && u.cross(v) != 0
                && !solid(board, layer, net, p)
            {
                out.push(RightAngle { at: p, layer, net });
            }
        }
    }
    out
}

/// A via that sits where its traces jog: a bend right next to it, or a layer change whose
/// two traces do not continue in one straight line.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ViaJog {
    pub at: Point,
    pub net: NetId,
}

/// Bends within this distance of a via centre count as "at the via": the via diameter plus
/// two track widths.
#[must_use]
pub const fn via_jog_reach(via_diameter: Coord, track_width: Coord) -> Coord {
    via_diameter + 2 * track_width
}

fn within(a: Point, b: Point, reach: Coord) -> bool {
    let r = u128::try_from(reach.max(0)).unwrap_or(0);
    (a - b).norm2() <= r * r
}

/// Do the directions `u` and `v` leaving a point continue one straight line?
fn straight_through(u: Point, v: Point) -> bool {
    u.cross(v) == 0 && u.dot(v) < 0
}

/// One via trace for [`polylines_jog`]: its polyline ends at the via centre.
#[derive(Clone, Copy, Debug)]
pub struct ViaTrace<'a> {
    pub layer: LayerId,
    pub width: Coord,
    pub pts: &'a [Point],
}

/// How badly a via at `at` jogs, given the polylines of its same-net traces (each ending at
/// `at`): one per bend within [`via_jog_reach`] plus two for a layer change whose traces
/// do not continue in one straight line. Zero means no jog. Shared by [`via_jogs`] and the
/// router's via relocation.
#[must_use]
pub fn polylines_jog_weight(at: Point, via_diameter: Coord, traces: &[ViaTrace<'_>]) -> u32 {
    let mut weight = 0;
    for t in traces {
        let reach = via_jog_reach(via_diameter, t.width);
        weight += t
            .pts
            .windows(3)
            .filter(|w| within(w[1], at, reach) && !straight_through(w[0] - w[1], w[2] - w[1]))
            .count() as u32;
    }
    if let [a, b] = traces
        && a.layer != b.layer
        && a.pts.len() >= 2
        && b.pts.len() >= 2
        && !straight_through(a.pts[a.pts.len() - 2] - at, b.pts[b.pts.len() - 2] - at)
    {
        weight += 2;
    }
    weight
}

/// Does a via at `at` jog? See [`polylines_jog_weight`].
#[must_use]
pub fn polylines_jog(at: Point, via_diameter: Coord, traces: &[ViaTrace<'_>]) -> bool {
    polylines_jog_weight(at, via_diameter, traces) > 0
}

/// Every via at which same-net traces jog. A via counts when (a) a same-net bend (a point
/// where exactly two tracks meet and turn) lies within [`via_jog_reach`] of its centre on
/// any layer, or (b) it is a pure layer change (exactly one same-net track on each of two
/// layers ends there) whose tracks are not collinear. Exact integer arithmetic,
/// deterministic order (board via order).
#[must_use]
pub fn via_jogs(board: &Board) -> Vec<ViaJog> {
    let groups = groups(board);
    let mut bends: BTreeMap<u32, Vec<(Point, Coord)>> = BTreeMap::new();
    for (&(_, net), pieces) in &groups {
        for (p, dirs, width) in junctions(pieces) {
            if let [(u, false), (v, false)] = dirs[..]
                && !straight_through(u, v)
            {
                bends.entry(net).or_default().push((p, width));
            }
        }
    }
    let mut out = Vec::new();
    for via in &board.vias {
        if via.net == NetId::NONE {
            continue;
        }
        let near_bend = via.layers.iter().any(|l| {
            groups.contains_key(&(l.0, via.net.0))
                && bends.get(&via.net.0).is_some_and(|bs| {
                    bs.iter().any(|&(p, w)| within(p, via.pos, via_jog_reach(via.diameter, w)))
                })
        });
        // Tracks of the net ending exactly at the via, per layer.
        let mut ends: Vec<(LayerId, Point)> = Vec::new();
        for t in &board.tracks {
            if t.net == via.net && via.layers.contains(t.layer) && t.start != t.end {
                if t.start == via.pos {
                    ends.push((t.layer, t.end - via.pos));
                } else if t.end == via.pos {
                    ends.push((t.layer, t.start - via.pos));
                }
            }
        }
        let bent_change = matches!(
            ends[..],
            [(la, u), (lb, v)] if la != lb && !straight_through(u, v)
        );
        if near_bend || bent_change {
            out.push(ViaJog { at: via.pos, net: via.net });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use ferroute_board::Origin;
    use ferroute_board::builder::{BoardBuilder, mm, pt10};

    /// A track as ((x0, y0), (x1, y1)) in tenths of a millimetre.
    type Seg = ((i64, i64), (i64, i64));

    fn board_with(tracks: &[Seg]) -> ferroute_board::Board {
        let mut b = BoardBuilder::new(mm(30), mm(20), 2);
        let n = b.net("N");
        for &(a, c) in tracks {
            b.track(pt10(a.0, a.1), pt10(c.0, c.1), 250_000, 0, n, Origin::Routed);
        }
        b.build()
    }

    #[test]
    fn right_angle_tee_is_fine() {
        let b = board_with(&[
            ((50, 100), (150, 100)),
            ((150, 100), (250, 100)),
            ((150, 100), (150, 50)),
        ]);
        assert!(acute_angles(&b).is_empty());
    }

    /// A header at (10, 10) mm: square pin 1, round pin 2 at (10, 12.54); both on one net.
    fn header_board(tracks: &[Seg]) -> ferroute_board::Board {
        let mut bb = BoardBuilder::new(mm(30), mm(20), 2);
        let n = bb.net("N");
        bb.header("J1", pt10(100, 100), 0.0, &[n, n]);
        for &(a, c) in tracks {
            bb.track(pt10(a.0, a.1), pt10(c.0, c.1), 250_000, 0, n, Origin::Routed);
        }
        bb.build()
    }

    #[test]
    fn bend_at_round_pad_edge_is_flagged() {
        // Leaves the round pad horizontally, bends 45° right at its edge.
        let b = header_board(&[((100, 125), (110, 125)), ((110, 125), (150, 165))]);
        let nb = near_pad_bends(&b);
        assert_eq!(nb.len(), 1);
        assert_eq!(nb[0].at, pt10(110, 125));
    }

    #[test]
    fn bend_forced_by_rectangular_pad_axes_is_fine() {
        // Square pin 1 only leaves along its axes; the diagonal after the bend is forced.
        let b = header_board(&[((100, 100), (110, 100)), ((110, 100), (150, 140))]);
        assert!(near_pad_bends(&b).is_empty());
    }

    #[test]
    fn bend_far_from_pads_is_fine() {
        let b = header_board(&[((200, 50), (250, 50)), ((250, 50), (280, 80))]);
        assert!(near_pad_bends(&b).is_empty());
    }

    #[test]
    fn breakout_running_parallel_to_its_pad_is_a_sliver() {
        // R1 pad 1 sits at (14.2, 10.0); the track leaves it up, then bends 90° left and
        // runs along the pad's top edge at a small gap.
        let mut bb = BoardBuilder::new(mm(30), mm(20), 2);
        let n = bb.net("N");
        bb.smd2("R1", pt10(150, 100), 0.0, n, n);
        bb.track(pt10(142, 100), pt10(142, 93), 250_000, 0, n, Origin::Routed);
        bb.track(pt10(142, 93), pt10(100, 93), 250_000, 0, n, Origin::Routed);
        assert!(!slivers(&bb.build()).is_empty());
        let mut far = BoardBuilder::new(mm(30), mm(20), 2);
        let n = far.net("N");
        far.smd2("R1", pt10(150, 100), 0.0, n, n);
        far.track(pt10(142, 100), pt10(142, 60), 250_000, 0, n, Origin::Routed);
        far.track(pt10(142, 60), pt10(100, 60), 250_000, 0, n, Origin::Routed);
        assert!(slivers(&far.build()).is_empty());
    }

    #[test]
    fn forty_five_bend_and_spaced_legs_are_no_slivers() {
        let b = board_with(&[((50, 100), (100, 100)), ((100, 100), (150, 50))]);
        assert!(slivers(&b).is_empty());
        let m = board_with(&[
            ((50, 100), (150, 100)),
            ((150, 100), (150, 110)),
            ((150, 110), (50, 110)),
        ]);
        assert!(slivers(&m).is_empty());
    }

    #[test]
    fn u_turn_with_a_gap_below_clearance_is_a_sliver() {
        let b = board_with(&[
            ((50, 100), (150, 100)),
            ((150, 100), (150, 104)),
            ((150, 104), (50, 104)),
        ]);
        assert!(!slivers(&b).is_empty());
    }

    #[test]
    fn right_angle_l_bend_is_reported() {
        let b = board_with(&[((50, 100), (150, 100)), ((150, 100), (150, 50))]);
        let r = right_angles(&b);
        assert_eq!(r.len(), 1);
        assert_eq!(r[0].at, pt10(150, 100));
    }

    #[test]
    fn right_angle_ignores_tee_forty_five_and_pads() {
        let tee = board_with(&[
            ((50, 100), (150, 100)),
            ((150, 100), (250, 100)),
            ((150, 100), (150, 50)),
        ]);
        assert!(right_angles(&tee).is_empty());
        let d45 = board_with(&[((50, 100), (100, 100)), ((100, 100), (150, 50))]);
        assert!(right_angles(&d45).is_empty());
        let mut bb = BoardBuilder::new(mm(30), mm(20), 2);
        let n = bb.net("N");
        bb.smd2("R1", pt10(150, 100), 0.0, n, n);
        bb.track(pt10(142, 100), pt10(100, 100), 250_000, 0, n, Origin::Routed);
        bb.track(pt10(142, 100), pt10(142, 58), 250_000, 0, n, Origin::Routed);
        assert!(right_angles(&bb.build()).is_empty());
    }

    #[test]
    fn right_angle_ignores_arc_joints() {
        let mut b = board_with(&[((50, 100), (150, 100))]);
        let n = b.tracks[0].net;
        b.arcs.push(ferroute_board::ArcTrack {
            start: pt10(150, 100),
            mid: pt10(170, 105),
            end: pt10(190, 125),
            width: 250_000,
            layer: LayerId(0),
            net: n,
            locked: false,
            origin: Origin::Routed,
        });
        assert!(right_angles(&b).is_empty());
    }

    #[test]
    fn straight_and_obtuse_bends_are_fine() {
        let b = board_with(&[((50, 100), (100, 100)), ((100, 100), (150, 50))]);
        assert!(acute_angles(&b).is_empty());
    }

    #[test]
    fn forty_five_degree_branch_is_an_acid_trap() {
        let b = board_with(&[
            ((50, 100), (150, 100)),
            ((150, 100), (250, 100)),
            ((150, 100), (200, 50)),
        ]);
        let traps = acute_angles(&b);
        assert_eq!(traps.len(), 1);
        assert_eq!(traps[0].at, pt10(150, 100));
    }

    #[test]
    fn branch_ending_mid_segment_is_detected() {
        // The trunk is one segment; the branch ends on its interior at 45°.
        let b = board_with(&[((50, 100), (250, 100)), ((150, 100), (200, 50))]);
        assert_eq!(acute_angles(&b).len(), 1);
    }

    #[test]
    fn junction_inside_pad_copper_is_ignored() {
        let mut bb = BoardBuilder::new(mm(30), mm(20), 2);
        let n = bb.net("N");
        bb.smd2("R1", pt10(150, 100), 0.0, n, n);
        // Two tracks meeting at 45° at the centre of R1's first pad (14.2, 10.0).
        bb.track(pt10(142, 100), pt10(100, 100), 250_000, 0, n, Origin::Routed);
        bb.track(pt10(142, 100), pt10(100, 58), 250_000, 0, n, Origin::Routed);
        assert!(acute_angles(&bb.build()).is_empty());
    }

    #[test]
    fn different_nets_or_layers_never_form_traps() {
        let mut bb = BoardBuilder::new(mm(30), mm(20), 2);
        let a = bb.net("A");
        let c = bb.net("C");
        bb.track(pt10(50, 100), pt10(150, 100), 250_000, 0, a, Origin::Routed);
        bb.track(pt10(150, 100), pt10(200, 50), 250_000, 0, c, Origin::Routed);
        bb.track(pt10(150, 100), pt10(200, 150), 250_000, 1, a, Origin::Routed);
        assert!(acute_angles(&bb.build()).is_empty());
    }

    fn via_board(tracks: &[(u8, Seg)], via: (i64, i64)) -> ferroute_board::Board {
        let mut b = BoardBuilder::new(mm(30), mm(20), 2);
        let n = b.net("N");
        for &(l, (a, c)) in tracks {
            b.track(pt10(a.0, a.1), pt10(c.0, c.1), 250_000, l, n, Origin::Routed);
        }
        b.via(pt10(via.0, via.1), 600_000, 300_000, n, Origin::Routed);
        b.build()
    }

    #[test]
    fn straight_layer_change_is_not_a_jog() {
        let b =
            via_board(&[(0, ((50, 100), (150, 100))), (1, ((150, 100), (250, 100)))], (150, 100));
        assert!(via_jogs(&b).is_empty());
        // Perpendicular continuation on the other layer is a pure layer change that bends.
        let b =
            via_board(&[(0, ((50, 100), (150, 100))), (1, ((150, 100), (150, 160)))], (150, 100));
        assert_eq!(via_jogs(&b).len(), 1);
        // So is a 45° turn across the via.
        let b =
            via_board(&[(0, ((50, 100), (150, 100))), (1, ((150, 100), (210, 160)))], (150, 100));
        assert_eq!(via_jogs(&b).len(), 1);
    }

    #[test]
    fn bend_next_to_the_via_is_a_jog_but_a_far_one_is_not() {
        // Bend 0.6 mm before the via (reach = 0.6 + 2 * 0.25 = 1.1 mm).
        let near = via_board(
            &[
                (0, ((50, 100), (90, 100))),
                (0, ((90, 100), (96, 106))),
                (0, ((96, 106), (150, 106))),
                (1, ((150, 106), (250, 106))),
            ],
            (150, 106),
        );
        assert!(via_jogs(&near).is_empty(), "bend 5.4 mm away");
        let near = via_board(
            &[
                (0, ((50, 100), (144, 100))),
                (0, ((144, 100), (150, 106))),
                (1, ((150, 106), (250, 106))),
            ],
            (150, 106),
        );
        assert_eq!(via_jogs(&near).len(), 1, "bend 0.85 mm away");
    }

    #[test]
    fn t_junction_via_is_not_a_pure_layer_change() {
        let b = via_board(
            &[
                (0, ((50, 100), (150, 100))),
                (1, ((150, 100), (250, 100))),
                (1, ((150, 100), (150, 160))),
                (1, ((150, 100), (150, 40))),
            ],
            (150, 100),
        );
        assert!(via_jogs(&b).is_empty());
    }

    /// 0603 part at (10, 10) whose pad 1 (9.2, 10) carries the net, with `chain` (tenths of
    /// a millimetre, from the pad centre outward) routed on the front layer.
    fn pad_board(chain: &[(i64, i64)]) -> ferroute_board::Board {
        use ferroute_board::NetId;
        let mut b = BoardBuilder::new(mm(30), mm(20), 2);
        let n = b.net("N");
        b.smd2("R1", pt10(100, 100), 0.0, n, NetId::NONE);
        for w in chain.windows(2) {
            b.track(pt10(w[0].0, w[0].1), pt10(w[1].0, w[1].1), 150_000, 0, n, Origin::Routed);
        }
        b.build()
    }

    #[test]
    fn direct_pad_entries_are_not_wraps() {
        // Straight down, then one 45° break-out, then straight away.
        let b = pad_board(&[(92, 100), (92, 106), (100, 114), (140, 114)]);
        assert!(pad_wraps(&b).is_empty());
        // Straight out sideways (away from the part).
        let b = pad_board(&[(92, 100), (60, 100)]);
        assert!(pad_wraps(&b).is_empty());
    }

    #[test]
    fn dip_that_returns_across_the_pad_edge_is_a_wrap() {
        // Down, 45°, under the other pad, 45° back up above the pad's lower edge line.
        let b = pad_board(&[(92, 100), (92, 106), (96, 110), (106, 110), (112, 104), (140, 104)]);
        let w = pad_wraps(&b);
        assert_eq!(w.len(), 1, "{w:?}");
        assert_eq!(w[0].at, pt10(112, 104));
        // The same dip whose return lies beyond the examined radius is not reported.
        let far = pad_board(&[(92, 100), (92, 106), (96, 110), (140, 110), (146, 104), (170, 104)]);
        assert!(pad_wraps(&far).is_empty());
    }

    #[test]
    fn circling_the_pad_is_a_wrap() {
        let b = pad_board(&[(92, 100), (92, 106), (86, 112), (80, 112), (74, 106), (74, 100)]);
        assert_eq!(pad_wraps(&b).len(), 1);
    }

    #[test]
    fn turn_units_are_signed_octants() {
        assert_eq!(turn_units(Point::new(1, 0), Point::new(1, 1)), 1);
        assert_eq!(turn_units(Point::new(1, 0), Point::new(0, 1)), 2);
        assert_eq!(turn_units(Point::new(1, 0), Point::new(-1, 1)), 3);
        assert_eq!(turn_units(Point::new(1, 0), Point::new(-1, 0)), 4);
        assert_eq!(turn_units(Point::new(1, 0), Point::new(1, -1)), -1);
        assert_eq!(turn_units(Point::new(0, 1), Point::new(0, 1)), 0);
    }
}
