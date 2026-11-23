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
}
