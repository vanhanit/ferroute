// SPDX-License-Identifier: AGPL-3.0-only
//! Manufacturing-quality checks beyond electrical DRC.
//!
//! **Acid traps:** two copper segments of the same net meeting at an acute angle (< 90°)
//! form a narrow wedge where etchant pools and over-etches. A point is reported when two
//! track directions leaving it have a positive dot product. Junctions inside pad or via
//! copper are solid copper and are not reported.

use ferroute_board::{Board, LayerId, NetId};
use ferroute_geom::{Hull, Point, Segment, orient};
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

/// Reports every acute (< 90°) junction between same-net tracks on the same layer,
/// excluding junctions inside pad or via copper. Exact integer arithmetic.
#[must_use]
pub fn acute_angles(board: &Board) -> Vec<AcidTrap> {
    // Solid copper (pads, vias) per (layer, net): junctions inside it are harmless.
    let solid = |layer: LayerId, net: NetId, p: Point| {
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
    };
    let mut groups: BTreeMap<(u8, u32), Vec<Segment>> = BTreeMap::new();
    for t in &board.tracks {
        if t.start != t.end && t.net != NetId::NONE {
            groups.entry((t.layer.0, t.net.0)).or_default().push(Segment::new(t.start, t.end));
        }
    }
    let mut out = Vec::new();
    for (&(layer, net), segs) in &groups {
        let (layer, net) = (LayerId(layer), NetId(net));
        let mut points: Vec<Point> = segs.iter().flat_map(|s| [s.a, s.b]).collect();
        points.sort_unstable();
        points.dedup();
        for p in points {
            // Directions of copper leaving p (both ways for segments passing through p).
            let mut dirs: Vec<Point> = Vec::new();
            for s in segs {
                if s.a == p {
                    dirs.push(s.b - p);
                } else if s.b == p {
                    dirs.push(s.a - p);
                } else if on_interior(*s, p) {
                    dirs.push(s.a - p);
                    dirs.push(s.b - p);
                }
            }
            let acute =
                dirs.iter().enumerate().any(|(i, u)| dirs[i + 1..].iter().any(|v| u.dot(*v) > 0));
            if acute && !solid(layer, net, p) {
                out.push(AcidTrap { at: p, layer, net });
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
