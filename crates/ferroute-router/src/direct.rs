// SPDX-License-Identifier: AGPL-3.0-only
//! Direct pad-to-pad links: the straight track a professional draws between two nearby
//! pads of the same net that face each other along a shared entry axis.
//!
//! A link is created only when the centre-to-centre line is 0°/45°/90°, leaves both pads
//! along one of their entry axes (see [`crate::access::entry_dirs`]), is at most
//! [`MAX_LINK`] long and is exactly legal against every fixed obstacle. Links are chosen
//! greedily, shortest first (deterministic), and only between pads that are not yet
//! connected. They become fixed copper of their net before lattice routing.

use crate::access::entry_dirs;
use crate::legality::{ClassGeom, Obstacles, allows};
use ferroute_board::{Board, LayerId, NetId, Origin, PadId, Track};
use ferroute_drc::connectivity::UnionFind;
use ferroute_geom::{Coord, Dir8, Hull};

/// Longest direct link (adjacent pins at 0.1", passives, small connectors).
pub const MAX_LINK: Coord = 2_540_001;

/// A chosen link and the entry directions it occupies at both pads.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Link {
    pub track: Track,
    pub ends: [(PadId, Dir8); 2],
}

/// Computes direct links on the routable `layers` for every net.
#[must_use]
pub fn direct_links(
    board: &Board,
    obs: &Obstacles,
    geoms: &[ClassGeom],
    layers: &[LayerId],
) -> Vec<Link> {
    let pads_by_net = board.pads_by_net();
    // Pads already joined through a stored copper pour need no link.
    let joined = if board.zones.is_empty() { Vec::new() } else { crate::pad_islands(board) };
    let mut out = Vec::new();
    for (n, pads) in pads_by_net.iter().enumerate().skip(1) {
        if pads.len() < 2 {
            continue;
        }
        let net = NetId(u32::try_from(n).unwrap_or(u32::MAX));
        let cg = &geoms[usize::from(board.nets[n].class).min(geoms.len() - 1)];
        let mut cands: Vec<(u128, usize, usize, LayerId, Dir8)> = Vec::new();
        for (ia, &a) in pads.iter().enumerate() {
            for (ib, &b) in pads.iter().enumerate().skip(ia + 1) {
                let (ca, cb) = (board.pad_position(a), board.pad_position(b));
                let v = cb - ca;
                let Some(d) = Dir8::from_vector(v) else { continue };
                if v.norm2() > u128::from(MAX_LINK.unsigned_abs()).pow(2) {
                    continue;
                }
                let (da, db) = (entry_dirs(board, a), entry_dirs(board, b));
                let (dd, od) = (d.delta(), d.opposite().delta());
                let has = |dirs: &[(i64, i64)], x: (i32, i32)| {
                    dirs.contains(&(i64::from(x.0), i64::from(x.1)))
                };
                if !has(&da, dd) || !has(&db, od) {
                    continue;
                }
                let common = board.pad(a).layers.0 & board.pad(b).layers.0;
                let Some(&layer) = layers.iter().find(|l| common & (1 << l.0) != 0) else {
                    continue;
                };
                let hull = Hull::capsule(ca, cb, cg.track_radius);
                if allows(obs.classify_track(&hull, layer, cg), net) {
                    cands.push((v.norm2(), ia, ib, layer, d));
                }
            }
        }
        cands.sort_unstable_by_key(|c| (c.0, c.1, c.2));
        let mut uf = UnionFind::new(pads.len());
        for island in joined.get(n).into_iter().flatten() {
            let idx = |p: &PadId| pads.iter().position(|q| q == p);
            let mut members = island.iter().filter_map(idx);
            if let Some(first) = members.next() {
                for m in members {
                    uf.union(u32::try_from(first).unwrap_or(0), u32::try_from(m).unwrap_or(0));
                }
            }
        }
        for (_, ia, ib, layer, d) in cands {
            let (a, b) = (pads[ia], pads[ib]);
            // Links of different nets must never conflict with each other (each was only
            // checked against fixed obstacles): conservative per-net clearance bound.
            let hull = Hull::capsule(board.pad_position(a), board.pad_position(b), cg.track_radius);
            let conflicts = out.iter().any(|o: &Link| {
                o.track.net != net
                    && o.track.layer == layer
                    && hull.clearance_violated(
                        &o.track.hull(),
                        board
                            .rules
                            .max_clearance_for_net(board, net)
                            .max(board.rules.max_clearance_for_net(board, o.track.net)),
                    )
            });
            if conflicts
                || !uf.union(u32::try_from(ia).unwrap_or(0), u32::try_from(ib).unwrap_or(0))
            {
                continue;
            }
            out.push(Link {
                track: Track {
                    start: board.pad_position(a),
                    end: board.pad_position(b),
                    width: cg.track_width,
                    layer,
                    net,
                    locked: false,
                    origin: Origin::Routed,
                },
                ends: [(a, d), (b, d.opposite())],
            });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plan::class_geoms;
    use ferroute_board::builder::{BoardBuilder, mm, pt10};

    fn links_for(board: &Board) -> Vec<Link> {
        let obs = Obstacles::from_board(board);
        direct_links(board, &obs, &class_geoms(&board.rules), &[LayerId(0), LayerId(1)])
    }

    /// 2×2 block of round through-hole pads: GND on one diagonal, VCC on the other. Both
    /// diagonal links are individually legal but they cross.
    fn crossing_board() -> Board {
        use ferroute_board::builder::PadSpec;
        use ferroute_board::{PadShape, Side};
        let mut b = BoardBuilder::new(mm(20), mm(20), 2);
        let (g, v) = (b.net("GND"), b.net("VCC"));
        let pad = |num: &str, x: i64, y: i64, net| PadSpec {
            number: num.into(),
            local_pos: ferroute_geom::Point::new(x, y),
            shape: PadShape::Circle { diameter: 600_000 },
            drill: Some(300_000),
            net,
        };
        let pads = [
            pad("1", 0, 0, g),
            pad("2", 1_000_000, 1_000_000, g),
            pad("3", 1_000_000, 0, v),
            pad("4", 0, 1_000_000, v),
        ];
        b.footprint("U1", pt10(100, 100), 0.0, Side::Front, &pads);
        let mut board = b.build();
        board.rules.classes[0].clearance = 100_000;
        board.rules.classes[0].track_width = 100_000;
        board
    }

    #[test]
    fn links_of_different_nets_never_conflict() {
        let board = crossing_board();
        let l = links_for(&board);
        assert!(!l.is_empty());
        for (i, x) in l.iter().enumerate() {
            for y in &l[i + 1..] {
                if x.track.net != y.track.net && x.track.layer == y.track.layer {
                    let cl = board.clearance_between(x.track.net, y.track.net);
                    assert!(!x.track.hull().clearance_violated(&y.track.hull(), cl), "{x:?} {y:?}");
                }
            }
        }
    }

    #[test]
    fn crossing_link_candidates_route_and_terminate_clean() {
        let board = crossing_board();
        let out = crate::route(&board, &crate::RouterConfig::default()).expect("route");
        assert_eq!(out.stats.drc_gate_rips, 0);
        let mut after = board;
        after.apply(&out.edit);
        let rep = ferroute_drc::check(&after, &ferroute_drc::DrcConfig::default());
        assert_eq!(rep.introduced().count(), 0, "{:?}", rep.violations);
    }

    #[test]
    fn links_both_pads_of_a_same_net_resistor() {
        let mut b = BoardBuilder::new(mm(30), mm(20), 2);
        let n = b.net("N");
        b.smd2("R1", pt10(150, 100), 0.0, n, n);
        let board = b.build();
        let l = links_for(&board);
        assert_eq!(l.len(), 1);
        assert_eq!((l[0].track.start, l[0].track.end), (pt10(142, 100), pt10(158, 100)));
        assert_eq!(l[0].ends[0].1, Dir8::E);
    }

    #[test]
    fn no_link_across_a_foreign_pad() {
        use ferroute_board::builder::PadSpec;
        use ferroute_board::{PadShape, Side};
        let mut b = BoardBuilder::new(mm(30), mm(20), 2);
        let n = b.net("N");
        let m = b.net("M");
        let pad = |num: &str, x: i64, net| PadSpec {
            number: num.into(),
            local_pos: ferroute_geom::Point::new(x, 0),
            shape: PadShape::Rect { w: 500_000, h: 800_000 },
            drill: None,
            net,
        };
        // N – M – N in a row, 1.2 mm pitch: the N pads face each other but M is between.
        b.footprint(
            "U1",
            pt10(150, 100),
            0.0,
            Side::Front,
            &[pad("1", -1_200_000, n), pad("2", 0, m), pad("3", 1_200_000, n)],
        );
        assert!(links_for(&b.build()).is_empty());
    }

    #[test]
    fn no_link_when_too_far_or_skewed() {
        let mut b = BoardBuilder::new(mm(40), mm(20), 2);
        let k = b.net("K");
        b.smd2("R3", pt10(100, 50), 0.0, NetId::NONE, k);
        b.smd2("R4", pt10(110, 62), 0.0, k, NetId::NONE); // skewed
        b.smd2("R5", pt10(300, 100), 0.0, NetId::NONE, k);
        b.smd2("R6", pt10(340, 100), 0.0, k, NetId::NONE); // 2.4 mm gap + pads: 2.4 < d? (4.0-1.6 = 2.4)
        let l = links_for(&b.build());
        // R5.2 (30.8) – R6.1 (33.2): 2.4 mm, facing E/W → this one *is* linkable.
        assert_eq!(l.len(), 1);
        assert_eq!(l[0].track.start.dist(l[0].track.end).round(), 2_400_000.0);
    }

    #[test]
    fn links_only_join_unconnected_pads_shortest_first() {
        let mut b = BoardBuilder::new(mm(40), mm(20), 2);
        let n = b.net("N");
        // Header pins 2.54 mm apart on one net: a chain of links, never a redundant one.
        b.header("J1", pt10(100, 50), 0.0, &[n, n, n]);
        let l = links_for(&b.build());
        assert_eq!(l.len(), 2);
        assert!(l.iter().all(|x| x.track.start.dist(x.track.end) <= 2_540_001.0));
    }
}
