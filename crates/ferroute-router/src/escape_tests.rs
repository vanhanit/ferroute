// SPDX-License-Identifier: AGPL-3.0-only
//! Tests of the area-array escape stage ([`crate::escape`]).

use crate::escape::{detect_arrays, plan};
use crate::grid::Grid;
use crate::legality::Obstacles;
use crate::{Accel, RouterConfig, route_once};
use ferroute_board::builder::{BoardBuilder, PadSpec, mm};
use ferroute_board::{Board, LayerId, Mobility, NetId, PadShape, Side};
use ferroute_drc::{DrcConfig, check};
use ferroute_geom::{Point, Rect};

/// 6 × 6 BGA (1 mm pitch, 0.5 mm balls), 0.1 mm rules, two layers: the 20 outer balls and
/// the 12 balls of the second ring each go to an 0603 part around the package.
fn bga6() -> Board {
    let mut b = BoardBuilder::new(mm(50), mm(50), 2);
    {
        let r = b.rules_mut();
        r.min_track_width = 100_000;
        r.min_via_diameter = 400_000;
        r.min_via_drill = 200_000;
        r.hole_clearance = 200_000;
        r.hole_to_hole = 250_000;
        let c = &mut r.classes[0];
        c.clearance = 100_000;
        c.track_width = 100_000;
        c.via_diameter = 450_000;
        c.via_drill = 200_000;
    }
    let centre = Point::new(mm(25), mm(25));
    let n = 6i64;
    let off = |i: i64| (2 * i - (n - 1)) * 500_000;
    let mut pads = Vec::new();
    // (ring, side, lateral coordinate, net)
    let mut parts: Vec<(i64, usize, i64, NetId)> = Vec::new();
    let mut k = 0;
    for r in 0..n {
        for c in 0..n {
            let ring = r.min(c).min(n - 1 - r).min(n - 1 - c);
            let net = if ring <= 1 {
                let id = b.net(&format!("S{k}"));
                k += 1;
                id
            } else {
                NetId::NONE
            };
            if ring <= 1 {
                let side = if r == ring && c < n - 1 - ring {
                    0
                } else if c == n - 1 - ring && r < n - 1 - ring {
                    1
                } else if r == n - 1 - ring && c > ring {
                    2
                } else {
                    3
                };
                let lat = if side % 2 == 0 { off(c) } else { off(r) };
                parts.push((ring, side, lat, net));
            }
            pads.push(PadSpec {
                number: format!("{}{}", char::from(b'A' + u8::try_from(r).unwrap_or(0)), c + 1),
                local_pos: Point::new(off(c), off(r)),
                shape: PadShape::Circle { diameter: 500_000 },
                drill: None,
                net,
            });
        }
    }
    let u1 = b.footprint("U1", centre, 0.0, Side::Front, &pads);
    b.mobility(u1, Mobility::Locked);
    let mut per_side = [0usize; 4];
    for (i, &(ring, side, lat, net)) in parts.iter().enumerate() {
        let slot = i64::try_from(per_side[side]).unwrap_or(0);
        per_side[side] += 1;
        // Staggered depths keep the 0603 parts apart.
        let depth = mm(7) + mm(3) * (slot % 3) + mm(9) * ring;
        let lat = lat * 25 / 10;
        let (x, y, rot) = match side {
            0 => (centre.x + lat, centre.y - depth, 90.0),
            1 => (centre.x + depth, centre.y + lat, 0.0),
            2 => (centre.x - lat, centre.y + depth, 90.0),
            _ => (centre.x - depth, centre.y - lat, 0.0),
        };
        let fp = b.smd2(&format!("R{i}"), Point::new(x, y), rot, net, NetId::NONE);
        b.mobility(fp, Mobility::Locked);
    }
    b.build()
}

fn cfg() -> RouterConfig {
    RouterConfig { accel: Accel::None, max_iterations: 30, ..RouterConfig::default() }
}

#[test]
fn detects_the_array_and_its_rings() {
    let board = bga6();
    let arrays = detect_arrays(&board);
    assert_eq!(arrays.len(), 1, "only the BGA is an array (0603 parts are not)");
    let a = &arrays[0];
    assert_eq!((a.cols, a.rows, a.pitch), (6, 6, 1_000_000));
    assert_eq!(a.pads.iter().filter(|p| p.ring == 0).count(), 20);
    assert_eq!(a.pads.iter().filter(|p| p.ring == 1).count(), 12);
    assert_eq!(a.pads.iter().filter(|p| p.ring == 2).count(), 4);
}

#[test]
fn boards_without_an_array_have_none() {
    let mut b = BoardBuilder::new(mm(20), mm(20), 2);
    let n = b.net("N");
    b.smd2("R1", Point::new(mm(5), mm(5)), 0.0, n, NetId::NONE);
    assert!(detect_arrays(&b.build()).is_empty());
}

#[test]
fn bga_rings_escape_clean_and_deterministic() {
    let board = bga6();
    let out = route_once(&board, &cfg()).expect("route");
    assert_eq!(out.stats.escape_balls, 32, "{:?}", out.stats);
    assert_eq!(out.stats.drc_gate_rips, 0);
    let mut after = board.clone();
    after.apply(&out.edit);
    let rep = check(&after, &DrcConfig::default());
    assert!(rep.introduced().next().is_none(), "{:#?}", rep.introduced().collect::<Vec<_>>());
    assert_eq!(rep.connectivity.connections_missing, 0, "{:?}", rep.connectivity.unconnected);
    for t in &out.edit.new_tracks {
        assert!((t.end - t.start).is_octilinear(), "{t:?}");
    }
    let again = route_once(&board, &cfg()).expect("route");
    assert_eq!(out.edit, again.edit, "INV-DET: same input, same copper");
}

#[test]
fn escapes_respect_clearance_between_each_other() {
    let board = bga6();
    let obs = Obstacles::from_board(&board);
    let layers: Vec<LayerId> = (0..board.layer_count()).map(LayerId).collect();
    let bbox = board.outline.bbox().expect("outline");
    let (pitch, origin) = crate::choose_lattice(&board, bbox, None);
    let grid = Grid::covering(Rect::new(origin, bbox.max), pitch, layers);
    let arrays = detect_arrays(&board);
    let geoms = crate::plan::class_geoms(&board.rules);
    let planned = plan(&board, &obs, &geoms, &grid, &arrays, &|p| {
        (board.pad(p).net != NetId::NONE).then_some(p.idx())
    });
    assert_eq!(planned.len(), 32);
    let clr = board.rules.classes[0].clearance;
    for (i, a) in planned.iter().enumerate() {
        for c in &planned[i + 1..] {
            for t in &a.tracks {
                for u in c.tracks.iter().filter(|u| u.layer == t.layer) {
                    assert!(!t.hull().clearance_violated(&u.hull(), clr), "{t:?} vs {u:?}");
                }
            }
        }
    }
}

#[test]
fn escapes_never_complete_less_than_plain_routing() {
    let board = bga6();
    let missing = |escape: bool| {
        let out = route_once(&board, &RouterConfig { escape, ..cfg() }).expect("route");
        let mut after = board.clone();
        after.apply(&out.edit);
        check(&after, &DrcConfig::default()).connectivity.connections_missing
    };
    let (with, without) = (missing(true), missing(false));
    assert!(with <= without, "{with} > {without}");
}
