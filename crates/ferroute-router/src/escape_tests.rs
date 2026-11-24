// SPDX-License-Identifier: AGPL-3.0-only
//! Tests of the area-array escape stage ([`crate::escape`]).

use crate::escape::{detect_arrays, plan};
use crate::grid::Grid;
use crate::legality::Obstacles;
use crate::vspan::ViaModel;
use crate::{Accel, RouterConfig, route_once};
use ferroute_board::builder::{BoardBuilder, PadSpec, mm};
use ferroute_board::{Board, LayerId, Mobility, NetId, PadShape, Side, ViaKind, ViaPolicy};
use ferroute_drc::{DrcConfig, check};
use ferroute_geom::{Point, Rect};

/// 6 × 6 BGA (1 mm pitch, 0.5 mm balls), 0.1 mm rules, two layers: the 20 outer balls and
/// the 12 balls of the second ring each go to an 0603 part around the package.
fn bga6() -> Board {
    bga6_on(2, 2)
}

/// The same package with `sig_rings` signal rings on `layers` copper layers.
fn bga6_on(layers: u8, sig_rings: i64) -> Board {
    let (size, per_ring) = if sig_rings > 2 { (60, 5) } else { (50, 9) };
    let mut b = BoardBuilder::new(mm(size), mm(size), layers);
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
    let centre = Point::new(mm(size / 2), mm(size / 2));
    let n = 6i64;
    let off = |i: i64| (2 * i - (n - 1)) * 500_000;
    let mut pads = Vec::new();
    // (ring, side, lateral coordinate, net)
    let mut parts: Vec<(i64, usize, i64, NetId)> = Vec::new();
    let mut k = 0;
    for r in 0..n {
        for c in 0..n {
            let ring = r.min(c).min(n - 1 - r).min(n - 1 - c);
            let net = if ring < sig_rings {
                let id = b.net(&format!("S{k}"));
                k += 1;
                id
            } else {
                NetId::NONE
            };
            if ring < sig_rings {
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
        let depth = mm(7) + mm(3) * (slot % 3) + mm(per_ring) * ring;
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
    assert_eq!(out.stats.escape_balls, 20, "outer ring only by default: {:?}", out.stats);
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

fn planned(
    board: &Board,
    policy: ViaPolicy,
    max_ring: usize,
) -> (Grid, Vec<crate::escape::Escape>) {
    let obs = Obstacles::from_board(board);
    let layers: Vec<LayerId> = (0..board.layer_count()).map(LayerId).collect();
    let bbox = board.outline.bbox().expect("outline");
    let (pitch, origin) = crate::choose_lattice(board, bbox, None);
    let grid = Grid::covering(Rect::new(origin, bbox.max), pitch, layers);
    let vm = ViaModel::new(
        policy,
        &grid,
        board.copper_layers.len(),
        crate::vspan::MICRO_FACTOR,
        crate::vspan::SPAN_FACTOR,
    );
    let arrays = detect_arrays(board);
    let geoms = crate::plan::class_geoms(&board.rules);
    let out = plan(board, &obs, &geoms, &grid, &vm, &arrays, max_ring, &|p| {
        board.pad(p).net != NetId::NONE
    });
    (grid, out)
}

#[test]
fn escapes_respect_clearance_between_each_other() {
    let board = bga6();
    let (_, planned) = planned(&board, ViaPolicy::default(), 1);
    assert!(planned.len() >= 28, "{}", planned.len());
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
fn every_escape_ends_in_a_diagonal_departure_stub_on_the_lattice() {
    let board = bga6();
    let (grid, planned) = planned(&board, ViaPolicy::default(), 1);
    assert!(!planned.is_empty());
    for e in &planned {
        let last = e.tracks.last().expect("track");
        assert_eq!(last.end, e.end.point);
        let v = last.end - last.start;
        assert!(v.x != 0 && v.x.abs() == v.y.abs(), "last leg must be a 45° stub: {last:?}");
        assert!(e.end.heading.is_diagonal(), "{:?}", e.end);
        assert_eq!((e.end.point.x - grid.origin.x) % grid.pitch, 0, "{:?}", e.end);
        assert_eq!((e.end.point.y - grid.origin.y) % grid.pitch, 0, "{:?}", e.end);
        for t in &e.tracks {
            assert!((t.end - t.start).is_octilinear(), "{t:?}");
        }
        // The stub is at least half a pitch long: no tiny kink.
        assert!(v.x.abs() * 2 >= grid.pitch, "{last:?}");
    }
}

#[test]
fn neighbouring_escape_ends_are_staggered() {
    let board = bga6();
    let (_, planned) = planned(&board, ViaPolicy::default(), 1);
    // The straight top-side escapes of the outer ring do not all end on one row.
    let mut ys: Vec<i64> = planned
        .iter()
        .filter(|e| {
            e.end.heading == ferroute_geom::Dir8::SE || e.end.heading == ferroute_geom::Dir8::SW
        })
        .map(|e| e.end.point.y)
        .collect();
    ys.sort_unstable();
    ys.dedup();
    assert!(ys.len() >= 2, "ends all on one row: {ys:?}");
}

#[test]
fn fan_out_nests_connections_that_turn_the_same_way() {
    let board = bga6();
    let arrays = detect_arrays(&board);
    let arr = &arrays[0];
    let balls: Vec<&crate::escape::ArrayPad> =
        arr.pads.iter().filter(|b| board.pad(b.pad).net != NetId::NONE).collect();
    let fan = crate::escape::fan_levels(&board, arr, &balls, &board.pads_by_net());
    for u in [(0, -1), (0, 1), (-1, 0), (1, 0)] {
        let lat = |b: &crate::escape::ArrayPad| if u.0 == 0 { b.centre.x } else { b.centre.y };
        let mut side: Vec<(i64, i64, i64)> = balls
            .iter()
            .filter_map(|b| fan.get(&(b.pad.idx(), u)).map(|&(t, l)| (lat(b), t, l)))
            .collect();
        side.sort_unstable();
        // Connections turning towards +lateral: the further from the turn, the deeper.
        let plus: Vec<i64> = side.iter().filter(|s| s.1 == 1).map(|s| s.2).collect();
        assert!(plus.windows(2).all(|w| w[0] > w[1]), "side {u:?}: {side:?}");
        let minus: Vec<i64> = side.iter().filter(|s| s.1 == -1).map(|s| s.2).collect();
        assert!(minus.windows(2).all(|w| w[0] < w[1]), "side {u:?}: {side:?}");
    }
}

#[test]
fn the_plan_is_deterministic() {
    let board = bga6();
    let (_, a) = planned(&board, ViaPolicy::default(), 1);
    let (_, b) = planned(&board, ViaPolicy::default(), 1);
    assert_eq!(a, b);
}

#[test]
fn through_policy_plans_through_vias_only() {
    let board = bga6_on(4, 3);
    let (_, p) = planned(&board, ViaPolicy::default(), 2);
    assert!(p.iter().any(|e| !e.vias.is_empty()), "deep balls need vias");
    assert!(p.iter().all(|e| e.vias.iter().all(|v| v.kind == ViaKind::Through)));
}

#[test]
fn span_policy_picks_span_vias_when_through_is_not_allowed() {
    let board = bga6_on(4, 3);
    let policy = ViaPolicy { through: false, blind: true, micro: true, ..ViaPolicy::default() };
    let (_, p) = planned(&board, policy, 2);
    let vias: Vec<_> = p.iter().flat_map(|e| e.vias.iter()).collect();
    assert!(!vias.is_empty(), "deep balls need vias");
    assert!(vias.iter().all(|v| v.kind != ViaKind::Through && v.kind != ViaKind::Buried));
}

#[test]
fn routing_is_identical_on_one_and_many_threads() {
    let board = bga6();
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("pool");
        pool.install(|| route_once(&board, &cfg()).expect("route").edit)
    };
    assert_eq!(run(1), run(4));
}

#[test]
fn escape_end_continuations_make_no_right_angle() {
    let board = bga6();
    let out = route_once(&board, &cfg()).expect("route");
    let mut after = board;
    after.apply(&out.edit);
    assert!(ferroute_drc::quality::right_angles(&after).is_empty());
    assert!(ferroute_drc::quality::acute_angles(&after).is_empty());
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

#[test]
fn inner_rings_escape_clean_when_asked() {
    let board = bga6();
    let c = RouterConfig { escape_rings: 1, ..cfg() };
    let out = route_once(&board, &c).expect("route");
    assert!(out.stats.escape_balls >= 28, "{:?}", out.stats);
    assert_eq!(out.stats.drc_gate_rips, 0);
    let mut after = board;
    after.apply(&out.edit);
    let rep = check(&after, &DrcConfig::default());
    assert!(rep.introduced().next().is_none());
    assert!(ferroute_drc::quality::acute_angles(&after).is_empty());
}
