// SPDX-License-Identifier: AGPL-3.0-only
use super::*;
use ferroute_board::builder::{BoardBuilder, mm, pt10};
use ferroute_board::{Board, Mobility, Origin};
use ferroute_geom::{Angle, Point};
use proptest::prelude::*;

/// Two resistors side by side, nets A and B, nothing routed.
fn two_resistors() -> (Board, ferroute_board::NetId, ferroute_board::NetId) {
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let a = b.net("A");
    let bn = b.net("B");
    b.smd2("R1", pt10(100, 50), 0.0, a, bn);
    b.smd2("R2", pt10(100, 150), 0.0, a, bn);
    (b.build(), a, bn)
}

#[test]
fn unrouted_board_is_clean_but_incomplete() {
    let (board, ..) = two_resistors();
    let r = check(&board, &DrcConfig::default());
    assert!(r.is_clean(), "{:#?}", r.violations);
    assert_eq!(r.connectivity.nets_total, 2);
    assert_eq!(r.connectivity.connections_missing, 2);
    assert_eq!(r.connectivity.unconnected.len(), 2);
    assert!(r.connectivity.completion().abs() < 1e-12);
}

#[test]
fn straight_tracks_complete_the_board() {
    let (board, a, bn) = two_resistors();
    let mut board = board;
    // R1 pad1 (9.2,5.0) → R2 pad1 (9.2,15.0); R1 pad2 (10.8,5) → R2 pad2 (10.8,15).
    board.tracks.push(track(pt10(92, 50), pt10(92, 150), a));
    board.tracks.push(track(pt10(108, 50), pt10(108, 150), bn));
    let r = check(&board, &DrcConfig::default());
    assert!(r.is_clean(), "{:#?}", r.violations);
    assert_eq!(r.connectivity.connections_missing, 0);
    assert!((r.connectivity.completion() - 1.0).abs() < 1e-12);
}

fn track(a: Point, b: Point, net: ferroute_board::NetId) -> ferroute_board::Track {
    ferroute_board::Track {
        start: a,
        end: b,
        width: 250_000,
        layer: ferroute_board::LayerId(0),
        net,
        locked: false,
        origin: Origin::Routed,
    }
}

#[test]
fn detects_short_and_clearance() {
    let (mut board, a, bn) = two_resistors();
    // Track of net A crossing net B's pad = short.
    board.tracks.push(track(pt10(92, 50), pt10(120, 50), a));
    let r = check(&board, &DrcConfig::default());
    assert!(r.count(ViolationKind::Short) >= 1, "{:#?}", r.violations);
    assert!(r.introduced().count() >= 1);
    // Track of net B at 0.3 mm centre distance from a net-A track (gap 0.05 mm < 0.2 mm).
    let (mut board, a, bn2) = two_resistors();
    let _ = bn;
    board.tracks.push(track(Point::new(mm(2), mm(10)), Point::new(mm(6), mm(10)), a));
    board.tracks.push(track(
        Point::new(mm(2), mm(10) + 300_000),
        Point::new(mm(6), mm(10) + 300_000),
        bn2,
    ));
    let r = check(&board, &DrcConfig::default());
    assert_eq!(r.count(ViolationKind::Clearance), 1, "{:#?}", r.violations);
    let v = &r.violations[0];
    assert!((v.actual - 50_000.0).abs() < 1.0);
    assert_eq!(v.required, 200_000);
}

#[test]
fn detects_edge_clearance() {
    let (mut board, a, _) = two_resistors();
    board.tracks.push(track(Point::new(200_000, mm(2)), Point::new(200_000, mm(4)), a));
    let r = check(&board, &DrcConfig::default());
    assert_eq!(r.count(ViolationKind::EdgeClearance), 1, "{:#?}", r.violations);
}

#[test]
fn detects_courtyard_overlap_only_on_same_side() {
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let n = b.net("N");
    b.smd2("R1", pt10(100, 100), 0.0, n, n);
    b.smd2("R2", pt10(110, 100), 0.0, n, n);
    let board = b.build();
    let r = check(&board, &DrcConfig::default());
    assert_eq!(r.count(ViolationKind::CourtyardOverlap), 1);
}

#[test]
fn detects_hole_to_hole_and_keepout() {
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let n = b.net("N");
    let m = b.net("M");
    b.via(pt10(100, 100), 600_000, 300_000, n, Origin::Routed);
    b.via(Point::new(mm(10) + 500_000, mm(10)), 400_000, 300_000, n, Origin::Routed);
    b.keepout(ferroute_geom::Rect::new(pt10(200, 50), pt10(250, 150)), true, true);
    b.track(pt10(190, 100), pt10(260, 100), 250_000, 0, m, Origin::Routed);
    let r = check(&b.build(), &DrcConfig::default());
    assert_eq!(r.count(ViolationKind::HoleToHole), 1, "{:#?}", r.violations);
    assert_eq!(r.count(ViolationKind::Keepout), 1, "{:#?}", r.violations);
}

#[test]
fn net_class_max_semantics() {
    let (mut board, a, bn) = two_resistors();
    board.rules.classes.push(ferroute_board::NetClass {
        name: "HV".into(),
        clearance: 500_000,
        ..Default::default()
    });
    board.nets[bn.idx()].class = 1;
    // Gap 0.4 mm: legal for Default (0.2) but not for HV (0.5) → must be flagged.
    board.tracks.push(track(Point::new(mm(2), mm(10)), Point::new(mm(6), mm(10)), a));
    board.tracks.push(track(
        Point::new(mm(2), mm(10) + 650_000),
        Point::new(mm(6), mm(10) + 650_000),
        bn,
    ));
    let r = check(&board, &DrcConfig::default());
    assert_eq!(r.count(ViolationKind::Clearance), 1, "{:#?}", r.violations);
}

#[test]
fn moved_footprint_violations_are_marked_introduced() {
    let (mut board, ..) = two_resistors();
    board.footprints[1].mobility =
        Mobility::Movable { max_displacement: mm(20), allow_rotation: true };
    board.footprints[1].placement.pos = pt10(100, 60);
    board.footprints[1].placement.rot = Angle::from_degrees(90.0);
    let r = check(&board, &DrcConfig::default());
    assert!(r.violations.iter().any(|v| v.kind == ViolationKind::CourtyardOverlap && v.introduced));
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    /// Bucketed DRC must agree exactly with an O(n²) brute force over all pairs.
    #[test]
    fn bucketed_copper_check_equals_brute_force(
        segs in prop::collection::vec((0i64..300, 0i64..200, 0i64..300, 0i64..200, 0u8..2, 1u32..4), 1..40)
    ) {
        let mut b = BoardBuilder::new(mm(30), mm(20), 2);
        let nets: Vec<_> = (0..4).map(|i| b.net(&format!("N{i}"))).collect();
        for &(x0, y0, x1, y1, l, n) in &segs {
            b.track(pt10(x0, y0), pt10(x1, y1), 200_000, l, nets[n as usize], Origin::Routed);
        }
        let board = b.build();
        let cfg = DrcConfig { edge: false, holes: false, keepouts: false, courtyards: false, sizes: false, connectivity: false, ..DrcConfig::default() };
        let fast: Vec<_> = check(&board, &cfg).violations.iter().map(|v| (v.a, v.b)).collect();
        let items = copper_items(&board);
        let mut slow = Vec::new();
        for i in 0..items.len() {
            for j in i + 1..items.len() {
                let (x, y) = (&items[i], &items[j]);
                if x.net != y.net && x.layers.intersects(y.layers) && x.hull.clearance_violated(&y.hull, board.clearance_between(x.net, y.net)) {
                    slow.push((x.item, Some(y.item)));
                }
            }
        }
        slow.sort();
        let mut fast_sorted = fast;
        fast_sorted.sort();
        prop_assert_eq!(fast_sorted, slow);
    }
}

/// A plated 0.6 x 1.8 mm slot (long axis along the pad's x) next to a 0.3 mm via.
fn slot_board(via_offset: Point) -> Board {
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let (n, m) = (b.net("N"), b.net("M"));
    b.smd2("J1", pt10(100, 100), 0.0, n, n);
    b.via(pt10(200, 150), 600_000, 300_000, m, Origin::Routed);
    let mut board = b.build();
    board.pads[0].drill = Some(ferroute_board::Drill {
        diameter: 1_800_000,
        plated: true,
        slot: Some([1_800_000, 600_000]),
    });
    let c = board.pad_position(ferroute_board::PadId(0));
    board.vias[0].pos = c + via_offset;
    board
}

#[test]
fn slot_hole_uses_the_capsule_not_a_disk_of_its_length() {
    let h2h = |board: &Board| check(board, &DrcConfig::default()).count(ViolationKind::HoleToHole);
    // 1.0 mm off the slot's flank: capsule gap 0.55 mm is fine (a 1.8 mm disk would hit).
    assert_eq!(h2h(&slot_board(Point::new(0, 1_000_000))), 0);
    // 1.0 mm beyond the slot's end: gap -0.05 mm.
    assert_eq!(h2h(&slot_board(Point::new(1_000_000, 0))), 1);
}

#[test]
fn touching_courtyards_do_not_overlap_but_overlapping_ones_do() {
    let place = |dx_mm: i64| {
        let mut b = BoardBuilder::new(mm(30), mm(20), 2);
        let n = b.net("N");
        b.smd2("R1", pt10(100, 100), 0.0, n, n);
        b.smd2("R2", Point::new(mm(10) + dx_mm, mm(10)), 0.0, n, n);
        let mut board = b.build();
        for f in &mut board.footprints {
            f.courtyard_local = ferroute_geom::Polygon::from_rect(ferroute_geom::Rect::new(
                Point::new(-1_000_000, -1_000_000),
                Point::new(1_000_000, 1_000_000),
            ));
        }
        check_courtyards(&board).len()
    };
    assert_eq!(place(2_000_000), 0, "shared edge");
    assert_eq!(place(1_999_999), 1, "1 nm of overlap");
    assert_eq!(place(3_000_000), 0);
}

#[test]
fn footprints_without_a_drawn_courtyard_never_overlap() {
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let n = b.net("N");
    b.smd2("R1", pt10(100, 100), 0.0, n, n);
    b.smd2("R2", pt10(110, 100), 0.0, n, n);
    let mut board = b.build();
    assert_eq!(check_courtyards(&board).len(), 1);
    board.footprints[1].has_courtyard = false;
    assert!(check_courtyards(&board).is_empty());
}
