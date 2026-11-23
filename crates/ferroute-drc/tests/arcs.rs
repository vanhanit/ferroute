// SPDX-License-Identifier: AGPL-3.0-only
//! Arc tracks are first-class copper for every DRC check.

use ferroute_board::builder::{BoardBuilder, PadSpec, mm, pt10};
use ferroute_board::{ArcTrack, Board, LayerId, NetId, Origin, PadShape, Side};
use ferroute_drc::{DrcConfig, ItemRef, ViolationKind, check, quality, rules};
use ferroute_geom::Point;

const W: i64 = 250_000;

/// Quarter circle of radius 1 mm around (6, 6) mm: (6,5) -> (7,6), shifted by `(dx, dy)`.
fn arc(dx: i64, dy: i64, w: i64, net: NetId, origin: Origin) -> ArcTrack {
    let p = |x: i64, y: i64| Point::new(x + dx, y + dy);
    ArcTrack {
        start: p(6_000_000, 5_000_000),
        mid: p(6_707_107, 5_292_893),
        end: p(7_000_000, 6_000_000),
        width: w,
        layer: LayerId(0),
        net,
        locked: false,
        origin,
    }
}

fn pad_fp(b: &mut BoardBuilder, name: &str, at: Point, net: NetId) {
    let spec = PadSpec {
        number: "1".into(),
        local_pos: Point::ORIGIN,
        shape: PadShape::Rect { w: 900_000, h: 950_000 },
        drill: None,
        net,
    };
    b.footprint(name, at, 0.0, Side::Front, &[spec]);
}

/// Net-B pad whose right edge is `gap` mm-nm from the arc start's round cap.
fn board_with_pad(pad_right_edge: i64, origin: Origin) -> Board {
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let a = b.net("A");
    let bn = b.net("B");
    pad_fp(&mut b, "R1", Point::new(pad_right_edge - 450_000, 5_000_000), bn);
    let mut board = b.build();
    board.arcs.push(arc(0, 0, W, a, origin));
    board
}

fn is_arc0(v: &ferroute_drc::Violation) -> bool {
    v.a == ItemRef::Arc(0) || v.b == Some(ItemRef::Arc(0))
}

#[test]
fn arc_near_pad_is_a_clearance_violation_and_introduced_when_routed() {
    // Cap reaches x = 5.875 mm; pad edge at 5.775 mm: gap 0.1 mm < 0.2 mm class clearance.
    let board = board_with_pad(5_775_000, Origin::Routed);
    let r = check(&board, &DrcConfig::default());
    let v: Vec<_> = r.violations.iter().filter(|v| v.kind == ViolationKind::Clearance).collect();
    assert!(!v.is_empty(), "{:#?}", r.violations);
    assert!(v.iter().all(|v| is_arc0(v) && v.introduced));
    assert_eq!(r.count(ViolationKind::Short), 0);
}

#[test]
fn existing_arc_violation_is_not_introduced() {
    let board = board_with_pad(5_775_000, Origin::Existing);
    let r = check(&board, &DrcConfig::default());
    assert!(r.count(ViolationKind::Clearance) >= 1);
    assert!(r.violations.iter().all(|v| !v.introduced), "{:#?}", r.violations);
}

#[test]
fn arc_one_millimetre_away_is_clean() {
    let board = board_with_pad(4_775_000, Origin::Routed);
    let r = check(&board, &DrcConfig::default());
    assert!(r.is_clean(), "{:#?}", r.violations);
}

#[test]
fn arc_overlapping_pad_is_a_short() {
    let board = board_with_pad(6_050_000, Origin::Routed);
    let r = check(&board, &DrcConfig::default());
    assert!(r.violations.iter().any(|v| v.kind == ViolationKind::Short && is_arc0(v)));
}

#[test]
fn each_arc_pair_conflict_is_reported_once() {
    // Two crossing arcs of different nets produce many piece pairs but one violation.
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let a = b.net("A");
    let c = b.net("C");
    let mut board = b.build();
    board.arcs.push(arc(0, 0, W, a, Origin::Routed));
    board.arcs.push(arc(100_000, 0, W, c, Origin::Routed));
    let r = check(&board, &DrcConfig::default());
    assert_eq!(r.count(ViolationKind::Short), 1, "{:#?}", r.violations);
}

#[test]
fn arc_crossing_the_board_edge_is_flagged() {
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let a = b.net("A");
    let mut board = b.build();
    board.arcs.push(arc(-6_000_000, 0, W, a, Origin::Routed));
    let r = check(&board, &DrcConfig::default());
    let v: Vec<_> =
        r.violations.iter().filter(|v| v.kind == ViolationKind::EdgeClearance).collect();
    assert_eq!(v.len(), 1, "{:#?}", r.violations);
    assert_eq!(v[0].a, ItemRef::Arc(0));
    assert!(v[0].introduced);
}

#[test]
fn arc_inside_a_no_tracks_keepout_is_flagged() {
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let a = b.net("A");
    b.keepout(ferroute_geom::Rect::new(pt10(55, 45), pt10(75, 65)), true, false);
    let mut board = b.build();
    board.arcs.push(arc(0, 0, W, a, Origin::Routed));
    let r = check(&board, &DrcConfig::default());
    let v: Vec<_> = r.violations.iter().filter(|v| v.kind == ViolationKind::Keepout).collect();
    assert_eq!(v.len(), 1, "{:#?}", r.violations);
    assert_eq!(v[0].a, ItemRef::Arc(0));
}

#[test]
fn arc_narrower_than_minimum_width_is_flagged() {
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let a = b.net("A");
    b.rules_mut().min_track_width = 200_000;
    let mut board = b.build();
    board.arcs.push(arc(0, 0, 100_000, a, Origin::Routed));
    let r = check(&board, &DrcConfig::default());
    let v: Vec<_> = r.violations.iter().filter(|v| v.kind == ViolationKind::TrackWidth).collect();
    assert_eq!(v.len(), 1, "{:#?}", r.violations);
    assert_eq!(v[0].a, ItemRef::Arc(0));
    assert!(v[0].introduced);
}

/// Pads of net A at (5,5) and (7,8) joined by track - arc - track.
fn chain(with_arc: bool) -> Board {
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let a = b.net("A");
    pad_fp(&mut b, "R1", Point::new(5_000_000, 5_000_000), a);
    pad_fp(&mut b, "R2", Point::new(7_000_000, 8_000_000), a);
    b.track(
        Point::new(5_000_000, 5_000_000),
        Point::new(6_000_000, 5_000_000),
        W,
        0,
        a,
        Origin::Routed,
    );
    b.track(
        Point::new(7_000_000, 6_000_000),
        Point::new(7_000_000, 8_000_000),
        W,
        0,
        a,
        Origin::Routed,
    );
    let mut board = b.build();
    if with_arc {
        board.arcs.push(arc(0, 0, W, a, Origin::Routed));
    }
    board
}

#[test]
fn arc_connects_the_copper_it_touches() {
    let r = check(&chain(true), &DrcConfig::default());
    assert_eq!(r.connectivity.connections_missing, 0, "{:#?}", r.connectivity);
    assert!(r.is_clean(), "{:#?}", r.violations);
    let r = check(&chain(false), &DrcConfig::default());
    assert_eq!(r.connectivity.connections_missing, 1);
}

fn acute_board(line: (Point, Point)) -> Board {
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let n = b.net("N");
    b.track(line.0, line.1, W, 0, n, Origin::Routed);
    let mut board = b.build();
    board.arcs.push(arc(0, 0, W, n, Origin::Routed));
    board
}

#[test]
fn tangent_line_arc_joint_is_not_an_acid_trap() {
    let board = acute_board((Point::new(4_000_000, 5_000_000), Point::new(6_000_000, 5_000_000)));
    assert!(quality::acute_angles(&board).is_empty());
    // The same at the arc end (tangent is vertical there).
    let board = acute_board((Point::new(7_000_000, 6_000_000), Point::new(7_000_000, 8_000_000)));
    assert!(quality::acute_angles(&board).is_empty());
}

#[test]
fn acute_line_meeting_arc_end_is_reported() {
    let board = acute_board((Point::new(7_000_000, 6_000_000), Point::new(8_000_000, 5_000_000)));
    let traps = quality::acute_angles(&board);
    assert_eq!(traps.len(), 1, "{traps:?}");
    assert_eq!(traps[0].at, Point::new(7_000_000, 6_000_000));
}

#[test]
fn net_lengths_include_arc_length() {
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let a = b.net("A");
    let mut board = b.build();
    board.arcs.push(arc(0, 0, W, a, Origin::Routed));
    let len = rules::net_lengths(&board)[a.idx()];
    assert!((len - 1_570_796).abs() <= 10, "{len}");
    board.tracks.push(ferroute_board::Track {
        start: Point::new(0, 0),
        end: Point::new(1_000_000, 0),
        width: W,
        layer: LayerId(0),
        net: a,
        locked: false,
        origin: Origin::Routed,
    });
    let len = rules::net_lengths(&board)[a.idx()];
    assert!((len - 2_570_796).abs() <= 10, "{len}");
}

#[test]
fn constraints_report_a_too_narrow_arc() {
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let a = b.net("A");
    let mut board = b.build();
    board.arcs.push(arc(0, 0, 50_000, a, Origin::Routed));
    let issues = rules::check_constraints(&board);
    assert!(
        issues.iter().any(|i| i.net == a && matches!(i.kind, rules::IssueKind::Width { actual: 50_000, .. })),
        "{issues:?}"
    );
}
