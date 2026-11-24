// SPDX-License-Identifier: AGPL-3.0-only
//! Copper pours: clearance to other-net copper and connectivity through the fill.

use ferroute_board::builder::{BoardBuilder, mm, pt10};
use ferroute_board::{
    Board, LayerId, LayerSet, NetId, Origin, PadConnection, Track, Zone, ZoneFill,
};
use ferroute_drc::{DrcConfig, ItemRef, ViolationKind, check};
use ferroute_geom::{Point, Polygon};

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Polygon {
    Polygon::new(vec![pt10(x0, y0), pt10(x1, y0), pt10(x1, y1), pt10(x0, y1)])
}

/// GND pour on F.Cu covering x 2..28, y 12..18 mm with a notch; R1 (GND,GND) and R2
/// (GND,GND) inside it, R3 (A,B) outside (y = 5 mm).
fn board(connect: PadConnection) -> (Board, NetId, NetId, NetId) {
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let (gnd, a, bn) = (b.net("GND"), b.net("A"), b.net("B"));
    b.smd2("R1", pt10(80, 150), 0.0, gnd, gnd);
    b.smd2("R2", pt10(200, 150), 0.0, gnd, gnd);
    b.smd2("R3", pt10(100, 50), 0.0, a, bn);
    let mut board = b.build();
    board.zones.push(Zone {
        net: gnd,
        layers: LayerSet::single(LayerId(0)),
        outline: rect(20, 120, 280, 180),
        fills: vec![ZoneFill { layer: LayerId(0), polys: vec![rect(20, 120, 280, 180)] }],
        clearance: 200_000,
        min_thickness: 250_000,
        priority: 0,
        pad_connection: connect,
        islands: Default::default(),
        yields: false,
    });
    (board, gnd, a, bn)
}

fn track(a: Point, b: Point, net: NetId, layer: u8, origin: Origin) -> Track {
    Track { start: a, end: b, width: 250_000, layer: LayerId(layer), net, locked: false, origin }
}

#[test]
fn track_of_another_net_crossing_a_fill_is_a_violation() {
    let (mut board, _, a, _) = board(PadConnection::ThermalRelief);
    board.tracks.push(track(pt10(150, 100), pt10(150, 140), a, 0, Origin::Routed));
    let r = check(&board, &DrcConfig::default());
    let v: Vec<_> = r.violations.iter().filter(|v| v.b == Some(ItemRef::Zone(0))).collect();
    assert_eq!(v.len(), 1, "{:#?}", r.violations);
    assert_eq!(v[0].kind, ViolationKind::Short);
    assert_eq!(v[0].a, ItemRef::Track(0));
    assert!(v[0].introduced);
}

#[test]
fn clearance_to_the_fill_edge_uses_the_zone_clearance() {
    let (mut board, _, a, _) = board(PadConnection::ThermalRelief);
    board.rules.classes[0].clearance = 100_000;
    // Fill edge at y = 12 mm; track copper edge (half width 0.125) at gap 0.3 mm.
    let y = |gap: i64| 12_000_000 - gap - 125_000;
    let at = |gap: i64| {
        track(Point::new(15_000_000, y(gap)), Point::new(25_000_000, y(gap)), a, 0, Origin::Routed)
    };
    board.tracks.push(at(300_000));
    assert!(check(&board, &DrcConfig::default()).is_clean());
    // Gap 0.1 mm: below the zone clearance (0.2) though not below the board clearance.
    board.tracks[0] = at(100_000);
    let r = check(&board, &DrcConfig::default());
    assert_eq!(r.count(ViolationKind::Clearance), 1, "{:#?}", r.violations);
    assert!((r.violations[0].actual - 100_000.0).abs() < 1.0);
    assert_eq!(r.violations[0].required, 200_000);
}

#[test]
fn other_layers_and_same_net_copper_are_not_in_conflict() {
    let (mut board, gnd, a, _) = board(PadConnection::ThermalRelief);
    board.tracks.push(track(pt10(150, 100), pt10(150, 140), a, 1, Origin::Routed));
    board.tracks.push(track(pt10(120, 130), pt10(180, 130), gnd, 0, Origin::Routed));
    assert!(check(&board, &DrcConfig::default()).is_clean());
}

#[test]
fn existing_violations_are_not_introduced() {
    let (mut board, _, a, _) = board(PadConnection::ThermalRelief);
    board.tracks.push(track(pt10(150, 100), pt10(150, 140), a, 0, Origin::Existing));
    let r = check(&board, &DrcConfig::default());
    assert_eq!(r.violations.len(), 1);
    assert!(!r.violations[0].introduced);
}

#[test]
fn pads_inside_a_pour_of_their_net_are_connected() {
    let (mut board, ..) = board(PadConnection::ThermalRelief);
    let with = check(&board, &DrcConfig::default()).connectivity;
    assert_eq!(with.connections_missing, 0, "{with:#?}");
    assert_eq!((with.nets_total, with.nets_complete), (1, 1));
    board.zones.clear();
    let without = check(&board, &DrcConfig::default()).connectivity;
    assert_eq!(without.connections_missing, 3);
}

#[test]
fn pad_connection_none_isolates_pads() {
    let (board, ..) = board(PadConnection::None);
    let c = check(&board, &DrcConfig::default()).connectivity;
    assert_eq!(c.connections_missing, 3);
}

#[test]
fn smd_pads_need_a_connecting_mode_for_thru_hole_only() {
    let (board, ..) = board(PadConnection::ThruHoleOnly);
    assert_eq!(check(&board, &DrcConfig::default()).connectivity.connections_missing, 3);
}

#[test]
fn tracks_and_vias_of_the_net_join_the_fill() {
    let (mut board, gnd, ..) = board(PadConnection::None);
    // R1 pad 2 and R2 pad 1 are tied to the fill by tracks (pad connection is off).
    board.tracks.push(track(pt10(88, 150), pt10(140, 150), gnd, 0, Origin::Routed));
    board.tracks.push(track(pt10(140, 150), pt10(192, 150), gnd, 0, Origin::Routed));
    let c = check(&board, &DrcConfig::default()).connectivity;
    assert_eq!(c.connections_missing, 2, "{c:#?}");
}

#[test]
fn unfilled_zone_contributes_nothing() {
    let (mut board, ..) = board(PadConnection::Solid);
    board.zones[0].fills.clear();
    assert!(board.zones[0].is_unfilled());
    assert_eq!(check(&board, &DrcConfig::default()).connectivity.connections_missing, 3);
}

#[test]
fn concave_fill_is_exact() {
    // L-shaped fill: a track in the notch is fine, one in the arm is not.
    let (mut board, _, a, _) = board(PadConnection::ThermalRelief);
    let l = Polygon::new(vec![
        pt10(20, 120),
        pt10(280, 120),
        pt10(280, 135),
        pt10(60, 135),
        pt10(60, 180),
        pt10(20, 180),
    ]);
    board.zones[0].fills[0].polys = vec![l];
    board.tracks.push(track(pt10(200, 160), pt10(250, 160), a, 0, Origin::Routed));
    let r = check(&board, &DrcConfig::default());
    assert_eq!(r.violations.iter().filter(|v| v.b == Some(ItemRef::Zone(0))).count(), 0);
    board.tracks.push(track(pt10(200, 100), pt10(200, 130), a, 0, Origin::Routed));
    let r = check(&board, &DrcConfig::default());
    assert_eq!(r.violations.iter().filter(|v| v.b == Some(ItemRef::Zone(0))).count(), 1);
}

#[test]
fn existing_copper_gets_a_micron_of_slack_but_new_copper_does_not() {
    let (mut board, _, a, _) = board(PadConnection::ThermalRelief);
    // Gap 0.1999 mm to the fill edge: KiCad fill rounding for old copper, a violation for new.
    let y = 12_000_000 - 199_900 - 125_000;
    let t = |origin| track(Point::new(15_000_000, y), Point::new(25_000_000, y), a, 0, origin);
    board.tracks.push(t(Origin::Existing));
    assert!(check(&board, &DrcConfig::default()).is_clean());
    board.tracks[0] = t(Origin::Routed);
    assert_eq!(check(&board, &DrcConfig::default()).count(ViolationKind::Clearance), 1);
}
