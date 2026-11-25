// SPDX-License-Identifier: AGPL-3.0-only
//! KiCad `starved_thermal`: a thermal relief needs `min_resolved_spokes` (2) spokes.

use ferroute_board::builder::{BoardBuilder, mm, pt10};
use ferroute_board::{
    Board, BoardEdit, IslandRemoval, LayerId, LayerSet, NetId, Origin, PadConnection, Track, Zone,
    ZoneFill,
};
use ferroute_drc::refill::refill_zones;
use ferroute_drc::{DrcConfig, ViolationKind, check};
use ferroute_geom::boolean::{bridge, difference, regions_of};
use ferroute_geom::{Point, Polygon};

fn rect_nm(x0: i64, y0: i64, x1: i64, y1: i64) -> Polygon {
    Polygon::new(vec![
        Point::new(x0, y0),
        Point::new(x1, y0),
        Point::new(x1, y1),
        Point::new(x0, y1),
    ])
}

/// GND pour (F.Cu, 2..28 x 12..18 mm) with a thermal relief around pad 1 of R1 (pad at
/// x 6.75..7.65, y 14.525..15.475; gap 0.3 mm) and the given spokes (0.4 mm wide):
/// `l` left, `r` right. R2 (GND, x 20) is solidly inside the fill and keeps it connected.
fn board(l: bool, r: bool) -> (Board, NetId) {
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let (gnd, a) = (b.net("GND"), b.net("A"));
    b.smd2("R1", pt10(80, 150), 0.0, gnd, gnd);
    b.smd2("R2", pt10(200, 150), 0.0, gnd, gnd);
    b.smd2("R3", pt10(100, 50), 0.0, a, a);
    let mut board = b.build();
    let outer = rect_nm(2_000_000, 12_000_000, 28_000_000, 18_000_000);
    let void = rect_nm(6_450_000, 14_225_000, 7_950_000, 15_775_000);
    let mut polys: Vec<Polygon> =
        difference(&regions_of(&[outer]), &[void]).iter().filter_map(bridge).collect();
    if l {
        polys.push(rect_nm(6_400_000, 14_800_000, 6_800_000, 15_200_000));
    }
    if r {
        polys.push(rect_nm(7_600_000, 14_800_000, 8_000_000, 15_200_000));
    }
    let fill: Vec<Polygon> = regions_of(&polys).iter().filter_map(bridge).collect();
    board.zones.push(Zone {
        net: gnd,
        layers: LayerSet::single(LayerId(0)),
        outline: rect_nm(2_000_000, 12_000_000, 28_000_000, 18_000_000),
        fills: vec![ZoneFill { layer: LayerId(0), polys: fill }],
        clearance: 200_000,
        min_thickness: 250_000,
        priority: 0,
        pad_connection: PadConnection::ThermalRelief,
        islands: IslandRemoval::Always,
        yields: false,
    });
    (board, a)
}

fn starved(board: &Board) -> usize {
    check(board, &DrcConfig::default()).count(ViolationKind::StarvedThermal)
}

#[test]
fn two_spokes_are_enough() {
    let (board, _) = board(true, true);
    assert_eq!(starved(&board), 0);
}

#[test]
fn one_spoke_is_starved() {
    let (board, _) = board(true, false);
    let rep = check(&board, &DrcConfig::default());
    let v: Vec<_> =
        rep.violations.iter().filter(|v| v.kind == ViolationKind::StarvedThermal).collect();
    assert_eq!(v.len(), 1, "{:#?}", rep.violations);
    assert_eq!(v[0].required, 2);
    assert!((v[0].actual - 1.0).abs() < 1e-9);
}

#[test]
fn a_pad_without_any_spoke_is_not_reported() {
    // KiCad skips pads the fill does not reach at all; connectivity owns that case.
    let (board, _) = board(false, false);
    assert_eq!(starved(&board), 0);
}

#[test]
fn a_solid_connection_has_no_spokes_to_count() {
    let (mut board, _) = board(true, false);
    board.zones[0].pad_connection = PadConnection::Solid;
    assert_eq!(starved(&board), 0);
}

#[test]
fn a_same_net_track_into_the_fill_counts_as_a_manual_spoke() {
    let (mut board, _) = board(true, false);
    let gnd = board.zones[0].net;
    board.tracks.push(Track {
        start: Point::new(7_200_000, 15_000_000),
        end: Point::new(7_200_000 + 1_500_000, 15_000_000),
        width: 250_000,
        layer: LayerId(0),
        net: gnd,
        locked: false,
        origin: Origin::Routed,
    });
    assert_eq!(starved(&board), 0);
}

fn refilled(board: &Board) -> (Board, Vec<String>) {
    let mut b = board.clone();
    b.zones[0].yields = true;
    let r = refill_zones(&b);
    let mut after = b;
    after.apply(&BoardEdit { zone_fills: r.fills, ..BoardEdit::default() });
    after.zones[0].yields = false;
    (after, r.notes)
}

fn cut_right_spoke(board: &mut Board, net: NetId) {
    board.tracks.push(Track {
        start: Point::new(8_000_000, 12_500_000),
        end: Point::new(8_000_000, 17_500_000),
        width: 250_000,
        layer: LayerId(0),
        net,
        locked: false,
        origin: Origin::Routed,
    });
}

#[test]
fn a_refill_that_would_starve_a_relief_isolates_the_pad_instead() {
    let (mut board, a) = board(true, true);
    cut_right_spoke(&mut board, a);
    let (after, notes) = refilled(&board);
    assert_eq!(starved(&after), 0, "{notes:?}");
    assert!(notes.iter().any(|n| n.contains("starved") && n.contains("R1.1")), "{notes:?}");
    // The pad is cut off from the fill: the engine sees it as a refill disconnect.
    let only_conn = DrcConfig { copper: false, edge: false, holes: false, ..DrcConfig::default() };
    let rep = check(&after, &only_conn);
    assert!(rep.connectivity.connections_missing > 0);
    let full = check(&after, &DrcConfig::default());
    assert!(full.is_clean(), "{:#?}", full.violations);
}

#[test]
fn a_refill_that_keeps_both_spokes_changes_nothing_about_the_relief() {
    let (mut board, a) = board(true, true);
    board.tracks.push(Track {
        start: Point::new(12_000_000, 12_500_000),
        end: Point::new(12_000_000, 17_500_000),
        width: 250_000,
        layer: LayerId(0),
        net: a,
        locked: false,
        origin: Origin::Routed,
    });
    let (after, notes) = refilled(&board);
    assert!(notes.is_empty(), "{notes:?}");
    assert_eq!(starved(&after), 0);
}

#[test]
fn a_relief_that_was_already_starved_is_not_cut_by_the_refill() {
    let (mut board, a) = board(true, false);
    cut_right_spoke(&mut board, a);
    let (_, notes) = refilled(&board);
    assert!(notes.iter().all(|n| !n.contains("starved")), "{notes:?}");
}
