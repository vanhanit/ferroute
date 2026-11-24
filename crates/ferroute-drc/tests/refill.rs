// SPDX-License-Identifier: AGPL-3.0-only
//! Copper pours are recomputed around routed copper: clearance, islands, thin slivers.

use ferroute_board::builder::{BoardBuilder, mm, pt10};
use ferroute_board::{
    Board, BoardEdit, IslandRemoval, LayerId, LayerSet, NetId, Origin, PadConnection, Track, Zone,
    ZoneFill,
};
use ferroute_drc::refill::refill_zones;
use ferroute_drc::{DrcConfig, ItemRef, check};
use ferroute_geom::boolean::regions_of;
use ferroute_geom::{Point, Polygon};

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Polygon {
    Polygon::new(vec![pt10(x0, y0), pt10(x1, y0), pt10(x1, y1), pt10(x0, y1)])
}

/// GND pour on F.Cu covering x 2..28, y 12..18 mm; R1 (x 8) and R2 (x 20) GND pads inside;
/// R3 (nets A/B) outside. The zone is marked as yielding (a refill is pending).
fn board() -> (Board, NetId, NetId) {
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
        pad_connection: PadConnection::Solid,
        islands: IslandRemoval::Always,
        yields: true,
    });
    (board, gnd, a)
}

fn track(a: Point, b: Point, net: NetId) -> Track {
    Track {
        start: a,
        end: b,
        width: 250_000,
        layer: LayerId(0),
        net,
        locked: false,
        origin: Origin::Routed,
    }
}

fn refilled(board: &Board) -> Board {
    let r = refill_zones(board);
    let mut after = board.clone();
    after.apply(&BoardEdit { zone_fills: r.fills, ..BoardEdit::default() });
    after
}

fn pieces(board: &Board) -> usize {
    regions_of(&board.zones[0].fills[0].polys).len()
}

fn area(b: &Board) -> i128 {
    regions_of(&b.zones[0].fills[0].polys).iter().map(ferroute_geom::boolean::Region::area2).sum()
}

#[test]
fn a_crossing_track_splits_the_pour_and_the_result_is_drc_clean() {
    let (mut board, _, a) = board();
    board.tracks.push(track(pt10(140, 110), pt10(140, 190), a));
    // With the stale fill the track would be a short; DRC skips yielding zones.
    let stale = check(&board, &DrcConfig::default());
    assert!(stale.violations.iter().all(|v| v.b != Some(ItemRef::Zone(0))));
    let after = refilled(&board);
    assert!(!after.zones[0].yields);
    assert_eq!(pieces(&after), 2, "left and right of the track");
    let rep = check(&after, &DrcConfig::default());
    assert!(rep.is_clean(), "{:#?}", rep.violations);
    // Both halves still carry a GND pad, so nothing was dropped as an island.
    assert!(area(&after) < area(&board));
    assert!(area(&after) > area(&board) / 2);
}

#[test]
fn the_pour_keeps_the_required_clearance_to_every_other_net_copper() {
    let (mut board, _, a) = board();
    board.rules.classes[0].clearance = 350_000; // larger than the zone's own 0.2 mm
    board.tracks.push(track(pt10(100, 100), pt10(180, 160), a));
    let after = refilled(&board);
    let rep = check(&after, &DrcConfig::default());
    assert!(rep.is_clean(), "{:#?}", rep.violations);
}

#[test]
fn a_cut_off_piece_without_a_same_net_item_is_removed() {
    let (mut board, _, a) = board();
    // Cuts a 1.7 mm wide strip at the right end that holds no GND pad.
    board.tracks.push(track(pt10(260, 110), pt10(260, 190), a));
    let after = refilled(&board);
    assert_eq!(pieces(&after), 1);
    let bb = regions_of(&after.zones[0].fills[0].polys)[0].bbox().unwrap();
    assert!(bb.max.x < mm(26), "{bb:?}");
    assert!(check(&after, &DrcConfig::default()).is_clean());
}

#[test]
fn island_modes_never_and_below_area_keep_the_cut_off_piece() {
    let (mut board, _, a) = board();
    board.zones[0].islands = IslandRemoval::Never;
    board.tracks.push(track(pt10(260, 110), pt10(260, 190), a));
    assert_eq!(pieces(&refilled(&board)), 2);
    board.zones[0].islands = IslandRemoval::BelowArea(5_000_000_000_000); // 5 mm²
    assert_eq!(pieces(&refilled(&board)), 2, "the strip is ~10 mm²");
    board.zones[0].islands = IslandRemoval::BelowArea(20_000_000_000_000);
    assert_eq!(pieces(&refilled(&board)), 1);
}

#[test]
fn slivers_thinner_than_min_thickness_are_dropped() {
    let (mut board, _, a) = board();
    board.zones[0].islands = IslandRemoval::Never;
    // Fill edge y = 12 mm; track edge 0.2 mm + 0.125 mm below it leaves a 0.1 mm strip.
    board.tracks.push(track(Point::new(mm(5), 12_425_000), Point::new(mm(25), 12_425_000), a));
    let after = refilled(&board);
    for r in regions_of(&after.zones[0].fills[0].polys) {
        let bb = r.bbox().unwrap();
        assert!(bb.max.y - bb.min.y > 250_000, "{bb:?}");
    }
    assert!(check(&after, &DrcConfig::default()).is_clean());
}

#[test]
fn nothing_to_refill_returns_no_edit_and_refill_is_deterministic() {
    let (mut board, _, a) = board();
    // Far from the pour (R3 area): nothing to refill.
    board.tracks.push(track(pt10(60, 20), pt10(140, 20), a));
    let r = refill_zones(&board);
    assert!(r.fills.is_empty(), "{:?}", r.notes);
    board.tracks.push(track(pt10(100, 100), pt10(180, 160), a));
    let (x, y) = (refill_zones(&board), refill_zones(&board));
    assert_eq!(x.fills.len(), 1);
    assert_eq!(x.fills, y.fills);
}
