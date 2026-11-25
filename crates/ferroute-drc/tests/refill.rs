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

#[test]
fn pour_splits_name_the_wall_that_separates_two_pad_groups() {
    let (mut board, gnd, a) = board();
    board.tracks.push(track(pt10(140, 110), pt10(140, 190), a));
    let r = refill_zones(&board);
    let splits = ferroute_drc::refill::pour_splits(&board, &r);
    assert_eq!(splits.len(), 1);
    let s = &splits[0];
    assert_eq!((s.net, s.groups), (gnd, 2));
    assert_eq!(s.cutters.len(), 1);
    assert_eq!((s.cutters[0].item, s.cutters[0].net), (ItemRef::Track(0), a));
    assert!(s.cutters[0].introduced);
    assert_eq!(s.movable().count(), 1);
}

#[test]
fn a_track_that_leaves_a_channel_does_not_split_the_pour() {
    let (mut board, _, a) = board();
    // Stops short of the pour edge: the pour wraps around its end.
    board.tracks.push(track(pt10(140, 110), pt10(140, 160), a));
    let r = refill_zones(&board);
    assert!(ferroute_drc::refill::pour_splits(&board, &r).is_empty());
}

#[test]
fn pour_splits_ignore_pours_the_refill_did_not_change() {
    let (board, _, _) = board();
    let r = refill_zones(&board);
    assert!(ferroute_drc::refill::pour_splits(&board, &r).is_empty());
}

#[test]
fn pre_existing_copper_that_walls_the_pour_is_reported_as_fixed() {
    let (mut board, _, a) = board();
    let mut t = track(pt10(140, 110), pt10(140, 190), a);
    t.origin = Origin::Existing;
    board.tracks.push(t);
    let r = refill_zones(&board);
    let splits = ferroute_drc::refill::pour_splits(&board, &r);
    assert_eq!(splits.len(), 1);
    assert_eq!(splits[0].cutters.len(), 1);
    assert_eq!(splits[0].movable().count(), 0, "existing copper cannot be re-routed");
}

#[test]
fn a_neck_narrower_than_min_thickness_is_removed_from_the_fill() {
    use ferroute_geom::Point;
    let (mut board, _, a) = board();
    let b2 =
        board.nets.iter().position(|n| n.name == "B").map(|i| NetId(u32::try_from(i).unwrap()));
    let other = b2.unwrap_or(a);
    // Two parallel 3 mm tracks (0.25 wide, 0.2 clearance): the strip between them is
    // 0.8 - 0.25 - 0.4 = 0.15 mm wide, below the 0.25 mm min_thickness of the zone.
    board.tracks.push(track(pt10(120, 150), pt10(150, 150), a));
    board.tracks.push(track(pt10(120, 158), pt10(150, 158), other));
    let after = refilled(&board);
    let mid = pt10(135, 154);
    let covered = after.zones[0].fills[0].polys.iter().any(|p| p.contains(mid));
    assert!(!covered, "the 0.15 mm neck must be opened away");
    let _ = Point::ORIGIN;
}

#[test]
fn a_neck_between_two_vias_is_removed_from_the_fill() {
    use ferroute_board::{Via, ViaKind};
    let (mut board, _, a) = board();
    for x in [10, 11, 12] {
        board.vias.push(Via {
            pos: Point::new(x * 1_000_000, 15_000_000),
            diameter: 450_000,
            drill: 200_000,
            layers: LayerSet::single(LayerId(0)),
            kind: ViaKind::Through,
            net: a,
            locked: false,
            origin: Origin::Routed,
        });
    }
    let after = refilled(&board);
    let at =
        |x: i64, y: i64| after.zones[0].fills[0].polys.iter().any(|p| p.contains(Point::new(x, y)));
    assert!(at(10_500_000, 14_000_000), "open copper above the row stays");
    assert!(!at(10_500_000, 15_000_000), "the 0.15 mm neck must be opened away");
}
