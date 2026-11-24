// SPDX-License-Identifier: AGPL-3.0-only
//! Vias with a partial span (blind, buried, micro): every check applies only on the layers
//! the via occupies.

use super::*;
use ferroute_board::builder::{BoardBuilder, mm, pt10};
use ferroute_board::{Board, LayerSet, NetId, Origin, ViaKind};
use ferroute_geom::Point;

fn four_layers() -> (BoardBuilder, NetId, NetId) {
    let mut b = BoardBuilder::new(mm(30), mm(20), 4);
    let n = b.net("N");
    let m = b.net("M");
    (b, n, m)
}

fn span(a: u8, z: u8) -> LayerSet {
    LayerSet::span(LayerId(a), LayerId(z))
}

fn set_via(board: &mut Board, kind: ViaKind, layers: LayerSet) {
    board.vias[0].kind = kind;
    board.vias[0].layers = layers;
}

/// Track of net `m` across the via at (10, 10) on `layer`.
fn under(b: &mut BoardBuilder, m: NetId, layer: u8) {
    b.track(pt10(60, 100), pt10(140, 100), 250_000, layer, m, Origin::Routed);
}

fn count(board: &Board, kind: ViolationKind) -> usize {
    check(board, &DrcConfig::default()).count(kind)
}

#[test]
fn foreign_track_under_a_blind_via_is_legal_on_a_layer_it_does_not_span() {
    for layer in [2, 3] {
        let (mut b, n, m) = four_layers();
        b.via(pt10(100, 100), 600_000, 300_000, n, Origin::Routed);
        under(&mut b, m, layer);
        let mut board = b.build();
        set_via(&mut board, ViaKind::Blind, span(0, 1));
        let r = check(&board, &DrcConfig::default());
        assert!(r.is_clean(), "layer {layer}: {:#?}", r.violations);
    }
}

#[test]
fn foreign_track_on_a_spanned_layer_is_a_short() {
    for layer in [0, 1] {
        let (mut b, n, m) = four_layers();
        b.via(pt10(100, 100), 600_000, 300_000, n, Origin::Routed);
        under(&mut b, m, layer);
        let mut board = b.build();
        set_via(&mut board, ViaKind::Blind, span(0, 1));
        assert_eq!(count(&board, ViolationKind::Short), 1, "layer {layer}");
    }
}

#[test]
fn through_via_hits_every_layer() {
    let (mut b, n, m) = four_layers();
    b.via(pt10(100, 100), 600_000, 300_000, n, Origin::Routed);
    under(&mut b, m, 3);
    assert_eq!(count(&b.build(), ViolationKind::Short), 1);
}

#[test]
fn hole_clearance_applies_only_to_copper_on_the_holes_layers() {
    // 0.7 mm centre distance: clear of the via copper (0.625 needed) but inside the
    // 0.5 mm hole clearance (0.775 needed).
    let hole_violations = |layer: u8, kind: ViaKind, layers: LayerSet| {
        let (mut b, n, m) = four_layers();
        b.via(pt10(100, 100), 600_000, 300_000, n, Origin::Routed);
        b.track(pt10(60, 107), pt10(140, 107), 250_000, layer, m, Origin::Routed);
        let mut board = b.build();
        board.rules.hole_clearance = 500_000;
        set_via(&mut board, kind, layers);
        let r = check(&board, &DrcConfig::default());
        assert_eq!(r.count(ViolationKind::Clearance) + r.count(ViolationKind::Short), 0);
        r.count(ViolationKind::HoleClearance)
    };
    assert_eq!(hole_violations(1, ViaKind::Blind, span(0, 1)), 1, "In1 is under the hole");
    assert_eq!(hole_violations(3, ViaKind::Blind, span(0, 1)), 0, "B.Cu is not");
    assert_eq!(hole_violations(3, ViaKind::Through, span(0, 3)), 1, "through: every layer");
}

#[test]
fn buried_via_ignores_the_outer_layers() {
    for layer in [0, 3] {
        let (mut b, n, m) = four_layers();
        b.via(pt10(100, 100), 600_000, 300_000, n, Origin::Routed);
        under(&mut b, m, layer);
        let mut board = b.build();
        set_via(&mut board, ViaKind::Buried, span(1, 2));
        let r = check(&board, &DrcConfig::default());
        assert!(r.is_clean(), "layer {layer}: {:#?}", r.violations);
    }
    let (mut b, n, m) = four_layers();
    b.via(pt10(100, 100), 600_000, 300_000, n, Origin::Routed);
    under(&mut b, m, 2);
    let mut board = b.build();
    set_via(&mut board, ViaKind::Buried, span(1, 2));
    assert_eq!(count(&board, ViolationKind::Short), 1);
}

#[test]
fn clearance_to_other_vias_needs_a_shared_layer() {
    // Two vias at the same spot: a blind F–In1 and a blind In2–B never touch.
    let (mut b, n, m) = four_layers();
    b.via(pt10(100, 100), 600_000, 300_000, n, Origin::Routed);
    b.via(pt10(100, 100), 600_000, 300_000, m, Origin::Routed);
    let mut board = b.build();
    set_via(&mut board, ViaKind::Blind, span(0, 1));
    board.vias[1].kind = ViaKind::Blind;
    board.vias[1].layers = span(2, 3);
    assert_eq!(count(&board, ViolationKind::Short), 0);
    // ... but their drills coincide, which hole-to-hole still reports.
    assert_eq!(count(&board, ViolationKind::HoleToHole), 1);
}

#[test]
fn same_net_track_only_connects_to_a_via_on_a_spanned_layer() {
    let connections_missing = |layer: u8| {
        let mut b = BoardBuilder::new(mm(30), mm(20), 4);
        let n = b.net("N");
        b.header("J1", pt10(100, 50), 0.0, &[n, n]);
        b.via(pt10(100, 63), 600_000, 300_000, n, Origin::Routed);
        let pin1 = pt10(100, 50);
        let pin2 = Point::new(pin1.x, pin1.y + 2_540_000);
        let via = pt10(100, 63);
        b.track(pin1, via, 250_000, 0, n, Origin::Routed);
        b.track(via, pin2, 250_000, layer, n, Origin::Routed);
        let mut board = b.build();
        set_via(&mut board, ViaKind::Blind, span(0, 1));
        check(&board, &DrcConfig::default()).connectivity.connections_missing
    };
    assert_eq!(connections_missing(1), 0, "In1 is spanned");
    assert_eq!(connections_missing(2), 1, "In2 is not");
}

#[test]
fn microvias_have_their_own_minimum_sizes() {
    let (mut b, n, _) = four_layers();
    b.via(pt10(100, 100), 300_000, 100_000, n, Origin::Routed);
    let mut board = b.build();
    // As a through via 0.3 / 0.1 is below the minimums...
    assert_eq!(count(&board, ViolationKind::ViaDiameter), 1);
    assert_eq!(count(&board, ViolationKind::ViaDrill), 1);
    // ... as a microvia it meets the microvia minimums (0.2 / 0.1).
    set_via(&mut board, ViaKind::Micro, span(0, 1));
    assert_eq!(count(&board, ViolationKind::ViaDiameter), 0);
    assert_eq!(count(&board, ViolationKind::ViaDrill), 0);
    board.vias[0].diameter = 150_000;
    assert_eq!(count(&board, ViolationKind::ViaDiameter), 1);
}

#[test]
fn spans_must_fit_the_via_kind() {
    let kinds = |kind: ViaKind, layers: LayerSet| {
        let (mut b, n, _) = four_layers();
        b.via(pt10(100, 100), 600_000, 300_000, n, Origin::Routed);
        let mut board = b.build();
        set_via(&mut board, kind, layers);
        count(&board, ViolationKind::ViaSpan)
    };
    assert_eq!(kinds(ViaKind::Micro, span(0, 1)), 0);
    assert_eq!(kinds(ViaKind::Micro, span(2, 3)), 0);
    assert_eq!(kinds(ViaKind::Micro, span(0, 2)), 1, "micro spans one layer pair");
    assert_eq!(kinds(ViaKind::Micro, span(1, 2)), 1, "micro starts at an outer layer");
    assert_eq!(kinds(ViaKind::Blind, span(0, 2)), 0);
    assert_eq!(kinds(ViaKind::Blind, span(1, 2)), 1, "blind needs an outer end");
    assert_eq!(kinds(ViaKind::Buried, span(1, 2)), 0);
    assert_eq!(kinds(ViaKind::Buried, span(0, 2)), 1, "buried has no outer end");
    let noncontiguous = LayerSet::EMPTY.with(LayerId(0)).with(LayerId(2));
    assert_eq!(kinds(ViaKind::Blind, noncontiguous), 1);
}
