// SPDX-License-Identifier: AGPL-3.0-only
//! DRC behaviour under KiCad custom rules (`.kicad_dru`).

use super::*;
use ferroute_board::builder::{BoardBuilder, mm, pt10};
use ferroute_board::custom::{CmpOp, Cond, ConstraintKind, CustomRule, Field, Limits, RuleSide};
use ferroute_board::{Board, Track, Via};
use ferroute_geom::Point;

fn two_resistors() -> (Board, NetId, NetId) {
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let a = b.net("A");
    let bn = b.net("B");
    b.smd2("R1", pt10(100, 50), 0.0, a, bn);
    b.smd2("R2", pt10(100, 150), 0.0, a, bn);
    (b.build(), a, bn)
}

fn track(s: Point, e: Point, net: NetId) -> Track {
    Track {
        start: s,
        end: e,
        width: 250_000,
        layer: LayerId(0),
        net,
        locked: false,
        origin: Origin::Routed,
    }
}

fn rule(cond: Cond, kind: ConstraintKind, min: i64) -> CustomRule {
    CustomRule {
        name: "r".into(),
        condition: cond,
        layer: None,
        constraints: vec![(kind, Limits { min: Some(min), ..Limits::default() })],
    }
}

fn name_is(side: RuleSide, v: &str) -> Cond {
    Cond::Cmp { side, field: Field::NetName, op: CmpOp::Eq, value: v.into() }
}

fn routed() -> (Board, NetId, NetId) {
    let (mut board, a, bn) = two_resistors();
    board.tracks.push(track(pt10(92, 50), pt10(92, 150), a));
    board.tracks.push(track(pt10(108, 50), pt10(108, 150), bn));
    (board, a, bn)
}

#[test]
fn pair_clearance_rule_is_enforced_exactly() {
    let (mut board, ..) = routed();
    assert!(check(&board, &DrcConfig::default()).is_clean());
    let req = mm(1) + 500_000; // gap is 1.35 mm
    board.rules.custom.push(rule(
        Cond::And(vec![name_is(RuleSide::A, "A"), name_is(RuleSide::B, "B")]),
        ConstraintKind::Clearance,
        req,
    ));
    let r = check(&board, &DrcConfig::default());
    assert!(
        r.violations.iter().any(|v| v.kind == ViolationKind::Clearance && v.required == req),
        "{:#?}",
        r.violations
    );
    // A later, lower rule overrides (last matching rule wins, KiCad semantics).
    board.rules.custom.push(rule(Cond::True, ConstraintKind::Clearance, 100_000));
    assert!(check(&board, &DrcConfig::default()).is_clean());
}

#[test]
fn unsupported_condition_only_raises() {
    let (mut board, ..) = routed();
    let u = |v| rule(Cond::Unsupported("A.insideArea('X')".into()), ConstraintKind::Clearance, v);
    board.rules.custom.push(u(mm(2)));
    assert!(!check(&board, &DrcConfig::default()).is_clean());
    board.rules.custom[0] = u(50_000);
    assert!(check(&board, &DrcConfig::default()).is_clean());
}

#[test]
fn width_and_annular_rules() {
    let (mut board, a, _) = two_resistors();
    board.tracks.push(track(pt10(92, 50), pt10(92, 150), a));
    let w = board.tracks[0].width;
    board.rules.custom.push(rule(name_is(RuleSide::A, "A"), ConstraintKind::TrackWidth, w + 1));
    let r = check(&board, &DrcConfig::default());
    assert!(
        r.violations.iter().any(|v| v.kind == ViolationKind::TrackWidth && v.required == w + 1)
    );

    board.rules.custom.clear();
    board.vias.push(Via {
        pos: pt10(150, 100),
        diameter: 600_000,
        drill: 300_000,
        kind: ferroute_board::ViaKind::Through,
        layers: LayerSet::all(2),
        net: a,
        locked: false,
        origin: Origin::Routed,
    });
    board.rules.custom.push(rule(Cond::True, ConstraintKind::AnnularWidth, 200_000));
    let r = check(&board, &DrcConfig::default());
    assert!(
        r.violations.iter().any(|v| v.kind == ViolationKind::AnnularWidth),
        "{:?}",
        r.violations
    );
    board.rules.custom[0].constraints[0].1.min = Some(150_000);
    let r = check(&board, &DrcConfig::default());
    assert!(!r.violations.iter().any(|v| v.kind == ViolationKind::AnnularWidth));
}

#[test]
fn edge_clearance_rule() {
    let (mut board, ..) = routed();
    board.rules.custom.push(rule(Cond::True, ConstraintKind::EdgeClearance, mm(8)));
    let r = check(&board, &DrcConfig::default());
    assert!(
        r.violations.iter().any(|v| v.kind == ViolationKind::EdgeClearance && v.required == mm(8))
    );
}

#[test]
fn boards_without_custom_rules_are_unchanged() {
    let (board, ..) = routed();
    let base = check(&board, &DrcConfig::default());
    let mut with_empty = board;
    with_empty.rules.custom = Vec::new();
    assert_eq!(check(&with_empty, &DrcConfig::default()).violations, base.violations);
}
