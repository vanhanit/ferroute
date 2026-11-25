// SPDX-License-Identifier: AGPL-3.0-only
//! DRC behaviour under KiCad custom rules (`.kicad_dru`).

use super::*;
use ferroute_board::builder::{BoardBuilder, mm, pt10};
use ferroute_board::custom::{
    CmpOp, Cond, ConstraintKind, CustomRule, Field, Limits, RuleSide, Severity,
};
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
        severity: Severity::Error,
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

fn limits_rule(cond: Cond, kind: ConstraintKind, l: Limits, severity: Severity) -> CustomRule {
    CustomRule {
        name: "r".into(),
        condition: cond,
        layer: None,
        constraints: vec![(kind, l)],
        severity,
    }
}

fn max_of(v: i64) -> Limits {
    Limits { max: Some(v), ..Limits::default() }
}

fn min_of(v: i64) -> Limits {
    Limits { min: Some(v), ..Limits::default() }
}

fn any_rule(k: ConstraintKind, l: Limits) -> CustomRule {
    limits_rule(Cond::True, k, l, Severity::Error)
}

fn via(pos: Point, diameter: i64, drill: i64, net: NetId) -> Via {
    Via {
        pos,
        diameter,
        drill,
        kind: ferroute_board::ViaKind::Through,
        layers: LayerSet::all(2),
        net,
        locked: false,
        origin: Origin::Routed,
    }
}

#[test]
fn track_width_max_is_enforced_and_opt_is_informational() {
    let (mut board, a, _) = two_resistors();
    board.tracks.push(track(pt10(92, 50), pt10(92, 150), a)); // 0.25 mm
    let k = ConstraintKind::TrackWidth;
    board.rules.custom.push(any_rule(k, max_of(250_000)));
    assert!(check(&board, &DrcConfig::default()).is_clean(), "equal to max passes");
    board.rules.custom[0].constraints[0].1.max = Some(249_999);
    let r = check(&board, &DrcConfig::default());
    let v: Vec<_> =
        r.violations.iter().filter(|v| v.kind == ViolationKind::TrackWidthMax).collect();
    assert_eq!(v.len(), 1, "{:?}", r.violations);
    assert_eq!((v[0].required, v[0].severity), (249_999, Severity::Error));
    assert!(v[0].introduced);
    assert!(!r.is_clean());
    // `opt` alone never produces a violation.
    board.rules.custom[0].constraints[0].1 = Limits { opt: Some(100_000), ..Limits::default() };
    assert!(check(&board, &DrcConfig::default()).violations.is_empty());
}

#[test]
fn via_and_hole_max_and_annular_max() {
    let (mut board, a, _) = two_resistors();
    board.vias.push(via(pt10(150, 100), 800_000, 400_000, a)); // ring 0.2 mm
    board.rules.custom = vec![
        any_rule(ConstraintKind::ViaDiameter, max_of(700_000)),
        any_rule(ConstraintKind::HoleSize, max_of(300_000)),
        any_rule(ConstraintKind::AnnularWidth, max_of(150_000)),
    ];
    let r = check(&board, &DrcConfig::default());
    for k in
        [ViolationKind::ViaDiameterMax, ViolationKind::ViaDrillMax, ViolationKind::AnnularWidthMax]
    {
        assert_eq!(r.count(k), 1, "{k:?}: {:?}", r.violations);
    }
    board.rules.custom = vec![
        any_rule(ConstraintKind::ViaDiameter, max_of(800_000)),
        any_rule(ConstraintKind::HoleSize, max_of(400_000)),
        any_rule(ConstraintKind::AnnularWidth, max_of(200_000)),
    ];
    assert!(check(&board, &DrcConfig::default()).is_clean());
}

#[test]
fn hole_to_hole_rule_is_exact_and_per_pair() {
    let (mut board, a, bn) = two_resistors();
    // Two vias, drills 0.3 mm, centres 0.7 mm apart: hole gap 0.4 mm.
    board.vias.push(via(pt10(150, 100), 600_000, 300_000, a));
    board.vias.push(via(pt10(157, 100), 600_000, 300_000, bn));
    board.rules.hole_to_hole = 100_000;
    let n_h2h = |b: &Board| check(b, &DrcConfig::default()).count(ViolationKind::HoleToHole);
    assert_eq!(n_h2h(&board), 0);
    board.rules.custom = vec![any_rule(ConstraintKind::HoleToHole, min_of(400_000))];
    assert_eq!(n_h2h(&board), 0, "exactly at the limit passes");
    board.rules.custom[0].constraints[0].1 = min_of(400_001);
    assert_eq!(n_h2h(&board), 1);
    board.rules.custom[0].condition = name_is(RuleSide::A, "nope");
    assert_eq!(n_h2h(&board), 0, "a non-matching pair rule leaves the default");
}

#[test]
fn pad_hole_size_and_annular_rules() {
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let n = b.net("N");
    b.header("J1", pt10(100, 100), 0.0, &[n]); // 1.7 mm square pad, 1.0 mm drill: ring 0.35
    let mut board = b.build();
    let ck = |b: &Board, k| check(b, &DrcConfig::default()).count(k);
    board.rules.custom = vec![any_rule(ConstraintKind::AnnularWidth, min_of(350_000))];
    assert_eq!(ck(&board, ViolationKind::AnnularWidth), 0, "exact ring passes");
    board.rules.custom = vec![any_rule(ConstraintKind::AnnularWidth, min_of(350_001))];
    assert_eq!(ck(&board, ViolationKind::AnnularWidth), 1);
    board.rules.custom = vec![any_rule(ConstraintKind::AnnularWidth, max_of(349_999))];
    assert_eq!(ck(&board, ViolationKind::AnnularWidthMax), 1);
    board.rules.custom = vec![any_rule(ConstraintKind::HoleSize, min_of(1_100_000))];
    assert_eq!(ck(&board, ViolationKind::PadDrill), 1);
    board.rules.custom = vec![any_rule(ConstraintKind::HoleSize, max_of(900_000))];
    assert_eq!(ck(&board, ViolationKind::PadDrillMax), 1);
}

#[test]
fn severity_warning_is_reported_but_not_an_error() {
    let (mut board, ..) = routed();
    let k = ConstraintKind::Clearance;
    board.rules.custom =
        vec![limits_rule(Cond::True, k, min_of(mm(1) + 500_000), Severity::Warning)];
    let r = check(&board, &DrcConfig::default());
    assert!(r.count(ViolationKind::Clearance) > 0, "{:?}", r.violations);
    assert!(r.violations.iter().all(|v| v.severity == Severity::Warning));
    assert!(r.is_clean(), "warnings do not break cleanliness");
    assert_eq!(r.introduced().count(), 0, "warnings never trip the introduced gate");
    assert_eq!(r.warnings().count(), r.violations.len());
    for s in [Severity::Ignore, Severity::Exclusion] {
        board.rules.custom[0].severity = s;
        assert!(check(&board, &DrcConfig::default()).violations.is_empty(), "{s:?} suppresses");
    }
    board.rules.custom[0].severity = Severity::Error;
    let r = check(&board, &DrcConfig::default());
    assert!(!r.is_clean());
    assert!(r.introduced().count() > 0);
}

#[test]
fn maybe_matching_error_rule_keeps_violations_hard() {
    let (mut board, ..) = routed();
    let k = ConstraintKind::Clearance;
    let req = mm(1) + 500_000;
    board.rules.custom = vec![
        limits_rule(Cond::True, k, min_of(req), Severity::Warning),
        limits_rule(Cond::Unsupported("A.insideArea('X')".into()), k, min_of(req), Severity::Error),
    ];
    let r = check(&board, &DrcConfig::default());
    assert!(!r.is_clean());
    assert!(r.violations.iter().all(|v| v.severity == Severity::Error));
}

#[test]
fn member_of_footprint_selects_pads_by_owner_reference() {
    let (mut board, ..) = routed();
    board.rules.custom.push(rule(
        Cond::MemberOfFootprint { side: RuleSide::A, pattern: "R2".into() },
        ConstraintKind::Clearance,
        mm(5),
    ));
    let r = check(&board, &DrcConfig::default());
    let owner = |i: ItemRef| match i {
        ItemRef::Pad(p) => Some(board.footprint(board.pad(p).footprint).reference.clone()),
        _ => None,
    };
    let hits: Vec<_> = r.violations.iter().filter(|v| v.kind == ViolationKind::Clearance).collect();
    assert!(!hits.is_empty(), "{:#?}", r.violations);
    for v in hits {
        let owners = [owner(v.a), v.b.and_then(owner)];
        assert!(owners.iter().flatten().all(|o| o == "R2"), "{v:?}");
        assert!(owners.iter().any(Option::is_some), "{v:?}");
    }
    // No footprint matches: the rule never applies (tracks are never footprint members).
    board.rules.custom[0].condition =
        Cond::MemberOfFootprint { side: RuleSide::A, pattern: "U*".into() };
    assert!(check(&board, &DrcConfig::default()).is_clean());
}

#[test]
fn pad_over_a_hole_is_not_flagged_when_the_hole_clearance_rule_is_zero() {
    // KiCad tests a pad's copper against another item's hole only for a positive hole
    // clearance (`doTestHole` is guarded by `clearance > 0`).
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let (a, bn) = (b.net("A"), b.net("B"));
    b.header("H1", pt10(100, 100), 0.0, &[a]);
    b.smd2("R1", Point::new(10_800_000, 10_000_000), 0.0, bn, bn);
    let mut board = b.build();
    let holes =
        |board: &Board| check(board, &DrcConfig::default()).count(ViolationKind::HoleClearance);
    assert!(holes(&board) > 0);
    board.rules.custom.push(rule(Cond::True, ConstraintKind::HoleClearance, 0));
    assert_eq!(holes(&board), 0);
}
