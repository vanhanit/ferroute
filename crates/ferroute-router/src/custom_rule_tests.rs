// SPDX-License-Identifier: AGPL-3.0-only
//! Custom `.kicad_dru` hole-to-hole rules in the router (task #15b): the router must
//! keep the spacing the oracle will enforce, so the gate never has to rip.

use super::*;
use crate::legality::{BLOCKED, Obstacles};
use ferroute_board::Origin;
use ferroute_board::builder::{BoardBuilder, mm, pt10};
use ferroute_board::custom::{Cond, ConstraintKind, CustomRule, Limits, Severity};
use ferroute_drc::{DrcConfig, ViolationKind, check};
use ferroute_geom::Point;

fn h2h_rule(min: i64) -> CustomRule {
    CustomRule {
        name: "h2h".into(),
        condition: Cond::True,
        layer: None,
        constraints: vec![(
            ConstraintKind::HoleToHole,
            Limits { min: Some(min), ..Limits::default() },
        )],
        severity: Severity::Error,
    }
}

#[test]
fn via_classification_honours_the_custom_hole_to_hole() {
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let n = b.net("A");
    b.via(pt10(150, 100), 600_000, 300_000, n, Origin::Existing);
    let mut board = b.build();
    let cg = plan::class_geoms(&board.rules)[0];
    // Far enough for copper clearance and the global hole-to-hole, well inside 4 mm.
    let probe = Point::new(
        pt10(150, 100).x
            + cg.via_radius
            + 300_000
            + cg.clearance
            + board.rules.hole_to_hole
            + 1_000_000,
        pt10(150, 100).y,
    );
    let free = Obstacles::from_board(&board).classify_via(probe, &cg);
    assert_ne!(free, BLOCKED, "legal with the global hole-to-hole only");
    board.rules.custom = vec![h2h_rule(mm(4))];
    let blocked = Obstacles::from_board(&board).classify_via(probe, &cg);
    assert_eq!(blocked, BLOCKED, "custom hole_to_hole must block");
}

#[test]
fn routed_vias_respect_a_larger_custom_hole_to_hole() {
    let mut b = BoardBuilder::new(mm(24), mm(24), 2);
    let x = b.net("X");
    let y = b.net("Y");
    let g = b.net("G");
    b.smd2("R1", pt10(50, 50), 0.0, x, g);
    b.smd2("R2", pt10(190, 190), 0.0, g, x);
    b.smd2("R3", pt10(190, 50), 0.0, y, g);
    b.smd2("R4", pt10(50, 190), 0.0, g, y);
    let mut board = b.build();
    board.rules.custom = vec![h2h_rule(mm(2))];
    let cfg = RouterConfig { accel: Accel::None, ..RouterConfig::default() };
    let out = route(&board, &cfg).expect("route");
    let mut after = board;
    after.apply(&out.edit);
    let rep = check(&after, &DrcConfig::default());
    assert_eq!(rep.count(ViolationKind::HoleToHole), 0, "{:#?}", rep.violations);
    assert_eq!(out.stats.drc_gate_rips, 0, "gate had to intervene: {:?}", out.stats);
}
