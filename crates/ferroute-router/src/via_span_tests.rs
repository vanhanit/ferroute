// SPDX-License-Identifier: AGPL-3.0-only
//! Blind, buried and micro vias end to end: the opt-in policy, the spans of the emitted
//! vias, and exact DRC cleanliness.

use super::*;
use ferroute_board::builder::{BoardBuilder, mm, pt10};
use ferroute_board::{LayerId, LayerSet, Origin, ViaKind, ViaPolicy};
use ferroute_drc::{DrcConfig, check};

fn cfg(policy: ViaPolicy) -> RouterConfig {
    RouterConfig { accel: Accel::None, via_policy: policy, ..RouterConfig::default() }
}

fn policy(names: &[&str]) -> ViaPolicy {
    ViaPolicy::from_names(names.iter().copied()).expect("kinds")
}

/// Four layers; a fixed F.Cu wall of net W splits the board, a keepout forbids vias on
/// In2.Cu / B.Cu (so no through via exists anywhere), and net A has a pad on each side.
fn wall_board() -> Board {
    let mut b = BoardBuilder::new(mm(30), mm(20), 4);
    let a = b.net("A");
    let w = b.net("W");
    let l = b.net("L");
    let r = b.net("R");
    b.smd2("R1", pt10(80, 100), 0.0, a, l);
    b.smd2("R2", pt10(220, 100), 0.0, a, r);
    b.track(pt10(150, 0), pt10(150, 200), 250_000, 0, w, Origin::Existing);
    b.keepout(ferroute_geom::Rect::new(pt10(0, 0), pt10(300, 200)), false, true);
    let mut board = b.build();
    board.keepouts[0].layers = LayerSet::EMPTY.with(LayerId(2)).with(LayerId(3));
    board
}

fn assert_clean(board: &Board, out: &RouteOutcome) {
    let mut after = board.clone();
    after.apply(&out.edit);
    let rep = check(&after, &DrcConfig::default());
    let introduced: Vec<_> = rep.introduced().collect();
    assert!(introduced.is_empty(), "introduced violations: {introduced:#?}");
    assert_eq!(out.stats.drc_gate_rips, 0);
}

#[test]
fn through_only_cannot_cross_the_wall() {
    let board = wall_board();
    let out = route(&board, &cfg(ViaPolicy::default())).expect("route");
    assert!(!out.failed.is_empty(), "no through via is legal anywhere: {:?}", out.stats);
    assert!(out.edit.new_vias.is_empty());
}

#[test]
fn blind_vias_cross_the_wall_on_the_inner_layer() {
    let board = wall_board();
    let out = route(&board, &cfg(policy(&["through", "blind"]))).expect("route");
    assert!(out.failed.is_empty(), "{:?}", out.stats);
    assert_clean(&board, &out);
    assert!(out.edit.new_vias.len() >= 2);
    let f_in1 = LayerSet::span(LayerId(0), LayerId(1));
    for v in &out.edit.new_vias {
        assert_eq!(v.kind, ViaKind::Blind);
        assert_eq!(v.layers, f_in1, "a blind via spans F.Cu and In1.Cu only");
    }
    assert_eq!(out.stats.blind_vias, out.edit.new_vias.len());
    assert_eq!(out.stats.vias, out.edit.new_vias.len());
}

#[test]
fn micro_vias_use_the_class_microvia_sizes() {
    let board = wall_board();
    let out = route(&board, &cfg(policy(&["micro"]))).expect("route");
    assert!(out.failed.is_empty(), "{:?}", out.stats);
    assert_clean(&board, &out);
    let class = board.rules.class_of(&board, ferroute_board::NetId(1));
    for v in &out.edit.new_vias {
        assert_eq!(v.kind, ViaKind::Micro);
        assert_eq!((v.diameter, v.drill), (class.microvia_diameter, class.microvia_drill));
        assert_eq!(v.layers, LayerSet::span(LayerId(0), LayerId(1)));
    }
    assert_eq!(out.stats.micro_vias, out.edit.new_vias.len());
}

#[test]
fn micro_beats_blind_for_adjacent_layers_when_both_are_enabled() {
    let board = wall_board();
    let out = route(&board, &cfg(policy(&["blind", "micro"]))).expect("route");
    assert!(out.failed.is_empty());
    assert!(out.edit.new_vias.iter().all(|v| v.kind == ViaKind::Micro));
}

#[test]
fn spanned_routes_are_deterministic_and_thread_independent() {
    let board = wall_board();
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("pool");
        pool.install(|| route(&board, &cfg(policy(&["through", "blind", "micro"]))).expect("route"))
    };
    assert_eq!(run(1).edit, run(4).edit);
}

#[test]
fn default_policy_leaves_through_vias_untouched() {
    // Two layers, forced layer change: the result is a plain through via.
    let mut b = BoardBuilder::new(mm(24), mm(24), 2);
    let x = b.net("X");
    let y = b.net("Y");
    let g = b.net("G");
    b.smd2("R1", pt10(50, 50), 0.0, x, g);
    b.smd2("R2", pt10(190, 190), 0.0, g, x);
    b.smd2("R3", pt10(190, 50), 0.0, y, g);
    b.smd2("R4", pt10(50, 190), 0.0, g, y);
    let board = b.build();
    let plain = route(&board, &cfg(ViaPolicy::default())).expect("route");
    let all = route(&board, &cfg(policy(&["through", "blind", "buried", "micro"]))).expect("route");
    assert_eq!(plain.edit, all.edit, "two layers: spans cannot exist");
    assert!(plain.edit.new_vias.iter().all(|v| v.kind == ViaKind::Through));
}

#[test]
fn buried_via_passes_under_a_wall_on_two_layers() {
    // F.Cu and In1.Cu are walled off; only In2.Cu is free at the wall, reachable from the
    // pads through a micro via (F-In1) followed by a buried via (In1-In2).
    let mut b = BoardBuilder::new(mm(30), mm(20), 4);
    let a = b.net("A");
    let w = b.net("W");
    let l = b.net("L");
    let r = b.net("R");
    b.smd2("R1", pt10(80, 100), 0.0, a, l);
    b.smd2("R2", pt10(220, 100), 0.0, a, r);
    b.track(pt10(150, 0), pt10(150, 200), 250_000, 0, w, Origin::Existing);
    b.track(pt10(150, 0), pt10(150, 200), 250_000, 1, w, Origin::Existing);
    b.keepout(ferroute_geom::Rect::new(pt10(0, 0), pt10(300, 200)), false, true);
    let mut board = b.build();
    board.keepouts[0].layers = LayerSet::EMPTY.with(LayerId(3)); // no through vias
    let out = route(&board, &cfg(policy(&["micro", "buried"]))).expect("route");
    assert!(out.failed.is_empty(), "{:?}", out.stats);
    assert_clean(&board, &out);
    let buried: Vec<_> = out.edit.new_vias.iter().filter(|v| v.kind == ViaKind::Buried).collect();
    assert!(buried.len() >= 2, "{:?}", out.edit.new_vias);
    for v in buried {
        assert_eq!(v.layers, LayerSet::span(LayerId(1), LayerId(2)));
    }
    assert_eq!(out.stats.buried_vias + out.stats.micro_vias, out.edit.new_vias.len());
}
