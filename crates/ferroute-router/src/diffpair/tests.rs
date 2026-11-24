// SPDX-License-Identifier: AGPL-3.0-only
use super::*;
use crate::{Accel, RouterConfig, route};
use ferroute_board::builder::{BoardBuilder, PadSpec, mm};
use ferroute_board::{Mobility, PadShape, Side};
use ferroute_drc::quality;

/// Header (GND, D+, D-) → SOIC-like part 12 mm lower, behind a keep-out; pair rule 0.2/0.25.
fn board(wall: bool) -> Board {
    let mut b = BoardBuilder::new(mm(50), mm(34), 2);
    let n: Vec<NetId> = ["GND", "D+", "D-", "VCC"].iter().map(|x| b.net(x)).collect();
    let j1 = b.header("J1", Point::new(mm(5), mm(8)), 0.0, &[n[0], n[1], n[2]]);
    b.mobility(j1, Mobility::Locked);
    let pad = |i: usize, x: i64, y: i64, net: NetId| PadSpec {
        number: i.to_string(),
        local_pos: Point::new(x, y),
        shape: PadShape::Rect { w: 1_550_000, h: 600_000 },
        drill: None,
        net,
    };
    let rows = [-1_905_000, -635_000, 635_000, 1_905_000];
    let nets = [n[3], n[1], n[2], n[0]];
    let pads: Vec<PadSpec> = rows
        .iter()
        .zip(nets)
        .enumerate()
        .map(|(i, (&y, net))| pad(i + 1, -2_700_000, y, net))
        .collect();
    b.footprint("U1", Point::new(mm(38), mm(24)), 0.0, Side::Front, &pads);
    if wall {
        // Full-height wall at x = 20..21 with two 0.72 mm slots 4 mm apart (a single trace
        // fits through a slot, the pair does not).
        let slot = |y: i64| (y * 100_000, y * 100_000 + 720_000);
        let (s1, s2) = (slot(150), slot(190));
        for (a, c) in [(0, s1.0), (s1.1, s2.0), (s2.1, mm(34))] {
            let r = ferroute_geom::Rect::new(Point::new(mm(20), a), Point::new(mm(21), c));
            b.keepout(r, true, true);
        }
    } else {
        let r = ferroute_geom::Rect::new(Point::new(mm(16), mm(5)), Point::new(mm(24), mm(17)));
        b.keepout(r, true, true);
    }
    let mut board = b.build();
    board.diff_pairs.push(DiffPair {
        p: n[1],
        n: n[2],
        gap: 200_000,
        width: Some(250_000),
        max_uncoupled: Some(mm(6)),
        skew_max: Some(100_000),
    });
    board
}

fn cfg() -> RouterConfig {
    RouterConfig { accel: Accel::Cpu, ..RouterConfig::default() }
}

fn routed(board: &Board) -> (Board, crate::RouteOutcome) {
    let out = route(board, &cfg()).expect("route");
    let mut after = board.clone();
    after.apply(&out.edit);
    (after, out)
}

#[test]
fn pair_routes_coupled_with_constant_gap_and_clean_quality() {
    let b = board(false);
    let (after, out) = routed(&b);
    assert_eq!(out.stats.coupled_pairs, ["D+/D-"], "{:?}", out.stats.uncoupled_pairs);
    assert!(out.failed.is_empty());
    assert_eq!(out.stats.drc_gate_rips, 0);
    let rep = ferroute_drc::check(&after, &DrcConfig::default());
    assert!(rep.introduced().next().is_none(), "{:?}", rep.violations);
    let pair = &b.diff_pairs[0];
    let m = ferroute_drc::pairs::pair_metrics(&after, pair).expect("metrics");
    let (lo, hi) = (m.min_gap.expect("coupled"), m.max_gap.expect("coupled"));
    assert!(lo >= 200_000 - 2 && hi <= 220_000, "gap {lo}..{hi}");
    assert!(m.coupled > mm(20), "{m:?}");
    assert!(m.uncoupled() <= mm(6), "{m:?}");
    // Equal length within the skew (the P/N fans are symmetric, S-bends cancel).
    let len = ferroute_drc::rules::net_lengths(&after);
    assert!((len[pair.p.idx()] - len[pair.n.idx()]).abs() <= pair.skew(), "{len:?}");
    for t in after.tracks.iter().filter(|t| pair.contains(t.net)) {
        assert!((t.end - t.start).is_octilinear());
        assert_eq!(t.width, 250_000);
    }
    assert!(quality::acute_angles(&after).is_empty());
    assert!(quality::slivers(&after).is_empty());
    assert!(quality::right_angles(&after).is_empty());
    let issues = ferroute_drc::rules::check_constraints(&after);
    assert!(issues.iter().all(|i| !i.kind.label().starts_with("pair")), "{issues:?}");
}

#[test]
fn pair_stage_is_deterministic() {
    let b = board(false);
    let layers = [LayerId(0), LayerId(1)];
    let a = route_pairs(&b, &layers, 450_000, None);
    assert_eq!(a, route_pairs(&b, &layers, 450_000, None));
    assert_eq!(a.coupled.len(), 1);
    assert!(a.tracks().iter().all(|t| t.origin == Origin::Routed));
}

#[test]
fn pair_that_cannot_squeeze_through_is_reported_and_routed_apart() {
    let b = board(true);
    let (after, out) = routed(&b);
    assert!(out.stats.coupled_pairs.is_empty());
    assert_eq!(out.stats.uncoupled_pairs.len(), 1, "{:?}", out.stats);
    assert!(out.stats.uncoupled_pairs[0].starts_with("D+/D-: "));
    assert!(out.failed.is_empty(), "both nets still route independently: {:?}", out.failed);
    let rep = ferroute_drc::check(&after, &DrcConfig::default());
    assert!(rep.introduced().next().is_none());
    let issues = ferroute_drc::rules::check_constraints(&after);
    assert!(issues.iter().any(|i| i.kind.label() == "pair-uncoupled"), "{issues:?}");
}

#[test]
fn pairs_outside_the_requested_nets_are_skipped() {
    let b = board(false);
    let only = [b.diff_pairs[0].p];
    let st = route_pairs(&b, &[LayerId(0)], 450_000, Some(&only));
    assert!(st.coupled.is_empty() && st.uncoupled.is_empty());
}
