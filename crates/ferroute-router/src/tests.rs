// SPDX-License-Identifier: AGPL-3.0-only
use super::*;
use ferroute_board::builder::{BoardBuilder, mm, pt10};
use ferroute_board::{Mobility, NetClass};
use ferroute_drc::{DrcConfig, check};
use proptest::prelude::*;

fn exact_cfg() -> RouterConfig {
    RouterConfig { accel: Accel::None, ..RouterConfig::default() }
}

fn assert_clean_and_complete(board: &Board, out: &RouteOutcome, expect_complete: bool) {
    out.edit.check_invariants(board).expect("INV-EDIT-1");
    let mut after = board.clone();
    after.apply(&out.edit);
    let rep = check(&after, &DrcConfig::default());
    let introduced: Vec<_> = rep.introduced().collect();
    assert!(introduced.is_empty(), "introduced violations: {introduced:#?}");
    assert_eq!(out.stats.drc_gate_rips, 0, "DRC gate had to intervene: {:?}", out.stats);
    if expect_complete {
        assert_eq!(
            rep.connectivity.connections_missing, 0,
            "unrouted: {:?} stats {:?}",
            rep.connectivity.unconnected, out.stats
        );
    }
}

#[test]
fn routes_two_resistors() {
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let a = b.net("A");
    let n = b.net("B");
    b.smd2("R1", pt10(100, 50), 0.0, a, n);
    b.smd2("R2", pt10(200, 150), 90.0, n, a);
    let board = b.build();
    let out = route(&board, &exact_cfg()).expect("route");
    assert_eq!(out.stats.nets_routed, 2, "{:?}", out.stats);
    assert!(out.failed.is_empty());
    assert_clean_and_complete(&board, &out, true);
}

#[test]
fn crossing_nets_use_vias_on_two_layers() {
    // Four parts in a square with nets crossing diagonally: on one layer the two nets
    // would have to cross, so a complete solution needs the second layer.
    let mut b = BoardBuilder::new(mm(24), mm(24), 2);
    let x = b.net("X");
    let y = b.net("Y");
    let g = b.net("G");
    // A fence of GND parts makes the single-layer detour impossible.
    b.smd2("R1", pt10(50, 50), 0.0, x, g);
    b.smd2("R2", pt10(190, 190), 0.0, g, x);
    b.smd2("R3", pt10(190, 50), 0.0, y, g);
    b.smd2("R4", pt10(50, 190), 0.0, g, y);
    let board = b.build();
    let out = route(&board, &exact_cfg()).expect("route");
    assert_clean_and_complete(&board, &out, true);
}

#[test]
fn header_and_fat_power_class() {
    let mut b = BoardBuilder::new(mm(40), mm(30), 2);
    let vcc = b.net("VCC");
    let gnd = b.net("GND");
    let s1 = b.net("S1");
    b.rules_mut().classes.push(NetClass {
        name: "Power".into(),
        clearance: 300_000,
        track_width: 500_000,
        via_diameter: 800_000,
        via_drill: 400_000,
    });
    b.header("J1", pt10(50, 80), 0.0, &[vcc, gnd, s1]);
    b.smd2("C1", pt10(250, 100), 0.0, vcc, gnd);
    b.smd2("R1", pt10(250, 200), 0.0, s1, gnd);
    let mut board = b.build();
    board.nets[vcc.idx()].class = 1;
    board.nets[gnd.idx()].class = 1;
    let out = route(&board, &exact_cfg()).expect("route");
    assert_eq!(out.stats.reservation[1].0, 1, "power class reserves one ring");
    assert_clean_and_complete(&board, &out, true);
}

#[test]
fn deterministic_across_thread_counts_and_accel() {
    let board = random_board(7, 10);
    let run = |threads: usize, accel: Accel| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("pool");
        pool.install(|| {
            route(&board, &RouterConfig { accel, ..RouterConfig::default() }).expect("route")
        })
    };
    let a = run(1, Accel::None);
    let b = run(4, Accel::None);
    let c = run(3, Accel::Cpu);
    assert_eq!(a.edit, b.edit, "thread count changed the result");
    assert_eq!(a.edit, c.edit, "accelerator changed the result");
    assert_clean_and_complete(&board, &a, false);
}

#[test]
fn accel_legality_equals_exact() {
    let board = random_board(3, 12);
    let pitch = default_pitch(&board.rules);
    let plan = Plan::new(&board.rules, pitch);
    let grid = Grid::covering(
        board.outline.bbox().expect("outline"),
        pitch,
        vec![ferroute_board::LayerId(0), ferroute_board::LayerId(1)],
    );
    let obs = Obstacles::from_board(&board);
    let (accel, name) = build_accel(&grid, &obs, &plan, Accel::Cpu);
    assert!(accel.is_some(), "{name}");
    for cp in &plan.classes {
        let exact = legality::compute(&grid, &obs, &cp.geom, None);
        let fast = legality::compute(&grid, &obs, &cp.geom, accel.as_ref());
        assert_eq!(exact, fast);
    }
}

#[test]
fn locked_parts_are_never_moved_by_router() {
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let a = b.net("A");
    let f = b.smd2("R1", pt10(100, 100), 0.0, a, a);
    b.mobility(f, Mobility::Locked);
    let board = b.build();
    let out = route(&board, &exact_cfg()).expect("route");
    assert!(out.edit.moves.is_empty());
}

/// Random board: `n` two-pin parts on a coarse placement grid with random nets.
fn random_board(seed: u64, n: usize) -> Board {
    let mut s = seed;
    let mut next = |m: u64| {
        s = splitmix64(s);
        s % m
    };
    let mut b = BoardBuilder::new(mm(40), mm(30), 2);
    let nets: Vec<_> = (0..(n / 2).max(2)).map(|i| b.net(&format!("N{i}"))).collect();
    let mut used = std::collections::BTreeSet::new();
    let mut k = 0;
    while k < n {
        let (cx, cy) = (next(7), next(5));
        if !used.insert((cx, cy)) {
            continue;
        }
        let pos = pt10(50 + i64::try_from(cx).unwrap() * 50, 50 + i64::try_from(cy).unwrap() * 50);
        let rot = if next(2) == 0 { 0.0 } else { 90.0 };
        let (na, nb) = (
            nets[usize::try_from(next(nets.len() as u64)).unwrap()],
            nets[usize::try_from(next(nets.len() as u64)).unwrap()],
        );
        b.smd2(&format!("R{k}"), pos, rot, na, nb);
        k += 1;
    }
    b.build()
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: std::env::var("FERROUTE_PROP_CASES").ok().and_then(|v| v.parse().ok()).unwrap_or(24),
        ..ProptestConfig::default()
    })]

    /// THE router invariant: whatever the board, routed copper never introduces a DRC
    /// violation, and the DRC gate never has to intervene.
    #[test]
    fn routing_never_introduces_violations(seed in 0u64..1_000_000, n in 2usize..30) {
        let board = random_board(seed, n);
        let out = route(&board, &RouterConfig { accel: Accel::Cpu, max_iterations: 30, ..RouterConfig::default() }).expect("route");
        let mut after = board;
        after.apply(&out.edit);
        let rep = check(&after, &DrcConfig::default());
        let introduced: Vec<_> = rep.introduced().cloned().collect();
        prop_assert!(introduced.is_empty(), "seed {} n {}: {:#?}", seed, n, introduced);
        prop_assert_eq!(out.stats.drc_gate_rips, 0);
    }
}

#[test]
#[ignore = "diagnostic"]
fn diag_header() {
    let mut b = BoardBuilder::new(mm(40), mm(30), 2);
    let vcc = b.net("VCC");
    let gnd = b.net("GND");
    let s1 = b.net("S1");
    b.rules_mut().classes.push(NetClass {
        name: "Power".into(),
        clearance: 300_000,
        track_width: 500_000,
        via_diameter: 800_000,
        via_drill: 400_000,
    });
    b.header("J1", pt10(50, 80), 0.0, &[vcc, gnd, s1]);
    b.smd2("C1", pt10(250, 100), 0.0, vcc, gnd);
    b.smd2("R1", pt10(250, 200), 0.0, s1, gnd);
    let mut board = b.build();
    board.nets[vcc.idx()].class = 1;
    board.nets[gnd.idx()].class = 1;
    let out = route(&board, &exact_cfg()).expect("route");
    let pitch = out.stats.pitch;
    for (i, j, l) in &out.stats.negotiation.overused_sites {
        eprintln!(
            "overused ({i},{j},{l}) at ({:.2},{:.2}) mm",
            f64::from(*i) * pitch as f64 / 1e6,
            f64::from(*j) * pitch as f64 / 1e6
        );
    }
    for t in &out.edit.new_tracks {
        eprintln!("{:?} {:?}->{:?} L{}", board.nets[t.net.idx()].name, t.start, t.end, t.layer.0);
    }
}
