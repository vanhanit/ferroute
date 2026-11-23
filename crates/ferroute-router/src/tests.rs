// SPDX-License-Identifier: AGPL-3.0-only
use super::*;
use ferroute_board::NetId;
use ferroute_board::builder::{BoardBuilder, mm, pt10};
use ferroute_board::{Mobility, NetClass};
use ferroute_drc::{DrcConfig, check};
use ferroute_geom::Dir8;
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
        let traps = ferroute_drc::quality::acute_angles(&after);
        prop_assert!(traps.is_empty(), "seed {} n {}: acid traps {:?}", seed, n, traps);
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

#[test]
fn tracks_terminate_at_pad_centres_and_stay_octilinear() {
    for seed in 0..16 {
        let board = random_board(seed, 12);
        let out = route(&board, &exact_cfg()).expect("route");
        let centres: Vec<(ferroute_board::NetId, ferroute_geom::Point)> =
            board.pad_ids().map(|p| (board.pad(p).net, board.pad_position(p))).collect();
        let pads_per_net = board.pads_by_net();
        for &(net, c) in &centres {
            if net == ferroute_board::NetId::NONE
                || pads_per_net[net.idx()].len() < 2
                || out.failed.contains(&net)
            {
                continue;
            }
            assert!(
                out.edit.new_tracks.iter().any(|t| t.net == net && (t.start == c || t.end == c)),
                "seed {seed}: no track of net {net:?} ends at pad centre {c:?}"
            );
        }
        // Pads are entered only through a side midpoint along a pad axis (rectangular
        // shapes) or along a 0/45/90-degree line through the centre (circular pads).
        for pid in board.pad_ids() {
            let c = board.pad_position(pid);
            let dirs = access::entry_dirs(&board, pid);
            for t in out.edit.new_tracks.iter().filter(|t| t.start == c || t.end == c) {
                let o = if t.start == c { t.end } else { t.start } - c;
                let d = (o.x.signum(), o.y.signum());
                assert!(
                    dirs.contains(&d) && (o.x == 0 || o.y == 0 || o.x.abs() == o.y.abs()),
                    "seed {seed}: pad {pid:?} entered along {o:?}, allowed {dirs:?}"
                );
            }
        }
        for t in &out.edit.new_tracks {
            // Every segment, including the final approach into a pad, is 0°/45°/90°.
            let d = t.end - t.start;
            let octi = d.x == 0 || d.y == 0 || d.x.abs() == d.y.abs();
            assert!(octi, "seed {seed}: non-octilinear segment {t:?}");
        }
    }
}

/// Directions (unit grid steps) of the routed tracks leaving `p` on each layer.
fn exits_at(out: &RouteOutcome, p: ferroute_geom::Point) -> Vec<(u8, Option<ferroute_geom::Dir8>)> {
    out.edit
        .new_tracks
        .iter()
        .filter_map(|t| {
            let other = if t.start == p {
                t.end
            } else if t.end == p {
                t.start
            } else {
                return None;
            };
            Some((t.layer.0, ferroute_geom::Dir8::from_vector(other - p)))
        })
        .collect()
}

#[test]
fn vias_sit_in_line_with_straight_traces() {
    // Two nets that must cross: the via of the crossing net should not introduce a bend.
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let x = b.net("X");
    let y = b.net("Y");
    b.smd2("R1", pt10(50, 100), 0.0, x, x);
    b.smd2("R2", pt10(250, 100), 0.0, x, x);
    b.smd2("R3", pt10(150, 30), 90.0, y, y);
    b.smd2("R4", pt10(150, 170), 90.0, y, y);
    // Join R1.2 -> R2.1 (X, horizontal) and R3.2 -> R4.1 (Y, vertical): they cross.
    let mut board = b.build();
    board.pads[0].net = NetId::NONE;
    board.pads[3].net = NetId::NONE;
    board.pads[4].net = NetId::NONE;
    board.pads[7].net = NetId::NONE;
    let out = route(&board, &exact_cfg()).expect("route");
    assert_clean_and_complete(&board, &out, true);
    assert!(!out.edit.new_vias.is_empty(), "crossing requires a via");
    for v in &out.edit.new_vias {
        let ex = exits_at(&out, v.pos);
        let straight = ex.iter().any(|&(la, da)| {
            ex.iter().any(|&(lb, db)| {
                la != lb && da.is_some() && db.is_some() && da.map(Dir8::opposite) == db
            })
        });
        assert!(straight, "via at {:?} bends: exits {ex:?}", v.pos);
    }
}

#[test]
fn no_short_kinks_next_to_pads() {
    for seed in 0..12 {
        let board = random_board(seed, 12);
        let out = route(&board, &exact_cfg()).expect("route");
        let centres: Vec<_> = board.pad_ids().map(|p| board.pad_position(p)).collect();
        let pitch = out.stats.pitch;
        for t in &out.edit.new_tracks {
            if centres.contains(&t.start) || centres.contains(&t.end) {
                continue;
            }
            #[allow(clippy::cast_precision_loss)]
            let min = pitch as f64 - 1.0;
            assert!(
                t.start.dist(t.end) >= min,
                "seed {seed}: kink segment {t:?} shorter than the pitch {pitch}"
            );
        }
    }
}

#[test]
fn routed_copper_has_no_acid_traps() {
    for seed in 0..16 {
        let board = random_board(seed, 14);
        let out = route(&board, &exact_cfg()).expect("route");
        let mut after = board;
        after.apply(&out.edit);
        let traps = ferroute_drc::quality::acute_angles(&after);
        assert!(traps.is_empty(), "seed {seed}: acid traps {traps:?}");
    }
}

#[test]
#[ignore = "diagnostic"]
fn diag_acid() {
    let board = random_board(
        std::env::var("SEED").ok().and_then(|v| v.parse().ok()).unwrap_or(1),
        std::env::var("N").ok().and_then(|v| v.parse().ok()).unwrap_or(14),
    );
    let out = route(&board, &exact_cfg()).expect("route");
    let mut after = board.clone();
    after.apply(&out.edit);
    for t in ferroute_drc::quality::acute_angles(&after) {
        eprintln!("TRAP {:?}", t.at);
        for tr in after.tracks.iter().filter(|x| x.net == t.net && x.layer == t.layer) {
            let s = ferroute_geom::Segment::new(tr.start, tr.end);
            if ferroute_geom::point_segment_dist(t.at, s) < 1.0 {
                eprintln!("   {:?} -> {:?}", tr.start, tr.end);
            }
        }
        for p in board.pad_ids() {
            if board.pad_position(p).dist(t.at) < 2_000_000.0 && board.pad(p).net == t.net {
                eprintln!("   pad centre {:?}", board.pad_position(p));
            }
        }
    }
}

#[test]
#[ignore = "diagnostic"]
fn diag_kink_stubs() {
    let probe = || {
        ferroute_geom::Point::new(
            std::env::var("PX").ok().and_then(|v| v.parse().ok()).unwrap_or(24_800_000),
            std::env::var("PY").ok().and_then(|v| v.parse().ok()).unwrap_or(5_000_000),
        )
    };
    let board = random_board(0, 12);
    let pitch = default_pitch(&board.rules);
    let plan = Plan::new(&board.rules, pitch);
    let bbox = board.outline.bbox().expect("outline");
    let grid = Grid::covering(
        ferroute_geom::Rect::new(choose_lattice(&board, bbox, Some(pitch)).1, bbox.max),
        pitch,
        vec![ferroute_board::LayerId(0), ferroute_board::LayerId(1)],
    );
    let obs = Obstacles::from_board(&board);
    let leg = legality::compute(&grid, &obs, &plan.classes[0].geom, None);
    for pid in board.pad_ids() {
        let c = board.pad_position(pid);
        if c.dist(probe()) > 1_000_000.0 {
            continue;
        }
        let mut stubs = Vec::new();
        let ts = access::pad_terminals(
            &board,
            &grid,
            &obs,
            &leg,
            &plan.classes[0],
            &plan,
            board.pad(pid).net,
            pid,
            &mut stubs,
        );
        eprintln!("pad {:?} at {:?} net {:?}: {} terminals", pid, c, board.pad(pid).net, ts.len());
        for s in &stubs {
            eprintln!("   dir {:?} kink {} bend {} path {:?}", s.dir, s.kink, s.bend, s.path);
        }
    }
    let out = route(&board, &exact_cfg()).expect("route");
    for t in out
        .edit
        .new_tracks
        .iter()
        .filter(|t| t.start.dist(probe()) < 1_500_000.0 || t.end.dist(probe()) < 1_500_000.0)
    {
        eprintln!("TRACK net {:?} {:?} -> {:?}", t.net, t.start, t.end);
    }
}

#[test]
fn lattice_aligns_pad_rows_and_columns() {
    // Parts on a 5 mm placement grid: a 0.5 mm lattice puts every pad *row* on a lattice
    // line (straight E/W approaches), the minimum 0.45 mm pitch would not.
    let mut b = BoardBuilder::new(mm(40), mm(30), 2);
    let n: Vec<_> = (0..4).map(|i| b.net(&format!("N{i}"))).collect();
    for k in 0..6i64 {
        b.smd2(
            &format!("R{k}"),
            pt10(50 + k * 50, 50 + (k % 3) * 50),
            0.0,
            n[(k % 4) as usize],
            n[((k + 1) % 4) as usize],
        );
    }
    let board = b.build();
    let (pitch, origin) = choose_lattice(&board, board.outline.bbox().expect("outline"), None);
    assert_eq!(pitch, 500_000);
    for p in board.pad_ids() {
        let c = board.pad_position(p) - origin;
        assert_eq!(c.y.rem_euclid(pitch), 0, "pad {p:?} row off lattice");
    }
    // An explicit pitch is respected.
    assert_eq!(
        choose_lattice(&board, board.outline.bbox().expect("outline"), Some(450_000)).0,
        450_000
    );
}
