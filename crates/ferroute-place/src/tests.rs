// SPDX-License-Identifier: AGPL-3.0-only
use super::*;
use ferroute_board::builder::{BoardBuilder, mm, pt10};
use ferroute_board::{BoardEdit, Mobility, Origin};
use ferroute_drc::{DrcConfig, ViolationKind, check};

#[test]
fn glob_matching() {
    assert!(glob("R*", "R12"));
    assert!(glob("R?", "R1"));
    assert!(!glob("R?", "R12"));
    assert!(glob("*", ""));
    assert!(glob("U*1", "U101"));
    assert!(!glob("C*", "R1"));
    assert!(glob("FB*", "FB3"));
}

/// R2 connects to J1 and R1 but sits far away on the right.
fn stretched() -> Board {
    let mut b = BoardBuilder::new(mm(40), mm(30), 2);
    let a = b.net("A");
    let c = b.net("C");
    let j = b.header("J1", pt10(50, 100), 0.0, &[a, c]);
    b.mobility(j, Mobility::Locked);
    b.smd2("R1", pt10(120, 100), 0.0, a, c);
    b.smd2("R2", pt10(300, 150), 0.0, a, c);
    b.build()
}

fn edit_of(r: &PlaceResult) -> BoardEdit {
    BoardEdit { moves: r.moves.clone(), ..BoardEdit::default() }
}

#[test]
fn policy_respects_locks_and_existing_copper() {
    let mut board = stretched();
    // Existing track on R1 pad 1 → R1 must be auto-fixed.
    let p = board.pad_position(board.footprints[1].pads[0]);
    board.tracks.push(ferroute_board::Track {
        start: p,
        end: p + Point::new(mm(1), 0),
        width: 250_000,
        layer: ferroute_board::LayerId(0),
        net: board.pad(board.footprints[1].pads[0]).net,
        locked: false,
        origin: Origin::Existing,
    });
    let n = apply_policy(&mut board, &MovePolicy::default());
    assert_eq!(n, 1);
    assert_eq!(board.footprints[0].mobility, Mobility::Locked);
    assert_eq!(board.footprints[1].mobility, Mobility::Fixed);
    assert!(board.footprints[2].mobility.is_movable());
}

#[test]
fn nudger_shortens_wires_legally_and_deterministically() {
    let mut board = stretched();
    apply_policy(&mut board, &MovePolicy { max_displacement: mm(10), ..MovePolicy::default() });
    let cfg = PlaceConfig { seed: 9, ..PlaceConfig::default() };
    let r = optimize(&board, &cfg);
    assert!(!r.moves.is_empty(), "{r:?}");
    assert!(r.after.wire_mm < r.before.wire_mm, "{:?} -> {:?}", r.before, r.after);
    let edit = edit_of(&r);
    edit.check_invariants(&board).expect("INV-EDIT-1");
    assert!(
        r.moves.iter().all(|m| board.footprint(m.footprint).reference != "J1"),
        "locked J1 moved"
    );
    let mut after = board.clone();
    after.apply(&edit);
    let rep = check(&after, &DrcConfig::default());
    assert_eq!(rep.count(ViolationKind::CourtyardOverlap), 0, "{:#?}", rep.violations);
    assert!(rep.introduced().next().is_none(), "{:#?}", rep.violations);
    // Determinism, including across thread pools.
    let pool = rayon::ThreadPoolBuilder::new().num_threads(1).build().expect("pool");
    let r2 = pool.install(|| optimize(&board, &cfg));
    assert_eq!(r.moves, r2.moves);
}

#[test]
fn nothing_movable_means_no_moves() {
    let board = stretched();
    let r = optimize(&board, &PlaceConfig::default());
    assert!(r.moves.is_empty());
    assert_eq!(r.movable, 0);
}

proptest::proptest! {
    #![proptest_config(proptest::prelude::ProptestConfig::with_cases(16))]
    #[test]
    fn nudger_never_violates_hard_constraints(seed in 0u64..100_000) {
        let mut b = BoardBuilder::new(mm(30), mm(25), 2);
        let nets: Vec<_> = (0..4).map(|i| b.net(&format!("N{i}"))).collect();
        let mut s = seed;
        for k in 0..8 {
            s = splitmix64(s);
            let x = 40 + i64::try_from(s % 6).unwrap() * 40;
            let y = 40 + i64::try_from((s >> 8) % 4).unwrap() * 45;
            if k > 0 && (s >> 16) % 3 == 0 { continue; }
            b.smd2(&format!("R{k}"), pt10(x, y), 0.0, nets[(s >> 20) as usize % 4], nets[(s >> 24) as usize % 4]);
        }
        let mut board = b.build();
        apply_policy(&mut board, &MovePolicy { max_displacement: mm(3), ..MovePolicy::default() });
        let before = check(&board, &DrcConfig::default());
        let r = optimize(&board, &PlaceConfig { seed, moves_per_fp: 100, starts: 2, ..PlaceConfig::default() });
        let edit = edit_of(&r);
        proptest::prop_assert!(edit.check_invariants(&board).is_ok());
        let mut after = board.clone();
        after.apply(&edit);
        let rep = check(&after, &DrcConfig::default());
        // Moves may fix pre-existing overlaps but must never create new violations.
        let new_viol = rep.introduced().filter(|v| !before.violations.iter().any(|b| b.kind == v.kind && b.a == v.a && b.b == v.b)).count();
        proptest::prop_assert_eq!(new_viol, 0, "{:#?}", rep.violations);
    }
}

#[test]
fn moves_land_on_the_placement_grid() {
    let mut board = stretched();
    // Put R2 off-grid first: its new position must still be snapped.
    board.footprints[2].placement.pos =
        board.footprints[2].placement.pos + Point::new(12_345, -6_789);
    board.footprints[2].original = board.footprints[2].placement;
    apply_policy(&mut board, &MovePolicy { max_displacement: mm(10), ..MovePolicy::default() });
    for grid in [50_000, 100_000, 250_000] {
        let r = optimize(&board, &PlaceConfig { seed: 3, grid, ..PlaceConfig::default() });
        assert!(!r.moves.is_empty());
        for m in &r.moves {
            if m.to.pos != m.from.pos {
                assert_eq!(
                    (m.to.pos.x % grid, m.to.pos.y % grid),
                    (0, 0),
                    "{m:?} off the {grid} nm grid"
                );
            }
        }
    }
}
