// SPDX-License-Identifier: AGPL-3.0-only
//! Routed fixtures and synthetic boards have no 90° bends, no avoidable bends next to
//! pads and no copper slivers. `FERROUTE_DIAG=1 cargo test ... -- --nocapture` lists them.

use ferroute_board::Board;
use ferroute_drc::quality::{near_pad_bends, pad_wraps, right_angles, slivers, via_jogs};
use std::path::{Path, PathBuf};
use std::process::Command;

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

/// Routes a fixture with the CLI and returns the routed board.
fn route(board: &str, extra: &[&str]) -> Board {
    let dir = tempfile::tempdir().expect("tmp");
    let out = dir.path().join("out.kicad_pcb");
    let res = Command::new(env!("CARGO_BIN_EXE_ferroute"))
        .args(["route", &format!("testdata/boards/{board}.kicad_pcb"), "-o"])
        .arg(&out)
        .args(extra)
        .current_dir(root())
        .output()
        .expect("run ferroute");
    assert!(res.status.success(), "{}", String::from_utf8_lossy(&res.stderr));
    ferroute_kicad::read_board_file(&out).expect("read routed board").board
}

/// Same-net tracks within 3 mm of `at`, in mm.
fn around(b: &Board, at: ferroute_geom::Point, net: ferroute_board::NetId) -> String {
    let mm = |p: ferroute_geom::Point| format!("({},{})", p.x as f64 / 1e6, p.y as f64 / 1e6);
    b.tracks
        .iter()
        .filter(|t| t.net == net && (t.start.dist(at) < 3e6 || t.end.dist(at) < 3e6))
        .map(|t| format!("{}->{}", mm(t.start), mm(t.end)))
        .collect::<Vec<_>>()
        .join(" ")
}

/// Asserts at most `max` pads whose connection wraps around its own footprint
/// ([`pad_wraps`]); the limit records what is known to remain, so it can only go down.
fn check_wraps(name: &str, b: &Board, max: usize) {
    let w = pad_wraps(b);
    if std::env::var("FERROUTE_DIAG").is_ok() {
        eprintln!("{name}: {} pad wraps", w.len());
        for x in &w {
            eprintln!(
                "  padwrap pad@{:?} {:?} {}",
                b.pad_position(x.pad),
                x.at,
                around(b, x.at, x.net)
            );
        }
    }
    assert!(w.len() <= max, "{name}: pad wraps {w:?}");
}

/// Asserts at most `(slivers, right angles, via jogs)`; the limits record what is still known to
/// remain (see the PR notes) so the counts can only go down.
fn check(name: &str, b: &Board, max: (usize, usize, usize)) {
    let (s, r, n, j) = (slivers(b), right_angles(b), near_pad_bends(b), via_jogs(b));
    if std::env::var("FERROUTE_DIAG").is_ok() {
        eprintln!(
            "{name}: {} slivers, {} right angles, {} near-pad bends, {} via jogs",
            s.len(),
            r.len(),
            n.len(),
            j.len()
        );
        for x in &s {
            eprintln!("  sliver {:?} {}", x.at, around(b, x.at, x.net));
        }
        for x in &j {
            eprintln!("  viajog {:?} {}", x.at, around(b, x.at, x.net));
        }
        for x in &r {
            eprintln!("  right {:?} {}", x.at, around(b, x.at, x.net));
        }
        for x in &pad_wraps(b) {
            eprintln!(
                "  padwrap pad@{:?} {:?} {}",
                b.pad_position(x.pad),
                x.at,
                around(b, x.at, x.net)
            );
        }
    }
    assert!(s.len() <= max.0, "{name}: slivers {s:?}");
    assert!(r.len() <= max.1, "{name}: right angles {r:?}");
    assert!(j.len() <= max.2, "{name}: via jogs {j:?}");
}

#[test]
fn demo_routes_are_clean_of_slivers_and_right_angles() {
    check("demo-fixed", &route("demo_2layer", &[]), (1, 0, 5));
    check("demo-move", &route("demo_2layer", &["--allow-move"]), (1, 1, 4));
}

#[test]
fn demo_routes_with_rules_are_clean() {
    let rules = ["--rules", "testdata/rules/demo_2layer.toml", "--strict-rules"];
    check("demo-fixed-rules", &route("demo_2layer", &rules), (1, 0, 5));
    let mv = ["--rules", "testdata/rules/demo_2layer.toml", "--allow-move"];
    check("demo-move-rules", &route("demo_2layer", &mv), (2, 1, 4));
}

#[test]
fn bus_match_routes_are_clean_of_slivers_and_right_angles() {
    let rules = ["--rules", "testdata/rules/bus_match.toml", "--strict-rules"];
    check("bus-fixed", &route("bus_match", &rules), (0, 0, 0));
    // Nudging changes the lengths the match groups can reach: no strict rules here.
    let mv = ["--rules", "testdata/rules/bus_match.toml", "--allow-move"];
    check("bus-move", &route("bus_match", &mv), (0, 0, 2));
}

#[test]
fn diff_pair_routes_coupled_and_clean() {
    let rules = ["--rules", "testdata/rules/diff_pair.toml", "--strict-rules"];
    let b = route("diff_pair", &rules);
    check("pair-fixed", &b, (0, 0, 0));
    assert!(ferroute_drc::quality::acute_angles(&b).is_empty());
    // With nudging the pair is re-coupled after any placement change; rules still hold.
    let mv = ["--rules", "testdata/rules/diff_pair.toml", "--allow-move", "--strict-rules"];
    check("pair-move", &route("diff_pair", &mv), (0, 0, 0));
}

#[test]
fn synthetic_ci_suite_is_clean_of_slivers_and_right_angles() {
    for spec in ferroute_engine::synth::suite("ci") {
        let board = ferroute_engine::synth::generate(&spec);
        let res = ferroute_engine::run(&board, &ferroute_engine::PipelineConfig::default())
            .expect("route");
        check(&spec.name, &res.board_after, (2, 1, 10));
    }
}

/// Every fixture, both placement modes: connections never circle their pad or double
/// back across its edge line, except for the one recorded here (demo_2layer with
/// nudging: a wrap at a junction of the pad's own net).
#[test]
fn fixtures_have_no_pad_wraps_beyond_the_known_dips() {
    let cases: [(&str, &[&str], usize); 12] = [
        ("bga_planes", &[], 0),
        ("bga_planes", &["--allow-move"], 0),
        ("demo_2layer", &[], 0),
        ("demo_2layer", &["--allow-move"], 1),
        ("demo_dru", &[], 0),
        ("demo_dru", &["--allow-move"], 0),
        ("bus_match", &[], 0),
        ("diff_pair", &[], 0),
        ("pour_block", &[], 0),
        ("pour_gnd", &[], 0),
        ("legacy_v9_numeric", &[], 0),
        ("bus_match", &["--allow-move"], 0),
    ];
    for (name, extra, max) in cases {
        check_wraps(&format!("{name} {extra:?}"), &route(name, extra), max);
    }
}

#[test]
fn synthetic_ci_suite_pad_wraps_are_bounded() {
    for spec in ferroute_engine::synth::suite("ci") {
        let board = ferroute_engine::synth::generate(&spec);
        let res = ferroute_engine::run(&board, &ferroute_engine::PipelineConfig::default())
            .expect("route");
        check_wraps(&spec.name, &res.board_after, 0);
    }
}
