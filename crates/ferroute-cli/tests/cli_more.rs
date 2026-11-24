// SPDX-License-Identifier: AGPL-3.0-only
//! More end-to-end `ferroute` behaviour: DRC reporting of violations, `gen` suites,
//! benchmark gate regressions and baseline updates.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn ferroute(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_ferroute"))
        .args(args)
        .current_dir(root())
        .output()
        .expect("run ferroute")
}

fn stdout(o: &Output) -> String {
    String::from_utf8_lossy(&o.stdout).into_owned()
}

fn s(p: &Path) -> &str {
    p.to_str().expect("utf-8 path")
}

/// `demo_2layer` with R2 moved on top of R1 (courtyard/copper overlap).
fn overlapped_board(dir: &Path) -> PathBuf {
    let text = std::fs::read_to_string(root().join("testdata/boards/demo_2layer.kicad_pcb"))
        .expect("fixture");
    let moved = text.replacen("(at 108 114)", "(at 108 108.4)", 1);
    assert_ne!(moved, text, "fixture layout changed: update the overlap edit");
    let path = dir.join("overlap.kicad_pcb");
    std::fs::write(&path, moved).expect("write");
    path
}

#[test]
fn drc_lists_violations_with_item_labels_and_fails_on_request() {
    let dir = tempfile::tempdir().expect("tmp");
    let board = overlapped_board(dir.path());
    let o = ferroute(&["drc", s(&board), "--fail-on-violation"]);
    assert_eq!(o.status.code(), Some(1), "{}", stdout(&o));
    let out = stdout(&o);
    assert!(!out.contains(": 0 violation(s)"), "{out}");
    // Every listed violation names pads as REF.NUMBER and carries a position in mm.
    let line = out.lines().find(|l| l.starts_with("  - ")).expect("a violation line");
    assert!(line.contains(" ↔ ") && line.contains(" mm: "), "{line}");
    assert!(line.contains("R1.") || line.contains("R2."), "{line}");
    // The per-kind histogram precedes the details.
    assert!(out.lines().any(|l| l.starts_with("  ") && !l.starts_with("  - ")), "{out}");
}

#[test]
fn drc_fail_on_unconnected_uses_exit_code_1() {
    let o = ferroute(&[
        "drc",
        s(&root().join("testdata/boards/demo_2layer.kicad_pcb")),
        "--fail-on-unconnected",
    ]);
    assert_eq!(o.status.code(), Some(1));
    assert!(stdout(&o).contains("unrouted connection(s), completion 0.0%"), "{}", stdout(&o));
}

#[test]
fn gen_fixtures_suite_writes_boards_with_project_files() {
    let dir = tempfile::tempdir().expect("tmp");
    let o = ferroute(&["gen", "--suite", "fixtures", "-o", s(dir.path())]);
    assert!(o.status.success());
    for name in ["bus_match", "diff_pair", "bga_planes"] {
        assert!(dir.path().join(format!("{name}.kicad_pcb")).is_file(), "{name}");
        assert!(dir.path().join(format!("{name}.kicad_pro")).is_file(), "{name}");
        assert!(stdout(&o).contains(&format!("{name}.kicad_pcb")));
    }
}

#[test]
fn gen_ci_suite_is_deterministic() {
    let a = tempfile::tempdir().expect("tmp");
    let b = tempfile::tempdir().expect("tmp");
    for d in [&a, &b] {
        assert!(ferroute(&["gen", "--suite", "ci", "-o", s(d.path())]).status.success());
    }
    let first = std::fs::read(a.path().join("ci-s2-small.kicad_pcb")).expect("board");
    assert_eq!(first, std::fs::read(b.path().join("ci-s2-small.kicad_pcb")).expect("board"));
}

#[test]
fn bench_update_baseline_then_gate_detects_a_regression() {
    let dir = tempfile::tempdir().expect("tmp");
    let out = dir.path().join("out");
    let base = dir.path().join("base/baseline.json");
    let common = [
        "bench",
        "--suite",
        "ci",
        "--only",
        "ci-s2-small",
        "--iterations",
        "5",
        "--lns-rounds",
        "0",
        "--out",
        s(&out),
        "--baseline",
        s(&base),
    ];
    let mut update: Vec<&str> = common.to_vec();
    update.extend(["--update-baseline", "--write-boards"]);
    let o = ferroute(&update);
    assert!(o.status.success(), "{}{}", stdout(&o), String::from_utf8_lossy(&o.stderr));
    assert!(stdout(&o).contains("baseline updated"), "{}", stdout(&o));
    assert!(out.join("ci-s2-small.kicad_pcb").is_file(), "--write-boards writes routed boards");
    assert!(out.join("summary.md").is_file() && out.join("records.json").is_file());

    // Same run against its own baseline passes.
    let o = ferroute(&common);
    assert!(stdout(&o).contains("benchmark gate: PASS"), "{}", stdout(&o));

    // Claim a better baseline than reality: the gate must fail with exit code 1.
    let mut json: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&base).expect("baseline")).expect("json");
    json["records"][0]["metrics"]["vias"] = serde_json::json!(0);
    json["records"][0]["metrics"]["wirelength_mm"] = serde_json::json!(0.001);
    std::fs::write(&base, serde_json::to_string(&json).expect("ser")).expect("write");
    let o = ferroute(&common);
    assert_eq!(o.status.code(), Some(1), "{}", stdout(&o));
    assert!(stdout(&o).contains("REGRESSION ci-s2-small"), "{}", stdout(&o));
}

#[test]
fn bench_dense_suite_prints_per_board_details() {
    let dir = tempfile::tempdir().expect("tmp");
    let o = ferroute(&[
        "bench",
        "--suite",
        "dense",
        "--only",
        "qfn_dense_2l",
        "--iterations",
        "2",
        "--lns-rounds",
        "0",
        "--out",
        s(dir.path()),
    ]);
    assert!(o.status.success(), "{}{}", stdout(&o), String::from_utf8_lossy(&o.stderr));
    let out = stdout(&o);
    assert!(out.contains("qfn_dense_2l: connections "), "{out}");
    assert!(out.contains("failed nets:"), "{out}");
}
