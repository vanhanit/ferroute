// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end tests of the `ferroute` binary (exit codes, files written, messages).

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn fixture(name: &str) -> PathBuf {
    root().join("testdata/boards").join(name)
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

#[test]
fn route_writes_a_drc_clean_board_report_and_svg() {
    let dir = tempfile::tempdir().expect("tmp");
    let out = dir.path().join("demo.kicad_pcb");
    let report = dir.path().join("demo.json");
    let svg = dir.path().join("demo.svg");
    let rules = root().join("testdata/rules/demo_2layer.toml");
    let o = ferroute(&[
        "route",
        s(&fixture("demo_2layer.kicad_pcb")),
        "-o",
        s(&out),
        "--allow-move",
        "--rules",
        s(&rules),
        "--strict-rules",
        "--fail-on-incomplete",
        "--report",
        s(&report),
        "--svg",
        s(&svg),
        "--accel",
        "cpu",
    ]);
    assert!(o.status.success(), "{}{}", stdout(&o), String::from_utf8_lossy(&o.stderr));
    assert!(stdout(&o).contains("completion 100.0%"), "{}", stdout(&o));
    assert!(out.with_extension("kicad_pro").exists(), "project rules copied next to output");
    let json: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&report).expect("report")).expect("json");
    assert_eq!(json["metrics"]["drc_clean"], true);
    assert_eq!(json["rule_issues"].as_array().map(Vec::len), Some(0));
    assert!(std::fs::read_to_string(&svg).expect("svg").starts_with("<svg"));

    // The routed board passes the oracle with strict flags.
    let d = ferroute(&["drc", s(&out), "--fail-on-violation", "--fail-on-unconnected"]);
    assert!(d.status.success(), "{}", stdout(&d));
    assert!(stdout(&d).contains("0 violation(s)"), "{}", stdout(&d));
}

#[test]
fn strict_rules_fail_with_exit_code_2_when_unmet() {
    let dir = tempfile::tempdir().expect("tmp");
    let rules = dir.path().join("r.toml");
    std::fs::write(&rules, "[[net]]\nnets = [\"SDA\"]\nlength_max_mm = 0.5\n").expect("write");
    let out = dir.path().join("o.kicad_pcb");
    let o = ferroute(&[
        "route",
        s(&fixture("demo_2layer.kicad_pcb")),
        "-o",
        s(&out),
        "--rules",
        s(&rules),
        "--strict-rules",
        "--layers",
        "F.Cu,B.Cu",
        "--accel",
        "none",
    ]);
    assert_eq!(o.status.code(), Some(2), "{}", stdout(&o));
    assert!(stdout(&o).contains("rule not met: net SDA: routed length"), "{}", stdout(&o));
}

#[test]
fn bad_inputs_fail_cleanly() {
    let dir = tempfile::tempdir().expect("tmp");
    let out = dir.path().join("o.kicad_pcb");
    let bad = dir.path().join("bad.toml");
    std::fs::write(&bad, "[[net]]\nnets = [\"A\"]\nmax_curent_a = 1\n").expect("write");
    let demo = fixture("demo_2layer.kicad_pcb");
    for args in [
        vec!["route", s(&demo), "-o", s(&out), "--rules", s(&bad)],
        vec!["route", s(&demo), "-o", s(&out), "--layers", "In7.Cu"],
        vec!["drc", "does/not/exist.kicad_pcb"],
        vec!["ipc", "probe", "--socket", "ipc:///nonexistent/ferroute-test.sock"],
    ] {
        let o = ferroute(&args);
        assert!(!o.status.success(), "{args:?} should fail");
        assert!(!String::from_utf8_lossy(&o.stderr).is_empty(), "{args:?} explains why");
    }
    assert!(!out.exists(), "nothing written on failure");
}

#[test]
fn in_place_routing_keeps_a_backup() {
    let dir = tempfile::tempdir().expect("tmp");
    let board = dir.path().join("b.kicad_pcb");
    std::fs::copy(fixture("legacy_v9_numeric.kicad_pcb"), &board).expect("copy");
    let o = ferroute(&["route", s(&board), "-o", s(&board), "--accel", "cpu"]);
    assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
    assert!(board.with_extension("kicad_pcb.ferroute-bak").exists());
}

#[test]
fn render_gen_info_and_drc_json() {
    let dir = tempfile::tempdir().expect("tmp");
    let svg = dir.path().join("r.svg");
    let o = ferroute(&["render", s(&fixture("demo_2layer.kicad_pcb")), "-o", s(&svg)]);
    assert!(o.status.success());
    assert!(std::fs::read_to_string(&svg).expect("svg").contains("</svg>"));

    let gen_dir = dir.path().join("gen");
    let o = ferroute(&["gen", "--suite", "ci", "-o", s(&gen_dir)]);
    assert!(o.status.success());
    let n = std::fs::read_dir(&gen_dir).expect("dir").count();
    assert!(n >= 6 && n.is_multiple_of(2), "board + project per spec: {n}");

    let o = ferroute(&["--threads", "2", "info"]);
    assert!(o.status.success());
    assert!(stdout(&o).contains("accelerator"), "{}", stdout(&o));

    let json = dir.path().join("drc.json");
    let o = ferroute(&["drc", s(&fixture("demo_2layer.kicad_pcb")), "--json", s(&json)]);
    assert!(o.status.success(), "unrouted input is not a violation without flags");
    let v: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&json).expect("json")).expect("parse");
    assert!(v.get("connectivity").is_some(), "{v}");
    let o = ferroute(&["drc", s(&fixture("demo_2layer.kicad_pcb")), "--fail-on-unconnected"]);
    assert_eq!(o.status.code(), Some(1), "input board has unrouted connections");
}

#[test]
fn bench_gates_against_a_baseline() {
    let dir = tempfile::tempdir().expect("tmp");
    let base = dir.path().join("baseline.json");
    let out = dir.path().join("bench");
    let args = |update: bool| {
        let mut v = vec![
            "bench".to_owned(),
            "--suite".into(),
            "ci".into(),
            "--baseline".into(),
            s(&base).into(),
            "--out".into(),
            s(&out).into(),
        ];
        if update {
            v.push("--update-baseline".into());
        }
        v
    };
    let run = |a: Vec<String>| ferroute(&a.iter().map(String::as_str).collect::<Vec<_>>());
    let o = run(args(true));
    assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
    assert!(base.exists());
    let o = run(args(false));
    assert!(o.status.success(), "{}", stdout(&o));
    assert!(stdout(&o).contains("benchmark gate: PASS"), "{}", stdout(&o));
}

#[test]
fn dsn_export_and_ses_import_round_trip() {
    let dir = tempfile::tempdir().expect("tmp");
    let dsn = dir.path().join("demo.dsn");
    let o = ferroute(&["dsn", "export", s(&fixture("demo_2layer.kicad_pcb")), "-o", s(&dsn)]);
    assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
    let text = std::fs::read_to_string(&dsn).expect("dsn");
    assert!(text.starts_with("(pcb ferroute"));
    // An empty session imports nothing and leaves a loadable board.
    let ses = dir.path().join("demo.ses");
    std::fs::write(&ses, "(session demo (routes (resolution um 1000) (network_out)))")
        .expect("ses");
    let out = dir.path().join("out.kicad_pcb");
    let o = ferroute(&[
        "dsn",
        "import-ses",
        s(&fixture("demo_2layer.kicad_pcb")),
        s(&ses),
        "-o",
        s(&out),
    ]);
    assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
    assert!(stdout(&o).contains("imported 0 tracks and 0 vias"));
    assert!(out.exists());
}
