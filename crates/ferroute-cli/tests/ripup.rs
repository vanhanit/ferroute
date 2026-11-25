// SPDX-License-Identifier: AGPL-3.0-only
//! `ferroute route --ripup-all` / `--ripup-nets`: explicit removal of existing copper.

use ferroute_kicad::read_board_file;
use std::collections::BTreeMap;
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

fn text(o: &Output) -> String {
    format!("{}{}", String::from_utf8_lossy(&o.stdout), String::from_utf8_lossy(&o.stderr))
}

fn s(p: &Path) -> &str {
    p.to_str().expect("utf-8 path")
}

const DEMO: &str = "testdata/boards/demo_2layer.kicad_pcb";

/// Routes `input` into `out` with the common flags plus `extra`; asserts success.
fn route(input: &str, out: &Path, extra: &[&str]) -> String {
    let mut args = vec!["route", input, "-o", s(out), "--accel", "cpu", "--fail-on-incomplete"];
    args.extend_from_slice(extra);
    let o = ferroute(&args);
    assert!(o.status.success(), "{}", text(&o));
    text(&o)
}

fn assert_drc_clean(p: &Path) {
    let d = ferroute(&["drc", s(p), "--fail-on-violation", "--fail-on-unconnected"]);
    assert!(d.status.success(), "{}", text(&d));
    assert!(text(&d).contains("0 violation(s)"), "{}", text(&d));
}

/// Multiset of copper: `T net layer start end` per segment, `V net pos` per via.
fn copper(p: &Path) -> BTreeMap<String, usize> {
    let kb = read_board_file(p).expect("read");
    let b = &kb.board;
    let mut m = BTreeMap::new();
    for t in &b.tracks {
        let k = format!("T {} {} {:?} {:?}", b.nets[t.net.idx()].name, t.layer.0, t.start, t.end);
        *m.entry(k).or_insert(0) += 1;
    }
    for v in &b.vias {
        let k = format!("V {} {:?}", b.nets[v.net.idx()].name, v.pos);
        *m.entry(k).or_insert(0) += 1;
    }
    m
}

fn net_of(key: &str) -> &str {
    key.split(' ').nth(1).unwrap_or("")
}

#[test]
fn ripup_all_reroutes_a_routed_board_from_scratch_without_duplicates() {
    let dir = tempfile::tempdir().expect("tmp");
    let first = dir.path().join("first.kicad_pcb");
    let second = dir.path().join("second.kicad_pcb");
    route(DEMO, &first, &["--seed", "3"]);
    let said = route(s(&first), &second, &["--seed", "3", "--ripup-all"]);
    assert!(said.contains("rip-up: removed"), "{said}");
    assert!(said.contains("deleted from the output"), "{said}");
    assert!(said.contains("completion 100.0%"), "{said}");
    assert_drc_clean(&second);
    let c = copper(&second);
    assert!(!c.is_empty());
    assert!(c.values().all(|&n| n == 1), "duplicate copper: {c:?}");
    // The demo has no pre-existing copper, so ripping up the routed result is the same
    // problem as the first run: same seed, byte-identical output (INV-IO-1, INV-DET).
    assert_eq!(copper(&first), c);
    assert_eq!(
        std::fs::read_to_string(&first).expect("first"),
        std::fs::read_to_string(&second).expect("second")
    );
}

#[test]
fn without_ripup_existing_copper_is_kept() {
    let dir = tempfile::tempdir().expect("tmp");
    let first = dir.path().join("first.kicad_pcb");
    let second = dir.path().join("second.kicad_pcb");
    route(DEMO, &first, &[]);
    route(s(&first), &second, &[]);
    assert_eq!(copper(&first), copper(&second), "an already complete board is left alone");
}

#[test]
fn ripup_nets_removes_only_the_named_nets_old_copper() {
    let dir = tempfile::tempdir().expect("tmp");
    let first = dir.path().join("first.kicad_pcb");
    let second = dir.path().join("second.kicad_pcb");
    let report = dir.path().join("r.json");
    route(DEMO, &first, &[]);
    let before = copper(&first);
    let sda_before = before.keys().filter(|k| net_of(k) == "SDA").count();
    assert!(sda_before > 0, "demo routes SDA");
    route(s(&first), &second, &["--ripup-nets", "SDA", "--report", s(&report)]);
    assert_drc_clean(&second);
    let after = copper(&second);
    assert!(after.values().all(|&n| n == 1), "duplicate copper: {after:?}");
    // Every other net's copper is untouched.
    for (k, n) in before.iter().filter(|(k, _)| net_of(k) != "SDA") {
        assert_eq!(after.get(k), Some(n), "lost {k}");
    }
    let json: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&report).expect("report")).expect("json");
    assert_eq!(json["ripup"]["nets"], serde_json::json!(["SDA"]));
    let removed = json["ripup"]["tracks"].as_u64().expect("tracks");
    let sda_tracks: usize =
        before.iter().filter(|(k, _)| k.starts_with("T SDA")).map(|(_, n)| n).sum();
    assert_eq!(removed, sda_tracks as u64, "exactly SDA's old segments were removed");
    assert_eq!(json["metrics"]["completion"], 1.0);
}

#[test]
fn unknown_pattern_warns_and_flags_conflict() {
    let dir = tempfile::tempdir().expect("tmp");
    let first = dir.path().join("first.kicad_pcb");
    let second = dir.path().join("second.kicad_pcb");
    route(DEMO, &first, &[]);
    let said = route(s(&first), &second, &["--ripup-nets", "NOPE_*,SCL"]);
    assert!(said.contains("matches no net"), "{said}");
    let o = ferroute(&["route", DEMO, "-o", s(&second), "--ripup-all", "--ripup-nets", "A"]);
    assert!(!o.status.success(), "mutually exclusive flags");
}

#[test]
fn rules_file_ripup_applies_and_cli_wins() {
    let dir = tempfile::tempdir().expect("tmp");
    let first = dir.path().join("first.kicad_pcb");
    let second = dir.path().join("second.kicad_pcb");
    let rules = dir.path().join("r.toml");
    route(DEMO, &first, &[]);
    std::fs::write(&rules, "[routing]\nripup = [\"SDA\"]\n").expect("write");
    let said = route(s(&first), &second, &["--rules", s(&rules)]);
    assert!(said.contains("(SDA)"), "{said}");
    let said = route(s(&first), &second, &["--rules", s(&rules), "--ripup-nets", "SCL"]);
    assert!(said.contains("(SCL)"), "{said}");
}

const POUR: &str = "testdata/boards/pour_gnd.kicad_pcb";

#[test]
fn ripup_all_keeps_pour_net_copper_and_the_pours_stay_refilled() {
    let dir = tempfile::tempdir().expect("tmp");
    let first = dir.path().join("first.kicad_pcb");
    let second = dir.path().join("second.kicad_pcb");
    route(POUR, &first, &[]);
    let said = route(s(&first), &second, &["--ripup-all"]);
    assert!(said.contains("kept: owns a copper pour"), "{said}");
    assert!(said.contains("kept 1 pour net(s) (GND)"), "{said}");
    assert!(said.contains("completion 100.0%"), "{said}");
    assert!(!said.contains("copper pours kept as obstacles"), "{said}");
    assert_drc_clean(&second);
    let (before, after) = (copper(&first), copper(&second));
    for (k, n) in before.iter().filter(|(k, _)| net_of(k) == "GND") {
        assert_eq!(after.get(k), Some(n), "lost pour-net copper {k}");
    }
}

#[test]
fn ripup_pour_nets_flag_rips_them_and_needs_a_scope() {
    let dir = tempfile::tempdir().expect("tmp");
    let first = dir.path().join("first.kicad_pcb");
    let second = dir.path().join("second.kicad_pcb");
    route(POUR, &first, &[]);
    let said = route(s(&first), &second, &["--ripup-all", "--ripup-pour-nets"]);
    assert!(said.contains("plane fan-out"), "{said}");
    assert!(said.contains("completion 100.0%"), "{said}");
    let o = ferroute(&["route", POUR, "-o", s(&second), "--ripup-pour-nets"]);
    assert!(!o.status.success(), "--ripup-pour-nets alone is invalid");
}
