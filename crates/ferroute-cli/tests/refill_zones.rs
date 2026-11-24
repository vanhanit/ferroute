// SPDX-License-Identifier: AGPL-3.0-only
//! `--no-refill-zones`: pours block routes (phase A); by default they yield and are refilled.

use std::path::{Path, PathBuf};
use std::process::Command;

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn route(board: &str, out: &Path, extra: &[&str]) -> String {
    let o = Command::new(env!("CARGO_BIN_EXE_ferroute"))
        .args(["route", &format!("testdata/boards/{board}.kicad_pcb"), "-o"])
        .arg(out)
        .args(extra)
        .current_dir(root())
        .output()
        .expect("run ferroute");
    String::from_utf8_lossy(&o.stdout).into_owned()
}

fn drc(out: &Path) -> String {
    let o = Command::new(env!("CARGO_BIN_EXE_ferroute"))
        .arg("drc")
        .arg(out)
        .args(["--fail-on-violation", "--fail-on-unconnected"])
        .current_dir(root())
        .output()
        .expect("run ferroute");
    assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stdout));
    String::from_utf8_lossy(&o.stdout).into_owned()
}

#[test]
fn pour_block_routes_only_when_the_pour_yields() {
    let dir = tempfile::tempdir().expect("tmp");
    let blocked = dir.path().join("blocked.kicad_pcb");
    let said = route("pour_block", &blocked, &["--no-refill-zones"]);
    assert!(said.contains("completion 0.0%"), "{said}");

    let routed = dir.path().join("routed.kicad_pcb");
    let said = route("pour_block", &routed, &[]);
    assert!(said.contains("completion 100.0%"), "{said}");
    assert!(drc(&routed).contains("0 violation(s)"));
    // The refilled pour is deterministic: the same run writes the same bytes.
    let again = dir.path().join("again.kicad_pcb");
    route("pour_block", &again, &[]);
    assert_eq!(
        std::fs::read_to_string(&routed).expect("a"),
        std::fs::read_to_string(&again).expect("b")
    );
}

#[test]
fn pour_gnd_output_stays_clean_with_refill() {
    let dir = tempfile::tempdir().expect("tmp");
    let out = dir.path().join("o.kicad_pcb");
    route("pour_gnd", &out, &[]);
    assert!(drc(&out).contains("0 violation(s)"));
}
