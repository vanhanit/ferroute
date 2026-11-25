// SPDX-License-Identifier: AGPL-3.0-only
//! KiCad custom rules (`demo_dru.kicad_dru`) are honoured end to end: the routed result
//! keeps the pair clearances and minimum widths, and our DRC oracle agrees.

use ferroute_board::Board;
use ferroute_drc::{DrcConfig, check, copper_items};
use std::path::{Path, PathBuf};
use std::process::Command;

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn route(extra: &[&str]) -> Board {
    let dir = tempfile::tempdir().expect("tmp");
    let out = dir.path().join("out.kicad_pcb");
    let res = Command::new(env!("CARGO_BIN_EXE_ferroute"))
        .args(["route", "testdata/boards/demo_dru.kicad_pcb", "-o"])
        .arg(&out)
        .args(extra)
        .current_dir(root())
        .output()
        .expect("run ferroute");
    assert!(res.status.success(), "{}", String::from_utf8_lossy(&res.stderr));
    let mut b = ferroute_kicad::read_board_file(&out).expect("read routed board").board;
    // The output directory has no sibling `.kicad_dru`: take the rules from the fixture.
    let input = ferroute_kicad::read_board_file(&root().join("testdata/boards/demo_dru.kicad_pcb"))
        .expect("read fixture");
    assert_eq!(input.board.rules.custom.len(), 3);
    b.rules.custom = input.board.rules.custom;
    b
}

fn assert_honoured(b: &Board) {
    let report = check(b, &DrcConfig::default());
    assert!(report.is_clean(), "{:#?}", report.violations);
    assert_eq!(report.connectivity.connections_missing, 0);

    let nm = |name: &str| b.net_by_name(name).expect(name);
    let (v33, gnd) = (nm("+3V3"), nm("GND"));
    let items = copper_items(b);
    let mut min_pwr = f64::INFINITY;
    let mut min_power_sig = f64::INFINITY;
    for (i, x) in items.iter().enumerate() {
        for y in &items[i + 1..] {
            if !x.layers.intersects(y.layers) || x.net == y.net {
                continue;
            }
            let d = x.hull.distance(&y.hull);
            let pair = |p: ferroute_board::NetId, q: ferroute_board::NetId| {
                (x.net == p && y.net == q) || (x.net == q && y.net == p)
            };
            if pair(v33, gnd) {
                min_pwr = min_pwr.min(d);
            }
            let class = |n| b.rules.class_of(b, n).name.clone();
            let is_power = |n| class(n).starts_with("Power");
            let is_default = |n| class(n).starts_with("Default");
            // Footprint-internal pad pairs come from library geometry, not routing.
            let routed = |it: &ferroute_drc::CopperItem| {
                matches!(it.item, ferroute_drc::ItemRef::Track(_) | ferroute_drc::ItemRef::Via(_))
            };
            if (routed(x) || routed(y))
                && ((is_power(x.net) && is_default(y.net))
                    || (is_power(y.net) && is_default(x.net)))
            {
                min_power_sig = min_power_sig.min(d);
            }
        }
    }
    assert!(min_pwr >= 500_000.0, "+3V3/GND distance {min_pwr}");
    assert!(min_power_sig >= 400_000.0, "Power/Default distance {min_power_sig}");
    for t in &b.tracks {
        if b.rules.class_of(b, t.net).name.starts_with("Default") {
            assert!(t.width >= 300_000, "Default-class track {} nm", t.width);
        }
    }
}

#[test]
fn fixed_placement_honours_custom_rules() {
    assert_honoured(&route(&[]));
}

#[test]
fn nudging_honours_custom_rules() {
    assert_honoured(&route(&["--allow-move"]));
}

fn run_route(input: &Path, output: &Path) -> std::process::Output {
    Command::new(env!("CARGO_BIN_EXE_ferroute"))
        .arg("route")
        .arg(input)
        .arg("-o")
        .arg(output)
        .current_dir(root())
        .output()
        .expect("run ferroute")
}

#[test]
fn route_writes_sidecars_named_after_the_output_stem() {
    let fixture = root().join("testdata/boards/demo_dru.kicad_pcb");
    let dir = tempfile::tempdir().expect("tmp");
    let out = dir.path().join("sub").join("renamed.kicad_pcb");
    // A stale, different sidecar at the target is overwritten.
    std::fs::create_dir_all(out.parent().unwrap()).unwrap();
    std::fs::write(out.with_extension("kicad_dru"), "stale").unwrap();
    let res = run_route(&fixture, &out);
    assert!(res.status.success(), "{}", String::from_utf8_lossy(&res.stderr));
    for ext in ["kicad_pro", "kicad_dru"] {
        assert_eq!(
            std::fs::read(out.with_extension(ext)).expect(ext),
            std::fs::read(fixture.with_extension(ext)).expect(ext),
            "{ext} copied byte for byte"
        );
    }
    assert!(String::from_utf8_lossy(&res.stdout).contains("wrote sidecar"));
}

#[test]
fn in_place_routing_never_touches_the_input_sidecars() {
    let dir = tempfile::tempdir().expect("tmp");
    let board = dir.path().join("b.kicad_pcb");
    for ext in ["kicad_pcb", "kicad_pro", "kicad_dru"] {
        std::fs::copy(
            root().join("testdata/boards/demo_dru").with_extension(ext),
            board.with_extension(ext),
        )
        .unwrap();
    }
    let read = || -> Vec<Vec<u8>> {
        ["kicad_pro", "kicad_dru"]
            .iter()
            .map(|e| std::fs::read(board.with_extension(e)).unwrap())
            .collect()
    };
    let before = read();
    let res = run_route(&board, &board);
    assert!(res.status.success(), "{}", String::from_utf8_lossy(&res.stderr));
    assert!(!String::from_utf8_lossy(&res.stdout).contains("wrote sidecar"));
    assert_eq!(before, read());
}

#[test]
fn missing_input_sidecars_are_not_invented() {
    let dir = tempfile::tempdir().expect("tmp");
    let input = dir.path().join("in.kicad_pcb");
    std::fs::copy(root().join("testdata/boards/demo_dru.kicad_pcb"), &input).unwrap();
    let out = dir.path().join("out.kicad_pcb");
    let res = run_route(&input, &out);
    assert!(res.status.success(), "{}", String::from_utf8_lossy(&res.stderr));
    assert!(!out.with_extension("kicad_pro").exists());
    assert!(!out.with_extension("kicad_dru").exists());
}
