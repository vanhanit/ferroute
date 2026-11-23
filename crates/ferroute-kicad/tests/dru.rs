// SPDX-License-Identifier: AGPL-3.0-only
//! Loading a board with a sibling `.kicad_dru`.

use ferroute_board::custom::ConstraintKind;
use ferroute_kicad::read_board_file;
use std::path::Path;

fn fixture(name: &str) -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/boards").join(name)
}

#[test]
fn sibling_dru_is_loaded_and_resolved() {
    let kb = read_board_file(&fixture("demo_dru.kicad_pcb")).unwrap();
    assert!(kb.rules_warnings.is_empty(), "{:?}", kb.rules_warnings);
    let mut b = kb.board;
    assert_eq!(b.rules.custom.len(), 3);
    let (v33, gnd, sda) = (
        b.net_by_name("+3V3").unwrap(),
        b.net_by_name("GND").unwrap(),
        b.net_by_name("SDA").unwrap(),
    );
    assert_eq!(b.rules.max_clearance_for_net(&b, v33), 500_000);
    assert_eq!(b.rules.min_width_for_net(&b, sda), 300_000);
    assert_eq!(b.rules.min_width_for_net(&b, gnd), 0, "Power class is not constrained");
    let notes = ferroute_board::constraints::resolve(&mut b);
    assert!(notes.iter().any(|n| n.contains("SDA") && n.contains("0.3 mm")), "{notes:?}");
    assert_eq!(b.rules.class_of(&b, sda).track_width, 300_000);
    assert_eq!(b.rules.class_of(&b, v33).clearance, 500_000);
    assert!(b.rules.has_custom(ConstraintKind::TrackWidth));
}

#[test]
fn board_without_dru_has_no_custom_rules() {
    let kb = read_board_file(&fixture("demo_2layer.kicad_pcb")).unwrap();
    assert!(kb.board.rules.custom.is_empty());
    assert!(kb.rules_warnings.is_empty());
}
