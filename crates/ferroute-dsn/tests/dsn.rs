// SPDX-License-Identifier: AGPL-3.0-only
//! DSN export / SES import tests.

use ferroute_board::builder::{BoardBuilder, mm, pt10};
use ferroute_board::{Board, Mobility, Origin};
use ferroute_dsn::{DsnError, read_ses, write_dsn};
use ferroute_geom::Point;
use std::path::Path;

fn dsn(b: &Board) -> String {
    let mut v = Vec::new();
    write_dsn(b, &mut v).expect("write");
    String::from_utf8(v).expect("utf-8")
}

fn sample() -> Board {
    let mut b = BoardBuilder::new(mm(40), mm(20), 2);
    let a = b.net("SIG A");
    let g = b.net("GND");
    let r1 = b.smd2("R1", pt10(100, 100), 0.0, a, g);
    b.smd2("R2", pt10(250, 100), 90.0, a, g);
    b.header("J1", pt10(300, 50), 0.0, &[a, g]);
    b.mobility(r1, Mobility::Locked);
    b.track(pt10(100, 60), pt10(150, 60), 250_000, 0, a, Origin::Existing);
    b.via(pt10(150, 60), 600_000, 300_000, a, Origin::Existing);
    b.keepout(ferroute_geom::Rect::new(pt10(0, 0), pt10(20, 20)), true, true);
    b.build()
}

#[test]
fn export_has_expected_sections_and_exact_units() {
    let d = dsn(&sample());
    assert!(d.starts_with("(pcb ferroute\n  (parser\n    (string_quote \")"));
    assert!(d.contains("(resolution um 1)"));
    assert!(d.contains("(layer F.Cu (type signal) (property (index 0)))"));
    assert!(d.contains("(layer B.Cu (type signal) (property (index 1)))"));
    // 40 x 20 mm outline in um, y negated, ring closed.
    assert!(d.contains("(boundary (path pcb 0 0 0 40000 0 40000 -20000 0 -20000 0 0))"));
    assert!(d.contains("(keepout KO1 (polygon F.Cu 0 "));
    assert!(d.contains("(via Via[0-1]_600000:300000_nm)"));
    // Locked part, quoted net names, protected existing copper.
    assert!(d.contains("(place R1 10000 -10000 front 0 (lock_type position))"));
    assert!(d.contains("(place R2 25000 -10000 front 0)"));
    assert!(d.contains("(net \"SIG A\" (pins R1-1 R2-1 J1-1))"));
    assert!(
        d.contains("(wire (path F.Cu 250 10000 -6000 15000 -6000) (net \"SIG A\") (type protect))")
    );
    assert!(
        d.contains("(via Via[0-1]_600000:300000_nm 15000 -6000 (net \"SIG A\") (type protect))")
    );
    assert!(d.contains("(class Default \"SIG A\" GND (circuit (use_via Via[0-1]_600000:300000_nm)) (rule (width 250) (clearance 200)))"));
    // Through-hole header pad exists on both layers; rotated SMD pad swaps its extents.
    assert!(d.contains("(shape (circle F.Cu 1700 0 0)) (shape (circle B.Cu 1700 0 0))"));
    assert!(d.contains("(shape (rect F.Cu -475 -450 475 450))"));
}

#[test]
fn export_is_deterministic_and_output_parses() {
    let b = sample();
    let (x, y) = (dsn(&b), dsn(&b));
    assert_eq!(x, y);
    // Balanced parentheses.
    let depth = x.chars().fold(0i64, |d, c| match c {
        '(' => d + 1,
        ')' => d - 1,
        _ => d,
    });
    assert_eq!(depth, 0);
}

#[test]
fn quote_in_a_net_name_is_rejected() {
    let mut b = BoardBuilder::new(mm(10), mm(10), 2);
    let n = b.net("bad\"name");
    b.smd2("R1", pt10(50, 50), 0.0, n, n);
    let mut v = Vec::new();
    assert!(matches!(write_dsn(&b.build(), &mut v), Err(DsnError::Unrepresentable(_))));
}

fn routed_board() -> Board {
    let mut b = BoardBuilder::new(mm(40), mm(20), 2);
    let a = b.net("A");
    let c = b.net("/B (x)");
    b.smd2("R1", pt10(100, 100), 0.0, a, c);
    b.track(pt10(10, 10), pt10(20, 10), 250_000, 0, a, Origin::Existing);
    b.build()
}

const SES: &str = r#"(session routed
  (base_design routed.dsn)
  (placement (resolution um 10))
  (routes
    (resolution um 10)
    (parser (string_quote ") (space_in_quoted_tokens on))
    (network_out
      (net A
        (wire (path F.Cu 2500 100000 -50000 100000 -80000.5 150000 -130000.5) (type route))
        (wire (path B.Cu 2500 0 0 0 0))
        (wire (path F.Cu 2500 0 0 5 5) (type protect))
        (via "Via[0-1]_600000:300000_nm" 100000 -50000)
        (via Via[0-1]_600:300_um 7 8)
      )
      (net "/B (x)"
        (wire (path B.Cu 2000 -10 20 30 20))
        (via Padstack_unknown 1 1)
      )
    )
  )
)"#;

#[test]
fn ses_converts_coordinates_exactly() {
    let b = routed_board();
    let e = read_ses(SES, &b).expect("ses");
    e.check_invariants(&b).expect("additive");
    let a = b.net_by_name("A").expect("A");
    // um 10 => 100 nm per unit; y negated.
    let t = &e.new_tracks;
    assert_eq!(t.len(), 3);
    assert_eq!(
        (t[0].start, t[0].end),
        (Point::new(10_000_000, 5_000_000), Point::new(10_000_000, 8_000_050))
    );
    assert_eq!(
        (t[1].start, t[1].end),
        (Point::new(10_000_000, 8_000_050), Point::new(15_000_000, 13_000_050))
    );
    assert_eq!(t[0].width, 250_000);
    assert_eq!(t[0].net, a);
    assert_eq!(t[0].layer.0, 0);
    assert!(t.iter().all(|x| x.origin == Origin::Routed && !x.locked));
    // B.Cu wire of net "/B (x)": zero-length path skipped for A, kept for B.
    assert_eq!((t[2].start, t[2].end), (Point::new(-1_000, -2_000), Point::new(3_000, -2_000)));
    assert_eq!(t[2].layer.0, 1);
    assert_eq!(t[2].width, 200_000);
    // Vias: our own name, KiCad-style um name, unknown padstack falls back to the class.
    let v = &e.new_vias;
    assert_eq!(v.len(), 3);
    assert_eq!(
        (v[0].pos, v[0].diameter, v[0].drill),
        (Point::new(10_000_000, 5_000_000), 600_000, 300_000)
    );
    assert_eq!((v[1].pos, v[1].diameter, v[1].drill), (Point::new(700, -800), 600_000, 300_000));
    assert_eq!((v[2].diameter, v[2].drill), (600_000, 300_000));
}

#[test]
fn ses_skips_existing_copper_and_rejects_unknowns() {
    let b = routed_board();
    // Existing track (1,1)-(2,1) mm; SES y is negated.
    let dup = "(session s (routes (resolution um 1000) (network_out (net A \
               (wire (path F.Cu 250000 2000000 -1000000 1000000 -1000000))))))";
    let e = read_ses(dup, &b).expect("ses");
    assert!(e.new_tracks.is_empty(), "duplicate of existing copper skipped: {e:?}");

    let unknown_net = "(session s (routes (resolution um 10) (network_out (net Q))))";
    assert!(matches!(read_ses(unknown_net, &b), Err(DsnError::Session(_))));
    let unknown_layer = "(session s (routes (resolution um 10) (network_out (net A (wire (path X.Cu 1 0 0 1 1))))))";
    assert!(matches!(read_ses(unknown_layer, &b), Err(DsnError::Session(_))));
    assert!(matches!(read_ses("(session s (routes))", &b), Err(DsnError::Session(_))));
    assert!(matches!(read_ses("(pcb x)", &b), Err(DsnError::Session(_))));
    assert!(matches!(read_ses("(session", &b), Err(DsnError::Syntax(_))));
}

#[test]
fn ses_mil_and_mm_resolutions_are_exact() {
    let b = routed_board();
    let mil = "(session s (routes (resolution mil 1) (network_out (net A (wire (path F.Cu 10 0 0 100 0))))))";
    let e = read_ses(mil, &b).expect("mil");
    assert_eq!(e.new_tracks[0].width, 254_000);
    assert_eq!(e.new_tracks[0].end, Point::new(2_540_000, 0));
    let mm_ = "(session s (routes (resolution mm 1000) (network_out (net A (wire (path F.Cu 0.25 0 0 1.5 0))))))";
    let e = read_ses(mm_, &b).expect("mm");
    assert_eq!(e.new_tracks[0].width, 250);
    assert_eq!(e.new_tracks[0].end, Point::new(1_500, 0));
}

#[test]
fn edit_round_trips_through_dsn_units() {
    // An SES generated in our own DSN units (nm) reproduces the edit exactly.
    let b = routed_board();
    let ses = "(session s (routes (resolution um 1000) (network_out (net A \
        (wire (path F.Cu 250000 1000000 -2000000 3000000 -2000000 3000000 -4000000)) \
        (via \"Via[0-1]_600000:300000_nm\" 3000000 -4000000)))))";
    let e = read_ses(ses, &b).expect("ses");
    assert_eq!(e.new_tracks.len(), 2);
    assert_eq!(e.new_tracks[1].end, Point::new(3_000_000, 4_000_000));
    assert_eq!(e.new_vias[0].pos, Point::new(3_000_000, 4_000_000));
    assert_eq!(e.new_vias[0].layers, b.all_layers());
}

#[test]
fn demo_board_export_is_byte_identical_and_complete() {
    let p =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/boards/demo_2layer.kicad_pcb");
    let kb = ferroute_kicad::read_board_file(&p).expect("read demo");
    let (x, y) = (dsn(&kb.board), dsn(&kb.board));
    assert_eq!(x, y, "byte-identical across runs");
    assert!(x.starts_with("(pcb ferroute\n"));
    assert!(x.contains("(resolution um 1)"));
    for fp in &kb.board.footprints {
        assert!(x.contains(&format!("(place {} ", fp.reference)), "{} placed", fp.reference);
    }
    for n in kb.board.nets.iter().skip(1) {
        assert!(
            x.contains(&format!(
                "(net {}",
                if n.name.contains([' ', '(', ')']) {
                    format!("\"{}\"", n.name)
                } else {
                    n.name.clone()
                }
            )),
            "net {}",
            n.name
        );
    }
    let pins = x.matches("      (pin ").count();
    assert_eq!(pins, kb.board.pads.len());
    // Golden head: the first structure lines are stable.
    let head: Vec<&str> = x.lines().take(9).collect();
    assert_eq!(head[0], "(pcb ferroute");
    assert_eq!(head[7], "  (resolution um 1)");
    assert_eq!(head[8], "  (structure");
}

#[test]
fn units_are_freerouting_safe_and_round_conservatively() {
    // Regression: at (resolution um 1000) Freerouting 2.5 overflowed its coordinates
    // (circle diameters read -1, keep-outs "degenerate", nothing routed). `um 10` makes its SES
    // writer scale twice, so `um 1` is used; sizes round up, positions to nearest.
    let mut b = BoardBuilder::new(mm(40), mm(20), 2);
    let a = b.net("A");
    b.track(Point::new(1_049, 0), Point::new(2_051, 0), 250_001, 0, a, Origin::Existing);
    let d = dsn(&b.build());
    assert!(d.contains("(resolution um 1)"));
    assert!(d.contains("(path F.Cu 251 1 0 2 0)"), "{d}");
}

#[test]
fn freerouting_session_for_demo_imports_clean() {
    // `testdata/dsn/demo_2layer.freerouting.ses` is the real Freerouting 2.5.0 output for
    // our DSN export of the demo board (regenerate with scripts/freerouting_compare.sh).
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata");
    let kb = ferroute_kicad::read_board_file(&root.join("boards/demo_2layer.kicad_pcb"))
        .expect("read demo");
    let ses = std::fs::read_to_string(root.join("dsn/demo_2layer.freerouting.ses")).expect("ses");
    let edit = read_ses(&ses, &kb.board).expect("Freerouting SES parses");
    assert!(!edit.new_tracks.is_empty() && !edit.new_vias.is_empty());
    // Resolution regression: Freerouting echoes `um 1`; widths stay plausible.
    assert!(edit.new_tracks.iter().all(|t| (100_000..=2_000_000).contains(&t.width)));
    let mut routed = kb.board;
    routed.apply(&edit);
    let report = ferroute_drc::check(&routed, &ferroute_drc::DrcConfig::default());
    let errors: Vec<_> = report.errors().collect();
    assert!(errors.is_empty(), "{errors:?}");
}

#[test]
fn pours_are_exported_as_planes() {
    // Regression: Freerouting routed straight through other-net pours (DRC shorts).
    let p = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/boards/pour_gnd.kicad_pcb");
    let kb = ferroute_kicad::read_board_file(&p).expect("read pour board");
    let d = dsn(&kb.board);
    assert!(d.contains("(plane GND (polygon "), "{d}");
}

#[test]
fn class_width_is_the_nets_required_width() {
    // Regression: the class width must be the widest of class/current/impedance rules, never
    // the board default (Freerouting treats it as the default width of the net).
    let mut b = sample();
    b.rules.classes[0].track_width = 300_000;
    let d = dsn(&b);
    assert!(d.contains("(rule (width 300) (clearance "), "{d}");
    assert!(!d.contains("(rule (width 250)"), "{d}");
}

#[test]
fn freerouting_settings_cover_what_the_dsn_cannot_express() {
    // Regression (Freerouting 2.5.0): neckdown/fanout narrow traces below the net width,
    // drill clearance and pour obstacles have no DSN keyword.
    let p = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/boards/pour_gnd.kicad_pcb");
    let kb = ferroute_kicad::read_board_file(&p).expect("read pour board");
    let s = ferroute_dsn::freerouting_settings(&kb.board);
    assert!(s.plane_as_obstacle);
    assert_eq!(s.hole_clearance_um, (kb.board.rules.hole_clearance + 999) / 1000);
    let args = s.args();
    for want in [
        "--router.fanout.enabled=false",
        "--router.automatic_neckdown=false",
        "--router.plane_as_obstacle=true",
    ] {
        assert!(args.iter().any(|a| a == want), "{args:?}");
    }
    assert!(args.iter().any(|a| a.starts_with("--router.hole_clearance_um=")), "{args:?}");
    // No pour: pours are not obstacles.
    assert!(!ferroute_dsn::freerouting_settings(&sample()).plane_as_obstacle);
}
