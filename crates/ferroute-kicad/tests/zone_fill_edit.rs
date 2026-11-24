// SPDX-License-Identifier: AGPL-3.0-only
//! `BoardEdit::zone_fills` rewrites only the `(filled_polygon ...)` nodes of edited zones.
mod common;

use common::mm;
use ferroute_board::{BoardEdit, LayerId, ZoneFill};
use ferroute_geom::{Point, Polygon};
use ferroute_kicad::{KicadBoard, ReadOptions, read_board};

const LAYERS: &str = r#"(layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))"#;
const RECT: &str = r#"(gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts") (width 0.1))"#;

const TWO_ZONES: &str = r#"(zone (net 1) (net_name "N1") (layer "F.Cu") (uuid "aaaa") (min_thickness 0.2)
    (fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5))
    (polygon (pts (xy 1 1) (xy 9 1) (xy 9 9) (xy 1 9)))
    (filled_polygon (layer "F.Cu") (pts (xy 1 1) (xy 9 1) (xy 9 9) (xy 1 9))))
  (zone (net 1) (net_name "N1") (layer "B.Cu") (uuid "bbbb") (min_thickness 0.2)
    (fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5))
    (polygon (pts (xy 1 1) (xy 9 1) (xy 9 9) (xy 1 9)))
    (filled_polygon (layer "B.Cu") (pts (xy 1 1) (xy 9 1) (xy 9 9) (xy 1 9))))"#;

fn read(extra: &str) -> KicadBoard {
    let src = format!(
        "(kicad_pcb (version 20211014) (generator pcbnew) (general (thickness 1.6)) (paper \"A4\") {LAYERS} (net 0 \"\") (net 1 \"N1\") {extra} {RECT})"
    );
    read_board(&src, &ReadOptions::default()).unwrap()
}

fn tri(x: f64) -> Polygon {
    Polygon::new(vec![
        Point::new(mm(x), mm(2.0)),
        Point::new(mm(x + 1.5), mm(2.0)),
        Point::new(mm(x), mm(3.25)),
    ])
}

#[test]
fn only_the_edited_zones_fill_nodes_change() {
    let mut kb = read(TWO_ZONES);
    let before = kb.doc.to_kicad_string();
    let edit = BoardEdit {
        zone_fills: vec![(
            1,
            vec![ZoneFill { layer: LayerId(1), polys: vec![tri(2.0), tri(5.0)] }],
        )],
        ..BoardEdit::default()
    };
    kb.doc.apply_edit(&kb.board, &edit, 1).unwrap();
    let after = kb.doc.to_kicad_string();
    assert_ne!(before, after);
    // Zone 0 is byte-identical, zone 1 differs only in its filled polygons.
    let first = |s: &str| {
        s[s.find("(uuid \"aaaa\")").unwrap()..s.find("(uuid \"bbbb\")").unwrap()].to_owned()
    };
    assert_eq!(first(&before), first(&after));
    let head = |s: &str| {
        let tail = &s[s.find("(uuid \"bbbb\")").unwrap()..];
        tail[..tail.find("(filled_polygon").unwrap()].to_owned()
    };
    assert_eq!(head(&before), head(&after));
    let again = read_board(&after, &ReadOptions::default()).unwrap();
    assert_eq!(again.board.zones[0].fills, kb.board.zones[0].fills);
    assert_eq!(again.board.zones[1].fills[0].polys, vec![tri(2.0), tri(5.0)]);
    assert!(after.contains("(xy 3.5 2)"), "{after}");
    assert_eq!(again.doc.to_kicad_string(), after, "idempotent re-serialisation");
}

#[test]
fn an_emptied_zone_loses_its_fills_and_bad_indices_are_rejected() {
    let mut kb = read(TWO_ZONES);
    let edit = BoardEdit {
        zone_fills: vec![(0, vec![ZoneFill { layer: LayerId(0), polys: vec![] }])],
        ..BoardEdit::default()
    };
    kb.doc.apply_edit(&kb.board, &edit, 1).unwrap();
    let after = kb.doc.to_kicad_string();
    assert_eq!(after.matches("(filled_polygon").count(), 1);
    let bad = BoardEdit { zone_fills: vec![(7, vec![])], ..BoardEdit::default() };
    assert!(kb.doc.apply_edit(&kb.board, &bad, 1).is_err());
    assert_eq!(kb.doc.to_kicad_string(), after, "failed edits leave the document unchanged");
}
