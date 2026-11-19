// SPDX-License-Identifier: AGPL-3.0-only
mod common;

use common::mm;
use ferroute_board::{Board, LayerId, LayerSet, Mobility, PadShape};
use ferroute_geom::{Point, Polygon};
use ferroute_kicad::project::read_project_rules;
use ferroute_kicad::{KicadError, ReadOptions, read_board};

const LAYERS: &str = r#"(layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))"#;
const RECT: &str = r#"(gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts") (width 0.1))"#;

fn board_src(version: &str, layers: &str, extra: &str) -> String {
    format!(
        "(kicad_pcb (version {version}) (generator pcbnew) (general (thickness 1.6)) (paper \"A4\") {layers} (net 0 \"\") (net 1 \"N1\") {extra} {RECT})"
    )
}

fn read(extra: &str) -> ferroute_kicad::KicadBoard {
    read_board(&board_src("20211014", LAYERS, extra), &ReadOptions::default()).unwrap()
}

fn pad(shape_and_rest: &str) -> String {
    format!(
        r#"(footprint "L:X" (layer "F.Cu") (at 10 10) (fp_text reference "X1" (at 0 0) (layer "F.SilkS")) (pad "1" {shape_and_rest}))"#
    )
}

fn area_mm2(p: &Polygon) -> f64 {
    #[allow(clippy::cast_precision_loss)]
    let a = p.area2().unsigned_abs() as f64 / 2.0 / 1e12;
    a
}

#[test]
fn kicad8_layer_ids_do_not_matter() {
    let layers = r#"(layers (31 "B.Cu" signal) (0 "F.Cu" signal) (2 "In2.Cu" power) (1 "In1.Cu" signal) (44 "Edge.Cuts" user))"#;
    let kb = read_board(&board_src("20231014", layers, ""), &ReadOptions::default()).unwrap();
    let names: Vec<&str> = kb.board.copper_layers.iter().map(|l| l.name.as_str()).collect();
    assert_eq!(names, ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]);
    assert!(!kb.board.copper_layers[2].routable && kb.board.copper_layers[1].routable);
    // KiCad 9 ids
    let layers9 = r#"(layers (0 "F.Cu" signal) (4 "In1.Cu" signal) (6 "In2.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))"#;
    let kb = read_board(&board_src("20241229", layers9, ""), &ReadOptions::default()).unwrap();
    assert_eq!(
        kb.board.copper_layers.iter().map(|l| l.name.as_str()).collect::<Vec<_>>(),
        ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]
    );
}

#[test]
fn kicad6_style_footprint() {
    let kb = read(
        r#"(footprint "Lib:R" locked (layer "F.Cu") (tedit 5F) (tstamp 1234) (at 5 5 30)
            (fp_text reference "R7" (at 0 0 30) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
            (fp_text value "1k" (at 0 1) (layer "F.Fab"))
            (pad 1 smd rect (at 1 0 30) (size 1 1) (layers F.Cu F.Paste F.Mask) (net 1 "N1") (tstamp 1))
            (pad 2 smd rect (at -1 0 75) (size 1 1) (layers F.Cu F.Paste F.Mask) (tstamp 2)))"#,
    );
    let b = &kb.board;
    let f = &b.footprints[0];
    assert!(f.locked);
    assert_eq!(f.mobility, Mobility::Locked);
    assert_eq!(f.reference, "R7");
    assert_eq!(b.pads.len(), 2);
    assert_eq!(b.pads[0].local_rot.degrees().to_bits(), 0.0f64.to_bits());
    assert!((b.pads[1].local_rot.degrees() - 45.0).abs() < 1e-9);
    assert_eq!(b.nets[b.pads[0].net.idx()].name, "N1");
    assert_eq!(b.pads[1].net.0, 0);
    assert_eq!(b.pads[0].layers, LayerSet::single(LayerId(0)));
    // no courtyard: pad bounding box (pad 2 is a 1x1 square turned by 45 degrees) + 0.25 mm
    let c = f.courtyard_local.bbox().unwrap();
    assert!(common::near(c.min, Point::new(-1_957_107, -957_107), 2), "{c:?}");
    assert!(common::near(c.max, Point::new(1_750_000, 957_107), 2), "{c:?}");
}

#[test]
fn pad_shapes() {
    // trapezoid: TransformTrapezoidToPolygon corners
    let b =
        read(&pad(r#"smd trapezoid (at 0 0) (size 2 1) (rect_delta 0 0.5) (layers "F.Cu")"#)).board;
    let PadShape::Convex { pts } = &b.pads[0].shape else { panic!("{:?}", b.pads[0].shape) };
    let mut got = pts.clone();
    got.sort();
    let mut want = vec![
        Point::new(mm(-0.75), mm(-0.5)),
        Point::new(mm(0.75), mm(-0.5)),
        Point::new(mm(1.25), mm(0.5)),
        Point::new(mm(-1.25), mm(0.5)),
    ];
    want.sort();
    assert_eq!(got, want);

    // custom pad: hull covers anchor, polygon and its stroke
    let b = read(&pad(r#"smd custom (at 0 0) (size 0.4 0.4) (layers "F.Cu") (options (clearance outline) (anchor circle)) (primitives (gr_poly (pts (xy 0 0) (xy 2 0) (xy 2 1)) (width 0.1) (fill yes)))"#)).board;
    let PadShape::Convex { .. } = &b.pads[0].shape else { panic!() };
    let bb = b.pads[0].shape.local_hull().bbox();
    assert!(bb.max.x >= mm(2.05) && bb.max.x < mm(2.07), "{bb:?}");
    assert!(bb.max.y >= mm(1.05) && bb.max.y < mm(1.07));
    assert!(bb.min.x <= mm(-0.2) && bb.min.y <= mm(-0.2));

    // chamfered rect is a plain rect superset
    let b = read(&pad(r#"smd roundrect (at 0 0) (size 2 1) (layers "F.Cu") (roundrect_rratio 0.2) (chamfer_ratio 0.2) (chamfer top_left)"#)).board;
    assert_eq!(b.pads[0].shape, PadShape::Rect { w: mm(2.0), h: mm(1.0) });

    // roundrect radius rounds down
    let b = read(&pad(
        r#"smd roundrect (at 0 0) (size 1.3 0.7) (layers "F.Cu") (roundrect_rratio 0.2499999)"#,
    ))
    .board;
    assert_eq!(
        b.pads[0].shape,
        PadShape::RoundRect { w: mm(1.3), h: mm(0.7), radius: mm(0.174_999) }
    );

    // non-plated hole without copper keeps its drill
    let b =
        read(&pad(r#"np_thru_hole circle (at 0 0) (size 3 3) (drill 3) (layers "*.Mask")"#)).board;
    assert!(b.pads[0].layers.is_empty());
    let d = b.pads[0].drill.unwrap();
    assert_eq!((d.diameter, d.plated), (mm(3.0), false));
    assert_eq!(b.pads[0].net.0, 0);

    // slot drill: the larger dimension
    let b = read(&pad(
        r#"thru_hole oval (at 0 0) (size 2 1) (drill oval 1.2 0.6) (layers "*.Cu" "*.Mask")"#,
    ))
    .board;
    assert_eq!(b.pads[0].drill.map(|d| d.diameter), Some(mm(1.2)));
    assert_eq!(b.pads[0].layers, b.all_layers());

    // shape offset: translated bounding box
    let b = read(&pad(
        r#"thru_hole circle (at 0 0) (size 2 2) (drill 1 (offset 0.5 0)) (layers "*.Cu")"#,
    ))
    .board;
    let bb = b.pads[0].shape.local_hull().bbox();
    assert_eq!((bb.min.x, bb.max.x, bb.min.y, bb.max.y), (mm(-0.5), mm(1.5), mm(-1.0), mm(1.0)));

    // per-layer padstack shapes are merged into a superset and reported
    let kb = read(&pad(
        r#"thru_hole circle (at 0 0) (size 1.6 1.6) (drill 0.8) (layers "*.Cu" "*.Mask") (padstack (mode front_inner_back) (layer "Inner" (shape circle) (size 2 2)) (layer "B.Cu" (shape rect) (size 1.6 1.6)))"#,
    ));
    assert_eq!(kb.board.pads[0].shape, PadShape::Rect { w: mm(2.0), h: mm(2.0) });
    assert!(kb.warnings.iter().any(|w| w.contains("padstack")));
}

#[test]
fn net_dialects() {
    // KiCad 10: names only, a quoted number is a name
    let src = r#"(kicad_pcb (version 20260206) (generator "pcbnew") (paper "A4")
        (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
        (footprint "L:X" (layer "F.Cu") (at 1 1) (property "Reference" "U1" (at 0 0 0) (layer "F.SilkS"))
          (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net "1"))
          (pad "2" smd rect (at 2 0) (size 1 1) (layers "F.Cu") (net "GND")))
        (segment (start 1 1) (end 3 1) (width 0.2) (layer "F.Cu") (net "GND") (uuid "x"))
        (gr_rect (start 0 0) (end 5 5) (layer "Edge.Cuts"))
    )"#;
    let kb = read_board(src, &ReadOptions::default()).unwrap();
    assert!(kb.doc.uses_net_names());
    let names: Vec<&str> = kb.board.nets.iter().map(|n| n.name.as_str()).collect();
    assert_eq!(names, ["", "1", "GND"]);
    assert_eq!(kb.board.tracks[0].net.0, 2);

    // unknown numeric code -> unconnected + warning
    let kb =
        read(r#"(segment (start 1 1) (end 3 1) (width 0.2) (layer "F.Cu") (net 9) (tstamp 1))"#);
    assert_eq!(kb.board.tracks[0].net.0, 0);
    assert!(kb.warnings.iter().any(|w| w.contains("net code 9")));
}

#[test]
fn keepouts_and_outline_features() {
    let kb = read(
        r#"(zone (net 0) (net_name "") (layers "F&B.Cu") (tstamp 1) (hatch edge 0.5)
             (keepout (tracks allowed) (vias not_allowed) (pads allowed) (copperpour allowed) (footprints allowed))
             (polygon (pts (xy 1 1) (xy 5 1) (xy 5 5))))
           (zone (net 0) (net_name "") (layer "F.Cu") (tstamp 2) (hatch edge 0.5)
             (keepout (tracks allowed) (vias allowed) (pads allowed) (copperpour not_allowed) (footprints allowed))
             (polygon (pts (xy 1 1) (xy 5 1) (xy 5 5))))
           (gr_circle (center 10 10) (end 11 10) (layer "Edge.Cuts") (width 0.1))
           (footprint "L:Slot" (layer "F.Cu") (at 15 10 90) (fp_rect (start -1 -2) (end 1 2) (layer "Edge.Cuts") (width 0.1)))"#,
    );
    let b = &kb.board;
    assert_eq!(b.keepouts.len(), 1, "pour-only rule areas are not obstacles for routing");
    let k = &b.keepouts[0];
    assert!(!k.no_tracks && k.no_vias);
    assert_eq!(k.layers, LayerSet::all(2));
    // circle hole: a superset of the true disc
    assert_eq!(b.outline.holes.len(), 2);
    let circle = b.outline.holes.iter().max_by_key(|h| h.pts.len()).unwrap();
    let a = area_mm2(circle);
    assert!((std::f64::consts::PI * (1.0 - 1e-6)..std::f64::consts::PI + 0.01).contains(&a), "{a}");
    for k in 0..720 {
        let t = f64::from(k) * std::f64::consts::TAU / 720.0;
        let r = 0.999e6;
        #[allow(clippy::cast_possible_truncation)]
        let p = Point::new(
            10_000_000 + (r * t.cos()).round() as i64,
            10_000_000 + (r * t.sin()).round() as i64,
        );
        assert!(circle.contains(p), "{p:?}");
    }
    // footprint Edge.Cuts rect rotated 90 degrees: x in [-2,2], y in [-1,1] about (15,10)
    let slot = b.outline.holes.iter().min_by_key(|h| h.pts.len()).unwrap();
    let bb = slot.bbox().unwrap();
    assert_eq!((bb.min, bb.max), (Point::new(mm(13.0), mm(9.0)), Point::new(mm(17.0), mm(11.0))));
    assert!(kb.warnings.iter().all(|w| !w.contains("rule area inside")));
}

#[test]
fn errors() {
    assert!(matches!(
        read_board("(kicad_pcb", &ReadOptions::default()),
        Err(KicadError::Parse { .. })
    ));
    assert!(matches!(read_board("(foo)", &ReadOptions::default()), Err(KicadError::Format(_))));
    assert!(
        matches!(read_board("(kicad_pcb (version 1))", &ReadOptions::default()), Err(KicadError::Format(m)) if m.contains("layers"))
    );
    let no_edge = format!("(kicad_pcb (version 1) {LAYERS})");
    assert!(
        matches!(read_board(&no_edge, &ReadOptions::default()), Err(KicadError::Format(m)) if m.contains("Edge.Cuts"))
    );
    let bad_num = board_src(
        "1",
        LAYERS,
        r#"(segment (start 1 x) (end 3 1) (width 0.2) (layer "F.Cu") (net 0))"#,
    );
    assert!(matches!(read_board(&bad_num, &ReadOptions::default()), Err(KicadError::Format(_))));
    let mut b = Board::default();
    assert!(matches!(read_project_rules("{", &mut b), Err(KicadError::Json(_))));
    assert!(matches!(read_project_rules("[]", &mut b), Err(KicadError::Json(_))));
    assert!(matches!(
        ferroute_kicad::read_board_file(std::path::Path::new("/nonexistent/x.kicad_pcb")),
        Err(KicadError::Io(_))
    ));
}

#[test]
fn project_patterns_and_inheritance() {
    let mut b = read(
        r#"(footprint "L:X" (layer "F.Cu") (at 1 1) (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "N1")))"#,
    )
    .board;
    b.nets.push(ferroute_board::Net { name: "+5V_USB".into(), class: 0 });
    b.nets.push(ferroute_board::Net { name: "CLK".into(), class: 0 });
    let json = r#"{
      "board": {"design_settings": {"rules": {"min_hole_to_hole": 0.3, "min_through_hole_diameter": 0.2}}},
      "net_settings": {
        "classes": [
          {"name": "Default", "clearance": 0.15, "track_width": 0.2, "via_diameter": 0.5, "via_drill": 0.25, "priority": 2147483647},
          {"name": "Power", "priority": 1, "track_width": 0.6},
          {"name": "Fast", "priority": 0, "clearance": 0.3}],
        "netclass_assignments": {"N1": "Fast", "CLK": ["Fast", "Power"]},
        "netclass_patterns": [{"pattern": "+5V*", "netclass": "Power"}, {"pattern": "*", "netclass": "Fast"}]
      }}"#;
    read_project_rules(json, &mut b).unwrap();
    let r = &b.rules;
    assert_eq!(
        r.classes.iter().map(|c| c.name.as_str()).collect::<Vec<_>>(),
        ["Default", "Power", "Fast"]
    );
    // inheritance of missing keys from Default
    assert_eq!(
        (r.classes[1].clearance, r.classes[1].track_width, r.classes[1].via_drill),
        (mm(0.15), mm(0.6), mm(0.25))
    );
    assert_eq!(r.classes[2].track_width, mm(0.2));
    assert_eq!((r.hole_to_hole, r.min_via_drill), (mm(0.3), mm(0.2)));
    // priority: lower number wins (Fast=0 beats Power=1 for "+5V_USB" matching both patterns)
    assert_eq!(b.nets[1].class, 2);
    assert_eq!(b.nets[2].class, 2);
    assert_eq!(b.nets[3].class, 2);
    assert_eq!(b.nets[0].class, 0);
}
