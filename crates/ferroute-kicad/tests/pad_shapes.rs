// SPDX-License-Identifier: AGPL-3.0-only
//! Pad copper shape parsing: every shape token, custom primitives and padstacks.
mod common;

use common::mm;
use ferroute_board::PadShape;
use ferroute_kicad::{KicadBoard, ReadOptions, read_board};

fn read_pad(rest: &str) -> KicadBoard {
    let src = format!(
        r#"(kicad_pcb (version 20211014) (generator pcbnew) (general (thickness 1.6)) (paper "A4")
        (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user)) (net 0 "") (net 1 "N1")
        (footprint "L:X" (layer "F.Cu") (at 10 10) (fp_text reference "X1" (at 0 0) (layer "F.SilkS"))
          (pad "1" {rest}))
        (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts") (width 0.1)))"#
    );
    read_board(&src, &ReadOptions::default()).unwrap()
}

fn shape(rest: &str) -> PadShape {
    read_pad(rest).board.pads[0].shape.clone()
}

fn custom(anchor: &str, size: &str, prims: &str) -> PadShape {
    shape(&format!(
        r#"smd custom (at 0 0) (size {size}) (layers "F.Cu") (options (clearance outline) (anchor {anchor})) (primitives {prims})"#
    ))
}

#[test]
fn circle_uses_larger_dimension() {
    assert_eq!(
        shape(r#"smd circle (at 0 0) (size 1.5 1) (layers "F.Cu")"#),
        PadShape::Circle { diameter: mm(1.5) }
    );
    // size with a single value: height defaults to width
    assert_eq!(
        shape(r#"smd circle (at 0 0) (size 0.8) (layers "F.Cu")"#),
        PadShape::Circle { diameter: mm(0.8) }
    );
}

#[test]
fn rect_and_oval_keep_their_size() {
    assert_eq!(
        shape(r#"smd rect (at 0 0) (size 2 1) (layers "F.Cu")"#),
        PadShape::Rect { w: mm(2.0), h: mm(1.0) }
    );
    assert_eq!(
        shape(r#"smd oval (at 0 0) (size 2 1) (layers "F.Cu")"#),
        PadShape::Oval { w: mm(2.0), h: mm(1.0) }
    );
}

#[test]
fn missing_size_is_zero() {
    assert_eq!(shape(r#"smd rect (at 0 0) (layers "F.Cu")"#), PadShape::Rect { w: 0, h: 0 });
}

#[test]
fn roundrect_default_ratio_and_clamp() {
    // default ratio 0.25 of the short side
    assert_eq!(
        shape(r#"smd roundrect (at 0 0) (size 2 1) (layers "F.Cu")"#),
        PadShape::RoundRect { w: mm(2.0), h: mm(1.0), radius: mm(0.25) }
    );
    // ratio above 0.5 is clamped to half the short side
    assert_eq!(
        shape(r#"smd roundrect (at 0 0) (size 2 1) (layers "F.Cu") (roundrect_rratio 0.9)"#),
        PadShape::RoundRect { w: mm(2.0), h: mm(1.0), radius: mm(0.5) }
    );
    // zero ratio
    assert_eq!(
        shape(r#"smd roundrect (at 0 0) (size 2 1) (layers "F.Cu") (roundrect_rratio 0)"#),
        PadShape::RoundRect { w: mm(2.0), h: mm(1.0), radius: 0 }
    );
}

#[test]
fn chamfer_without_ratio_is_plain_rect() {
    assert_eq!(
        shape(r#"smd roundrect (at 0 0) (size 2 1) (layers "F.Cu") (chamfer top_left)"#),
        PadShape::Rect { w: mm(2.0), h: mm(1.0) }
    );
}

#[test]
fn trapezoid_without_delta_is_a_rectangle_hull() {
    let PadShape::Convex { pts } = shape(r#"smd trapezoid (at 0 0) (size 2 1) (layers "F.Cu")"#)
    else {
        panic!("expected convex")
    };
    assert_eq!(pts.len(), 4);
    let b = PadShape::Convex { pts }.local_hull().bbox();
    assert_eq!((b.min.x, b.max.x, b.min.y, b.max.y), (mm(-1.0), mm(1.0), mm(-0.5), mm(0.5)));
}

#[test]
fn trapezoid_with_both_deltas() {
    let PadShape::Convex { pts } =
        shape(r#"smd trapezoid (at 0 0) (size 2 2) (rect_delta 0.4 0.2) (layers "F.Cu")"#)
    else {
        panic!("expected convex")
    };
    assert!(pts.len() >= 3);
}

#[test]
fn degenerate_trapezoid_falls_back_to_rect() {
    assert_eq!(
        shape(r#"smd trapezoid (at 0 0) (size 0 0) (layers "F.Cu")"#),
        PadShape::Rect { w: 0, h: 0 }
    );
}

#[test]
fn unknown_shape_warns_and_uses_bounding_rect() {
    let kb = read_pad(r#"smd hexagon (at 0 0) (size 2 1) (layers "F.Cu")"#);
    assert_eq!(kb.board.pads[0].shape, PadShape::Rect { w: mm(2.0), h: mm(1.0) });
    assert!(kb.warnings.iter().any(|w| w.contains("unknown pad shape")), "{:?}", kb.warnings);
}

#[test]
fn custom_without_primitives_is_anchor_polygon() {
    let PadShape::Convex { pts } = custom("circle", "1 1", "") else { panic!("expected convex") };
    let b = PadShape::Convex { pts }.local_hull().bbox();
    assert!(b.max.x >= mm(0.5) && b.max.x < mm(0.52), "{b:?}");
}

#[test]
fn custom_rect_anchor_covers_rectangle() {
    let PadShape::Convex { pts } = custom("rect", "2 1", "") else { panic!("expected convex") };
    let b = PadShape::Convex { pts }.local_hull().bbox();
    assert_eq!((b.min.x, b.max.x, b.min.y, b.max.y), (mm(-1.0), mm(1.0), mm(-0.5), mm(0.5)));
}

#[test]
fn custom_zero_size_without_primitives_is_a_tiny_convex_shape() {
    let PadShape::Convex { pts } = custom("circle", "0 0", "") else { panic!("expected convex") };
    assert!(pts.len() >= 3);
    let b = PadShape::Convex { pts }.local_hull().bbox();
    assert!(b.max.x <= 2 && b.max.y <= 2, "{b:?}");
}

#[test]
fn custom_line_and_vector_primitives() {
    let s = custom(
        "circle",
        "0.2 0.2",
        "(gr_line (start 0 0) (end 3 0) (width 0.2)) (gr_vector (start 0 0) (end 0 2))",
    );
    let b = s.local_hull().bbox();
    assert!(b.max.x >= mm(3.1) && b.max.x < mm(3.12), "{b:?}");
    assert!(b.max.y >= mm(2.0) && b.max.y < mm(2.12), "{b:?}");
}

#[test]
fn custom_rect_and_bbox_primitives_with_stroke_node() {
    let s = custom(
        "circle",
        "0.2 0.2",
        "(gr_rect (start 0 0) (end 2 1) (stroke (width 0.2) (type solid)) (fill yes)) \
         (gr_bbox (start -1 -1) (end 0 0))",
    );
    let b = s.local_hull().bbox();
    assert!(b.max.x >= mm(2.1) && b.max.y >= mm(1.1), "{b:?}");
    assert!(b.min.x <= mm(-1.0) && b.min.y <= mm(-1.0), "{b:?}");
}

#[test]
fn custom_circle_primitive_includes_radius_and_stroke() {
    let s = custom("circle", "0.2 0.2", "(gr_circle (center 2 0) (end 3 0) (width 0.2))");
    let b = s.local_hull().bbox();
    assert!(b.max.x >= mm(3.1) && b.max.x < mm(3.2), "{b:?}");
}

#[test]
fn custom_arc_primitive_uses_circumcircle() {
    let s = custom("circle", "0.2 0.2", "(gr_arc (start 1 0) (mid 0 1) (end -1 0) (width 0.1))");
    let b = s.local_hull().bbox();
    assert!(b.max.y >= mm(1.05), "{b:?}");
    assert!(b.min.y <= mm(-1.0), "whole circumcircle is covered: {b:?}");
}

#[test]
fn custom_collinear_arc_degrades_to_endpoints() {
    let s = custom("circle", "0.2 0.2", "(gr_arc (start 0 0) (mid 1 0) (end 2 0) (width 0.1))");
    let b = s.local_hull().bbox();
    assert!(b.max.x >= mm(2.0) && b.max.x < mm(2.1), "{b:?}");
}

#[test]
fn custom_curve_and_poly_primitives() {
    let s = custom(
        "circle",
        "0.2 0.2",
        "(gr_curve (pts (xy 0 0) (xy 1 2) (xy 2 2) (xy 3 0)) (width 0.1)) \
         (gr_poly (pts (xy 0 0) (xy -2 0) (xy -2 -1)) (width 0.1) (fill yes))",
    );
    let b = s.local_hull().bbox();
    assert!(b.max.x >= mm(3.0) && b.max.y >= mm(2.0), "{b:?}");
    assert!(b.min.x <= mm(-2.0) && b.min.y <= mm(-1.0), "{b:?}");
}

#[test]
fn custom_ignores_unknown_primitives_and_incomplete_ones() {
    let s = custom(
        "circle",
        "1 1",
        "(gr_text \"x\") (gr_rect (start 0 0)) (gr_circle (center 1 1)) (gr_arc (start 0 0) (end 1 1))",
    );
    let b = s.local_hull().bbox();
    assert!(b.max.x < mm(0.6) && b.max.y < mm(0.6), "{b:?}");
}

#[test]
fn offset_forms_translate_the_shape() {
    // bare (offset x y)
    let kb = read_pad(r#"smd rect (at 0 0) (size 2 2) (offset 1 0.5) (layers "F.Cu")"#);
    let b = kb.board.pads[0].shape.local_hull().bbox();
    assert_eq!((b.min.x, b.max.x, b.min.y, b.max.y), (mm(0.0), mm(2.0), mm(-0.5), mm(1.5)));
    // zero offset leaves the exact shape alone
    let s = shape(r#"smd rect (at 0 0) (size 2 2) (offset 0 0) (layers "F.Cu")"#);
    assert_eq!(s, PadShape::Rect { w: mm(2.0), h: mm(2.0) });
    // offset with a single coordinate: y defaults to 0
    let kb = read_pad(r#"smd rect (at 0 0) (size 2 2) (offset 1) (layers "F.Cu")"#);
    let b = kb.board.pads[0].shape.local_hull().bbox();
    assert_eq!((b.min.y, b.max.y), (mm(-1.0), mm(1.0)));
}

#[test]
fn padstack_same_kind_takes_per_dimension_maximum() {
    let kb = read_pad(
        r#"thru_hole rect (at 0 0) (size 1 1) (drill 0.5) (layers "*.Cu") (padstack (mode front_inner_back) (layer "F.Cu" (shape rect) (size 1.5 0.5)))"#,
    );
    assert_eq!(kb.board.pads[0].shape, PadShape::Rect { w: mm(1.5), h: mm(1.0) });
    assert!(kb.warnings.iter().any(|w| w.contains("padstack")));
}

#[test]
fn padstack_identical_layers_stay_silent() {
    let kb = read_pad(
        r#"smd oval (at 0 0) (size 2 1) (layers "F.Cu") (padstack (layer "F.Cu" (shape oval) (size 2 1)) (layer "B.Cu" (shape oval) (size 1 1)))"#,
    );
    assert_eq!(kb.board.pads[0].shape, PadShape::Oval { w: mm(2.0), h: mm(1.0) });
    assert!(kb.warnings.iter().all(|w| !w.contains("padstack")), "{:?}", kb.warnings);
}

#[test]
fn padstack_circle_and_roundrect_merge_in_kind() {
    let kb = read_pad(
        r#"smd circle (at 0 0) (size 1 1) (layers "F.Cu") (padstack (layer "F.Cu" (shape circle) (size 1.4 1.2)))"#,
    );
    assert_eq!(kb.board.pads[0].shape, PadShape::Circle { diameter: mm(1.4) });
    let kb = read_pad(
        r#"smd roundrect (at 0 0) (size 2 1) (layers "F.Cu") (roundrect_rratio 0.25) (padstack (layer "F.Cu" (shape roundrect) (size 3 1)))"#,
    );
    assert_eq!(
        kb.board.pads[0].shape,
        PadShape::RoundRect { w: mm(3.0), h: mm(1.0), radius: mm(0.25) }
    );
}

#[test]
fn padstack_layer_without_shape_name_keeps_kind() {
    let kb = read_pad(
        r#"smd rect (at 0 0) (size 1 1) (layers "F.Cu") (padstack (layer "F.Cu" (size 2 2)))"#,
    );
    assert_eq!(kb.board.pads[0].shape, PadShape::Rect { w: mm(2.0), h: mm(2.0) });
}

#[test]
fn padstack_layer_without_size_is_skipped() {
    let kb = read_pad(
        r#"smd rect (at 0 0) (size 1 1) (layers "F.Cu") (padstack (layer "F.Cu" (shape rect)))"#,
    );
    assert_eq!(kb.board.pads[0].shape, PadShape::Rect { w: mm(1.0), h: mm(1.0) });
}

#[test]
fn padstack_different_kind_becomes_bounding_rect() {
    let kb = read_pad(
        r#"smd circle (at 0 0) (size 1 1) (layers "F.Cu") (padstack (layer "F.Cu" (shape rect) (size 2 1)))"#,
    );
    assert_eq!(kb.board.pads[0].shape, PadShape::Rect { w: mm(2.0), h: mm(1.0) });
    assert!(kb.warnings.iter().any(|w| w.contains("padstack")));
}

#[test]
fn padstack_on_convex_pad_with_unnamed_layer_becomes_rect() {
    let kb = read_pad(
        r#"smd trapezoid (at 0 0) (size 2 1) (rect_delta 0 0.4) (layers "F.Cu") (padstack (layer "F.Cu" (size 3 3)))"#,
    );
    assert_eq!(kb.board.pads[0].shape, PadShape::Rect { w: mm(3.0), h: mm(3.0) });
}

#[test]
fn padstack_on_custom_pad_keeps_convex_when_same_named() {
    let kb = read_pad(
        r#"smd custom (at 0 0) (size 1 1) (layers "F.Cu") (options (anchor circle)) (primitives (gr_line (start 0 0) (end 1 0) (width 0.1))) (padstack (layer "F.Cu" (shape circle) (size 0.5 0.5)))"#,
    );
    assert!(matches!(kb.board.pads[0].shape, PadShape::Rect { .. } | PadShape::Convex { .. }));
}
