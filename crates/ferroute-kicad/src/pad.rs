// SPDX-License-Identifier: AGPL-3.0-only
//! Pad copper shapes: KiCad shape tokens to conservative [`PadShape`]s.
//!
//! Every approximation errs on the *larger* side (the produced shape is a superset of the
//! true copper) so that a clean DRC on the model implies a clean DRC in KiCad.

use crate::KicadError;
use crate::outline::{point_child, poly_points};
use crate::sexpr::SExpr;
use crate::util::{circumscribed_polygon, float, mm};
use ferroute_board::PadShape;
use ferroute_geom::{Coord, Point, Rect, convex_hull};

fn half_up(v: Coord) -> Coord {
    (v + 1) / 2
}

fn size_of(node: &SExpr) -> Result<(Coord, Coord), KicadError> {
    let s = node.child("size");
    let w = s.and_then(|s| s.atom(1)).map(mm).transpose()?.unwrap_or(0);
    let h = s.and_then(|s| s.atom(2)).map(mm).transpose()?.unwrap_or(w);
    Ok((w, h))
}

fn stroke_width(n: &SExpr) -> Result<Coord, KicadError> {
    if let Some(w) = n.child("width").and_then(|w| w.atom(1)) {
        return mm(w);
    }
    if let Some(w) = n.child("stroke").and_then(|s| s.child("width")).and_then(|w| w.atom(1)) {
        return mm(w);
    }
    Ok(0)
}

fn add_point(out: &mut Vec<Point>, p: Point, half_w: Coord) {
    if half_w > 0 {
        #[allow(clippy::cast_precision_loss)]
        out.extend(circumscribed_polygon(p.x as f64, p.y as f64, half_w as f64, 16));
    } else {
        out.push(p);
    }
}

/// Convex superset of a custom pad: anchor plus all primitives (stroke width included).
fn custom_hull(node: &SExpr, w: Coord, h: Coord) -> Vec<Point> {
    let mut pts: Vec<Point> = Vec::new();
    let anchor_rect = node
        .child("options")
        .and_then(|o| o.child("anchor"))
        .and_then(|a| a.atom(1))
        .is_some_and(|a| a == "rect");
    if anchor_rect {
        let (hw, hh) = (half_up(w), half_up(h));
        pts.extend(Rect::new(Point::new(-hw, -hh), Point::new(hw, hh)).corners());
    } else {
        #[allow(clippy::cast_precision_loss)]
        pts.extend(circumscribed_polygon(0.0, 0.0, w.max(h) as f64 / 2.0, 16));
    }
    for prim in node.child("primitives").and_then(SExpr::items).unwrap_or(&[]).iter().skip(1) {
        let half = half_up(stroke_width(prim).unwrap_or(0));
        let get = |name: &str| point_child(prim, name).ok();
        match prim.head() {
            Some("gr_line" | "gr_vector") => {
                for k in ["start", "end"] {
                    if let Some(p) = get(k) {
                        add_point(&mut pts, p, half);
                    }
                }
            }
            Some("gr_rect" | "gr_bbox") => {
                if let (Some(a), Some(b)) = (get("start"), get("end")) {
                    for c in Rect::new(a, b).corners() {
                        add_point(&mut pts, c, half);
                    }
                }
            }
            Some("gr_circle") => {
                if let (Some(c), Some(e)) = (get("center"), get("end")) {
                    #[allow(clippy::cast_precision_loss)]
                    let r = c.dist(e) + half as f64;
                    #[allow(clippy::cast_precision_loss)]
                    pts.extend(circumscribed_polygon(c.x as f64, c.y as f64, r, 16));
                }
            }
            Some("gr_arc") => {
                if let (Some(s), Some(m), Some(e)) = (get("start"), get("mid"), get("end")) {
                    // Every point of the arc is within the circle through the three points.
                    match crate::util::arc_from_three(s, m, e) {
                        Some(g) => {
                            #[allow(clippy::cast_precision_loss)]
                            pts.extend(circumscribed_polygon(g.cx, g.cy, g.r + half as f64, 16));
                        }
                        None => {
                            add_point(&mut pts, s, half);
                            add_point(&mut pts, e, half);
                        }
                    }
                }
            }
            Some("gr_curve" | "gr_poly") => {
                if let Some(p) = prim.child("pts") {
                    let mut v: Vec<Point> = poly_points(p).unwrap_or_default();
                    // Bezier control points enclose the curve (convex hull property).
                    v.extend(p.children("xy").filter_map(|n| {
                        Some(Point::new(mm(n.atom(1)?).ok()?, mm(n.atom(2)?).ok()?))
                    }));
                    for q in v {
                        add_point(&mut pts, q, half);
                    }
                }
            }
            _ => {}
        }
    }
    pts
}

/// Builds the pad copper shape from a `(pad ...)` node (front/default padstack layer).
pub(crate) fn build_shape(
    node: &SExpr,
    warnings: &mut Vec<String>,
    label: &str,
) -> Result<PadShape, KicadError> {
    let name = node.atom(3).unwrap_or("rect");
    let (w, h) = size_of(node)?;
    let mut shape = match name {
        "circle" => PadShape::Circle { diameter: w.max(h) },
        "rect" => PadShape::Rect { w, h },
        "oval" => PadShape::Oval { w, h },
        "roundrect" => {
            if node.child("chamfer_ratio").is_some() || node.child("chamfer").is_some() {
                // Chamfered rounded rect: the plain rectangle is a superset.
                PadShape::Rect { w, h }
            } else {
                let ratio = node
                    .child("roundrect_rratio")
                    .and_then(|r| r.atom(1))
                    .map(float)
                    .transpose()?
                    .unwrap_or(0.25);
                let m = w.min(h);
                // Floor (never round up): a larger radius would shrink the corner.
                #[allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]
                let radius = ((ratio * m as f64 + 1e-4).floor() as Coord).clamp(0, m / 2);
                PadShape::RoundRect { w, h, radius }
            }
        }
        "trapezoid" => {
            let d = node.child("rect_delta");
            let dx = d.and_then(|d| d.atom(1)).map(mm).transpose()?.unwrap_or(0) / 2;
            let dy = d.and_then(|d| d.atom(2)).map(mm).transpose()?.unwrap_or(0) / 2;
            let (sx, sy) = (half_up(w), half_up(h));
            // TransformTrapezoidToPolygon (KiCad convert_basic_shapes_to_polygon.cpp)
            let corners = [
                Point::new(-sx + dy, -sy - dx),
                Point::new(sx - dy, -sy + dx),
                Point::new(sx + dy, sy - dx),
                Point::new(-sx - dy, sy + dx),
            ];
            let hull = convex_hull(&corners);
            if hull.len() >= 3 { PadShape::Convex { pts: hull } } else { PadShape::Rect { w, h } }
        }
        "custom" => {
            let hull = convex_hull(&custom_hull(node, w, h));
            if hull.len() >= 3 {
                PadShape::Convex { pts: hull }
            } else {
                PadShape::Circle { diameter: w.max(h).max(1) }
            }
        }
        other => {
            warnings.push(format!(
                "{label}: unknown pad shape {other:?}, using its bounding rectangle"
            ));
            PadShape::Rect { w, h }
        }
    };
    shape = apply_padstack(shape, node, warnings, label)?;
    Ok(shape)
}

fn kind_of(s: &PadShape) -> u8 {
    match s {
        PadShape::Circle { .. } => 0,
        PadShape::Rect { .. } => 1,
        PadShape::Oval { .. } => 2,
        PadShape::RoundRect { .. } => 3,
        PadShape::Convex { .. } => 4,
    }
}

fn dims(s: &PadShape) -> (Coord, Coord) {
    match s {
        PadShape::Circle { diameter } => (*diameter, *diameter),
        PadShape::Rect { w, h } | PadShape::Oval { w, h } | PadShape::RoundRect { w, h, .. } => {
            (*w, *h)
        }
        PadShape::Convex { .. } => {
            let b = s.local_hull().bbox();
            (b.width(), b.height())
        }
    }
}

/// KiCad 9+ complex padstacks (`front_inner_back` / `custom`) carry per-layer shapes.
/// The model has a single shape per pad, so we use the union (bounding rectangle) of all
/// layer shapes when they differ, and the per-dimension maximum otherwise.
fn apply_padstack(
    base: PadShape,
    node: &SExpr,
    warnings: &mut Vec<String>,
    label: &str,
) -> Result<PadShape, KicadError> {
    let Some(ps) = node.child("padstack") else { return Ok(base) };
    let mut shape = base;
    let mut note = false;
    for layer in ps.children("layer") {
        let Some(sz) = layer.child("size") else { continue };
        let w = sz.atom(1).map(mm).transpose()?.unwrap_or(0);
        let h = sz.atom(2).map(mm).transpose()?.unwrap_or(w);
        let lname = layer.child("shape").and_then(|s| s.atom(1)).unwrap_or("");
        let same_kind = match (&shape, lname) {
            (PadShape::Circle { .. }, "circle")
            | (PadShape::Rect { .. }, "rect")
            | (PadShape::Oval { .. }, "oval") => true,
            (PadShape::RoundRect { .. }, "roundrect") => true,
            (_, "") => kind_of(&shape) != 4,
            _ => false,
        };
        let (bw, bh) = dims(&shape);
        let (nw, nh) = (bw.max(w), bh.max(h));
        if (nw, nh) != (bw, bh) || !same_kind {
            note = true;
        }
        shape = if same_kind {
            match shape {
                PadShape::Circle { .. } => PadShape::Circle { diameter: nw.max(nh) },
                PadShape::Rect { .. } => PadShape::Rect { w: nw, h: nh },
                PadShape::Oval { .. } => PadShape::Oval { w: nw, h: nh },
                PadShape::RoundRect { radius, .. } => PadShape::RoundRect { w: nw, h: nh, radius },
                PadShape::Convex { pts } => PadShape::Convex { pts },
            }
        } else {
            PadShape::Rect { w: nw, h: nh }
        };
    }
    if note {
        warnings.push(format!("{label}: per-layer padstack shapes merged into one superset shape"));
    }
    Ok(shape)
}

/// Offset of the copper shape from the pad origin (`(drill ... (offset x y))` or a bare
/// `(offset x y)`).
pub(crate) fn shape_offset(node: &SExpr) -> Result<Point, KicadError> {
    let off = node.child("drill").and_then(|d| d.child("offset")).or_else(|| node.child("offset"));
    match off {
        Some(o) => Ok(Point::new(
            o.atom(1).map(mm).transpose()?.unwrap_or(0),
            o.atom(2).map(mm).transpose()?.unwrap_or(0),
        )),
        None => Ok(Point::ORIGIN),
    }
}

/// Re-centres a shape with a non-zero copper offset: the bounding box of the shape,
/// translated (a superset of the true offset shape).
pub(crate) fn offset_shape(shape: &PadShape, off: Point) -> PadShape {
    if off == Point::ORIGIN {
        return shape.clone();
    }
    let b = shape.local_hull().bbox().translate(off);
    PadShape::Convex { pts: b.corners().to_vec() }
}
