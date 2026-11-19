// SPDX-License-Identifier: AGPL-3.0-only
//! Graphics flattening and `Edge.Cuts` outline assembly.

use crate::KicadError;
use crate::sexpr::SExpr;
use crate::util::{ArcGeom, arc_from_three, dist, mm};
use ferroute_board::{Outline, Placement};
use ferroute_geom::{Coord, Point, Polygon};
use std::collections::HashMap;

/// Endpoint matching tolerance when chaining outline pieces (nm).
const CHAIN_TOL: f64 = 10.0;
/// Maximum sagitta of flattened arcs (nm).
pub(crate) const MAX_SAG: f64 = 1000.0;

/// A flattened graphic item in its own coordinate frame.
#[derive(Clone, Debug)]
pub(crate) enum Gfx {
    Line(Point, Point),
    Rect(Point, Point),
    Poly(Vec<Point>),
    /// Centre and a point on the circle.
    Circle(Point, Point),
    Arc(Point, Point, Point),
    /// KiCad <= 6 arc: centre, start point and sweep in degrees (direction unverified, so
    /// only the full circle is ever used from it).
    ArcLegacy(Point, Point),
    Curve([Point; 4]),
}

fn pt(n: &SExpr) -> Result<Point, KicadError> {
    let (Some(x), Some(y)) = (n.atom(1), n.atom(2)) else {
        return Err(KicadError::Format(format!("malformed ({}) node", n.head().unwrap_or("?"))));
    };
    Ok(Point::new(mm(x)?, mm(y)?))
}

pub(crate) fn point_child(n: &SExpr, name: &str) -> Result<Point, KicadError> {
    n.child(name)
        .ok_or_else(|| {
            KicadError::Format(format!("({}) without ({name})", n.head().unwrap_or("?")))
        })
        .and_then(pt)
}

/// Flattens the `(pts ...)` node of a polygon: `(xy x y)` and `(arc (start) (mid) (end))`.
pub(crate) fn poly_points(pts: &SExpr) -> Result<Vec<Point>, KicadError> {
    let mut out = Vec::new();
    for it in pts.items().unwrap_or(&[]).iter().skip(1) {
        match it.head() {
            Some("xy") => out.push(pt(it)?),
            Some("arc") => {
                let (s, m, e) =
                    (point_child(it, "start")?, point_child(it, "mid")?, point_child(it, "end")?);
                out.extend(flatten_arc(s, m, e, 0.0).0);
            }
            _ => {}
        }
    }
    Ok(out)
}

/// Flattens an arc into points including both exact endpoints. Interior vertices are
/// pushed radially by `outset` nm. Returns the points and the sagitta actually used.
pub(crate) fn flatten_arc(s: Point, m: Point, e: Point, outset: f64) -> (Vec<Point>, f64) {
    let Some(g) = arc_from_three(s, m, e) else {
        return (vec![s, e], 0.0);
    };
    let n = g.segments(MAX_SAG * 0.5);
    let mut v = Vec::with_capacity(n + 1);
    v.push(s);
    for k in 1..n {
        v.push(g.point(k, n, outset));
    }
    v.push(e);
    (v, g.sagitta(n))
}

/// Parses a `fp_*`/`gr_*` graphic node. Returns `Ok(None)` for kinds that are not
/// geometry we care about.
pub(crate) fn parse_gfx(n: &SExpr) -> Result<Option<Gfx>, KicadError> {
    let Some(head) = n.head() else { return Ok(None) };
    let kind = head.strip_prefix("fp_").or_else(|| head.strip_prefix("gr_")).unwrap_or("");
    Ok(Some(match kind {
        "line" => Gfx::Line(point_child(n, "start")?, point_child(n, "end")?),
        "rect" => Gfx::Rect(point_child(n, "start")?, point_child(n, "end")?),
        "poly" => {
            let pts =
                n.child("pts").ok_or_else(|| KicadError::Format("poly without pts".into()))?;
            Gfx::Poly(poly_points(pts)?)
        }
        "circle" => Gfx::Circle(point_child(n, "center")?, point_child(n, "end")?),
        "arc" => {
            if n.child("mid").is_some() {
                Gfx::Arc(point_child(n, "start")?, point_child(n, "mid")?, point_child(n, "end")?)
            } else {
                Gfx::ArcLegacy(point_child(n, "start")?, point_child(n, "end")?)
            }
        }
        "curve" => {
            let pts =
                n.child("pts").ok_or_else(|| KicadError::Format("curve without pts".into()))?;
            let v: Vec<Point> = pts.children("xy").map(pt).collect::<Result<_, _>>()?;
            let [a, b, c, d] = v[..] else {
                return Err(KicadError::Format("curve needs 4 points".into()));
            };
            Gfx::Curve([a, b, c, d])
        }
        _ => return Ok(None),
    }))
}

/// Arc metadata used to bias flattened vertices towards the conservative side.
#[derive(Copy, Clone, Debug)]
struct ArcInfo {
    cx: f64,
    cy: f64,
    r: f64,
    sag_up: f64,
}

#[derive(Clone, Debug)]
struct Piece {
    pts: Vec<Point>,
    /// Arc index for interior arc vertices (`None` for exact vertices).
    tag: Vec<Option<usize>>,
    closed: bool,
}

/// Collects `Edge.Cuts` shapes and assembles them into an [`Outline`].
#[derive(Default, Debug)]
pub(crate) struct EdgeBuilder {
    pieces: Vec<Piece>,
    arcs: Vec<ArcInfo>,
    pub warnings: Vec<String>,
}

#[allow(clippy::cast_precision_loss)]
fn pf(p: Point) -> (f64, f64) {
    (p.x as f64, p.y as f64)
}

impl EdgeBuilder {
    fn xf(pl: Option<Placement>, p: Point) -> Point {
        pl.map_or(p, |pl| pl.apply(p))
    }

    fn add_arc_piece(
        &mut self,
        g: &ArcGeom,
        s: Point,
        e: Point,
        pl: Option<Placement>,
        closed: bool,
    ) {
        let n = g.segments(MAX_SAG * 0.5);
        let idx = self.arcs.len();
        let c = Self::xf(pl, Point::new(rnd(g.cx), rnd(g.cy)));
        let (cx, cy) = pf(c);
        #[allow(clippy::cast_possible_truncation)]
        self.arcs.push(ArcInfo { cx, cy, r: g.r, sag_up: g.sagitta(n).ceil() + 1.0 });
        let mut pts = vec![Self::xf(pl, s)];
        // A full circle has no exact endpoint to preserve: bias every vertex.
        let mut tag = vec![if closed { Some(idx) } else { None }];
        for k in 1..n {
            pts.push(Self::xf(pl, g.point(k, n, 0.0)));
            tag.push(Some(idx));
        }
        if !closed {
            pts.push(Self::xf(pl, e));
            tag.push(None);
        }
        self.pieces.push(Piece { pts, tag, closed });
    }

    /// Adds one `Edge.Cuts` graphic; `pl` maps footprint-local to board coordinates.
    pub(crate) fn add(&mut self, g: &Gfx, pl: Option<Placement>) {
        match g {
            Gfx::Line(a, b) => self.push_open(vec![Self::xf(pl, *a), Self::xf(pl, *b)]),
            Gfx::Rect(a, b) => {
                let r = ferroute_geom::Rect::new(*a, *b);
                self.push_closed(r.corners().iter().map(|&p| Self::xf(pl, p)).collect());
            }
            Gfx::Poly(v) => self.push_closed(v.iter().map(|&p| Self::xf(pl, p)).collect()),
            Gfx::Circle(c, e) => {
                let r = dist(*c, *e);
                if r < 1.0 {
                    return;
                }
                let g = ArcGeom {
                    cx: pf(*c).0,
                    cy: pf(*c).1,
                    r,
                    a0: 0.0,
                    sweep: std::f64::consts::TAU,
                };
                let s = Point::new(c.x + rnd(r), c.y);
                self.add_arc_piece(&g, s, s, pl, true);
            }
            Gfx::Arc(s, m, e) => match arc_from_three(*s, *m, *e) {
                Some(g) => self.add_arc_piece(&g, *s, *e, pl, false),
                None => self.push_open(vec![Self::xf(pl, *s), Self::xf(pl, *e)]),
            },
            Gfx::ArcLegacy(..) => {
                self.warnings.push("legacy (pre-KiCad 7) Edge.Cuts arc ignored".into());
            }
            Gfx::Curve(c) => {
                let n = 48usize;
                let mut pts = Vec::with_capacity(n + 1);
                for k in 0..=n {
                    #[allow(clippy::cast_precision_loss)]
                    let t = k as f64 / n as f64;
                    let u = 1.0 - t;
                    let w = [u * u * u, 3.0 * u * u * t, 3.0 * u * t * t, t * t * t];
                    let x: f64 = c.iter().zip(w).map(|(p, w)| pf(*p).0 * w).sum();
                    let y: f64 = c.iter().zip(w).map(|(p, w)| pf(*p).1 * w).sum();
                    pts.push(Self::xf(pl, Point::new(rnd(x), rnd(y))));
                }
                self.warnings.push("Edge.Cuts bezier curve approximated by 48 chords".into());
                self.push_open(pts);
            }
        }
    }

    fn push_open(&mut self, pts: Vec<Point>) {
        let tag = vec![None; pts.len()];
        self.pieces.push(Piece { pts, tag, closed: false });
    }

    fn push_closed(&mut self, mut pts: Vec<Point>) {
        if pts.len() > 1 && pts.first() == pts.last() {
            pts.pop();
        }
        let tag = vec![None; pts.len()];
        self.pieces.push(Piece { pts, tag, closed: true });
    }

    /// Chains the pieces into loops and classifies them. The loop with the largest
    /// absolute area is the board boundary, the rest are cut-outs.
    pub(crate) fn finish(mut self) -> Result<(Outline, Vec<String>), KicadError> {
        let loops = self.chain();
        let mut polys: Vec<(Polygon, Vec<Option<usize>>)> = Vec::new();
        for (pts, tag) in loops {
            let poly = Polygon::new(pts);
            if poly.pts.len() < 3 || poly.area2() == 0 {
                self.warnings.push("degenerate Edge.Cuts loop ignored".into());
                continue;
            }
            polys.push((poly, tag));
        }
        let Some(outer_i) = (0..polys.len()).max_by_key(|&i| polys[i].0.area2().unsigned_abs())
        else {
            return Err(KicadError::Format("no closed Edge.Cuts outline found".into()));
        };
        let mut outer = Polygon::default();
        let mut holes = Vec::new();
        for (i, (poly, tag)) in polys.iter().enumerate() {
            let is_hole = i != outer_i;
            let mut p = self.biased(poly, tag, is_hole);
            if p.area2() < 0 {
                p.pts.reverse();
            }
            if is_hole {
                holes.push(p);
            } else {
                outer = p;
            }
        }
        Ok((Outline { outer, holes }, self.warnings))
    }

    /// Pushes arc vertices to the conservative side: holes may only grow, the outer
    /// boundary may only shrink (error bound: the flattening sagitta).
    fn biased(&self, poly: &Polygon, tag: &[Option<usize>], is_hole: bool) -> Polygon {
        let mut convex: HashMap<usize, bool> = HashMap::new();
        let mut out = poly.clone();
        for (i, t) in tag.iter().enumerate() {
            let Some(a) = *t else { continue };
            let info = self.arcs[a];
            let v = poly.pts[i];
            let (vx, vy) = pf(v);
            let d = ferroute_geom::detmath::hypot(vx - info.cx, vy - info.cy);
            if d < 1.0 {
                continue;
            }
            let is_convex = *convex.entry(a).or_insert_with(|| {
                if info.r <= 5000.0 {
                    return true;
                }
                let k = (info.r - 5000.0) / d;
                let probe = Point::new(
                    rnd(info.cx + (vx - info.cx) * k),
                    rnd(info.cy + (vy - info.cy) * k),
                );
                poly.contains(probe)
            });
            // hole & convex: grow; outer & concave: grow away from the centre.
            if is_hole == is_convex {
                let k = (d + info.sag_up) / d;
                out.pts[i] = Point::new(
                    rnd(info.cx + (vx - info.cx) * k),
                    rnd(info.cy + (vy - info.cy) * k),
                );
            }
        }
        out
    }

    #[allow(clippy::type_complexity)]
    fn chain(&mut self) -> Vec<(Vec<Point>, Vec<Option<usize>>)> {
        let mut loops = Vec::new();
        let pieces = std::mem::take(&mut self.pieces);
        let mut used = vec![false; pieces.len()];
        // Bucket index of piece endpoints.
        let cell = |p: Point| (p.x.div_euclid(16), p.y.div_euclid(16));
        let mut buckets: HashMap<(Coord, Coord), Vec<(usize, bool)>> = HashMap::new();
        for (i, p) in pieces.iter().enumerate() {
            if p.closed || p.pts.len() < 2 {
                continue;
            }
            buckets.entry(cell(p.pts[0])).or_default().push((i, false));
            buckets.entry(cell(p.pts[p.pts.len() - 1])).or_default().push((i, true));
        }
        for i in 0..pieces.len() {
            if used[i] {
                continue;
            }
            used[i] = true;
            let p = &pieces[i];
            if p.closed {
                loops.push((p.pts.clone(), p.tag.clone()));
                continue;
            }
            if p.pts.len() < 2 {
                continue;
            }
            let (mut pts, mut tag) = (p.pts.clone(), p.tag.clone());
            let mut flipped = false;
            let mut closed = false;
            loop {
                let last = pts[pts.len() - 1];
                if pts.len() > 2 && dist(pts[0], last) <= CHAIN_TOL {
                    pts.pop();
                    tag.pop();
                    closed = true;
                    break;
                }
                let (cx, cy) = cell(last);
                let mut best: Option<(usize, bool, f64)> = None;
                for dx in -1..=1 {
                    for dy in -1..=1 {
                        for &(j, is_end) in buckets.get(&(cx + dx, cy + dy)).into_iter().flatten() {
                            if used[j] {
                                continue;
                            }
                            let q = &pieces[j];
                            let e = if is_end { q.pts[q.pts.len() - 1] } else { q.pts[0] };
                            let d = dist(last, e);
                            if d <= CHAIN_TOL && best.is_none_or(|b| d < b.2) {
                                best = Some((j, is_end, d));
                            }
                        }
                    }
                }
                match best {
                    Some((j, is_end, _)) => {
                        used[j] = true;
                        let q = &pieces[j];
                        let (mut qp, mut qt) = (q.pts.clone(), q.tag.clone());
                        if is_end {
                            qp.reverse();
                            qt.reverse();
                        }
                        // The shared junction point is taken from the existing chain.
                        pts.extend(qp.into_iter().skip(1));
                        tag.extend(qt.into_iter().skip(1));
                    }
                    None if !flipped => {
                        pts.reverse();
                        tag.reverse();
                        flipped = true;
                    }
                    None => break,
                }
            }
            if !closed {
                self.warnings.push("open Edge.Cuts chain closed implicitly".into());
            }
            loops.push((pts, tag));
        }
        loops
    }
}

#[allow(clippy::cast_possible_truncation)]
fn rnd(v: f64) -> Coord {
    v.round() as Coord
}
