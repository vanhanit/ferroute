// SPDX-License-Identifier: AGPL-3.0-only
//! Circular arcs given KiCad-style by start, mid and end points (nm).
//!
//! Everything here is deterministic (INV-DET-3): circle fitting and chord bisection use only
//! IEEE `+ − × ÷ sqrt`, angles use [`crate::detmath::atan2`]. [`Arc::cover`] returns convex
//! capsules whose union is a *superset* of the arc's copper (INV-DRC-1), so the exact
//! clearance predicates stay conservative for curved copper.

use crate::detmath::{atan2, hypot};
use crate::{Coord, Hull, Point};

/// Circle in floating point (centre and radius, nm).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Circle {
    pub cx: f64,
    pub cy: f64,
    pub r: f64,
}

/// An arc from `start` through `mid` to `end`. `mid` must lie strictly inside the arc and
/// split it into two parts of less than 180° each (KiCad writes the true midpoint).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Arc {
    pub start: Point,
    pub mid: Point,
    pub end: Point,
}

#[allow(clippy::cast_precision_loss)]
fn pf(p: Point) -> (f64, f64) {
    (p.x as f64, p.y as f64)
}

impl Arc {
    #[must_use]
    pub const fn new(start: Point, mid: Point, end: Point) -> Self {
        Self { start, mid, end }
    }

    /// Circle through the three points; `None` when they are collinear or coincide.
    #[must_use]
    pub fn circle(&self) -> Option<Circle> {
        let (ax, ay) = pf(self.start);
        let (bx, by) = (pf(self.mid).0 - ax, pf(self.mid).1 - ay);
        let (cx, cy) = (pf(self.end).0 - ax, pf(self.end).1 - ay);
        let d = 2.0 * (bx * cy - by * cx);
        if d == 0.0 {
            return None;
        }
        let (b2, c2) = (bx * bx + by * by, cx * cx + cy * cy);
        let ux = (cy * b2 - by * c2) / d;
        let uy = (bx * c2 - cx * b2) / d;
        Some(Circle { cx: ax + ux, cy: ay + uy, r: hypot(ux, uy) })
    }

    /// Signed sweep (radians, positive = counter-clockwise in the maths frame, i.e.
    /// clockwise on a y-down screen); `None` for a degenerate arc.
    #[must_use]
    pub fn sweep(&self) -> Option<f64> {
        let c = self.circle()?;
        let ang = |p: Point, q: Point| {
            let (px, py) = (pf(p).0 - c.cx, pf(p).1 - c.cy);
            let (qx, qy) = (pf(q).0 - c.cx, pf(q).1 - c.cy);
            atan2(px * qy - py * qx, px * qx + py * qy)
        };
        Some(ang(self.start, self.mid) + ang(self.mid, self.end))
    }

    /// Arc length (nm); the polyline length for a degenerate (collinear) arc.
    #[must_use]
    pub fn length(&self) -> f64 {
        match (self.circle(), self.sweep()) {
            (Some(c), Some(s)) => c.r * s.abs(),
            _ => self.start.dist(self.mid) + self.mid.dist(self.end),
        }
    }

    /// Integer polyline from `start` to `end` whose chords stay within the returned bound
    /// (nm, including the ≤ 1 nm of rounding) of the true arc. Chords are bisected until
    /// their sagitta is at most `max_sag`.
    #[must_use]
    pub fn flatten(&self, max_sag: f64) -> (Vec<Point>, f64) {
        let Some(c) = self.circle() else {
            return (vec![self.start, self.mid, self.end], 0.0);
        };
        let mut pts = vec![self.start];
        let mut worst = 0.0f64;
        for (p, q) in [(self.start, self.mid), (self.mid, self.end)] {
            bisect(&c, pf(p), pf(q), max_sag.max(1.0), 0, &mut pts, &mut worst);
            pts.push(q);
        }
        pts.dedup();
        (pts, worst + 1.0)
    }

    /// Convex capsules covering the copper of this arc drawn `width` wide: a superset of
    /// the true annular sector (chord deviation and rounding are added to the radius).
    #[must_use]
    pub fn cover(&self, width: Coord, max_sag: f64) -> Vec<Hull> {
        let (pts, bound) = self.flatten(max_sag);
        #[allow(clippy::cast_possible_truncation)]
        let grow = bound.ceil() as Coord;
        pts.windows(2).map(|w| Hull::capsule(w[0], w[1], width / 2 + grow)).collect()
    }
}

/// Appends the interior points of the minor arc `p → q` (exclusive of both ends).
fn bisect(
    c: &Circle,
    p: (f64, f64),
    q: (f64, f64),
    max_sag: f64,
    depth: u32,
    out: &mut Vec<Point>,
    worst: &mut f64,
) {
    let (mx, my) = ((p.0 + q.0) / 2.0 - c.cx, (p.1 + q.1) / 2.0 - c.cy);
    let d = hypot(mx, my);
    let sag = c.r - d;
    if sag <= max_sag || depth >= 24 || d == 0.0 {
        *worst = worst.max(sag);
        return;
    }
    let m = (c.cx + mx * c.r / d, c.cy + my * c.r / d);
    bisect(c, p, m, max_sag, depth + 1, out, worst);
    #[allow(clippy::cast_possible_truncation)]
    out.push(Point::new(m.0.round() as Coord, m.1.round() as Coord));
    bisect(c, m, q, max_sag, depth + 1, out, worst);
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};

    /// Quarter circle of radius 1 mm centred at the origin: (1,0) → (√½,√½) → (0,1) mm.
    fn quarter() -> Arc {
        Arc::new(Point::new(1_000_000, 0), Point::new(707_107, 707_107), Point::new(0, 1_000_000))
    }

    #[test]
    fn circle_sweep_and_length() {
        let a = quarter();
        let c = a.circle().expect("circle");
        assert!(c.cx.abs() < 1.0 && c.cy.abs() < 1.0 && (c.r - 1e6).abs() < 1.0, "{c:?}");
        let s = a.sweep().expect("sweep");
        // The integer mid is ≤ 0.5 nm off the circle, which moves the centre by ~1 nm.
        assert!((s - FRAC_PI_2).abs() < 1e-5, "{s}");
        assert!((a.length() - 1e6 * FRAC_PI_2).abs() < 10.0, "{}", a.length());
        let rev = Arc::new(a.end, a.mid, a.start);
        assert!((rev.sweep().expect("sweep") + FRAC_PI_2).abs() < 1e-5, "reversed is negative");
        // A 270° arc (mid on the far side) is measured correctly.
        let big = Arc::new(Point::new(1_000_000, 0), Point::new(-707_107, -707_107), a.end);
        assert!((big.length() - 1e6 * 1.5 * PI).abs() < 20.0, "{}", big.length());
    }

    #[test]
    fn degenerate_arcs_fall_back_to_the_polyline() {
        let line = Arc::new(Point::new(0, 0), Point::new(5, 0), Point::new(10, 0));
        assert_eq!(line.circle(), None);
        assert_eq!(line.length(), 10.0);
        assert_eq!(line.flatten(1.0), (vec![line.start, line.mid, line.end], 0.0));
    }

    #[test]
    fn flatten_stays_within_the_bound_and_keeps_endpoints() {
        let a = quarter();
        let (pts, bound) = a.flatten(500.0);
        assert_eq!((pts[0], *pts.last().expect("end")), (a.start, a.end));
        assert!(pts.contains(&a.mid));
        assert!(bound <= 501.0 && pts.len() >= 5, "{bound} {}", pts.len());
        let c = a.circle().expect("circle");
        for p in &pts {
            let r = hypot(p.x as f64 - c.cx, p.y as f64 - c.cy);
            assert!((r - c.r).abs() <= 1.0, "vertex on the circle: {r}");
        }
    }

    #[test]
    fn cover_is_a_superset_of_the_copper() {
        let a = quarter();
        let w = 250_000;
        let hulls = a.cover(w, 2_000.0);
        let c = a.circle().expect("circle");
        // Sample the true copper (annular sector) densely: every sample is covered.
        for i in 0..=400 {
            let t = FRAC_PI_2 * f64::from(i) / 400.0;
            for k in [-1.0, -0.5, 0.0, 0.5, 1.0] {
                let r = c.r + k * f64::from(w as i32) / 2.0;
                #[allow(clippy::cast_possible_truncation)]
                let p = Point::new((r * t.cos()).round() as Coord, (r * t.sin()).round() as Coord);
                assert!(
                    hulls.iter().any(|h| h.clearance_violated(&Hull::disk(p, 0), 1)),
                    "uncovered copper at {p:?}"
                );
            }
        }
    }
}
