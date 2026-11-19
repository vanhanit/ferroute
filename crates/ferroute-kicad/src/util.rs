// SPDX-License-Identifier: AGPL-3.0-only
//! Small shared helpers: number parsing/formatting, deterministic UUIDs, arc flattening.

use crate::KicadError;
use crate::sexpr::SExpr;
use ferroute_geom::units::{format_mm, parse_mm};
use ferroute_geom::{Coord, Point};

/// Exact millimetre-decimal to nanometre conversion with a crate error.
pub(crate) fn mm(s: &str) -> Result<Coord, KicadError> {
    parse_mm(s).map_err(|e| KicadError::Format(e.to_string()))
}

/// Parses a floating point number (angles, ratios).
pub(crate) fn float(s: &str) -> Result<f64, KicadError> {
    s.trim()
        .parse::<f64>()
        .ok()
        .filter(|v| v.is_finite())
        .ok_or_else(|| KicadError::Format(format!("invalid number {s:?}")))
}

/// Normalises degrees into `(-180, 180]` like KiCad's `Normalize180`.
pub(crate) fn norm180(deg: f64) -> f64 {
    let mut d = deg % 360.0;
    if d > 180.0 {
        d -= 360.0;
    } else if d <= -180.0 {
        d += 360.0;
    }
    d
}

/// Formats an angle in degrees: integers without a fractional part, otherwise the
/// shortest decimal that round-trips the `f64`.
pub(crate) fn format_angle(deg: f64) -> String {
    let r = deg.round();
    if (deg - r).abs() < 1e-9 {
        // Normalise "-0".
        format!("{}", if r == 0.0 { 0.0 } else { r })
    } else {
        format!("{deg}")
    }
}

/// `true` when two angles are equal modulo 360 degrees (within 1e-9).
pub(crate) fn same_angle(a: f64, b: f64) -> bool {
    norm180(a - b).abs() < 1e-9
}

/// `(x y)` style node: `(name x y)`.
pub(crate) fn xy_node(name: &str, p: Point) -> SExpr {
    SExpr::list(vec![SExpr::sym(name), SExpr::sym(format_mm(p.x)), SExpr::sym(format_mm(p.y))])
}

/// `(name value)` node with a bare value.
pub(crate) fn kv(name: &str, value: impl Into<String>) -> SExpr {
    SExpr::list(vec![SExpr::sym(name), SExpr::sym(value)])
}

/// `(name "value")` node with a quoted value.
pub(crate) fn kq(name: &str, value: impl Into<String>) -> SExpr {
    SExpr::list(vec![SExpr::sym(name), SExpr::quoted(value)])
}

/// Node with a head and arbitrary children.
pub(crate) fn node(head: &str, rest: Vec<SExpr>) -> SExpr {
    let mut v = Vec::with_capacity(rest.len() + 1);
    v.push(SExpr::sym(head));
    v.extend(rest);
    SExpr::List(v)
}

/// SplitMix64 step.
pub(crate) fn splitmix64(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// Deterministic RFC-4122 version-4 UUID string from `(seed, kind, index)`.
pub(crate) fn uuid_for(seed: u64, kind: u64, index: u64) -> String {
    let mut st =
        seed ^ kind.wrapping_mul(0xD6E8_FEB8_6659_FD93) ^ index.wrapping_mul(0xA076_1D64_78BD_642F);
    // One warm-up round so nearby (kind, index) pairs diverge.
    let _ = splitmix64(&mut st);
    let hi = splitmix64(&mut st);
    let lo = splitmix64(&mut st);
    let mut b = [0u8; 16];
    b[..8].copy_from_slice(&hi.to_be_bytes());
    b[8..].copy_from_slice(&lo.to_be_bytes());
    b[6] = (b[6] & 0x0f) | 0x40;
    b[8] = (b[8] & 0x3f) | 0x80;
    let h: String = b.iter().map(|x| format!("{x:02x}")).collect();
    format!("{}-{}-{}-{}-{}", &h[0..8], &h[8..12], &h[12..16], &h[16..20], &h[20..32])
}

/// Geometry of a circular arc recovered from three points.
#[derive(Copy, Clone, Debug)]
pub(crate) struct ArcGeom {
    pub cx: f64,
    pub cy: f64,
    pub r: f64,
    /// Start angle (radians, `atan2` convention on raw coordinates).
    pub a0: f64,
    /// Signed sweep in radians (|sweep| in `(0, 2*pi)`).
    pub sweep: f64,
}

#[allow(clippy::cast_precision_loss)]
fn pf(p: Point) -> (f64, f64) {
    (p.x as f64, p.y as f64)
}

/// Circle through `start`, `mid`, `end`; `None` when (nearly) collinear or degenerate.
pub(crate) fn arc_from_three(start: Point, mid: Point, end: Point) -> Option<ArcGeom> {
    let (ax, ay) = pf(start);
    // Work relative to the start for numerical stability.
    let (bx, by) = (pf(mid).0 - ax, pf(mid).1 - ay);
    let (cx, cy) = (pf(end).0 - ax, pf(end).1 - ay);
    let d = 2.0 * (bx * cy - by * cx);
    if d.abs() < 1e-9 {
        return None;
    }
    let b2 = bx * bx + by * by;
    let c2 = cx * cx + cy * cy;
    let ux = (cy * b2 - by * c2) / d;
    let uy = (bx * c2 - cx * b2) / d;
    let r = ferroute_geom::detmath::hypot(ux, uy);
    let a0 = (-uy).atan2(-ux);
    let a_end = (cy - uy).atan2(cx - ux);
    // Orientation of start -> mid -> end.
    let cross = bx * (cy - by) - by * (cx - bx);
    let tau = std::f64::consts::TAU;
    let mut sweep = a_end - a0;
    if cross > 0.0 {
        while sweep <= 0.0 {
            sweep += tau;
        }
        while sweep > tau {
            sweep -= tau;
        }
    } else {
        while sweep >= 0.0 {
            sweep -= tau;
        }
        while sweep < -tau {
            sweep += tau;
        }
    }
    Some(ArcGeom { cx: ax + ux, cy: ay + uy, r, a0, sweep })
}

impl ArcGeom {
    /// Number of chords so that the sagitta stays at most `max_sag` nm.
    pub(crate) fn segments(&self, max_sag: f64) -> usize {
        let ratio = (1.0 - max_sag / self.r).clamp(-1.0, 1.0);
        let step = 2.0 * ratio.acos();
        if step <= 1e-6 {
            return 4096;
        }
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let n = (self.sweep.abs() / step).ceil() as usize;
        n.clamp(1, 4096)
    }

    /// Actual sagitta (nm) when split into `n` chords.
    pub(crate) fn sagitta(&self, n: usize) -> f64 {
        #[allow(clippy::cast_precision_loss)]
        let half = self.sweep.abs() / (2.0 * n as f64);
        self.r * (1.0 - half.cos())
    }

    /// Point on the circle at parameter `k / n` of the sweep, with the radius increased by
    /// `outset` nm.
    pub(crate) fn point(&self, k: usize, n: usize, outset: f64) -> Point {
        #[allow(clippy::cast_precision_loss)]
        let a = self.a0 + self.sweep * (k as f64) / (n as f64);
        let r = self.r + outset;
        #[allow(clippy::cast_possible_truncation)]
        Point::new(
            (self.cx + r * a.cos()).round() as Coord,
            (self.cy + r * a.sin()).round() as Coord,
        )
    }
}

/// Vertices of a polygon *circumscribing* the circle (center, radius `r` nm): every point
/// of the disc is inside the polygon.
pub(crate) fn circumscribed_polygon(cx: f64, cy: f64, r: f64, n: usize) -> Vec<Point> {
    #[allow(clippy::cast_precision_loss)]
    let nf = n as f64;
    let rr = r / (std::f64::consts::PI / nf).cos() + 1.0;
    (0..n)
        .map(|k| {
            #[allow(clippy::cast_precision_loss)]
            let a = std::f64::consts::TAU * (k as f64) / nf;
            #[allow(clippy::cast_possible_truncation)]
            Point::new((cx + rr * a.cos()).round() as Coord, (cy + rr * a.sin()).round() as Coord)
        })
        .collect()
}

/// Euclidean distance between two points as `f64` nm.
pub(crate) fn dist(a: Point, b: Point) -> f64 {
    a.dist(b)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uuid_is_deterministic_v4() {
        let a = uuid_for(7, 1, 0);
        assert_eq!(a, uuid_for(7, 1, 0));
        assert_ne!(a, uuid_for(7, 1, 1));
        assert_ne!(a, uuid_for(8, 1, 0));
        let p: Vec<&str> = a.split('-').collect();
        assert_eq!(p.iter().map(|s| s.len()).collect::<Vec<_>>(), vec![8, 4, 4, 4, 12]);
        assert!(p[2].starts_with('4'));
        assert!(matches!(p[3].as_bytes()[0], b'8' | b'9' | b'a' | b'b'));
    }

    #[test]
    fn angles() {
        assert_eq!(format_angle(90.0), "90");
        assert_eq!(format_angle(-0.0), "0");
        assert_eq!(format_angle(45.5), "45.5");
        assert!((norm180(270.0) + 90.0).abs() < 1e-12);
        assert!((norm180(180.0) - 180.0).abs() < 1e-12);
        assert!((norm180(-180.0) - 180.0).abs() < 1e-12);
        assert!(same_angle(270.0, -90.0));
    }

    #[test]
    fn arc_quarter() {
        // Proper quarter circle about the origin.
        let s = Point::new(3_000_000, 0);
        let m = Point::new(2_121_320, 2_121_320);
        let e = Point::new(0, 3_000_000);
        let g = arc_from_three(s, m, e).unwrap();
        assert!(g.cx.abs() < 2.0 && g.cy.abs() < 2.0);
        assert!((g.r - 3_000_000.0).abs() < 2.0);
        assert!((g.sweep - std::f64::consts::FRAC_PI_2).abs() < 1e-6);
        let n = g.segments(1000.0);
        assert!(g.sagitta(n) <= 1000.0);
        assert_eq!(g.point(n, n, 0.0), e);
        assert_eq!(g.point(0, n, 0.0), s);
    }
}
