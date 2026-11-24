// SPDX-License-Identifier: AGPL-3.0-only
//! Exact integer geometry of coupled differential pairs.
//!
//! A pair is described by one octilinear *centre-line*; the two traces are its offsets at
//! `±hs` (half the centre-to-centre spacing `gap + width`). All coordinates stay integer:
//!
//! * axis-aligned runs are shifted by exactly `hs`;
//! * diagonal runs are shifted by `(∓a, ±a)` with `a = ⌈hs/√2⌉` (never closer than `hs`,
//!   at most 1 nm wider);
//! * at a bend the two offset lines are intersected exactly (the *mitre*): outer trace
//!   longer, inner trace shorter, gap constant along both legs.
//!
//! [`fan`] builds the symmetric break-out from two side-by-side pads to the coupled
//! start: a straight lead along the pad axis (INV-ROUTE-2), then a 45° jog that closes the
//! pad spacing to the coupled spacing.

use ferroute_geom::{Coord, Point};

/// Smallest `a` with `2a² ≥ hs²`: the diagonal shift whose perpendicular distance
/// `a·√2` is at least `hs`.
#[must_use]
pub fn diag_offset(hs: Coord) -> Coord {
    let target = i128::from(hs) * i128::from(hs);
    let mut a = i128::from(f64_sqrt(target / 2));
    while 2 * a * a < target {
        a += 1;
    }
    while a > 0 && 2 * (a - 1) * (a - 1) >= target {
        a -= 1;
    }
    Coord::try_from(a).unwrap_or(Coord::MAX)
}

/// Integer square root estimate (exactness is restored by the callers' correction loops).
#[allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]
fn f64_sqrt(v: i128) -> u64 {
    (v as f64).sqrt() as u64
}

fn unit(v: Point) -> Option<Point> {
    (v != Point::ORIGIN && v.is_octilinear()).then(|| Point::new(v.x.signum(), v.y.signum()))
}

/// Removes repeated points and collinear interior points (same direction).
#[must_use]
pub fn simplify(points: &[Point]) -> Vec<Point> {
    let mut out: Vec<Point> = Vec::with_capacity(points.len());
    for &p in points {
        if out.last() == Some(&p) {
            continue;
        }
        while out.len() >= 2 {
            let (a, b) = (out[out.len() - 2], out[out.len() - 1]);
            let (u, v) = (b - a, p - b);
            if u.cross(v) == 0 && u.dot(v) > 0 {
                out.pop();
            } else {
                break;
            }
        }
        out.push(p);
    }
    out
}

/// Replaces every 90° vertex of `points` by two 45° vertices `cut` before and after it
/// (legs shorter than `2·cut` are left alone).
#[must_use]
pub fn chamfer_right_angles(points: &[Point], cut: Coord) -> Vec<Point> {
    let pts = simplify(points);
    let mut out = Vec::with_capacity(pts.len() + 4);
    for (i, &v) in pts.iter().enumerate() {
        let (Some(&a), Some(&b)) = (i.checked_sub(1).and_then(|k| pts.get(k)), pts.get(i + 1))
        else {
            out.push(v);
            continue;
        };
        let (d1, d2) = (v - a, b - v);
        let (Some(u1), Some(u2)) = (unit(d1), unit(d2)) else {
            out.push(v);
            continue;
        };
        let leg = |d: Point, u: Point| {
            let k = u.x.abs().max(u.y.abs());
            d.x.abs().max(d.y.abs()) / k
        };
        if u1.dot(u2) == 0 && leg(d1, u1) >= 2 * cut && leg(d2, u2) >= 2 * cut {
            out.push(v - u1 * cut);
            out.push(v + u2 * cut);
        } else {
            out.push(v);
        }
    }
    out
}

/// Offsets the octilinear polyline `centre` by `hs` to its left (`left`) or right.
///
/// `None` when the offset is not exactly representable in integers, a segment would
/// invert (legs too short for the bend) or `centre` is not octilinear.
#[must_use]
pub fn offset_polyline(centre: &[Point], hs: Coord, left: bool) -> Option<Vec<Point>> {
    let pts = simplify(centre);
    if pts.len() < 2 || hs <= 0 {
        return None;
    }
    let diag = diag_offset(hs);
    let side = if left { 1 } else { -1 };
    let segs: Vec<(Point, Point)> = pts
        .windows(2)
        .map(|w| {
            let u = unit(w[1] - w[0])?;
            let mag = if u.x != 0 && u.y != 0 { diag } else { hs };
            Some((u, Point::new(-u.y, u.x) * (mag * side)))
        })
        .collect::<Option<_>>()?;
    let mut out = vec![pts[0] + segs[0].1];
    for (i, pair) in segs.windows(2).enumerate() {
        let ((u1, o1), (u2, o2)) = (pair[0], pair[1]);
        let (q1, q2) = (pts[i + 1] + o1, pts[i + 1] + o2);
        let kappa = u1.cross(u2);
        if kappa == 0 {
            if u1.dot(u2) < 0 {
                return None;
            }
            out.push(q1);
            continue;
        }
        let num = (q2 - q1).cross(u2);
        if num % kappa != 0 {
            return None;
        }
        let t = Coord::try_from(num / kappa).ok()?;
        out.push(q1 + u1 * t);
    }
    out.push(pts[pts.len() - 1] + segs[segs.len() - 1].1);
    let sane = out.windows(2).zip(&segs).all(|(w, &(u, _))| {
        let d = w[1] - w[0];
        d.cross(u) == 0 && d.dot(u) > 0
    });
    sane.then_some(out)
}

/// Break-out of a pair from two side-by-side pads to the coupled start.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Fan {
    /// Start of the coupled section on the centre-line (travelling away from the pads).
    pub merge: Point,
    /// Positive / negative trace from its pad centre to its coupled start point.
    pub p: Vec<Point>,
    pub n: Vec<Point>,
    /// `true` when the positive trace lies on the *left* of the outward direction.
    pub p_left: bool,
}

/// Board point at `alpha` along `d` and `lambda` along the left normal of `d`.
fn at(d: Point, alpha: Coord, lambda: Coord) -> Point {
    let e = Point::new(-d.y, d.x);
    d * alpha + e * lambda
}

/// Builds the symmetric fan of pads `pad_p`, `pad_n` leaving along the axis direction
/// `d` (unit, axis-aligned) with a straight lead of at least `lead`, closing to the
/// centre spacing `spacing` (= gap + width, even).
///
/// `None` if the pads are not side by side across `d` (equal position along `d`), if
/// `spacing` is odd or not positive, or if the pads coincide laterally.
#[must_use]
pub fn fan(pad_p: Point, pad_n: Point, d: Point, lead: Coord, spacing: Coord) -> Option<Fan> {
    let axis = d.x.abs() + d.y.abs() == 1;
    if !axis || spacing <= 0 || spacing % 2 != 0 || lead <= 0 {
        return None;
    }
    let e = Point::new(-d.y, d.x);
    let (alpha, lp, ln) = (pad_p.dot(d), pad_p.dot(e), pad_n.dot(e));
    if pad_n.dot(d) != alpha || lp == ln {
        return None;
    }
    let (alpha, lp, ln) =
        (Coord::try_from(alpha).ok()?, Coord::try_from(lp).ok()?, Coord::try_from(ln).ok()?);
    let sigma = if ln > lp { 1 } else { -1 };
    let hs = spacing / 2;
    let lc = (lp + ln).div_euclid(2);
    let (tp, tn) = (lc - sigma * hs, lc + sigma * hs);
    let (dp, dn) = ((tp - lp).abs(), (tn - ln).abs());
    let dm = dp.max(dn);
    let alpha_m = alpha + lead + dm;
    let leg = |pad: Point, from: Coord, to: Coord, shift: Coord| {
        let mut v = vec![pad, at(d, alpha + lead + dm - shift, from)];
        v.push(at(d, alpha_m, to));
        simplify(&v)
    };
    Some(Fan {
        merge: at(d, alpha_m, lc),
        p: leg(pad_p, lp, tp, dp),
        n: leg(pad_n, ln, tn, dn),
        p_left: sigma < 0,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: i64, y: i64) -> Point {
        Point::new(x, y)
    }

    /// Perpendicular distance between the parallel lines through `a` and `b`, direction `u`.
    fn perp(u: Point, a: Point, b: Point) -> f64 {
        u.cross(b - a).unsigned_abs() as f64 / (u.norm2() as f64).sqrt()
    }

    #[test]
    fn diag_offset_is_the_smallest_integer_covering_hs() {
        for hs in [1, 2, 3, 100, 225_000, 1_000_000, 123_457] {
            let a = diag_offset(hs);
            assert!(2 * i128::from(a) * i128::from(a) >= i128::from(hs) * i128::from(hs));
            assert!(
                a == 0
                    || 2 * i128::from(a - 1) * i128::from(a - 1) < i128::from(hs) * i128::from(hs)
            );
        }
        assert_eq!(diag_offset(225_000), 159_100);
    }

    #[test]
    fn straight_offset_is_exact_on_both_sides() {
        let c = [p(0, 0), p(10_000_000, 0)];
        let hs = 225_000;
        let (l, r) =
            (offset_polyline(&c, hs, true).unwrap(), offset_polyline(&c, hs, false).unwrap());
        // "Left" is the +normal `(-u.y, u.x)`: south of an eastward run (y grows downwards).
        assert_eq!(l, vec![p(0, hs), p(10_000_000, hs)]);
        assert_eq!(r, vec![p(0, -hs), p(10_000_000, -hs)]);
        assert_eq!(l[0].y - r[0].y, 2 * hs);
    }

    #[test]
    fn forty_five_degree_bend_keeps_the_gap_constant_with_exact_mitres() {
        // East, then north-east (a 45° bend), then east again (an S-bend).
        let c = [p(0, 0), p(2_000_000, 0), p(3_000_000, -1_000_000), p(5_000_000, -1_000_000)];
        let hs = 225_000;
        for left in [true, false] {
            let o = offset_polyline(&c, hs, left).expect("offset");
            assert_eq!(o.len(), 4);
            // All segments stay octilinear.
            assert!(o.windows(2).all(|w| (w[1] - w[0]).is_octilinear()));
            // Distance of each offset leg to its centre leg is hs (axis) or ≥ hs (diag).
            let d0 = perp(p(1, 0), c[0], o[0]);
            assert_eq!(d0 as i64, hs);
            let d1 = perp(p(1, -1), c[1], o[1]);
            assert!(d1 >= hs as f64 && d1 < hs as f64 + 2.0, "{d1}");
        }
        // Both traces together: spacing 2·hs on the straight runs, ≤ 3 nm more on the diagonal.
        let (l, r) =
            (offset_polyline(&c, hs, true).unwrap(), offset_polyline(&c, hs, false).unwrap());
        assert_eq!(perp(p(1, 0), l[0], r[0]) as i64, 2 * hs);
        let diag = perp(p(1, -1), l[1], r[1]);
        assert!(diag >= (2 * hs) as f64 && diag < (2 * hs) as f64 + 4.0, "{diag}");
        assert_eq!(perp(p(1, 0), l[3], r[3]) as i64, 2 * hs);
    }

    #[test]
    fn outer_trace_is_longer_than_the_inner_one_at_a_bend() {
        let c = [
            p(0, 0),
            p(4_000_000, 0),
            p(6_000_000, 2_000_000),
            p(6_000_000, 2_000_000 + 4_000_000),
        ];
        let hs = 200_000;
        let len = |v: &[Point]| v.windows(2).map(|w| w[0].dist(w[1])).sum::<f64>();
        let (a, b) =
            (offset_polyline(&c, hs, true).unwrap(), offset_polyline(&c, hs, false).unwrap());
        let (la, lb) = (len(&a), len(&b));
        // Two 45° bends turning the same way: the difference is 2·(2·hs·tan 22.5°)·2.
        let expect = 4.0 * 2.0 * hs as f64 * (std::f64::consts::PI / 8.0).tan();
        assert!(((la - lb).abs() - expect).abs() < 8.0, "{} vs {expect}", (la - lb).abs());
    }

    #[test]
    fn ninety_degree_corner_is_chamfered_into_two_exact_45_bends() {
        let c = [p(0, 0), p(5_000_000, 0), p(5_000_000, 5_000_000)];
        assert_eq!(
            chamfer_right_angles(&c, 1_000_000),
            vec![p(0, 0), p(4_000_000, 0), p(5_000_000, 1_000_000), p(5_000_000, 5_000_000)]
        );
        // Too-short legs are left alone; collinear input is simplified.
        let short = [p(0, 0), p(1_000_000, 0), p(1_000_000, 5_000_000)];
        assert_eq!(chamfer_right_angles(&short, 1_000_000), short.to_vec());
        let ch = chamfer_right_angles(&c, 1_000_000);
        for left in [true, false] {
            let o = offset_polyline(&ch, 225_000, left).expect("offset");
            assert_eq!(o.len(), 4);
            assert!(o.windows(2).all(|w| (w[1] - w[0]).is_octilinear()));
        }
        // A diagonal-diagonal 90° corner offsets exactly as well.
        let dd = [p(0, 0), p(1_000_001, 1_000_001), p(2_000_002, 0)];
        let o = offset_polyline(&dd, 225_000, true).expect("offset");
        assert!(o.windows(2).all(|w| (w[1] - w[0]).is_octilinear()));
    }

    #[test]
    fn inverted_offsets_are_rejected() {
        // A 0.14 mm diagonal between two same-way 45° bends cannot host a 225 µm offset on
        // its inner side (it would shrink by 2·hs·tan 22.5° = 0.19 mm): exactly one side fails.
        let c = [p(0, 0), p(1_000_000, 0), p(1_100_000, -100_000), p(1_100_000, -1_100_000)];
        let (l, r) = (offset_polyline(&c, 225_000, true), offset_polyline(&c, 225_000, false));
        assert!(l.is_some() != r.is_some(), "{l:?} {r:?}");
        assert!(
            offset_polyline(&c, 20_000, true).is_some()
                && offset_polyline(&c, 20_000, false).is_some()
        );
        assert!(offset_polyline(&[p(0, 0)], 1, true).is_none());
        assert!(offset_polyline(&[p(0, 0), p(3, 1)], 1, true).is_none(), "not octilinear");
    }

    #[test]
    fn fan_is_symmetric_and_closes_the_pad_spacing_exactly() {
        // Pads 2.54 mm apart on a vertical column, leaving east; coupled spacing 450 µm.
        let (pp, pn) = (p(0, 0), p(0, 2_540_000));
        let f = fan(pp, pn, p(1, 0), 700_000, 450_000).expect("fan");
        let (dp, dn) = (f.p.last().copied().unwrap(), f.n.last().copied().unwrap());
        assert_eq!(dp.x, dn.x, "jogs end at the same along-position");
        assert_eq!(dn.y - dp.y, 450_000);
        assert_eq!(f.merge, p(dp.x, 1_270_000));
        assert!(!f.p_left, "P is on the -normal (north) side when leaving east");
        // Lead ≥ 0.7 mm along the pad axis, then one 45° jog of (2.54-0.45)/2 mm.
        assert_eq!(f.p[0], pp);
        assert_eq!(f.p[1], p(700_000, 0));
        assert_eq!(f.p.len(), 3);
        let jog = f.p[2] - f.p[1];
        assert_eq!((jog.x, jog.y), (1_045_000, 1_045_000));
        // Mirror image for N.
        assert_eq!(f.n[1], p(700_000, 2_540_000));
        assert_eq!(f.n[2] - f.n[1], p(1_045_000, -1_045_000));
        // Rejects misaligned or odd inputs.
        assert!(fan(pp, p(10, 2_540_000), p(1, 0), 700_000, 450_000).is_none());
        assert!(fan(pp, pn, p(1, 0), 700_000, 451_001).is_none());
        assert!(fan(pp, pn, p(1, 1), 700_000, 450_000).is_none());
        // Odd pad midpoint: centre spacing still exact.
        let f = fan(pp, p(0, 2_540_001), p(1, 0), 700_000, 450_000).expect("fan");
        assert_eq!(f.n.last().unwrap().y - f.p.last().unwrap().y, 450_000);
        // Pads closer than the spacing diverge instead.
        let f = fan(pp, p(0, 300_000), p(1, 0), 700_000, 450_000).expect("fan");
        assert_eq!(f.n.last().unwrap().y - f.p.last().unwrap().y, 450_000);
    }

    #[test]
    fn fan_works_in_every_axis_direction() {
        let (pp, pn) = (p(10_000_000, 10_000_000), p(10_000_000 + 1_270_000, 10_000_000));
        for d in [p(0, 1), p(0, -1)] {
            let f = fan(pp, pn, d, 500_000, 450_000).expect("fan");
            let (a, b) = (*f.p.last().unwrap(), *f.n.last().unwrap());
            assert_eq!((b.x - a.x).abs(), 450_000);
            assert_eq!(a.y, b.y);
            assert_eq!(f.p[1].x, pp.x);
        }
        assert!(fan(pp, pn, p(1, 0), 500_000, 450_000).is_none(), "pads not side by side");
    }
}
