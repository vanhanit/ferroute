// SPDX-License-Identifier: AGPL-3.0-only
//! Integer polygon booleans for copper pours (`i_overlay`, exact `i64` engine).
//!
//! Fills are sets of [`Region`]s (an outer ring plus hole rings). KiCad stores holes as
//! *bridged* single outlines, so [`regions_of`] resolves them and [`bridge`] writes them
//! back. Obstacles are approximated by **circumscribing** convex polygons
//! ([`circumscribed`]): they always contain the true keep-out region, so a difference never
//! leaves copper inside it (the new fill is a subset of the legal one). All vertices are
//! integers; results are deterministic.

use crate::{Coord, Hull, Point, Polygon, Segment, convex_hull, orient, segments_intersect};
use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay::{Overlay, ShapeType};
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::i_float::int::angle::Angle;
use i_overlay::i_float::int::point::IntPoint;
use i_overlay::mesh::int::outline::offset::IntOutlineOffset;
use i_overlay::mesh::int::style::{IntLineJoin, IntOutlineStyle};

type Ip = IntPoint<i64>;
type Shape = Vec<Vec<Ip>>;

/// Extra nanometres added to every obstacle so that vertex rounding of the circumscribing
/// polygon and of boolean intersection points can never reach into the true keep-out.
pub const SAFETY_NM: Coord = 3;

/// A connected copper region: outer ring and holes (no bridges).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Region {
    pub outer: Vec<Point>,
    pub holes: Vec<Vec<Point>>,
}

impl Region {
    /// Twice the net area (outer minus holes), in nm².
    #[must_use]
    pub fn area2(&self) -> i128 {
        ring_area2(&self.outer).abs() - self.holes.iter().map(|h| ring_area2(h).abs()).sum::<i128>()
    }

    /// Point-in-region (boundary counts as inside the outer ring and outside the holes).
    #[must_use]
    pub fn contains(&self, p: Point) -> bool {
        let inside = |r: &Vec<Point>| Polygon::new(r.clone()).contains(p);
        inside(&self.outer) && !self.holes.iter().any(inside)
    }

    /// Bounding box of the outer ring.
    #[must_use]
    pub fn bbox(&self) -> Option<crate::Rect> {
        crate::Rect::from_points(&self.outer)
    }
}

fn ring_area2(pts: &[Point]) -> i128 {
    let n = pts.len();
    (0..n).map(|i| pts[i].cross(pts[(i + 1) % n])).sum()
}

fn ip(p: Point) -> Ip {
    IntPoint::new(p.x, p.y)
}

fn pt(p: &Ip) -> Point {
    Point::new(p.x, p.y)
}

/// Ring as engine points with the requested winding (`ccw` = positive shoelace area).
fn ring(pts: &[Point], ccw: bool) -> Vec<Ip> {
    let mut v: Vec<Ip> = pts.iter().map(|&p| ip(p)).collect();
    if (ring_area2(pts) > 0) != ccw {
        v.reverse();
    }
    v
}

fn shape_of(r: &Region) -> Shape {
    let mut s = vec![ring(&r.outer, true)];
    s.extend(r.holes.iter().map(|h| ring(h, false)));
    s
}

fn regions_from(shapes: Vec<Shape>) -> Vec<Region> {
    shapes
        .into_iter()
        .filter_map(|s| {
            let mut it = s.into_iter();
            let outer: Vec<Point> = it.next()?.iter().map(pt).collect();
            (outer.len() >= 3)
                .then(|| Region { outer, holes: it.map(|h| h.iter().map(pt).collect()).collect() })
        })
        .collect()
}

fn run(subject: &[Shape], clip: &[Shape], rule: OverlayRule) -> Vec<Region> {
    let cap = subject.iter().chain(clip).flatten().map(Vec::len).sum();
    let mut ov = Overlay::<i64>::new(cap);
    for s in subject {
        ov.add_source(s, ShapeType::Subject);
    }
    for s in clip {
        ov.add_source(s, ShapeType::Clip);
    }
    regions_from(ov.overlay(rule, FillRule::NonZero))
}

/// Resolves (possibly bridged, self-touching, overlapping) polygons into disjoint regions
/// with explicit holes (their union).
#[must_use]
pub fn regions_of(polys: &[Polygon]) -> Vec<Region> {
    let shapes: Vec<Shape> =
        polys.iter().filter(|p| p.pts.len() >= 3).map(|p| vec![ring(&p.pts, true)]).collect();
    run(&shapes, &[], OverlayRule::Subject)
}

/// `subject` minus the union of `clips` (any winding).
#[must_use]
pub fn difference(subject: &[Region], clips: &[Polygon]) -> Vec<Region> {
    if clips.is_empty() {
        return subject.to_vec();
    }
    let s: Vec<Shape> = subject.iter().map(shape_of).collect();
    let c: Vec<Shape> =
        clips.iter().filter(|p| p.pts.len() >= 3).map(|p| vec![ring(&p.pts, true)]).collect();
    run(&s, &c, OverlayRule::Difference)
}

/// Intersection of two region sets.
#[must_use]
pub fn intersection(a: &[Region], b: &[Region]) -> Vec<Region> {
    let (a, b): (Vec<Shape>, Vec<Shape>) =
        (a.iter().map(shape_of).collect(), b.iter().map(shape_of).collect());
    run(&a, &b, OverlayRule::Intersect)
}

/// Offsets every region by `d` nm (positive grows copper, negative shrinks it). Convex
/// corners are mitred (clipped), so a grow after a shrink restores sharp corners.
#[must_use]
pub fn offset(regions: &[Region], d: Coord) -> Vec<Region> {
    if regions.is_empty() || d == 0 {
        return regions.to_vec();
    }
    let shapes: Vec<Shape> = regions.iter().map(shape_of).collect();
    let style =
        IntOutlineStyle::<i64>::new(d).line_join(IntLineJoin::Miter(Angle::from_bits(357_913_941)));
    shapes.outline(&style).map(regions_from).unwrap_or_default()
}

/// [`offset`] with bevelled convex corners: growing never reaches beyond the true (round)
/// offset, so `offset_bevel(offset(r, -d), d)` is a conservative morphological opening
/// (a mitred grow restores acute tips, e.g. the neck between two nearby holes).
#[must_use]
pub fn offset_bevel(regions: &[Region], d: Coord) -> Vec<Region> {
    if regions.is_empty() || d == 0 {
        return regions.to_vec();
    }
    let shapes: Vec<Shape> = regions.iter().map(shape_of).collect();
    let style = IntOutlineStyle::<i64>::new(d).line_join(IntLineJoin::Bevel);
    shapes.outline(&style).map(regions_from).unwrap_or_default()
}

const SEC_PI_32: f64 = 1.004_838_572_376_311_4;

#[rustfmt::skip]
#[allow(clippy::approx_constant)] // the unit-circle table is literal on purpose (frozen, libm-free)
const UNIT: [(f64, f64); 32] = [
    (1.0, 0.0), (0.980_785_280_403_230_4, 0.195_090_322_016_128_25),
    (0.923_879_532_511_286_7, 0.382_683_432_365_089_8), (0.831_469_612_302_545_2, 0.555_570_233_019_602_2),
    (0.707_106_781_186_547_6, 0.707_106_781_186_547_5), (0.555_570_233_019_602_3, 0.831_469_612_302_545_2),
    (0.382_683_432_365_089_84, 0.923_879_532_511_286_7), (0.195_090_322_016_128_33, 0.980_785_280_403_230_4),
    (6.123_233_995_736_766e-17, 1.0), (-0.195_090_322_016_128_2, 0.980_785_280_403_230_4),
    (-0.382_683_432_365_089_7, 0.923_879_532_511_286_7), (-0.555_570_233_019_602, 0.831_469_612_302_545_5),
    (-0.707_106_781_186_547_5, 0.707_106_781_186_547_6), (-0.831_469_612_302_545_3, 0.555_570_233_019_602_2),
    (-0.923_879_532_511_286_7, 0.382_683_432_365_089_9), (-0.980_785_280_403_230_4, 0.195_090_322_016_128_6),
    (-1.0, 1.224_646_799_147_353_2e-16), (-0.980_785_280_403_230_4, -0.195_090_322_016_128_36),
    (-0.923_879_532_511_286_8, -0.382_683_432_365_089_67), (-0.831_469_612_302_545_5, -0.555_570_233_019_602),
    (-0.707_106_781_186_547_7, -0.707_106_781_186_547_5), (-0.555_570_233_019_602_2, -0.831_469_612_302_545_2),
    (-0.382_683_432_365_090_34, -0.923_879_532_511_286_5), (-0.195_090_322_016_128_66, -0.980_785_280_403_230_3),
    (-1.836_970_198_721_029_7e-16, -1.0), (0.195_090_322_016_128_3, -0.980_785_280_403_230_4),
    (0.382_683_432_365_09, -0.923_879_532_511_286_6), (0.555_570_233_019_601_8, -0.831_469_612_302_545_5),
    (0.707_106_781_186_547_4, -0.707_106_781_186_547_7), (0.831_469_612_302_545_2, -0.555_570_233_019_602_2),
    (0.923_879_532_511_286_5, -0.382_683_432_365_090_4), (0.980_785_280_403_230_3, -0.195_090_322_016_128_72),
];

/// Convex polygon (CCW in a y-up frame) that **contains** `hull` grown by `extra`:
/// the hull core ⊕ a disk of radius `hull.radius + extra`, outer-approximated by 32-gons
/// around every core vertex plus [`SAFETY_NM`]. Never smaller than the true shape.
#[must_use]
pub fn circumscribed(hull: &Hull, extra: Coord) -> Polygon {
    #[allow(clippy::cast_precision_loss)]
    let rc = (hull.radius + extra.max(0)) as f64 * SEC_PI_32 + (2 * SAFETY_NM) as f64;
    let mut cloud: Vec<Point> = Vec::with_capacity(hull.pts.len() * UNIT.len());
    for c in &hull.pts {
        for &(ux, uy) in &UNIT {
            #[allow(clippy::cast_possible_truncation)]
            let (dx, dy) = ((ux * rc).round() as Coord, (uy * rc).round() as Coord);
            cloud.push(Point::new(c.x + dx, c.y + dy));
        }
    }
    Polygon::new(convex_hull(&cloud))
}

/// `true` when segment `s` (bridge candidate) cleanly crosses no edge of `rings` apart from
/// touching at its own endpoints.
fn bridge_clear(s: Segment, rings: &[&[Point]]) -> bool {
    for r in rings {
        let n = r.len();
        for i in 0..n {
            let e = Segment::new(r[i], r[(i + 1) % n]);
            let incident = [e.a, e.b].iter().filter(|&&q| q == s.a || q == s.b).count();
            if incident == 0 {
                if segments_intersect(s, e) {
                    return false;
                }
            } else if orient(e.a, e.b, s.a) == 0 && orient(e.a, e.b, s.b) == 0 {
                // collinear with an incident edge: the bridge would overlap it
                if e.a != e.b {
                    return false;
                }
            } else {
                // an incident edge touches only at the shared endpoint, except when the
                // *other* bridge endpoint lies on it
                let other_on_edge = [s.a, s.b].iter().any(|&q| {
                    q != e.a
                        && q != e.b
                        && orient(e.a, e.b, q) == 0
                        && segments_intersect(e, Segment::new(q, q))
                });
                if other_on_edge {
                    return false;
                }
            }
        }
    }
    true
}

/// KiCad-style single outline for `region`: every hole is joined to the outer ring (or to an
/// already joined hole) by a zero-width bridge. `None` when no valid bridge exists (the
/// caller must then drop the region, never emit it without its hole).
#[must_use]
pub fn bridge(region: &Region) -> Option<Polygon> {
    let mut outer = region.outer.clone();
    if ring_area2(&outer) < 0 {
        outer.reverse();
    }
    let mut holes: Vec<Vec<Point>> = region
        .holes
        .iter()
        .map(|h| {
            let mut h = h.clone();
            if ring_area2(&h) > 0 {
                h.reverse();
            }
            h
        })
        .collect();
    // Leftmost hole first (deterministic order).
    holes.sort_by_key(|h| h.iter().min().copied());
    let mut merged = outer;
    for k in 0..holes.len() {
        let h = &holes[k];
        let (ih, &hp) = h.iter().enumerate().min_by_key(|&(_, p)| *p)?;
        let rest: Vec<&[Point]> = std::iter::once(merged.as_slice())
            .chain(holes[k..].iter().map(Vec::as_slice))
            .collect();
        let mut cand: Vec<(u128, usize)> =
            merged.iter().enumerate().map(|(i, &q)| ((q - hp).norm2(), i)).collect();
        cand.sort_unstable();
        let iv = cand
            .iter()
            .find(|&&(_, i)| {
                // `rest` already contains hole `k` itself, so bridge crossings of its edges are caught
                bridge_clear(Segment::new(hp, merged[i]), &rest)
            })
            .map(|&(_, i)| i)?;
        let mut next = Vec::with_capacity(merged.len() + h.len() + 2);
        next.extend_from_slice(&merged[..=iv]);
        next.extend_from_slice(&h[ih..]);
        next.extend_from_slice(&h[..=ih]);
        next.push(merged[iv]);
        next.extend_from_slice(&merged[iv + 1..]);
        merged = next;
    }
    Some(Polygon::new(merged))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rng::Rng;

    fn sq(x0: i64, y0: i64, x1: i64, y1: i64) -> Polygon {
        Polygon::new(vec![
            Point::new(x0, y0),
            Point::new(x1, y0),
            Point::new(x1, y1),
            Point::new(x0, y1),
        ])
    }

    fn area2(rs: &[Region]) -> i128 {
        rs.iter().map(Region::area2).sum()
    }

    #[test]
    fn square_minus_inner_square_is_a_ring() {
        let base = regions_of(&[sq(0, 0, 100, 100)]);
        let r = difference(&base, &[sq(40, 40, 60, 60)]);
        assert_eq!(r.len(), 1);
        assert_eq!(r[0].holes.len(), 1);
        assert_eq!(area2(&r), 2 * (100 * 100 - 20 * 20));
    }

    #[test]
    fn square_minus_capsule_cuts_the_square_in_two() {
        let base = regions_of(&[sq(0, 0, 1_000_000, 1_000_000)]);
        let cap = circumscribed(
            &Hull::capsule(Point::new(-10, 500_000), Point::new(1_000_010, 500_000), 50_000),
            100_000,
        );
        let r = difference(&base, &[cap]);
        assert_eq!(r.len(), 2, "{r:?}");
        // gap >= 2 * (50_000 + 100_000) between the halves
        for reg in &r {
            let bb = reg.bbox().unwrap();
            assert!(bb.max.y < 350_000 || bb.min.y > 650_000, "{bb:?}");
        }
    }

    #[test]
    fn bridged_input_is_resolved_and_bridge_round_trips() {
        let base = regions_of(&[sq(0, 0, 100, 100)]);
        let ring = difference(&base, &[sq(40, 40, 60, 60)]);
        let bridged = bridge(&ring[0]).expect("bridge");
        assert!(bridged.pts.len() >= 10);
        let back = regions_of(&[bridged]);
        assert_eq!(back.len(), 1);
        assert_eq!(back[0].holes.len(), 1);
        assert_eq!(area2(&back), area2(&ring));
    }

    #[test]
    fn multiple_holes_bridge_and_round_trip() {
        let base = regions_of(&[sq(0, 0, 1000, 1000)]);
        let holes: Vec<Polygon> = (0..4)
            .flat_map(|i| {
                (0..3).map(move |j| sq(100 + i * 200, 100 + j * 300, 200 + i * 200, 200 + j * 300))
            })
            .collect();
        let r = difference(&base, &holes);
        assert_eq!(r.len(), 1);
        assert_eq!(r[0].holes.len(), 12);
        let b = bridge(&r[0]).expect("bridge");
        let back = regions_of(&[b]);
        assert_eq!(back.len(), 1);
        assert_eq!(back[0].holes.len(), 12);
        assert_eq!(area2(&back), area2(&r));
    }

    #[test]
    fn offset_opening_removes_thin_necks_and_keeps_bulk() {
        let wide = sq(0, 0, 1000, 1000);
        let thin = sq(1000, 495, 1500, 505);
        let blob = sq(1500, 0, 2500, 1000);
        let base = regions_of(&[wide, thin, blob]);
        assert_eq!(base.len(), 1);
        let opened = intersection(&offset(&offset(&base, -20), 20), &base);
        assert_eq!(opened.len(), 2, "{opened:?}");
        assert_eq!(area2(&opened), 2 * 2 * 1_000_000);
    }

    #[test]
    fn circumscribed_contains_the_exact_shape() {
        let h = Hull::capsule(Point::new(0, 0), Point::new(1_000_000, 300_000), 100_000);
        let c = circumscribed(&h, 200_000);
        let mut rng = Rng::new(3);
        for _ in 0..2000 {
            let p =
                Point::new(rng.range_i64(-600_000, 1_600_000), rng.range_i64(-600_000, 900_000));
            let near = h.clearance_violated(&Hull::disk(p, 0), 200_000);
            if near {
                assert!(c.contains(p), "{p:?} inside the exact keep-out but outside the polygon");
            }
        }
    }

    #[test]
    fn difference_result_never_enters_the_keepout_and_is_deterministic() {
        let mut rng = Rng::new(11);
        let hulls: Vec<Hull> = (0..12)
            .map(|_| {
                let a = Point::new(rng.range_i64(0, 4_000_000), rng.range_i64(0, 4_000_000));
                let b = Point::new(rng.range_i64(0, 4_000_000), rng.range_i64(0, 4_000_000));
                Hull::capsule(a, b, rng.range_i64(50_000, 200_000))
            })
            .collect();
        let clr = 250_000;
        let obstacles: Vec<Polygon> = hulls.iter().map(|h| circumscribed(h, clr)).collect();
        let base = regions_of(&[sq(0, 0, 4_000_000, 4_000_000)]);
        let a = difference(&base, &obstacles);
        assert_eq!(a, difference(&base, &obstacles));
        let mut inside = 0;
        for _ in 0..20_000 {
            let p = Point::new(rng.range_i64(1, 3_999_999), rng.range_i64(1, 3_999_999));
            if a.iter().any(|r| r.contains(p)) {
                inside += 1;
                for h in &hulls {
                    assert!(
                        !h.clearance_violated(&Hull::disk(p, 0), clr),
                        "{p:?} too close to {h:?}"
                    );
                }
            }
        }
        assert!(inside > 100, "test must exercise the result");
    }
}
