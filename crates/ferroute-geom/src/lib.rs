// SPDX-License-Identifier: AGPL-3.0-only
//! # ferroute-geom
//!
//! Exact integer geometry kernel for ferroute.
//!
//! All coordinates are **KiCad internal units: signed nanometres** stored in `i64`.
//! Every predicate that decides design-rule legality (intersection, containment,
//! "is the distance between these two shapes smaller than the clearance?") is evaluated
//! with exact integer arithmetic in `i128`/`u128`, never with floating point.
//!
//! ## Invariant INV-GEOM-1 (overflow freedom)
//! All coordinates handed to predicates satisfy `|c| <= MAX_COORD = 2^30` nm (≈ 1.07 m)
//! and all radii / clearances satisfy `0 <= r <= MAX_COORD`. Under this bound
//! coordinate differences fit in 31 bits, cross products in 63 bits and every squared
//! quantity used below in at most 126 bits, so `i128`/`u128` arithmetic is exact.
//! [`check_coord`] validates inputs at the board-loading boundary.
//!
//! Floating point is used only for *reporting* (e.g. [`Hull::distance`]) and for
//! cost heuristics, never for legality decisions.

pub mod arc;
pub mod detmath;
pub mod rng;
pub mod units;

use serde::{Deserialize, Serialize};
use smallvec::SmallVec;
use std::cmp::Ordering;
use std::ops::{Add, Mul, Neg, Sub};

/// A coordinate in nanometres.
pub type Coord = i64;

/// Largest absolute coordinate (and radius) accepted by the exact predicates (INV-GEOM-1).
pub const MAX_COORD: Coord = 1 << 30;

/// Returns `true` when `c` satisfies INV-GEOM-1.
#[must_use]
pub const fn check_coord(c: Coord) -> bool {
    c >= -MAX_COORD && c <= MAX_COORD
}

/// A 2-D point / vector in nanometres. KiCad convention: +x right, **+y down**.
#[derive(
    Copy, Clone, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize,
)]
pub struct Point {
    pub x: Coord,
    pub y: Coord,
}

impl Point {
    pub const ORIGIN: Self = Self { x: 0, y: 0 };

    #[must_use]
    pub const fn new(x: Coord, y: Coord) -> Self {
        Self { x, y }
    }

    /// Exact dot product.
    #[must_use]
    pub fn dot(self, o: Self) -> i128 {
        i128::from(self.x) * i128::from(o.x) + i128::from(self.y) * i128::from(o.y)
    }

    /// Exact 2-D cross product (z component).
    #[must_use]
    pub fn cross(self, o: Self) -> i128 {
        i128::from(self.x) * i128::from(o.y) - i128::from(self.y) * i128::from(o.x)
    }

    /// Exact squared Euclidean norm.
    #[must_use]
    pub fn norm2(self) -> u128 {
        self.dot(self).unsigned_abs()
    }

    /// Euclidean distance (reporting only).
    #[must_use]
    pub fn dist(self, o: Self) -> f64 {
        #[allow(clippy::cast_precision_loss)]
        let (dx, dy) = ((self.x - o.x) as f64, (self.y - o.y) as f64);
        detmath::hypot(dx, dy)
    }

    /// `true` when the vector is 0°, 45° or 90° (or zero).
    #[must_use]
    pub const fn is_octilinear(self) -> bool {
        self.x == 0 || self.y == 0 || self.x.abs() == self.y.abs()
    }

    /// Chebyshev (L∞) distance.
    #[must_use]
    pub fn chebyshev(self, o: Self) -> Coord {
        (self.x - o.x).abs().max((self.y - o.y).abs())
    }

    /// `true` when both coordinates satisfy INV-GEOM-1.
    #[must_use]
    pub const fn in_bounds(self) -> bool {
        check_coord(self.x) && check_coord(self.y)
    }

    /// Rotates this point about the origin by `angle`, using KiCad's convention
    /// (`RotatePoint`, y-down screen coordinates: positive angles turn counter-clockwise
    /// *on screen*). Multiples of 90° are exact; other angles are evaluated in `f64` and
    /// rounded half-away-from-zero exactly like KiCad's `KiROUND`.
    #[must_use]
    pub fn rotated(self, angle: Angle) -> Self {
        match angle.quadrant() {
            Some(0) => self,
            Some(1) => Self::new(self.y, -self.x),
            Some(2) => Self::new(-self.x, -self.y),
            Some(3) => Self::new(-self.y, self.x),
            _ => {
                let r = angle.degrees().to_radians();
                let (s, c) = r.sin_cos();
                #[allow(clippy::cast_precision_loss)]
                let (x, y) = (self.x as f64, self.y as f64);
                #[allow(clippy::cast_possible_truncation)]
                Self::new((x * c + y * s).round() as Coord, (-x * s + y * c).round() as Coord)
            }
        }
    }
}

impl Add for Point {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y)
    }
}

impl Sub for Point {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y)
    }
}

/// Scales a vector by an integer factor (exact).
impl Mul<Coord> for Point {
    type Output = Self;
    fn mul(self, k: Coord) -> Self {
        Self::new(self.x * k, self.y * k)
    }
}

impl Neg for Point {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y)
    }
}

/// An angle in degrees, normalised to `[0, 360)`. KiCad stores angles as degrees.
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct Angle(f64);

impl Angle {
    pub const ZERO: Self = Self(0.0);

    #[must_use]
    pub fn from_degrees(deg: f64) -> Self {
        let mut d = deg % 360.0;
        if d < 0.0 {
            d += 360.0;
        }
        // Collapse -0.0 and values that round to 360.
        if d == 0.0 || d >= 360.0 {
            d = 0.0;
        }
        Self(d)
    }

    #[must_use]
    pub const fn degrees(self) -> f64 {
        self.0
    }

    /// `Some(k)` when the angle is exactly `k * 90°`.
    #[must_use]
    pub fn quadrant(self) -> Option<u8> {
        let k = self.0 / 90.0;
        if k.fract() == 0.0 {
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            Some((k as u8) % 4)
        } else {
            None
        }
    }

    /// Sum, normalised to `[0, 360)`.
    #[must_use]
    #[allow(clippy::should_implement_trait)]
    pub fn add(self, o: Self) -> Self {
        Self::from_degrees(self.0 + o.0)
    }

    /// Difference, normalised to `[0, 360)`.
    #[must_use]
    #[allow(clippy::should_implement_trait)]
    pub fn sub(self, o: Self) -> Self {
        Self::from_degrees(self.0 - o.0)
    }
}

impl Add for Angle {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::from_degrees(self.0 + o.0)
    }
}

impl Sub for Angle {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::from_degrees(self.0 - o.0)
    }
}

/// Axis-aligned rectangle with inclusive bounds (`min <= max` component-wise).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Rect {
    pub min: Point,
    pub max: Point,
}

impl Rect {
    #[must_use]
    pub fn new(a: Point, b: Point) -> Self {
        Self {
            min: Point::new(a.x.min(b.x), a.y.min(b.y)),
            max: Point::new(a.x.max(b.x), a.y.max(b.y)),
        }
    }

    /// Bounding box of a non-empty point set.
    #[must_use]
    pub fn from_points(pts: &[Point]) -> Option<Self> {
        let first = *pts.first()?;
        let mut r = Self { min: first, max: first };
        for &p in &pts[1..] {
            r = r.include(p);
        }
        Some(r)
    }

    #[must_use]
    pub fn include(self, p: Point) -> Self {
        Self {
            min: Point::new(self.min.x.min(p.x), self.min.y.min(p.y)),
            max: Point::new(self.max.x.max(p.x), self.max.y.max(p.y)),
        }
    }

    #[must_use]
    pub fn union(self, o: Self) -> Self {
        self.include(o.min).include(o.max)
    }

    /// Grows (or shrinks, for negative `d`) the rectangle on every side.
    #[must_use]
    pub fn inflate(self, d: Coord) -> Self {
        Self {
            min: Point::new(self.min.x - d, self.min.y - d),
            max: Point::new(self.max.x + d, self.max.y + d),
        }
    }

    #[must_use]
    pub fn translate(self, v: Point) -> Self {
        Self { min: self.min + v, max: self.max + v }
    }

    #[must_use]
    pub const fn contains(self, p: Point) -> bool {
        p.x >= self.min.x && p.x <= self.max.x && p.y >= self.min.y && p.y <= self.max.y
    }

    /// Closed-set intersection test (touching rectangles intersect).
    #[must_use]
    pub const fn intersects(self, o: Self) -> bool {
        self.min.x <= o.max.x
            && o.min.x <= self.max.x
            && self.min.y <= o.max.y
            && o.min.y <= self.max.y
    }

    #[must_use]
    pub const fn width(self) -> Coord {
        self.max.x - self.min.x
    }

    #[must_use]
    pub const fn height(self) -> Coord {
        self.max.y - self.min.y
    }

    #[must_use]
    pub const fn center(self) -> Point {
        Point::new(self.min.x + self.width() / 2, self.min.y + self.height() / 2)
    }

    /// Corner points in counter-clockwise order (y-down: visually clockwise).
    #[must_use]
    pub const fn corners(self) -> [Point; 4] {
        [self.min, Point::new(self.max.x, self.min.y), self.max, Point::new(self.min.x, self.max.y)]
    }
}

/// A closed line segment.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Segment {
    pub a: Point,
    pub b: Point,
}

impl Segment {
    #[must_use]
    pub const fn new(a: Point, b: Point) -> Self {
        Self { a, b }
    }

    #[must_use]
    pub fn bbox(self) -> Rect {
        Rect::new(self.a, self.b)
    }

    #[must_use]
    pub fn length(self) -> f64 {
        self.a.dist(self.b)
    }
}

/// Sign of the orientation of the triple `(a, b, c)`: `1` counter-clockwise in a
/// y-up frame, `-1` clockwise, `0` collinear. Exact.
#[must_use]
pub fn orient(a: Point, b: Point, c: Point) -> i8 {
    match (b - a).cross(c - a).cmp(&0) {
        Ordering::Greater => 1,
        Ordering::Less => -1,
        Ordering::Equal => 0,
    }
}

/// `true` when collinear point `p` lies within the bounding box of segment `s`.
fn on_segment_bbox(s: Segment, p: Point) -> bool {
    p.x >= s.a.x.min(s.b.x)
        && p.x <= s.a.x.max(s.b.x)
        && p.y >= s.a.y.min(s.b.y)
        && p.y <= s.a.y.max(s.b.y)
}

/// Exact closed-segment intersection test (touching / collinear overlap counts).
#[must_use]
pub fn segments_intersect(s: Segment, t: Segment) -> bool {
    let d1 = orient(t.a, t.b, s.a);
    let d2 = orient(t.a, t.b, s.b);
    let d3 = orient(s.a, s.b, t.a);
    let d4 = orient(s.a, s.b, t.b);
    if d1 * d2 < 0 && d3 * d4 < 0 {
        return true;
    }
    (d1 == 0 && on_segment_bbox(t, s.a))
        || (d2 == 0 && on_segment_bbox(t, s.b))
        || (d3 == 0 && on_segment_bbox(s, t.a))
        || (d4 == 0 && on_segment_bbox(s, t.b))
}

/// Squared distance from `p` to segment `s` as an exact rational `num / den` (`den > 0`).
fn point_segment_dist2_rational(p: Point, s: Segment) -> (u128, u128) {
    let d = s.b - s.a;
    let v = p - s.a;
    let dd = d.norm2();
    if dd == 0 {
        return (v.norm2(), 1);
    }
    let t = v.dot(d);
    if t <= 0 {
        return (v.norm2(), 1);
    }
    if t.unsigned_abs() >= dd {
        return ((p - s.b).norm2(), 1);
    }
    let c = v.cross(d).unsigned_abs();
    (c * c, dd)
}

/// Exact test: is the Euclidean distance from `p` to segment `s` strictly less than `r`?
#[must_use]
pub fn point_segment_dist_lt(p: Point, s: Segment, r: Coord) -> bool {
    if r <= 0 {
        return false;
    }
    let (num, den) = point_segment_dist2_rational(p, s);
    let r2 = u128::from(r.unsigned_abs()) * u128::from(r.unsigned_abs());
    num < r2 * den
}

/// Distance from `p` to segment `s` (reporting / heuristics only).
#[must_use]
pub fn point_segment_dist(p: Point, s: Segment) -> f64 {
    let (num, den) = point_segment_dist2_rational(p, s);
    #[allow(clippy::cast_precision_loss)]
    (num as f64 / den as f64).sqrt()
}

/// Exact test: is the distance between two closed segments strictly less than `r`?
#[must_use]
pub fn segment_segment_dist_lt(s: Segment, t: Segment, r: Coord) -> bool {
    if r <= 0 {
        return false;
    }
    if segments_intersect(s, t) {
        return true;
    }
    point_segment_dist_lt(s.a, t, r)
        || point_segment_dist_lt(s.b, t, r)
        || point_segment_dist_lt(t.a, s, r)
        || point_segment_dist_lt(t.b, s, r)
}

/// Distance between two closed segments (reporting only).
#[must_use]
pub fn segment_segment_dist(s: Segment, t: Segment) -> f64 {
    if segments_intersect(s, t) {
        return 0.0;
    }
    point_segment_dist(s.a, t)
        .min(point_segment_dist(s.b, t))
        .min(point_segment_dist(t.a, s))
        .min(point_segment_dist(t.b, s))
}

/// The universal copper primitive: a **convex** point set (1 point = disk centre,
/// 2 points = capsule spine, ≥3 points = convex polygon, any winding) Minkowski-summed
/// with a disk of `radius`.
///
/// Tracks are 2-point hulls with `radius = width / 2`; vias and circular pads are
/// 1-point hulls; oval pads are capsules; rectangles are 4-point hulls with radius 0;
/// round-rects are the inner rectangle plus the corner radius. Shapes that are not
/// convex are represented by a conservative convex *superset* (never a subset), so a
/// clean DRC result on hulls implies a clean result on the true shapes.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Hull {
    pub pts: SmallVec<[Point; 4]>,
    pub radius: Coord,
}

impl Hull {
    #[must_use]
    pub fn disk(c: Point, radius: Coord) -> Self {
        Self { pts: smallvec::smallvec![c], radius }
    }

    #[must_use]
    pub fn capsule(a: Point, b: Point, radius: Coord) -> Self {
        if a == b {
            return Self::disk(a, radius);
        }
        Self { pts: smallvec::smallvec![a, b], radius }
    }

    /// Convex polygon (caller guarantees convexity) with an optional rounding radius.
    #[must_use]
    pub fn polygon(pts: &[Point], radius: Coord) -> Self {
        Self { pts: pts.iter().copied().collect(), radius }
    }

    /// Exact: does the disk of radius `s` centred at `q` lie inside this hull
    /// (i.e. is `q` inside the hull eroded by `s`)? Boundary cases count as outside
    /// (conservative).
    #[must_use]
    pub fn contains_disk(&self, q: Point, s: Coord) -> bool {
        let h = self;
        if h.radius >= s {
            let inner = Hull { pts: h.pts.clone(), radius: h.radius - s };
            if inner.radius == 0 && inner.pts.len() < 3 {
                return inner.pts.contains(&q)
                    || (inner.pts.len() == 2
                        && orient(inner.pts[0], inner.pts[1], q) == 0
                        && Segment::new(inner.pts[0], inner.pts[1]).bbox().contains(q));
            }
            return inner.clearance_violated(&Hull::disk(q, 0), 0);
        }
        if h.pts.len() < 3 {
            return false;
        }
        let poly = Polygon::new(h.pts.to_vec());
        let need = s - h.radius;
        poly.contains(q) && !poly.edges().any(|e| point_segment_dist_lt(q, e, need))
    }

    /// Axis-aligned bounding box including the radius.
    #[must_use]
    pub fn bbox(&self) -> Rect {
        Rect::from_points(&self.pts)
            .map_or(Rect::new(Point::ORIGIN, Point::ORIGIN), |r| r.inflate(self.radius))
    }

    #[must_use]
    pub fn translated(&self, v: Point) -> Self {
        Self { pts: self.pts.iter().map(|&p| p + v).collect(), radius: self.radius }
    }

    /// `true` when every vertex and the radius satisfy INV-GEOM-1.
    #[must_use]
    pub fn in_bounds(&self) -> bool {
        self.pts.iter().all(|p| p.in_bounds()) && (0..=MAX_COORD).contains(&self.radius)
    }

    /// Boundary edges of the core (for a 2-point hull: the single spine segment).
    fn edges(&self) -> impl Iterator<Item = Segment> + '_ {
        let n = self.pts.len();
        let count = match n {
            0 | 1 => 0,
            2 => 1,
            _ => n,
        };
        (0..count).map(move |i| Segment::new(self.pts[i], self.pts[(i + 1) % n]))
    }

    /// Exact: does the *core* polygon (≥3 vertices) contain `p` (boundary inclusive)?
    fn core_contains(&self, p: Point) -> bool {
        if self.pts.len() < 3 {
            return false;
        }
        let mut sign = 0i8;
        for e in self.edges() {
            let o = orient(e.a, e.b, p);
            if o != 0 {
                if sign == 0 {
                    sign = o;
                } else if sign != o {
                    return false;
                }
            }
        }
        true
    }

    /// Exact: do the two *cores* intersect (closed sets)?
    fn cores_intersect(&self, o: &Self) -> bool {
        match (self.pts.len(), o.pts.len()) {
            (0, _) | (_, 0) => false,
            (1, 1) => self.pts[0] == o.pts[0],
            (1, _) => {
                let p = self.pts[0];
                o.core_contains(p)
                    || o.edges().any(|e| orient(e.a, e.b, p) == 0 && on_segment_bbox(e, p))
            }
            (_, 1) => o.cores_intersect(self),
            _ => {
                self.edges().any(|e| o.edges().any(|f| segments_intersect(e, f)))
                    || o.core_contains(self.pts[0])
                    || self.core_contains(o.pts[0])
            }
        }
    }

    /// **Exact DRC predicate.** Returns `true` when the gap between the two shapes is
    /// strictly smaller than `clearance` (i.e. the pair *violates* the clearance rule).
    /// Overlapping shapes always violate any clearance `>= 0` unless both are degenerate
    /// and merely touch with `clearance == 0`.
    #[must_use]
    pub fn clearance_violated(&self, o: &Self, clearance: Coord) -> bool {
        let r = clearance + self.radius + o.radius;
        if self.cores_intersect(o) {
            return r > 0 || self.pts.len() >= 3 || o.pts.len() >= 3;
        }
        if r <= 0 {
            return false;
        }
        // Cores are disjoint convex sets: their distance is attained between a vertex
        // of one and an edge (or vertex) of the other.
        let vert_vs = |a: &Self, b: &Self| {
            a.pts.iter().any(|&p| {
                if b.pts.len() == 1 {
                    point_segment_dist_lt(p, Segment::new(b.pts[0], b.pts[0]), r)
                } else {
                    b.edges().any(|e| point_segment_dist_lt(p, e, r))
                }
            })
        };
        vert_vs(self, o) || vert_vs(o, self)
    }

    /// Gap between the two shapes in nm (0 when overlapping). Reporting only.
    #[must_use]
    pub fn distance(&self, o: &Self) -> f64 {
        #[allow(clippy::cast_precision_loss)]
        let radii = (self.radius + o.radius) as f64;
        if self.cores_intersect(o) {
            return 0.0;
        }
        let mut best = f64::INFINITY;
        let segs = |h: &Self| -> SmallVec<[Segment; 4]> {
            if h.pts.len() == 1 {
                smallvec::smallvec![Segment::new(h.pts[0], h.pts[0])]
            } else {
                h.edges().collect()
            }
        };
        for e in segs(self) {
            for f in segs(o) {
                best = best.min(segment_segment_dist(e, f));
            }
        }
        (best - radii).max(0.0)
    }

    /// Exact: does this hull come strictly closer than `clearance` to segment `s`
    /// (e.g. a board-edge segment)?
    #[must_use]
    pub fn segment_clearance_violated(&self, s: Segment, clearance: Coord) -> bool {
        self.clearance_violated(&Self::capsule(s.a, s.b, 0), clearance)
    }

    /// Representative interior point (first vertex).
    #[must_use]
    pub fn anchor(&self) -> Point {
        self.pts.first().copied().unwrap_or_default()
    }
}

/// A simple (non-self-intersecting) polygon, implicitly closed, any winding.
/// Used for board outlines, keep-out areas and courtyards. Need not be convex.
#[derive(Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Polygon {
    pub pts: Vec<Point>,
}

impl Polygon {
    #[must_use]
    pub fn new(pts: Vec<Point>) -> Self {
        Self { pts }
    }

    #[must_use]
    pub fn from_rect(r: Rect) -> Self {
        Self { pts: r.corners().to_vec() }
    }

    #[must_use]
    pub fn bbox(&self) -> Option<Rect> {
        Rect::from_points(&self.pts)
    }

    pub fn edges(&self) -> impl Iterator<Item = Segment> + '_ {
        let n = self.pts.len();
        (0..if n >= 2 { n } else { 0 })
            .map(move |i| Segment::new(self.pts[i], self.pts[(i + 1) % n]))
    }

    /// Exact point-in-polygon (crossing number). Points on the boundary count as inside.
    #[must_use]
    pub fn contains(&self, p: Point) -> bool {
        if self.pts.len() < 3 {
            return false;
        }
        let mut inside = false;
        for e in self.edges() {
            if orient(e.a, e.b, p) == 0 && on_segment_bbox(e, p) {
                return true;
            }
            let (a, b) = (e.a, e.b);
            if (a.y > p.y) != (b.y > p.y) {
                // x-coordinate of the edge at height p.y compared exactly:
                // p.x < a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y)
                let lhs = i128::from(p.x - a.x) * i128::from(b.y - a.y);
                let rhs = i128::from(p.y - a.y) * i128::from(b.x - a.x);
                let crosses = if b.y > a.y { lhs < rhs } else { lhs > rhs };
                if crosses {
                    inside = !inside;
                }
            }
        }
        inside
    }

    /// Twice the signed area (positive = counter-clockwise in a y-up frame).
    #[must_use]
    pub fn area2(&self) -> i128 {
        let n = self.pts.len();
        (0..n).map(|i| self.pts[i].cross(self.pts[(i + 1) % n])).sum()
    }

    /// Exact: is hull `h` entirely inside this polygon with at least `clearance` to
    /// every boundary edge?
    #[must_use]
    pub fn contains_hull_with_clearance(&self, h: &Hull, clearance: Coord) -> bool {
        if !h.pts.iter().all(|&p| self.contains(p)) {
            return false;
        }
        !self.edges().any(|e| h.segment_clearance_violated(e, clearance))
    }

    /// Exact: does hull `h` come closer than `clearance` to this polygon's area
    /// (overlap or proximity)?
    #[must_use]
    pub fn hull_violates(&self, h: &Hull, clearance: Coord) -> bool {
        // Either the hull core reaches into the area, or some boundary edge is within
        // `clearance` of the hull (this also covers an area entirely inside the hull,
        // because then its edges intersect the hull core).
        h.pts.iter().any(|&p| self.contains(p))
            || self.edges().any(|e| h.segment_clearance_violated(e, clearance))
    }

    /// Exact: do two simple polygons overlap (interiors or boundaries touch)?
    #[must_use]
    pub fn intersects(&self, o: &Self) -> bool {
        if self.pts.len() < 3 || o.pts.len() < 3 {
            return false;
        }
        match (self.bbox(), o.bbox()) {
            (Some(a), Some(b)) if !a.intersects(b) => return false,
            _ => {}
        }
        self.edges().any(|e| o.edges().any(|f| segments_intersect(e, f)))
            || self.contains(o.pts[0])
            || o.contains(self.pts[0])
    }

    #[must_use]
    pub fn translated(&self, v: Point) -> Self {
        Self { pts: self.pts.iter().map(|&p| p + v).collect() }
    }

    /// Rotates every vertex about `pivot` (KiCad convention, see [`Point::rotated`]).
    #[must_use]
    pub fn rotated_about(&self, pivot: Point, a: Angle) -> Self {
        Self { pts: self.pts.iter().map(|&p| (p - pivot).rotated(a) + pivot).collect() }
    }
}

/// Andrew's monotone-chain convex hull (exact). Returns vertices without repetition.
#[must_use]
pub fn convex_hull(points: &[Point]) -> Vec<Point> {
    let mut pts: Vec<Point> = points.to_vec();
    pts.sort_unstable();
    pts.dedup();
    if pts.len() <= 2 {
        return pts;
    }
    let mut lower: Vec<Point> = Vec::with_capacity(pts.len());
    for &p in &pts {
        while lower.len() >= 2 && orient(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 0 {
            lower.pop();
        }
        lower.push(p);
    }
    let mut upper: Vec<Point> = Vec::with_capacity(pts.len());
    for &p in pts.iter().rev() {
        while upper.len() >= 2 && orient(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 0 {
            upper.pop();
        }
        upper.push(p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

/// The eight octilinear directions on a routing grid, in grid units `(di, dj)`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub enum Dir8 {
    E,
    NE,
    N,
    NW,
    W,
    SW,
    S,
    SE,
}

impl Dir8 {
    pub const ALL: [Self; 8] =
        [Self::E, Self::NE, Self::N, Self::NW, Self::W, Self::SW, Self::S, Self::SE];

    /// Grid step. Note: grid `j` grows with board `y` (downwards), so `N` is `dj = -1`.
    #[must_use]
    pub const fn delta(self) -> (i32, i32) {
        match self {
            Self::E => (1, 0),
            Self::NE => (1, -1),
            Self::N => (0, -1),
            Self::NW => (-1, -1),
            Self::W => (-1, 0),
            Self::SW => (-1, 1),
            Self::S => (0, 1),
            Self::SE => (1, 1),
        }
    }

    /// Direction of a unit grid step `(di, dj)`; `None` for anything else.
    #[must_use]
    pub fn from_delta(di: i64, dj: i64) -> Option<Self> {
        Self::ALL.into_iter().find(|d| {
            let (a, b) = d.delta();
            (i64::from(a), i64::from(b)) == (di, dj)
        })
    }

    /// Direction of a non-zero octilinear vector (board coordinates, +y down);
    /// `None` for zero or non-octilinear vectors.
    #[must_use]
    pub fn from_vector(v: Point) -> Option<Self> {
        if v == Point::ORIGIN || !v.is_octilinear() {
            return None;
        }
        Self::from_delta(v.x.signum(), v.y.signum())
    }

    #[must_use]
    pub const fn is_diagonal(self) -> bool {
        matches!(self, Self::NE | Self::NW | Self::SW | Self::SE)
    }

    #[must_use]
    pub const fn index(self) -> usize {
        self as usize
    }

    #[must_use]
    pub const fn opposite(self) -> Self {
        Self::ALL[(self as usize + 4) % 8]
    }

    /// Turn magnitude between two directions in 45° steps (0..=4).
    #[must_use]
    pub const fn turn(self, o: Self) -> u8 {
        let d = (self as i32 - o as i32).rem_euclid(8);
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        if d > 4 { (8 - d) as u8 } else { d as u8 }
    }
}

#[cfg(test)]
mod tests;
