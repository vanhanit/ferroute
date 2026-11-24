// SPDX-License-Identifier: AGPL-3.0-only
use super::*;
use proptest::prelude::*;

fn p(x: Coord, y: Coord) -> Point {
    Point::new(x, y)
}

#[test]
fn rotation_matches_kicad_convention() {
    // KiCad RotatePoint(+90°) maps +x to -y (up on screen, y-down coordinates).
    assert_eq!(p(1, 0).rotated(Angle::from_degrees(90.0)), p(0, -1));
    assert_eq!(p(1, 0).rotated(Angle::from_degrees(180.0)), p(-1, 0));
    assert_eq!(p(1, 0).rotated(Angle::from_degrees(270.0)), p(0, 1));
    assert_eq!(p(1, 0).rotated(Angle::from_degrees(-90.0)), p(0, 1));
    assert_eq!(p(1_000_000, 0).rotated(Angle::from_degrees(45.0)), p(707_107, -707_107));
}

#[test]
fn angle_normalisation() {
    assert_eq!(Angle::from_degrees(-90.0).degrees(), 270.0);
    assert_eq!(Angle::from_degrees(720.0).degrees(), 0.0);
    assert_eq!(Angle::from_degrees(-0.0).quadrant(), Some(0));
    assert_eq!(Angle::from_degrees(45.0).quadrant(), None);
}

#[test]
fn segment_intersection_cases() {
    let s = Segment::new(p(0, 0), p(10, 10));
    assert!(segments_intersect(s, Segment::new(p(0, 10), p(10, 0))));
    assert!(segments_intersect(s, Segment::new(p(10, 10), p(20, 0)))); // touching endpoint
    assert!(segments_intersect(s, Segment::new(p(5, 5), p(20, 20)))); // collinear overlap
    assert!(!segments_intersect(s, Segment::new(p(11, 11), p(20, 20)))); // collinear disjoint
    assert!(!segments_intersect(s, Segment::new(p(1, 0), p(10, 9)))); // parallel
}

#[test]
fn track_clearance_exact_boundary() {
    // Two horizontal 0.2 mm tracks whose centrelines are 0.4 mm apart: gap is exactly 0.2 mm.
    let a = Hull::capsule(p(0, 0), p(1_000_000, 0), 100_000);
    let b = Hull::capsule(p(0, 400_000), p(1_000_000, 400_000), 100_000);
    assert!(!a.clearance_violated(&b, 200_000), "exactly at clearance is legal");
    assert!(a.clearance_violated(&b, 200_001), "1 nm more clearance must fail");
    assert!((a.distance(&b) - 200_000.0).abs() < 1e-6);
}

#[test]
fn rect_pad_vs_diagonal_track() {
    let pad = Hull::polygon(&Rect::new(p(-500_000, -500_000), p(500_000, 500_000)).corners(), 0);
    // A diagonal track passing near the top-right corner at distance d from (0.5,0.5)mm.
    let t = Hull::capsule(p(1_000_000, 0), p(0, 1_000_000), 0);
    // Line x + y = 1 mm; the corner (0.5, 0.5) lies on it -> touching.
    assert!(pad.clearance_violated(&t, 1));
    let t2 = Hull::capsule(p(1_200_000, 0), p(0, 1_200_000), 0);
    // Distance from corner to x+y=1.2mm is 0.2mm/sqrt(2) ≈ 141_421 nm.
    assert!(pad.clearance_violated(&t2, 141_422));
    assert!(!pad.clearance_violated(&t2, 141_421));
}

#[test]
fn polygon_containment() {
    let sq = Polygon::from_rect(Rect::new(p(0, 0), p(10, 10)));
    assert!(sq.contains(p(5, 5)));
    assert!(sq.contains(p(0, 5)), "boundary counts as inside");
    assert!(sq.contains(p(10, 10)));
    assert!(!sq.contains(p(11, 5)));
    // Concave "L" shape.
    let l = Polygon::new(vec![p(0, 0), p(10, 0), p(10, 4), p(4, 4), p(4, 10), p(0, 10)]);
    assert!(l.contains(p(2, 8)));
    assert!(!l.contains(p(8, 8)));
    let h = Hull::disk(p(2, 2), 1);
    assert!(l.contains_hull_with_clearance(&h, 1));
    assert!(!l.contains_hull_with_clearance(&h, 2));
}

#[test]
fn polygon_contains_disk_exactly() {
    let l = Polygon::new(vec![p(0, 0), p(100, 0), p(100, 40), p(40, 40), p(40, 100), p(0, 100)]);
    assert!(l.contains_disk(p(20, 20), 18));
    assert!(!l.contains_disk(p(20, 20), 20), "touches the boundary");
    assert!(!l.contains_disk(p(30, 30), 15), "reflex corner");
    assert!(!l.contains_disk(p(80, 80), 1), "outside");
    assert!(l.contains_disk(p(70, 20), 18));
}

#[test]
fn convex_hull_of_square_with_interior_point() {
    let h = convex_hull(&[p(0, 0), p(10, 0), p(10, 10), p(0, 10), p(5, 5), p(10, 0)]);
    assert_eq!(h.len(), 4);
}

#[test]
fn dir8_turns() {
    assert_eq!(Dir8::E.turn(Dir8::E), 0);
    assert_eq!(Dir8::E.turn(Dir8::NE), 1);
    assert_eq!(Dir8::E.turn(Dir8::W), 4);
    assert_eq!(Dir8::SE.turn(Dir8::E), 1);
    assert_eq!(Dir8::N.opposite(), Dir8::S);
}

const R: Coord = 2_000_000; // keep proptest coordinates in a 4 mm box for good collision rates

fn arb_point() -> impl Strategy<Value = Point> {
    (-R..R, -R..R).prop_map(|(x, y)| p(x, y))
}

fn arb_hull() -> impl Strategy<Value = Hull> {
    prop_oneof![
        (arb_point(), 0..R / 4).prop_map(|(c, r)| Hull::disk(c, r)),
        (arb_point(), arb_point(), 0..R / 4).prop_map(|(a, b, r)| Hull::capsule(a, b, r)),
        (arb_point(), 1..R / 2, 1..R / 2, 0..R / 8)
            .prop_map(|(c, w, h, r)| { Hull::polygon(&Rect::new(c, c + p(w, h)).corners(), r) }),
    ]
}

/// Dense sampling-based reference: minimal distance between hull boundaries estimated
/// from below by sampling the spine/edges (f64). Used only to sanity-check the exact
/// predicate away from the decision boundary.
fn approx_gap(a: &Hull, b: &Hull) -> f64 {
    a.distance(b)
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(2048))]

    #[test]
    fn clearance_is_symmetric(a in arb_hull(), b in arb_hull(), c in 0..R / 4) {
        prop_assert_eq!(a.clearance_violated(&b, c), b.clearance_violated(&a, c));
    }

    #[test]
    fn clearance_is_translation_invariant(a in arb_hull(), b in arb_hull(), c in 0..R / 4, v in arb_point()) {
        prop_assert_eq!(a.clearance_violated(&b, c), a.translated(v).clearance_violated(&b.translated(v), c));
    }

    #[test]
    fn clearance_is_monotone_in_c(a in arb_hull(), b in arb_hull(), c in 0..R / 4, dc in 0..R / 4) {
        // If a pair violates clearance c it must violate every larger clearance.
        if a.clearance_violated(&b, c) {
            prop_assert!(a.clearance_violated(&b, c + dc));
        }
    }

    #[test]
    fn exact_agrees_with_float_away_from_boundary(a in arb_hull(), b in arb_hull(), c in 0..R / 4) {
        let gap = approx_gap(&a, &b);
        #[allow(clippy::cast_precision_loss)]
        let cf = c as f64;
        // Only compare where f64 is unambiguous (2 nm guard band).
        if gap > cf + 2.0 {
            prop_assert!(!a.clearance_violated(&b, c), "gap {gap} > c {c} but violated");
        } else if gap + 2.0 < cf {
            prop_assert!(a.clearance_violated(&b, c), "gap {gap} < c {c} but not violated");
        }
    }

    #[test]
    fn rotation_by_quadrants_is_exact_and_invertible(q in arb_point(), k in 0u8..4) {
        let a = Angle::from_degrees(f64::from(k) * 90.0);
        let back = Angle::from_degrees(-f64::from(k) * 90.0);
        prop_assert_eq!(q.rotated(a).rotated(back), q);
        prop_assert_eq!(q.rotated(a).norm2(), q.norm2());
    }

    #[test]
    fn rect_polygon_contains_matches_rect(q in arb_point(), a in arb_point(), b in arb_point()) {
        let r = Rect::new(a, b);
        if r.width() > 0 && r.height() > 0 {
            prop_assert_eq!(Polygon::from_rect(r).contains(q), r.contains(q));
        }
    }

    #[test]
    fn extreme_coordinates_do_not_overflow(
        ax in -MAX_COORD..=MAX_COORD, ay in -MAX_COORD..=MAX_COORD,
        bx in -MAX_COORD..=MAX_COORD, by in -MAX_COORD..=MAX_COORD,
        cx in -MAX_COORD..=MAX_COORD, cy in -MAX_COORD..=MAX_COORD,
        r in 0..=MAX_COORD,
    ) {
        // Must not panic (debug builds trap on overflow).
        let s = Segment::new(p(ax, ay), p(bx, by));
        let _ = point_segment_dist_lt(p(cx, cy), s, r);
        let _ = segment_segment_dist_lt(s, Segment::new(p(cx, cy), p(ay, ax)), r);
        let h = Hull::capsule(p(ax, ay), p(bx, by), r / 2);
        let _ = h.clearance_violated(&Hull::disk(p(cx, cy), r / 2), r / 2);
    }
}

#[test]
fn dir8_from_delta_roundtrips_and_rejects_non_unit_steps() {
    for d in Dir8::ALL {
        let (di, dj) = d.delta();
        assert_eq!(Dir8::from_delta(i64::from(di), i64::from(dj)), Some(d));
    }
    assert_eq!(Dir8::from_delta(0, 0), None);
    assert_eq!(Dir8::from_delta(2, 0), None);
    assert_eq!(Dir8::from_delta(1, 2), None);
}

#[test]
fn octilinear_vectors_and_their_direction() {
    assert!(p(0, 5).is_octilinear());
    assert!(p(-7, 0).is_octilinear());
    assert!(p(3, -3).is_octilinear());
    assert!(p(0, 0).is_octilinear());
    assert!(!p(3, 2).is_octilinear());
    // Grid j grows with board y, so (+x, +y) is SE and (0, -y) is N.
    assert_eq!(Dir8::from_vector(p(5, 5)), Some(Dir8::SE));
    assert_eq!(Dir8::from_vector(p(0, -9)), Some(Dir8::N));
    assert_eq!(Dir8::from_vector(p(-4, 0)), Some(Dir8::W));
    assert_eq!(Dir8::from_vector(p(3, 2)), None);
    assert_eq!(Dir8::from_vector(p(0, 0)), None);
}

#[test]
fn point_scales_exactly() {
    assert_eq!(Point::new(3, -4) * 5, Point::new(15, -20));
    assert_eq!(Point::new(2, 7) * -1, -Point::new(2, 7));
}
