// SPDX-License-Identifier: AGPL-3.0-only
//! Accordion meanders for length tuning: uniform square-wave bumps whose every corner is a
//! circular fillet (true arcs, never mitres) on an octilinear straight run. Pure integer
//! tangent geometry; the caller verifies the result with the exact DRC oracle
//! ([`crate::tune`]).
//!
//! Skeleton of bump `k` in local coordinates (along `x`, normal `y`), spacing `s` and
//! amplitude `h` (the top run and the gap between bumps both equal `s`, so bumps repeat
//! every `2s`): corners `(x,0) (x,h) (x+s,h) (x+s,0)`. Each corner is rounded with radius
//! `r ≤ s/2`: tangent points `T1 = K − d_in·r`, `T2 = K + d_out·r`, centre
//! `C = T1 + d_out·r`, arc mid point `C + (K − C)/√2`. With `r = s/2` the top is a full
//! semicircle and the base turns are quarter circles; the legs are straight for `h − 2r`.
//! Each bump adds `2h − (8 − 2π)·r` of length.

use ferroute_geom::{Coord, Point, arc::Arc};
use std::f64::consts::{FRAC_1_SQRT_2, PI};

/// One element of a meander centre line.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Piece {
    /// Straight octilinear segment.
    Line(Point, Point),
    /// Quarter-circle fillet.
    Arc(Arc),
}

/// Shape of a meander on one straight run (local units, see [`run_frame`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MeanderSpec {
    /// Centre-to-centre spacing of parallel legs (even).
    pub spacing: Coord,
    /// Bump height above the original run (`≥ 2·radius`).
    pub amplitude: Coord,
    /// Fillet radius of every corner (`≤ spacing / 2`).
    pub radius: Coord,
    /// Number of bumps.
    pub bumps: u32,
    /// Side of the run the bumps rise on (+1 = left of `a → b`, −1 = right).
    pub side: i8,
}

/// Length lost per bump and unit radius by rounding the four corners: `8 − 2π`.
const FILLET_LOSS: f64 = 8.0 - 2.0 * PI;
/// Gain per unit radius of a minimum bump (`h = 2r`): `4 − (8 − 2π)`.
const MIN_GAIN: f64 = 2.0 * PI - 4.0;

impl MeanderSpec {
    /// Length along the run occupied by the bumps (`2s` per bump, minus the trailing gap).
    #[must_use]
    pub fn extent(&self) -> Coord {
        Coord::from(self.bumps) * 2 * self.spacing - self.spacing
    }

    /// Extra length (local units) the meander adds over the straight run.
    #[must_use]
    pub fn extra_length(&self) -> f64 {
        #[allow(clippy::cast_precision_loss)]
        let (h, r) = (self.amplitude as f64, self.radius as f64);
        f64::from(self.bumps) * (2.0 * h - FILLET_LOSS * r)
    }

    /// Plans the meander for a run of length `run` that must gain `need` (all local units),
    /// with bumps no higher than `max_amplitude`, at least `margin` of straight run before
    /// the first and after the last fillet, and fillets no tighter than `min_radius`.
    /// Uses the lowest amplitude that meets the need (taller bumps only to gain more on a
    /// short run); a need too small for a bump of radius `spacing/2` shrinks the radius.
    /// `None` if nothing fits, the need is negligible or even `min_radius` overshoots it.
    #[must_use]
    pub fn plan(
        run: Coord,
        need: f64,
        spacing: Coord,
        max_amplitude: Coord,
        margin: Coord,
        min_radius: Coord,
        side: i8,
    ) -> Option<Self> {
        if spacing <= 0 || need < 1_000.0 {
            return None;
        }
        let spacing = spacing + spacing % 2;
        let r0 = spacing / 2;
        if r0 < min_radius.max(1) || max_amplitude < 2 * r0 {
            return None;
        }
        let room = run - 2 * (margin + r0);
        let capacity = u32::try_from((room + spacing).div_euclid(2 * spacing)).ok()?.min(1 << 16);
        if capacity == 0 {
            return None;
        }
        #[allow(clippy::cast_precision_loss)]
        let (per_full, per_min) =
            (2.0 * max_amplitude as f64 - FILLET_LOSS * r0 as f64, MIN_GAIN * r0 as f64);
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let wanted = (need / per_full).ceil().max(1.0) as u32;
        let bumps = wanted.min(capacity);
        let per_bump = (need / f64::from(bumps)).min(per_full);
        #[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
        let (radius, amplitude) = if per_bump >= per_min {
            let h = ((per_bump + FILLET_LOSS * r0 as f64) / 2.0 + 1e-9).floor() as Coord;
            (r0, h.max(2 * r0))
        } else {
            let r = (per_bump / MIN_GAIN + 1e-9).floor() as Coord;
            if r < min_radius.max(1) {
                return None;
            }
            (r, 2 * r)
        };
        Some(Self { spacing, amplitude, radius, bumps, side })
    }

    /// [`MeanderSpec::plan`] for the run `a → b` in real nanometres: on a 45° run the
    /// local unit is √2 long, so spacing, margin and minimum radius are rounded *up*
    /// (never tighter) and the amplitude *down* (never taller) when converted.
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub fn plan_run(
        a: Point,
        b: Point,
        need: f64,
        spacing: Coord,
        max_amplitude: Coord,
        margin: Coord,
        min_radius: Coord,
        side: i8,
    ) -> Option<Self> {
        let (len, unit) = run_frame(a, b)?;
        #[allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]
        let (up, down) = (
            |v: Coord| (v as f64 / unit).ceil() as Coord,
            |v: Coord| (v as f64 / unit).floor() as Coord,
        );
        Self::plan(
            len,
            need / unit,
            up(spacing),
            down(max_amplitude),
            up(margin),
            up(min_radius),
            side,
        )
    }

    /// Centre line from `a` to `b` (an octilinear run) with the meander centred on the run:
    /// straight lines joined only through tangent arcs. `None` if `a → b` is not
    /// octilinear, the spec is inconsistent or the run is too short.
    #[must_use]
    pub fn pieces(&self, a: Point, b: Point) -> Option<Vec<Piece>> {
        let (len, _) = run_frame(a, b)?;
        let (s, r, h) = (self.spacing, self.radius, self.amplitude);
        let extent = self.extent();
        if self.bumps == 0 || r <= 0 || 2 * r > s || h < 2 * r || extent > len {
            return None;
        }
        let x0 = (len - extent) / 2;
        if x0 < r {
            return None;
        }
        let d = b - a;
        let u = Point::new(d.x.signum(), d.y.signum());
        // Left normal of u in y-down screen coordinates is (u.y, −u.x); `side` flips it.
        let n = Point::new(u.y, -u.x) * Coord::from(self.side);
        let at = |(x, y): (Coord, Coord)| a + u * x + n * y;
        let (fwd, up, down) = ((1, 0), (0, 1), (0, -1));
        let mut out = Vec::new();
        let mut cur = a;
        let line_to = |out: &mut Vec<Piece>, cur: &mut Point, p: Point| {
            if p != *cur {
                out.push(Piece::Line(*cur, p));
            }
            *cur = p;
        };
        for k in 0..Coord::from(self.bumps) {
            let x = x0 + 2 * s * k;
            let corners = [
                ((x, 0), fwd, up),
                ((x, h), up, fwd),
                ((x + s, h), fwd, down),
                ((x + s, 0), down, fwd),
            ];
            for (kp, din, dout) in corners {
                let back = |(px, py): (Coord, Coord), (dx, dy): (Coord, Coord), m: Coord| {
                    (px + dx * m, py + dy * m)
                };
                let t1 = back(kp, din, -r);
                let t2 = back(kp, dout, r);
                let c = back(t1, dout, r);
                let (kr, cr) = (at(kp), at(c));
                #[allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]
                let mid = {
                    let f =
                        |k: Coord, c: Coord| (c as f64 + (k - c) as f64 * FRAC_1_SQRT_2).round();
                    Point::new(f(kr.x, cr.x) as Coord, f(kr.y, cr.y) as Coord)
                };
                line_to(&mut out, &mut cur, at(t1));
                out.push(Piece::Arc(Arc::new(at(t1), mid, at(t2))));
                cur = at(t2);
            }
        }
        line_to(&mut out, &mut cur, b);
        Some(out)
    }
}

/// Local length of an octilinear run and the real length of one local unit: axis-aligned
/// runs step in unit vectors (1 nm), 45° runs in diagonal steps (√2 nm).
#[must_use]
pub fn run_frame(a: Point, b: Point) -> Option<(Coord, f64)> {
    let d = b - a;
    match (d.x.abs(), d.y.abs()) {
        (0, 0) => None,
        (x, 0) | (0, x) => Some((x, 1.0)),
        (x, y) if x == y => Some((x, std::f64::consts::SQRT_2)),
        _ => None,
    }
}

/// Euclidean centre-line length of a piece chain.
#[must_use]
pub fn pieces_length(p: &[Piece]) -> f64 {
    p.iter()
        .map(|piece| match piece {
            Piece::Line(a, b) => a.dist(*b),
            Piece::Arc(arc) => arc.length(),
        })
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    const S: Coord = 750_000;
    const RMIN: Coord = 125_001;

    fn runs() -> Vec<(Point, Point)> {
        vec![
            (Point::new(0, 0), Point::new(20_000_000, 0)),
            (Point::new(5_000_000, 0), Point::new(5_000_000, -18_000_000)),
            (Point::new(0, 7_000_000), Point::new(-21_000_000, 7_000_000)),
            (Point::new(0, 0), Point::new(15_000_000, 15_000_000)),
            (Point::new(3_000_000, 2_000_000), Point::new(-12_000_000, 17_000_000)),
        ]
    }

    fn start_of(p: &Piece) -> Point {
        match p {
            Piece::Line(a, _) => *a,
            Piece::Arc(a) => a.start,
        }
    }

    fn end_of(p: &Piece) -> Point {
        match p {
            Piece::Line(_, b) => *b,
            Piece::Arc(a) => a.end,
        }
    }

    /// Unit-free cosine of the angle between two vectors given as f64 pairs.
    fn cos(u: (f64, f64), v: (f64, f64)) -> f64 {
        (u.0 * v.0 + u.1 * v.1) / (u.0.hypot(u.1) * v.0.hypot(v.1))
    }

    #[allow(clippy::cast_precision_loss)]
    fn vec2(p: Point) -> (f64, f64) {
        (p.x as f64, p.y as f64)
    }

    fn radial(arc: &Arc, p: Point) -> (f64, f64) {
        let c = arc.circle().expect("circle");
        (vec2(p).0 - c.cx, vec2(p).1 - c.cy)
    }

    #[test]
    fn plan_meets_the_gain_with_uniform_bumps() {
        let spec = MeanderSpec::plan(20_000_000, 9_000_000.0, S, 3 * S, S, RMIN, 1).expect("fits");
        assert_eq!(spec.bumps, 3, "a full 2.25 mm bump gains ~3.86 mm: {spec:?}");
        assert!(spec.amplitude <= 3 * S && spec.amplitude >= 2 * spec.radius);
        assert_eq!(spec.radius, S / 2);
        assert!((spec.extra_length() - 9_000_000.0).abs() < 3.0 * 2.0, "{}", spec.extra_length());
        assert!(spec.extent() + 2 * S <= 20_000_000);
    }

    #[test]
    fn short_runs_give_partial_gain_and_tiny_needs_shrink_the_radius() {
        let spec = MeanderSpec::plan(4_000_000, 50_000_000.0, S, 3 * S, S, RMIN, 1).expect("one");
        assert_eq!(spec.bumps, 1, "only one bump fits in 4 mm");
        assert_eq!(spec.amplitude, 3 * S, "full height when partial");
        let tiny = MeanderSpec::plan(20_000_000, 400_000.0, S, 3 * S, S, RMIN, -1).expect("tiny");
        assert_eq!(tiny.bumps, 1);
        assert!(tiny.radius < S / 2 && tiny.radius >= RMIN, "{tiny:?}");
        assert_eq!(tiny.amplitude, 2 * tiny.radius);
        assert!((tiny.extra_length() - 400_000.0).abs() <= 3.0, "{}", tiny.extra_length());
        assert_eq!(MeanderSpec::plan(1_000_000, 1e6, S, 3 * S, S, RMIN, 1), None, "no room");
        assert_eq!(MeanderSpec::plan(20_000_000, 10.0, S, 3 * S, S, RMIN, 1), None, "negligible");
        assert_eq!(
            MeanderSpec::plan(20_000_000, 100_000.0, S, 3 * S, S, RMIN, 1),
            None,
            "even the minimum radius overshoots"
        );
    }

    #[test]
    fn pieces_are_a_tangent_chain_of_octilinear_lines_and_quarter_arcs() {
        for (a, b) in runs() {
            for side in [1, -1] {
                for need in [6_500_000.0, 700_000.0] {
                    let spec =
                        MeanderSpec::plan_run(a, b, need, S, 3 * S, S, RMIN, side).expect("fits");
                    let (_, unit) = run_frame(a, b).expect("octilinear");
                    let ps = spec.pieces(a, b).expect("pieces");
                    assert_eq!((start_of(&ps[0]), end_of(ps.last().expect("end"))), (a, b));
                    for w in ps.windows(2) {
                        assert_eq!(end_of(&w[0]), start_of(&w[1]), "continuous chain");
                        match (&w[0], &w[1]) {
                            (Piece::Line(p, q), Piece::Line(r, t)) => {
                                let (u, v) = (*q - *p, *t - *r);
                                let cross = i128::from(u.x) * i128::from(v.y)
                                    - i128::from(u.y) * i128::from(v.x);
                                assert_eq!(cross, 0, "no corner without an arc at {q:?}");
                            }
                            (Piece::Line(p, q), Piece::Arc(arc)) => {
                                let c = cos(vec2(*q - *p), radial(arc, arc.start));
                                assert!(c.abs() < 1e-5, "tangent into arc: {c}");
                            }
                            (Piece::Arc(arc), Piece::Line(p, q)) => {
                                let c = cos(vec2(*q - *p), radial(arc, arc.end));
                                assert!(c.abs() < 1e-5, "tangent out of arc: {c}");
                            }
                            (Piece::Arc(x), Piece::Arc(y)) => {
                                let c = cos(radial(x, x.end), radial(y, y.start));
                                assert!((c.abs() - 1.0).abs() < 1e-5, "arc-arc tangent: {c}");
                            }
                        }
                    }
                    for p in &ps {
                        match p {
                            Piece::Line(p, q) => assert!((*q - *p).is_octilinear(), "{p:?}"),
                            Piece::Arc(arc) => {
                                let sweep = arc.sweep().expect("sweep").abs();
                                assert!((sweep - FRAC_PI_2).abs() < 1e-5, "{sweep}");
                                let rad = arc.circle().expect("circle").r;
                                assert!(rad + 2.0 >= RMIN as f64, "radius {rad}");
                            }
                        }
                    }
                    let gain = pieces_length(&ps) - a.dist(b);
                    let tol = 5.0 * f64::from(spec.bumps);
                    assert!((gain - need).abs() <= tol, "real gain {gain} vs {need}");
                    assert!((gain - unit * spec.extra_length()).abs() <= tol, "{gain}");
                    assert!(unit * spec.spacing as f64 + 2.0 >= S as f64, "spacing never tighter");
                }
            }
        }
    }

    #[test]
    fn parallel_legs_keep_the_requested_spacing() {
        for (a, b) in runs() {
            let spec = MeanderSpec::plan_run(a, b, 6_500_000.0, S, 3 * S, S, RMIN, 1).expect("ok");
            let ps = spec.pieces(a, b).expect("pieces");
            let arcs: Vec<Arc> = ps
                .iter()
                .filter_map(|p| if let Piece::Arc(x) = p { Some(*x) } else { None })
                .collect();
            assert_eq!(arcs.len(), 4 * spec.bumps as usize, "four fillets per bump");
            let (len, unit) = run_frame(a, b).expect("octilinear");
            let dir = vec2(b - a);
            let dist = |p: Point, q: Point| {
                let d = vec2(q - p);
                (d.0 * dir.0 + d.1 * dir.1) / (dir.0.hypot(dir.1))
            };
            let want = unit * spec.spacing as f64 - 2.0;
            assert!(len > 0);
            for i in 0..spec.bumps as usize {
                // rising leg → falling leg of one bump, then falling leg → next rising leg
                assert!(dist(arcs[4 * i].end, arcs[4 * i + 2].end).abs() >= want);
                if i + 1 < spec.bumps as usize {
                    assert!(dist(arcs[4 * i + 3].start, arcs[4 * i + 4].end).abs() >= want);
                }
            }
        }
    }

    #[test]
    fn bumps_rise_on_the_requested_side() {
        let (a, b) = (Point::new(0, 0), Point::new(20_000_000, 0));
        let up = MeanderSpec::plan(20_000_000, 4e6, S, 3 * S, S, RMIN, 1).expect("fits");
        let down = MeanderSpec { side: -1, ..up };
        let ys = |s: MeanderSpec| {
            s.pieces(a, b)
                .expect("ok")
                .iter()
                .flat_map(|p| [start_of(p).y, end_of(p).y])
                .collect::<Vec<_>>()
        };
        assert!(ys(up).iter().all(|&y| y <= 0), "left of +x in y-down is −y");
        assert!(ys(down).iter().all(|&y| y >= 0));
        assert_eq!(up.pieces(a, Point::new(5, 7)), None, "non-octilinear runs are not tuned");
        assert_eq!(run_frame(a, a), None);
        assert_eq!(up.pieces(a, Point::new(1_000_000, 0)), None, "too short");
    }
}
