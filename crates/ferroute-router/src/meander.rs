// SPDX-License-Identifier: AGPL-3.0-only
//! Accordion meanders for length tuning: uniform bumps with 45° mitred corners on an
//! axis-aligned straight run. Pure integer geometry; the caller verifies the result
//! with the exact DRC oracle ([`crate::tune`]).
//!
//! One bump, in local coordinates (along `x`, normal `y`), spacing `s`, mitre `c`,
//! amplitude `h` and top/gap run `t = s − 2c`:
//!
//! ```text
//!          (2c,h)───(2c+t,h)
//!          ╱               ╲
//!    (c,c)│                 │(3c+t,c)        next bump
//!   ──────╯(0,0)            ╰(4c+t,0)──────── ...
//! ```
//!
//! The rising and falling legs of a bump, and of neighbouring bumps, are exactly `s`
//! apart (centre to centre). Each bump adds `2h − 4(2 − √2)c` of length.

use ferroute_geom::{Coord, Point};

/// Shape of a meander on one straight run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MeanderSpec {
    /// Centre-to-centre spacing of parallel legs.
    pub spacing: Coord,
    /// Bump height above the original run.
    pub amplitude: Coord,
    /// 45° mitre length along each axis (0 = square corners).
    pub mitre: Coord,
    /// Number of bumps.
    pub bumps: u32,
    /// Side of the run the bumps rise on (+1 = left of `a → b`, −1 = right).
    pub side: i8,
}

const MITRE_LOSS: f64 = 4.0 * (2.0 - std::f64::consts::SQRT_2);

impl MeanderSpec {
    /// Length along the run occupied by the bumps (`2s` per bump, minus the trailing gap).
    #[must_use]
    pub fn extent(&self) -> Coord {
        let t = self.spacing - 2 * self.mitre;
        Coord::from(self.bumps) * 2 * self.spacing - t
    }

    /// Extra length the meander adds over the straight run.
    #[must_use]
    pub fn extra_length(&self) -> f64 {
        #[allow(clippy::cast_precision_loss)]
        let (h, c) = (self.amplitude as f64, self.mitre as f64);
        f64::from(self.bumps) * (2.0 * h - MITRE_LOSS * c)
    }

    /// Plans the meander for a run of length `run` that must gain `need` nm, with bumps no
    /// higher than `max_amplitude` and at least `margin` of straight run at each end.
    /// Returns the largest gain ≤ `need` that fits (fewer, full-height bumps when the run is
    /// too short); `None` if not even one bump fits or `need` is negligible.
    #[must_use]
    pub fn plan(
        run: Coord,
        need: f64,
        spacing: Coord,
        max_amplitude: Coord,
        margin: Coord,
        side: i8,
    ) -> Option<Self> {
        if spacing <= 0 || need < 1_000.0 {
            return None;
        }
        let mitre = spacing / 4;
        let t = spacing - 2 * mitre;
        let room = run - 2 * margin;
        // bumps · 2s − t ≤ room
        let capacity = u32::try_from((room + t).div_euclid(2 * spacing)).ok()?.min(1 << 16);
        if capacity == 0 {
            return None;
        }
        #[allow(clippy::cast_precision_loss)]
        let per_full = 2.0 * max_amplitude as f64 - MITRE_LOSS * mitre as f64;
        if per_full <= 0.0 {
            return None;
        }
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let wanted = (need / per_full).ceil().max(1.0) as u32;
        let bumps = wanted.min(capacity);
        let per_bump = (need / f64::from(bumps)).min(per_full);
        // Square corners for gains too small for a mitred bump (h ≥ 2c).
        #[allow(clippy::cast_precision_loss)]
        let min_mitred = 4.0 * mitre as f64 - MITRE_LOSS * mitre as f64;
        let (mitre, loss) = if per_bump < min_mitred {
            (0, 0.0)
        } else {
            #[allow(clippy::cast_precision_loss)]
            (mitre, MITRE_LOSS * mitre as f64)
        };
        #[allow(clippy::cast_possible_truncation)]
        let amplitude = ((per_bump + loss) / 2.0).floor() as Coord;
        (amplitude > 0).then_some(Self { spacing, amplitude, mitre, bumps, side })
    }

    /// [`MeanderSpec::plan`] for the run `a → b` in real nanometres: on a 45° run the
    /// local unit is √2 long, so spacing and margin are rounded *up* (never tighter) and the
    /// amplitude *down* (never taller) when converted.
    #[must_use]
    pub fn plan_run(
        a: Point,
        b: Point,
        need: f64,
        spacing: Coord,
        max_amplitude: Coord,
        margin: Coord,
        side: i8,
    ) -> Option<Self> {
        let (len, unit) = run_frame(a, b)?;
        #[allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]
        let (up, down) = (
            |v: Coord| (v as f64 / unit).ceil() as Coord,
            |v: Coord| (v as f64 / unit).floor() as Coord,
        );
        Self::plan(len, need / unit, up(spacing), down(max_amplitude), up(margin), side)
    }

    /// Polyline from `a` to `b` (an axis-aligned or 45° run) with the meander centred on the
    /// run, in the run's local units (see [`run_frame`]). `None` if `a → b` is not
    /// octilinear or too short for [`MeanderSpec::extent`].
    #[must_use]
    pub fn polyline(&self, a: Point, b: Point) -> Option<Vec<Point>> {
        let d = b - a;
        let (len, _) = run_frame(a, b)?;
        let extent = self.extent();
        if extent > len || self.bumps == 0 {
            return None;
        }
        let u = Point::new(d.x.signum(), d.y.signum());
        // Left normal of u in y-down screen coordinates is (u.y, −u.x); `side` flips it.
        let n = Point::new(u.y, -u.x) * Coord::from(self.side);
        let at = |x: Coord, y: Coord| a + u * x + n * y;
        let (s, c, h) = (self.spacing, self.mitre, self.amplitude);
        let t = s - 2 * c;
        let mut x = (len - extent) / 2;
        let mut pts = vec![a];
        for _ in 0..self.bumps {
            let local = if c == 0 {
                vec![(x, 0), (x, h), (x + t, h), (x + t, 0)]
            } else {
                vec![
                    (x, 0),
                    (x + c, c),
                    (x + c, h - c),
                    (x + 2 * c, h),
                    (x + 2 * c + t, h),
                    (x + 3 * c + t, h - c),
                    (x + 3 * c + t, c),
                    (x + 4 * c + t, 0),
                ]
            };
            pts.extend(local.into_iter().map(|(px, py)| at(px, py)));
            x += 2 * s;
        }
        pts.push(b);
        pts.dedup();
        Some(pts)
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

/// Euclidean length of a polyline.
#[must_use]
pub fn polyline_length(pts: &[Point]) -> f64 {
    pts.windows(2).map(|w| w[0].dist(w[1])).sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    const S: Coord = 750_000;

    fn turn_ok(a: Point, b: Point, c: Point) -> bool {
        // Interior angle at b ≥ 90°  ⇔  (a−b)·(c−b) ≤ 0.
        let (u, v) = (a - b, c - b);
        i128::from(u.x) * i128::from(v.x) + i128::from(u.y) * i128::from(v.y) <= 0
    }

    #[test]
    fn plan_meets_the_gain_with_uniform_bumps() {
        let spec = MeanderSpec::plan(20_000_000, 9_000_000.0, S, 3 * S, S, 1).expect("fits");
        assert_eq!(spec.bumps, 3, "a full 2.25 mm bump gains ~4.06 mm: {spec:?}");
        assert!(spec.amplitude <= 3 * S);
        assert!((spec.extra_length() - 9_000_000.0).abs() < 10.0, "{}", spec.extra_length());
        assert!(spec.extent() + 2 * S <= 20_000_000);
    }

    #[test]
    fn short_runs_give_partial_gain_and_tiny_gains_use_square_corners() {
        let spec = MeanderSpec::plan(4_000_000, 50_000_000.0, S, 3 * S, S, 1).expect("one");
        assert_eq!(spec.bumps, 1, "only one bump fits in 4 mm");
        assert_eq!(spec.amplitude, 3 * S, "full height when partial");
        let tiny = MeanderSpec::plan(20_000_000, 100_000.0, S, 3 * S, S, -1).expect("tiny");
        assert_eq!((tiny.bumps, tiny.mitre), (1, 0));
        assert!((tiny.extra_length() - 100_000.0).abs() <= 2.0);
        assert_eq!(MeanderSpec::plan(1_000_000, 1e6, S, 3 * S, S, 1), None, "no room");
        assert_eq!(MeanderSpec::plan(20_000_000, 10.0, S, 3 * S, S, 1), None, "negligible");
    }

    #[test]
    fn polyline_is_octilinear_obtuse_and_adds_exactly_the_planned_length() {
        for (a, b) in [
            (Point::new(0, 0), Point::new(20_000_000, 0)),
            (Point::new(5_000_000, 0), Point::new(5_000_000, -18_000_000)),
            (Point::new(0, 7_000_000), Point::new(-21_000_000, 7_000_000)),
            (Point::new(0, 0), Point::new(15_000_000, 15_000_000)),
            (Point::new(3_000_000, 2_000_000), Point::new(-12_000_000, 17_000_000)),
        ] {
            for side in [1, -1] {
                let spec =
                    MeanderSpec::plan_run(a, b, 6_500_000.0, S, 3 * S, S, side).expect("fits");
                let (_, unit) = run_frame(a, b).expect("octilinear");
                let pts = spec.polyline(a, b).expect("axis-aligned");
                assert_eq!((pts[0], *pts.last().expect("end")), (a, b));
                for w in pts.windows(2) {
                    assert!((w[1] - w[0]).is_octilinear(), "{w:?}");
                }
                for w in pts.windows(3) {
                    assert!(turn_ok(w[0], w[1], w[2]), "acute turn at {:?}", w[1]);
                }
                let gain = polyline_length(&pts) - a.dist(b);
                assert!((gain - unit * spec.extra_length()).abs() < 1.0, "{gain}");
                assert!((gain - 6_500_000.0).abs() < 10.0, "real gain {gain}");
                // Parallel legs are exactly one spacing apart: every rising leg's base is
                // `s` from the next falling leg's base along the run.
                let bases: Vec<Point> = pts
                    .iter()
                    .copied()
                    .filter(|p| {
                        let off = (*p - a).x * (b - a).y - (*p - a).y * (b - a).x;
                        off == 0 && *p != a && *p != b
                    })
                    .collect();
                for w in bases.windows(2) {
                    let gap = w[0].dist(w[1]);
                    assert!(gap + 2.0 >= unit * (spec.spacing - 2 * spec.mitre) as f64, "{gap}");
                }
                assert!(unit * spec.spacing as f64 >= S as f64, "spacing never tighter");
            }
        }
    }

    #[test]
    fn bumps_rise_on_the_requested_side() {
        let (a, b) = (Point::new(0, 0), Point::new(20_000_000, 0));
        let up = MeanderSpec::plan(20_000_000, 4e6, S, 3 * S, S, 1).expect("fits");
        let down = MeanderSpec { side: -1, ..up };
        let ys =
            |s: MeanderSpec| s.polyline(a, b).expect("ok").iter().map(|p| p.y).collect::<Vec<_>>();
        assert!(ys(up).iter().all(|&y| y <= 0), "left of +x in y-down is −y");
        assert!(ys(down).iter().all(|&y| y >= 0));
        assert_eq!(up.polyline(a, Point::new(5, 7)), None, "non-octilinear runs are not tuned");
        assert_eq!(run_frame(a, a), None);
    }
}
