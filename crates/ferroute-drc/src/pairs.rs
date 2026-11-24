// SPDX-License-Identifier: AGPL-3.0-only
//! Differential-pair coupling metrics: how tightly and how long two nets run side by side.
//!
//! Two track segments of the pair are *coupled* where they are parallel, overlap along their
//! common direction and are within [`COUPLING_REACH`] gaps of each other (edge to edge). The
//! rest of a trace is *uncoupled* (pad break-outs, fan-in jogs, meanders, detours).

use ferroute_board::{Board, DiffPair, NetId, Track};
use ferroute_geom::Coord;

/// Segments closer than this many target gaps (edge to edge) count as coupled.
pub const COUPLING_REACH: i64 = 3;

/// Coupling of one pair.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PairMetrics {
    /// Smallest / largest edge-to-edge gap over the coupled sections (nm, rounded).
    pub min_gap: Option<Coord>,
    pub max_gap: Option<Coord>,
    /// Total length run side by side (per trace, the longer of the two sums).
    pub coupled: Coord,
    /// Uncoupled copper length of the P and N trace (straight tracks only).
    pub uncoupled_p: Coord,
    pub uncoupled_n: Coord,
}

impl PairMetrics {
    /// Longer of the two uncoupled lengths.
    #[must_use]
    pub fn uncoupled(&self) -> Coord {
        self.uncoupled_p.max(self.uncoupled_n)
    }
}

struct Side {
    length: f64,
    coupled: f64,
}

/// Running `(min, max)` of the coupled gaps (nm).
type GapRange = Option<(f64, f64)>;

#[allow(clippy::cast_precision_loss)]
fn coupling(a: &[&Track], b: &[&Track], reach_gap: Coord, gaps: &mut GapRange) -> Side {
    let mut side = Side { length: 0.0, coupled: 0.0 };
    for s in a {
        let d = s.end - s.start;
        let len = s.start.dist(s.end);
        side.length += len;
        if len == 0.0 {
            continue;
        }
        let mut covered = 0.0f64;
        for t in b.iter().filter(|t| t.layer == s.layer) {
            let e = t.end - t.start;
            if d.cross(e) != 0 || e.norm2() == 0 {
                continue;
            }
            // Perpendicular centre distance and overlap along `s`.
            let off = d.cross(t.start - s.start).unsigned_abs() as f64 / len;
            let gap = off - (s.width + t.width) as f64 / 2.0;
            if gap > reach_gap as f64 {
                continue;
            }
            let n2 = d.norm2() as f64;
            let p0 = d.dot(t.start - s.start) as f64 / n2;
            let p1 = d.dot(t.end - s.start) as f64 / n2;
            let (lo, hi) = (p0.min(p1).max(0.0), p0.max(p1).min(1.0));
            if hi <= lo {
                continue;
            }
            covered += (hi - lo) * len;
            let g = gaps.get_or_insert((gap, gap));
            g.0 = g.0.min(gap);
            g.1 = g.1.max(gap);
        }
        side.coupled += covered.min(len);
    }
    side
}

/// Coupling metrics of `pair` on `board` (all tracks of both nets, any origin). `None` when
/// either net has no copper track (nothing to measure).
#[must_use]
pub fn pair_metrics(board: &Board, pair: &DiffPair) -> Option<PairMetrics> {
    let of = |n: NetId| board.tracks.iter().filter(|t| t.net == n).collect::<Vec<_>>();
    let (p, n) = (of(pair.p), of(pair.n));
    if p.is_empty() || n.is_empty() {
        return None;
    }
    let reach = pair.gap.saturating_mul(COUPLING_REACH);
    let mut gaps = None;
    let sp = coupling(&p, &n, reach, &mut gaps);
    let sn = coupling(&n, &p, reach, &mut gaps);
    #[allow(clippy::cast_possible_truncation)]
    let r = |v: f64| v.round() as Coord;
    Some(PairMetrics {
        min_gap: gaps.map(|g| r(g.0)),
        max_gap: gaps.map(|g| r(g.1)),
        coupled: r(sp.coupled.max(sn.coupled)),
        uncoupled_p: r((sp.length - sp.coupled).max(0.0)),
        uncoupled_n: r((sn.length - sn.coupled).max(0.0)),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use ferroute_board::builder::{BoardBuilder, mm};
    use ferroute_board::{LayerId, Origin};
    use ferroute_geom::Point;

    fn track(net: NetId, a: (i64, i64), b: (i64, i64)) -> Track {
        Track {
            start: Point::new(a.0, a.1),
            end: Point::new(b.0, b.1),
            width: 250_000,
            layer: LayerId(0),
            net,
            locked: false,
            origin: Origin::Routed,
        }
    }

    fn board_with(tracks: Vec<Track>) -> (Board, DiffPair) {
        let mut b = BoardBuilder::new(mm(40), mm(40), 2);
        let (p, n) = (b.net("D+"), b.net("D-"));
        let mut board = b.build();
        board.tracks = tracks;
        let pair =
            DiffPair { p, n, gap: 200_000, width: None, max_uncoupled: None, skew_max: None };
        (board, pair)
    }

    #[test]
    fn parallel_straight_pair_measures_the_gap_and_full_coupling() {
        let (p, n) = (NetId(1), NetId(2));
        let (board, pair) = board_with(vec![
            track(p, (0, 0), (10_000_000, 0)),
            track(n, (0, 450_000), (10_000_000, 450_000)),
        ]);
        let m = pair_metrics(&board, &pair).expect("m");
        assert_eq!((m.min_gap, m.max_gap), (Some(200_000), Some(200_000)));
        assert_eq!(m.coupled, 10_000_000);
        assert_eq!(m.uncoupled(), 0);
    }

    #[test]
    fn diagonal_runs_and_partial_overlap_are_measured() {
        let (p, n) = (NetId(1), NetId(2));
        // 45° pair whose perpendicular centre distance is 450 µm; N is half as long.
        let a = 318_198; // 450 000 / sqrt 2
        let (board, pair) = board_with(vec![
            track(p, (0, 0), (4_000_000, 4_000_000)),
            track(n, (-a, a), (-a + 2_000_000, a + 2_000_000)),
        ]);
        let m = pair_metrics(&board, &pair).expect("m");
        let g = m.min_gap.expect("gap");
        assert!((g - 200_000).abs() < 10, "{g}");
        // P is coupled along half its length; N entirely.
        assert!((m.uncoupled_p - 2_828_427).abs() < 100, "{m:?}");
        assert!(m.uncoupled_n < 100, "{m:?}");
    }

    #[test]
    fn far_apart_and_unparallel_copper_is_uncoupled() {
        let (p, n) = (NetId(1), NetId(2));
        let (board, pair) = board_with(vec![
            track(p, (0, 0), (5_000_000, 0)),
            track(n, (0, 3_000_000), (5_000_000, 3_000_000)),
            track(n, (5_000_000, 3_000_000), (6_000_000, 4_000_000)),
        ]);
        let m = pair_metrics(&board, &pair).expect("m");
        assert_eq!((m.min_gap, m.coupled), (None, 0));
        assert_eq!(m.uncoupled_p, 5_000_000);
        let (empty, pair) = board_with(vec![]);
        assert_eq!(pair_metrics(&empty, &pair), None);
    }
}
