// SPDX-License-Identifier: AGPL-3.0-only
//! Detour metric: how much longer a routed net is than its ideal octilinear tree.
//!
//! PathFinder only reacts to *conflicts*, so a net that was routed after a competitor
//! took its corridor can end up with a long detour that negotiation never repairs. This
//! module measures that excess (pure, deterministic) so [`crate::route`] can promote the
//! worst offenders in the net order and re-route.

use ferroute_board::{Board, BoardEdit, NetId};
use ferroute_geom::Point;

/// Octilinear distance `max(|dx|,|dy|) + (√2−1)·min(|dx|,|dy|)` in nm.
#[must_use]
#[allow(clippy::cast_precision_loss)]
pub fn octilinear_dist(a: Point, b: Point) -> f64 {
    let dx = (a.x - b.x).abs() as f64;
    let dy = (a.y - b.y).abs() as f64;
    dx.max(dy) + (std::f64::consts::SQRT_2 - 1.0) * dx.min(dy)
}

/// Length (nm) of the minimum spanning tree over `pts` under [`octilinear_dist`].
///
/// Deterministic Prim: starts at index 0; ties pick the lowest index.
#[must_use]
pub fn mst_length(pts: &[Point]) -> f64 {
    let n = pts.len();
    if n < 2 {
        return 0.0;
    }
    let mut in_tree = vec![false; n];
    let mut best = vec![f64::INFINITY; n];
    best[0] = 0.0;
    let mut total = 0.0;
    for _ in 0..n {
        let mut pick = usize::MAX;
        for i in 0..n {
            if !in_tree[i] && (pick == usize::MAX || best[i] < best[pick]) {
                pick = i;
            }
        }
        in_tree[pick] = true;
        total += best[pick];
        for i in 0..n {
            if !in_tree[i] {
                let d = octilinear_dist(pts[pick], pts[i]);
                if d < best[i] {
                    best[i] = d;
                }
            }
        }
    }
    total
}

/// Detour of one routed net.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct NetDetour {
    pub net: NetId,
    /// MST over the net's pad centres (nm).
    pub ideal_nm: f64,
    /// Sum of the net's new track and arc lengths (nm).
    pub routed_nm: f64,
}

impl NetDetour {
    /// `routed − ideal`, clamped at 0 (existing copper can make a net shorter than ideal).
    #[must_use]
    pub fn excess_nm(&self) -> f64 {
        (self.routed_nm - self.ideal_nm).max(0.0)
    }
}

/// Detour of every net that received new copper in `edit`, in ascending net id.
#[must_use]
pub fn net_detours(board: &Board, edit: &BoardEdit) -> Vec<NetDetour> {
    let mut routed = vec![0.0f64; board.nets.len()];
    let mut has = vec![false; board.nets.len()];
    for t in &edit.new_tracks {
        routed[t.net.idx()] += t.start.dist(t.end);
        has[t.net.idx()] = true;
    }
    for a in &edit.new_arcs {
        routed[a.net.idx()] += a.length();
        has[a.net.idx()] = true;
    }
    let mut pads: Vec<Vec<Point>> = vec![Vec::new(); board.nets.len()];
    for id in board.pad_ids() {
        let net = board.pad(id).net;
        if net != NetId::NONE && has[net.idx()] {
            pads[net.idx()].push(board.pad_position(id));
        }
    }
    board
        .net_ids()
        .filter(|n| has[n.idx()])
        .map(|net| NetDetour {
            net,
            ideal_nm: mst_length(&pads[net.idx()]),
            routed_nm: routed[net.idx()],
        })
        .collect()
}

/// Lexicographic quality (lower is better): failed nets, Σ excess in µm, vias.
///
/// Excess precedes vias so that a re-order can never trade wirelength for vias (the
/// benchmark baseline gates total wirelength).
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Quality {
    pub failed: usize,
    pub excess_um: u64,
    pub vias: usize,
    /// Total routed track length (µm).
    pub wire_um: u64,
}

impl Quality {
    /// Pareto improvement: no more failed nets, vias or wire, and strictly less detour.
    /// A re-ordered routing is only worth keeping if nothing else gets worse.
    #[must_use]
    pub const fn improves_on(&self, best: &Self) -> bool {
        self.failed <= best.failed
            && self.vias <= best.vias
            && self.wire_um <= best.wire_um
            && self.excess_um < best.excess_um
    }
}

/// Quality of a routing result; failed nets do not contribute excess.
#[must_use]
pub fn quality(board: &Board, out: &crate::RouteOutcome) -> Quality {
    Quality {
        failed: out.failed.len(),
        excess_um: excess_um(&net_detours(board, &out.edit), &out.failed),
        vias: out.edit.new_vias.len(),
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        wire_um: (out.edit.new_tracks.iter().map(|t| t.start.dist(t.end)).sum::<f64>() / 1e3)
            .round() as u64,
    }
}

/// Σ excess (µm, rounded) over nets not in `failed`.
#[must_use]
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
pub fn excess_um(d: &[NetDetour], failed: &[NetId]) -> u64 {
    let nm: f64 = d.iter().filter(|x| !failed.contains(&x.net)).map(NetDetour::excess_nm).sum();
    (nm / 1000.0).round() as u64
}

/// Relative excess (fraction of the ideal length) above which a net is promoted.
pub const REL_THRESHOLD: f64 = 0.02;

/// Nets worth re-ordering: excess > max(0.5 mm, 2 % of ideal), worst first
/// (ties by ascending net id).
#[must_use]
pub fn worst_nets(d: &[NetDetour], failed: &[NetId]) -> Vec<NetId> {
    let mut v: Vec<&NetDetour> = d
        .iter()
        .filter(|x| {
            !failed.contains(&x.net) && x.excess_nm() > (REL_THRESHOLD * x.ideal_nm).max(500_000.0)
        })
        .collect();
    v.sort_by(|a, b| b.excess_nm().total_cmp(&a.excess_nm()).then(a.net.cmp(&b.net)));
    v.into_iter().map(|x| x.net).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: i64, y: i64) -> Point {
        Point::new(x * 1_000_000, y * 1_000_000)
    }

    #[test]
    fn octilinear_distance_matches_straight_and_diagonal() {
        assert_eq!(octilinear_dist(p(0, 0), p(10, 0)), 10e6);
        let d = octilinear_dist(p(0, 0), p(3, 3));
        assert!((d - 3e6 * std::f64::consts::SQRT_2).abs() < 1e-6);
    }

    #[test]
    fn straight_two_pin_net_has_zero_excess() {
        let ideal = mst_length(&[p(0, 0), p(10, 0)]);
        let d = NetDetour { net: NetId(1), ideal_nm: ideal, routed_nm: 10e6 };
        assert_eq!(d.excess_nm(), 0.0);
    }

    #[test]
    fn l_shaped_detour_has_expected_excess() {
        let ideal = mst_length(&[p(0, 0), p(10, 10)]);
        let d = NetDetour { net: NetId(1), ideal_nm: ideal, routed_nm: 20e6 };
        assert!((d.excess_nm() - (20e6 - 10e6 * std::f64::consts::SQRT_2)).abs() < 1e-3);
    }

    #[test]
    fn mst_over_three_pins_is_deterministic_and_minimal() {
        let pts = [p(0, 0), p(10, 0), p(5, 0)];
        let a = mst_length(&pts);
        assert_eq!(a, 10e6);
        assert_eq!(a, mst_length(&pts));
        assert_eq!(a, mst_length(&[pts[2], pts[0], pts[1]]));
        assert_eq!(mst_length(&pts[..1]), 0.0);
    }

    #[test]
    fn worst_nets_threshold_and_order() {
        let mk = |n, ideal, routed| NetDetour { net: NetId(n), ideal_nm: ideal, routed_nm: routed };
        let d = [mk(1, 10e6, 10.4e6), mk(2, 10e6, 13e6), mk(3, 10e6, 14e6), mk(4, 1e6, 3e6)];
        assert_eq!(worst_nets(&d, &[]), vec![NetId(3), NetId(2), NetId(4)]);
        assert_eq!(worst_nets(&d, &[NetId(3)]), vec![NetId(2), NetId(4)]);
    }
}
