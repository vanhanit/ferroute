// SPDX-License-Identifier: AGPL-3.0-only
//! Junction geometry of a net's lattice route: which directions copper leaves each node,
//! which new departures would create an acid trap, and whether a route contains one.
//!
//! An *acid trap* is an acute (< 90°) angle between two copper segments of the same net.
//! On the octilinear lattice two directions leaving a node form an acute angle exactly
//! when they differ by less than two 45° steps (`Dir8::turn < 2`).

use crate::grid::{Grid, NodeIdx};
use ferroute_geom::Dir8;
use std::collections::BTreeMap;

/// Bit set of [`Dir8`] directions (bit = `Dir8::index`).
pub type DirMask = u8;

#[must_use]
pub const fn bit(d: Dir8) -> DirMask {
    1 << d.index()
}

/// Planar direction of a lattice step `a → b` (`None` for via steps or non-unit steps).
#[must_use]
pub fn step_dir(grid: &Grid, a: NodeIdx, b: NodeIdx) -> Option<Dir8> {
    let (ai, aj, al) = grid.coords(a);
    let (bi, bj, bl) = grid.coords(b);
    if al != bl {
        return None;
    }
    Dir8::from_delta(i64::from(bi) - i64::from(ai), i64::from(bj) - i64::from(aj))
}

/// Directions in which copper leaves each node: lattice path steps (both ends of every
/// step) plus `extra` (e.g. the heading into a pad stub at a terminal node).
#[must_use]
pub fn incident(
    grid: &Grid,
    paths: &[Vec<NodeIdx>],
    extra: &[(NodeIdx, Dir8)],
) -> BTreeMap<NodeIdx, DirMask> {
    let mut m: BTreeMap<NodeIdx, DirMask> = BTreeMap::new();
    for p in paths {
        for w in p.windows(2) {
            if let Some(d) = step_dir(grid, w[0], w[1]) {
                *m.entry(w[0]).or_default() |= bit(d);
                *m.entry(w[1]).or_default() |= bit(d.opposite());
            }
        }
    }
    for &(n, d) in extra {
        *m.entry(n).or_default() |= bit(d);
    }
    m
}

/// Departures from a node with copper leaving in `mask` that would form an acute angle
/// (or overlap) with existing copper.
#[must_use]
pub fn forbidden_departures(mask: DirMask) -> DirMask {
    let mut f = 0;
    for d in Dir8::ALL {
        if Dir8::ALL.iter().any(|&e| mask & bit(e) != 0 && d.turn(e) < 2) {
            f |= bit(d);
        }
    }
    f
}

/// `true` when some node has two copper directions closer than 90°.
#[must_use]
pub fn has_acute(incident: &BTreeMap<NodeIdx, DirMask>) -> bool {
    incident.values().any(|&m| {
        Dir8::ALL
            .iter()
            .any(|&d| m & bit(d) != 0 && forbidden_departures(m & !bit(d)) & bit(d) != 0)
    })
}

/// `true` when the copper of `paths` (plus `extra` stub headings) has an acid trap or
/// overlapping segments: some node with two copper directions closer than 90°, or the
/// same direction used twice (two paths running on top of each other).
#[must_use]
pub fn has_acute_or_overlap(
    grid: &Grid,
    paths: &[Vec<NodeIdx>],
    extra: &[(NodeIdx, Dir8)],
) -> bool {
    let mut counts: BTreeMap<NodeIdx, [u8; 8]> = BTreeMap::new();
    let mut bump = |n: NodeIdx, d: Dir8| {
        let c = &mut counts.entry(n).or_default()[d.index()];
        *c = c.saturating_add(1);
    };
    for p in paths {
        for w in p.windows(2) {
            if let Some(d) = step_dir(grid, w[0], w[1]) {
                bump(w[0], d);
                bump(w[1], d.opposite());
            }
        }
    }
    for &(n, d) in extra {
        bump(n, d);
    }
    counts.values().any(|c| {
        c.iter().any(|&k| k > 1)
            || Dir8::ALL.iter().any(|&a| {
                c[a.index()] > 0
                    && Dir8::ALL.iter().any(|&b| b != a && c[b.index()] > 0 && a.turn(b) < 2)
            })
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use ferroute_board::LayerId;
    use ferroute_geom::{Point, Rect};

    fn grid() -> Grid {
        Grid::covering(
            Rect::new(Point::new(0, 0), Point::new(9_000, 9_000)),
            1_000,
            vec![LayerId(0), LayerId(1)],
        )
    }

    #[test]
    fn straight_trunk_allows_only_perpendicular_branches() {
        let f = forbidden_departures(bit(Dir8::E) | bit(Dir8::W));
        let allowed: Vec<Dir8> = Dir8::ALL.into_iter().filter(|&d| f & bit(d) == 0).collect();
        assert_eq!(allowed, vec![Dir8::N, Dir8::S]);
    }

    #[test]
    fn no_copper_forbids_nothing() {
        assert_eq!(forbidden_departures(0), 0);
    }

    #[test]
    fn incident_records_both_ends_and_extras_and_skips_vias() {
        let g = grid();
        let path = vec![g.idx(1, 1, 0), g.idx(2, 1, 0), g.idx(2, 1, 1), g.idx(3, 2, 1)];
        let m = incident(&g, &[path], &[(g.idx(1, 1, 0), Dir8::W)]);
        assert_eq!(m[&g.idx(1, 1, 0)], bit(Dir8::E) | bit(Dir8::W));
        assert_eq!(m[&g.idx(2, 1, 0)], bit(Dir8::W));
        assert_eq!(m[&g.idx(2, 1, 1)], bit(Dir8::SE));
        assert_eq!(m[&g.idx(3, 2, 1)], bit(Dir8::NW));
    }

    #[test]
    fn detects_forty_five_degree_junction_but_not_tee() {
        let g = grid();
        let trunk = vec![g.idx(1, 4, 0), g.idx(2, 4, 0), g.idx(3, 4, 0)];
        let tee = vec![g.idx(2, 4, 0), g.idx(2, 3, 0)];
        let skew = vec![g.idx(2, 4, 0), g.idx(3, 3, 0)];
        assert!(!has_acute(&incident(&g, &[trunk.clone(), tee], &[])));
        assert!(has_acute(&incident(&g, &[trunk, skew], &[])));
    }

    #[test]
    fn overlapping_paths_are_detected() {
        let g = grid();
        let a = vec![g.idx(1, 4, 0), g.idx(2, 4, 0), g.idx(3, 4, 0)];
        let b = vec![g.idx(2, 4, 0), g.idx(3, 4, 0), g.idx(4, 3, 0)];
        assert!(has_acute_or_overlap(&g, &[a.clone(), b], &[]));
        let tee = vec![g.idx(2, 4, 0), g.idx(2, 3, 0)];
        assert!(!has_acute_or_overlap(&g, &[a, tee], &[]));
    }

    #[test]
    fn obtuse_bend_is_not_acute() {
        let g = grid();
        let bend = vec![g.idx(1, 4, 0), g.idx(2, 4, 0), g.idx(3, 3, 0)];
        assert!(!has_acute(&incident(&g, &[bend], &[])));
    }
}
