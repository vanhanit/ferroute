// SPDX-License-Identifier: AGPL-3.0-only
//! Pin access: which lattice nodes connect a pad to the routing graph.
//!
//! * **Interior terminals** — lattice nodes lying inside the pad copper on one of its
//!   layers. A track ending there overlaps the pad, so it is connected.
//! * **Stub terminals** — for small or off-grid pads with no interior node, a short
//!   straight stub from the pad centre to a nearby legal node. The stub is checked
//!   exactly against fixed obstacles, and it *reserves* every lattice node close enough
//!   that other nets' lattice copper could violate clearance with it (see [`Stub`]).

use crate::grid::{Grid, NodeIdx};
use crate::legality::{ClassLegality, Obstacles, allows};
use crate::plan::{ClassPlan, Plan};
use ferroute_board::{Board, LayerId, NetId, PadId};
use ferroute_geom::{Coord, Hull, Point, Segment, point_segment_dist_lt};

/// A pad-centre → lattice-node stub.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Stub {
    /// Octilinear polyline from the pad centre (first) to the lattice node (last); every
    /// segment is 0°, 45° or 90°. Intermediate bend points may lie off the lattice.
    pub path: Vec<Point>,
    pub layer: LayerId,
    /// Lattice nodes reserved by the stub (same grid layer).
    pub reserved: Vec<NodeIdx>,
}

/// A lattice entry point of a pad.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Terminal {
    pub node: NodeIdx,
    pub stub: Option<u32>,
}

/// Maximum number of stub alternatives kept per pad.
const MAX_STUBS: usize = 4;

/// Computes the terminals of `pad` for `net` routed with class plan `cp`.
#[allow(clippy::too_many_arguments)]
pub fn pad_terminals(
    board: &Board,
    grid: &Grid,
    obs: &Obstacles,
    leg: &ClassLegality,
    cp: &ClassPlan,
    plan: &Plan,
    net: NetId,
    pad: PadId,
    stubs: &mut Vec<Stub>,
) -> Vec<Terminal> {
    let p = board.pad(pad);
    let hull = board.pad_hull(pad);
    let centre = board.pad_position(pad);
    let mut out = Vec::new();
    let layers: Vec<u32> =
        (0..grid.nl()).filter(|&g| p.layers.contains(grid.layers[g as usize])).collect();
    let r = cp.geom.track_radius;
    // 1. Pad centre exactly on a lattice node: terminate there, no stub.
    let d = centre - grid.origin;
    if d.x.rem_euclid(grid.pitch) == 0 && d.y.rem_euclid(grid.pitch) == 0 {
        let (ci, cj) = grid.cell_floor(centre);
        if ci >= 0 && cj >= 0 && ci < i64::from(grid.nx) && cj < i64::from(grid.ny) {
            for &l in &layers {
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                let n = grid.idx(ci as u32, cj as u32, l);
                if allows(leg.node[n as usize], net) {
                    out.push(Terminal { node: n, stub: None });
                }
            }
        }
        if !out.is_empty() {
            return out;
        }
    }
    // 2. Interior nodes joined to the centre by an octilinear stub (straight, or one 45°
    //    + one 0°/90° run with an off-lattice bend) whose copper lies entirely inside the
    //    pad copper. Such a stub adds no copper outside the pad, so it is legal by the
    //    pad's own legality and needs no reservation. Prefer single segments, then short.
    if contains_eroded(&hull, centre, r) {
        let mut cands: Vec<(usize, u64, NodeIdx, u32, Vec<Point>)> = Vec::new();
        if let Some((i0, j0, i1, j1)) = grid.index_range(hull.bbox()) {
            for &l in &layers {
                for j in j0..=j1 {
                    for i in i0..=i1 {
                        let n = grid.idx(i, j, l);
                        let q = grid.pos_ij(i, j);
                        if !allows(leg.node[n as usize], net) || !contains_eroded(&hull, q, r) {
                            continue;
                        }
                        // Convex pad: a polyline is inside iff all its vertices are.
                        if let Some(path) = octilinear_paths(centre, q)
                            .into_iter()
                            .find(|path| path.iter().all(|&v| contains_eroded(&hull, v, r)))
                        {
                            cands.push((path.len(), path_len(&path), n, l, path));
                        }
                    }
                }
            }
        }
        cands.sort_by_key(|a| (a.0, a.1, a.2));
        for (_, _, n, l, path) in cands.into_iter().take(MAX_STUBS) {
            let id = u32::try_from(stubs.len()).unwrap_or(u32::MAX);
            stubs.push(Stub { path, layer: grid.layers[l as usize], reserved: Vec::new() });
            out.push(Terminal { node: n, stub: Some(id) });
        }
        if !out.is_empty() {
            return out;
        }
    }
    // 3. Exterior stubs: nodes within 2 lattice steps of the centre's cell, joined by an
    //    exactly checked octilinear polyline that reserves the lattice around it.
    let (ci, cj) = grid.cell_floor(centre);
    let mut cands: Vec<(usize, u64, NodeIdx, u32, Vec<Point>)> = Vec::new();
    for &l in &layers {
        let layer = grid.layers[l as usize];
        for dj in -2..=3 {
            for di in -2..=3 {
                let (i, j) = (ci + di, cj + dj);
                if i < 0 || j < 0 || i >= i64::from(grid.nx) || j >= i64::from(grid.ny) {
                    continue;
                }
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                let n = grid.idx(i as u32, j as u32, l);
                if !allows(leg.node[n as usize], net) {
                    continue;
                }
                let to = grid.pos(n);
                if let Some(path) = octilinear_paths(centre, to).into_iter().find(|path| {
                    path.windows(2).all(|w| {
                        allows(
                            obs.classify_track(&Hull::capsule(w[0], w[1], r), layer, &cp.geom),
                            net,
                        )
                    })
                }) {
                    cands.push((path.len(), path_len(&path), n, l, path));
                }
            }
        }
    }
    cands.sort_by_key(|a| (a.0, a.1, a.2));
    let reach = stub_reach(plan, cp);
    for (_, _, n, l, path) in cands.into_iter().take(MAX_STUBS) {
        let mut reserved = Vec::new();
        for w in path.windows(2) {
            let spine = Segment::new(w[0], w[1]);
            if let Some((i0, j0, i1, j1)) = grid.index_range(spine.bbox().inflate(reach)) {
                for j in j0..=j1 {
                    for i in i0..=i1 {
                        if point_segment_dist_lt(grid.pos_ij(i, j), spine, reach) {
                            reserved.push(grid.idx(i, j, l));
                        }
                    }
                }
            }
        }
        reserved.sort_unstable();
        reserved.dedup();
        let id = u32::try_from(stubs.len()).unwrap_or(u32::MAX);
        stubs.push(Stub { path, layer: grid.layers[l as usize], reserved });
        out.push(Terminal { node: n, stub: Some(id) });
    }
    out
}

/// Radius around a stub spine inside which other nets' lattice nodes would be unsafe:
/// the largest required centre distance to any lattice element plus half a diagonal
/// (so every point of an adjacent lattice edge is covered).
fn stub_reach(plan: &Plan, cp: &ClassPlan) -> Coord {
    let r = cp.geom.track_radius;
    let worst = plan
        .classes
        .iter()
        .map(|o| {
            let c = o.geom.clearance.max(cp.geom.clearance);
            (r + o.geom.track_radius + c).max(r + o.geom.via_radius + c)
        })
        .max()
        .unwrap_or(0);
    #[allow(clippy::cast_possible_truncation)]
    let half_diag = (plan.pitch as f64 * std::f64::consts::FRAC_1_SQRT_2).ceil() as Coord;
    worst + half_diag + 1
}

/// Exact: does the disk of radius `s` centred at `q` lie inside the pad copper `h`
/// (i.e. is `q` inside `h` eroded by `s`)? Boundary cases count as outside (conservative).
#[must_use]
pub fn contains_eroded(h: &Hull, q: Point, s: Coord) -> bool {
    if h.radius >= s {
        let inner = Hull { pts: h.pts.clone(), radius: h.radius - s };
        if inner.radius == 0 && inner.pts.len() < 3 {
            return inner.pts.contains(&q)
                || (inner.pts.len() == 2
                    && ferroute_geom::orient(inner.pts[0], inner.pts[1], q) == 0
                    && Segment::new(inner.pts[0], inner.pts[1]).bbox().contains(q));
        }
        return inner.clearance_violated(&Hull::disk(q, 0), 0);
    }
    if h.pts.len() < 3 {
        return false;
    }
    let poly = ferroute_geom::Polygon::new(h.pts.to_vec());
    let need = s - h.radius;
    poly.contains(q) && !poly.edges().any(|e| point_segment_dist_lt(q, e, need))
}

fn is_octilinear(v: Point) -> bool {
    v.x == 0 || v.y == 0 || v.x.abs() == v.y.abs()
}

/// Octilinear polylines from `c` to `q`: the direct segment if it is already 0°/45°/90°,
/// otherwise the two dog-legs (45° run then straight run, and straight then 45°).
#[must_use]
pub fn octilinear_paths(c: Point, q: Point) -> Vec<Vec<Point>> {
    let d = q - c;
    if is_octilinear(d) {
        return vec![vec![c, q]];
    }
    let (sx, sy) = (d.x.signum(), d.y.signum());
    let m = d.x.abs().min(d.y.abs());
    let diag_first = c + Point::new(sx * m, sy * m);
    let straight_first = if d.x.abs() > d.y.abs() {
        c + Point::new(sx * (d.x.abs() - m), 0)
    } else {
        c + Point::new(0, sy * (d.y.abs() - m))
    };
    vec![vec![c, diag_first, q], vec![c, straight_first, q]]
}

fn path_len(path: &[Point]) -> u64 {
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let l = path.windows(2).map(|w| w[0].dist(w[1])).sum::<f64>().round() as u64;
    l
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dog_legs_are_octilinear_and_connect() {
        let c = Point::new(0, 0);
        for q in [
            Point::new(300, 100),
            Point::new(-100, 450),
            Point::new(70, -20),
            Point::new(5, 5),
            Point::new(0, 9),
        ] {
            for path in octilinear_paths(c, q) {
                assert_eq!((path[0], *path.last().unwrap()), (c, q));
                assert!(path.windows(2).all(|w| is_octilinear(w[1] - w[0])), "{path:?}");
            }
        }
    }
}
