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
    pub from: Point,
    pub to: Point,
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
    // 2. Interior nodes joined to the centre by a stub whose copper lies entirely inside
    //    the pad copper. Such a stub adds no copper outside the pad, so it is legal by the
    //    pad's own legality and needs no reservation. Prefer octilinear stubs, then short.
    if contains_eroded(&hull, centre, r) {
        let mut cands: Vec<(bool, u128, NodeIdx, u32)> = Vec::new();
        if let Some((i0, j0, i1, j1)) = grid.index_range(hull.bbox()) {
            for &l in &layers {
                for j in j0..=j1 {
                    for i in i0..=i1 {
                        let n = grid.idx(i, j, l);
                        let q = grid.pos_ij(i, j);
                        if allows(leg.node[n as usize], net) && contains_eroded(&hull, q, r) {
                            let v = q - centre;
                            let octi = v.x == 0 || v.y == 0 || v.x.abs() == v.y.abs();
                            cands.push((!octi, v.norm2(), n, l));
                        }
                    }
                }
            }
        }
        cands.sort_unstable();
        for &(_, _, n, l) in cands.iter().take(MAX_STUBS) {
            let id = u32::try_from(stubs.len()).unwrap_or(u32::MAX);
            stubs.push(Stub {
                from: centre,
                to: grid.pos(n),
                layer: grid.layers[l as usize],
                reserved: Vec::new(),
            });
            out.push(Terminal { node: n, stub: Some(id) });
        }
        if !out.is_empty() {
            return out;
        }
    }
    // Stub candidates: nodes within 2 lattice steps of the pad centre's cell.
    let (ci, cj) = grid.cell_floor(centre);
    let mut cands: Vec<(bool, u128, NodeIdx, u32)> = Vec::new();
    for &l in &layers {
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
                let st = obs.classify_track(
                    &Hull::capsule(centre, to, cp.geom.track_radius),
                    grid.layers[l as usize],
                    &cp.geom,
                );
                if allows(st, net) {
                    let v = to - centre;
                    let octi = v.x == 0 || v.y == 0 || v.x.abs() == v.y.abs();
                    cands.push((!octi, v.norm2(), n, l));
                }
            }
        }
    }
    cands.sort_unstable();
    let reach = stub_reach(plan, cp);
    for &(_, _, n, l) in cands.iter().take(MAX_STUBS) {
        let to = grid.pos(n);
        let spine = Segment::new(centre, to);
        let mut reserved = Vec::new();
        if let Some((i0, j0, i1, j1)) = grid.index_range(spine.bbox().inflate(reach)) {
            for j in j0..=j1 {
                for i in i0..=i1 {
                    if point_segment_dist_lt(grid.pos_ij(i, j), spine, reach) {
                        reserved.push(grid.idx(i, j, l));
                    }
                }
            }
        }
        let id = u32::try_from(stubs.len()).unwrap_or(u32::MAX);
        stubs.push(Stub { from: centre, to, layer: grid.layers[l as usize], reserved });
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
