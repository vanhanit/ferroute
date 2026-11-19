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
use ferroute_board::{Board, LayerId, NetId, PadId, PadShape};
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
    /// The lattice node the stub connects to (its terminal).
    pub node: NodeIdx,
    pub net: NetId,
    /// Track half-width and net-class clearance of the stub copper.
    pub radius: Coord,
    pub clearance: Coord,
}

/// A lattice entry point of a pad.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Terminal {
    pub node: NodeIdx,
    pub stub: Option<u32>,
}

/// Maximum number of stub alternatives kept per pad, and per entry direction.
const MAX_STUBS: usize = 8;
const MAX_PER_DIR: usize = 2;

/// Allowed entry/exit directions of a pad, as integer unit steps `(ux, uy)`:
/// * circular pads (and therefore through-hole pads, vias and holes): all 8 octilinear
///   directions through the centre;
/// * every other shape: the 4 pad axes, i.e. perpendicular to each side through its
///   midpoint (short and long sides), following the pad rotation. Pads rotated by a
///   non-octilinear angle fall back to the 8 octilinear directions.
#[must_use]
pub fn entry_dirs(board: &Board, pad: PadId) -> Vec<(i64, i64)> {
    const ALL8: [(i64, i64); 8] =
        [(1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1)];
    let p = board.pad(pad);
    if matches!(p.shape, PadShape::Circle { .. }) {
        return ALL8.to_vec();
    }
    let rot = board.footprint(p.footprint).placement.rot.add(p.local_rot);
    let mut out = Vec::with_capacity(4);
    for axis in [(1, 0), (0, 1), (-1, 0), (0, -1)] {
        let v = Point::new(axis.0 * 1_000_000, axis.1 * 1_000_000).rotated(rot);
        let d = (v.x.signum(), v.y.signum());
        let octi = v.x == 0 || v.y == 0 || v.x.abs() == v.y.abs();
        if !octi {
            return ALL8.to_vec();
        }
        out.push(d);
    }
    out
}

/// Smallest `k ≥ 0` such that `pred(k)` is false, assuming `pred` is monotone
/// (true … true false … false) on `[0, hi]`; `hi + 1` if never false.
fn first_false(hi: i64, pred: impl Fn(i64) -> bool) -> i64 {
    if !pred(0) {
        return 0;
    }
    let (mut lo, mut hi) = (0i64, hi);
    if pred(hi) {
        return hi + 1;
    }
    while hi - lo > 1 {
        let mid = lo + (hi - lo) / 2;
        if pred(mid) { lo = mid } else { hi = mid }
    }
    hi
}

/// Computes the terminals of `pad` for `net` routed with class plan `cp`.
///
/// Every terminal carries a stub `centre → m (→ q)`: the first segment leaves the pad
/// centre along an [`entry_dirs`] direction and crosses the pad edge there (through the
/// side midpoint for rectangular shapes); the bend `m` lies at or beyond the pad edge and
/// may be off the lattice; the optional second segment is 0°/45°/90° and points away from
/// the pad to the lattice node `q`. Stubs are checked exactly; only their copper outside
/// the pad reserves lattice nodes.
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
    let c = board.pad_position(pad);
    let r = cp.geom.track_radius;
    let layers: Vec<u32> =
        (0..grid.nl()).filter(|&g| p.layers.contains(grid.layers[g as usize])).collect();
    let dirs = entry_dirs(board, pad);
    let inside = |q: Point| hull.clearance_violated(&Hull::disk(q, 0), 0);
    let bb = hull.bbox();
    let span = bb.width().max(bb.height()) + 1;
    // Per direction: first step index outside the pad, and last index whose track disk
    // is still contained in the pad copper (-1 if even the centre is not).
    let k_edge: Vec<i64> = dirs
        .iter()
        .map(|&(ux, uy)| first_false(span, |k| inside(c + Point::new(k * ux, k * uy))))
        .collect();
    let k_in: Vec<i64> = dirs
        .iter()
        .map(|&(ux, uy)| {
            first_false(span, |k| contains_eroded(&hull, c + Point::new(k * ux, k * uy), r)) - 1
        })
        .collect();
    let reach = stub_reach(plan, cp, obs.rules.hole_clearance);
    // (length, segments, dir index, node, layer, path)
    let mut cands: Vec<(u64, usize, usize, NodeIdx, u32, Vec<Point>)> = Vec::new();
    let Some((i0, j0, i1, j1)) = grid.index_range(bb.inflate(3 * grid.pitch)) else {
        return Vec::new();
    };
    for &l in &layers {
        let layer = grid.layers[l as usize];
        for j in j0..=j1 {
            for i in i0..=i1 {
                let n = grid.idx(i, j, l);
                let q = grid.pos_ij(i, j);
                if !allows(leg.node[n as usize], net) || inside(q) {
                    continue;
                }
                let (a, b) = (q.x - c.x, q.y - c.y);
                for (di, &(ux, uy)) in dirs.iter().enumerate() {
                    // Break-out point m = c + k·u such that q − m is 0°/45°/90°.
                    let mut ks: Vec<i64> = Vec::new();
                    let mut push = |num: i64, den: i64| {
                        if den != 0 && num % den == 0 {
                            ks.push(num / den);
                        }
                    };
                    push(a, ux);
                    push(b, uy);
                    push(a - b, ux - uy);
                    push(a + b, ux + uy);
                    ks.sort_unstable();
                    ks.dedup();
                    for k in ks {
                        if k < k_edge[di] {
                            continue;
                        }
                        let m = c + Point::new(k * ux, k * uy);
                        let v = q - m;
                        if !(v.x == 0 || v.y == 0 || v.x.abs() == v.y.abs()) {
                            continue;
                        }
                        // Must leave the pad: second leg points strictly outward.
                        if m != q && v.x * ux + v.y * uy <= 0 {
                            continue;
                        }
                        let path = if m == q { vec![c, q] } else { vec![c, m, q] };
                        let ok = path.windows(2).all(|w| {
                            allows(
                                obs.classify_track(&Hull::capsule(w[0], w[1], r), layer, &cp.geom),
                                net,
                            )
                        });
                        if ok {
                            cands.push((path_len(&path), path.len(), di, n, l, path));
                        }
                    }
                }
            }
        }
    }
    cands.sort_by_key(|x| (x.0, x.1, x.2, x.3));
    let mut per_dir = vec![0usize; dirs.len()];
    let mut out = Vec::new();
    for (_, _, di, n, l, path) in cands {
        if out.len() >= MAX_STUBS
            || per_dir[di] >= MAX_PER_DIR
            || out.iter().any(|t: &Terminal| t.node == n)
        {
            continue;
        }
        per_dir[di] += 1;
        // Reserve the lattice only around copper outside the pad: the portion of the
        // first leg up to k_in is contained in the pad copper (adds nothing).
        let (ux, uy) = dirs[di];
        let start = if k_in[di] >= 0 { c + Point::new(k_in[di] * ux, k_in[di] * uy) } else { c };
        let mut segs = vec![Segment::new(start, path[1])];
        if path.len() == 3 {
            segs.push(Segment::new(path[1], path[2]));
        }
        let mut reserved = Vec::new();
        for spine in segs {
            if let Some((a0, b0, a1, b1)) = grid.index_range(spine.bbox().inflate(reach)) {
                for j in b0..=b1 {
                    for i in a0..=a1 {
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
        stubs.push(Stub {
            path,
            layer: grid.layers[l as usize],
            reserved,
            node: n,
            net,
            radius: r,
            clearance: cp.geom.clearance,
        });
        out.push(Terminal { node: n, stub: Some(id) });
    }
    out
}

/// Lattice nodes strictly inside pad copper (on the pad's layers). Routes must not run
/// across pads; pads are entered only through their stubs.
#[must_use]
pub fn pad_interior_nodes(board: &Board, grid: &Grid) -> Vec<NodeIdx> {
    let mut v = Vec::new();
    for id in board.pad_ids() {
        let p = board.pad(id);
        if p.layers.is_empty() {
            continue;
        }
        let hull = board.pad_hull(id);
        let Some((i0, j0, i1, j1)) = grid.index_range(hull.bbox()) else { continue };
        for l in (0..grid.nl()).filter(|&g| p.layers.contains(grid.layers[g as usize])) {
            for j in j0..=j1 {
                for i in i0..=i1 {
                    if hull.clearance_violated(&Hull::disk(grid.pos_ij(i, j), 0), 0) {
                        v.push(grid.idx(i, j, l));
                    }
                }
            }
        }
    }
    v
}

/// Radius around a stub's outside copper inside which other nets' lattice nodes would be
/// unsafe. Track elements need half a lattice diagonal of slack (their edges run between
/// nodes); vias sit exactly on nodes and need none.
fn stub_reach(plan: &Plan, cp: &ClassPlan, hole_clearance: Coord) -> Coord {
    let r = cp.geom.track_radius;
    #[allow(clippy::cast_possible_truncation)]
    let half_diag = (plan.pitch as f64 * std::f64::consts::FRAC_1_SQRT_2).ceil() as Coord;
    plan.classes
        .iter()
        .map(|o| {
            let c = o.geom.clearance.max(cp.geom.clearance);
            let track = r + o.geom.track_radius + c + half_diag;
            let via = r + (o.geom.via_radius + c).max(o.geom.via_drill / 2 + hole_clearance);
            track.max(via)
        })
        .max()
        .unwrap_or(0)
        + 1
}

/// Pairs of stubs of different nets whose copper would violate clearance (exact).
/// Returned as an adjacency list indexed by stub id. Negotiation treats a conflicting
/// pair like a shared resource, so at most one of the two can be used.
#[must_use]
pub fn stub_conflicts(stubs: &[Stub]) -> Vec<Vec<u32>> {
    let hulls: Vec<Vec<Hull>> = stubs
        .iter()
        .map(|s| s.path.windows(2).map(|w| Hull::capsule(w[0], w[1], s.radius)).collect())
        .collect();
    let reach = stubs.iter().map(|s| s.clearance).max().unwrap_or(0) + 1;
    let boxes: Vec<ferroute_geom::Rect> = hulls
        .iter()
        .map(|hs| {
            hs.iter()
                .map(Hull::bbox)
                .reduce(ferroute_geom::Rect::union)
                .unwrap_or(ferroute_geom::Rect::new(Point::ORIGIN, Point::ORIGIN))
                .inflate(reach)
        })
        .collect();
    let index = ferroute_drc::BucketGrid::new(&boxes, 2_000_000);
    let mut out = vec![Vec::new(); stubs.len()];
    for (a, b) in index.pairs(&boxes) {
        let (sa, sb) = (&stubs[a as usize], &stubs[b as usize]);
        if sa.net == sb.net || sa.layer != sb.layer {
            continue;
        }
        let c = sa.clearance.max(sb.clearance);
        let hit = hulls[a as usize]
            .iter()
            .any(|x| hulls[b as usize].iter().any(|y| x.clearance_violated(y, c)));
        if hit {
            out[a as usize].push(b);
            out[b as usize].push(a);
        }
    }
    out
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
