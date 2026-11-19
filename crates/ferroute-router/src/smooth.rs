// SPDX-License-Identifier: AGPL-3.0-only
//! Lattice-preserving path smoothing.
//!
//! Negotiated A\* paths can contain stair-steps and jogs. This pass replaces a sub-path
//! between two of its nodes by a *straight* or *two-segment octilinear* lattice
//! connection (one 45° run + one 0°/90° run) whenever that connection is legal for the
//! net and strictly cheaper (length + bend penalties). The result stays on the lattice
//! (every vertex is a lattice node, every segment is 0°/45°/90°), so the separation
//! theorem keeps applying; the caller re-reserves the route and keeps it only if it
//! creates no conflict (INV-ROUTE-1).
//!
//! Pinned nodes (path endpoints, Steiner junctions, via sites) are never skipped.

use crate::grid::{Grid, NodeIdx, forward_slot};
use crate::legality::{ClassLegality, allows};
use crate::search::Costs;
use ferroute_board::NetId;
use ferroute_geom::Dir8;

/// Longest window (in path nodes) considered for one shortcut.
const MAX_WINDOW: usize = 96;

fn dir_between(grid: &Grid, a: NodeIdx, b: NodeIdx) -> Option<Dir8> {
    let (ai, aj, al) = grid.coords(a);
    let (bi, bj, bl) = grid.coords(b);
    if al != bl {
        return None;
    }
    let d = (i64::from(bi) - i64::from(ai), i64::from(bj) - i64::from(aj));
    Dir8::ALL.into_iter().find(|x| {
        let (di, dj) = x.delta();
        (i64::from(di), i64::from(dj)) == d
    })
}

/// Cost of a node sequence (length + bends), including the turn from `prev` into the
/// first step and from the last step into `next`. `None` if it contains an acute turn.
fn cost(
    grid: &Grid,
    nodes: &[NodeIdx],
    prev: Option<Dir8>,
    next: Option<Dir8>,
    c: &Costs,
) -> Option<f32> {
    let mut total = 0.0f32;
    let mut last = prev;
    for w in nodes.windows(2) {
        let d = dir_between(grid, w[0], w[1])?;
        total += if d.is_diagonal() { std::f32::consts::SQRT_2 } else { 1.0 };
        if let Some(p) = last {
            match p.turn(d) {
                0 => {}
                1 => total += c.bend45,
                2 => total += c.bend90,
                _ => return None,
            }
        }
        last = Some(d);
    }
    if let (Some(l), Some(n)) = (last, next) {
        match l.turn(n) {
            0 => {}
            1 => total += c.bend45,
            2 => total += c.bend90,
            _ => return None,
        }
    }
    Some(total)
}

fn step_ok(leg: &ClassLegality, net: NetId, a: NodeIdx, b: NodeIdx, d: Dir8) -> bool {
    if !allows(leg.node[b as usize], net) {
        return false;
    }
    let st = match forward_slot(d) {
        Some(s) => leg.edge_state(a, s),
        None => match forward_slot(d.opposite()) {
            Some(s) => leg.edge_state(b, s),
            None => return false,
        },
    };
    allows(st, net)
}

/// Walks `n` steps in direction `d` from `from`, appending nodes; `false` if illegal.
fn walk(
    grid: &Grid,
    leg: &ClassLegality,
    net: NetId,
    from: NodeIdx,
    d: Dir8,
    n: u32,
    out: &mut Vec<NodeIdx>,
) -> bool {
    let mut cur = from;
    for _ in 0..n {
        let Some(nx) = grid.step(cur, d) else { return false };
        if !step_ok(leg, net, cur, nx, d) {
            return false;
        }
        out.push(nx);
        cur = nx;
    }
    true
}

/// Candidate octilinear connections from `a` to `b` (same layer), excluding `a`.
fn candidates(
    grid: &Grid,
    leg: &ClassLegality,
    net: NetId,
    a: NodeIdx,
    b: NodeIdx,
) -> Vec<Vec<NodeIdx>> {
    let (ai, aj, _) = grid.coords(a);
    let (bi, bj, _) = grid.coords(b);
    let dx = i64::from(bi) - i64::from(ai);
    let dy = i64::from(bj) - i64::from(aj);
    let (adx, ady) = (dx.unsigned_abs(), dy.unsigned_abs());
    let diag_n = u32::try_from(adx.min(ady)).unwrap_or(0);
    let straight_n = u32::try_from(adx.max(ady) - adx.min(ady)).unwrap_or(0);
    let sx = dx.signum();
    let sy = dy.signum();
    let find = |di: i64, dj: i64| {
        Dir8::ALL.into_iter().find(|x| {
            let (a, b) = x.delta();
            (i64::from(a), i64::from(b)) == (di, dj)
        })
    };
    let diag = find(sx, sy);
    let straight = if adx >= ady { find(sx, 0) } else { find(0, sy) };
    let mut out = Vec::new();
    let mut orders: Vec<[(Option<Dir8>, u32); 2]> = vec![[(diag, diag_n), (straight, straight_n)]];
    if diag_n > 0 && straight_n > 0 {
        orders.push([(straight, straight_n), (diag, diag_n)]);
    }
    for order in orders {
        let mut nodes = Vec::new();
        let mut cur = a;
        let mut ok = true;
        for (d, n) in order {
            if n == 0 {
                continue;
            }
            let Some(d) = d else {
                ok = false;
                break;
            };
            if !walk(grid, leg, net, cur, d, n, &mut nodes) {
                ok = false;
                break;
            }
            cur = *nodes.last().unwrap_or(&cur);
        }
        if ok && cur == b {
            out.push(nodes);
        }
    }
    out
}

/// Smooths one path. `pinned(n)` marks nodes that must stay on the path.
#[must_use]
pub fn smooth_path(
    grid: &Grid,
    leg: &ClassLegality,
    net: NetId,
    path: &[NodeIdx],
    pinned: &dyn Fn(NodeIdx) -> bool,
    costs: &Costs,
) -> Vec<NodeIdx> {
    if path.len() < 3 {
        return path.to_vec();
    }
    let mut out = vec![path[0]];
    let mut i = 0usize;
    while i + 1 < path.len() {
        let layer = grid.coords(path[i]).2;
        let prev = if out.len() >= 2 {
            dir_between(grid, out[out.len() - 2], out[out.len() - 1])
        } else {
            None
        };
        // Farthest same-layer index reachable without skipping a pinned node.
        let mut lim = i + 1;
        while lim + 1 < path.len()
            && lim < i + MAX_WINDOW
            && grid.coords(path[lim + 1]).2 == layer
            && grid.coords(path[lim]).2 == layer
            && !pinned(path[lim])
        {
            lim += 1;
        }
        let mut done = false;
        for j in (i + 2..=lim).rev() {
            if grid.coords(path[j]).2 != layer {
                continue;
            }
            let next = path.get(j + 1).and_then(|&n| dir_between(grid, path[j], n));
            let Some(old) = cost(grid, &path[i..=j], prev, next, costs) else { continue };
            let mut best: Option<(f32, Vec<NodeIdx>)> = None;
            for cand in candidates(grid, leg, net, path[i], path[j]) {
                let mut seq = Vec::with_capacity(cand.len() + 1);
                seq.push(path[i]);
                seq.extend_from_slice(&cand);
                if let Some(c) = cost(grid, &seq, prev, next, costs)
                    && c + 1e-4 < old
                    && best.as_ref().is_none_or(|b| c < b.0)
                {
                    best = Some((c, cand));
                }
            }
            if let Some((_, cand)) = best {
                out.extend(cand);
                i = j;
                done = true;
                break;
            }
        }
        if !done {
            out.push(path[i + 1]);
            i += 1;
        }
    }
    out
}
