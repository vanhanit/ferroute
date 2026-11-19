// SPDX-License-Identifier: AGPL-3.0-only
//! Region-bounded multi-source / multi-target A* on the lattice with PathFinder costs.

use crate::grid::{Grid, NodeIdx, forward_slot};
use crate::legality::{ClassLegality, allows};
use ferroute_board::NetId;
use ferroute_geom::Dir8;
use std::cmp::Reverse;
use std::collections::BinaryHeap;

/// Cost weights (in units of one orthogonal lattice step).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Costs {
    pub via: f32,
    pub bend45: f32,
    pub bend90: f32,
}

/// Inclusive planar index box `(i0, j0, i1, j1)`.
pub type IBox = (u32, u32, u32, u32);

/// Read-only search context (shared across parallel searches in a batch).
#[derive(Debug)]
pub struct Ctx<'a> {
    pub grid: &'a Grid,
    pub leg: &'a ClassLegality,
    /// Ring occupancy (nets holding the node in a ring).
    pub occ: &'a [u16],
    /// Base occupancy (nets with copper on the node).
    pub nb: &'a [u16],
    pub hole_occ: &'a [u16],
    pub hist: &'a [f32],
    pub pres_fac: f32,
    pub costs: Costs,
    pub net: NetId,
    pub region: IBox,
    /// Track reservation offsets (Chebyshev ball of radius k).
    pub k_offs: &'a [(i32, i32)],
    /// Via copper reservation offsets (radius rv), applied on every layer.
    pub rv_offs: &'a [(i32, i32)],
    /// Hole-channel offsets (radius rh), planar.
    pub rh_offs: &'a [(i32, i32)],
}

const FROM_SOURCE: u8 = 8;
const FROM_VIA: u8 = 9;

/// Chebyshev-ball offsets of radius `r`.
#[must_use]
pub fn ball(r: u32) -> Vec<(i32, i32)> {
    let r = i32::try_from(r).unwrap_or(0);
    let mut v = Vec::with_capacity(((2 * r + 1) * (2 * r + 1)) as usize);
    for dj in -r..=r {
        for di in -r..=r {
            v.push((di, dj));
        }
    }
    v
}

impl Ctx<'_> {
    /// Congestion seen by a base node at `(i, j, l)` with ring `offs`: other nets' rings
    /// covering the node plus other nets' base nodes inside the ring.
    fn sum_occ(&self, i: u32, j: u32, l: u32, offs: &[(i32, i32)]) -> f32 {
        let g = self.grid;
        let mut s = 0u32;
        for &(di, dj) in offs {
            let (x, y) = (i64::from(i) + i64::from(di), i64::from(j) + i64::from(dj));
            if x >= 0 && y >= 0 && x < i64::from(g.nx) && y < i64::from(g.ny) {
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                {
                    s += u32::from(self.nb[g.idx(x as u32, y as u32, l) as usize]);
                }
            }
        }
        s += u32::from(self.occ[g.idx(i, j, l) as usize]);
        #[allow(clippy::cast_precision_loss)]
        let f = s as f32;
        f
    }

    fn sum_hole(&self, i: u32, j: u32) -> f32 {
        let g = self.grid;
        let mut s = 0u32;
        for &(di, dj) in self.rh_offs {
            let (x, y) = (i64::from(i) + i64::from(di), i64::from(j) + i64::from(dj));
            if x >= 0 && y >= 0 && x < i64::from(g.nx) && y < i64::from(g.ny) {
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                {
                    s += u32::from(self.hole_occ[(y as u32 * g.nx + x as u32) as usize]);
                }
            }
        }
        #[allow(clippy::cast_precision_loss)]
        let f = s as f32;
        f
    }

    fn edge_ok(&self, a: NodeIdx, b: NodeIdx, d: Dir8) -> bool {
        let st = match forward_slot(d) {
            Some(s) => self.leg.edge_state(a, s),
            None => match forward_slot(d.opposite()) {
                Some(s) => self.leg.edge_state(b, s),
                None => return false,
            },
        };
        allows(st, self.net)
    }
}

fn octile(i: u32, j: u32, t: IBox) -> f32 {
    let dx = t.0.saturating_sub(i).max(i.saturating_sub(t.2));
    let dy = t.1.saturating_sub(j).max(j.saturating_sub(t.3));
    let (lo, hi) = if dx < dy { (dx, dy) } else { (dy, dx) };
    #[allow(clippy::cast_precision_loss)]
    let h = (hi - lo) as f32 + std::f32::consts::SQRT_2 * lo as f32;
    h
}

/// Finds a least-cost path from any `sources` node to any node with `is_target`,
/// restricted to `ctx.region` (all layers). Returns the node sequence source → target.
pub fn astar(
    ctx: &Ctx<'_>,
    sources: &[NodeIdx],
    is_target: &dyn Fn(NodeIdx) -> bool,
    target_box: IBox,
) -> Option<Vec<NodeIdx>> {
    let g = ctx.grid;
    let (i0, j0, i1, j1) = ctx.region;
    let (w, h) = (i1 - i0 + 1, j1 - j0 + 1);
    let nl = g.nl();
    let size = w as usize * h as usize * nl as usize;
    let local = |n: NodeIdx| -> Option<usize> {
        let (i, j, l) = g.coords(n);
        if i < i0 || i > i1 || j < j0 || j > j1 {
            return None;
        }
        Some(((l * h + (j - j0)) * w + (i - i0)) as usize)
    };
    let global = |x: usize| -> NodeIdx {
        let x = u32::try_from(x).unwrap_or(u32::MAX);
        let (i, r) = (x % w, x / w);
        g.idx(i + i0, r % h + j0, r / h)
    };
    let mut cost = vec![f32::INFINITY; size];
    let mut parent = vec![u32::MAX; size];
    let mut arrive = vec![FROM_SOURCE; size];
    let mut closed = vec![false; size];
    let mut heap: BinaryHeap<Reverse<(u32, u32)>> = BinaryHeap::new();
    for &s in sources {
        if let Some(ls) = local(s)
            && cost[ls] > 0.0
        {
            cost[ls] = 0.0;
            let (i, j, _) = g.coords(s);
            heap.push(Reverse((
                octile(i, j, target_box).to_bits(),
                u32::try_from(ls).unwrap_or(u32::MAX),
            )));
        }
    }
    let costs = ctx.costs;
    while let Some(Reverse((_, lx))) = heap.pop() {
        let lx = lx as usize;
        if closed[lx] {
            continue;
        }
        closed[lx] = true;
        let n = global(lx);
        if is_target(n) {
            let mut path = vec![n];
            let mut cur = lx;
            while parent[cur] != u32::MAX {
                cur = parent[cur] as usize;
                path.push(global(cur));
            }
            path.reverse();
            return Some(path);
        }
        let (i, j, l) = g.coords(n);
        let base = cost[lx];
        let arr = arrive[lx];
        let mut relax =
            |m: NodeIdx, step: f32, dir: u8, heap: &mut BinaryHeap<Reverse<(u32, u32)>>| {
                let Some(lm) = local(m) else { return };
                if closed[lm] {
                    return;
                }
                let c = base + step;
                if c < cost[lm] {
                    cost[lm] = c;
                    parent[lm] = u32::try_from(lx).unwrap_or(u32::MAX);
                    arrive[lm] = dir;
                    let (mi, mj, _) = g.coords(m);
                    heap.push(Reverse((
                        (c + octile(mi, mj, target_box)).to_bits(),
                        u32::try_from(lm).unwrap_or(u32::MAX),
                    )));
                }
            };
        for d in Dir8::ALL {
            let Some(m) = g.step(n, d) else { continue };
            if !allows(ctx.leg.node[m as usize], ctx.net) || !ctx.edge_ok(n, m, d) {
                continue;
            }
            let mut bend = 0.0;
            if arr < FROM_SOURCE {
                match Dir8::ALL[arr as usize].turn(d) {
                    0 => {}
                    1 => bend = costs.bend45,
                    2 => bend = costs.bend90,
                    _ => continue, // no acute or U turns
                }
            }
            let (mi, mj, _) = g.coords(m);
            let mut cong = ctx.sum_occ(mi, mj, l, ctx.k_offs);
            let len = if d.is_diagonal() {
                let (di, dj) = d.delta();
                let s1 = g.idx(i.wrapping_add_signed(di), j, l);
                let s2 = g.idx(i, j.wrapping_add_signed(dj), l);
                cong += f32::from(ctx.occ[s1 as usize]) + f32::from(ctx.occ[s2 as usize]);
                std::f32::consts::SQRT_2
            } else {
                1.0
            };
            let step = len * (1.0 + ctx.hist[m as usize]) * (1.0 + ctx.pres_fac * cong) + bend;
            relax(m, step, u8::try_from(d.index()).unwrap_or(0), &mut heap);
        }
        if arr != FROM_VIA && nl > 1 && allows(ctx.leg.via[g.planar(n) as usize], ctx.net) {
            let mut cong = ctx.sum_hole(i, j);
            for ll in 0..nl {
                cong += ctx.sum_occ(i, j, ll, ctx.rv_offs);
            }
            for ll in (0..nl).filter(|&x| x != l) {
                let m = g.idx(i, j, ll);
                if !allows(ctx.leg.node[m as usize], ctx.net) {
                    continue;
                }
                let step = costs.via * (1.0 + ctx.hist[m as usize]) * (1.0 + ctx.pres_fac * cong);
                relax(m, step, FROM_VIA, &mut heap);
            }
        }
    }
    None
}
