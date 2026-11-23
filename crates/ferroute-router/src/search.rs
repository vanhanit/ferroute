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
    /// Extra penalty for turning immediately after a via, so vias sit in line with
    /// straight traces and bends happen elsewhere.
    pub via_turn: f32,
    /// Penalty for using a pad break-out whose outward leg is a short kink.
    pub kink: f32,
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
    /// Per node: conflicting in-use stubs of other nets (stub-vs-stub congestion).
    pub sextra: &'a [u16],
    /// The net's own stub keep-out (nodes / canonical `(min, max)` edges it must avoid).
    pub own_nodes: &'a rustc_hash::FxHashSet<NodeIdx>,
    pub own_edges: &'a rustc_hash::FxHashSet<(NodeIdx, NodeIdx)>,
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
        let c = g.idx(i, j, l) as usize;
        s += u32::from(self.occ[c]) + u32::from(self.sextra[c]);
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

/// A search endpoint: a lattice node and the heading the route has there, if fixed
/// (e.g. the direction of a pad stub). For a source it is the heading the route arrives
/// with; for a target, the heading it must continue with.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Endpoint {
    pub node: NodeIdx,
    pub heading: Option<Dir8>,
    /// First-step directions that are not allowed from this source (bit = `Dir8::index`),
    /// e.g. departures that would form an acid trap with copper already at a junction.
    pub forbid: u8,
    /// Cost already incurred at this source (e.g. the quality penalty of its stub).
    pub cost: f32,
}

/// What a target node requires: the heading to continue with (into a stub) and the
/// cost of finishing there (e.g. the quality penalty of the stub).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct TargetInfo {
    pub heading: Option<Dir8>,
    pub cost: f32,
}

/// Bend penalty for turning from `from` into `to`; `None` for acute or U-turns
/// (forbidden). Shared by the search and the smoothing pass so both score paths the same.
#[must_use]
pub fn turn_cost(costs: &Costs, from: Option<Dir8>, to: Dir8) -> Option<f32> {
    match from.map_or(0, |f| f.turn(to)) {
        0 => Some(0.0),
        1 => Some(costs.bend45),
        2 => Some(costs.bend90),
        _ => None,
    }
}

/// Per-net cost policy, derived only from the net's own rules:
/// * high-speed nets (≥ 50 MHz or impedance-controlled) avoid layer changes and right
///   angles: vias ×3 and 90° bends ×4 (45° bends are preferred);
/// * nets with a via limit pay `1 + 4/(1 + max_vias)` more per via (×5 for "no vias"), so
///   the search spends vias only where a single layer cannot route.
#[must_use]
pub fn net_costs(base: &Costs, c: &ferroute_board::constraints::NetConstraints) -> Costs {
    let mut k = *base;
    if c.is_high_speed() {
        k.via *= 3.0;
        k.bend90 *= 4.0;
        k.via_turn *= 2.0;
    }
    if let Some(max) = c.max_vias {
        #[allow(clippy::cast_precision_loss)]
        let f = 1.0 + 4.0 / (1.0 + max.min(1 << 20) as f32);
        k.via *= f;
    }
    k
}

/// Quality penalty of a pad break-out: its bend (as a regular bend cost) plus the kink
/// penalty if its outward leg is shorter than a lattice pitch.
#[must_use]
pub fn stub_penalty(costs: &Costs, bend: u8, kink: bool) -> f32 {
    let b = match bend {
        0 => 0.0,
        1 => costs.bend45,
        _ => costs.bend90,
    };
    b + if kink { costs.kink } else { 0.0 }
}

/// Packed arrival state per search node: heading (0–7, 8 = none) + "arrived by via".
#[derive(Copy, Clone)]
struct Arrival(u8);

impl Arrival {
    const VIA: u8 = 0x10;
    fn new(heading: Option<Dir8>, via: bool) -> Self {
        let h = heading.map_or(8, |d| u8::try_from(d.index()).unwrap_or(8));
        Self(h | if via { Self::VIA } else { 0 })
    }
    fn heading(self) -> Option<Dir8> {
        Dir8::ALL.get(usize::from(self.0 & 0x0f)).copied()
    }
    fn via(self) -> bool {
        self.0 & Self::VIA != 0
    }
}

/// Finds a least-cost path from any source to any target, restricted to `ctx.region`
/// (all layers). `target(n)` returns `Some(heading)` for target nodes, where `heading`
/// is the direction the route must continue with (bend costs and the no-acute-turn rule
/// apply to it). Headings are carried through vias, so a via in line with a straight
/// trace is cheaper than one that also bends. Returns the node sequence source → target.
pub fn astar(
    ctx: &Ctx<'_>,
    sources: &[Endpoint],
    target: &dyn Fn(NodeIdx) -> Option<TargetInfo>,
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
    let mut arrive = vec![Arrival::new(None, false); size];
    let mut closed = vec![false; size];
    let mut heap: BinaryHeap<Reverse<(u32, u32)>> = BinaryHeap::new();
    let mut src_forbid: rustc_hash::FxHashMap<usize, u8> = rustc_hash::FxHashMap::default();
    for s in sources {
        if let Some(ls) = local(s.node) {
            *src_forbid.entry(ls).or_default() |= s.forbid;
        }
        if let Some(ls) = local(s.node)
            && s.cost < cost[ls]
        {
            cost[ls] = s.cost;
            arrive[ls] = Arrival::new(s.heading, false);
            let (i, j, _) = g.coords(s.node);
            // A source that is also a target finishes with zero length: its priority must
            // still include the target's finishing cost (e.g. a kinked stub's penalty).
            let finish = target(s.node).map_or(0.0, |t| t.cost);
            heap.push(Reverse((
                (s.cost + finish + octile(i, j, target_box)).to_bits(),
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
        // A source that is also a target (zero-length connection, e.g. two pad stubs on
        // one lattice node) only counts if the target's heading is a legal departure.
        let zero_len_ok = |t: TargetInfo| {
            let hd = t.heading;
            parent[lx] != u32::MAX
                || hd.is_none_or(|h| {
                    src_forbid.get(&lx).copied().unwrap_or(0) & (1 << h.index()) == 0
                })
        };
        if target(n).is_some_and(zero_len_ok) {
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
        let forbid =
            if parent[lx] == u32::MAX { src_forbid.get(&lx).copied().unwrap_or(0) } else { 0 };
        let mut relax =
            |m: NodeIdx, step: f32, next: Arrival, heap: &mut BinaryHeap<Reverse<(u32, u32)>>| {
                let Some(lm) = local(m) else { return };
                if closed[lm] {
                    return;
                }
                // Entering a target: charge the turn into its required heading.
                let exit = match target(m) {
                    Some(t) => {
                        let turn = match t.heading {
                            Some(hd) => match turn_cost(&costs, next.heading(), hd) {
                                Some(x) if next.via() && x > 0.0 => x + costs.via_turn,
                                Some(x) => x,
                                None => return,
                            },
                            None => 0.0,
                        };
                        turn + t.cost
                    }
                    None => 0.0,
                };
                let c = base + step + exit;
                if c < cost[lm] {
                    cost[lm] = c;
                    parent[lm] = u32::try_from(lx).unwrap_or(u32::MAX);
                    arrive[lm] = next;
                    let (mi, mj, _) = g.coords(m);
                    heap.push(Reverse((
                        (c + octile(mi, mj, target_box)).to_bits(),
                        u32::try_from(lm).unwrap_or(u32::MAX),
                    )));
                }
            };
        for d in Dir8::ALL {
            let Some(m) = g.step(n, d) else { continue };
            if ctx.own_nodes.contains(&m) || ctx.own_edges.contains(&(n.min(m), n.max(m))) {
                continue;
            }
            if forbid & (1 << d.index()) != 0
                || !allows(ctx.leg.node[m as usize], ctx.net)
                || !ctx.edge_ok(n, m, d)
            {
                continue;
            }
            let Some(mut bend) = turn_cost(&costs, arr.heading(), d) else { continue };
            if arr.via() && bend > 0.0 {
                bend += costs.via_turn;
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
            relax(m, step, Arrival::new(Some(d), false), &mut heap);
        }
        if !arr.via() && nl > 1 && allows(ctx.leg.via[g.planar(n) as usize], ctx.net) {
            let mut cong = ctx.sum_hole(i, j);
            for ll in 0..nl {
                cong += ctx.sum_occ(i, j, ll, ctx.rv_offs);
            }
            for ll in (0..nl).filter(|&x| x != l) {
                let m = g.idx(i, j, ll);
                if !allows(ctx.leg.node[m as usize], ctx.net) || ctx.own_nodes.contains(&m) {
                    continue;
                }
                let step = costs.via * (1.0 + ctx.hist[m as usize]) * (1.0 + ctx.pres_fac * cong);
                // The heading survives the via: turning right after it costs a bend.
                relax(m, step, Arrival::new(arr.heading(), true), &mut heap);
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use ferroute_board::constraints::NetConstraints;

    fn base() -> Costs {
        Costs { via: 8.0, bend45: 0.3, bend90: 1.0, via_turn: 2.0, kink: 20.0 }
    }

    #[test]
    fn high_speed_nets_pay_more_for_vias_and_right_angles() {
        let slow = net_costs(&base(), &NetConstraints::default());
        assert_eq!(slow, base());
        let hs = net_costs(
            &base(),
            &NetConstraints { frequency_hz: Some(480_000_000), ..NetConstraints::default() },
        );
        assert!(hs.via > slow.via && hs.bend90 > slow.bend90);
        assert!(hs.bend90 > 2.0 * hs.bend45, "two 45° bends beat one 90° bend");
        assert_eq!(hs.kink, slow.kink);
    }

    #[test]
    fn via_limited_nets_pay_more_for_vias() {
        let limited = |max| NetConstraints { max_vias: Some(max), ..NetConstraints::default() };
        let none = net_costs(&base(), &limited(0));
        let two = net_costs(&base(), &limited(2));
        let many = net_costs(&base(), &limited(10));
        assert!(none.via > two.via && two.via > many.via && many.via > base().via);
        assert_eq!(two.bend90, base().bend90, "only vias are affected");
        // Compounds with the high-speed policy.
        let hs = NetConstraints { impedance_mohm: Some(50_000), ..limited(2) };
        assert!(net_costs(&base(), &hs).via > two.via);
    }

    #[test]
    fn turn_costs_forbid_acute_turns() {
        let c = base();
        assert_eq!(turn_cost(&c, None, Dir8::E), Some(0.0));
        assert_eq!(turn_cost(&c, Some(Dir8::E), Dir8::NE), Some(0.3));
        assert_eq!(turn_cost(&c, Some(Dir8::E), Dir8::N), Some(1.0));
        assert_eq!(turn_cost(&c, Some(Dir8::E), Dir8::NW), None);
        assert_eq!(stub_penalty(&c, 1, true), 20.3);
    }
}
