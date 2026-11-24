// SPDX-License-Identifier: AGPL-3.0-only
//! Region-bounded multi-source / multi-target A* on the lattice with PathFinder costs.

use crate::grid::{Grid, NodeIdx, forward_slot};
use crate::legality::{ClassLegality, allows};
use ferroute_board::NetId;
use ferroute_geom::Dir8;

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
    // A 90° bend never beats two 45° bends (the chamfer pass would undo it anyway).
    k.bend90 = k.bend90.max(2.0 * k.bend45 * 1.25);
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

/// Per-node search record. A record is valid only while its `stamp` equals the current
/// search generation, so scratch memory is never cleared between searches.
#[derive(Copy, Clone)]
struct Rec {
    cost: f32,
    parent: u32,
    stamp: u32,
    /// Memoised track congestion of the node (independent of the parent node).
    cong: f32,
    arrive: Arrival,
    closed: bool,
    has_cong: bool,
}

impl Rec {
    const FRESH: Self = Self {
        cost: f32::INFINITY,
        parent: u32::MAX,
        stamp: 0,
        cong: 0.0,
        arrive: Arrival(8),
        closed: false,
        has_cong: false,
    };
}

/// Reusable per-thread A* memory (results never depend on its previous contents).
#[derive(Default)]
struct Scratch {
    generation: u32,
    recs: Vec<Rec>,
    /// Memoised via-site congestion per window column `(j - j0) * w + (i - i0)`.
    vstamp: Vec<u32>,
    vcong: Vec<f32>,
    heap: MinHeap,
}

/// 4-ary min-heap of packed `(cost bits << 32 | local index)` keys. Keys are unique per
/// `(cost, node)` pair, so the pop sequence equals that of any other correct min-heap.
#[derive(Default)]
struct MinHeap {
    v: Vec<u64>,
}

impl MinHeap {
    fn clear(&mut self) {
        self.v.clear();
    }

    fn push(&mut self, x: u64) {
        let mut i = self.v.len();
        self.v.push(x);
        while i > 0 {
            let p = (i - 1) / 4;
            if self.v[p] <= x {
                break;
            }
            self.v[i] = self.v[p];
            i = p;
        }
        self.v[i] = x;
    }

    fn pop(&mut self) -> Option<u64> {
        let last = self.v.pop()?;
        if self.v.is_empty() {
            return Some(last);
        }
        let top = self.v[0];
        let n = self.v.len();
        let mut i = 0;
        loop {
            let c = 4 * i + 1;
            if c >= n {
                break;
            }
            let mut m = c;
            for k in c + 1..(c + 4).min(n) {
                if self.v[k] < self.v[m] {
                    m = k;
                }
            }
            if self.v[m] >= last {
                break;
            }
            self.v[i] = self.v[m];
            i = m;
        }
        self.v[i] = last;
        Some(top)
    }
}

/// Largest record array kept alive between searches.
const KEEP_RECS: usize = 1 << 22;

thread_local! {
    static SCRATCH: std::cell::RefCell<Scratch> = std::cell::RefCell::new(Scratch::default());
}

/// Reservation offsets with their linear index deltas for the interior fast path.
struct Offs<'a> {
    list: &'a [(i32, i32)],
    lin: Vec<i64>,
    r: u32,
}

impl<'a> Offs<'a> {
    fn new(list: &'a [(i32, i32)], nx: u32) -> Self {
        let lin =
            list.iter().map(|&(di, dj)| i64::from(dj) * i64::from(nx) + i64::from(di)).collect();
        let r =
            list.iter().map(|&(a, b)| a.unsigned_abs().max(b.unsigned_abs())).max().unwrap_or(0);
        Self { list, lin, r }
    }

    /// Whether the whole ball around planar `(i, j)` lies inside the grid.
    fn interior(&self, g: &Grid, i: u32, j: u32) -> bool {
        i >= self.r && j >= self.r && i + self.r < g.nx && j + self.r < g.ny
    }
}

impl Ctx<'_> {
    /// [`Ctx::sum_occ`] with an interior fast path (no per-offset bounds checks).
    fn sum_occ_fast(&self, o: &Offs<'_>, i: u32, j: u32, l: u32) -> f32 {
        let g = self.grid;
        if !o.interior(g, i, j) {
            return self.sum_occ(i, j, l, o.list);
        }
        let c = g.idx(i, j, l) as usize;
        let mut s = 0u32;
        for &d in &o.lin {
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            {
                s += u32::from(self.nb[(c as i64 + d) as usize]);
            }
        }
        s += u32::from(self.occ[c]) + u32::from(self.sextra[c]);
        #[allow(clippy::cast_precision_loss)]
        let f = s as f32;
        f
    }

    /// [`Ctx::sum_hole`] with an interior fast path.
    fn sum_hole_fast(&self, o: &Offs<'_>, i: u32, j: u32) -> f32 {
        let g = self.grid;
        if !o.interior(g, i, j) {
            return self.sum_hole(i, j);
        }
        let c = j as usize * g.nx as usize + i as usize;
        let mut s = 0u32;
        for &d in &o.lin {
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            {
                s += u32::from(self.hole_occ[(c as i64 + d) as usize]);
            }
        }
        #[allow(clippy::cast_precision_loss)]
        let f = s as f32;
        f
    }
}

/// Finds a least-cost path from any source to any target, restricted to `ctx.region`
/// (all layers). `target(n)` returns `Some(heading)` for target nodes, where `heading`
/// is the direction the route must continue with (bend costs and the no-acute-turn rule
/// apply to it). Headings are carried through vias, so a via in line with a straight
/// trace is cheaper than one that also bends. Returns the node sequence source → target.
///
/// Search memory is reused per thread (generation-stamped, never cleared); the result is a
/// pure function of the arguments.
pub fn astar(
    ctx: &Ctx<'_>,
    sources: &[Endpoint],
    target: &dyn Fn(NodeIdx) -> Option<TargetInfo>,
    target_box: IBox,
) -> Option<Vec<NodeIdx>> {
    SCRATCH.with(|cell| {
        let mut sc = cell.borrow_mut();
        let r = astar_in(&mut sc, ctx, sources, target, target_box);
        if sc.recs.len() > KEEP_RECS {
            let generation = sc.generation;
            *sc = Scratch { generation, ..Scratch::default() };
        }
        r
    })
}

#[allow(clippy::too_many_lines)]
fn astar_in(
    sc: &mut Scratch,
    ctx: &Ctx<'_>,
    sources: &[Endpoint],
    target: &dyn Fn(NodeIdx) -> Option<TargetInfo>,
    target_box: IBox,
) -> Option<Vec<NodeIdx>> {
    let g = ctx.grid;
    let (i0, j0, i1, j1) = ctx.region;
    let (w, h) = (i1 - i0 + 1, j1 - j0 + 1);
    let nl = g.nl();
    let columns = w as usize * h as usize;
    let size = columns * nl as usize;
    if sc.generation == u32::MAX {
        for r in &mut sc.recs {
            r.stamp = 0;
        }
        sc.vstamp.iter_mut().for_each(|s| *s = 0);
        sc.generation = 0;
    }
    sc.generation += 1;
    let generation = sc.generation;
    if sc.recs.len() < size {
        sc.recs.resize(size, Rec::FRESH);
    }
    if sc.vstamp.len() < columns {
        sc.vstamp.resize(columns, 0);
        sc.vcong.resize(columns, 0.0);
    }
    sc.heap.clear();
    let Scratch { recs, vstamp, vcong, heap, .. } = sc;
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
    let touch = |recs: &mut [Rec], x: usize| {
        let r = &mut recs[x];
        if r.stamp != generation {
            *r = Rec { stamp: generation, ..Rec::FRESH };
        }
    };
    let key = |c: f32, lx: usize| -> u64 {
        (u64::from(c.to_bits()) << 32) | u64::from(u32::try_from(lx).unwrap_or(u32::MAX))
    };
    let mut src_forbid: rustc_hash::FxHashMap<usize, u8> = rustc_hash::FxHashMap::default();
    for s in sources {
        if let Some(ls) = local(s.node) {
            *src_forbid.entry(ls).or_default() |= s.forbid;
            touch(recs, ls);
            if s.cost < recs[ls].cost {
                recs[ls].cost = s.cost;
                recs[ls].arrive = Arrival::new(s.heading, false);
                let (i, j, _) = g.coords(s.node);
                // A source that is also a target finishes with zero length: its priority must
                // still include the target's finishing cost (e.g. a kinked stub's penalty).
                let finish = target(s.node).map_or(0.0, |t| t.cost);
                heap.push(key(s.cost + finish + octile(i, j, target_box), ls));
            }
        }
    }
    let costs = ctx.costs;
    let k_offs = Offs::new(ctx.k_offs, g.nx);
    let rv_offs = Offs::new(ctx.rv_offs, g.nx);
    let rh_offs = Offs::new(ctx.rh_offs, g.nx);
    let has_own = !ctx.own_nodes.is_empty() || !ctx.own_edges.is_empty();
    while let Some(packed) = heap.pop() {
        let lx = (packed & 0xffff_ffff) as usize;
        if recs[lx].closed {
            continue;
        }
        recs[lx].closed = true;
        let (i, j, l) = {
            let x = u32::try_from(lx).unwrap_or(u32::MAX);
            let (c, r) = (x % w, x / w);
            (c + i0, r % h + j0, r / h)
        };
        let n = g.idx(i, j, l);
        // A source that is also a target (zero-length connection, e.g. two pad stubs on
        // one lattice node) only counts if the target's heading is a legal departure.
        let parent_lx = recs[lx].parent;
        let zero_len_ok = |t: TargetInfo| {
            let hd = t.heading;
            parent_lx != u32::MAX
                || hd.is_none_or(|h| {
                    src_forbid.get(&lx).copied().unwrap_or(0) & (1 << h.index()) == 0
                })
        };
        if target(n).is_some_and(zero_len_ok) {
            let mut path = vec![n];
            let mut cur = lx;
            while recs[cur].parent != u32::MAX {
                cur = recs[cur].parent as usize;
                path.push(global(cur));
            }
            path.reverse();
            return Some(path);
        }
        let base = recs[lx].cost;
        let arr = recs[lx].arrive;
        let forbid =
            if parent_lx == u32::MAX { src_forbid.get(&lx).copied().unwrap_or(0) } else { 0 };
        let relax = |recs: &mut [Rec],
                     heap: &mut MinHeap,
                     lm: usize,
                     m: NodeIdx,
                     (mi, mj): (u32, u32),
                     step: f32,
                     next: Arrival| {
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
            if c < recs[lm].cost {
                recs[lm].cost = c;
                recs[lm].parent = u32::try_from(lx).unwrap_or(u32::MAX);
                recs[lm].arrive = next;
                heap.push(key(c + octile(mi, mj, target_box), lm));
            }
        };
        for d in Dir8::ALL {
            let (di, dj) = d.delta();
            let (ni, nj) = (i64::from(i) + i64::from(di), i64::from(j) + i64::from(dj));
            if ni < i64::from(i0) || nj < i64::from(j0) || ni > i64::from(i1) || nj > i64::from(j1)
            {
                // Outside the grid or the search window: never relaxed.
                continue;
            }
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let (mi, mj) = (ni as u32, nj as u32);
            let m = g.idx(mi, mj, l);
            let lm = ((l * h + (mj - j0)) * w + (mi - i0)) as usize;
            touch(recs, lm);
            if recs[lm].closed {
                continue;
            }
            if has_own
                && (ctx.own_nodes.contains(&m) || ctx.own_edges.contains(&(n.min(m), n.max(m))))
            {
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
            if !recs[lm].has_cong {
                recs[lm].cong = ctx.sum_occ_fast(&k_offs, mi, mj, l);
                recs[lm].has_cong = true;
            }
            let mut cong = recs[lm].cong;
            let len = if d.is_diagonal() {
                let s1 = g.idx(mi, j, l);
                let s2 = g.idx(i, mj, l);
                cong += f32::from(ctx.occ[s1 as usize]) + f32::from(ctx.occ[s2 as usize]);
                std::f32::consts::SQRT_2
            } else {
                1.0
            };
            let step = len * (1.0 + ctx.hist[m as usize]) * (1.0 + ctx.pres_fac * cong) + bend;
            relax(recs, heap, lm, m, (mi, mj), step, Arrival::new(Some(d), false));
        }
        if !arr.via() && nl > 1 && allows(ctx.leg.via[(j * g.nx + i) as usize], ctx.net) {
            let col = ((j - j0) * w + (i - i0)) as usize;
            if vstamp[col] != generation {
                let mut cong = ctx.sum_hole_fast(&rh_offs, i, j);
                for ll in 0..nl {
                    cong += ctx.sum_occ_fast(&rv_offs, i, j, ll);
                }
                vstamp[col] = generation;
                vcong[col] = cong;
            }
            let cong = vcong[col];
            for ll in (0..nl).filter(|&x| x != l) {
                let m = g.idx(i, j, ll);
                if !allows(ctx.leg.node[m as usize], ctx.net) || ctx.own_nodes.contains(&m) {
                    continue;
                }
                let lm = ((ll * h + (j - j0)) * w + (i - i0)) as usize;
                touch(recs, lm);
                if recs[lm].closed {
                    continue;
                }
                let step = costs.via * (1.0 + ctx.hist[m as usize]) * (1.0 + ctx.pres_fac * cong);
                // The heading survives the via: turning right after it costs a bend.
                relax(recs, heap, lm, m, (i, j), step, Arrival::new(arr.heading(), true));
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
    fn right_angle_always_costs_more_than_two_forty_fives() {
        let odd = Costs { bend45: 0.8, bend90: 1.0, ..base() };
        for c in [
            NetConstraints::default(),
            NetConstraints { frequency_hz: Some(480_000_000), ..NetConstraints::default() },
            NetConstraints { max_vias: Some(0), ..NetConstraints::default() },
        ] {
            for b in [base(), odd] {
                let k = net_costs(&b, &c);
                assert!(k.bend90 > 2.0 * k.bend45, "{k:?}");
            }
        }
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

    /// Search memory is reused per thread: results must not depend on what ran before, and
    /// the heap must pop in key order.
    #[test]
    fn scratch_reuse_and_heap_do_not_change_results() {
        use crate::legality::{ClassLegality, FREE};
        use ferroute_board::LayerId;
        use ferroute_geom::{Point, Rect};
        let grid = Grid::covering(
            Rect::new(Point::new(0, 0), Point::new(30_000, 30_000)),
            1_000,
            vec![LayerId(0), LayerId(1)],
        );
        let len = grid.len();
        let leg = ClassLegality {
            node: vec![FREE; len],
            edge: vec![FREE; len * 4],
            via: vec![FREE; grid.plane()],
        };
        let zeros16 = vec![0u16; len];
        let zeros_plane = vec![0u16; grid.plane()];
        let hist = vec![0.0f32; len];
        let (k, rv, rh) = (ball(1), ball(1), ball(1));
        let none_nodes = rustc_hash::FxHashSet::default();
        let none_edges = rustc_hash::FxHashSet::default();
        let ctx = |region: IBox| Ctx {
            grid: &grid,
            leg: &leg,
            occ: &zeros16,
            nb: &zeros16,
            sextra: &zeros16,
            own_nodes: &none_nodes,
            own_edges: &none_edges,
            hole_occ: &zeros_plane,
            hist: &hist,
            pres_fac: 1.0,
            costs: base(),
            net: NetId(3),
            region,
            k_offs: &k,
            rv_offs: &rv,
            rh_offs: &rh,
        };
        let (a, b) = (grid.idx(2, 3, 0), grid.idx(20, 11, 1));
        let src = [Endpoint { node: a, heading: None, forbid: 0, cost: 0.0 }];
        let target = move |n: NodeIdx| (n == b).then_some(TargetInfo { heading: None, cost: 0.0 });
        let tbox = (20, 11, 20, 11);
        let first = astar(&ctx((0, 0, 30, 30)), &src, &target, tbox).expect("path");
        // A bigger and a smaller search in between leave stale records behind.
        let other = Endpoint { node: grid.idx(1, 1, 1), heading: None, forbid: 0, cost: 0.0 };
        let far = grid.idx(29, 29, 0);
        let tfar = move |n: NodeIdx| (n == far).then_some(TargetInfo { heading: None, cost: 0.0 });
        assert!(astar(&ctx((0, 0, 30, 30)), &[other], &tfar, (29, 29, 29, 29)).is_some());
        assert!(astar(&ctx((0, 0, 8, 8)), &[other], &tfar, (29, 29, 29, 29)).is_none());
        let again = astar(&ctx((0, 0, 30, 30)), &src, &target, tbox).expect("path");
        assert_eq!(first, again);
        assert_eq!((first[0], first[first.len() - 1]), (a, b));

        let mut h = MinHeap::default();
        let mut x = 0x9E37_79B9_7F4A_7C15u64;
        let mut keys = Vec::new();
        for _ in 0..2000 {
            x = x.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            keys.push(x >> 20);
            h.push(x >> 20);
            if keys.len() % 3 == 0 {
                keys.sort_unstable();
                assert_eq!(h.pop(), Some(keys.remove(0)));
            }
        }
        keys.sort_unstable();
        for k in keys {
            assert_eq!(h.pop(), Some(k));
        }
        assert_eq!(h.pop(), None);
    }
}
