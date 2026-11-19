// SPDX-License-Identifier: AGPL-3.0-only
//! PathFinder negotiated-congestion loop with deterministic parallel batches.
//!
//! Nets are routed in a fixed priority order. Consecutive nets whose *footprint regions*
//! (search window ∪ previous route, both inflated by the largest reservation radius) are
//! pairwise disjoint form a batch and are searched in parallel against the occupancy
//! snapshot at the start of the batch; results are committed in priority order. Because
//! batch members can neither read nor write each other's lattice cells, the outcome is
//! **identical to fully sequential routing** for any thread count (INV-DET-1).

use crate::access::{Stub, Terminal};
use crate::grid::{Grid, NodeIdx};
use crate::legality::ClassLegality;
use crate::plan::Plan;
use crate::search::{Costs, Ctx, IBox, astar, ball};
use ferroute_board::NetId;
use rayon::prelude::*;
use rustc_hash::FxHashMap;

/// One net to route.
#[derive(Clone, Debug)]
pub struct NetJob {
    pub net: NetId,
    /// Index into `Plan::classes` / legality tables.
    pub class: usize,
    /// Terminal groups; each island is already internally connected.
    pub islands: Vec<Vec<Terminal>>,
    /// Planar bounding box of all terminals.
    pub tbox: IBox,
}

/// The lattice route of one net.
#[derive(Clone, Debug, Default)]
pub struct NetRoute {
    pub paths: Vec<Vec<NodeIdx>>,
    pub stubs: Vec<u32>,
    /// Deduplicated base nodes (actual copper sites incl. diagonal side nodes).
    pub base: Vec<NodeIdx>,
    /// Deduplicated ring nodes (base nodes dilated by their reservation radius, plus
    /// stub reservations). `base ⊆ ring`.
    pub reserved: Vec<NodeIdx>,
    /// Deduplicated via sites (planar).
    pub vias: Vec<u32>,
    pub complete: bool,
    /// Planar bbox of reserved nodes.
    pub bbox: Option<IBox>,
}

/// Negotiation parameters.
#[derive(Clone, Debug)]
pub struct NegotiationConfig {
    pub max_iterations: u32,
    pub pres_fac_first: f32,
    pub pres_fac_mult: f32,
    pub hist_fac: f32,
    pub costs: Costs,
    pub margin: u32,
}

/// Negotiation statistics.
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct NegotiationStats {
    pub iterations: u32,
    pub batches: u32,
    pub max_batch: u32,
    pub searches: u64,
    pub overused_after_negotiation: usize,
    pub ripped_for_legality: usize,
    /// Up to 32 lattice sites `(i, j, layer)` still overused after negotiation.
    pub overused_sites: Vec<(u32, u32, u32)>,
    /// Nets whose route was shortened / straightened by the smoothing pass.
    pub smoothed_nets: usize,
}

struct State {
    /// Routes currently using each stub (0/1).
    stub_used: Vec<u16>,
    /// Per node: in-use stubs of other nets conflicting with a stub ending at the node.
    sextra: Vec<u16>,
    sconf: Vec<Vec<u32>>,
    snode: Vec<NodeIdx>,
    /// Number of nets whose base set contains the node.
    nb: Vec<u16>,
    /// Number of nets whose ring set contains the node.
    occ: Vec<u16>,
    hole_occ: Vec<u16>,
    hist: Vec<f32>,
}

fn union_box(a: IBox, b: IBox) -> IBox {
    (a.0.min(b.0), a.1.min(b.1), a.2.max(b.2), a.3.max(b.3))
}

fn inflate(b: IBox, r: u32, g: &Grid) -> IBox {
    (b.0.saturating_sub(r), b.1.saturating_sub(r), (b.2 + r).min(g.nx - 1), (b.3 + r).min(g.ny - 1))
}

fn disjoint(a: IBox, b: IBox) -> bool {
    a.2 < b.0 || b.2 < a.0 || a.3 < b.1 || b.3 < a.1
}

/// Computes reserved nodes / via sites of a route.
fn reserve(grid: &Grid, plan: &Plan, class: usize, stubs: &[Stub], r: &mut NetRoute) {
    let cp = &plan.classes[class];
    let k_offs = ball(cp.k);
    let rv_offs = ball(cp.rv);
    let mut set: Vec<NodeIdx> = Vec::new();
    let mut base: Vec<NodeIdx> = Vec::new();
    let mut vias: Vec<u32> = Vec::new();
    let push_ball = |set: &mut Vec<NodeIdx>, i: u32, j: u32, l: u32, offs: &[(i32, i32)]| {
        for &(di, dj) in offs {
            let (x, y) = (i64::from(i) + i64::from(di), i64::from(j) + i64::from(dj));
            if x >= 0 && y >= 0 && x < i64::from(grid.nx) && y < i64::from(grid.ny) {
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                set.push(grid.idx(x as u32, y as u32, l));
            }
        }
    };
    for path in &r.paths {
        for (k, &n) in path.iter().enumerate() {
            let (i, j, l) = grid.coords(n);
            base.push(n);
            push_ball(&mut set, i, j, l, &k_offs);
            if let Some(&m) = path.get(k + 1) {
                let (mi, mj, ml) = grid.coords(m);
                if ml != l {
                    vias.push(grid.planar(n));
                } else if mi != i && mj != j {
                    base.push(grid.idx(mi, j, l));
                    base.push(grid.idx(i, mj, l));
                    push_ball(&mut set, mi, j, l, &k_offs);
                    push_ball(&mut set, i, mj, l, &k_offs);
                }
            }
        }
    }
    vias.sort_unstable();
    vias.dedup();
    for &v in &vias {
        let (i, j) = (v % grid.nx, v / grid.nx);
        for l in 0..grid.nl() {
            base.push(grid.idx(i, j, l));
            push_ball(&mut set, i, j, l, &rv_offs);
        }
    }
    for &s in &r.stubs {
        set.extend_from_slice(&stubs[s as usize].reserved);
    }
    set.sort_unstable();
    set.dedup();
    base.sort_unstable();
    base.dedup();
    r.base = base;
    r.bbox = set.iter().fold(None, |acc: Option<IBox>, &n| {
        let (i, j, _) = grid.coords(n);
        Some(acc.map_or((i, j, i, j), |b| union_box(b, (i, j, i, j))))
    });
    r.reserved = set;
    r.vias = vias;
}

fn commit(st: &mut State, grid: &Grid, plan: &Plan, r: &NetRoute, sign: i32) {
    for &s in &r.stubs {
        let u = &mut st.stub_used[s as usize];
        *u = if sign > 0 { u.saturating_add(1) } else { u.saturating_sub(1) };
        for k in 0..st.sconf[s as usize].len() {
            let t = st.sconf[s as usize][k];
            let n = st.snode[t as usize] as usize;
            st.sextra[n] = if sign > 0 {
                st.sextra[n].saturating_add(1)
            } else {
                st.sextra[n].saturating_sub(1)
            };
        }
    }
    for &n in &r.base {
        let o = &mut st.nb[n as usize];
        *o = if sign > 0 { o.saturating_add(1) } else { o.saturating_sub(1) };
    }
    for &n in &r.reserved {
        let o = &mut st.occ[n as usize];
        *o = if sign > 0 { o.saturating_add(1) } else { o.saturating_sub(1) };
    }
    let rh = ball(plan.rh);
    for &v in &r.vias {
        let (i, j) = (v % grid.nx, v / grid.nx);
        for &(di, dj) in &rh {
            let (x, y) = (i64::from(i) + i64::from(di), i64::from(j) + i64::from(dj));
            if x >= 0 && y >= 0 && x < i64::from(grid.nx) && y < i64::from(grid.ny) {
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                let o = &mut st.hole_occ[(y as u32 * grid.nx + x as u32) as usize];
                *o = if sign > 0 { o.saturating_add(1) } else { o.saturating_sub(1) };
            }
        }
    }
}

/// A node is overused when some net has a base node there and at least two nets
/// (hence another net) hold it in a ring.
fn overused(st: &State, n: usize) -> bool {
    st.nb[n] >= 1 && st.occ[n] >= 2
}

fn stub_conflict(st: &State, s: u32) -> bool {
    st.sconf[s as usize].iter().any(|&t| st.stub_used[t as usize] > 0)
}

fn conflicts(st: &State, grid: &Grid, plan: &Plan, r: &NetRoute) -> bool {
    if r.stubs.iter().any(|&s| stub_conflict(st, s)) {
        return true;
    }
    if r.reserved.iter().any(|&n| overused(st, n as usize)) {
        return true;
    }
    let rh = ball(plan.rh);
    r.vias.iter().any(|&v| {
        let (i, j) = (v % grid.nx, v / grid.nx);
        rh.iter().any(|&(di, dj)| {
            let (x, y) = (i64::from(i) + i64::from(di), i64::from(j) + i64::from(dj));
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let over = x >= 0
                && y >= 0
                && x < i64::from(grid.nx)
                && y < i64::from(grid.ny)
                && st.hole_occ[(y as u32 * grid.nx + x as u32) as usize] > 1;
            over
        })
    })
}

#[allow(clippy::too_many_arguments)]
fn route_one(
    grid: &Grid,
    legal: &[ClassLegality],
    plan: &Plan,
    stubs: &[Stub],
    st: &State,
    job: &NetJob,
    margin: u32,
    pres_fac: f32,
    cfg: &NegotiationConfig,
) -> (NetRoute, u64) {
    let cp = &plan.classes[job.class];
    let k_offs = ball(cp.k);
    let rv_offs = ball(cp.rv);
    let rh_offs = ball(plan.rh);
    let ctx = Ctx {
        grid,
        leg: &legal[job.class],
        occ: &st.occ,
        nb: &st.nb,
        sextra: &st.sextra,
        hole_occ: &st.hole_occ,
        hist: &st.hist,
        pres_fac,
        costs: cfg.costs,
        net: job.net,
        region: inflate(job.tbox, margin, grid),
        k_offs: &k_offs,
        rv_offs: &rv_offs,
        rh_offs: &rh_offs,
    };
    let mut term_of: FxHashMap<NodeIdx, Vec<(usize, Option<u32>)>> = FxHashMap::default();
    for (isl, ts) in job.islands.iter().enumerate() {
        for t in ts {
            term_of.entry(t.node).or_default().push((isl, t.stub));
        }
    }
    let mut route = NetRoute::default();
    let mut connected = vec![false; job.islands.len()];
    connected[0] = true;
    let mut sources: Vec<NodeIdx> = job.islands[0].iter().map(|t| t.node).collect();
    let mut searches = 0u64;
    let mut used_stubs: Vec<u32> = Vec::new();
    let mark_stub = |n: NodeIdx, isl_ok: &dyn Fn(usize) -> bool, used: &mut Vec<u32>| {
        if let Some(v) = term_of.get(&n) {
            for &(isl, stub) in v {
                if let Some(s) = stub
                    && isl_ok(isl)
                {
                    used.push(s);
                }
            }
        }
    };
    while connected.iter().any(|c| !c) {
        let remaining: Vec<usize> = (0..job.islands.len()).filter(|&i| !connected[i]).collect();
        let tbox = remaining
            .iter()
            .flat_map(|&isl| job.islands[isl].iter())
            .map(|t| {
                let (i, j, _) = grid.coords(t.node);
                (i, j, i, j)
            })
            .reduce(union_box)
            .unwrap_or(job.tbox);
        let is_target =
            |n: NodeIdx| term_of.get(&n).is_some_and(|v| v.iter().any(|&(isl, _)| !connected[isl]));
        searches += 1;
        let Some(path) = astar(&ctx, &sources, &is_target, tbox) else {
            break;
        };
        let (start, end) = (path[0], path[path.len() - 1]);
        let conn_now = connected.clone();
        mark_stub(start, &|isl| conn_now[isl], &mut used_stubs);
        let mut reached = Vec::new();
        if let Some(v) = term_of.get(&end) {
            for &(isl, stub) in v {
                if !connected[isl] {
                    reached.push(isl);
                    if let Some(s) = stub {
                        used_stubs.push(s);
                    }
                }
            }
        }
        // Reaching one terminal connects every island that owns it (shared stub node).
        for isl in reached {
            connected[isl] = true;
            sources.extend(job.islands[isl].iter().map(|t| t.node));
        }
        sources.extend(path.iter().copied());
        route.paths.push(path);
    }
    route.complete = connected.iter().all(|&c| c);
    used_stubs.sort_unstable();
    used_stubs.dedup();
    route.stubs = used_stubs;
    reserve(grid, plan, job.class, stubs, &mut route);
    (route, searches)
}

/// Runs PathFinder. Returns one route per job (empty and `complete = false` for nets
/// that had to be dropped) and statistics. Post-condition (INV-ROUTE-1): no base node of
/// one net lies in a ring of another net, and no hole-channel cell holds two vias.
#[must_use]
#[allow(clippy::too_many_arguments)]
pub fn negotiate(
    grid: &Grid,
    legal: &[ClassLegality],
    plan: &Plan,
    jobs: &[NetJob],
    stubs: &[Stub],
    stub_conflicts: &[Vec<u32>],
    cfg: &NegotiationConfig,
    order: &[usize],
) -> (Vec<NetRoute>, NegotiationStats) {
    let mut st = State {
        stub_used: vec![0; stubs.len()],
        sextra: vec![0; grid.len()],
        sconf: stub_conflicts.to_vec(),
        snode: stubs.iter().map(|s| s.node).collect(),
        nb: vec![0; grid.len()],
        occ: vec![0; grid.len()],
        hole_occ: vec![0; grid.plane()],
        hist: vec![0.0; grid.len()],
    };
    let mut routes: Vec<NetRoute> = vec![NetRoute::default(); jobs.len()];
    let mut margins: Vec<u32> = vec![cfg.margin; jobs.len()];
    let mut attempted: Vec<bool> = vec![false; jobs.len()];
    let mut stats = NegotiationStats::default();
    let rho = plan.max_radius() + 1;
    let full_margin = grid.nx.max(grid.ny);
    let mut pres_fac = cfg.pres_fac_first;
    for iter in 0..cfg.max_iterations {
        let todo: Vec<usize> = order
            .iter()
            .copied()
            .filter(|&j| {
                !attempted[j] || !routes[j].complete || conflicts(&st, grid, plan, &routes[j])
            })
            .filter(|&j| {
                !(attempted[j]
                    && !routes[j].complete
                    && margins[j] > full_margin
                    && !conflicts(&st, grid, plan, &routes[j]))
            })
            .collect();
        if todo.is_empty() {
            break;
        }
        stats.iterations = iter + 1;
        // Greedy order-preserving batches of mutually independent nets.
        let mut k = 0;
        while k < todo.len() {
            let mut batch: Vec<usize> = Vec::new();
            let mut boxes: Vec<IBox> = Vec::new();
            while k < todo.len() {
                let j = todo[k];
                let mut fb = inflate(jobs[j].tbox, margins[j] + rho, grid);
                if let Some(b) = routes[j].bbox {
                    fb = union_box(fb, inflate(b, rho, grid));
                }
                if boxes.iter().all(|&b| disjoint(b, fb)) {
                    batch.push(j);
                    boxes.push(fb);
                    k += 1;
                } else {
                    break;
                }
            }
            stats.batches += 1;
            stats.max_batch = stats.max_batch.max(u32::try_from(batch.len()).unwrap_or(u32::MAX));
            for &j in &batch {
                let old = std::mem::take(&mut routes[j]);
                commit(&mut st, grid, plan, &old, -1);
            }
            let results: Vec<(NetRoute, u64)> = batch
                .par_iter()
                .map(|&j| {
                    route_one(grid, legal, plan, stubs, &st, &jobs[j], margins[j], pres_fac, cfg)
                })
                .collect();
            for (&j, (r, s)) in batch.iter().zip(results) {
                stats.searches += s;
                commit(&mut st, grid, plan, &r, 1);
                attempted[j] = true;
                if !r.complete {
                    margins[j] = margins[j].saturating_mul(3).max(margins[j] + 8);
                }
                routes[j] = r;
            }
        }
        // History on terminals whose stub conflicts with another net's stub.
        for (sidx, &used) in st.stub_used.clone().iter().enumerate() {
            if used > 0 && stub_conflict(&st, u32::try_from(sidx).unwrap_or(u32::MAX)) {
                let n = st.snode[sidx] as usize;
                st.hist[n] += cfg.hist_fac;
            }
        }
        // History update on overused resources.
        for n in 0..st.occ.len() {
            if overused(&st, n) {
                st.hist[n] += cfg.hist_fac * f32::from(st.occ[n] - 1);
            }
        }
        let plane = grid.plane();
        for (p, &o) in st.hole_occ.iter().enumerate() {
            if o > 1 {
                for l in 0..grid.layers.len() {
                    st.hist[l * plane + p] += cfg.hist_fac * f32::from(o - 1);
                }
            }
        }
        pres_fac *= cfg.pres_fac_mult;
    }
    stats.overused_after_negotiation = (0..st.occ.len()).filter(|&n| overused(&st, n)).count()
        + (0..st.stub_used.len())
            .filter(|&k| {
                st.stub_used[k] > 0 && stub_conflict(&st, u32::try_from(k).unwrap_or(u32::MAX))
            })
            .count()
        + st.hole_occ.iter().filter(|&&o| o > 1).count();
    stats.overused_sites = (0..st.occ.len())
        .filter(|&n| overused(&st, n))
        .take(32)
        .map(|n| grid.coords(u32::try_from(n).unwrap_or(u32::MAX)))
        .collect();
    // Legalisation: drop lowest-priority conflicting nets until no resource is shared.
    for &j in order.iter().rev() {
        if !routes[j].paths.is_empty() && conflicts(&st, grid, plan, &routes[j]) {
            let old = std::mem::take(&mut routes[j]);
            commit(&mut st, grid, plan, &old, -1);
            stats.ripped_for_legality += 1;
        }
    }
    // Smoothing: straighten each route on the lattice; keep only conflict-free results.
    for _pass in 0..2 {
        for &j in order {
            if routes[j].paths.is_empty() {
                continue;
            }
            let job = &jobs[j];
            let pins: rustc_hash::FxHashSet<NodeIdx> =
                routes[j].paths.iter().flat_map(|p| [p[0], p[p.len() - 1]]).collect();
            let mut cand = routes[j].clone();
            cand.paths = routes[j]
                .paths
                .iter()
                .map(|p| {
                    crate::smooth::smooth_path(
                        grid,
                        &legal[job.class],
                        job.net,
                        p,
                        &|n| pins.contains(&n),
                        &cfg.costs,
                    )
                })
                .collect();
            if cand.paths == routes[j].paths {
                continue;
            }
            reserve(grid, plan, job.class, stubs, &mut cand);
            commit(&mut st, grid, plan, &routes[j], -1);
            commit(&mut st, grid, plan, &cand, 1);
            if conflicts(&st, grid, plan, &cand) {
                commit(&mut st, grid, plan, &cand, -1);
                commit(&mut st, grid, plan, &routes[j], 1);
            } else {
                routes[j] = cand;
                stats.smoothed_nets += 1;
            }
        }
    }
    debug_assert!(
        (0..st.occ.len()).all(|n| !overused(&st, n)) && st.hole_occ.iter().all(|&o| o <= 1)
    );
    (routes, stats)
}
