// SPDX-License-Identifier: AGPL-3.0-only
//! # ferroute-router
//!
//! Negotiated-congestion (PathFinder, McMurchie & Ebeling 1995) router on a multi-layer
//! **octilinear** lattice, with:
//!
//! * exact per-class legality against fixed obstacles ([`legality`]),
//! * a reservation plan that makes inter-net clearance a *theorem* of the lattice
//!   ([`plan`]),
//! * pin access with exact stubs for off-grid pads ([`access`]),
//! * deterministic parallel batches whose result equals sequential routing
//!   ([`negotiate`]),
//! * and a final gate through the independent DRC oracle (`ferroute-drc`): copper that
//!   would introduce a violation is never emitted.

pub mod access;
pub mod direct;
pub mod emit;
pub mod grid;
pub mod junction;
pub mod legality;
pub mod meander;
pub mod negotiate;
pub mod plan;
pub mod search;
pub mod smooth;
pub mod straighten;
pub mod tune;

use access::{Stub, Terminal, pad_terminals};
use ferroute_board::{Board, BoardEdit, LayerSet, NetId, PadId};
use ferroute_drc::{BucketGrid, DrcConfig, ItemRef, connectivity::UnionFind, copper_items};
use ferroute_geom::{Coord, Rect};
use grid::Grid;
use legality::{AccelField, ClassLegality, Obstacles};
use negotiate::{NegotiationConfig, NegotiationStats, NetJob, negotiate};
use plan::{Plan, default_pitch};
use search::Costs;
use serde::{Deserialize, Serialize};
use std::time::Instant;

/// Which accelerator computes the obstacle distance field used to skip far-away sites.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Accel {
    /// Exact checks everywhere (reference behaviour).
    None,
    /// GPU if available, else multi-core CPU field.
    Auto,
    Cpu,
    Gpu,
}

/// Router configuration.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RouterConfig {
    /// Lattice pitch (default: default-class track width + clearance).
    pub pitch: Option<Coord>,
    /// Restrict routing to these copper layers (default: all routable layers).
    pub layers: Option<LayerSet>,
    pub max_iterations: u32,
    pub via_cost: f32,
    pub bend45_cost: f32,
    pub bend90_cost: f32,
    /// Extra cost for a bend immediately after a via (keeps vias in line with traces).
    pub via_turn_cost: f32,
    /// Penalty for a pad break-out with a short (< pitch) outward leg.
    pub kink_cost: f32,
    pub pres_fac_first: f32,
    pub pres_fac_mult: f32,
    pub hist_fac: f32,
    /// Initial search-window margin around a net's terminals (lattice steps).
    pub margin: u32,
    /// Seed for deterministic tie-breaking of the net order.
    pub seed: u64,
    pub accel: Accel,
    /// Route only these nets (default: all).
    pub nets: Option<Vec<NetId>>,
}

impl Default for RouterConfig {
    fn default() -> Self {
        Self {
            pitch: None,
            layers: None,
            max_iterations: 50,
            via_cost: 8.0,
            bend45_cost: 0.3,
            bend90_cost: 1.0,
            via_turn_cost: 2.0,
            kink_cost: 20.0,
            pres_fac_first: 0.6,
            pres_fac_mult: 1.6,
            hist_fac: 0.5,
            margin: 8,
            seed: 1,
            accel: Accel::Auto,
            nets: None,
        }
    }
}

/// Routing statistics (deterministic fields only; timings are separate).
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct RouteStats {
    pub pitch: Coord,
    pub grid: (u32, u32, u32),
    pub reservation: Vec<(u32, u32)>,
    pub hole_channel: u32,
    pub nets_attempted: usize,
    pub nets_routed: usize,
    pub nets_failed: usize,
    pub tracks: usize,
    pub vias: usize,
    pub wirelength_nm: f64,
    pub negotiation: NegotiationStats,
    pub accel_backend: String,
    /// Nets removed by the final DRC gate (should always be 0; non-zero indicates a
    /// gap between the lattice theory and the oracle and is reported as a bug).
    pub drc_gate_rips: usize,
}

/// Wall-clock timings (not deterministic; excluded from reproducibility checks).
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Timings {
    pub legality_ms: f64,
    pub negotiation_ms: f64,
    pub gate_ms: f64,
}

/// Router output.
#[derive(Clone, Debug, Default)]
pub struct RouteOutcome {
    pub edit: BoardEdit,
    pub stats: RouteStats,
    pub timings: Timings,
    /// Nets left (partially) unrouted.
    pub failed: Vec<NetId>,
}

/// Router errors.
#[derive(Debug, thiserror::Error)]
pub enum RouteError {
    #[error("invalid board: {0}")]
    InvalidBoard(String),
    #[error("no routable copper layers")]
    NoRoutableLayers,
    #[error("routing lattice too large ({0} nodes); increase the pitch")]
    GridTooLarge(usize),
}

/// SplitMix64 — tiny, well-known, stable mixing function used for all seeded decisions
/// (stable across dependency upgrades, unlike external RNG crates).
#[must_use]
pub const fn splitmix64(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = x;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// Groups each net's pads into islands already connected by existing copper.
#[must_use]
pub fn pad_islands(board: &Board) -> Vec<Vec<Vec<PadId>>> {
    let items = copper_items(board);
    let boxes: Vec<Rect> = items.iter().map(|it| it.bbox.inflate(1)).collect();
    let grid = BucketGrid::new(&boxes, 2_000_000);
    let mut uf = UnionFind::new(items.len());
    for (i, j) in grid.pairs(&boxes) {
        let (a, b) = (&items[i as usize], &items[j as usize]);
        if a.net == b.net
            && a.net != NetId::NONE
            && a.layers.intersects(b.layers)
            && a.hull.clearance_violated(&b.hull, 1)
        {
            uf.union(i, j);
        }
    }
    let mut out: Vec<Vec<Vec<PadId>>> = vec![Vec::new(); board.nets.len()];
    let mut root_of: Vec<Vec<u32>> = vec![Vec::new(); board.nets.len()];
    for (k, it) in items.iter().enumerate() {
        let ItemRef::Pad(p) = it.item else { continue };
        if it.net == NetId::NONE {
            continue;
        }
        let r = uf.find(u32::try_from(k).unwrap_or(u32::MAX));
        let n = it.net.idx();
        match root_of[n].iter().position(|&x| x == r) {
            Some(pos) => out[n][pos].push(p),
            None => {
                root_of[n].push(r);
                out[n].push(vec![p]);
            }
        }
    }
    out
}

/// Routes `board` (pads at their current placements; existing copper is kept fixed).
///
/// # Errors
/// [`RouteError`] for invalid boards or impossible lattices.
pub fn route(board: &Board, cfg: &RouterConfig) -> Result<RouteOutcome, RouteError> {
    board.validate().map_err(|e| RouteError::InvalidBoard(e.to_string()))?;
    let layers: Vec<_> = (0..board.layer_count())
        .map(ferroute_board::LayerId)
        .filter(|&l| {
            board.copper_layers[l.0 as usize].routable && cfg.layers.is_none_or(|s| s.contains(l))
        })
        .collect();
    if layers.is_empty() {
        return Err(RouteError::NoRoutableLayers);
    }
    let bbox0 =
        board.outline.bbox().ok_or_else(|| RouteError::InvalidBoard("no outline".into()))?;
    let (pitch, origin) = choose_lattice(board, bbox0, cfg.pitch);
    let plan = Plan::new(&board.rules, pitch);
    let bbox = board.outline.bbox().ok_or_else(|| RouteError::InvalidBoard("no outline".into()))?;
    let grid = Grid::covering(Rect::new(origin, bbox.max), pitch, layers);
    if grid.len() > (1 << 26) {
        return Err(RouteError::GridTooLarge(grid.len()));
    }
    let t0 = Instant::now();
    // Direct pad-to-pad links first; they become fixed copper of their nets.
    let links = direct::direct_links(
        board,
        &Obstacles::from_board(board),
        &plan::class_geoms(&board.rules),
        &grid.layers,
    );
    let original = board;
    let mut work = board.clone();
    work.tracks.extend(links.iter().map(|l| ferroute_board::Track {
        origin: ferroute_board::Origin::Existing,
        ..l.track.clone()
    }));
    let board = &work;
    let taken: Vec<(PadId, ferroute_geom::Dir8)> = links.iter().flat_map(|l| l.ends).collect();
    let obs = Obstacles::from_board(board);
    let (accel, accel_name) = build_accel(&grid, &obs, &plan, cfg.accel);
    let legal: Vec<ClassLegality> = plan
        .classes
        .iter()
        .map(|cp| legality::compute(&grid, &obs, &cp.geom, accel.as_ref()))
        .collect();
    // Routes never run across pads: pads are entered only through their stubs.
    let interior = access::pad_interior_nodes(board, &grid);
    let mut legal = legal;
    for t in &mut legal {
        for &n in &interior {
            t.node[n as usize] = legality::BLOCKED;
        }
    }
    let legality_ms = t0.elapsed().as_secs_f64() * 1e3;

    // Jobs.
    let base_costs = Costs {
        via: cfg.via_cost,
        bend45: cfg.bend45_cost,
        bend90: cfg.bend90_cost,
        via_turn: cfg.via_turn_cost,
        kink: cfg.kink_cost,
    };
    let islands = pad_islands(board);
    let mut stubs: Vec<Stub> = Vec::new();
    let mut jobs: Vec<NetJob> = Vec::new();
    let mut failed: Vec<NetId> = Vec::new();
    for net in board.net_ids().skip(1) {
        if cfg.nets.as_ref().is_some_and(|v| !v.contains(&net)) {
            continue;
        }
        let groups = &islands[net.idx()];
        if groups.len() < 2 {
            continue;
        }
        let class = usize::from(board.nets[net.idx()].class).min(plan.classes.len() - 1);
        let cp = &plan.classes[class];
        let mut term_groups: Vec<Vec<Terminal>> = Vec::with_capacity(groups.len());
        for g in groups {
            let mut ts = Vec::new();
            for &pad in g {
                let first = stubs.len();
                ts.extend(pad_terminals(
                    board,
                    &grid,
                    &obs,
                    &legal[class],
                    cp,
                    &plan,
                    net,
                    pad,
                    &mut stubs,
                ));
                // A direction already used by a direct link is not a break-out any more.
                ts.retain(|t| {
                    t.stub.is_none_or(|sid| {
                        (sid as usize) < first || !taken.contains(&(pad, stubs[sid as usize].dir))
                    })
                });
            }
            term_groups.push(ts);
        }
        if term_groups.iter().any(Vec::is_empty) {
            failed.push(net);
            continue;
        }
        let tbox = term_groups
            .iter()
            .flatten()
            .map(|t| {
                let (i, j, _) = grid.coords(t.node);
                (i, j, i, j)
            })
            .reduce(|a, b| (a.0.min(b.0), a.1.min(b.1), a.2.max(b.2), a.3.max(b.3)))
            .unwrap_or((0, 0, 0, 0));
        let costs = search::net_costs(&base_costs, &board.nets[net.idx()].constraints);
        jobs.push(NetJob { net, class, islands: term_groups, tbox, costs });
    }
    let mut order: Vec<usize> = (0..jobs.len()).collect();
    order.sort_by_key(|&j| {
        let b = jobs[j].tbox;
        let hpwl = (b.2 - b.0) + (b.3 - b.1);
        (hpwl, splitmix64(cfg.seed ^ u64::from(jobs[j].net.0)))
    });
    let ncfg = NegotiationConfig {
        max_iterations: cfg.max_iterations,
        pres_fac_first: cfg.pres_fac_first,
        pres_fac_mult: cfg.pres_fac_mult,
        hist_fac: cfg.hist_fac,
        costs: base_costs,
        margin: cfg.margin,
    };
    let t1 = Instant::now();
    let (routes, nstats) = negotiate(
        &grid,
        &legal,
        &plan,
        &jobs,
        &stubs,
        &access::stub_conflicts(&stubs),
        &ncfg,
        &order,
    );
    let negotiation_ms = t1.elapsed().as_secs_f64() * 1e3;

    // Emit + final DRC gate.
    let t2 = Instant::now();
    let mut per_net: Vec<(NetId, Vec<ferroute_board::Track>, Vec<ferroute_board::Via>, bool)> =
        jobs.iter()
            .zip(&routes)
            .map(|(job, r)| {
                let (t, v) =
                    emit::emit(board, &grid, &stubs, job.net, &plan.classes[job.class].geom, r);
                (job.net, t, v, r.complete)
            })
            .collect();
    let mut gate_rips = 0usize;
    loop {
        let edit = assemble(&per_net, &links);
        let mut after = original.clone();
        after.apply(&edit);
        let rep = ferroute_drc::check(
            &after,
            &DrcConfig { courtyards: false, connectivity: false, ..DrcConfig::default() },
        );
        let mut bad: Vec<NetId> = Vec::new();
        for v in rep.introduced() {
            for item in std::iter::once(v.a).chain(v.b) {
                match item {
                    ItemRef::Track(t) if t as usize >= original.tracks.len() => {
                        bad.push(after.tracks[t as usize].net);
                    }
                    ItemRef::Arc(a) if a as usize >= original.arcs.len() => {
                        bad.push(after.arcs[a as usize].net);
                    }
                    ItemRef::Via(t) if t as usize >= original.vias.len() => {
                        bad.push(after.vias[t as usize].net);
                    }
                    _ => {}
                }
            }
        }
        bad.sort_unstable();
        bad.dedup();
        if bad.is_empty() {
            break;
        }
        tracing::error!(?bad, "DRC gate removed routed nets (router/oracle disagreement)");
        for e in &mut per_net {
            if bad.contains(&e.0) {
                e.1.clear();
                e.2.clear();
                e.3 = false;
                gate_rips += 1;
            }
        }
    }
    // Gridless clean-up of the emitted copper (guarded by the oracle inside).
    let edit = straighten::straighten(original, &assemble(&per_net, &links));
    let gate_ms = t2.elapsed().as_secs_f64() * 1e3;
    failed.extend(per_net.iter().filter(|e| !e.3).map(|e| e.0));
    failed.sort_unstable();
    failed.dedup();
    let stats = RouteStats {
        pitch,
        grid: (grid.nx, grid.ny, grid.nl()),
        reservation: plan.classes.iter().map(|c| (c.k, c.rv)).collect(),
        hole_channel: plan.rh,
        nets_attempted: jobs.len(),
        nets_routed: per_net.iter().filter(|e| e.3).count(),
        nets_failed: failed.len(),
        tracks: edit.new_tracks.len(),
        vias: edit.new_vias.len(),
        wirelength_nm: edit.new_tracks.iter().map(|t| t.start.dist(t.end)).sum::<f64>() + 0.0,
        negotiation: nstats,
        accel_backend: accel_name,
        drc_gate_rips: gate_rips,
    };
    Ok(RouteOutcome {
        edit,
        stats,
        timings: Timings { legality_ms, negotiation_ms, gate_ms },
        failed,
    })
}

/// Chooses the lattice pitch and origin.
///
/// Straight pad approaches need pad *axes* on lattice lines, so the origin is chosen per
/// axis: the x offset that puts the most pad centres on lattice columns and, independently,
/// the y offset that puts the most on lattice rows. Unless `pitch` is given, pitches from
/// the minimum (default-class width + clearance) up to +15 % in 0.05 mm steps are tried
/// and the one aligning the most pad axes wins (ties → the smaller pitch, i.e. more
/// routing capacity). Deterministic.
#[must_use]
pub fn choose_lattice(
    board: &Board,
    bbox: Rect,
    pitch: Option<Coord>,
) -> (Coord, ferroute_geom::Point) {
    const STEP: Coord = 50_000;
    let centres: Vec<ferroute_geom::Point> = board
        .pad_ids()
        .filter(|&id| !board.pad(id).layers.is_empty() && board.pad(id).net != NetId::NONE)
        .map(|id| board.pad_position(id))
        .collect();
    let best_offset = |vals: &mut dyn Iterator<Item = Coord>, min: Coord, p: Coord| {
        let mut counts: std::collections::BTreeMap<Coord, u32> = std::collections::BTreeMap::new();
        for v in vals {
            *counts.entry((v - min).rem_euclid(p)).or_insert(0) += 1;
        }
        counts
            .iter()
            .max_by(|a, b| a.1.cmp(b.1).then(b.0.cmp(a.0)))
            .map_or((0, 0), |(&r, &c)| (r, c))
    };
    let evaluate = |p: Coord| {
        let (rx, sx) = best_offset(&mut centres.iter().map(|c| c.x), bbox.min.x, p);
        let (ry, sy) = best_offset(&mut centres.iter().map(|c| c.y), bbox.min.y, p);
        // Largest origin ≤ bbox.min congruent to the chosen residues.
        let ox = if rx == 0 { bbox.min.x } else { bbox.min.x + rx - p };
        let oy = if ry == 0 { bbox.min.y } else { bbox.min.y + ry - p };
        (sx + sy, ferroute_geom::Point::new(ox, oy))
    };
    let candidates: Vec<Coord> = match pitch {
        Some(p) => vec![p],
        None => {
            let p0 = default_pitch(&board.rules);
            let first = (p0 + STEP - 1) / STEP * STEP;
            std::iter::once(p0)
                .chain((first..=p0 + p0 * 15 / 100).step_by(50_000).filter(|&p| p > p0))
                .collect()
        }
    };
    let mut best: Option<(u32, Coord, ferroute_geom::Point)> = None;
    for p in candidates {
        let (score, origin) = evaluate(p);
        if best.is_none_or(|b| score > b.0) {
            best = Some((score, p, origin));
        }
    }
    best.map_or((default_pitch(&board.rules), bbox.min), |b| (b.1, b.2))
}

type NetCopper = (NetId, Vec<ferroute_board::Track>, Vec<ferroute_board::Via>, bool);

fn assemble(per_net: &[NetCopper], links: &[direct::Link]) -> BoardEdit {
    BoardEdit {
        new_arcs: Vec::new(),
        moves: Vec::new(),
        new_tracks: links
            .iter()
            .map(|l| l.track.clone())
            .chain(per_net.iter().flat_map(|e| e.1.iter().cloned()))
            .collect(),
        new_vias: per_net.iter().flat_map(|e| e.2.iter().cloned()).collect(),
    }
}

fn build_accel(
    grid: &Grid,
    obs: &Obstacles,
    plan: &Plan,
    pref: Accel,
) -> (Option<AccelField>, String) {
    let bp = match pref {
        Accel::None => return (None, "exact".into()),
        Accel::Auto => ferroute_gpu::BackendPreference::Auto,
        Accel::Cpu => ferroute_gpu::BackendPreference::Cpu,
        Accel::Gpu => ferroute_gpu::BackendPreference::Gpu,
    };
    let Ok(backend) = ferroute_gpu::select_backend(bp) else {
        return (None, "exact (accelerator unavailable)".into());
    };
    let reach = obs.reach
        + plan.classes.iter().map(|c| c.geom.track_radius.max(c.geom.clearance)).max().unwrap_or(0)
        + 2 * plan.pitch;
    match AccelField::compute(grid, obs, backend.as_ref(), reach.max(1)) {
        Ok(f) => {
            let name = f.backend.to_owned();
            (Some(f), name)
        }
        Err(e) => (None, format!("exact (accelerator error: {e})")),
    }
}

#[cfg(test)]
mod tests;
