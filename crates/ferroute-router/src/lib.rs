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
pub mod chamfer;
pub mod detour;
pub mod diffpair;
pub mod direct;
pub mod emit;
pub mod escape;
pub mod grid;
pub mod junction;
pub mod legality;
pub mod lns;
pub mod meander;
pub mod negotiate;
pub mod pairgeom;
pub mod plan;
pub mod plane;
pub mod search;
pub mod smooth;
pub mod straighten;
pub mod tune;
pub mod vias;
pub mod vspan;

use access::{Stub, Terminal, pad_terminals};
use ferroute_board::{Board, BoardEdit, LayerSet, NetId, PadId};
use ferroute_drc::{BucketGrid, DrcConfig, ItemRef, connectivity::UnionFind, copper_items};
use ferroute_geom::{Coord, Rect};
use grid::Grid;
use legality::{AccelField, ClassLegality, Obstacles, allows};
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
    /// Nets routed first, in this order, before the usual HPWL+seed order.
    #[serde(default)]
    pub first: Vec<NetId>,
    /// Maximum number of detour-driven re-ordering rounds after the first pass.
    #[serde(default = "default_order_rounds")]
    pub order_rounds: u32,
    /// Allow vias inside same-net SMD pads (opt-in: such vias must be filled and capped,
    /// IPC-4761 type VII, an extra fabrication cost). Pins gain a via-in-pad break-out
    /// (priced like any via) and routed vias may be relocated into the pad centre.
    /// Off by default.
    #[serde(default)]
    pub via_in_pad: bool,
    /// Via kinds the router may create (default: through vias only). Blind, buried and
    /// micro vias occupy only the layers they span, which frees routing space on the
    /// others, but need HDI fabrication (extra cost).
    #[serde(default)]
    pub via_policy: ferroute_board::ViaPolicy,
    /// Cost of a micro via relative to a through via.
    #[serde(default = "default_micro_factor")]
    pub micro_via_factor: f32,
    /// Cost of a blind or buried via relative to a through via.
    #[serde(default = "default_span_factor")]
    pub span_via_factor: f32,
    /// Coordinated rip-up & re-route of small net neighbourhoods after the first passes.
    #[serde(default)]
    pub lns: lns::LnsConfig,
    /// Plan the break-out of area arrays (BGA/LGA) before lattice routing ([`escape`]).
    /// Boards without an area array are unaffected.
    #[serde(default = "default_escape")]
    pub escape: bool,
    /// Outermost array ring that gets a planned escape (`0` = outer ring only).
    #[serde(default = "default_escape_rings")]
    pub escape_rings: usize,
}

const fn default_escape_rings() -> usize {
    escape::DEFAULT_MAX_RING
}

const fn default_escape() -> bool {
    true
}

const fn default_order_rounds() -> u32 {
    3
}

const fn default_micro_factor() -> f32 {
    vspan::MICRO_FACTOR
}

const fn default_span_factor() -> f32 {
    vspan::SPAN_FACTOR
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
            first: Vec::new(),
            order_rounds: default_order_rounds(),
            via_in_pad: false,
            via_policy: ferroute_board::ViaPolicy::default(),
            micro_via_factor: vspan::MICRO_FACTOR,
            span_via_factor: vspan::SPAN_FACTOR,
            lns: lns::LnsConfig::default(),
            escape: default_escape(),
            escape_rings: default_escape_rings(),
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
    /// Of those vias, the blind / buried / micro ones (HDI fabrication, extra cost).
    #[serde(default)]
    pub blind_vias: usize,
    #[serde(default)]
    pub buried_vias: usize,
    #[serde(default)]
    pub micro_vias: usize,
    /// Of those vias, the ones inside an SMD pad (need filling and capping).
    #[serde(default)]
    pub via_in_pad: usize,
    /// Pad-to-pad connections of nets with a pour whose plane was unreachable.
    #[serde(default)]
    pub plane_fallback_links: usize,
    /// 90° bends left in the emitted copper after the chamfer pass.
    pub right_angle_bends: usize,
    /// Avoidable bends next to same-net pads left in the emitted copper.
    pub near_pad_bends: usize,
    /// Same-net copper pairs closer than the clearance without touching (slivers).
    pub slivers: usize,
    /// Vias at which traces jog (bend next to the via or a bent layer change).
    pub via_jogs: usize,
    pub wirelength_nm: f64,
    pub negotiation: NegotiationStats,
    pub accel_backend: String,
    /// Nets removed by the final DRC gate (should always be 0; non-zero indicates a
    /// gap between the lattice theory and the oracle and is reported as a bug).
    pub drc_gate_rips: usize,
    /// Detour-driven re-ordering rounds that were run (0 = first pass was kept as is).
    pub order_rounds_run: u32,
    /// Balls of area arrays that received planned escape copper ([`escape`]).
    #[serde(default)]
    pub escape_balls: usize,
    /// Dog-bone vias placed by the escape stage.
    #[serde(default)]
    pub escape_vias: usize,
    /// Large-neighbourhood re-route attempts that were run.
    #[serde(default)]
    pub lns_rounds_run: u32,
    /// Of those, the ones that improved the result and were kept.
    #[serde(default)]
    pub lns_accepted: u32,
    /// Sum of detour excess (mm) of the returned outcome.
    pub detour_excess_mm: f64,
    /// Differential pairs routed as coupled units (`P/N` names).
    #[serde(default)]
    pub coupled_pairs: Vec<String>,
    /// Pairs the router could not couple and left to the ordinary single-net router
    /// (`P/N: reason`); reported, never hidden.
    #[serde(default)]
    pub uncoupled_pairs: Vec<String>,
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
    pad_islands_with_planes(board).0
}

/// [`pad_islands`] plus, per net and island, whether the island touches a stored fill of
/// its net (a plane): such islands need no via to the pour.
#[must_use]
pub fn pad_islands_with_planes(board: &Board) -> (Vec<Vec<Vec<PadId>>>, Vec<Vec<bool>>) {
    let items = copper_items(board);
    let boxes: Vec<Rect> = items.iter().map(|it| it.bbox.inflate(1)).collect();
    let grid = BucketGrid::new(&boxes, 2_000_000);
    let mut uf = UnionFind::new(items.len() + ferroute_drc::zones::fill_count(board));
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
    // Pads (and copper) of a net joined through its stored pour need no route.
    for (a, b) in ferroute_drc::zones::fill_links(board, &items) {
        uf.union(a, b);
    }
    let mut plane_roots: Vec<Vec<u32>> = vec![Vec::new(); board.nets.len()];
    for (k, f) in ferroute_drc::zones::fill_polys(board).iter().enumerate() {
        if f.net != NetId::NONE {
            let r = uf.find(u32::try_from(items.len() + k).unwrap_or(u32::MAX));
            plane_roots[f.net.idx()].push(r);
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
    let touches = root_of
        .iter()
        .zip(&plane_roots)
        .map(|(roots, pr)| roots.iter().map(|r| pr.contains(r)).collect())
        .collect();
    (out, touches)
}

/// Routes `board` (pads at their current placements; existing copper is kept fixed).
///
/// # Errors
/// [`RouteError`] for invalid boards or impossible lattices.
pub fn route(board: &Board, cfg: &RouterConfig) -> Result<RouteOutcome, RouteError> {
    // Lattice, links and legality do not depend on the net order: built once for all rounds.
    let prep = prepare(board, cfg)?;
    let mut best = route_prepared(&prep, cfg)?;
    let mut best_q = detour::quality(board, &best);
    let mut run = 0u32;
    for _ in 0..cfg.order_rounds {
        let d = detour::net_detours(board, &best.edit);
        let mut first = detour::worst_nets(&d, &best.failed);
        if first.is_empty() {
            break;
        }
        run += 1;
        let extra: Vec<NetId> = cfg.first.iter().copied().filter(|n| !first.contains(n)).collect();
        first.extend(extra);
        let cand = route_prepared(&prep, &RouterConfig { first, ..cfg.clone() })?;
        let q = detour::quality(board, &cand);
        if q.improves_on(&best_q) {
            best = cand;
            best_q = q;
        } else {
            break;
        }
    }
    best.stats.order_rounds_run = run;
    if cfg.lns.rounds > 0 {
        best = lns::improve(board, cfg, best)?;
        best_q = detour::quality(board, &best);
    }
    #[allow(clippy::cast_precision_loss)]
    {
        best.stats.detour_excess_mm = best_q.excess_um as f64 / 1000.0;
    }
    Ok(best)
}

/// The part of a routing pass that depends only on the board and the board-level settings
/// (`pitch`, `layers`, `nets`, `accel`) — not on the net order: lattice, pair stage, direct
/// links and the exact legality tables. Reusable across passes that differ only in order.
#[derive(Debug)]
pub struct Prepared {
    grid: Grid,
    plan: Plan,
    pitch: Coord,
    stage: diffpair::PairStage,
    pair_tracks: Vec<ferroute_board::Track>,
    /// The input board with the pair stage's copper added (`original` of the pass).
    pre: Board,
    /// `pre` plus the direct links.
    work: Board,
    links: Vec<direct::Link>,
    taken: Vec<(PadId, ferroute_geom::Dir8)>,
    /// Planned area-array escapes (already part of `work`).
    escapes: Vec<escape::Escape>,
    obs: Obstacles,
    legal: Vec<ClassLegality>,
    accel_name: String,
    legality_ms: f64,
    /// Via kinds and costs (`via_policy` and the cost factors are board-level settings).
    vm: vspan::ViaModel,
}

/// One routing pass with the net order given by `cfg.first`, then HPWL and seed.
///
/// # Errors
/// [`RouteError`] for invalid boards or impossible lattices.
pub fn route_once(board: &Board, cfg: &RouterConfig) -> Result<RouteOutcome, RouteError> {
    route_prepared(&prepare(board, cfg)?, cfg)
}

/// Builds the order-independent part of a routing pass (see [`Prepared`]).
///
/// # Errors
/// [`RouteError`] for invalid boards or impossible lattices.
pub fn prepare(board: &Board, cfg: &RouterConfig) -> Result<Prepared, RouteError> {
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
    let bbox = board.outline.bbox().ok_or_else(|| RouteError::InvalidBoard("no outline".into()))?;
    let grid = Grid::covering(Rect::new(origin, bbox.max), pitch, layers);
    let vm = vspan::ViaModel::new(
        cfg.via_policy,
        &grid,
        board.copper_layers.len(),
        cfg.micro_via_factor,
        cfg.span_via_factor,
    );
    let plan = Plan::with_micro(&board.rules, pitch, vm.policy.micro);
    if grid.len() > (1 << 26) {
        return Err(RouteError::GridTooLarge(grid.len()));
    }
    let t0 = Instant::now();
    // Differential pairs first, as coupled units; their copper is fixed for everything else.
    let stage = diffpair::route_pairs(board, &grid.layers, pitch, cfg.nets.as_deref());
    let pair_tracks = stage.tracks();
    let pair_nets = stage.nets(board);
    let mut pre = board.clone();
    pre.tracks.extend(
        pair_tracks.iter().map(|t| ferroute_board::Track {
            origin: ferroute_board::Origin::Existing,
            ..t.clone()
        }),
    );
    let board = &pre;
    // Direct pad-to-pad links next; they become fixed copper of their nets.
    let links = direct::direct_links(
        board,
        &Obstacles::from_board(board),
        &plan::class_geoms(&board.rules),
        &grid.layers,
    );
    // Only the requested nets get links (the rest is fixed copper of the caller); coupled
    // pairs are already connected.
    let links: Vec<direct::Link> = links
        .into_iter()
        .filter(|l| cfg.nets.as_ref().is_none_or(|v| v.contains(&l.track.net)))
        .filter(|l| !pair_nets.contains(&l.track.net))
        .collect();
    let mut work = board.clone();
    work.tracks.extend(links.iter().map(|l| ferroute_board::Track {
        origin: ferroute_board::Origin::Existing,
        ..l.track.clone()
    }));
    let taken: Vec<(PadId, ferroute_geom::Dir8)> = links.iter().flat_map(|l| l.ends).collect();
    // Area arrays (BGA/LGA): planned escapes become fixed copper of their nets.
    let mut escapes: Vec<escape::Escape> = Vec::new();
    if cfg.escape {
        let arrays = escape::detect_arrays(&work);
        if !arrays.is_empty() {
            escapes = plan_escapes(&work, &grid, &vm, &arrays, cfg, &pair_nets);
            for e in &escapes {
                work.tracks.extend(e.tracks.iter().map(|t| ferroute_board::Track {
                    origin: ferroute_board::Origin::Existing,
                    ..t.clone()
                }));
                work.vias.extend(e.vias.iter().map(|v| ferroute_board::Via {
                    origin: ferroute_board::Origin::Existing,
                    ..v.clone()
                }));
            }
        }
    }
    let board = &work;
    let obs = Obstacles::from_board(board);
    let (accel, accel_name) = build_accel(&grid, &obs, &plan, cfg.accel);
    let legal: Vec<ClassLegality> = plan
        .classes
        .iter()
        .map(|cp| legality::compute_with(&grid, &obs, &cp.geom, accel.as_ref(), &vm))
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
    Ok(Prepared {
        grid,
        plan,
        pitch,
        stage,
        pair_tracks,
        pre,
        work,
        links,
        taken,
        escapes,
        obs,
        legal,
        accel_name,
        legality_ms,
        vm,
    })
}

/// Routes with the prepared tables; `cfg` must agree with the one given to [`prepare`] on
/// `pitch`, `layers`, `nets` and `accel` (net order, costs and iteration limits are free).
///
/// # Errors
/// [`RouteError`] for invalid boards or impossible lattices.
#[allow(clippy::too_many_lines)]
pub fn route_prepared(prep: &Prepared, cfg: &RouterConfig) -> Result<RouteOutcome, RouteError> {
    let Prepared {
        grid,
        plan,
        pitch,
        stage,
        pair_tracks,
        pre: original,
        work: board,
        taken,
        obs,
        legal,
        accel_name,
        legality_ms,
        vm,
        ..
    } = prep;
    let (pitch, legality_ms) = (*pitch, *legality_ms);
    let board_pairs = &original.diff_pairs;
    let mut links = prep.links.clone();
    let mut escapes = prep.escapes.clone();
    // Jobs.
    let base_costs = Costs {
        via: cfg.via_cost,
        bend45: cfg.bend45_cost,
        bend90: cfg.bend90_cost,
        via_turn: cfg.via_turn_cost,
        kink: cfg.kink_cost,
    };
    let (islands, island_on_plane) = pad_islands_with_planes(board);
    let fills = ferroute_drc::zones::fill_polys(board);
    let mut stubs: Vec<Stub> = Vec::new();
    let mut jobs: Vec<NetJob> = Vec::new();
    let mut failed: Vec<NetId> = Vec::new();
    let mut plane_fallback_links = 0usize;
    for net in board.net_ids().skip(1) {
        if cfg.nets.as_ref().is_some_and(|v| !v.contains(&net)) {
            continue;
        }
        let groups = &islands[net.idx()];
        let class = usize::from(board.nets[net.idx()].class).min(plan.classes.len() - 1);
        let cp = &plan.classes[class];
        let on_plane = &island_on_plane[net.idx()];
        let group_terms = |stubs: &mut Vec<Stub>, g: &[PadId]| -> Vec<Terminal> {
            let mut ts = Vec::new();
            for &pad in g {
                // A ball that is already broken out: the route starts at the escape end.
                if let Some(t) = escape_terminal(grid, &legal[class], net, &escapes, pad) {
                    ts.push(t);
                    continue;
                }
                let first = stubs.len();
                ts.extend(pad_terminals(
                    board,
                    grid,
                    obs,
                    &legal[class],
                    cp,
                    plan,
                    net,
                    pad,
                    stubs,
                ));
                // A direction already used by a direct link is not a break-out any more.
                ts.retain(|t| {
                    t.stub.is_none_or(|sid| {
                        (sid as usize) < first || !taken.contains(&(pad, stubs[sid as usize].dir))
                    })
                });
                if cfg.via_in_pad {
                    ts.extend(access::via_terminals(
                        board,
                        grid,
                        obs,
                        &legal[class],
                        cp,
                        plan,
                        net,
                        pad,
                        stubs,
                    ));
                }
            }
            ts
        };
        let push_job = |jobs: &mut Vec<NetJob>, term_groups: Vec<Vec<Terminal>>, plane| {
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
            let mine: Vec<&escape::Escape> = escapes.iter().filter(|e| e.net == net).collect();
            let (fixed_nodes, fixed_edges) = if mine.is_empty() {
                (Vec::new(), Vec::new())
            } else {
                escape::keepout(grid, &mine, cp.geom.track_radius)
            };
            jobs.push(NetJob {
                net,
                class,
                islands: term_groups,
                tbox,
                costs,
                plane,
                fixed_nodes,
                fixed_edges,
            });
        };
        // Where to look for plane sites: around each pad's centre and, for pads wider than
        // the search window (exposed pads), around their side midpoints, where break-outs
        // of large pads leave.
        let centres_of = |g: &[PadId]| -> Vec<ferroute_geom::Point> {
            let reach = plane::WINDOW * grid.pitch;
            g.iter()
                .flat_map(|&p| {
                    let (c, bb) = (board.pad_position(p), board.pad_hull(p).bbox());
                    let mut v = vec![c];
                    if (bb.max.x - bb.min.x).max(bb.max.y - bb.min.y) / 2 > reach {
                        let (mx, my) = ((bb.min.x + bb.max.x) / 2, (bb.min.y + bb.max.y) / 2);
                        v.extend([
                            ferroute_geom::Point::new(bb.min.x, my),
                            ferroute_geom::Point::new(bb.max.x, my),
                            ferroute_geom::Point::new(mx, bb.min.y),
                            ferroute_geom::Point::new(mx, bb.max.y),
                        ]);
                    }
                    v
                })
                .collect()
        };
        let all_centres = centres_of(&groups.iter().flatten().copied().collect::<Vec<_>>());
        let plane_terms = if groups.is_empty() {
            Vec::new()
        } else {
            plane::plane_terminals(grid, &legal[class], &cp.geom, net, &fills, &all_centres)
        };
        if plane_terms.is_empty() {
            // No pour to reach: the pads of the net connect among themselves.
            if groups.len() < 2 {
                continue;
            }
            let term_groups: Vec<Vec<Terminal>> =
                groups.iter().map(|g| group_terms(&mut stubs, g)).collect();
            if term_groups.iter().any(Vec::is_empty) {
                failed.push(net);
                continue;
            }
            push_job(&mut jobs, term_groups, Vec::new());
            continue;
        }
        // A net with a pour: every pad island that does not touch the fill gets its own
        // dog-bone to the plane (one job each: local negotiation, independent rip-up) and
        // pads are never chained. Only islands that cannot reach the plane at all (nothing
        // to land a via on, or walled in) link to the nearest island that can.
        let mut unreachable: Vec<(usize, Vec<Terminal>)> = Vec::new();
        let mut reachable: Vec<(usize, Vec<Terminal>)> = Vec::new();
        for (gi, g) in groups.iter().enumerate().filter(|&(gi, _)| !on_plane[gi]) {
            let pad_terms = group_terms(&mut stubs, g);
            if pad_terms.is_empty() {
                failed.push(net);
                continue;
            }
            let terms =
                plane::plane_terminals(grid, &legal[class], &cp.geom, net, &fills, &centres_of(g));
            let from: Vec<_> = pad_terms.iter().map(|t| t.node).collect();
            if plane::reaches(grid, &legal[class], net, &from, &terms) {
                reachable.push((gi, pad_terms.clone()));
                push_job(&mut jobs, vec![pad_terms, terms], vec![1]);
            } else {
                unreachable.push((gi, pad_terms));
            }
        }
        // Each unreachable island links to the nearest islands that have a dog-bone (their
        // terminals pooled as alternatives of one target island), or to the plane if none.
        for (gi, terms) in unreachable {
            let anchor = board.pad_position(groups[gi][0]);
            let dist = |gj: usize| {
                groups[gj]
                    .iter()
                    .map(|&p| (board.pad_position(p) - anchor).norm2())
                    .min()
                    .unwrap_or(u128::MAX)
            };
            reachable.sort_by_key(|(gj, _)| (dist(*gj), *gj));
            let pool: Vec<Terminal> =
                reachable.iter().take(8).flat_map(|(_, t)| t.iter().cloned()).collect();
            plane_fallback_links += 1;
            push_job(
                &mut jobs,
                vec![terms, if pool.is_empty() { plane_terms.clone() } else { pool }],
                Vec::new(),
            );
        }
    }
    let mut order: Vec<usize> = (0..jobs.len()).collect();
    order.sort_by_key(|&j| {
        let b = jobs[j].tbox;
        let hpwl = (b.2 - b.0) + (b.3 - b.1);
        // Dog-bones go last: signals claim their channels first, the dog-bones then take
        // whatever interstices stay free and are the first to yield (one connection each).
        let dogbone = !jobs[j].plane.is_empty();
        let rank = cfg.first.iter().position(|&n| n == jobs[j].net).unwrap_or(usize::MAX);
        (rank, dogbone, hpwl, splitmix64(cfg.seed ^ u64::from(jobs[j].net.0)))
    });
    let ncfg = NegotiationConfig {
        max_iterations: cfg.max_iterations,
        pres_fac_first: cfg.pres_fac_first,
        pres_fac_mult: cfg.pres_fac_mult,
        hist_fac: cfg.hist_fac,
        costs: base_costs,
        margin: cfg.margin,
        vm: vm.clone(),
    };
    let t1 = Instant::now();
    let (routes, nstats) =
        negotiate(grid, legal, plan, &jobs, &stubs, &access::stub_conflicts(&stubs), &ncfg, &order);
    let negotiation_ms = t1.elapsed().as_secs_f64() * 1e3;

    // Emit + final DRC gate.
    let t2 = Instant::now();
    let mut per_net: Vec<(NetId, Vec<ferroute_board::Track>, Vec<ferroute_board::Via>, bool)> =
        jobs.iter()
            .zip(&routes)
            .map(|(job, r)| {
                let (t, v) =
                    emit::emit(board, grid, &stubs, job.net, &plan.classes[job.class].geom, r);
                (job.net, t, v, r.complete)
            })
            .collect();
    let mut gate_rips = 0usize;
    let mut link_rips: Vec<NetId> = Vec::new();
    loop {
        let edit = assemble(&per_net, &links, &escapes);
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
        // Only nets of this pass can be ripped (the pair stage's copper is verified apart).
        bad.retain(|n| {
            per_net.iter().any(|e| e.0 == *n)
                || links.iter().any(|l| l.track.net == *n)
                || escapes.iter().any(|e| e.net == *n)
        });
        if bad.is_empty() {
            break;
        }
        tracing::error!(?bad, "DRC gate removed routed nets (router/oracle disagreement)");
        let mut removed = false;
        for e in &mut per_net {
            if bad.contains(&e.0) && (e.3 || !e.1.is_empty() || !e.2.is_empty()) {
                e.1.clear();
                e.2.clear();
                e.3 = false;
                gate_rips += 1;
                removed = true;
            }
        }
        // A net's direct pad-to-pad links are part of its copper too.
        let before = links.len();
        links.retain(|l| {
            let rip = bad.contains(&l.track.net);
            if rip {
                link_rips.push(l.track.net);
            }
            !rip
        });
        removed |= links.len() != before;
        let before = escapes.len();
        escapes.retain(|e| !bad.contains(&e.net));
        removed |= escapes.len() != before;
        if !removed {
            // Nothing left to rip: stop instead of spinning. The pipeline's final oracle gate
            // still refuses any output with an introduced violation.
            tracing::error!(?bad, "DRC gate cannot remove the remaining violations");
            break;
        }
    }
    // Gridless clean-up of the emitted copper (guarded by the oracle inside).
    let edit = straighten::straighten(original, &assemble(&per_net, &links, &escapes));
    // Vias move onto the lines of their traces, then the traces are straightened again.
    let edit = vias::relocate_vias(original, &edit, cfg.via_in_pad);
    let edit = straighten::straighten(original, &edit);
    // No trace bends 90°: every right angle becomes two 45° bends where exactly legal.
    let mut edit = chamfer::chamfer(original, &edit);
    let gate_ms = t2.elapsed().as_secs_f64() * 1e3;
    failed.extend(per_net.iter().filter(|e| !e.3).map(|e| e.0));
    failed.extend(link_rips.iter().copied());
    failed.sort_unstable();
    failed.dedup();
    let (right_angle_bends, near_pad_bends, slivers, via_jogs, via_in_pad) = {
        let mut after = original.clone();
        after.apply(&edit);
        (
            ferroute_drc::quality::right_angles(&after).len(),
            ferroute_drc::quality::near_pad_bends(&after).len(),
            ferroute_drc::quality::slivers(&after).len(),
            ferroute_drc::quality::via_jogs(&after).len(),
            edit.new_vias.iter().filter(|v| after.via_in_pad(v).is_some()).count(),
        )
    };
    // Coupled pairs are emitted exactly as built: the gridless passes above only saw them
    // as fixed obstacles (they are Existing copper of `original`).
    edit.new_tracks.splice(0..0, pair_tracks.iter().cloned());
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
        blind_vias: edit
            .new_vias
            .iter()
            .filter(|v| v.kind == ferroute_board::ViaKind::Blind)
            .count(),
        buried_vias: edit
            .new_vias
            .iter()
            .filter(|v| v.kind == ferroute_board::ViaKind::Buried)
            .count(),
        micro_vias: edit
            .new_vias
            .iter()
            .filter(|v| v.kind == ferroute_board::ViaKind::Micro)
            .count(),
        via_in_pad,
        plane_fallback_links,
        right_angle_bends,
        near_pad_bends,
        slivers,
        via_jogs,
        wirelength_nm: edit.new_tracks.iter().map(|t| t.start.dist(t.end)).sum::<f64>() + 0.0,
        negotiation: nstats,
        accel_backend: accel_name.clone(),
        drc_gate_rips: gate_rips,
        order_rounds_run: 0,
        escape_balls: escapes.len(),
        escape_vias: escapes.iter().map(|e| e.vias.len()).sum(),
        lns_rounds_run: 0,
        lns_accepted: 0,
        detour_excess_mm: 0.0,
        coupled_pairs: stage.coupled.iter().map(|c| board_pairs[c.pair].name(original)).collect(),
        uncoupled_pairs: stage
            .uncoupled
            .iter()
            .map(|(k, why)| format!("{}: {why}", board_pairs[*k].name(original)))
            .collect(),
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

fn assemble(
    per_net: &[NetCopper],
    links: &[direct::Link],
    escapes: &[escape::Escape],
) -> BoardEdit {
    BoardEdit {
        new_arcs: Vec::new(),
        moves: Vec::new(),
        new_tracks: links
            .iter()
            .map(|l| l.track.clone())
            .chain(escapes.iter().flat_map(|e| e.tracks.iter().cloned()))
            .chain(per_net.iter().flat_map(|e| e.1.iter().cloned()))
            .collect(),
        new_vias: escapes
            .iter()
            .flat_map(|e| e.vias.iter().cloned())
            .chain(per_net.iter().flat_map(|e| e.2.iter().cloned()))
            .collect(),
        zone_fills: Vec::new(),
    }
}

/// Terminal at the end of the planned escape of `pad`, when the escape is still usable:
/// its end node must be legal for `net`.
fn escape_terminal(
    grid: &Grid,
    leg: &ClassLegality,
    net: NetId,
    escapes: &[escape::Escape],
    pad: PadId,
) -> Option<access::Terminal> {
    let end = escapes.iter().find(|e| e.pad == pad)?.end;
    let l = grid.grid_layer(end.layer)?;
    let (i, j) = (
        u32::try_from((end.point.x - grid.origin.x) / grid.pitch).ok()?,
        u32::try_from((end.point.y - grid.origin.y) / grid.pitch).ok()?,
    );
    let node = grid.idx(i, j, l);
    allows(leg.node[node as usize], net).then_some(access::Terminal {
        node,
        stub: None,
        heading: Some(end.heading),
    })
}

/// Escape plan for the area `arrays` of `work` (see [`escape`]): lone balls of requested
/// nets without a pour of their own, outside coupled pairs.
fn plan_escapes(
    work: &Board,
    grid: &Grid,
    vm: &vspan::ViaModel,
    arrays: &[escape::Array],
    cfg: &RouterConfig,
    pair_nets: &[NetId],
) -> Vec<escape::Escape> {
    let (islands, on_plane) = pad_islands_with_planes(work);
    let pours: Vec<NetId> = ferroute_drc::zones::fill_polys(work).iter().map(|f| f.net).collect();
    let mut lone = vec![false; work.pads.len()];
    for (n, groups) in islands.iter().enumerate() {
        for (g, grp) in groups.iter().enumerate() {
            if grp.len() == 1 && !on_plane[n][g] {
                lone[grp[0].idx()] = true;
            }
        }
    }
    let eligible = |pad: PadId| {
        let net = work.pad(pad).net;
        lone[pad.idx()]
            && !pours.contains(&net)
            && !pair_nets.contains(&net)
            && cfg.nets.as_ref().is_none_or(|v| v.contains(&net))
    };
    let obs = Obstacles::from_board(work);
    escape::plan(
        work,
        &obs,
        &plan::class_geoms(&work.rules),
        grid,
        vm,
        arrays,
        cfg.escape_rings,
        &eligible,
    )
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

#[cfg(test)]
mod via_span_tests;

#[cfg(test)]
mod escape_tests;

#[cfg(test)]
mod custom_rule_tests;
