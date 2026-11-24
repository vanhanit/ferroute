// SPDX-License-Identifier: AGPL-3.0-only
//! Coordinated rip-up & re-route (large-neighbourhood search, in the spirit of MAPF-LNS2 /
//! priority-based search).
//!
//! PathFinder and the detour-driven order refinement ([`crate::detour`]) fix one net at a
//! time, but detours, via jogs and kinks are often caused by a *few nets blocking each
//! other*: fixing one alone does not help. [`improve`] picks a small neighbourhood of nets
//! around a defect (detour, quality blemish or failed net), removes their copper, fixes
//! everything else as existing copper and re-routes just the neighbourhood in other
//! priority orders. A result is kept only if it is a Pareto improvement and passes the
//! oracle gate. Everything is deterministic: seeded by `RouterConfig::seed`, sequential
//! decisions, and `route_once` is itself independent of the thread count.

use crate::detour;
use crate::plan::default_pitch;
use crate::{RouteError, RouteOutcome, RouterConfig, prepare, route_prepared, splitmix64};
use ferroute_board::{ArcTrack, Board, BoardEdit, NetId, Origin, PadId, Track, Via};
use ferroute_drc::{DrcConfig, check, quality};
use ferroute_geom::rng::Rng;
use ferroute_geom::{Coord, Point, Rect};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

/// Large-neighbourhood-search settings.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LnsConfig {
    /// Maximum number of neighbourhoods tried (0 disables the pass).
    pub rounds: u32,
    /// Maximum number of nets re-routed together.
    pub max_neighbourhood: usize,
}

impl Default for LnsConfig {
    fn default() -> Self {
        Self { rounds: 12, max_neighbourhood: 4 }
    }
}

/// Consecutive failed neighbourhoods after which the search stops.
const MAX_CONSECUTIVE_FAILS: u32 = 3;

/// Which kind of defect seeds a neighbourhood.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Kind {
    Detour,
    Quality,
    Conflict,
}

const KINDS: [Kind; 3] = [Kind::Detour, Kind::Quality, Kind::Conflict];

/// Everything the Pareto acceptance compares (lower is better, field-wise).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Metrics {
    pub failed: usize,
    pub vias: usize,
    pub slivers: usize,
    pub right_angles: usize,
    pub via_jogs: usize,
    pub near_pad_bends: usize,
    pub excess_um: u64,
    pub wire_um: u64,
}

impl Metrics {
    #[allow(clippy::cast_possible_truncation)]
    fn arr(&self) -> [u64; 8] {
        [
            self.failed as u64,
            self.vias as u64,
            self.slivers as u64,
            self.right_angles as u64,
            self.via_jogs as u64,
            self.near_pad_bends as u64,
            self.excess_um,
            self.wire_um,
        ]
    }

    /// Pareto improvement: nothing worse, at least one strictly better.
    #[must_use]
    pub fn improves_on(&self, base: &Self) -> bool {
        let (a, b) = (self.arr(), base.arr());
        a.iter().zip(&b).all(|(x, y)| x <= y) && a.iter().zip(&b).any(|(x, y)| x < y)
    }
}

/// Measures `edit` on `board` (failed nets contribute no detour excess).
#[must_use]
pub fn measure(board: &Board, edit: &BoardEdit, failed: &[NetId]) -> Metrics {
    let mut after = board.clone();
    after.apply(edit);
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let wire_um =
        (edit.new_tracks.iter().map(|t| t.start.dist(t.end)).sum::<f64>() / 1e3).round() as u64;
    Metrics {
        failed: failed.len(),
        vias: edit.new_vias.len(),
        slivers: quality::slivers(&after).len(),
        right_angles: quality::right_angles(&after).len(),
        via_jogs: quality::via_jogs(&after).len(),
        near_pad_bends: quality::near_pad_bends(&after).len(),
        excess_um: detour::excess_um(&detour::net_detours(board, edit), failed),
        wire_um,
    }
}

/// Oracle gate figures of a result.
#[derive(Copy, Clone, Debug)]
struct Gate {
    missing: usize,
    acute: usize,
}

/// `None` if `edit` introduces any violation.
fn gate(board: &Board, edit: &BoardEdit) -> Option<Gate> {
    let mut after = board.clone();
    after.apply(edit);
    let rep = check(&after, &DrcConfig { courtyards: false, ..DrcConfig::default() });
    if rep.introduced().next().is_some() {
        return None;
    }
    Some(Gate {
        missing: rep.connectivity.connections_missing,
        acute: quality::acute_angles(&after).len(),
    })
}

/// Bounding boxes of every new copper item with its net.
fn copper_boxes(edit: &BoardEdit) -> Vec<(NetId, Rect)> {
    let mut v = Vec::new();
    for t in &edit.new_tracks {
        v.push((t.net, Rect::new(t.start, t.start).include(t.end).inflate(t.width / 2)));
    }
    for a in &edit.new_arcs {
        v.push((
            a.net,
            Rect::new(a.start, a.start).include(a.mid).include(a.end).inflate(a.width / 2),
        ));
    }
    for x in &edit.new_vias {
        v.push((x.net, Rect::new(x.pos, x.pos).inflate(x.diameter / 2)));
    }
    v
}

/// Bounding box of a net's pad centres, inflated by `margin` (its ideal corridor).
fn corridor(board: &Board, pads: &[Vec<PadId>], net: NetId, margin: Coord) -> Option<Rect> {
    let pts: Vec<Point> = pads[net.idx()].iter().map(|&p| board.pad_position(p)).collect();
    Rect::from_points(&pts).map(|r| r.inflate(margin))
}

fn centre(r: Rect) -> Point {
    Point::new(r.min.x / 2 + r.max.x / 2, r.min.y / 2 + r.max.y / 2)
}

/// `seed` plus the nets whose copper touches `region`, closest to `anchor` first, capped
/// at `max` nets.
fn neighbourhood(
    seed: NetId,
    region: Rect,
    anchor: Point,
    boxes: &[(NetId, Rect)],
    max: usize,
) -> Vec<NetId> {
    let mut near: Vec<(f64, NetId)> = Vec::new();
    for &(net, r) in boxes {
        if net == seed || !r.intersects(region) {
            continue;
        }
        let d = anchor.dist(centre(r));
        match near.iter_mut().find(|e| e.1 == net) {
            Some(e) => e.0 = e.0.min(d),
            None => near.push((d, net)),
        }
    }
    near.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    std::iter::once(seed).chain(near.into_iter().map(|e| e.1)).take(max.max(1)).collect()
}

/// Candidate neighbourhoods of one kind, most promising first (seed net always first).
fn candidates(
    kind: Kind,
    board: &Board,
    cur: &RouteOutcome,
    pitch: Coord,
    max: usize,
) -> Vec<Vec<NetId>> {
    let boxes = copper_boxes(&cur.edit);
    let pads = board.pads_by_net();
    let corridor_hood = |net: NetId| {
        let r = corridor(board, &pads, net, 2 * pitch)?;
        Some(neighbourhood(net, r, centre(r), &boxes, max))
    };
    match kind {
        Kind::Detour => {
            let d = detour::net_detours(board, &cur.edit);
            detour::worst_nets(&d, &cur.failed).into_iter().filter_map(corridor_hood).collect()
        }
        Kind::Conflict => cur.failed.iter().copied().filter_map(corridor_hood).collect(),
        Kind::Quality => {
            let mut after = board.clone();
            after.apply(&cur.edit);
            let mut spots: Vec<(Point, NetId)> = Vec::new();
            spots.extend(quality::via_jogs(&after).iter().map(|s| (s.at, s.net)));
            spots.extend(quality::right_angles(&after).iter().map(|s| (s.at, s.net)));
            spots.extend(quality::near_pad_bends(&after).iter().map(|s| (s.at, s.net)));
            spots.extend(quality::slivers(&after).iter().map(|s| (s.at, s.net)));
            spots
                .into_iter()
                .filter(|&(_, n)| boxes.iter().any(|b| b.0 == n))
                .map(|(at, n)| {
                    neighbourhood(n, Rect::new(at, at).inflate(2 * pitch), at, &boxes, max)
                })
                .collect()
        }
    }
}

/// Priority orders to try for `hood`: all permutations for ≤ 3 nets, else three heuristics
/// (worst excess first, shortest HPWL first, longest HPWL first).
fn orders(board: &Board, cur: &RouteOutcome, hood: &[NetId]) -> Vec<Vec<NetId>> {
    if hood.len() <= 3 {
        let mut out = Vec::new();
        permute(&mut hood.to_vec(), 0, &mut out);
        out.sort_by_key(|o| o.iter().map(|n| hood.iter().position(|h| h == n)).collect::<Vec<_>>());
        return out;
    }
    let d = detour::net_detours(board, &cur.edit);
    let excess = |n: NetId| d.iter().find(|x| x.net == n).map_or(0.0, detour::NetDetour::excess_nm);
    let pads = board.pads_by_net();
    let hpwl = |n: NetId| {
        let pts: Vec<Point> = pads[n.idx()].iter().map(|&p| board.pad_position(p)).collect();
        Rect::from_points(&pts).map_or(0, |r| r.width() + r.height())
    };
    let mut by_excess = hood.to_vec();
    by_excess.sort_by(|&a, &b| excess(b).total_cmp(&excess(a)).then(a.cmp(&b)));
    let mut by_hpwl = hood.to_vec();
    by_hpwl.sort_by_key(|&n| (hpwl(n), n));
    let mut rev = by_hpwl.clone();
    rev.reverse();
    let mut out = vec![by_excess];
    for o in [by_hpwl, rev] {
        if !out.contains(&o) {
            out.push(o);
        }
    }
    out
}

fn permute(v: &mut Vec<NetId>, k: usize, out: &mut Vec<Vec<NetId>>) {
    if k + 1 >= v.len() {
        out.push(v.clone());
        return;
    }
    for i in k..v.len() {
        v.swap(k, i);
        permute(v, k + 1, out);
        v.swap(k, i);
    }
}

/// `board` with all copper of `cur` outside `hood` fixed as existing copper.
fn fixed_board(board: &Board, cur: &BoardEdit, hood: &[NetId]) -> Board {
    let mut b = board.clone();
    let keep = |n: NetId| !hood.contains(&n);
    b.tracks.extend(
        cur.new_tracks
            .iter()
            .filter(|t| keep(t.net))
            .map(|t| Track { origin: Origin::Existing, ..t.clone() }),
    );
    b.arcs.extend(
        cur.new_arcs
            .iter()
            .filter(|t| keep(t.net))
            .map(|t| ArcTrack { origin: Origin::Existing, ..t.clone() }),
    );
    b.vias.extend(
        cur.new_vias
            .iter()
            .filter(|t| keep(t.net))
            .map(|t| Via { origin: Origin::Existing, ..t.clone() }),
    );
    b
}

/// An accepted improvement.
struct Accepted {
    out: RouteOutcome,
    metrics: Metrics,
    gate: Gate,
}

/// Re-routes `hood` in each priority order; returns the first Pareto improvement that
/// passes the oracle gate.
fn replan(
    board: &Board,
    cfg: &RouterConfig,
    cur: &RouteOutcome,
    cur_m: &Metrics,
    cur_gate: Gate,
    hood: &[NetId],
) -> Result<Option<Accepted>, RouteError> {
    // Coupled pairs stay as built: re-routing one net alone would break the coupling.
    let coupled = |n: NetId| {
        ferroute_board::pairs::pair_of(board, n)
            .is_some_and(|p| cur.stats.coupled_pairs.contains(&p.name(board)))
    };
    let hood: Vec<NetId> = hood.iter().copied().filter(|&n| !coupled(n)).collect();
    let hood = hood.as_slice();
    if hood.is_empty() {
        return Ok(None);
    }
    let fixed = fixed_board(board, &cur.edit, hood);
    let sub = RouterConfig {
        nets: Some(hood.to_vec()),
        order_rounds: 0,
        lns: LnsConfig { rounds: 0, ..cfg.lns },
        ..cfg.clone()
    };
    // Legality, lattice and direct links depend only on `fixed`: built once for all orders.
    let prep = prepare(&fixed, &sub)?;
    let eval = |order: &Vec<NetId>| -> Result<Option<Accepted>, RouteError> {
        let cand = route_prepared(&prep, &RouterConfig { first: order.clone(), ..sub.clone() })?;
        let out_hood = |n: NetId| !hood.contains(&n);
        let mut edit = BoardEdit {
            moves: cur.edit.moves.clone(),
            new_tracks: cur.edit.new_tracks.iter().filter(|t| out_hood(t.net)).cloned().collect(),
            new_arcs: cur.edit.new_arcs.iter().filter(|t| out_hood(t.net)).cloned().collect(),
            new_vias: cur.edit.new_vias.iter().filter(|t| out_hood(t.net)).cloned().collect(),
            zone_fills: Vec::new(),
        };
        edit.new_tracks.extend(cand.edit.new_tracks.iter().cloned());
        edit.new_arcs.extend(cand.edit.new_arcs.iter().cloned());
        edit.new_vias.extend(cand.edit.new_vias.iter().cloned());
        let mut failed: Vec<NetId> = cur.failed.iter().copied().filter(|&n| out_hood(n)).collect();
        failed.extend(cand.failed.iter().copied());
        failed.sort_unstable();
        failed.dedup();
        let m = measure(board, &edit, &failed);
        if !m.improves_on(cur_m) {
            return Ok(None);
        }
        let Some(g) = gate(board, &edit) else { return Ok(None) };
        if g.missing > cur_gate.missing || g.acute > cur_gate.acute {
            return Ok(None);
        }
        let old_failed_in_hood = cur.failed.iter().filter(|n| hood.contains(n)).count();
        let mut out = cur.clone();
        out.edit = edit;
        out.failed = failed;
        out.stats.nets_routed = (cur.stats.nets_routed + cand.stats.nets_routed)
            .saturating_sub(hood.len().saturating_sub(old_failed_in_hood));
        out.stats.nets_failed = out.failed.len();
        out.stats.drc_gate_rips += cand.stats.drc_gate_rips;
        Ok(Some(Accepted { out, metrics: m, gate: g }))
    };
    // Each order is an independent pure evaluation: chunks run in parallel, but the first
    // accepted order *in sequence* wins, so the result never depends on the thread count.
    let tries = orders(board, cur, hood);
    let chunk = rayon::current_num_threads().max(1);
    for batch in tries.chunks(chunk) {
        let results: Vec<Result<Option<Accepted>, RouteError>> =
            batch.par_iter().map(&eval).collect();
        for r in results {
            if let Some(acc) = r? {
                return Ok(Some(acc));
            }
        }
    }
    Ok(None)
}

/// Improves `out` (a routing of `board`) by re-routing small neighbourhoods of nets
/// together; see the module docs. Never returns anything worse than `out`.
///
/// # Errors
/// [`RouteError`] from the underlying router.
#[allow(clippy::cast_possible_truncation)]
pub fn improve(
    board: &Board,
    cfg: &RouterConfig,
    mut out: RouteOutcome,
) -> Result<RouteOutcome, RouteError> {
    let lc = cfg.lns;
    if lc.rounds == 0 {
        return Ok(out);
    }
    let pitch = if out.stats.pitch > 0 { out.stats.pitch } else { default_pitch(&board.rules) };
    let Some(mut cur_gate) = gate(board, &out.edit) else { return Ok(out) };
    let mut cur_m = measure(board, &out.edit, &out.failed);
    let mut rng = Rng::new(splitmix64(cfg.seed ^ 0x4C4E_5332));
    let mut tried: BTreeSet<Vec<NetId>> = BTreeSet::new();
    let mut fails = 0u32;
    for round in 0..lc.rounds {
        if fails >= MAX_CONSECUTIVE_FAILS {
            break;
        }
        let kind = KINDS[round as usize % KINDS.len()];
        let mut seen = BTreeSet::new();
        let cands: Vec<Vec<NetId>> = candidates(kind, board, &out, pitch, lc.max_neighbourhood)
            .into_iter()
            .filter(|h| {
                let mut key = h.clone();
                key.sort_unstable();
                !tried.contains(&key) && seen.insert(key)
            })
            .collect();
        if cands.is_empty() {
            fails += 1;
            continue;
        }
        let pick = rng.below(cands.len().min(3) as u64) as usize;
        let hood = &cands[pick];
        let mut key = hood.clone();
        key.sort_unstable();
        tried.insert(key);
        out.stats.lns_rounds_run += 1;
        if let Some(acc) = replan(board, cfg, &out, &cur_m, cur_gate, hood)? {
            let (run, accepted) = (out.stats.lns_rounds_run, out.stats.lns_accepted + 1);
            out = acc.out;
            out.stats.lns_rounds_run = run;
            out.stats.lns_accepted = accepted;
            cur_m = acc.metrics;
            cur_gate = acc.gate;
            tried.clear();
            fails = 0;
        } else {
            fails += 1;
        }
    }
    refresh_stats(&mut out, &cur_m);
    Ok(out)
}

/// Re-derives the statistics that depend on the copper from `out.edit`.
#[allow(clippy::cast_precision_loss)]
fn refresh_stats(out: &mut RouteOutcome, m: &Metrics) {
    let s = &mut out.stats;
    s.tracks = out.edit.new_tracks.len();
    s.vias = out.edit.new_vias.len();
    s.right_angle_bends = m.right_angles;
    s.near_pad_bends = m.near_pad_bends;
    s.slivers = m.slivers;
    s.via_jogs = m.via_jogs;
    s.wirelength_nm = out.edit.new_tracks.iter().map(|t| t.start.dist(t.end)).sum::<f64>() + 0.0;
    s.detour_excess_mm = m.excess_um as f64 / 1000.0;
}
