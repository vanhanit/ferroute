// SPDX-License-Identifier: AGPL-3.0-only
//! # ferroute-place — placement-aware nudging of unlocked components
//!
//! Seeded simulated annealing (multi-start, deterministic) over the placements of
//! footprints whose [`Mobility`] is `Movable`, minimising
//!
//! `cost = w_wire · Σ_net weight(net) · HPWL(net) + w_cong · Σ_bin overflow(bin)² + w_disp · Σ_fp |Δ|`
//!
//! where overflow uses a RUDY (Rectangular Uniform wire DensitY, Spindler & Johannes
//! 2007) congestion estimate. Every candidate placement must pass exact hard
//! constraints ([`legal`]): displacement / rotation bounds, courtyard inside the board
//! and disjoint from every other courtyard on the same side, keep-out areas, and exact
//! copper clearance of the moved pads against all other copper.
//!
//! INV-PLACE-1: `Locked` and `Fixed` footprints never move.
//! INV-PLACE-2: the result is a pure function of (board, config) — the seed fixes it.

use ferroute_board::{Board, FootprintId, FootprintMove, Mobility, NetId, Placement, Side};
use ferroute_geom::rng::{Rng, splitmix64};
use ferroute_geom::{Angle, Coord, Point, Rect};
use serde::{Deserialize, Serialize};

/// Which footprints may move and how far.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MovePolicy {
    /// Reference glob patterns (`*`, `?`) of movable footprints, e.g. `R*`, `C*`.
    pub include: Vec<String>,
    /// Patterns that are never moved (wins over `include`).
    pub exclude: Vec<String>,
    pub max_displacement: Coord,
    pub allow_rotation: bool,
}

impl Default for MovePolicy {
    /// Passives only, ≤ 2 mm, with 90° rotations.
    fn default() -> Self {
        Self {
            include: vec!["R*".into(), "C*".into(), "L*".into(), "FB*".into()],
            exclude: Vec::new(),
            max_displacement: 2_000_000,
            allow_rotation: true,
        }
    }
}

pub use ferroute_board::glob;

/// Applies `policy` to the board's footprint mobilities. Locked footprints stay
/// `Locked`; footprints whose pads touch pre-existing copper are auto-`Fixed` so
/// existing routing is never broken. Returns the number of movable footprints.
pub fn apply_policy(board: &mut Board, policy: &MovePolicy) -> usize {
    let touched = footprints_with_existing_copper(board);
    let mut n = 0;
    for (i, fp) in board.footprints.iter_mut().enumerate() {
        if fp.locked {
            fp.mobility = Mobility::Locked;
            continue;
        }
        let want = policy.include.iter().any(|p| glob(p, &fp.reference))
            && !policy.exclude.iter().any(|p| glob(p, &fp.reference));
        fp.mobility = if want && !touched[i] {
            n += 1;
            Mobility::Movable {
                max_displacement: policy.max_displacement,
                allow_rotation: policy.allow_rotation,
            }
        } else {
            Mobility::Fixed
        };
    }
    n
}

fn footprints_with_existing_copper(board: &Board) -> Vec<bool> {
    let mut v = vec![false; board.footprints.len()];
    for id in board.pad_ids() {
        let pad = board.pad(id);
        let h = board.pad_hull(id);
        let hit = board.tracks.iter().any(|t| {
            t.net == pad.net && pad.layers.contains(t.layer) && h.clearance_violated(&t.hull(), 1)
        }) || board
            .vias
            .iter()
            .any(|via| via.net == pad.net && h.clearance_violated(&via.hull(), 1));
        if hit {
            v[pad.footprint.idx()] = true;
        }
    }
    v
}

/// Annealing configuration.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PlaceConfig {
    pub seed: u64,
    /// Independent annealing chains (run in parallel; best wins, ties → lowest index).
    pub starts: u32,
    /// Proposals per movable footprint per chain.
    pub moves_per_fp: u32,
    /// Placement grid (nm). A translated footprint's anchor always lands on an absolute
    /// multiple of this step (INV-PLACE-3), like KiCad's placement grid.
    pub grid: Coord,
    pub w_wire: f64,
    pub w_cong: f64,
    pub w_disp: f64,
    /// RUDY bin size (nm).
    pub bin: Coord,
    /// Per-net HPWL weight multipliers (e.g. boost nets the router failed on).
    pub net_weights: Vec<(NetId, f64)>,
}

impl Default for PlaceConfig {
    fn default() -> Self {
        Self {
            seed: 1,
            starts: 4,
            moves_per_fp: 400,
            grid: 100_000,
            w_wire: 1.0,
            w_cong: 4.0,
            w_disp: 0.25,
            bin: 1_000_000,
            net_weights: Vec::new(),
        }
    }
}

/// Cost breakdown (mm-based units so weights are intuitive).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Cost {
    pub wire_mm: f64,
    pub congestion: f64,
    pub displacement_mm: f64,
    pub total: f64,
}

/// Nudger result.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct PlaceResult {
    pub moves: Vec<FootprintMove>,
    pub before: Cost,
    pub after: Cost,
    pub movable: usize,
    pub accepted: u64,
    pub proposed: u64,
    pub best_chain: u32,
}

#[allow(clippy::cast_precision_loss)]
fn mm(v: f64) -> f64 {
    v / 1e6
}

struct Model<'a> {
    board: &'a Board,
    cfg: &'a PlaceConfig,
    weights: Vec<f64>,
    pads_by_net: Vec<Vec<ferroute_board::PadId>>,
    bins: (Point, i64, i64),
    capacity: f64,
}

impl<'a> Model<'a> {
    fn new(board: &'a Board, cfg: &'a PlaceConfig) -> Self {
        let mut weights = vec![1.0; board.nets.len()];
        weights[0] = 0.0;
        for &(n, w) in &cfg.net_weights {
            if let Some(x) = weights.get_mut(n.idx()) {
                *x = w;
            }
        }
        let bb = board.outline.bbox().unwrap_or(Rect::new(Point::ORIGIN, Point::ORIGIN));
        let bin = cfg.bin.max(1);
        let bins = (bb.min, bb.width() / bin + 1, bb.height() / bin + 1);
        // Capacity of a bin in "track lengths": layers × bin / (track pitch).
        let pitch = board.rules.classes[0].track_width + board.rules.classes[0].clearance;
        #[allow(clippy::cast_precision_loss)]
        let capacity = f64::from(board.layer_count()) * bin as f64 / pitch.max(1) as f64;
        Self { board, cfg, weights, pads_by_net: board.pads_by_net(), bins, capacity }
    }

    fn pad_pos(&self, pl: &[Placement], pad: ferroute_board::PadId) -> Point {
        let p = self.board.pad(pad);
        p.position(pl[p.footprint.idx()])
    }

    fn cost(&self, pl: &[Placement]) -> Cost {
        let mut wire = 0.0;
        let (origin, bx, by) = self.bins;
        #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
        let mut demand = vec![0.0f64; (bx * by) as usize];
        #[allow(clippy::cast_precision_loss)]
        let bin = self.cfg.bin.max(1) as f64;
        for (n, pads) in self.pads_by_net.iter().enumerate() {
            if pads.len() < 2 || self.weights[n] == 0.0 {
                continue;
            }
            let pts: Vec<Point> = pads.iter().map(|&p| self.pad_pos(pl, p)).collect();
            let Some(r) = Rect::from_points(&pts) else { continue };
            #[allow(clippy::cast_precision_loss)]
            let (w, h) = (r.width() as f64, r.height() as f64);
            wire += self.weights[n] * (w + h);
            // RUDY: wire length spread uniformly over the net's bounding box.
            let (w, h) = (w.max(bin), h.max(bin));
            // Wire length per bin area times bin² gives length in the bin; divide by the
            // bin size to express it in "bin-crossing track" units like the capacity.
            let density = (w + h) / (w * h) * bin;
            let i0 = ((r.min.x - origin.x) / self.cfg.bin).clamp(0, bx - 1);
            let i1 = ((r.max.x - origin.x) / self.cfg.bin).clamp(0, bx - 1);
            let j0 = ((r.min.y - origin.y) / self.cfg.bin).clamp(0, by - 1);
            let j1 = ((r.max.y - origin.y) / self.cfg.bin).clamp(0, by - 1);
            for j in j0..=j1 {
                for i in i0..=i1 {
                    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
                    {
                        demand[(j * bx + i) as usize] += density * self.weights[n];
                    }
                }
            }
        }
        let cong: f64 =
            demand.iter().map(|&d| (d - 0.6 * self.capacity).max(0.0).powi(2)).sum::<f64>()
                / self.capacity.max(1.0);
        let disp: f64 =
            self.board.footprints.iter().zip(pl).map(|(f, p)| p.pos.dist(f.original.pos)).sum();
        let (wire_mm, displacement_mm) = (mm(wire), mm(disp));
        Cost {
            wire_mm,
            congestion: cong,
            displacement_mm,
            total: self.cfg.w_wire * wire_mm
                + self.cfg.w_cong * cong
                + self.cfg.w_disp * displacement_mm,
        }
    }
}

/// Per-net cost terms derived from the net's pad bounding box.
#[derive(Copy, Clone, Debug)]
struct NetTerm {
    /// Weighted half-perimeter (nm).
    wire: f64,
    /// Weighted RUDY density added to every covered bin.
    contrib: f64,
    bins: (i64, i64, i64, i64),
}

/// Cached geometry of the placement being annealed, so a one-footprint move re-derives
/// only the nets and displacement it touches. Every sum is still accumulated in the
/// same order as [`Model::cost`], so both agree bit for bit.
#[derive(Debug)]
struct CostCache {
    padpos: Vec<Point>,
    term: Vec<Option<NetTerm>>,
    dist: Vec<f64>,
    demand: Vec<f64>,
    /// Nets (with a cost term) touched by each footprint's pads.
    fp_nets: Vec<Vec<usize>>,
}

/// What [`Model::eval_move`] changed, to undo a rejected move.
#[derive(Debug, Default)]
struct Undo {
    pads: Vec<(usize, Point)>,
    nets: Vec<(usize, Option<NetTerm>)>,
    fp: usize,
    dist: f64,
}

impl Model<'_> {
    fn net_term(&self, n: usize, pts: &[Point]) -> Option<NetTerm> {
        if pts.len() < 2 || self.weights[n] == 0.0 {
            return None;
        }
        let r = Rect::from_points(pts)?;
        let (origin, bx, by) = self.bins;
        #[allow(clippy::cast_precision_loss)]
        let bin = self.cfg.bin.max(1) as f64;
        #[allow(clippy::cast_precision_loss)]
        let (w, h) = (r.width() as f64, r.height() as f64);
        let wire = self.weights[n] * (w + h);
        let (w, h) = (w.max(bin), h.max(bin));
        let density = (w + h) / (w * h) * bin;
        let i0 = ((r.min.x - origin.x) / self.cfg.bin).clamp(0, bx - 1);
        let i1 = ((r.max.x - origin.x) / self.cfg.bin).clamp(0, bx - 1);
        let j0 = ((r.min.y - origin.y) / self.cfg.bin).clamp(0, by - 1);
        let j1 = ((r.max.y - origin.y) / self.cfg.bin).clamp(0, by - 1);
        Some(NetTerm { wire, contrib: density * self.weights[n], bins: (i0, i1, j0, j1) })
    }

    fn net_term_cached(&self, cc: &CostCache, n: usize) -> Option<NetTerm> {
        let pts: Vec<Point> = self.pads_by_net[n].iter().map(|p| cc.padpos[p.0 as usize]).collect();
        self.net_term(n, &pts)
    }

    fn cache(&self, pl: &[Placement]) -> CostCache {
        let board = self.board;
        let padpos: Vec<Point> = board.pad_ids().map(|p| self.pad_pos(pl, p)).collect();
        let (_, bx, by) = self.bins;
        #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
        let demand = vec![0.0f64; (bx * by) as usize];
        let mut cc = CostCache {
            padpos,
            term: Vec::new(),
            dist: board
                .footprints
                .iter()
                .zip(pl)
                .map(|(f, p)| p.pos.dist(f.original.pos))
                .collect(),
            demand,
            fp_nets: Vec::new(),
        };
        cc.term = (0..self.pads_by_net.len()).map(|n| self.net_term_cached(&cc, n)).collect();
        cc.fp_nets = board
            .footprints
            .iter()
            .map(|f| {
                let mut v: Vec<usize> = f
                    .pads
                    .iter()
                    .map(|&p| board.pad(p).net.idx())
                    .filter(|&n| cc.term[n].is_some())
                    .collect();
                v.sort_unstable();
                v.dedup();
                v
            })
            .collect();
        cc
    }

    /// Cost of the cached placement.
    fn cost_cached(&self, cc: &mut CostCache) -> Cost {
        let (_, bx, _) = self.bins;
        cc.demand.fill(0.0);
        let mut wire = 0.0;
        for t in cc.term.iter().flatten() {
            wire += t.wire;
            let (i0, i1, j0, j1) = t.bins;
            for j in j0..=j1 {
                for i in i0..=i1 {
                    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
                    {
                        cc.demand[(j * bx + i) as usize] += t.contrib;
                    }
                }
            }
        }
        let cong: f64 =
            cc.demand.iter().map(|&d| (d - 0.6 * self.capacity).max(0.0).powi(2)).sum::<f64>()
                / self.capacity.max(1.0);
        let disp: f64 = cc.dist.iter().copied().sum();
        let (wire_mm, displacement_mm) = (mm(wire), mm(disp));
        Cost {
            wire_mm,
            congestion: cong,
            displacement_mm,
            total: self.cfg.w_wire * wire_mm
                + self.cfg.w_cong * cong
                + self.cfg.w_disp * displacement_mm,
        }
    }

    /// Cost with footprint `fid` moved to `cand`; the cache is left in the moved state
    /// (call [`CostCache::undo`] to reject).
    fn eval_move(&self, cc: &mut CostCache, fid: FootprintId, cand: Placement) -> (Cost, Undo) {
        let fp = self.board.footprint(fid);
        let mut undo = Undo { fp: fid.idx(), dist: cc.dist[fid.idx()], ..Undo::default() };
        for &p in &fp.pads {
            let i = p.0 as usize;
            undo.pads.push((i, cc.padpos[i]));
            cc.padpos[i] = self.board.pad(p).position(cand);
        }
        cc.dist[fid.idx()] = cand.pos.dist(fp.original.pos);
        for k in 0..cc.fp_nets[fid.idx()].len() {
            let n = cc.fp_nets[fid.idx()][k];
            undo.nets.push((n, cc.term[n]));
            cc.term[n] = self.net_term_cached(cc, n);
        }
        (self.cost_cached(cc), undo)
    }
}

impl CostCache {
    fn undo(&mut self, u: Undo) {
        for (i, p) in u.pads {
            self.padpos[i] = p;
        }
        for (n, t) in u.nets {
            self.term[n] = t;
        }
        self.dist[u.fp] = u.dist;
    }
}

/// Rounds `v` to the nearest multiple of `g` (ties toward +∞), in absolute coordinates.
#[must_use]
pub const fn snap(v: Coord, g: Coord) -> Coord {
    (v + g / 2).div_euclid(g) * g
}

/// Conservative extents that let [`legal_with`] skip far-away footprints without building
/// their geometry. Every extent is an upper bound of the true distance from the
/// footprint's anchor (plus a rounding margin), so skipping never changes the verdict.
#[derive(Debug)]
pub struct Prefilter {
    /// Per footprint: radius of the courtyard around the anchor.
    court: Vec<Coord>,
    /// Per footprint: radius around the anchor that contains every pad hull.
    pads: Vec<Coord>,
}

/// Slack for the integer rounding of rotated points.
const EXTENT_MARGIN: Coord = 16;

fn norm_ceil(p: Point) -> Coord {
    #[allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]
    let n = (p.x as f64).hypot(p.y as f64).ceil() as Coord;
    n
}

impl Prefilter {
    /// Builds the extents of `board`'s footprints.
    #[must_use]
    pub fn new(board: &Board) -> Self {
        let court = board
            .footprints
            .iter()
            .map(|f| f.courtyard_local.pts.iter().map(|&p| norm_ceil(p)).max().unwrap_or(0))
            .map(|r| r + EXTENT_MARGIN)
            .collect();
        let pads = board
            .footprints
            .iter()
            .map(|f| {
                f.pads
                    .iter()
                    .map(|&id| {
                        let pad = board.pad(id);
                        let h = pad.shape.local_hull();
                        let hull_r = h.pts.iter().map(|&p| norm_ceil(p)).max().unwrap_or(0);
                        norm_ceil(pad.local_pos) + hull_r + h.radius
                    })
                    .max()
                    .unwrap_or(0)
                    + EXTENT_MARGIN
            })
            .collect();
        Self { court, pads }
    }
}

/// Exact hard-constraint check for placing footprint `fid` at `pl` while every other
/// footprint uses `placements`.
#[must_use]
pub fn legal(board: &Board, placements: &[Placement], fid: FootprintId, pl: Placement) -> bool {
    legal_with(board, &Prefilter::new(board), placements, fid, pl)
}

/// [`legal`] with precomputed [`Prefilter`] extents (same verdict, far less geometry).
#[must_use]
pub fn legal_with(
    board: &Board,
    pf: &Prefilter,
    placements: &[Placement],
    fid: FootprintId,
    pl: Placement,
) -> bool {
    let fp = board.footprint(fid);
    let Mobility::Movable { max_displacement, allow_rotation } = fp.mobility else {
        return pl == fp.original;
    };
    #[allow(clippy::cast_precision_loss)]
    if pl.pos.dist(fp.original.pos) > max_displacement as f64 {
        return false;
    }
    if pl.rot != fp.original.rot
        && (!allow_rotation || pl.rot.sub(fp.original.rot).quadrant().is_none())
    {
        return false;
    }
    let court = fp.courtyard_at(pl);
    // Courtyard fully inside the board.
    if !court.pts.iter().all(|&p| board.outline.outer.contains(p))
        || board
            .outline
            .outer
            .edges()
            .any(|e| court.edges().any(|f| ferroute_geom::segments_intersect(e, f)))
        || board.outline.holes.iter().any(|h| h.intersects(&court))
    {
        return false;
    }
    let bb = court.bbox().unwrap_or(Rect::new(pl.pos, pl.pos));
    for (j, other) in board.footprints.iter().enumerate() {
        if j == fid.idx() || other.side != fp.side {
            continue;
        }
        // Courtyards within disjoint anchor discs cannot intersect.
        let (dx, dy) = (
            i128::from(placements[j].pos.x) - i128::from(pl.pos.x),
            i128::from(placements[j].pos.y) - i128::from(pl.pos.y),
        );
        let reach = i128::from(pf.court[j] + pf.court[fid.idx()]);
        if dx * dx + dy * dy > reach * reach {
            continue;
        }
        let oc = other.courtyard_at(placements[j]);
        if oc.bbox().is_some_and(|ob| ob.intersects(bb)) && oc.intersects(&court) {
            return false;
        }
    }
    if board.keepouts.iter().any(|k| k.no_footprints && k.area.intersects(&court)) {
        return false;
    }
    // Exact copper clearance of the moved pads.
    let ec = board.rules.edge_clearance;
    for &pid in &fp.pads {
        let pad = board.pad(pid);
        if pad.layers.is_empty() {
            continue;
        }
        let h = pad.hull(pl);
        if !board.outline.admits(&h, ec) {
            return false;
        }
        if board
            .keepouts
            .iter()
            .any(|k| k.no_pads && k.layers.intersects(pad.layers) && k.area.hull_violates(&h, 0))
        {
            return false;
        }
        let hb = h.bbox().inflate(board.rules.max_clearance() + 1);
        for (k, other) in board.footprints.iter().enumerate() {
            if k == fid.idx() {
                continue;
            }
            // Every pad hull of `other` lies within `pf.pads[k]` of its anchor: skip the
            // whole footprint when that square misses the (inflated) candidate pad box.
            let (a, r) = (placements[k].pos, pf.pads[k]);
            if a.x + r < hb.min.x || a.x - r > hb.max.x || a.y + r < hb.min.y || a.y - r > hb.max.y
            {
                continue;
            }
            for &qid in &other.pads {
                let q = board.pad(qid);
                if !q.layers.intersects(pad.layers) || (q.net == pad.net && pad.net != NetId::NONE)
                {
                    continue;
                }
                let qh = q.hull(placements[k]);
                if qh.bbox().intersects(hb)
                    && h.clearance_violated(&qh, board.clearance_between(pad.net, q.net))
                {
                    return false;
                }
            }
        }
        for t in &board.tracks {
            if pad.layers.contains(t.layer)
                && (t.net != pad.net || t.net == NetId::NONE)
                && h.clearance_violated(&t.hull(), board.clearance_between(pad.net, t.net))
            {
                return false;
            }
        }
        for v in &board.vias {
            if pad.layers.intersects(v.layers)
                && (v.net != pad.net || v.net == NetId::NONE)
                && h.clearance_violated(&v.hull(), board.clearance_between(pad.net, v.net))
            {
                return false;
            }
        }
    }
    true
}

fn chain(
    board: &Board,
    cfg: &PlaceConfig,
    model: &Model<'_>,
    pf: &Prefilter,
    movable: &[FootprintId],
    seed: u64,
) -> (Vec<Placement>, Cost, u64, u64) {
    let mut rng = Rng::new(seed);
    let mut cur: Vec<Placement> = board.footprints.iter().map(|f| f.placement).collect();
    let mut cur_cost = model.cost(&cur);
    let mut cc = model.cache(&cur);
    let mut best = cur.clone();
    let mut best_cost = cur_cost;
    let total = u64::from(cfg.moves_per_fp) * movable.len() as u64;
    // Initial temperature: a typical uphill move (≈ quantum-scale wire change) is
    // accepted with probability ~0.5 at the start.
    #[allow(clippy::cast_precision_loss)]
    let t0 = (cfg.w_wire * mm(cfg.grid as f64) * 20.0).max(1e-6);
    let t_end = t0 * 1e-3;
    let (mut accepted, mut proposed) = (0u64, 0u64);
    for step in 0..total {
        #[allow(clippy::cast_precision_loss)]
        let frac = step as f64 / total.max(1) as f64;
        // Geometric cooling t0 → t_end evaluated with platform-independent exp (INV-DET-3).
        let temp = t0 * ferroute_geom::detmath::exp(frac * (t_end / t0).ln_const());
        let fid = movable[usize::try_from(rng.below(movable.len() as u64)).unwrap_or(0)];
        let fp = board.footprint(fid);
        let Mobility::Movable { max_displacement, allow_rotation } = fp.mobility else { continue };
        let mut cand = cur[fid.idx()];
        if allow_rotation && rng.below(8) == 0 {
            cand.rot =
                cand.rot.add(Angle::from_degrees(if rng.below(2) == 0 { 90.0 } else { -90.0 }));
        } else {
            // Window shrinks with temperature (coarse → fine).
            #[allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]
            let g = cfg.grid.max(1);
            let r =
                ((max_displacement as f64 * (1.0 - 0.9 * frac)) / g as f64).ceil().max(1.0) as i64;
            let dx = rng.range_i64(-r, r) * g;
            let dy = rng.range_i64(-r, r) * g;
            let p = cand.pos + Point::new(dx, dy);
            cand.pos = Point::new(snap(p.x, g), snap(p.y, g));
        }
        proposed += 1;
        if cand == cur[fid.idx()] || !legal_with(board, pf, &cur, fid, cand) {
            continue;
        }
        let old = cur[fid.idx()];
        cur[fid.idx()] = cand;
        let (c, undo) = model.eval_move(&mut cc, fid, cand);
        let delta = c.total - cur_cost.total;
        if delta <= 0.0 || rng.next_f64() < ferroute_geom::detmath::exp(-delta / temp) {
            cur_cost = c;
            accepted += 1;
            if c.total < best_cost.total {
                best_cost = c;
                best.clone_from(&cur);
            }
        } else {
            cur[fid.idx()] = old;
            cc.undo(undo);
        }
    }
    (best, best_cost, accepted, proposed)
}

/// Runs the nudger. Footprint mobilities must already be set (see [`apply_policy`]).
#[must_use]
pub fn optimize(board: &Board, cfg: &PlaceConfig) -> PlaceResult {
    let model = Model::new(board, cfg);
    let start: Vec<Placement> = board.footprints.iter().map(|f| f.placement).collect();
    let before = model.cost(&start);
    #[allow(clippy::cast_possible_truncation)]
    let movable: Vec<FootprintId> = board
        .footprints
        .iter()
        .enumerate()
        .filter(|(_, f)| f.mobility.is_movable())
        .map(|(i, _)| FootprintId(i as u32))
        .collect();
    if movable.is_empty() {
        return PlaceResult { before, after: before, ..PlaceResult::default() };
    }
    let starts = cfg.starts.max(1);
    let pf = Prefilter::new(board);
    let results: Vec<(Vec<Placement>, Cost, u64, u64)> = {
        use rayon::prelude::*;
        (0..starts)
            .into_par_iter()
            .map(|c| {
                chain(
                    board,
                    cfg,
                    &model,
                    &pf,
                    &movable,
                    splitmix64(cfg.seed ^ u64::from(c).wrapping_mul(0xA24B_AED4_963E_E407)),
                )
            })
            .collect()
    };
    let (best_idx, best) = results
        .iter()
        .enumerate()
        .min_by(|a, b| a.1.1.total.total_cmp(&b.1.1.total).then(a.0.cmp(&b.0)))
        .map(|(i, r)| (i, r.clone()))
        .unwrap_or((0, (start, before, 0, 0)));
    let (best_pl, after, _, _) = best;
    let moves = board
        .footprints
        .iter()
        .enumerate()
        .filter(|(i, f)| best_pl[*i] != f.placement)
        .map(|(i, f)| FootprintMove {
            footprint: FootprintId(u32::try_from(i).unwrap_or(u32::MAX)),
            from: f.placement,
            to: best_pl[i],
        })
        .collect();
    PlaceResult {
        moves,
        before,
        after,
        movable: movable.len(),
        accepted: results.iter().map(|r| r.2).sum(),
        proposed: results.iter().map(|r| r.3).sum(),
        best_chain: u32::try_from(best_idx).unwrap_or(0),
    }
}

/// Which side a footprint sits on (helper for callers rendering moves).
#[must_use]
pub fn side_of(board: &Board, f: FootprintId) -> Side {
    board.footprint(f).side
}

#[cfg(test)]
mod tests;

/// `ln(1e-3)`, the fixed cooling ratio, as a literal (no libm call).
trait LnConst {
    fn ln_const(self) -> f64;
}

impl LnConst for f64 {
    fn ln_const(self) -> f64 {
        debug_assert!((self - 1e-3).abs() < 1e-12, "cooling ratio is fixed at 1e-3");
        -6.907_755_278_982_137
    }
}
