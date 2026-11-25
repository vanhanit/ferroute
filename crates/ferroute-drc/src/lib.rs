// SPDX-License-Identifier: AGPL-3.0-only
//! # ferroute-drc — the deterministic DRC oracle
//!
//! A standalone, exact design-rule checker. It is the *final authority* on legality in
//! ferroute: the router, the nudger and every accelerator only *propose* geometry; a run
//! is accepted only if this oracle reports no violation introduced by ferroute.
//!
//! Guarantees:
//! * **Exactness** — every legality decision uses the integer predicates of
//!   `ferroute-geom` (no floating point), under INV-GEOM-1.
//! * **Determinism** — the report is sorted, so it is identical for identical input
//!   regardless of thread count or hash seeds.
//! * **Conservativeness** — shapes are convex supersets of the true copper, so a clean
//!   report implies the true design is clean for the checked rules.
//!
//! Checked rules: copper clearance & shorts (per net-class, KiCad "max of both"
//! semantics), copper-to-edge clearance and out-of-board copper, hole-to-hole spacing,
//! copper-to-hole clearance, keep-out areas, courtyard overlap, minimum track width /
//! via size, and connectivity (unrouted connections + completion ratio).

pub use ferroute_board::custom::Severity;
use ferroute_board::custom::{ConstraintKind, ItemCtx, ItemType};
use ferroute_board::{
    Board, FootprintId, LayerId, LayerSet, NetId, Origin, PadId, Side, Via, ViaKind,
};
use ferroute_geom::{Coord, Hull, Point, Rect};
use rayon::prelude::*;
use rustc_hash::FxHashMap;
use serde::{Deserialize, Serialize};

pub mod connectivity;
pub mod dfm;
pub mod pairs;
pub mod quality;
pub mod refill;
pub mod rules;
pub mod thermal;
pub mod zones;
pub use connectivity::{Connectivity, Unconnected};

/// What the oracle checks.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DrcConfig {
    pub copper: bool,
    pub edge: bool,
    pub holes: bool,
    pub keepouts: bool,
    pub courtyards: bool,
    pub sizes: bool,
    pub connectivity: bool,
    /// Skip clearance checks between pads of the same footprint (library geometry is
    /// outside ferroute's responsibility; KiCad reports these separately).
    pub skip_intra_footprint_pads: bool,
}

impl Default for DrcConfig {
    fn default() -> Self {
        Self {
            copper: true,
            edge: true,
            holes: true,
            keepouts: true,
            courtyards: true,
            sizes: true,
            connectivity: true,
            skip_intra_footprint_pads: true,
        }
    }
}

/// Category of a violation.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub enum ViolationKind {
    /// Copper of different nets overlaps.
    Short,
    /// Copper of different nets closer than the required clearance.
    Clearance,
    /// Copper closer to the board edge than `edge_clearance` (or outside the board).
    EdgeClearance,
    HoleToHole,
    HoleClearance,
    Keepout,
    CourtyardOverlap,
    TrackWidth,
    ViaDiameter,
    ViaDrill,
    /// Via / pad annular ring narrower than a custom `annular_width` rule.
    AnnularWidth,
    /// Track wider than a custom `track_width` `max`.
    TrackWidthMax,
    /// Via copper diameter above a custom `via_diameter` `max`.
    ViaDiameterMax,
    /// Via drill above a custom `hole_size` `max`.
    ViaDrillMax,
    /// Annular ring wider than a custom `annular_width` `max`.
    AnnularWidthMax,
    /// Plated or unplated pad hole below a custom `hole_size` `min`.
    PadDrill,
    /// Pad hole above a custom `hole_size` `max`.
    PadDrillMax,
    /// A via whose copper span does not fit its kind (e.g. a micro via over more than two
    /// adjacent layers, a blind via with both ends on outer layers).
    ViaSpan,
    /// A pad's thermal relief reaches the fill with fewer than `min_resolved_spokes` spokes
    /// (KiCad `starved_thermal`, see [`thermal`]).
    StarvedThermal,
}

/// Reference to a board item.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub enum ItemRef {
    Pad(PadId),
    Track(u32),
    /// Index into `board.arcs`.
    Arc(u32),
    Via(u32),
    Footprint(FootprintId),
    Keepout(u32),
    /// Index into `board.zones` (copper pour).
    Zone(u32),
    BoardEdge,
}

/// A single rule violation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Violation {
    pub kind: ViolationKind,
    pub a: ItemRef,
    pub b: Option<ItemRef>,
    pub layer: Option<LayerId>,
    /// Approximate location (for markers).
    pub at: Point,
    /// Measured gap / size in nm (reporting only).
    pub actual: f64,
    /// Required gap / size in nm.
    pub required: Coord,
    /// `true` when the violation involves copper created by ferroute or a footprint
    /// ferroute moved. The router invariant is: *no introduced violations, ever*.
    pub introduced: bool,
    /// [`Severity::Error`] fails the gate; warnings are reported but never fail it.
    /// Suppressed (`ignore` / `exclusion`) violations are not reported at all.
    #[serde(default)]
    pub severity: Severity,
}

/// Full DRC result.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DrcReport {
    pub violations: Vec<Violation>,
    pub connectivity: Connectivity,
}

impl DrcReport {
    /// No *error* violations (warnings do not count; connectivity not considered).
    #[must_use]
    pub fn is_clean(&self) -> bool {
        self.errors().next().is_none()
    }

    /// Hard violations (severity error).
    pub fn errors(&self) -> impl Iterator<Item = &Violation> {
        self.violations.iter().filter(|v| v.severity == Severity::Error)
    }

    /// Non-error violations (custom rules with `(severity warning)`).
    pub fn warnings(&self) -> impl Iterator<Item = &Violation> {
        self.violations.iter().filter(|v| v.severity == Severity::Warning)
    }

    /// Error violations caused by ferroute (routed copper / moved footprints).
    pub fn introduced(&self) -> impl Iterator<Item = &Violation> {
        self.errors().filter(|v| v.introduced)
    }

    #[must_use]
    pub fn count(&self, kind: ViolationKind) -> usize {
        self.violations.iter().filter(|v| v.kind == kind).count()
    }
}

/// A copper item flattened for checking.
#[derive(Clone, Debug)]
pub struct CopperItem {
    pub item: ItemRef,
    pub hull: Hull,
    pub layers: LayerSet,
    pub net: NetId,
    pub bbox: Rect,
    /// Footprint owning the item (pads only).
    pub footprint: Option<FootprintId>,
    pub introduced: bool,
}

/// Collects every copper item of the board (pads with copper, tracks, vias).
#[must_use]
pub fn copper_items(board: &Board) -> Vec<CopperItem> {
    let moved: Vec<bool> = board.footprints.iter().map(|f| f.placement != f.original).collect();
    let mut v = Vec::with_capacity(
        board.pads.len() + board.tracks.len() + board.arcs.len() + board.vias.len(),
    );
    for id in board.pad_ids() {
        let pad = board.pad(id);
        if pad.layers.is_empty() {
            continue;
        }
        let hull = board.pad_hull(id);
        v.push(CopperItem {
            item: ItemRef::Pad(id),
            bbox: hull.bbox(),
            hull,
            layers: pad.layers,
            net: pad.net,
            footprint: Some(pad.footprint),
            introduced: moved[pad.footprint.idx()],
        });
    }
    for (i, t) in board.tracks.iter().enumerate() {
        let hull = t.hull();
        v.push(CopperItem {
            item: ItemRef::Track(u32::try_from(i).unwrap_or(u32::MAX)),
            bbox: hull.bbox(),
            hull,
            layers: LayerSet::single(t.layer),
            net: t.net,
            footprint: None,
            introduced: t.origin == Origin::Routed,
        });
    }
    // Arcs: one item per covering capsule (a convex superset of the arc copper, INV-DRC-1).
    for (i, arc) in board.arcs.iter().enumerate() {
        for hull in arc.hulls() {
            v.push(CopperItem {
                item: ItemRef::Arc(idx(i)),
                bbox: hull.bbox(),
                hull,
                layers: LayerSet::single(arc.layer),
                net: arc.net,
                footprint: None,
                introduced: arc.origin == Origin::Routed,
            });
        }
    }
    for (i, via) in board.vias.iter().enumerate() {
        let hull = via.hull();
        v.push(CopperItem {
            item: ItemRef::Via(u32::try_from(i).unwrap_or(u32::MAX)),
            bbox: hull.bbox(),
            hull,
            layers: via.layers,
            net: via.net,
            footprint: None,
            introduced: via.origin == Origin::Routed,
        });
    }
    v
}

/// Uniform bucket grid over bounding boxes; yields each candidate pair exactly once.
#[derive(Debug)]
pub struct BucketGrid {
    origin: Point,
    cell: Coord,
    nx: i64,
    ny: i64,
    cells: Vec<Vec<u32>>,
}

impl BucketGrid {
    /// Builds a grid over `boxes` (already inflated by the caller as needed).
    #[must_use]
    pub fn new(boxes: &[Rect], cell: Coord) -> Self {
        let cell = cell.max(1);
        let all = boxes
            .iter()
            .copied()
            .reduce(Rect::union)
            .unwrap_or(Rect::new(Point::ORIGIN, Point::ORIGIN));
        let nx = (all.width() / cell + 1).clamp(1, 4096);
        let ny = (all.height() / cell + 1).clamp(1, 4096);
        let cell = cell.max(all.width() / nx + 1).max(all.height() / ny + 1);
        let mut g = Self { origin: all.min, cell, nx, ny, cells: Vec::new() };
        #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
        {
            g.cells = vec![Vec::new(); (nx * ny) as usize];
        }
        for (i, b) in boxes.iter().enumerate() {
            let (x0, y0, x1, y1) = g.range(*b);
            for cy in y0..=y1 {
                for cx in x0..=x1 {
                    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
                    g.cells[(cy * nx + cx) as usize].push(u32::try_from(i).unwrap_or(u32::MAX));
                }
            }
        }
        g
    }

    fn range(&self, b: Rect) -> (i64, i64, i64, i64) {
        let f = |v: Coord, o: Coord, n: i64| ((v - o).div_euclid(self.cell)).clamp(0, n - 1);
        (
            f(b.min.x, self.origin.x, self.nx),
            f(b.min.y, self.origin.y, self.ny),
            f(b.max.x, self.origin.x, self.nx),
            f(b.max.y, self.origin.y, self.ny),
        )
    }

    fn cell_of(&self, p: Point) -> (i64, i64) {
        let (x, y, _, _) = self.range(Rect::new(p, p));
        (x, y)
    }

    /// Candidate pairs `(i, j)` with `i < j` whose boxes intersect, each exactly once.
    #[must_use]
    pub fn pairs(&self, boxes: &[Rect]) -> Vec<(u32, u32)> {
        let mut out: Vec<(u32, u32)> = (0..self.cells.len())
            .into_par_iter()
            .flat_map_iter(|ci| {
                let cell = &self.cells[ci];
                let mut local = Vec::new();
                #[allow(clippy::cast_possible_wrap)]
                let (cx, cy) = (ci as i64 % self.nx, ci as i64 / self.nx);
                for (k, &i) in cell.iter().enumerate() {
                    for &j in &cell[k + 1..] {
                        let (a, b) = (boxes[i as usize], boxes[j as usize]);
                        if !a.intersects(b) {
                            continue;
                        }
                        // Report the pair only in the cell holding the min corner of the overlap.
                        let m = Point::new(a.min.x.max(b.min.x), a.min.y.max(b.min.y));
                        if self.cell_of(m) == (cx, cy) {
                            local.push(if i < j { (i, j) } else { (j, i) });
                        }
                    }
                }
                local
            })
            .collect();
        out.sort_unstable();
        out
    }

    /// Indices whose boxes intersect `q` (deduplicated, sorted).
    #[must_use]
    pub fn query(&self, q: Rect, boxes: &[Rect]) -> Vec<u32> {
        let (x0, y0, x1, y1) = self.range(q);
        let mut v = Vec::new();
        for cy in y0..=y1 {
            for cx in x0..=x1 {
                #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
                for &i in &self.cells[(cy * self.nx + cx) as usize] {
                    if boxes[i as usize].intersects(q) {
                        v.push(i);
                    }
                }
            }
        }
        v.sort_unstable();
        v.dedup();
        v
    }
}

fn mid(a: Rect, b: Rect) -> Point {
    let lo = Point::new(a.min.x.max(b.min.x), a.min.y.max(b.min.y));
    let hi = Point::new(a.max.x.min(b.max.x), a.max.y.min(b.max.y));
    Point::new(lo.x / 2 + hi.x / 2, lo.y / 2 + hi.y / 2)
}

/// An arc is covered by several capsules; collapse the violations of its pieces that share
/// (kind, items, layer) into the worst one. Input must be sorted; violations without an
/// arc are never touched.
fn dedup_arc_pieces(violations: &mut Vec<Violation>) {
    let involves_arc =
        |v: &Violation| matches!(v.a, ItemRef::Arc(_)) || matches!(v.b, Some(ItemRef::Arc(_)));
    let mut out: Vec<Violation> = Vec::with_capacity(violations.len());
    for v in violations.drain(..) {
        if involves_arc(&v)
            && let Some(last) = out.last_mut()
            && (last.kind, last.a, last.b, last.layer) == (v.kind, v.a, v.b, v.layer)
        {
            if v.actual < last.actual {
                *last = v;
            }
            continue;
        }
        out.push(v);
    }
    *violations = out;
}

/// Rule-evaluation context of a board item (`A.Type`, `A.NetName`, ...).
fn item_ctx(item: ItemRef, net: NetId) -> ItemCtx {
    let kind = match item {
        ItemRef::Pad(_) => ItemType::Pad,
        ItemRef::Via(_) => ItemType::Via,
        ItemRef::Footprint(_) => ItemType::Footprint,
        ItemRef::Track(_)
        | ItemRef::Arc(_)
        | ItemRef::Keepout(_)
        | ItemRef::Zone(_)
        | ItemRef::BoardEdge => ItemType::Track,
    };
    ItemCtx::new(net, kind)
}

fn first_common_layer(a: LayerSet, b: LayerSet) -> Option<LayerId> {
    LayerSet(a.0 & b.0).iter().next()
}

/// Runs the full DRC.
#[must_use]
pub fn check(board: &Board, cfg: &DrcConfig) -> DrcReport {
    let items = copper_items(board);
    let mut violations = Vec::new();
    let r = &board.rules;
    let max_req = r
        .max_clearance()
        .max(r.hole_clearance)
        .max(r.effective_hole_to_hole())
        .max(r.max_custom_min(ConstraintKind::Clearance))
        .max(r.max_custom_min(ConstraintKind::HoleClearance));
    let inflate = (max_req + 1) / 2 + 1;
    let boxes: Vec<Rect> = items.iter().map(|it| it.bbox.inflate(inflate)).collect();
    let grid = BucketGrid::new(&boxes, 2_000_000);

    if cfg.copper {
        violations.extend(check_copper(board, cfg, &items, &grid, &boxes));
    }
    if cfg.edge {
        violations.extend(check_edges(board, &items));
    }
    if cfg.holes {
        violations.extend(check_holes(board, &items));
    }
    if cfg.copper {
        violations.extend(zones::check_fills(board, &items));
        violations.extend(thermal::check_starved_thermals(board, &items));
    }
    if cfg.keepouts {
        violations.extend(check_keepouts(board, &items));
    }
    if cfg.courtyards {
        violations.extend(check_courtyards(board));
    }
    if cfg.sizes {
        violations.extend(check_sizes(board));
    }
    violations.sort_by(|a, b| {
        (a.kind, a.a, a.b, a.layer).cmp(&(b.kind, b.a, b.b, b.layer)).then(a.at.cmp(&b.at))
    });
    dedup_arc_pieces(&mut violations);
    apply_severity(board, &mut violations);
    let connectivity = if cfg.connectivity {
        connectivity::analyse(board, &items)
    } else {
        Connectivity::default()
    };
    DrcReport { violations, connectivity }
}

/// Constraint a violation kind is governed by (`None` = not a custom-rule constraint).
const fn constraint_of(kind: ViolationKind) -> Option<ConstraintKind> {
    Some(match kind {
        ViolationKind::Clearance => ConstraintKind::Clearance,
        ViolationKind::EdgeClearance => ConstraintKind::EdgeClearance,
        ViolationKind::HoleClearance => ConstraintKind::HoleClearance,
        ViolationKind::HoleToHole => ConstraintKind::HoleToHole,
        ViolationKind::TrackWidth | ViolationKind::TrackWidthMax => ConstraintKind::TrackWidth,
        ViolationKind::ViaDiameter | ViolationKind::ViaDiameterMax => ConstraintKind::ViaDiameter,
        ViolationKind::ViaDrill
        | ViolationKind::ViaDrillMax
        | ViolationKind::PadDrill
        | ViolationKind::PadDrillMax => ConstraintKind::HoleSize,
        ViolationKind::AnnularWidth | ViolationKind::AnnularWidthMax => {
            ConstraintKind::AnnularWidth
        }
        _ => return None,
    })
}

/// Net of a board item (`NetId::NONE` for items without one).
fn net_of(board: &Board, item: ItemRef) -> NetId {
    let at = |i: u32| usize::try_from(i).unwrap_or(usize::MAX);
    match item {
        ItemRef::Pad(id) => board.pad(id).net,
        ItemRef::Track(i) => board.tracks.get(at(i)).map_or(NetId::NONE, |t| t.net),
        ItemRef::Arc(i) => board.arcs.get(at(i)).map_or(NetId::NONE, |t| t.net),
        ItemRef::Via(i) => board.vias.get(at(i)).map_or(NetId::NONE, |v| v.net),
        _ => NetId::NONE,
    }
}

/// Re-classifies violations by the `(severity ..)` of the custom rule that governs them:
/// warnings stay in the report as non-errors, `ignore` / `exclusion` drop them.
fn apply_severity(board: &Board, violations: &mut Vec<Violation>) {
    if board.rules.custom.is_empty() {
        return;
    }
    let ctx = |item: ItemRef, layer: Option<LayerId>| {
        let c = item_ctx(item, net_of(board, item));
        layer.map_or(c, |l| c.on(l))
    };
    violations.retain_mut(|v| {
        let Some(kind) = constraint_of(v.kind) else { return true };
        let a = ctx(v.a, v.layer);
        let b = v.b.filter(|b| *b != ItemRef::BoardEdge).map(|b| ctx(b, v.layer));
        v.severity = board.rules.severity_for(board, kind, &a, b.as_ref());
        !v.severity.is_suppressed()
    });
}

fn check_copper(
    board: &Board,
    cfg: &DrcConfig,
    items: &[CopperItem],
    grid: &BucketGrid,
    boxes: &[Rect],
) -> Vec<Violation> {
    grid.pairs(boxes)
        .into_par_iter()
        .filter_map(|(i, j)| {
            let (a, b) = (&items[i as usize], &items[j as usize]);
            if (a.net == b.net && a.net != NetId::NONE) || a.item == b.item {
                return None;
            }
            if !a.layers.intersects(b.layers) {
                return None;
            }
            if cfg.skip_intra_footprint_pads && a.footprint.is_some() && a.footprint == b.footprint
            {
                return None;
            }
            let req = if board.rules.custom.is_empty() {
                board.clearance_between(a.net, b.net)
            } else {
                board.rules.clearance_for_layers(
                    board,
                    &item_ctx(a.item, a.net),
                    &item_ctx(b.item, b.net),
                    LayerSet(a.layers.0 & b.layers.0),
                )
            };
            if !a.hull.clearance_violated(&b.hull, req) {
                return None;
            }
            let overlap = a.hull.clearance_violated(&b.hull, 0);
            Some(Violation {
                kind: if overlap { ViolationKind::Short } else { ViolationKind::Clearance },
                a: a.item,
                b: Some(b.item),
                layer: first_common_layer(a.layers, b.layers),
                at: mid(a.bbox, b.bbox),
                actual: a.hull.distance(&b.hull),
                required: req,
                introduced: a.introduced || b.introduced,
                severity: Severity::Error,
            })
        })
        .collect()
}

fn check_edges(board: &Board, items: &[CopperItem]) -> Vec<Violation> {
    let edge_req = |it: &CopperItem| {
        if board.rules.custom.is_empty() {
            board.rules.edge_clearance
        } else {
            let ctx = item_ctx(it.item, it.net);
            let mut ls = it.layers.iter();
            let first = ls.next().map_or(ctx, |l| ctx.on(l));
            std::iter::once(board.rules.edge_clearance_for(board, &first))
                .chain(ls.map(|l| board.rules.edge_clearance_for(board, &ctx.on(l))))
                .max()
                .unwrap_or(0)
        }
    };
    items
        .par_iter()
        .filter(|it| !board.outline.admits(&it.hull, edge_req(it)))
        .map(|it| Violation {
            kind: ViolationKind::EdgeClearance,
            a: it.item,
            b: Some(ItemRef::BoardEdge),
            layer: it.layers.iter().next(),
            at: it.bbox.center(),
            actual: board
                .outline
                .outer
                .edges()
                .map(|e| it.hull.distance(&Hull::capsule(e.a, e.b, 0)))
                .fold(f64::INFINITY, f64::min),
            required: edge_req(it),
            introduced: it.introduced,
            severity: Severity::Error,
        })
        .collect()
}

struct Hole {
    item: ItemRef,
    hull: Hull,
    /// Copper layers the hole passes through (a blind via's hole ends at its span).
    layers: LayerSet,
    net: NetId,
    introduced: bool,
}

fn holes(board: &Board, items: &[CopperItem]) -> Vec<Hole> {
    let mut v = Vec::new();
    for id in board.pad_ids() {
        let pad = board.pad(id);
        if let Some(d) = pad.drill {
            let moved =
                board.footprint(pad.footprint).placement != board.footprint(pad.footprint).original;
            v.push(Hole {
                item: ItemRef::Pad(id),
                hull: Hull::disk(board.pad_position(id), d.diameter / 2),
                layers: board.all_layers(),
                net: pad.net,
                introduced: moved,
            });
        }
    }
    for (i, via) in board.vias.iter().enumerate() {
        let _ = items;
        v.push(Hole {
            item: ItemRef::Via(u32::try_from(i).unwrap_or(u32::MAX)),
            hull: Hull::disk(via.pos, via.drill / 2),
            layers: via.layers,
            net: via.net,
            introduced: via.origin == Origin::Routed,
        });
    }
    v
}

fn check_holes(board: &Board, items: &[CopperItem]) -> Vec<Violation> {
    let hs = holes(board, items);
    let r = &board.rules;
    let mut out = Vec::new();
    // Hole-to-hole (net independent).
    let reach_h2h = r.effective_hole_to_hole();
    let hb: Vec<Rect> = hs.iter().map(|h| h.hull.bbox().inflate(reach_h2h / 2 + 1)).collect();
    let hg = BucketGrid::new(&hb, 2_000_000);
    for (i, j) in hg.pairs(&hb) {
        let (a, b) = (&hs[i as usize], &hs[j as usize]);
        let req = if r.custom.is_empty() {
            r.hole_to_hole
        } else {
            r.hole_to_hole_for(board, &item_ctx(a.item, a.net), &item_ctx(b.item, b.net))
        };
        if a.hull.clearance_violated(&b.hull, req) {
            out.push(Violation {
                kind: ViolationKind::HoleToHole,
                a: a.item,
                b: Some(b.item),
                layer: None,
                at: mid(a.hull.bbox(), b.hull.bbox()),
                actual: a.hull.distance(&b.hull),
                required: req,
                introduced: a.introduced || b.introduced,
                severity: Severity::Error,
            });
        }
    }
    // Copper-to-hole for copper of other nets.
    let cb: Vec<Rect> = items.iter().map(|it| it.bbox).collect();
    let cg = BucketGrid::new(&cb, 2_000_000);
    for h in &hs {
        let reach = r.hole_clearance.max(r.max_custom_min(ConstraintKind::HoleClearance));
        for k in cg.query(h.hull.bbox().inflate(reach + 1), &cb) {
            let it = &items[k as usize];
            if it.item == h.item
                || (it.net == h.net && it.net != NetId::NONE)
                || !it.layers.intersects(h.layers)
            {
                continue;
            }
            let req = if r.custom.is_empty() {
                r.hole_clearance
            } else {
                r.hole_clearance_for(board, &item_ctx(h.item, h.net), &item_ctx(it.item, it.net))
            };
            if it.hull.clearance_violated(&h.hull, req) {
                out.push(Violation {
                    kind: ViolationKind::HoleClearance,
                    a: h.item,
                    b: Some(it.item),
                    layer: it.layers.iter().next(),
                    at: h.hull.anchor(),
                    actual: it.hull.distance(&h.hull),
                    required: req,
                    introduced: h.introduced || it.introduced,
                    severity: Severity::Error,
                });
            }
        }
    }
    out
}

fn check_keepouts(board: &Board, items: &[CopperItem]) -> Vec<Violation> {
    let mut out = Vec::new();
    for (ki, k) in board.keepouts.iter().enumerate() {
        let kref = ItemRef::Keepout(u32::try_from(ki).unwrap_or(u32::MAX));
        for it in items {
            let forbidden = match it.item {
                ItemRef::Track(_) | ItemRef::Arc(_) => k.no_tracks,
                ItemRef::Via(_) => k.no_vias,
                ItemRef::Pad(_) => k.no_pads,
                _ => false,
            };
            if forbidden && it.layers.intersects(k.layers) && k.area.hull_violates(&it.hull, 0) {
                out.push(Violation {
                    kind: ViolationKind::Keepout,
                    a: it.item,
                    b: Some(kref),
                    layer: first_common_layer(it.layers, k.layers),
                    at: it.bbox.center(),
                    actual: 0.0,
                    required: 0,
                    introduced: it.introduced,
                    severity: Severity::Error,
                });
            }
        }
        if k.no_footprints {
            for (fi, fp) in board.footprints.iter().enumerate() {
                if fp.courtyard().intersects(&k.area) {
                    out.push(Violation {
                        kind: ViolationKind::Keepout,
                        a: ItemRef::Footprint(FootprintId(u32::try_from(fi).unwrap_or(u32::MAX))),
                        b: Some(kref),
                        layer: None,
                        at: fp.placement.pos,
                        actual: 0.0,
                        required: 0,
                        introduced: fp.placement != fp.original,
                        severity: Severity::Error,
                    });
                }
            }
        }
    }
    out
}

/// Courtyard overlap between footprints on the same side (exact polygon test).
#[must_use]
pub fn check_courtyards(board: &Board) -> Vec<Violation> {
    let cys: Vec<_> = board.footprints.iter().map(ferroute_board::Footprint::courtyard).collect();
    let boxes: Vec<Rect> =
        cys.iter().map(|c| c.bbox().unwrap_or(Rect::new(Point::ORIGIN, Point::ORIGIN))).collect();
    let grid = BucketGrid::new(&boxes, 5_000_000);
    grid.pairs(&boxes)
        .into_iter()
        .filter_map(|(i, j)| {
            let (fa, fb) = (&board.footprints[i as usize], &board.footprints[j as usize]);
            if fa.side != fb.side || !cys[i as usize].intersects(&cys[j as usize]) {
                return None;
            }
            Some(Violation {
                kind: ViolationKind::CourtyardOverlap,
                a: ItemRef::Footprint(FootprintId(i)),
                b: Some(ItemRef::Footprint(FootprintId(j))),
                layer: Some(LayerId(if fa.side == Side::Front {
                    0
                } else {
                    board.layer_count().saturating_sub(1)
                })),
                at: mid(boxes[i as usize], boxes[j as usize]),
                actual: 0.0,
                required: 0,
                introduced: fa.placement != fa.original || fb.placement != fb.original,
                severity: Severity::Error,
            })
        })
        .collect()
}

fn check_sizes(board: &Board) -> Vec<Violation> {
    let r = &board.rules;
    let custom = !r.custom.is_empty();
    let track_min = |net: NetId, layer: LayerId| {
        if custom {
            r.min_track_width_for(board, &ItemCtx::new(net, ItemType::Track).on(layer))
        } else {
            r.min_track_width
        }
    };
    let mut out = Vec::new();
    let track_max = |net: NetId, layer: LayerId| {
        if custom {
            let ctx = ItemCtx::new(net, ItemType::Track).on(layer);
            r.max_limit_for(board, ConstraintKind::TrackWidth, &ctx, None)
        } else {
            None
        }
    };
    let mut width_check =
        |item: ItemRef, at: Point, width: Coord, net: NetId, layer: LayerId, origin: Origin| {
            let req = track_min(net, layer);
            if width < req {
                out.push(size_violation(ViolationKind::TrackWidth, item, at, width, req, origin));
            }
            if let Some(mx) = track_max(net, layer).filter(|&mx| width > mx) {
                out.push(size_violation(ViolationKind::TrackWidthMax, item, at, width, mx, origin));
            }
        };
    for (i, t) in board.tracks.iter().enumerate() {
        width_check(ItemRef::Track(idx(i)), t.start, t.width, t.net, t.layer, t.origin);
    }
    for (i, t) in board.arcs.iter().enumerate() {
        width_check(ItemRef::Arc(idx(i)), t.start, t.width, t.net, t.layer, t.origin);
    }
    for (i, v) in board.vias.iter().enumerate() {
        let mut ctx = ItemCtx::new(v.net, ItemType::Via);
        if let Some(l) = v.layers.iter().next() {
            ctx = ctx.on(l);
        }
        let (min_d, min_drill) = if v.kind == ViaKind::Micro {
            // Microvias are laser drilled: they have their own (smaller) minimum sizes.
            (r.min_microvia_diameter, r.min_microvia_drill)
        } else if custom {
            (r.min_via_diameter_for(board, &ctx), r.min_hole_size_for(board, &ctx))
        } else {
            (r.min_via_diameter, r.min_via_drill)
        };
        if !span_fits_kind(v, board.layer_count()) {
            out.push(Violation {
                kind: ViolationKind::ViaSpan,
                a: ItemRef::Via(idx(i)),
                b: None,
                layer: None,
                at: v.pos,
                actual: 0.0,
                required: 0,
                introduced: v.origin == Origin::Routed,
                severity: Severity::Error,
            });
        }
        if v.diameter < min_d {
            out.push(size_violation(
                ViolationKind::ViaDiameter,
                ItemRef::Via(idx(i)),
                v.pos,
                v.diameter,
                min_d,
                v.origin,
            ));
        }
        if v.drill < min_drill {
            out.push(size_violation(
                ViolationKind::ViaDrill,
                ItemRef::Via(idx(i)),
                v.pos,
                v.drill,
                min_drill,
                v.origin,
            ));
        }
        if custom {
            let at = ItemRef::Via(idx(i));
            let max_of = |k| r.max_limit_for(board, k, &ctx, None);
            for (kind, actual, mx) in [
                (ViolationKind::ViaDiameterMax, v.diameter, max_of(ConstraintKind::ViaDiameter)),
                (ViolationKind::ViaDrillMax, v.drill, max_of(ConstraintKind::HoleSize)),
            ] {
                if let Some(mx) = mx.filter(|&m| actual > m) {
                    out.push(size_violation(kind, at, v.pos, actual, mx, v.origin));
                }
            }
            let ring = (v.diameter - v.drill) / 2;
            out.extend(annular_violations(board, &ctx, at, v.pos, ring, v.origin));
        }
    }
    if custom {
        out.extend(pad_hole_violations(board));
    }
    out
}

/// Annular-ring `min` / `max` custom-rule violations for one ring width.
fn annular_violations(
    board: &Board,
    ctx: &ItemCtx,
    item: ItemRef,
    at: Point,
    ring: Coord,
    origin: Origin,
) -> Vec<Violation> {
    let r = &board.rules;
    let mut out = Vec::new();
    let ring_min = r.min_annular_for(board, ctx);
    if ring < ring_min {
        out.push(size_violation(ViolationKind::AnnularWidth, item, at, ring, ring_min, origin));
    }
    if let Some(mx) =
        r.max_limit_for(board, ConstraintKind::AnnularWidth, ctx, None).filter(|&m| ring > m)
    {
        out.push(size_violation(ViolationKind::AnnularWidthMax, item, at, ring, mx, origin));
    }
    out
}

/// Custom `hole_size` / `annular_width` rules for pad holes. Pads of a footprint ferroute
/// moved count as introduced. Pads whose copper is an arbitrary convex polygon have no
/// well-defined ring and are only checked for hole size.
fn pad_hole_violations(board: &Board) -> Vec<Violation> {
    let r = &board.rules;
    let mut out = Vec::new();
    for id in board.pad_ids() {
        let pad = board.pad(id);
        let Some(d) = pad.drill else { continue };
        let fp = board.footprint(pad.footprint);
        let origin = if fp.placement != fp.original { Origin::Routed } else { Origin::Existing };
        let at = board.pad_position(id);
        let item = ItemRef::Pad(id);
        let ctx = ItemCtx::new(pad.net, ItemType::Pad);
        let min = r.custom_min_for(board, ConstraintKind::HoleSize, &ctx, None);
        if d.diameter < min {
            out.push(size_violation(ViolationKind::PadDrill, item, at, d.diameter, min, origin));
        }
        if let Some(mx) =
            r.max_limit_for(board, ConstraintKind::HoleSize, &ctx, None).filter(|&m| d.diameter > m)
        {
            out.push(size_violation(ViolationKind::PadDrillMax, item, at, d.diameter, mx, origin));
        }
        if d.plated
            && let Some(m) = dfm::pad_min_dimension(&pad.shape)
        {
            out.extend(annular_violations(board, &ctx, item, at, (m - d.diameter) / 2, origin));
        }
    }
    out
}

/// `true` when the copper span of `v` is valid for its kind on a board of `layers` copper
/// layers: through = every layer; blind = one outer end, not through; buried = no outer
/// layer; micro = two adjacent layers, one of them outer.
fn span_fits_kind(v: &Via, layers: u8) -> bool {
    let ids: Vec<LayerId> = v.layers.iter().collect();
    let (Some(&lo), Some(&hi)) = (ids.first(), ids.last()) else { return false };
    let contiguous = usize::from(hi.0 - lo.0) + 1 == ids.len();
    let (first, last) = (lo.0 == 0, hi.0 + 1 == layers);
    contiguous
        && match v.kind {
            ViaKind::Through => true, // legacy boards may omit the keyword of partial spans
            ViaKind::Blind => first != last,
            ViaKind::Buried => !first && !last,
            ViaKind::Micro => (first || last) && ids.len() == 2,
        }
}

fn size_violation(
    kind: ViolationKind,
    a: ItemRef,
    at: Point,
    actual: Coord,
    required: Coord,
    origin: Origin,
) -> Violation {
    #[allow(clippy::cast_precision_loss)]
    let actual = actual as f64;
    Violation {
        kind,
        a,
        b: None,
        layer: None,
        at,
        actual,
        required,
        introduced: origin == Origin::Routed,
        severity: Severity::Error,
    }
}

fn idx(i: usize) -> u32 {
    u32::try_from(i).unwrap_or(u32::MAX)
}

/// Groups violations by kind (for summaries).
#[must_use]
pub fn summary(report: &DrcReport) -> FxHashMap<ViolationKind, usize> {
    let mut m = FxHashMap::default();
    for v in &report.violations {
        *m.entry(v.kind).or_insert(0) += 1;
    }
    m
}

#[cfg(test)]
mod custom_tests;
#[cfg(test)]
mod tests;

#[cfg(test)]
mod via_span_tests;
