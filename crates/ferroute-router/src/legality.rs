// SPDX-License-Identifier: AGPL-3.0-only
//! Exact legality of lattice nodes, edges and via sites with respect to **fixed**
//! obstacles (pads, pre-existing copper, holes, board edge, keep-outs).
//!
//! Each site gets a state: [`BLOCKED`], [`FREE`], or `only(net)` — legal only for the
//! net whose own copper is nearby. States are computed per net class because the track
//! width / clearance differ. Every decision uses the exact integer predicates of
//! `ferroute-geom`; an optional accelerator field (GPU or SIMD/rayon CPU) may only
//! *skip* sites that are provably far from every obstacle (it never decides
//! "blocked"), so results are bit-identical with and without acceleration.

use crate::grid::{FORWARD, Grid, NodeIdx};
use ferroute_board::{Board, DesignRules, LayerId, LayerSet, NetId, Origin};
use ferroute_drc::BucketGrid;
use ferroute_geom::{Coord, Hull, Point, Polygon, Rect, point_segment_dist};
use ferroute_gpu::{Capsule, ClearanceBackend, FieldRequest, GridSpec};
use rayon::prelude::*;

/// Site is illegal for every net.
pub const BLOCKED: u32 = 0;
/// Site is legal for every net.
pub const FREE: u32 = 1;

/// State value meaning "legal only for `net`".
#[must_use]
pub const fn only(net: NetId) -> u32 {
    net.0 + 2
}

/// Whether `state` admits copper of `net`.
#[must_use]
pub const fn allows(state: u32, net: NetId) -> bool {
    state == FREE || (net.0 != 0 && state == net.0 + 2)
}

/// Combines a state with a conflict against copper of `net`.
#[must_use]
pub const fn merge(state: u32, net: NetId) -> u32 {
    if net.0 == 0 || state == BLOCKED {
        return BLOCKED;
    }
    if state == FREE || state == net.0 + 2 { net.0 + 2 } else { BLOCKED }
}

/// Kind of fixed obstacle.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ObsKind {
    /// Copper of `net`; `clearance` is the obstacle's own net-class clearance.
    Copper { clearance: Coord, pad: bool },
    /// A drilled hole (pad drill or via drill).
    Hole,
    /// Board outline / cut-out edge segment.
    Edge,
    /// Keep-out area boundary segment.
    KeepoutEdge { no_tracks: bool, no_vias: bool },
}

/// A fixed obstacle.
#[derive(Clone, Debug)]
pub struct Obstacle {
    pub hull: Hull,
    pub layers: LayerSet,
    pub net: NetId,
    pub kind: ObsKind,
}

/// A keep-out area for containment tests.
#[derive(Clone, Debug)]
pub struct KeepArea {
    pub area: Polygon,
    pub layers: LayerSet,
    pub no_tracks: bool,
    pub no_vias: bool,
}

/// One stored copper-pour fill polygon (non-convex, tested exactly).
#[derive(Clone, Debug)]
pub struct FillObs {
    pub poly: Polygon,
    pub layer: LayerId,
    pub net: NetId,
    /// Required gap to other-net copper: the zone clearance or the net class, the larger.
    pub clearance: Coord,
}

/// Geometry of a net class as the router uses it.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ClassGeom {
    pub track_width: Coord,
    pub track_radius: Coord,
    pub clearance: Coord,
    pub via_diameter: Coord,
    pub via_radius: Coord,
    pub via_drill: Coord,
}

/// Spatially indexed fixed obstacles of a board.
#[derive(Debug)]
pub struct Obstacles {
    pub items: Vec<Obstacle>,
    boxes: Vec<Rect>,
    index: BucketGrid,
    pub keep_areas: Vec<KeepArea>,
    /// Stored zone fills (copper pours).
    pub fills: Vec<FillObs>,
    fill_boxes: Vec<Rect>,
    fill_index: BucketGrid,
    pub outline_outer: Polygon,
    pub outline_holes: Vec<Polygon>,
    pub rules: DesignRules,
    /// Largest clearance-type requirement of any obstacle (for query inflation).
    pub reach: Coord,
}

impl Obstacles {
    /// Collects every fixed obstacle of `board` (pads at their current placement and all
    /// copper whose origin is `Existing`).
    #[must_use]
    pub fn from_board(board: &Board) -> Self {
        let all = board.all_layers();
        let class_c = |net: NetId| board.rules.class_of(board, net).clearance;
        let mut items = Vec::new();
        for id in board.pad_ids() {
            let pad = board.pad(id);
            if !pad.layers.is_empty() {
                items.push(Obstacle {
                    hull: board.pad_hull(id),
                    layers: pad.layers,
                    net: pad.net,
                    kind: ObsKind::Copper { clearance: class_c(pad.net), pad: true },
                });
            }
            if let Some(d) = pad.drill {
                items.push(Obstacle {
                    hull: Hull::disk(board.pad_position(id), d.diameter / 2),
                    layers: all,
                    net: pad.net,
                    kind: ObsKind::Hole,
                });
            }
        }
        for t in board.tracks.iter().filter(|t| t.origin == Origin::Existing) {
            items.push(Obstacle {
                hull: t.hull(),
                layers: LayerSet::single(t.layer),
                net: t.net,
                kind: ObsKind::Copper { clearance: class_c(t.net), pad: false },
            });
        }
        for v in board.vias.iter().filter(|v| v.origin == Origin::Existing) {
            items.push(Obstacle {
                hull: v.hull(),
                layers: v.layers,
                net: v.net,
                kind: ObsKind::Copper { clearance: class_c(v.net), pad: false },
            });
            items.push(Obstacle {
                hull: Hull::disk(v.pos, v.drill / 2),
                layers: all,
                net: v.net,
                kind: ObsKind::Hole,
            });
        }
        let edge_poly = std::iter::once(&board.outline.outer).chain(board.outline.holes.iter());
        for poly in edge_poly {
            for e in poly.edges() {
                items.push(Obstacle {
                    hull: Hull::capsule(e.a, e.b, 0),
                    layers: all,
                    net: NetId::NONE,
                    kind: ObsKind::Edge,
                });
            }
        }
        let mut keep_areas = Vec::new();
        for k in &board.keepouts {
            if !(k.no_tracks || k.no_vias) {
                continue;
            }
            for e in k.area.edges() {
                items.push(Obstacle {
                    hull: Hull::capsule(e.a, e.b, 0),
                    layers: k.layers,
                    net: NetId::NONE,
                    kind: ObsKind::KeepoutEdge { no_tracks: k.no_tracks, no_vias: k.no_vias },
                });
            }
            keep_areas.push(KeepArea {
                area: k.area.clone(),
                layers: k.layers,
                no_tracks: k.no_tracks,
                no_vias: k.no_vias,
            });
        }
        let fills: Vec<FillObs> = ferroute_drc::zones::obstacle_fills(board)
            .iter()
            .map(|f| FillObs {
                poly: f.poly.clone(),
                layer: f.layer,
                net: f.net,
                clearance: f.clearance.max(class_c(f.net)),
            })
            .collect();
        let fill_boxes: Vec<Rect> = fills.iter().filter_map(|f| f.poly.bbox()).collect();
        let fill_index = BucketGrid::new(&fill_boxes, 2_000_000);
        let r = &board.rules;
        let reach = r
            .max_clearance()
            .max(r.hole_clearance)
            .max(r.hole_to_hole)
            .max(r.edge_clearance)
            .max(fills.iter().map(|f| f.clearance).max().unwrap_or(0))
            + 1;
        let boxes: Vec<Rect> = items.iter().map(|o| o.hull.bbox()).collect();
        let index = BucketGrid::new(&boxes, 2_000_000);
        Self {
            items,
            boxes,
            index,
            keep_areas,
            fills,
            fill_boxes,
            fill_index,
            outline_outer: board.outline.outer.clone(),
            outline_holes: board.outline.holes.clone(),
            rules: board.rules.clone(),
            reach,
        }
    }

    fn near(&self, q: Rect) -> Vec<u32> {
        if self.items.is_empty() {
            return Vec::new();
        }
        self.index.query(q.inflate(self.reach), &self.boxes)
    }

    /// Indices of fills whose box meets `q` inflated by the reach.
    fn near_fills(&self, q: Rect) -> Vec<u32> {
        if self.fills.is_empty() {
            return Vec::new();
        }
        self.fill_index.query(q.inflate(self.reach), &self.fill_boxes)
    }

    /// `true` when `p` lies inside a stored fill on `layer` (any net).
    #[must_use]
    pub fn in_fill(&self, p: Point, layer: LayerId) -> bool {
        self.near_fills(Rect::new(p, p)).into_iter().any(|i| {
            let f = &self.fills[i as usize];
            f.layer == layer && f.poly.contains(p)
        })
    }

    fn inside_board(&self, p: Point) -> bool {
        self.outline_outer.contains(p) && !self.outline_holes.iter().any(|h| h.contains(p))
    }

    /// Exact state of track copper `h` (a disk or capsule of track radius) on `layer`.
    #[must_use]
    pub fn classify_track(&self, h: &Hull, layer: LayerId, cg: &ClassGeom) -> u32 {
        if !h.pts.iter().all(|&p| self.inside_board(p)) {
            return BLOCKED;
        }
        for k in &self.keep_areas {
            if k.no_tracks && k.layers.contains(layer) && h.pts.iter().any(|&p| k.area.contains(p))
            {
                return BLOCKED;
            }
        }
        let mut state = FREE;
        for i in self.near(h.bbox()) {
            let o = &self.items[i as usize];
            match o.kind {
                ObsKind::Edge => {
                    if h.clearance_violated(&o.hull, self.rules.edge_clearance) {
                        return BLOCKED;
                    }
                }
                ObsKind::KeepoutEdge { no_tracks, .. } => {
                    if no_tracks && o.layers.contains(layer) && h.clearance_violated(&o.hull, 0) {
                        return BLOCKED;
                    }
                }
                ObsKind::Copper { clearance, .. } => {
                    if o.layers.contains(layer)
                        && h.clearance_violated(&o.hull, cg.clearance.max(clearance))
                    {
                        state = merge(state, o.net);
                    }
                }
                ObsKind::Hole => {
                    if h.clearance_violated(&o.hull, self.rules.hole_clearance) {
                        state = merge(state, o.net);
                    }
                }
            }
            if state == BLOCKED {
                return BLOCKED;
            }
        }
        for i in self.near_fills(h.bbox()) {
            let f = &self.fills[i as usize];
            if f.layer == layer && f.poly.hull_violates(h, cg.clearance.max(f.clearance)) {
                state = merge(state, f.net);
                if state == BLOCKED {
                    return BLOCKED;
                }
            }
        }
        state
    }

    /// Exact state of a through via of class `cg` centred at `p` (all layers).
    #[must_use]
    pub fn classify_via(&self, p: Point, cg: &ClassGeom) -> u32 {
        self.classify_via_in(p, cg, None)
    }

    /// Like [`Obstacles::classify_via`], but the via may sit inside its own pad: the pad
    /// copper `own` of net `net` is neither a policy block nor a clearance conflict (the
    /// caller guarantees the via copper lies within it). Every other pad stays blocking.
    #[must_use]
    pub fn classify_via_in_pad(&self, p: Point, cg: &ClassGeom, own: &Hull, net: NetId) -> u32 {
        self.classify_via_in(p, cg, Some((own, net)))
    }

    fn classify_via_in(&self, p: Point, cg: &ClassGeom, own: Option<(&Hull, NetId)>) -> u32 {
        if !self.inside_board(p) {
            return BLOCKED;
        }
        if self.keep_areas.iter().any(|k| k.no_vias && k.area.contains(p)) {
            return BLOCKED;
        }
        let copper = Hull::disk(p, cg.via_radius);
        let drill = Hull::disk(p, cg.via_drill / 2);
        let mut state = FREE;
        for i in self.near(copper.bbox()) {
            let o = &self.items[i as usize];
            match o.kind {
                ObsKind::Edge => {
                    if copper.clearance_violated(&o.hull, self.rules.edge_clearance) {
                        return BLOCKED;
                    }
                }
                ObsKind::KeepoutEdge { no_vias, .. } => {
                    if no_vias && copper.clearance_violated(&o.hull, 0) {
                        return BLOCKED;
                    }
                }
                ObsKind::Copper { pad: true, .. }
                    if own.is_some_and(|(h, n)| o.net == n && &o.hull == h) => {}
                ObsKind::Copper { clearance, pad } => {
                    // Policy: no via-in-pad (any net) unless explicitly allowed above.
                    if pad && copper.clearance_violated(&o.hull, 0) {
                        return BLOCKED;
                    }
                    if copper.clearance_violated(&o.hull, cg.clearance.max(clearance))
                        || drill.clearance_violated(&o.hull, self.rules.hole_clearance)
                    {
                        state = merge(state, o.net);
                    }
                }
                ObsKind::Hole => {
                    if drill.clearance_violated(&o.hull, self.rules.hole_to_hole) {
                        return BLOCKED;
                    }
                    if copper.clearance_violated(&o.hull, self.rules.hole_clearance) {
                        state = merge(state, o.net);
                    }
                }
            }
            if state == BLOCKED {
                return BLOCKED;
            }
        }
        for i in self.near_fills(copper.bbox()) {
            let f = &self.fills[i as usize];
            if f.poly.hull_violates(&copper, cg.clearance.max(f.clearance)) {
                state = merge(state, f.net);
                if state == BLOCKED {
                    return BLOCKED;
                }
            }
        }
        state
    }

    /// Conservative capsules (supersets) of the obstacles relevant on `layer`.
    #[must_use]
    pub fn capsules_on(&self, layer: LayerId) -> Vec<Capsule> {
        let mut caps: Vec<Capsule> = self
            .items
            .iter()
            .filter(|o| match o.kind {
                ObsKind::Copper { .. } | ObsKind::KeepoutEdge { .. } => o.layers.contains(layer),
                ObsKind::Hole | ObsKind::Edge => true,
            })
            .map(|o| bounding_capsule(&o.hull))
            .collect();
        // Fill boundaries (the accelerator only skips sites far from all boundaries; sites
        // *inside* a fill are never skipped, see `compute`).
        for f in self.fills.iter().filter(|f| f.layer == layer) {
            for e in f.poly.edges() {
                caps.push(bounding_capsule(&Hull::capsule(e.a, e.b, 0)));
            }
        }
        caps
    }
}

/// A capsule containing `h` (superset), used for accelerator fields.
#[must_use]
pub fn bounding_capsule(h: &Hull) -> Capsule {
    let c = |p: Point| [p.x, p.y];
    match h.pts.len() {
        0 => Capsule { a: [0, 0], b: [0, 0], radius: h.radius },
        1 => Capsule { a: c(h.pts[0]), b: c(h.pts[0]), radius: h.radius },
        2 => Capsule { a: c(h.pts[0]), b: c(h.pts[1]), radius: h.radius },
        _ => {
            let bb = Rect::from_points(&h.pts).unwrap_or(Rect::new(Point::ORIGIN, Point::ORIGIN));
            let cen = bb.center();
            let (a, b) = if bb.width() >= bb.height() {
                let d = (bb.width() - bb.height()) / 2;
                (Point::new(cen.x - d, cen.y), Point::new(cen.x + d, cen.y))
            } else {
                let d = (bb.height() - bb.width()) / 2;
                (Point::new(cen.x, cen.y - d), Point::new(cen.x, cen.y + d))
            };
            let seg = ferroute_geom::Segment::new(a, b);
            let far = h.pts.iter().map(|&p| point_segment_dist(p, seg)).fold(0.0, f64::max);
            #[allow(clippy::cast_possible_truncation)]
            let r = far.ceil() as Coord + 2 + h.radius;
            Capsule { a: c(a), b: c(b), radius: r }
        }
    }
}

/// Per-class legality tables.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ClassLegality {
    /// Per node.
    pub node: Vec<u32>,
    /// Per node × 4 forward directions ([`FORWARD`]).
    pub edge: Vec<u32>,
    /// Per planar site `(j * nx + i)`.
    pub via: Vec<u32>,
}

impl ClassLegality {
    /// State of the edge leaving `n` in forward slot `slot`.
    #[must_use]
    pub fn edge_state(&self, n: NodeIdx, slot: usize) -> u32 {
        self.edge[n as usize * 4 + slot]
    }
}

/// Optional accelerator: per-layer approximate distance field to obstacle supersets.
#[derive(Debug)]
pub struct AccelField {
    pub per_layer: Vec<Vec<f32>>,
    pub bound: f64,
    pub backend: &'static str,
}

impl AccelField {
    /// Computes the field with `backend` for every grid layer.
    ///
    /// # Errors
    /// Propagates backend errors.
    pub fn compute(
        grid: &Grid,
        obs: &Obstacles,
        backend: &dyn ClearanceBackend,
        cutoff: Coord,
    ) -> Result<Self, ferroute_gpu::ComputeError> {
        let spec = GridSpec {
            origin: [grid.origin.x, grid.origin.y],
            pitch: grid.pitch,
            nx: grid.nx,
            ny: grid.ny,
        };
        let mut per_layer = Vec::with_capacity(grid.layers.len());
        let mut bound: f64 = 0.0;
        for &l in &grid.layers {
            let caps = obs.capsules_on(l);
            let f =
                backend.clearance_field(&FieldRequest { grid: spec, obstacles: &caps, cutoff })?;
            bound = bound.max(f.error_bound_nm);
            per_layer.push(f.values);
        }
        Ok(Self { per_layer, bound, backend: backend.name() })
    }

    /// `true` when the node is provably farther than `threshold` from every obstacle.
    fn far(&self, grid: &Grid, n: NodeIdx, threshold: f64) -> bool {
        let (_, _, l) = grid.coords(n);
        let v = f64::from(self.per_layer[l as usize][grid.planar(n) as usize]);
        v > threshold + self.bound + 1.0
    }
}

/// Computes the legality tables of one class.
#[must_use]
pub fn compute(
    grid: &Grid,
    obs: &Obstacles,
    cg: &ClassGeom,
    accel: Option<&AccelField>,
) -> ClassLegality {
    let reach = {
        let r = &obs.rules;
        r.max_clearance().max(cg.clearance).max(r.hole_clearance).max(r.edge_clearance)
    };
    #[allow(clippy::cast_precision_loss)]
    let node_thr = (cg.track_radius + reach) as f64;
    #[allow(clippy::cast_precision_loss)]
    let edge_thr = node_thr + grid.pitch as f64 * std::f64::consts::FRAC_1_SQRT_2;
    let nx = grid.nx as usize;
    let mut node = vec![BLOCKED; grid.len()];
    node.par_chunks_mut(nx).enumerate().for_each(|(row, out)| {
        let row = u32::try_from(row).unwrap_or(u32::MAX);
        let (j, l) = (row % grid.ny, row / grid.ny);
        let layer = grid.layers[l as usize];
        for (i, s) in out.iter_mut().enumerate() {
            let i = u32::try_from(i).unwrap_or(u32::MAX);
            let p = grid.pos_ij(i, j);
            let n = grid.idx(i, j, l);
            *s = if accel.is_some_and(|a| a.far(grid, n, node_thr)) {
                // Provably far from all obstacle *boundaries*: only containment matters.
                if obs.inside_board(p)
                    && !obs.in_fill(p, layer)
                    && !obs
                        .keep_areas
                        .iter()
                        .any(|k| k.no_tracks && k.layers.contains(layer) && k.area.contains(p))
                {
                    FREE
                } else {
                    BLOCKED
                }
            } else {
                obs.classify_track(&Hull::disk(p, cg.track_radius), layer, cg)
            };
        }
    });
    let mut edge = vec![BLOCKED; grid.len() * 4];
    edge.par_chunks_mut(nx * 4).enumerate().for_each(|(row, out)| {
        let row = u32::try_from(row).unwrap_or(u32::MAX);
        let (j, l) = (row % grid.ny, row / grid.ny);
        let layer = grid.layers[l as usize];
        for i in 0..grid.nx {
            let a = grid.idx(i, j, l);
            if node[a as usize] == BLOCKED {
                continue;
            }
            for (slot, d) in FORWARD.iter().enumerate() {
                let Some(b) = grid.step(a, *d) else { continue };
                let (sa, sb) = (node[a as usize], node[b as usize]);
                if sa == BLOCKED || sb == BLOCKED {
                    continue;
                }
                let st = if sa == FREE
                    && sb == FREE
                    && accel.is_some_and(|f| f.far(grid, a, edge_thr) && f.far(grid, b, edge_thr))
                {
                    FREE
                } else {
                    obs.classify_track(
                        &Hull::capsule(grid.pos(a), grid.pos(b), cg.track_radius),
                        layer,
                        cg,
                    )
                };
                out[i as usize * 4 + slot] = st;
            }
        }
    });
    let via: Vec<u32> = (0..u32::try_from(grid.plane()).unwrap_or(u32::MAX))
        .into_par_iter()
        .map(|pl| {
            let (i, j) = (pl % grid.nx, pl / grid.nx);
            obs.classify_via(grid.pos_ij(i, j), cg)
        })
        .collect();
    ClassLegality { node, edge, via }
}
