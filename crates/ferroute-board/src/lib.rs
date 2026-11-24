// SPDX-License-Identifier: AGPL-3.0-only
//! # ferroute-board
//!
//! Transport-agnostic board data model. Everything the router, nudger and DRC oracle
//! need is expressed here in exact nanometre geometry; KiCad-specific syntax lives in
//! `ferroute-kicad` (files) and `ferroute-ipc` (live IPC), which both *produce* a
//! [`Board`] and *consume* a [`BoardEdit`].
//!
//! Identity: every collection is a dense `Vec` indexed by a typed id
//! ([`NetId`], [`FootprintId`], [`PadId`], ...). Ids are stable for the lifetime of a
//! `Board` value; edits never renumber.

use ferroute_geom::{Angle, Coord, Hull, Point, Polygon, Rect};
use serde::{Deserialize, Serialize};

pub mod builder;
pub mod constraints;
pub mod custom;
pub mod netrules;
pub mod pairs;
pub mod rules;
pub mod text;
pub use pairs::DiffPair;
pub use rules::{DesignRules, NetClass};
pub use text::glob;

macro_rules! id_type {
    ($(#[$m:meta])* $name:ident) => {
        $(#[$m])*
        #[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
        pub struct $name(pub u32);
        impl $name {
            #[must_use]
            pub const fn idx(self) -> usize {
                self.0 as usize
            }
        }
    };
}

id_type!(
    /// Index into [`Board::nets`]. `NetId(0)` is always the "no net" net.
    NetId
);
id_type!(
    /// Index into [`Board::footprints`].
    FootprintId
);
id_type!(
    /// Index into [`Board::pads`].
    PadId
);
id_type!(
    /// Index into [`Board::tracks`].
    TrackId
);
id_type!(
    /// Index into [`Board::vias`].
    ViaId
);

impl NetId {
    pub const NONE: Self = Self(0);
}

/// Copper layer index in physical stack order: `0` = top (`F.Cu`), last = bottom (`B.Cu`).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct LayerId(pub u8);

/// A set of copper layers as a bitmask (bit `i` = [`LayerId(i)`]). Supports up to 64 layers.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct LayerSet(pub u64);

impl LayerSet {
    pub const EMPTY: Self = Self(0);

    #[must_use]
    pub const fn single(l: LayerId) -> Self {
        Self(1 << l.0)
    }

    /// All layers `0..n`.
    #[must_use]
    pub const fn all(n: u8) -> Self {
        if n >= 64 { Self(u64::MAX) } else { Self((1u64 << n) - 1) }
    }

    /// Inclusive span `a..=b` (order-independent).
    #[must_use]
    pub fn span(a: LayerId, b: LayerId) -> Self {
        let (lo, hi) = if a.0 <= b.0 { (a.0, b.0) } else { (b.0, a.0) };
        let mut s = Self::EMPTY;
        for l in lo..=hi {
            s = s.with(LayerId(l));
        }
        s
    }

    #[must_use]
    pub const fn with(self, l: LayerId) -> Self {
        Self(self.0 | (1 << l.0))
    }

    #[must_use]
    pub const fn contains(self, l: LayerId) -> bool {
        self.0 & (1 << l.0) != 0
    }

    #[must_use]
    pub const fn intersects(self, o: Self) -> bool {
        self.0 & o.0 != 0
    }

    #[must_use]
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    pub fn iter(self) -> impl Iterator<Item = LayerId> {
        #[allow(clippy::cast_possible_truncation)]
        (0..64u8).filter(move |&i| self.0 & (1 << i) != 0).map(LayerId)
    }
}

/// A copper layer of the stack-up.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CopperLayer {
    /// Canonical KiCad name, e.g. `F.Cu`, `In1.Cu`, `B.Cu`.
    pub name: String,
    /// Whether the router may place tracks on this layer.
    pub routable: bool,
}

/// An electrical net.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Net {
    pub name: String,
    /// Index into [`DesignRules::classes`].
    pub class: u16,
    /// Electrical constraints (current, frequency, impedance, length, vias).
    #[serde(default)]
    pub constraints: constraints::NetConstraints,
}

impl Net {
    /// A net without electrical constraints.
    #[must_use]
    pub fn new(name: impl Into<String>, class: u16) -> Self {
        Self { name: name.into(), class, constraints: constraints::NetConstraints::default() }
    }
}

/// Which side of the board a footprint is mounted on.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Side {
    Front,
    Back,
}

/// A rigid placement (translation + rotation about the anchor).
#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Placement {
    pub pos: Point,
    pub rot: Angle,
}

impl Placement {
    /// Maps a footprint-local point to board coordinates (KiCad: rotate then translate).
    #[must_use]
    pub fn apply(self, local: Point) -> Point {
        local.rotated(self.rot) + self.pos
    }
}

/// How the placement optimiser may treat a footprint.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Mobility {
    /// Locked in KiCad (`(locked yes)`): never moved, never rotated. Hard invariant.
    Locked,
    /// Not locked, but not opted in for movement (default) or auto-fixed because it is
    /// already connected to pre-existing copper.
    Fixed,
    /// May be translated by at most `max_displacement` (Euclidean, from the *original*
    /// position) and rotated in 90° steps if `allow_rotation`.
    Movable { max_displacement: Coord, allow_rotation: bool },
}

impl Mobility {
    #[must_use]
    pub const fn is_movable(self) -> bool {
        matches!(self, Self::Movable { .. })
    }
}

/// A footprint (component).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Footprint {
    pub reference: String,
    /// Library id, e.g. `Resistor_SMD:R_0603_1608Metric`.
    pub lib_id: String,
    pub placement: Placement,
    /// Placement as loaded (displacement is measured against this).
    pub original: Placement,
    pub side: Side,
    pub locked: bool,
    pub mobility: Mobility,
    /// Courtyard in footprint-local coordinates (union approximated by convex hull or
    /// pad bounding box + margin when the footprint has no courtyard).
    pub courtyard_local: Polygon,
    pub pads: Vec<PadId>,
}

impl Footprint {
    /// Courtyard in board coordinates for the current placement.
    #[must_use]
    pub fn courtyard(&self) -> Polygon {
        self.courtyard_at(self.placement)
    }

    /// Courtyard in board coordinates for an arbitrary placement.
    #[must_use]
    pub fn courtyard_at(&self, pl: Placement) -> Polygon {
        Polygon::new(self.courtyard_local.pts.iter().map(|&p| pl.apply(p)).collect())
    }

    /// Euclidean displacement from the original placement in nm.
    #[must_use]
    pub fn displacement(&self) -> f64 {
        self.placement.pos.dist(self.original.pos)
    }
}

/// Pad copper shape in pad-local coordinates (centred at the pad origin, unrotated).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PadShape {
    Circle {
        diameter: Coord,
    },
    Rect {
        w: Coord,
        h: Coord,
    },
    /// Stadium; the long axis is the larger of `w`/`h`.
    Oval {
        w: Coord,
        h: Coord,
    },
    RoundRect {
        w: Coord,
        h: Coord,
        radius: Coord,
    },
    /// Arbitrary shape represented by a conservative convex polygon (superset).
    Convex {
        pts: Vec<Point>,
    },
}

impl PadShape {
    /// Local-frame hull (centred at origin, unrotated).
    #[must_use]
    pub fn local_hull(&self) -> Hull {
        let half = |v: Coord| v / 2;
        match *self {
            Self::Circle { diameter } => Hull::disk(Point::ORIGIN, half(diameter)),
            Self::Rect { w, h } => Hull::polygon(&rect_corners(w, h), 0),
            Self::Oval { w, h } => {
                if w >= h {
                    let d = half(w - h);
                    Hull::capsule(Point::new(-d, 0), Point::new(d, 0), half(h))
                } else {
                    let d = half(h - w);
                    Hull::capsule(Point::new(0, -d), Point::new(0, d), half(w))
                }
            }
            Self::RoundRect { w, h, radius } => {
                let r = radius.min(half(w.min(h))).max(0);
                Hull::polygon(&rect_corners(w - 2 * r, h - 2 * r), r)
            }
            Self::Convex { ref pts } => Hull::polygon(pts, 0),
        }
    }
}

fn rect_corners(w: Coord, h: Coord) -> [Point; 4] {
    // Use ceil on odd sizes so the hull is never smaller than the true pad.
    let (hw, hh) = ((w + 1) / 2, (h + 1) / 2);
    Rect::new(Point::new(-hw, -hh), Point::new(hw, hh)).corners()
}

/// Plated or non-plated drill hole.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Drill {
    pub diameter: Coord,
    pub plated: bool,
}

/// A pad. Geometry is stored footprint-local so placement edits stay exact.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pad {
    pub footprint: FootprintId,
    /// Pad "number" (name), e.g. `1`, `A3`.
    pub number: String,
    pub net: NetId,
    /// Position relative to the footprint anchor in the footprint's *unrotated* frame.
    pub local_pos: Point,
    /// Pad rotation relative to the footprint (KiCad stores the absolute angle; the
    /// reader subtracts the footprint rotation).
    pub local_rot: Angle,
    pub shape: PadShape,
    /// Copper layers this pad exists on.
    pub layers: LayerSet,
    pub drill: Option<Drill>,
}

impl Pad {
    /// Absolute pad centre for the given footprint placement.
    #[must_use]
    pub fn position(&self, fp: Placement) -> Point {
        fp.apply(self.local_pos)
    }

    /// Absolute copper hull for the given footprint placement.
    #[must_use]
    pub fn hull(&self, fp: Placement) -> Hull {
        let rot = fp.rot.add(self.local_rot);
        let centre = self.position(fp);
        let local = self.shape.local_hull();
        Hull {
            pts: local.pts.iter().map(|&p| p.rotated(rot) + centre).collect(),
            radius: local.radius,
        }
    }
}

/// Where a copper item came from.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Origin {
    /// Present in the input design; never deleted by ferroute.
    Existing,
    /// Created by ferroute in this run.
    Routed,
}

/// A straight track segment.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Track {
    pub start: Point,
    pub end: Point,
    pub width: Coord,
    pub layer: LayerId,
    pub net: NetId,
    pub locked: bool,
    pub origin: Origin,
}

impl Track {
    #[must_use]
    pub fn hull(&self) -> Hull {
        Hull::capsule(self.start, self.end, self.width / 2)
    }
}

/// Maximum chord sagitta (nm) used when covering arc copper with capsules.
pub const ARC_MAX_SAG: f64 = 2_000.0;

/// A circular-arc track (KiCad `(arc (start) (mid) (end))`). Its copper is covered by
/// [`ArcTrack::hulls`], a convex superset (INV-DRC-1).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ArcTrack {
    pub start: Point,
    pub mid: Point,
    pub end: Point,
    pub width: Coord,
    pub layer: LayerId,
    pub net: NetId,
    pub locked: bool,
    pub origin: Origin,
}

impl ArcTrack {
    #[must_use]
    pub const fn geom(&self) -> ferroute_geom::arc::Arc {
        ferroute_geom::arc::Arc::new(self.start, self.mid, self.end)
    }

    /// Capsules whose union covers the arc's copper.
    #[must_use]
    pub fn hulls(&self) -> Vec<Hull> {
        self.geom().cover(self.width, ARC_MAX_SAG)
    }

    /// Centre-line length (nm).
    #[must_use]
    pub fn length(&self) -> f64 {
        self.geom().length()
    }
}

/// Kind of a via (KiCad `VIATYPE`).
#[derive(
    Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
)]
#[serde(rename_all = "lowercase")]
pub enum ViaKind {
    /// Drilled through the whole board (no keyword in the `.kicad_pcb`).
    #[default]
    Through,
    /// Outer layer to an inner layer (`blind`).
    Blind,
    /// Inner layer to inner layer (`buried`).
    Buried,
    /// Laser microvia between adjacent layers, starting at an outer layer (`micro`).
    Micro,
}

impl ViaKind {
    pub const ALL: [Self; 4] = [Self::Through, Self::Blind, Self::Buried, Self::Micro];

    /// Lower-case name (`through`, `blind`, `buried`, `micro`).
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Through => "through",
            Self::Blind => "blind",
            Self::Buried => "buried",
            Self::Micro => "micro",
        }
    }

    /// Parses [`ViaKind::name`].
    #[must_use]
    pub fn parse(s: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|k| k.name() == s)
    }
}

/// Which via kinds the router may create (default: through vias only). Blind, buried and
/// micro vias need HDI fabrication (extra cost), hence opt-in.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ViaPolicy {
    pub through: bool,
    pub blind: bool,
    pub buried: bool,
    pub micro: bool,
    /// Micro vias may sit inside same-net SMD pads (filled and capped).
    pub micro_in_pad: bool,
}

impl Default for ViaPolicy {
    fn default() -> Self {
        Self { through: true, blind: false, buried: false, micro: false, micro_in_pad: false }
    }
}

impl ViaPolicy {
    /// `true` when only through vias are allowed.
    #[must_use]
    pub const fn through_only(&self) -> bool {
        self.through && !self.blind && !self.buried && !self.micro
    }

    /// `true` when any non-through kind is allowed.
    #[must_use]
    pub const fn any_span(&self) -> bool {
        self.blind || self.buried || self.micro
    }

    /// Whether `kind` may be created.
    #[must_use]
    pub const fn allows(&self, kind: ViaKind) -> bool {
        match kind {
            ViaKind::Through => self.through,
            ViaKind::Blind => self.blind,
            ViaKind::Buried => self.buried,
            ViaKind::Micro => self.micro,
        }
    }

    /// Builds a policy from kind names (`through`, `blind`, `buried`, `micro`).
    ///
    /// # Errors
    /// Returns the offending name when it is not a via kind.
    pub fn from_names<'a>(names: impl IntoIterator<Item = &'a str>) -> Result<Self, String> {
        let mut p = Self { through: false, ..Self::default() };
        for n in names {
            match ViaKind::parse(n.trim()) {
                Some(ViaKind::Through) => p.through = true,
                Some(ViaKind::Blind) => p.blind = true,
                Some(ViaKind::Buried) => p.buried = true,
                Some(ViaKind::Micro) => p.micro = true,
                None => return Err(n.to_owned()),
            }
        }
        Ok(p)
    }
}

/// A via (through, blind, buried or micro): `layers` is its authoritative copper span
/// (contiguous copper layers); copper, hole and connectivity exist only on those layers.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Via {
    pub pos: Point,
    pub diameter: Coord,
    pub drill: Coord,
    pub layers: LayerSet,
    /// Kind keyword of the via; defaults to [`ViaKind::Through`].
    #[serde(default)]
    pub kind: ViaKind,
    pub net: NetId,
    pub locked: bool,
    pub origin: Origin,
}

impl Via {
    #[must_use]
    pub fn hull(&self) -> Hull {
        Hull::disk(self.pos, self.diameter / 2)
    }
}

/// A rule area ("keep-out zone").
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Keepout {
    pub area: Polygon,
    pub layers: LayerSet,
    pub no_tracks: bool,
    pub no_vias: bool,
    pub no_pads: bool,
    pub no_footprints: bool,
}

/// How a copper zone joins pads of its own net.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum PadConnection {
    /// Pads are fused with the fill.
    Solid,
    /// Thermal reliefs (KiCad default): pads stay electrically connected.
    #[default]
    ThermalRelief,
    /// Thermal reliefs on through-hole pads only; SMD pads are not connected.
    ThruHoleOnly,
    /// Pads are isolated from the fill.
    None,
}

impl PadConnection {
    /// `true` when a pad (through-hole or not) of the zone's net joins the fill.
    #[must_use]
    pub fn joins(self, through_hole: bool) -> bool {
        match self {
            Self::Solid | Self::ThermalRelief => true,
            Self::ThruHoleOnly => through_hole,
            Self::None => false,
        }
    }
}

/// The copper KiCad stored for one zone on one layer (possibly non-convex polygons;
/// holes are encoded KiCad-style as bridged outlines).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ZoneFill {
    pub layer: LayerId,
    pub polys: Vec<Polygon>,
}

/// KiCad's `(island_removal_mode N)`: what happens to fill pieces connected to nothing.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum IslandRemoval {
    /// Mode 0 (KiCad default): always remove unconnected islands.
    #[default]
    Always,
    /// Mode 1: keep every island.
    Never,
    /// Mode 2: remove unconnected islands smaller than this area (nm²).
    BelowArea(i128),
}

/// A copper pour. The stored fill is the copper; ferroute refills it after routing only
/// when `yields` is set (see `ferroute_drc::refill`).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Zone {
    pub net: NetId,
    pub layers: LayerSet,
    pub outline: Polygon,
    pub fills: Vec<ZoneFill>,
    /// Zone clearance to other-net copper.
    pub clearance: Coord,
    pub min_thickness: Coord,
    pub priority: u32,
    pub pad_connection: PadConnection,
    /// Unconnected-island policy used when the fill is recomputed.
    #[serde(default)]
    pub islands: IslandRemoval,
    /// The pipeline will refill this zone around other-net copper: its stored fill is then
    /// no obstacle for the router and is not clearance-checked (it is replaced before the
    /// final DRC). Never read from or written to a file.
    #[serde(default)]
    pub yields: bool,
}

impl Zone {
    /// `true` when the zone carries no stored copper (never filled).
    #[must_use]
    pub fn is_unfilled(&self) -> bool {
        self.fills.iter().all(|f| f.polys.is_empty())
    }

    /// Stored fill polygons as `(layer, polygon)`.
    pub fn fill_polys(&self) -> impl Iterator<Item = (LayerId, &Polygon)> {
        self.fills.iter().flat_map(|f| f.polys.iter().map(move |p| (f.layer, p)))
    }
}

/// Board outline: outer boundary plus optional cut-outs (all simple polygons).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Outline {
    pub outer: Polygon,
    pub holes: Vec<Polygon>,
}

impl Outline {
    /// Exact: is `h` inside the board with at least `edge_clearance` to every edge?
    #[must_use]
    pub fn admits(&self, h: &Hull, edge_clearance: Coord) -> bool {
        self.outer.contains_hull_with_clearance(h, edge_clearance)
            && !self.holes.iter().any(|hole| hole.hull_violates(h, edge_clearance))
    }

    #[must_use]
    pub fn bbox(&self) -> Option<Rect> {
        self.outer.bbox()
    }
}

/// The complete board.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Board {
    pub copper_layers: Vec<CopperLayer>,
    pub nets: Vec<Net>,
    pub footprints: Vec<Footprint>,
    pub pads: Vec<Pad>,
    pub tracks: Vec<Track>,
    /// Arc tracks created by ferroute (input arcs are read as conservative chord tracks).
    #[serde(default)]
    pub arcs: Vec<ArcTrack>,
    pub vias: Vec<Via>,
    pub keepouts: Vec<Keepout>,
    /// Copper pours (stored fills only).
    #[serde(default)]
    pub zones: Vec<Zone>,
    pub outline: Outline,
    pub rules: DesignRules,
    #[serde(default)]
    pub stackup: constraints::Stackup,
    /// Explicit length-matching groups (on top of the legacy per-net `match_group` fields).
    #[serde(default)]
    pub match_groups: Vec<constraints::MatchGroup>,
    /// Differential pairs routed as coupled units.
    #[serde(default)]
    pub diff_pairs: Vec<pairs::DiffPair>,
}

/// Errors from [`Board::validate`].
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum BoardError {
    #[error("board has no copper layers")]
    NoCopperLayers,
    #[error("board has more than 64 copper layers")]
    TooManyLayers,
    #[error("net 0 must be the unnamed 'no net' net")]
    MissingNoNet,
    #[error("board outline is missing or degenerate")]
    NoOutline,
    #[error("coordinate out of range (|c| > 2^30 nm) in {0}")]
    OutOfRange(String),
    #[error("dangling reference: {0}")]
    Dangling(String),
}

impl Board {
    #[must_use]
    pub fn layer_count(&self) -> u8 {
        u8::try_from(self.copper_layers.len()).unwrap_or(u8::MAX)
    }

    #[must_use]
    pub fn all_layers(&self) -> LayerSet {
        LayerSet::all(self.layer_count())
    }

    #[must_use]
    pub fn footprint(&self, id: FootprintId) -> &Footprint {
        &self.footprints[id.idx()]
    }

    #[must_use]
    pub fn pad(&self, id: PadId) -> &Pad {
        &self.pads[id.idx()]
    }

    /// Absolute hull of a pad at its footprint's current placement.
    #[must_use]
    pub fn pad_hull(&self, id: PadId) -> Hull {
        let pad = self.pad(id);
        pad.hull(self.footprint(pad.footprint).placement)
    }

    /// Allowed entry/exit directions of a pad, as integer unit steps `(ux, uy)`:
    /// * circular pads (and therefore through-hole pads, vias and holes): all 8 octilinear
    ///   directions through the centre;
    /// * every other shape: the 4 pad axes, i.e. perpendicular to each side through its
    ///   midpoint (short and long sides), following the pad rotation. Pads rotated by a
    ///   non-octilinear angle fall back to the 8 octilinear directions.
    #[must_use]
    pub fn pad_entry_dirs(&self, id: PadId) -> Vec<(i64, i64)> {
        const ALL8: [(i64, i64); 8] =
            [(1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1)];
        let p = self.pad(id);
        if matches!(p.shape, PadShape::Circle { .. }) {
            return ALL8.to_vec();
        }
        let rot = self.footprint(p.footprint).placement.rot.add(p.local_rot);
        let mut out = Vec::with_capacity(4);
        for axis in [(1, 0), (0, 1), (-1, 0), (0, -1)] {
            let v = Point::new(axis.0 * 1_000_000, axis.1 * 1_000_000).rotated(rot);
            if !v.is_octilinear() {
                return ALL8.to_vec();
            }
            out.push((v.x.signum(), v.y.signum()));
        }
        out
    }

    #[must_use]
    pub fn pad_position(&self, id: PadId) -> Point {
        let pad = self.pad(id);
        pad.position(self.footprint(pad.footprint).placement)
    }

    /// The SMD pad of `via`'s net that is centred on the via and whose copper fully
    /// contains the via copper (a *via-in-pad*, which needs filled and capped vias).
    /// Through-hole pads never qualify.
    #[must_use]
    pub fn via_in_pad(&self, via: &Via) -> Option<PadId> {
        self.pad_ids().find(|&id| {
            let pad = self.pad(id);
            pad.net == via.net
                && pad.drill.is_none()
                && !pad.layers.is_empty()
                && self.pad_position(id) == via.pos
                && self.pad_hull(id).contains_disk(via.pos, via.diameter / 2)
        })
    }

    pub fn pad_ids(&self) -> impl Iterator<Item = PadId> {
        #[allow(clippy::cast_possible_truncation)]
        (0..self.pads.len() as u32).map(PadId)
    }

    pub fn net_ids(&self) -> impl Iterator<Item = NetId> {
        #[allow(clippy::cast_possible_truncation)]
        (0..self.nets.len() as u32).map(NetId)
    }

    /// Pads grouped by net (index = `NetId`). Net 0 is included but usually ignored.
    #[must_use]
    pub fn pads_by_net(&self) -> Vec<Vec<PadId>> {
        let mut v = vec![Vec::new(); self.nets.len()];
        for id in self.pad_ids() {
            v[self.pad(id).net.idx()].push(id);
        }
        v
    }

    #[must_use]
    pub fn net_by_name(&self, name: &str) -> Option<NetId> {
        #[allow(clippy::cast_possible_truncation)]
        self.nets.iter().position(|n| n.name == name).map(|i| NetId(i as u32))
    }

    /// Clearance required between items of nets `a` and `b` (KiCad semantics: the
    /// larger of the two net-class clearances).
    #[must_use]
    pub fn clearance_between(&self, a: NetId, b: NetId) -> Coord {
        self.rules.class_of(self, a).clearance.max(self.rules.class_of(self, b).clearance)
    }

    /// Validates structural invariants (INV-BOARD-*): ids in range, coordinates within
    /// INV-GEOM-1 bounds, net 0 present, outline present.
    ///
    /// # Errors
    /// The first violated invariant.
    pub fn validate(&self) -> Result<(), BoardError> {
        if self.copper_layers.is_empty() {
            return Err(BoardError::NoCopperLayers);
        }
        if self.copper_layers.len() > 64 {
            return Err(BoardError::TooManyLayers);
        }
        if self.nets.first().is_none_or(|n| !n.name.is_empty()) {
            return Err(BoardError::MissingNoNet);
        }
        if self.outline.outer.pts.len() < 3 || self.outline.outer.area2() == 0 {
            return Err(BoardError::NoOutline);
        }
        let n_nets = self.nets.len();
        let layers = self.all_layers();
        for (i, fp) in self.footprints.iter().enumerate() {
            if !fp.placement.pos.in_bounds() {
                return Err(BoardError::OutOfRange(format!("footprint {}", fp.reference)));
            }
            for &pid in &fp.pads {
                if pid.idx() >= self.pads.len() || self.pads[pid.idx()].footprint.idx() != i {
                    return Err(BoardError::Dangling(format!("pad {pid:?} of {}", fp.reference)));
                }
            }
        }
        for (i, pad) in self.pads.iter().enumerate() {
            if pad.footprint.idx() >= self.footprints.len() || pad.net.idx() >= n_nets {
                return Err(BoardError::Dangling(format!("pad #{i}")));
            }
            if !self.pad_hull(PadId(u32::try_from(i).unwrap_or(u32::MAX))).in_bounds() {
                return Err(BoardError::OutOfRange(format!("pad #{i}")));
            }
        }
        for (i, t) in self.tracks.iter().enumerate() {
            if t.net.idx() >= n_nets || !layers.contains(t.layer) {
                return Err(BoardError::Dangling(format!("track #{i}")));
            }
            if !t.hull().in_bounds() {
                return Err(BoardError::OutOfRange(format!("track #{i}")));
            }
        }
        for (i, v) in self.vias.iter().enumerate() {
            if v.net.idx() >= n_nets {
                return Err(BoardError::Dangling(format!("via #{i}")));
            }
            if !v.hull().in_bounds() {
                return Err(BoardError::OutOfRange(format!("via #{i}")));
            }
        }
        if !self.outline.outer.pts.iter().all(|p| p.in_bounds()) {
            return Err(BoardError::OutOfRange("outline".into()));
        }
        Ok(())
    }

    /// Applies an edit produced by the router / nudger.
    pub fn apply(&mut self, edit: &BoardEdit) {
        for m in &edit.moves {
            self.footprints[m.footprint.idx()].placement = m.to;
        }
        self.tracks.extend(edit.new_tracks.iter().cloned());
        self.arcs.extend(edit.new_arcs.iter().cloned());
        self.vias.extend(edit.new_vias.iter().cloned());
        for (zi, fills) in &edit.zone_fills {
            if let Some(z) = self.zones.get_mut(*zi as usize) {
                z.fills.clone_from(fills);
                z.yields = false;
            }
        }
    }
}

/// A footprint move.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FootprintMove {
    pub footprint: FootprintId,
    pub from: Placement,
    pub to: Placement,
}

/// The output of a ferroute run: purely additive copper plus footprint moves.
/// INV-EDIT-1: an edit never deletes or modifies `Origin::Existing` items and never
/// moves a footprint whose mobility is not `Movable`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct BoardEdit {
    pub moves: Vec<FootprintMove>,
    pub new_tracks: Vec<Track>,
    #[serde(default)]
    pub new_arcs: Vec<ArcTrack>,
    pub new_vias: Vec<Via>,
    /// Recomputed fills `(index into Board::zones, all fills of that zone)`.
    #[serde(default)]
    pub zone_fills: Vec<(u32, Vec<ZoneFill>)>,
}

impl BoardEdit {
    /// Checks INV-EDIT-1 against `board` (the board *before* the edit).
    ///
    /// # Errors
    /// Description of the first violation.
    pub fn check_invariants(&self, board: &Board) -> Result<(), String> {
        for m in &self.moves {
            let fp = board.footprints.get(m.footprint.idx()).ok_or("move of unknown footprint")?;
            match fp.mobility {
                Mobility::Movable { max_displacement, allow_rotation } => {
                    if m.to.pos.dist(fp.original.pos) > ferroute_geom_f64(max_displacement) + 0.5 {
                        return Err(format!(
                            "{} moved beyond its displacement bound",
                            fp.reference
                        ));
                    }
                    if !allow_rotation && m.to.rot != fp.original.rot {
                        return Err(format!(
                            "{} rotated although rotation is not allowed",
                            fp.reference
                        ));
                    }
                }
                _ => return Err(format!("{} is not movable but was moved", fp.reference)),
            }
        }
        if self.zone_fills.iter().any(|(zi, _)| *zi as usize >= board.zones.len()) {
            return Err("fill edit of an unknown zone".into());
        }
        if self.new_tracks.iter().any(|t| t.origin != Origin::Routed)
            || self.new_arcs.iter().any(|a| a.origin != Origin::Routed)
            || self.new_vias.iter().any(|v| v.origin != Origin::Routed)
        {
            return Err("new copper must be tagged Origin::Routed".into());
        }
        Ok(())
    }
}

#[allow(clippy::cast_precision_loss)]
fn ferroute_geom_f64(c: Coord) -> f64 {
    c as f64
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod via_tests;
