// SPDX-License-Identifier: AGPL-3.0-only
//! KiCad custom design rules (`<board>.kicad_dru`): data model and exact evaluation.
//!
//! Semantics verified against KiCad 10 (`pcbnew/drc/drc_engine.cpp`,
//! `drc_rule_condition.cpp`): rules are evaluated in file order and the *last* matching
//! rule wins for a constraint type (it overrides the net-class value, it is not merged);
//! conditions are commutative in `A`/`B`; a rule's `(layer ..)` filter must match the layer
//! under test. Conditions we cannot evaluate ([`Cond::Unsupported`]) are *maybe-matching*:
//! they are applied only where that makes a check stricter (never lowers a requirement).

use crate::{Board, FootprintId, LayerId, LayerSet, NetId, glob};
use ferroute_geom::Coord;
use serde::{Deserialize, Serialize};

/// Constraint types we understand.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ConstraintKind {
    Clearance,
    TrackWidth,
    ViaDiameter,
    HoleSize,
    HoleToHole,
    HoleClearance,
    EdgeClearance,
    AnnularWidth,
    DiffPairGap,
    Length,
    Skew,
}

impl ConstraintKind {
    /// KiCad keyword.
    #[must_use]
    pub const fn keyword(self) -> &'static str {
        match self {
            Self::Clearance => "clearance",
            Self::TrackWidth => "track_width",
            Self::ViaDiameter => "via_diameter",
            Self::HoleSize => "hole_size",
            Self::HoleToHole => "hole_to_hole",
            Self::HoleClearance => "hole_clearance",
            Self::EdgeClearance => "edge_clearance",
            Self::AnnularWidth => "annular_width",
            Self::DiffPairGap => "diff_pair_gap",
            Self::Length => "length",
            Self::Skew => "skew",
        }
    }

    /// Parses a KiCad keyword.
    #[must_use]
    pub fn from_keyword(s: &str) -> Option<Self> {
        Some(match s {
            "clearance" => Self::Clearance,
            "track_width" => Self::TrackWidth,
            "via_diameter" => Self::ViaDiameter,
            "hole_size" | "hole" => Self::HoleSize,
            "hole_to_hole" => Self::HoleToHole,
            "hole_clearance" => Self::HoleClearance,
            "edge_clearance" => Self::EdgeClearance,
            "annular_width" => Self::AnnularWidth,
            "diff_pair_gap" => Self::DiffPairGap,
            "length" => Self::Length,
            "skew" => Self::Skew,
            _ => return None,
        })
    }
}

/// `(min ..) (max ..) (opt ..)` of one constraint (nm).
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Limits {
    pub min: Option<Coord>,
    pub max: Option<Coord>,
    pub opt: Option<Coord>,
}

/// `(severity ..)` of a rule. Only [`Severity::Error`] is a hard failure (the gate);
/// warnings are reported as non-error violations, `Ignore` / `Exclusion` suppress them.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Severity {
    #[default]
    Error,
    Warning,
    Ignore,
    Exclusion,
}

impl Severity {
    /// Parses a KiCad severity keyword.
    #[must_use]
    pub fn from_keyword(s: &str) -> Option<Self> {
        Some(match s {
            "error" => Self::Error,
            "warning" => Self::Warning,
            "ignore" => Self::Ignore,
            "exclusion" => Self::Exclusion,
            _ => return None,
        })
    }

    /// Strictness order: error > warning > ignore = exclusion.
    const fn rank(self) -> u8 {
        match self {
            Self::Error => 2,
            Self::Warning => 1,
            Self::Ignore | Self::Exclusion => 0,
        }
    }

    /// The stricter of two severities.
    #[must_use]
    pub const fn stricter(self, o: Self) -> Self {
        if o.rank() > self.rank() { o } else { self }
    }

    /// `true` when violations of this severity are suppressed entirely.
    #[must_use]
    pub const fn is_suppressed(self) -> bool {
        self.rank() == 0
    }
}

/// `(layer ..)` filter of a rule.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum LayerFilter {
    Outer,
    Inner,
    Named(String),
}

/// Which item of the pair a condition term refers to.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RuleSide {
    A,
    B,
}

/// Item property a condition compares.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Field {
    NetClass,
    NetName,
    Type,
    Layer,
}

/// Comparison operator.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CmpOp {
    Eq,
    Ne,
}

/// Condition AST.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Cond {
    /// No condition (always matches).
    True,
    And(Vec<Cond>),
    Or(Vec<Cond>),
    Not(Box<Cond>),
    /// `A.NetClass == 'HV*'`; `value` may contain `*` / `?` wildcards.
    Cmp {
        side: RuleSide,
        field: Field,
        op: CmpOp,
        value: String,
    },
    /// `A.memberOfFootprint('R*')`: the item belongs to a footprint whose reference matches
    /// the KiCad wildcard `pattern` (`*`, `?`, case-sensitive like `wxString::Matches`).
    /// True for pads, footprint graphics and footprint holes; false for tracks, vias and
    /// zones (no parent footprint). Pads / footprint items match by reference when the
    /// evaluator knows their owner (`ItemCtx::footprint`); while it is unknown they
    /// maybe-match, and so do `${CLASS:..}` selectors.
    MemberOfFootprint {
        side: RuleSide,
        pattern: String,
    },
    /// An expression we cannot evaluate (kept for diagnostics); maybe-matches.
    Unsupported(String),
}

/// One `(rule ..)`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CustomRule {
    pub name: String,
    pub condition: Cond,
    pub layer: Option<LayerFilter>,
    pub constraints: Vec<(ConstraintKind, Limits)>,
    /// `(severity ..)`; defaults to [`Severity::Error`].
    #[serde(default)]
    pub severity: Severity,
}

/// Kind of a board item as seen by conditions (`A.Type`).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ItemType {
    Track,
    Via,
    Pad,
    Zone,
    Footprint,
}

impl ItemType {
    const fn name(self) -> &'static str {
        match self {
            Self::Track => "track",
            Self::Via => "via",
            Self::Pad => "pad",
            Self::Zone => "zone",
            Self::Footprint => "footprint",
        }
    }
}

/// One side of an evaluated pair. `None` fields are *unknown* (maybe-matching).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ItemCtx {
    pub net: Option<NetId>,
    pub kind: Option<ItemType>,
    pub layer: Option<LayerId>,
    /// Owning footprint (pads, footprint graphics and holes); `None` = unknown, which
    /// `memberOfFootprint` treats as a maybe for footprint-borne kinds.
    pub footprint: Option<FootprintId>,
}

impl ItemCtx {
    /// Nothing known about the item.
    pub const UNKNOWN: Self = Self { net: None, kind: None, layer: None, footprint: None };

    #[must_use]
    pub const fn new(net: NetId, kind: ItemType) -> Self {
        Self { net: Some(net), kind: Some(kind), layer: None, footprint: None }
    }

    /// The item belongs to footprint `fp` (`memberOfFootprint` is then decided exactly).
    #[must_use]
    pub const fn in_footprint(self, fp: FootprintId) -> Self {
        Self { footprint: Some(fp), ..self }
    }

    #[must_use]
    pub const fn on(self, layer: LayerId) -> Self {
        Self { layer: Some(layer), ..self }
    }
}

/// Three-valued truth.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Tri {
    False,
    True,
    Maybe,
}

impl Tri {
    const fn and(self, o: Self) -> Self {
        match (self, o) {
            (Self::False, _) | (_, Self::False) => Self::False,
            (Self::True, Self::True) => Self::True,
            _ => Self::Maybe,
        }
    }
    const fn or(self, o: Self) -> Self {
        match (self, o) {
            (Self::True, _) | (_, Self::True) => Self::True,
            (Self::False, Self::False) => Self::False,
            _ => Self::Maybe,
        }
    }
    const fn not(self) -> Self {
        match self {
            Self::True => Self::False,
            Self::False => Self::True,
            Self::Maybe => Self::Maybe,
        }
    }
    const fn from(b: bool) -> Self {
        if b { Self::True } else { Self::False }
    }
}

/// Class name of `net` as KiCad's `A.NetClass` sees it (derived `<class>/<net>` classes
/// created by ferroute map back to the original class).
fn class_name(board: &Board, net: NetId) -> &str {
    let name = board.rules.class_of(board, net).name.as_str();
    let net_name = board.nets.get(net.idx()).map_or("", |n| n.name.as_str());
    name.strip_suffix(net_name)
        .and_then(|s| s.strip_suffix('/'))
        .filter(|_| !net_name.is_empty())
        .unwrap_or(name)
}

fn layer_is_outer(board: &Board, l: LayerId) -> bool {
    l.0 == 0 || usize::from(l.0) + 1 == board.copper_layers.len()
}

impl Cond {
    /// `memberOfFootprint(pattern)` for an item of `kind` whose owning footprint reference
    /// is `owner` (`None` = unknown). KiCad matches with `wxString::Matches` (case-sensitive
    /// `*` / `?`); an empty pattern is an error and evaluates to false.
    #[must_use]
    pub fn member_of_footprint(pattern: &str, kind: Option<ItemType>, owner: Option<&str>) -> Tri {
        if pattern.is_empty() {
            return Tri::False;
        }
        if pattern.to_ascii_uppercase().starts_with("${CLASS:") {
            return Tri::Maybe;
        }
        match (owner, kind) {
            (Some(r), _) => Tri::from(glob(pattern, r)),
            (None, Some(ItemType::Track | ItemType::Via | ItemType::Zone)) => Tri::False,
            (None, _) => Tri::Maybe,
        }
    }

    fn cmp(board: &Board, field: Field, op: CmpOp, value: &str, ctx: &ItemCtx) -> Tri {
        let r = match field {
            Field::NetClass => ctx.net.map(|n| {
                let name = class_name(board, n);
                name.split(',').any(|c| glob(value, c.trim())) || glob(value, name)
            }),
            Field::NetName => ctx
                .net
                .map(|n| glob(value, board.nets.get(n.idx()).map_or("", |x| x.name.as_str()))),
            Field::Type => ctx.kind.map(|k| glob(&value.to_ascii_lowercase(), k.name())),
            Field::Layer => ctx.layer.map(|l| match value {
                "outer" => layer_is_outer(board, l),
                "inner" => !layer_is_outer(board, l),
                v => board.copper_layers.get(usize::from(l.0)).is_some_and(|c| glob(v, &c.name)),
            }),
        };
        match (r, op) {
            (None, _) => Tri::Maybe,
            (Some(b), CmpOp::Eq) => Tri::from(b),
            (Some(b), CmpOp::Ne) => Tri::from(!b),
        }
    }

    /// Evaluates the condition for the ordered pair `(a, b)`; `b = None` is an absent item
    /// (unary constraints), for which every `B.` comparison is false.
    #[must_use]
    pub fn eval(&self, board: &Board, a: &ItemCtx, b: Option<&ItemCtx>) -> Tri {
        match self {
            Self::True => Tri::True,
            Self::And(v) => v.iter().fold(Tri::True, |t, c| t.and(c.eval(board, a, b))),
            Self::Or(v) => v.iter().fold(Tri::False, |t, c| t.or(c.eval(board, a, b))),
            Self::Not(c) => c.eval(board, a, b).not(),
            Self::Unsupported(_) => Tri::Maybe,
            Self::MemberOfFootprint { side, pattern } => {
                let ctx = match side {
                    RuleSide::A => Some(a),
                    RuleSide::B => b,
                };
                ctx.map_or(Tri::False, |c| {
                    let owner = c
                        .footprint
                        .and_then(|f| board.footprints.get(f.idx()))
                        .map(|f| f.reference.as_str());
                    Self::member_of_footprint(pattern, c.kind, owner)
                })
            }
            Self::Cmp { side, field, op, value } => {
                let ctx = match side {
                    RuleSide::A => Some(a),
                    RuleSide::B => b,
                };
                ctx.map_or(Tri::False, |c| Self::cmp(board, *field, *op, value, c))
            }
        }
    }

    /// Commutative evaluation (KiCad tries `(a, b)` and then `(b, a)`).
    #[must_use]
    pub fn eval_commutative(&self, board: &Board, a: &ItemCtx, b: Option<&ItemCtx>) -> Tri {
        let t = self.eval(board, a, b);
        match b {
            Some(bb) if t != Tri::True => t.or(self.eval(board, bb, Some(a))),
            _ => t,
        }
    }
}

impl CustomRule {
    fn layer_tri(&self, board: &Board, layer: Option<LayerId>) -> Tri {
        let Some(f) = &self.layer else { return Tri::True };
        let Some(l) = layer else { return Tri::Maybe };
        Tri::from(match f {
            LayerFilter::Outer => layer_is_outer(board, l),
            LayerFilter::Inner => !layer_is_outer(board, l),
            LayerFilter::Named(n) => {
                board.copper_layers.get(usize::from(l.0)).is_some_and(|c| &c.name == n)
            }
        })
    }

    /// Whether the rule applies to the pair on the layer under test.
    #[must_use]
    pub fn matches(&self, board: &Board, a: &ItemCtx, b: Option<&ItemCtx>) -> Tri {
        let layer = a.layer.or_else(|| b.and_then(|x| x.layer));
        self.layer_tri(board, layer).and(self.condition.eval_commutative(board, a, b))
    }

    /// The limits for `kind`, when the rule has that constraint.
    #[must_use]
    pub fn limits(&self, kind: ConstraintKind) -> Option<Limits> {
        self.constraints.iter().find(|(k, _)| *k == kind).map(|(_, l)| *l)
    }
}

impl crate::DesignRules {
    /// Whether any custom rule constrains `kind`.
    #[must_use]
    pub fn has_custom(&self, kind: ConstraintKind) -> bool {
        self.custom.iter().any(|r| r.limits(kind).is_some())
    }

    /// Largest `min` of any custom rule of `kind` (0 when none) — sizes query windows.
    #[must_use]
    pub fn max_custom_min(&self, kind: ConstraintKind) -> Coord {
        self.custom.iter().filter_map(|r| r.limits(kind)?.min).max().unwrap_or(0)
    }

    /// Minimum of `kind` for the pair: `base` overridden by the last matching rule
    /// (KiCad winner-takes-all); a maybe-matching rule can only raise the value.
    fn resolve(
        &self,
        board: &Board,
        kind: ConstraintKind,
        a: &ItemCtx,
        b: Option<&ItemCtx>,
        base: Coord,
    ) -> Coord {
        let mut cur = base;
        for r in &self.custom {
            let Some(v) = r.limits(kind).and_then(|l| l.min) else { continue };
            match r.matches(board, a, b) {
                Tri::True => cur = v,
                Tri::Maybe => cur = cur.max(v),
                Tri::False => {}
            }
        }
        cur
    }

    /// Smallest `max` of `kind` for the pair: the last definite rule with a `max` sets it
    /// (winner-takes-all), a maybe-matching rule can only lower it. `None` = unbounded.
    #[must_use]
    pub fn max_limit_for(
        &self,
        board: &Board,
        kind: ConstraintKind,
        a: &ItemCtx,
        b: Option<&ItemCtx>,
    ) -> Option<Coord> {
        let mut cur: Option<Coord> = None;
        for r in &self.custom {
            let Some(v) = r.limits(kind).and_then(|l| l.max) else { continue };
            match r.matches(board, a, b) {
                Tri::True => cur = Some(v),
                Tri::Maybe => cur = Some(cur.map_or(v, |c| c.min(v))),
                Tri::False => {}
            }
        }
        cur
    }

    /// Smallest `max` of `kind` over every rule that may apply to an item of `net`
    /// (conservative; pair rules count when `net` can be either side).
    #[must_use]
    pub fn max_limit_for_net(
        &self,
        board: &Board,
        kind: ConstraintKind,
        net: NetId,
        item: Option<ItemType>,
        pair: bool,
    ) -> Option<Coord> {
        let a = ItemCtx { net: Some(net), kind: item, ..ItemCtx::UNKNOWN };
        let other = pair.then_some(ItemCtx::UNKNOWN);
        self.custom
            .iter()
            .filter_map(|r| {
                let v = r.limits(kind)?.max?;
                (r.matches(board, &a, other.as_ref()) != Tri::False).then_some(v)
            })
            .min()
    }

    /// Severity a violation of `kind` between `a` and `b` is reported with: the last
    /// definite rule constraining `kind` decides (default error); a maybe-matching rule can
    /// only make it stricter.
    #[must_use]
    pub fn severity_for(
        &self,
        board: &Board,
        kind: ConstraintKind,
        a: &ItemCtx,
        b: Option<&ItemCtx>,
    ) -> Severity {
        let mut cur = Severity::Error;
        for r in &self.custom {
            if r.limits(kind).is_none() {
                continue;
            }
            match r.matches(board, a, b) {
                Tri::True => cur = r.severity,
                Tri::Maybe => cur = cur.stricter(r.severity),
                Tri::False => {}
            }
        }
        cur
    }

    /// Minimum of `kind` demanded by custom rules alone (0 when none applies).
    #[must_use]
    pub fn custom_min_for(
        &self,
        board: &Board,
        kind: ConstraintKind,
        a: &ItemCtx,
        b: Option<&ItemCtx>,
    ) -> Coord {
        self.resolve(board, kind, a, b, 0)
    }

    /// Drill-edge to drill-edge distance required between two holes.
    #[must_use]
    pub fn hole_to_hole_for(&self, board: &Board, a: &ItemCtx, b: &ItemCtx) -> Coord {
        self.resolve(board, ConstraintKind::HoleToHole, a, Some(b), self.hole_to_hole)
    }

    /// Largest hole-to-hole distance any pair may need (sizes query windows and routing
    /// reach: use this instead of `hole_to_hole` wherever a conservative bound is needed).
    #[must_use]
    pub fn effective_hole_to_hole(&self) -> Coord {
        self.hole_to_hole.max(self.max_custom_min(ConstraintKind::HoleToHole))
    }

    /// Required copper clearance between two items on the layer in their contexts.
    #[must_use]
    pub fn clearance_for(&self, board: &Board, a: &ItemCtx, b: &ItemCtx) -> Coord {
        let base = match (a.net, b.net) {
            (Some(x), Some(y)) => board.clearance_between(x, y),
            (Some(x), None) | (None, Some(x)) => {
                board.rules.class_of(board, x).clearance.max(self.max_clearance())
            }
            (None, None) => self.max_clearance(),
        };
        self.resolve(board, ConstraintKind::Clearance, a, Some(b), base)
    }

    /// [`Self::clearance_for`], maximised over `layers` (each layer is tested separately by
    /// KiCad; the geometric distance is layer independent).
    #[must_use]
    pub fn clearance_for_layers(
        &self,
        board: &Board,
        a: &ItemCtx,
        b: &ItemCtx,
        layers: LayerSet,
    ) -> Coord {
        let mut it = layers.iter();
        let Some(first) = it.next() else { return self.clearance_for(board, a, b) };
        std::iter::once(first)
            .chain(it)
            .map(|l| self.clearance_for(board, &a.on(l), &b.on(l)))
            .max()
            .unwrap_or(0)
    }

    /// Copper-to-hole clearance for `hole` (owner of the hole) and `copper`.
    #[must_use]
    pub fn hole_clearance_for(&self, board: &Board, hole: &ItemCtx, copper: &ItemCtx) -> Coord {
        self.resolve(board, ConstraintKind::HoleClearance, hole, Some(copper), self.hole_clearance)
    }

    /// Copper-to-board-edge clearance of one item.
    #[must_use]
    pub fn edge_clearance_for(&self, board: &Board, a: &ItemCtx) -> Coord {
        self.resolve(board, ConstraintKind::EdgeClearance, a, None, self.edge_clearance)
    }

    /// Minimum track width of one track.
    #[must_use]
    pub fn min_track_width_for(&self, board: &Board, a: &ItemCtx) -> Coord {
        self.resolve(board, ConstraintKind::TrackWidth, a, None, self.min_track_width)
    }

    /// Minimum via (copper) diameter of one via.
    #[must_use]
    pub fn min_via_diameter_for(&self, board: &Board, a: &ItemCtx) -> Coord {
        self.resolve(board, ConstraintKind::ViaDiameter, a, None, self.min_via_diameter)
    }

    /// Minimum hole (drill) size of one via.
    #[must_use]
    pub fn min_hole_size_for(&self, board: &Board, a: &ItemCtx) -> Coord {
        self.resolve(board, ConstraintKind::HoleSize, a, None, self.min_via_drill)
    }

    /// Minimum annular ring width (0 = unchecked unless a rule demands one).
    #[must_use]
    pub fn min_annular_for(&self, board: &Board, a: &ItemCtx) -> Coord {
        self.resolve(board, ConstraintKind::AnnularWidth, a, None, 0)
    }

    /// Largest `min` of `kind` over every rule that may apply to an item of `net`
    /// (pair rules count when `net` can be either side). `None` when no rule may apply.
    fn bound_for_net(
        &self,
        board: &Board,
        kind: ConstraintKind,
        net: NetId,
        item: Option<ItemType>,
        pair: bool,
    ) -> Option<Coord> {
        let a = ItemCtx { net: Some(net), kind: item, ..ItemCtx::UNKNOWN };
        let other = pair.then_some(ItemCtx::UNKNOWN);
        self.custom
            .iter()
            .filter_map(|r| {
                let v = r.limits(kind)?.min?;
                (r.matches(board, &a, other.as_ref()) != Tri::False).then_some(v)
            })
            .max()
    }

    /// Conservative clearance for `net`: at least the class clearance and every custom
    /// clearance that may apply to any pair involving the net.
    #[must_use]
    pub fn max_clearance_for_net(&self, board: &Board, net: NetId) -> Coord {
        let class = self.class_of(board, net).clearance;
        self.bound_for_net(board, ConstraintKind::Clearance, net, None, true)
            .map_or(class, |v| v.max(class))
    }

    /// Conservative minimum track width of `net` imposed by custom rules (0 when none).
    #[must_use]
    pub fn min_width_for_net(&self, board: &Board, net: NetId) -> Coord {
        self.bound_for_net(board, ConstraintKind::TrackWidth, net, Some(ItemType::Track), false)
            .unwrap_or(0)
    }

    /// Conservative minimum via copper diameter of `net` (custom `via_diameter` and the
    /// ring needed by `annular_width` over `drill`).
    #[must_use]
    pub fn min_via_diameter_for_net(&self, board: &Board, net: NetId, drill: Coord) -> Coord {
        let via = Some(ItemType::Via);
        let d = self.bound_for_net(board, ConstraintKind::ViaDiameter, net, via, false);
        let ring = self.bound_for_net(board, ConstraintKind::AnnularWidth, net, via, false);
        d.unwrap_or(0).max(ring.map_or(0, |r| drill + 2 * r))
    }

    /// Conservative minimum drill of `net`'s vias.
    #[must_use]
    pub fn min_hole_for_net(&self, board: &Board, net: NetId) -> Coord {
        self.bound_for_net(board, ConstraintKind::HoleSize, net, Some(ItemType::Via), false)
            .unwrap_or(0)
    }

    /// Conservative `diff_pair_gap` minimum imposed by custom rules on pairs involving
    /// `net` (0 when none).
    #[must_use]
    pub fn diff_pair_gap_for_net(&self, board: &Board, net: NetId) -> Coord {
        self.bound_for_net(board, ConstraintKind::DiffPairGap, net, None, true).unwrap_or(0)
    }

    /// `(min, max)` routed-length limits that apply to `net` (definite matches only).
    #[must_use]
    pub fn length_limits_for_net(
        &self,
        board: &Board,
        net: NetId,
    ) -> (Option<Coord>, Option<Coord>) {
        let a = ItemCtx::new(net, ItemType::Track);
        let mut lim = (None::<Coord>, None::<Coord>);
        for r in &self.custom {
            let Some(l) = r.limits(ConstraintKind::Length) else { continue };
            if r.matches(board, &a, None) == Tri::True {
                lim.0 = l.min.map_or(lim.0, |m| Some(lim.0.map_or(m, |c| c.max(m))));
                lim.1 = l.max.map_or(lim.1, |m| Some(lim.1.map_or(m, |c| c.min(m))));
            }
        }
        lim
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::NetClass;
    use crate::builder::{BoardBuilder, mm};

    #[test]
    fn member_of_footprint_semantics() {
        let m = Cond::member_of_footprint;
        let pad = Some(ItemType::Pad);
        assert_eq!(m("RJ*", pad, Some("RJ1")), Tri::True);
        assert_eq!(m("USB1", pad, Some("USB1")), Tri::True);
        assert_eq!(m("USB1", pad, Some("USB10")), Tri::False);
        assert_eq!(m("rj*", pad, Some("RJ1")), Tri::False, "case-sensitive like wxString::Matches");
        assert_eq!(m("R?1", pad, Some("RJ1")), Tri::True);
        // Items without a parent footprint never match.
        for k in [ItemType::Track, ItemType::Via, ItemType::Zone] {
            assert_eq!(m("RJ*", Some(k), None), Tri::False);
        }
        // Unknown owner of a pad / footprint item: maybe; empty pattern: false.
        assert_eq!(m("RJ*", pad, None), Tri::Maybe);
        assert_eq!(m("RJ*", None, None), Tri::Maybe);
        assert_eq!(m("${CLASS:foo}", pad, Some("RJ1")), Tri::Maybe);
        assert_eq!(m("", pad, Some("RJ1")), Tri::False);
    }

    #[test]
    fn member_of_footprint_condition_on_tracks_is_false() {
        let board = BoardBuilder::new(mm(10), mm(10), 2).build();
        let c = Cond::MemberOfFootprint { side: RuleSide::A, pattern: "RJ*".into() };
        let track = ItemCtx { kind: Some(ItemType::Track), ..ItemCtx::UNKNOWN };
        assert_eq!(c.eval(&board, &track, None), Tri::False);
        assert_eq!(c.eval(&board, &ItemCtx::UNKNOWN, None), Tri::Maybe);
    }

    fn eq(side: RuleSide, field: Field, v: &str) -> Cond {
        Cond::Cmp { side, field, op: CmpOp::Eq, value: v.into() }
    }

    fn rule(name: &str, c: Cond, kind: ConstraintKind, min: Coord) -> CustomRule {
        CustomRule {
            name: name.into(),
            condition: c,
            layer: None,
            constraints: vec![(kind, Limits { min: Some(min), ..Limits::default() })],
            severity: Severity::Error,
        }
    }

    fn board() -> (Board, NetId, NetId, NetId) {
        let mut b = BoardBuilder::new(mm(30), mm(20), 2);
        let hv = b.net("/HV1");
        let lv = b.net("SIG");
        let gnd = b.net("GND");
        (b.build(), hv, lv, gnd)
    }

    #[test]
    fn netname_wildcard_pair_rule_is_commutative() {
        let (mut b, hv, lv, _) = board();
        b.rules.custom.push(rule(
            "hv",
            Cond::And(vec![
                eq(RuleSide::A, Field::NetName, "/HV*"),
                eq(RuleSide::B, Field::NetName, "SIG"),
            ]),
            ConstraintKind::Clearance,
            mm(1),
        ));
        let (a, c) = (ItemCtx::new(hv, ItemType::Track), ItemCtx::new(lv, ItemType::Pad));
        assert_eq!(b.rules.clearance_for(&b, &a, &c), mm(1));
        assert_eq!(b.rules.clearance_for(&b, &c, &a), mm(1));
        assert_eq!(b.rules.clearance_for(&b, &a, &a), 200_000, "same pair of HV nets untouched");
    }

    #[test]
    fn last_matching_rule_wins_even_when_lower() {
        let (mut b, hv, lv, _) = board();
        let any = |v| rule("r", Cond::True, ConstraintKind::Clearance, v);
        b.rules.custom = vec![any(mm(1)), any(100_000)];
        let (a, c) = (ItemCtx::new(hv, ItemType::Track), ItemCtx::new(lv, ItemType::Track));
        assert_eq!(b.rules.clearance_for(&b, &a, &c), 100_000);
    }

    #[test]
    fn netclass_and_layer_outer() {
        let (mut b, hv, lv, _) = board();
        b.rules.classes.push(NetClass {
            name: "HV".into(),
            clearance: 300_000,
            ..NetClass::default()
        });
        b.nets[hv.idx()].class = 1;
        let mut r = rule(
            "r",
            Cond::And(vec![
                eq(RuleSide::A, Field::NetClass, "HV"),
                eq(RuleSide::B, Field::NetClass, "Default"),
            ]),
            ConstraintKind::Clearance,
            mm(2),
        );
        r.layer = Some(LayerFilter::Outer);
        b.rules.custom.push(r);
        let (a, c) = (ItemCtx::new(hv, ItemType::Track), ItemCtx::new(lv, ItemType::Track));
        assert_eq!(b.rules.clearance_for(&b, &a.on(LayerId(0)), &c.on(LayerId(0))), mm(2));
        assert_eq!(b.rules.clearance_for_layers(&b, &a, &c, LayerSet(0b11)), mm(2));
        // Two-layer board: layer 1 is the bottom copper layer, also outer.
        assert_eq!(b.rules.clearance_for(&b, &a.on(LayerId(1)), &c.on(LayerId(1))), mm(2));
    }

    #[test]
    fn unsupported_condition_only_raises() {
        let (mut b, hv, lv, _) = board();
        let u = |v| {
            rule("u", Cond::Unsupported("A.insideArea('X')".into()), ConstraintKind::Clearance, v)
        };
        let (a, c) = (ItemCtx::new(hv, ItemType::Track), ItemCtx::new(lv, ItemType::Track));
        b.rules.custom = vec![u(mm(1))];
        assert_eq!(b.rules.clearance_for(&b, &a, &c), mm(1), "raises");
        b.rules.custom = vec![u(50_000)];
        assert_eq!(b.rules.clearance_for(&b, &a, &c), 200_000, "never lowers");
    }

    #[test]
    fn per_net_bound_covers_every_pair() {
        let (mut b, hv, lv, gnd) = board();
        b.rules.custom.push(rule(
            "p",
            Cond::And(vec![
                eq(RuleSide::A, Field::NetName, "GND"),
                eq(RuleSide::B, Field::NetName, "SIG"),
            ]),
            ConstraintKind::Clearance,
            mm(1),
        ));
        assert_eq!(b.rules.max_clearance_for_net(&b, gnd), mm(1));
        assert_eq!(b.rules.max_clearance_for_net(&b, lv), mm(1));
        assert_eq!(b.rules.max_clearance_for_net(&b, hv), 200_000, "unrelated net untouched");
    }

    #[test]
    fn width_and_length_rules() {
        let (mut b, hv, lv, _) = board();
        b.rules.custom = vec![
            rule("w", eq(RuleSide::A, Field::NetName, "SIG"), ConstraintKind::TrackWidth, 300_000),
            CustomRule {
                name: "len".into(),
                condition: eq(RuleSide::A, Field::NetName, "SIG"),
                layer: None,
                constraints: vec![(
                    ConstraintKind::Length,
                    Limits { min: Some(mm(5)), max: Some(mm(40)), opt: None },
                )],
                severity: Severity::Error,
            },
        ];
        assert_eq!(b.rules.min_width_for_net(&b, lv), 300_000);
        assert_eq!(b.rules.min_width_for_net(&b, hv), 0);
        assert_eq!(b.rules.length_limits_for_net(&b, lv), (Some(mm(5)), Some(mm(40))));
        assert_eq!(b.rules.min_track_width_for(&b, &ItemCtx::new(lv, ItemType::Track)), 300_000);
        assert_eq!(b.rules.min_track_width_for(&b, &ItemCtx::new(hv, ItemType::Track)), 0);
    }

    fn limits_rule(c: Cond, kind: ConstraintKind, l: Limits, sev: Severity) -> CustomRule {
        CustomRule {
            name: "l".into(),
            condition: c,
            layer: None,
            constraints: vec![(kind, l)],
            severity: sev,
        }
    }

    #[test]
    fn max_limit_last_definite_wins_maybe_only_lowers() {
        let (mut b, _, lv, _) = board();
        let mx = |v| Limits { max: Some(v), ..Limits::default() };
        let a = ItemCtx::new(lv, ItemType::Track);
        let k = ConstraintKind::TrackWidth;
        assert_eq!(b.rules.max_limit_for(&b, k, &a, None), None);
        b.rules.custom = vec![
            limits_rule(Cond::True, k, mx(mm(1)), Severity::Error),
            limits_rule(Cond::True, k, mx(mm(2)), Severity::Error),
        ];
        assert_eq!(b.rules.max_limit_for(&b, k, &a, None), Some(mm(2)));
        let maybe = |v| limits_rule(Cond::Unsupported("u".into()), k, mx(v), Severity::Error);
        b.rules.custom.push(maybe(mm(1)));
        assert_eq!(b.rules.max_limit_for(&b, k, &a, None), Some(mm(1)), "maybe lowers");
        b.rules.custom[2] = maybe(mm(3));
        assert_eq!(b.rules.max_limit_for(&b, k, &a, None), Some(mm(2)), "maybe never raises");
        let net = b.rules.max_limit_for_net(&b, k, lv, Some(ItemType::Track), false);
        assert_eq!(net, Some(mm(1)), "per-net bound is the smallest of all candidate rules");
    }

    #[test]
    fn severity_last_definite_wins_maybe_is_stricter() {
        let (mut b, hv, lv, _) = board();
        let k = ConstraintKind::Clearance;
        let mn = Limits { min: Some(mm(1)), ..Limits::default() };
        let (a, c) = (ItemCtx::new(hv, ItemType::Track), ItemCtx::new(lv, ItemType::Track));
        assert_eq!(b.rules.severity_for(&b, k, &a, Some(&c)), Severity::Error);
        b.rules.custom = vec![limits_rule(Cond::True, k, mn, Severity::Warning)];
        assert_eq!(b.rules.severity_for(&b, k, &a, Some(&c)), Severity::Warning);
        b.rules.custom.push(limits_rule(Cond::Unsupported("u".into()), k, mn, Severity::Error));
        assert_eq!(b.rules.severity_for(&b, k, &a, Some(&c)), Severity::Error);
        b.rules.custom[1].severity = Severity::Ignore;
        assert_eq!(b.rules.severity_for(&b, k, &a, Some(&c)), Severity::Warning);
        b.rules.custom[1] = limits_rule(Cond::True, k, mn, Severity::Exclusion);
        assert!(b.rules.severity_for(&b, k, &a, Some(&c)).is_suppressed());
        // A rule of another constraint kind does not influence it.
        let w = ConstraintKind::TrackWidth;
        b.rules.custom = vec![limits_rule(Cond::True, w, mn, Severity::Ignore)];
        assert_eq!(b.rules.severity_for(&b, k, &a, Some(&c)), Severity::Error);
    }

    #[test]
    fn hole_to_hole_rule_resolves_and_widens_reach() {
        let (mut b, hv, lv, _) = board();
        let a = ItemCtx::new(hv, ItemType::Via);
        let c = ItemCtx::new(lv, ItemType::Pad);
        assert_eq!(b.rules.hole_to_hole_for(&b, &a, &c), b.rules.hole_to_hole);
        b.rules.custom.push(rule(
            "h",
            eq(RuleSide::A, Field::Type, "via"),
            ConstraintKind::HoleToHole,
            mm(1),
        ));
        assert_eq!(b.rules.hole_to_hole_for(&b, &a, &c), mm(1));
        assert_eq!(b.rules.hole_to_hole_for(&b, &c, &a), mm(1), "commutative");
        assert_eq!(b.rules.effective_hole_to_hole(), mm(1));
        assert_eq!(ConstraintKind::from_keyword("hole_to_hole"), Some(ConstraintKind::HoleToHole));
    }
}
