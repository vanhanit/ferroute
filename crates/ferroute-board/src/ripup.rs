// SPDX-License-Identifier: AGPL-3.0-only
//! Explicit user rip-up: removal of unlocked *existing* tracks, arcs and vias of selected
//! nets before routing (`ferroute route --ripup-all` / `--ripup-nets`).
//!
//! Pure and deterministic: the result depends only on `(board, scope)`. Locked copper,
//! zones/pours, pads, footprints, graphics and no-net copper are never touched.

use crate::{Board, NetId, Origin, TrackId, ViaId, glob};
use serde::{Deserialize, Serialize};

/// Which existing copper to rip up.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum RipupScope {
    /// Keep everything (the default; edits stay purely additive).
    #[default]
    None,
    /// Every net with a name.
    All,
    /// Nets whose name matches any of these globs (`*`, `?`; case-sensitive).
    Nets(Vec<String>),
}

impl RipupScope {
    /// `true` when no rip-up is requested.
    #[must_use]
    pub fn is_none(&self) -> bool {
        matches!(self, Self::None)
    }

    /// Builds a scope from user patterns (entries may be comma separated; blanks ignored).
    /// `all` wins over patterns; no patterns and not `all` is [`RipupScope::None`].
    #[must_use]
    pub fn from_patterns<S: AsRef<str>>(all: bool, patterns: &[S]) -> Self {
        if all {
            return Self::All;
        }
        let pats: Vec<String> = patterns
            .iter()
            .flat_map(|p| p.as_ref().split(','))
            .map(str::trim)
            .filter(|p| !p.is_empty())
            .map(str::to_owned)
            .collect();
        if pats.is_empty() { Self::None } else { Self::Nets(pats) }
    }

    /// Whether copper of `net` is selected. The no-net net is never selected.
    #[must_use]
    pub fn selects(&self, board: &Board, net: NetId) -> bool {
        if net == NetId::NONE {
            return false;
        }
        match self {
            Self::None => false,
            Self::All => true,
            Self::Nets(pats) => board
                .nets
                .get(net.idx())
                .is_some_and(|n| !n.name.is_empty() && pats.iter().any(|p| glob(p, &n.name))),
        }
    }
}

/// What to do with nets that own a copper pour (a zone with that net on any layer).
///
/// Their stitching vias and short support tracks keep the pour connected after the refill
/// (INV-CONN-1), so removing them is opt-in.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum PourNetPolicy {
    /// Never remove copper of a net that owns a pour (default).
    #[default]
    Keep,
    /// Treat pour nets like any other selected net.
    Rip,
}

/// Nets (by index) that own a zone on any layer.
#[must_use]
pub fn pour_nets(board: &Board) -> Vec<bool> {
    let mut v = vec![false; board.nets.len()];
    for z in &board.zones {
        if let Some(f) = v.get_mut(z.net.idx())
            && z.net != NetId::NONE
        {
            *f = true;
        }
    }
    v
}

/// Items of the ORIGINAL board that a rip-up removes (indices into its collections).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct RemovedItems {
    pub tracks: Vec<TrackId>,
    /// Indices into [`Board::arcs`].
    pub arcs: Vec<u32>,
    pub vias: Vec<ViaId>,
}

impl RemovedItems {
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.tracks.is_empty() && self.arcs.is_empty() && self.vias.is_empty()
    }

    /// Checks INV-EDIT-1 for removals against `board` (the board *before* the edit):
    /// only with a rip-up scope, only unlocked `Origin::Existing` copper of a selected net.
    ///
    /// # Errors
    /// Description of the first violation.
    pub fn check_invariants(&self, board: &Board, scope: &RipupScope) -> Result<(), String> {
        if self.is_empty() {
            return Ok(());
        }
        if scope.is_none() {
            return Err("removals require an explicit rip-up scope".into());
        }
        let check = |what: &str, i: usize, found: Option<(NetId, bool, Origin)>| {
            let (net, locked, origin) =
                found.ok_or_else(|| format!("removal of unknown {what}"))?;
            if locked {
                return Err(format!("removal of locked {what} #{i}"));
            }
            if origin != Origin::Existing {
                return Err(format!("removal of non-existing {what} #{i}"));
            }
            if !scope.selects(board, net) {
                return Err(format!("removal of {what} #{i} outside the rip-up scope"));
            }
            Ok(())
        };
        for t in &self.tracks {
            let f = board.tracks.get(t.idx()).map(|t| (t.net, t.locked, t.origin));
            check("track", t.idx(), f)?;
        }
        for &a in &self.arcs {
            let f = board.arcs.get(a as usize).map(|t| (t.net, t.locked, t.origin));
            check("arc", a as usize, f)?;
        }
        for v in &self.vias {
            let f = board.vias.get(v.idx()).map(|t| (t.net, t.locked, t.origin));
            check("via", v.idx(), f)?;
        }
        Ok(())
    }
}

/// What a rip-up did (for the user; deterministic order).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct RipupReport {
    /// Removed track segments (an input arc is read as several chord segments).
    pub tracks: usize,
    pub arcs: usize,
    pub vias: usize,
    /// Names of nets that lost copper, in net-id order.
    pub nets: Vec<String>,
    /// Selected-net items kept because they are locked.
    pub skipped_locked: usize,
    /// Patterns that matched no net.
    pub unmatched_patterns: Vec<String>,
    /// Selected nets whose copper was kept because they own a copper pour.
    #[serde(default)]
    pub kept_pour_nets: Vec<String>,
    /// Subset of `kept_pour_nets` that a `--ripup-nets` pattern matched (user asked for them).
    #[serde(default)]
    pub named_pour_nets: Vec<String>,
}

impl RipupReport {
    /// Human-readable warnings (unknown patterns, locked items kept).
    #[must_use]
    pub fn warnings(&self) -> Vec<String> {
        let mut w: Vec<String> = self
            .unmatched_patterns
            .iter()
            .map(|p| format!("rip-up pattern {p:?} matches no net"))
            .collect();
        if self.skipped_locked > 0 {
            w.push(format!("{} locked track/via item(s) kept", self.skipped_locked));
        }
        if !self.kept_pour_nets.is_empty() {
            w.push(format!(
                "kept: owns a copper pour (use --ripup-pour-nets to include): {}",
                self.kept_pour_nets.join(", ")
            ));
        }
        if !self.named_pour_nets.is_empty() {
            w.push(format!(
                "--ripup-nets names pour net(s) {}: their copper is still kept because they own a copper pour; pass --ripup-pour-nets to rip them (pour connectivity then depends on the router's plane fan-out)",
                self.named_pour_nets.join(", ")
            ));
        }
        w
    }
}

/// The result of [`ripup`].
#[derive(Clone, Debug)]
pub struct Ripup {
    /// The board without the removed copper (route this one).
    pub board: Board,
    /// Removed items, as indices into the ORIGINAL board.
    pub removed: RemovedItems,
    pub scope: RipupScope,
    pub report: RipupReport,
}

fn id32(i: usize) -> u32 {
    u32::try_from(i).unwrap_or(u32::MAX)
}

/// Keeps the elements of `v` whose index is not in the (ascending) list `gone`.
fn drop_indices<T>(v: &mut Vec<T>, gone: &[usize]) {
    let mut i = 0usize;
    v.retain(|_| {
        let keep = gone.binary_search(&i).is_err();
        i += 1;
        keep
    });
}

/// Removes unlocked existing tracks, arcs and vias of the nets selected by `scope`,
/// keeping pour nets ([`PourNetPolicy::Keep`]).
#[must_use]
pub fn ripup(board: &Board, scope: &RipupScope) -> Ripup {
    ripup_with(board, scope, PourNetPolicy::Keep)
}

/// [`ripup`] with an explicit [`PourNetPolicy`].
#[must_use]
pub fn ripup_with(board: &Board, scope: &RipupScope, pours: PourNetPolicy) -> Ripup {
    let mut removed = RemovedItems::default();
    let mut skipped = 0usize;
    let mut touched = vec![false; board.nets.len()];
    let mut selected: Vec<bool> =
        (0..board.nets.len()).map(|i| scope.selects(board, NetId(id32(i)))).collect();
    let (mut kept_pour_nets, mut named_pour_nets) = (Vec::new(), Vec::new());
    if pours == PourNetPolicy::Keep {
        for (i, is_pour) in pour_nets(board).into_iter().enumerate() {
            if is_pour && selected[i] {
                selected[i] = false;
                kept_pour_nets.push(board.nets[i].name.clone());
                if matches!(scope, RipupScope::Nets(_)) {
                    named_pour_nets.push(board.nets[i].name.clone());
                }
            }
        }
    }
    let mut take = |net: NetId, locked: bool, origin: Origin| -> bool {
        if origin != Origin::Existing || !selected.get(net.idx()).copied().unwrap_or(false) {
            return false;
        }
        if locked {
            skipped += 1;
            return false;
        }
        touched[net.idx()] = true;
        true
    };
    for (i, t) in board.tracks.iter().enumerate() {
        if take(t.net, t.locked, t.origin) {
            removed.tracks.push(TrackId(id32(i)));
        }
    }
    for (i, a) in board.arcs.iter().enumerate() {
        if take(a.net, a.locked, a.origin) {
            removed.arcs.push(id32(i));
        }
    }
    for (i, v) in board.vias.iter().enumerate() {
        if take(v.net, v.locked, v.origin) {
            removed.vias.push(ViaId(id32(i)));
        }
    }
    let unmatched_patterns = match scope {
        RipupScope::Nets(pats) => pats
            .iter()
            .filter(|p| !board.nets.iter().any(|n| !n.name.is_empty() && glob(p, &n.name)))
            .cloned()
            .collect(),
        _ => Vec::new(),
    };
    let report = RipupReport {
        tracks: removed.tracks.len(),
        arcs: removed.arcs.len(),
        vias: removed.vias.len(),
        nets: board
            .nets
            .iter()
            .zip(&touched)
            .filter(|(_, t)| **t)
            .map(|(n, _)| n.name.clone())
            .collect(),
        skipped_locked: skipped,
        unmatched_patterns,
        kept_pour_nets,
        named_pour_nets,
    };
    let mut out = board.clone();
    drop_indices(&mut out.tracks, &removed.tracks.iter().map(|t| t.idx()).collect::<Vec<_>>());
    drop_indices(&mut out.arcs, &removed.arcs.iter().map(|&a| a as usize).collect::<Vec<_>>());
    drop_indices(&mut out.vias, &removed.vias.iter().map(|v| v.idx()).collect::<Vec<_>>());
    Ripup { board: out, removed, scope: scope.clone(), report }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builder::{BoardBuilder, mm, pt10};
    use crate::{IslandRemoval, LayerSet, PadConnection, Zone, ZoneFill};
    use ferroute_geom::Polygon;

    /// Nets A, B, USB_P, USB_N with one track + via each, plus a no-net track.
    fn sample() -> Board {
        let mut b = BoardBuilder::new(mm(50), mm(30), 2);
        for (i, name) in ["A", "B", "USB_P", "USB_N"].iter().enumerate() {
            let n = b.net(name);
            let y = i64::try_from(i).unwrap_or(0) * 40 + 20;
            b.track(pt10(10, y), pt10(100, y), 200_000, 0, n, Origin::Existing);
            b.via(pt10(100, y), 600_000, 300_000, n, Origin::Existing);
        }
        b.track(pt10(10, 300), pt10(100, 300), 200_000, 0, NetId::NONE, Origin::Existing);
        b.build()
    }

    fn names(b: &Board) -> Vec<&str> {
        b.tracks.iter().map(|t| b.nets[t.net.idx()].name.as_str()).collect()
    }

    #[test]
    fn all_removes_every_named_net_but_not_no_net_copper() {
        let b = sample();
        let r = ripup(&b, &RipupScope::All);
        assert_eq!((r.report.tracks, r.report.vias), (4, 4));
        assert_eq!(r.board.tracks.len(), 1);
        assert_eq!(r.board.tracks[0].net, NetId::NONE);
        assert!(r.board.vias.is_empty());
        assert_eq!(r.report.nets, ["A", "B", "USB_P", "USB_N"]);
        assert!(r.removed.check_invariants(&b, &r.scope).is_ok());
    }

    #[test]
    fn nets_scope_uses_globs_and_keeps_the_rest() {
        let b = sample();
        let r = ripup(&b, &RipupScope::Nets(vec!["USB_*".into(), "A".into()]));
        assert_eq!(r.report.nets, ["A", "USB_P", "USB_N"]);
        assert_eq!(names(&r.board), ["B", ""]);
        assert_eq!(r.board.vias.len(), 1);
        assert_eq!(r.removed.tracks, [TrackId(0), TrackId(2), TrackId(3)]);
        // Case-sensitive like other net patterns.
        assert!(ripup(&b, &RipupScope::Nets(vec!["a".into()])).removed.is_empty());
    }

    #[test]
    fn locked_and_routed_items_are_kept_and_counted() {
        let mut b = sample();
        b.tracks[0].locked = true;
        b.vias[0].locked = true;
        b.tracks[1].origin = Origin::Routed;
        let r = ripup(&b, &RipupScope::All);
        assert_eq!(r.report.skipped_locked, 2);
        assert_eq!(r.board.tracks.len(), 3, "locked A, routed B and the no-net track stay");
        assert_eq!(r.board.vias.len(), 1);
        assert!(r.report.warnings().iter().any(|w| w.contains("locked")));
        let mut bad = r.removed.clone();
        bad.tracks.push(TrackId(0));
        assert!(bad.check_invariants(&b, &r.scope).unwrap_err().contains("locked"));
    }

    #[test]
    fn zones_pads_footprints_are_untouched() {
        let mut b = sample();
        b.zones.push(Zone {
            net: NetId(1),
            layers: LayerSet::single(crate::LayerId(0)),
            outline: Polygon::default(),
            fills: vec![ZoneFill { layer: crate::LayerId(0), polys: vec![] }],
            clearance: 0,
            min_thickness: 0,
            priority: 0,
            pad_connection: PadConnection::Solid,
            islands: IslandRemoval::default(),
            yields: false,
        });
        let r = ripup(&b, &RipupScope::All);
        assert_eq!(r.board.zones, b.zones);
        assert_eq!(r.board.pads, b.pads);
        assert_eq!(r.board.footprints, b.footprints);
    }

    #[test]
    fn unknown_pattern_is_reported_not_an_error() {
        let b = sample();
        let r = ripup(&b, &RipupScope::Nets(vec!["NOPE*".into(), "B".into()]));
        assert_eq!(r.report.unmatched_patterns, ["NOPE*"]);
        assert_eq!(r.report.tracks, 1);
        assert!(r.report.warnings()[0].contains("NOPE*"));
    }

    #[test]
    fn none_scope_changes_nothing_and_is_deterministic() {
        let b = sample();
        let r = ripup(&b, &RipupScope::None);
        assert_eq!(r.board.tracks, b.tracks);
        assert!(r.removed.is_empty());
        let s = RipupScope::Nets(vec!["USB_*".into()]);
        let (x, y) = (ripup(&b, &s), ripup(&b, &s));
        assert_eq!((x.removed, x.report, x.board.tracks), (y.removed, y.report, y.board.tracks));
    }

    #[test]
    fn removals_need_a_scope_that_selects_the_net() {
        let b = sample();
        let removed = RemovedItems { tracks: vec![TrackId(1)], ..RemovedItems::default() };
        assert!(removed.check_invariants(&b, &RipupScope::None).is_err());
        assert!(removed.check_invariants(&b, &RipupScope::Nets(vec!["A".into()])).is_err());
        assert!(removed.check_invariants(&b, &RipupScope::Nets(vec!["B".into()])).is_ok());
        let ghost = RemovedItems { vias: vec![ViaId(99)], ..RemovedItems::default() };
        assert!(ghost.check_invariants(&b, &RipupScope::All).is_err());
    }

    #[test]
    fn scope_from_patterns_splits_commas_and_all_wins() {
        assert_eq!(RipupScope::from_patterns::<&str>(false, &[]), RipupScope::None);
        assert_eq!(RipupScope::from_patterns(true, &["X"]), RipupScope::All);
        assert_eq!(
            RipupScope::from_patterns(false, &["USB_*, GND", " ", "A"]),
            RipupScope::Nets(vec!["USB_*".into(), "GND".into(), "A".into()])
        );
    }

    /// GND (zone + stitching via + track) and SIG (track).
    fn pour_board() -> Board {
        let mut b = BoardBuilder::new(mm(50), mm(30), 2);
        let (gnd, sig) = (b.net("GND"), b.net("SIG"));
        b.track(pt10(10, 20), pt10(100, 20), 200_000, 0, gnd, Origin::Existing);
        b.via(pt10(100, 20), 600_000, 300_000, gnd, Origin::Existing);
        b.track(pt10(10, 60), pt10(100, 60), 200_000, 0, sig, Origin::Existing);
        let mut b = b.build();
        b.zones.push(Zone {
            net: gnd,
            layers: LayerSet::single(crate::LayerId(1)),
            outline: Polygon::default(),
            fills: vec![],
            clearance: 0,
            min_thickness: 0,
            priority: 0,
            pad_connection: PadConnection::Solid,
            islands: IslandRemoval::default(),
            yields: false,
        });
        b
    }

    #[test]
    fn keep_policy_spares_pour_nets_on_all() {
        let b = pour_board();
        let r = ripup(&b, &RipupScope::All);
        assert_eq!(names(&r.board), ["GND"]);
        assert_eq!(r.board.vias.len(), 1);
        assert_eq!(r.report.nets, ["SIG"]);
        assert_eq!(r.report.kept_pour_nets, ["GND"]);
        assert!(r.report.named_pour_nets.is_empty());
        assert!(r.report.warnings().iter().any(|w| w.contains("owns a copper pour")));
        assert!(r.removed.check_invariants(&b, &r.scope).is_ok());
    }

    #[test]
    fn rip_policy_removes_pour_nets_too() {
        let b = pour_board();
        let r = ripup_with(&b, &RipupScope::All, PourNetPolicy::Rip);
        assert!(r.board.tracks.is_empty() && r.board.vias.is_empty());
        assert!(r.report.kept_pour_nets.is_empty());
    }

    #[test]
    fn explicit_pour_net_pattern_is_kept_with_a_clear_warning() {
        let b = pour_board();
        let r = ripup(&b, &RipupScope::Nets(vec!["GND".into()]));
        assert!(r.removed.is_empty());
        assert_eq!(r.report.named_pour_nets, ["GND"]);
        assert!(r.report.warnings().iter().any(|w| w.contains("--ripup-nets names pour net")));
    }
}
