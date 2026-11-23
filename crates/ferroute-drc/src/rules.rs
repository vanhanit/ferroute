// SPDX-License-Identifier: AGPL-3.0-only
//! Electrical-constraint checks (separate from hard DRC): trace width vs. the net's
//! required width (INV-RULE-1), routed length limits, length-matching skew within a group,
//! and via-count limits. Unmet constraints are *reported*, never hidden.

use ferroute_board::constraints::required_width;
use ferroute_board::{Board, NetId};
use ferroute_geom::Coord;
use ferroute_geom::units::format_mm;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// What kind of constraint is unmet.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum IssueKind {
    /// A track narrower than the net's required width.
    Width { actual: Coord, required: Coord },
    /// Routed length outside `[min, max]`.
    Length { actual: Coord, min: Option<Coord>, max: Option<Coord> },
    /// Length spread within a matching group above the allowed skew.
    Skew { group: String, spread: Coord, max: Coord },
    /// More vias than allowed.
    ViaCount { actual: u32, max: u32 },
}

impl IssueKind {
    #[must_use]
    pub const fn label(&self) -> &'static str {
        match self {
            Self::Width { .. } => "width",
            Self::Length { .. } => "length",
            Self::Skew { .. } => "skew",
            Self::ViaCount { .. } => "vias",
        }
    }
}

/// One unmet constraint.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ConstraintIssue {
    pub net: NetId,
    pub kind: IssueKind,
}

impl ConstraintIssue {
    /// One-line human-readable description (mm for lengths).
    #[must_use]
    pub fn describe(&self, board: &Board) -> String {
        let net = board.nets.get(self.net.idx()).map_or("?", |n| n.name.as_str());
        let mm = |v: Coord| format_mm(v);
        let opt = |v: Option<Coord>| v.map_or_else(|| "-".to_owned(), mm);
        match &self.kind {
            IssueKind::Width { actual, required } => {
                format!("net {net}: track width {} mm < required {} mm", mm(*actual), mm(*required))
            }
            IssueKind::Length { actual, min, max } => format!(
                "net {net}: routed length {} mm outside [{}, {}] mm",
                mm(*actual),
                opt(*min),
                opt(*max)
            ),
            IssueKind::Skew { group, spread, max } => format!(
                "match group {group} (net {net}…): length spread {} mm > allowed {} mm",
                mm(*spread),
                mm(*max)
            ),
            IssueKind::ViaCount { actual, max } => {
                format!("net {net}: {actual} vias > allowed {max}")
            }
        }
    }
}

/// Routed copper length of every net (nm, sum of track lengths, rounded).
#[must_use]
pub fn net_lengths(board: &Board) -> Vec<Coord> {
    let mut len = vec![0.0f64; board.nets.len()];
    for t in &board.tracks {
        if let Some(l) = len.get_mut(t.net.idx()) {
            *l += t.start.dist(t.end);
        }
    }
    #[allow(clippy::cast_possible_truncation)]
    len.into_iter().map(|l| l.round() as Coord).collect()
}

/// Checks every net's electrical constraints. Deterministic order: per net (width,
/// length, vias), then per matching group (skew).
#[must_use]
pub fn check_constraints(board: &Board) -> Vec<ConstraintIssue> {
    let lengths = net_lengths(board);
    let mut vias = vec![0u32; board.nets.len()];
    for v in &board.vias {
        if let Some(c) = vias.get_mut(v.net.idx()) {
            *c += 1;
        }
    }
    let mut out = Vec::new();
    let mut groups: BTreeMap<&str, (Coord, Vec<NetId>)> = BTreeMap::new();
    for (i, net) in board.nets.iter().enumerate().skip(1) {
        let id = NetId(u32::try_from(i).unwrap_or(u32::MAX));
        let c = &net.constraints;
        let need = required_width(board, id);
        if let Some(t) =
            board.tracks.iter().filter(|t| t.net == id && t.width < need).min_by_key(|t| t.width)
        {
            out.push(ConstraintIssue {
                net: id,
                kind: IssueKind::Width { actual: t.width, required: need },
            });
        }
        let l = lengths[i];
        if c.length_min.is_some_and(|m| l < m) || c.length_max.is_some_and(|m| l > m) {
            out.push(ConstraintIssue {
                net: id,
                kind: IssueKind::Length { actual: l, min: c.length_min, max: c.length_max },
            });
        }
        if let Some(max) = c.max_vias
            && vias[i] > max
        {
            out.push(ConstraintIssue {
                net: id,
                kind: IssueKind::ViaCount { actual: vias[i], max },
            });
        }
        if let (Some(g), Some(skew)) = (c.match_group.as_deref(), c.skew_max) {
            let e = groups.entry(g).or_insert((skew, Vec::new()));
            e.0 = e.0.min(skew);
            e.1.push(id);
        }
    }
    for (g, (max, nets)) in groups {
        let ls: Vec<Coord> = nets.iter().map(|n| lengths[n.idx()]).collect();
        let spread = ls.iter().max().copied().unwrap_or(0) - ls.iter().min().copied().unwrap_or(0);
        if spread > max {
            out.push(ConstraintIssue {
                net: nets[0],
                kind: IssueKind::Skew { group: g.to_owned(), spread, max },
            });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use ferroute_board::builder::{BoardBuilder, mm, pt10};
    use ferroute_board::{Origin, Via};

    fn two_nets() -> (ferroute_board::Board, ferroute_board::NetId, ferroute_board::NetId) {
        let mut b = BoardBuilder::new(mm(40), mm(30), 2);
        let a = b.net("D_P");
        let c = b.net("D_N");
        b.track(pt10(10, 10), pt10(110, 10), 250_000, 0, a, Origin::Routed); // 10 mm
        b.track(pt10(10, 20), pt10(130, 20), 250_000, 0, c, Origin::Routed); // 12 mm
        let board = b.build();
        (board, a, c)
    }

    #[test]
    fn issues_describe_themselves_in_mm() {
        let (board, a, _) = two_nets();
        let d = |kind| ConstraintIssue { net: a, kind }.describe(&board);
        assert_eq!(
            d(IssueKind::Length { actual: mm(10), min: None, max: Some(mm(5)) }),
            "net D_P: routed length 10 mm outside [-, 5] mm"
        );
        assert_eq!(
            d(IssueKind::Width { actual: 250_000, required: 300_000 }),
            "net D_P: track width 0.25 mm < required 0.3 mm"
        );
        assert_eq!(d(IssueKind::ViaCount { actual: 3, max: 1 }), "net D_P: 3 vias > allowed 1");
        assert!(
            d(IssueKind::Skew { group: "usb".into(), spread: mm(2), max: 100_000 })
                .contains("spread 2 mm > allowed 0.1 mm")
        );
    }

    #[test]
    fn clean_board_has_no_issues() {
        let (board, ..) = two_nets();
        assert!(check_constraints(&board).is_empty());
    }

    #[test]
    fn reports_length_limits_and_group_skew() {
        let (mut board, a, c) = two_nets();
        board.nets[a.idx()].constraints.length_min = Some(mm(11));
        board.nets[c.idx()].constraints.length_max = Some(mm(11));
        for n in [a, c] {
            board.nets[n.idx()].constraints.match_group = Some("USB".into());
            board.nets[n.idx()].constraints.skew_max = Some(500_000);
        }
        let issues = check_constraints(&board);
        let kinds: Vec<&str> = issues.iter().map(|i| i.kind.label()).collect();
        assert_eq!(kinds, vec!["length", "length", "skew"], "{issues:?}");
        assert!(matches!(issues[2].kind, IssueKind::Skew { spread, .. } if spread == mm(2)));
    }

    #[test]
    fn reports_too_many_vias_and_too_narrow_traces() {
        let (mut board, a, _) = two_nets();
        board.nets[a.idx()].constraints.max_vias = Some(0);
        board.nets[a.idx()].constraints.max_current_ma = Some(3000); // needs a wide trace
        let all = board.all_layers();
        board.vias.push(Via {
            pos: pt10(60, 10),
            diameter: 600_000,
            drill: 300_000,
            layers: all,
            net: a,
            locked: false,
            origin: Origin::Routed,
        });
        let issues = check_constraints(&board);
        assert!(issues.iter().any(|i| matches!(i.kind, IssueKind::ViaCount { actual: 1, max: 0 })));
        assert!(issues.iter().any(|i| matches!(i.kind, IssueKind::Width { actual: 250_000, .. })));
    }
}
