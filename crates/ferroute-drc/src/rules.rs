// SPDX-License-Identifier: AGPL-3.0-only
//! Electrical-constraint checks (separate from hard DRC): trace width vs. the net's
//! required width (INV-RULE-1), routed length limits, length-matching skew within a group,
//! and via-count limits. Unmet constraints are *reported*, never hidden.

use ferroute_board::constraints::{MatchBy, match_groups, net_delays_fs, required_width};
use ferroute_board::{Board, NetId};
use ferroute_geom::Coord;
use ferroute_geom::units::format_mm;
use serde::{Deserialize, Serialize};

/// What kind of constraint is unmet.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum IssueKind {
    /// A track narrower than the net's required width.
    Width { actual: Coord, required: Coord },
    /// Routed length outside `[min, max]`.
    Length { actual: Coord, min: Option<Coord>, max: Option<Coord> },
    /// Length spread within a matching group above the allowed skew.
    Skew { group: String, spread: Coord, max: Coord },
    /// Propagation-delay spread (fs) within a delay-matching group above the allowed skew.
    SkewDelay { group: String, spread_fs: i64, max_fs: i64 },
    /// More vias than allowed.
    ViaCount { actual: u32, max: u32 },
    /// The impedance target cannot be reached by any width with this stack-up.
    Impedance { target_mohm: u32 },
}

impl IssueKind {
    #[must_use]
    pub const fn label(&self) -> &'static str {
        match self {
            Self::Width { .. } => "width",
            Self::Length { .. } => "length",
            Self::Skew { .. } | Self::SkewDelay { .. } => "skew",
            Self::ViaCount { .. } => "vias",
            Self::Impedance { .. } => "impedance",
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
                mm((*spread + 500) / 1000 * 1000),
                mm(*max)
            ),
            IssueKind::SkewDelay { group, spread_fs, max_fs } => format!(
                "match group {group} (net {net}…): delay spread {} ps > allowed {} ps",
                fmt_ps(*spread_fs),
                fmt_ps(*max_fs)
            ),
            IssueKind::ViaCount { actual, max } => {
                format!("net {net}: {actual} vias > allowed {max}")
            }
            IssueKind::Impedance { target_mohm } => format!(
                "net {net}: impedance {:.1} Ω unattainable with this stack-up",
                f64::from(*target_mohm) / 1000.0
            ),
        }
    }
}

/// Femtoseconds as picoseconds with three decimals, in integer maths.
fn fmt_ps(fs: i64) -> String {
    format!("{}{}.{:03}", if fs < 0 { "-" } else { "" }, fs.abs() / 1000, fs.abs() % 1000)
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
    for a in &board.arcs {
        if let Some(l) = len.get_mut(a.net.idx()) {
            *l += a.length();
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
    for (i, net) in board.nets.iter().enumerate().skip(1) {
        let id = NetId(u32::try_from(i).unwrap_or(u32::MAX));
        let c = &net.constraints;
        let need = required_width(board, id);
        let narrowest = board
            .tracks
            .iter()
            .filter(|t| t.net == id)
            .map(|t| t.width)
            .chain(board.arcs.iter().filter(|a| a.net == id).map(|a| a.width))
            .min();
        if let Some(w) = narrowest.filter(|&w| w < need) {
            out.push(ConstraintIssue {
                net: id,
                kind: IssueKind::Width { actual: w, required: need },
            });
        }
        let l = lengths[i];
        if c.length_min.is_some_and(|m| l < m) || c.length_max.is_some_and(|m| l > m) {
            out.push(ConstraintIssue {
                net: id,
                kind: IssueKind::Length { actual: l, min: c.length_min, max: c.length_max },
            });
        }
        if let Some(z) = c.impedance_mohm
            && ferroute_board::constraints::impedance_unattainable(board, id)
        {
            out.push(ConstraintIssue { net: id, kind: IssueKind::Impedance { target_mohm: z } });
        }
        if let Some(max) = c.max_vias
            && vias[i] > max
        {
            out.push(ConstraintIssue {
                net: id,
                kind: IssueKind::ViaCount { actual: vias[i], max },
            });
        }
    }
    let delays = net_delays_fs(board);
    for g in match_groups(board) {
        let spread = g.spread(&lengths, &delays);
        if spread > g.skew_max {
            let kind = match g.by {
                MatchBy::Length => IssueKind::Skew { group: g.name, spread, max: g.skew_max },
                MatchBy::Delay => {
                    IssueKind::SkewDelay { group: g.name, spread_fs: spread, max_fs: g.skew_max }
                }
            };
            out.push(ConstraintIssue { net: g.nets[0], kind });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use ferroute_board::builder::{BoardBuilder, mm, pt10};
    use ferroute_board::constraints::MatchGroup;
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

    #[test]
    fn outer_group_violation_is_one_issue_while_inner_pairs_are_fine() {
        let mut b = BoardBuilder::new(mm(60), mm(30), 2);
        let ids: Vec<_> = ["A_P", "A_N", "B_P", "B_N"].iter().map(|n| b.net(n)).collect();
        for (k, (y, len)) in [(5, 100), (10, 101), (15, 160), (20, 161)].into_iter().enumerate() {
            b.track(pt10(10, y * 10), pt10(10 + len, y * 10), 250_000, 0, ids[k], Origin::Routed);
        }
        let mut board = b.build();
        let g = |name: &str, skew, nets: &[ferroute_board::NetId]| MatchGroup {
            name: name.into(),
            skew_max: skew,
            by: ferroute_board::constraints::MatchBy::Length,
            nets: nets.to_vec(),
        };
        board.match_groups =
            vec![g("a", mm(1) / 5, &ids[..2]), g("b", mm(1) / 5, &ids[2..]), g("bus", mm(1), &ids)];
        let issues = check_constraints(&board);
        assert_eq!(issues.len(), 1, "{issues:?}");
        assert!(matches!(&issues[0].kind, IssueKind::Skew { group, .. } if group == "bus"));
        assert_eq!(issues[0].net, ids[0]);
    }
    #[test]
    fn delay_group_reports_spread_in_ps_not_length() {
        // Equal physical length (10 mm), but B sits on an inner layer of a 4-layer board.
        let mut b = BoardBuilder::new(mm(40), mm(30), 4);
        let a = b.net("A");
        let c = b.net("B");
        b.track(pt10(10, 10), pt10(110, 10), 250_000, 0, a, Origin::Routed);
        b.track(pt10(10, 20), pt10(110, 20), 250_000, 1, c, Origin::Routed);
        let mut board = b.build();
        let group = |by, skew_max| MatchGroup { name: "g".into(), skew_max, by, nets: vec![a, c] };
        board.match_groups = vec![group(ferroute_board::constraints::MatchBy::Length, mm(1) / 10)];
        assert!(check_constraints(&board).is_empty(), "length group is met");
        board.match_groups = vec![group(ferroute_board::constraints::MatchBy::Delay, 2_000)];
        let issues = check_constraints(&board);
        assert_eq!(issues.len(), 1, "{issues:?}");
        let IssueKind::SkewDelay { spread_fs, max_fs, .. } = issues[0].kind else {
            panic!("{issues:?}")
        };
        assert!(spread_fs > 2_000 && max_fs == 2_000, "{spread_fs}");
        assert_eq!(issues[0].kind.label(), "skew");
        let text = issues[0].describe(&board);
        assert!(text.contains("delay spread") && text.contains("> allowed 2.000 ps"), "{text}");
        board.match_groups = vec![group(ferroute_board::constraints::MatchBy::Delay, 20_000)];
        assert!(check_constraints(&board).is_empty());
    }
}
