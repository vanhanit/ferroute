// SPDX-License-Identifier: AGPL-3.0-only
//! Differential pairs: two nets routed as one coupled unit (constant edge-to-edge gap,
//! equal length, symmetric routing). Pairs come from the rules file (`[[pair]]`) and are
//! stored in [`Board::diff_pairs`]; the router couples them, the DRC oracle checks them.

use crate::constraints::required_width;
use crate::{Board, NetId};
use ferroute_geom::Coord;
use serde::{Deserialize, Serialize};

/// Default intra-pair skew (0.1 mm) when a pair rule does not set one.
pub const DEFAULT_PAIR_SKEW: Coord = 100_000;

/// One differential pair.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DiffPair {
    /// Positive net.
    pub p: NetId,
    /// Negative net.
    pub n: NetId,
    /// Target edge-to-edge gap of the coupled sections (nm); never below the clearance.
    pub gap: Coord,
    /// Required trace width (nm); the effective width is never below the nets' own rule
    /// widths ([`pair_width`]).
    pub width: Option<Coord>,
    /// Longest allowed total length of uncoupled copper per trace (nm).
    pub max_uncoupled: Option<Coord>,
    /// Allowed length difference between the two traces (nm); [`DEFAULT_PAIR_SKEW`] if unset.
    pub skew_max: Option<Coord>,
}

impl DiffPair {
    /// `P/N` display name.
    #[must_use]
    pub fn name(&self, board: &Board) -> String {
        let n = |id: NetId| board.nets.get(id.idx()).map_or("?", |n| n.name.as_str());
        format!("{}/{}", n(self.p), n(self.n))
    }

    #[must_use]
    pub fn contains(&self, net: NetId) -> bool {
        self.p == net || self.n == net
    }

    /// Effective skew limit.
    #[must_use]
    pub fn skew(&self) -> Coord {
        self.skew_max.unwrap_or(DEFAULT_PAIR_SKEW)
    }
}

/// The pair `net` belongs to.
#[must_use]
pub fn pair_of(board: &Board, net: NetId) -> Option<&DiffPair> {
    board.diff_pairs.iter().find(|p| p.contains(net))
}

/// Trace width of a pair: the widest of the pair rule's width and both nets' required
/// widths (INV-RULE-1: a pair never narrows a net).
#[must_use]
pub fn pair_width(board: &Board, pair: &DiffPair) -> Coord {
    pair.width.unwrap_or(0).max(required_width(board, pair.p)).max(required_width(board, pair.n))
}

/// Explicit pair width demanded for `net` (0 when it is in no pair or the pair sets none).
#[must_use]
pub fn pair_min_width(board: &Board, net: NetId) -> Coord {
    pair_of(board, net).and_then(|p| p.width).unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builder::{BoardBuilder, mm};

    #[test]
    fn pair_width_never_narrows_a_net() {
        let mut b = BoardBuilder::new(mm(10), mm(10), 2);
        let (p, n) = (b.net("D+"), b.net("D-"));
        let mut board = b.build();
        let class_w = board.rules.classes[0].track_width;
        let pair = DiffPair {
            p,
            n,
            gap: 200_000,
            width: Some(class_w / 2),
            max_uncoupled: None,
            skew_max: None,
        };
        assert_eq!(pair_width(&board, &pair), class_w);
        let wide = DiffPair { width: Some(class_w * 2), ..pair };
        assert_eq!(pair_width(&board, &wide), class_w * 2);
        board.diff_pairs.push(wide);
        assert_eq!(pair_min_width(&board, p), class_w * 2);
        assert_eq!(pair_min_width(&board, NetId(0)), 0);
        assert_eq!(pair_of(&board, n).map(|d| d.name(&board)).as_deref(), Some("D+/D-"));
        assert_eq!(board.diff_pairs[0].skew(), DEFAULT_PAIR_SKEW);
    }
}
