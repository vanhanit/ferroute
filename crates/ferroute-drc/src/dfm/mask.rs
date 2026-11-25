// SPDX-License-Identifier: AGPL-3.0-only
//! Solder-mask sliver check: the mask web between two adjacent pad openings.
//!
//! An opening is the pad hull grown by the pad's mask margin (pad override, else
//! footprint override, else the board `pad_to_mask_clearance`). Hulls are convex
//! supersets, so the web is a lower bound: a reported sliver may be slightly pessimistic,
//! a missed one is impossible (a negative margin keeps the unshrunk shape). Decisions use
//! the exact hull predicate; openings that touch or overlap merge into one and have no web.

use super::{DfmConfig, DfmIssue, DfmKind};
use crate::{BucketGrid, ItemRef};
use ferroute_board::{Board, LayerId, PadId};
use ferroute_geom::{Coord, Hull, Rect};

/// Default minimum mask web (nm) when neither the config nor the board sets one.
pub(super) const DEFAULT_MIN_WEB: Coord = 100_000;

/// Effective minimum web: explicit config, else the board's `solder_mask_min_width`,
/// else [`DEFAULT_MIN_WEB`].
fn min_web(board: &Board, cfg: &DfmConfig) -> Coord {
    cfg.min_mask_web.unwrap_or(if board.mask.min_width > 0 {
        board.mask.min_width
    } else {
        DEFAULT_MIN_WEB
    })
}

pub(super) fn mask_slivers(board: &Board, cfg: &DfmConfig) -> Vec<DfmIssue> {
    let required = min_web(board, cfg);
    let n = board.layer_count();
    let sides = if n > 1 { vec![LayerId(0), LayerId(n - 1)] } else { vec![LayerId(0)] };
    let mut out = Vec::new();
    for side in sides {
        let open: Vec<(PadId, Hull)> = board
            .pad_ids()
            .filter(|&id| board.pad(id).layers.contains(side))
            .map(|id| {
                let mut h = board.pad_hull(id);
                h.radius = (h.radius + board.mask.margin_for(id)).max(0);
                (id, h)
            })
            .collect();
        let boxes: Vec<Rect> = open.iter().map(|(_, h)| h.bbox()).collect();
        let grid = BucketGrid::new(&boxes, 2_000_000);
        for (j, (id, hull)) in open.iter().enumerate() {
            let mut near = grid.query(hull.bbox().inflate(required + 1), &boxes);
            near.sort_unstable();
            for k in near {
                let k = k as usize;
                if k <= j {
                    continue;
                }
                let (other, oh) = &open[k];
                if !hull.clearance_violated(oh, required) || hull.clearance_violated(oh, 1) {
                    continue;
                }
                #[allow(clippy::cast_possible_truncation)]
                let actual = hull.distance(oh).floor() as Coord;
                out.push(DfmIssue {
                    kind: DfmKind::MaskSliver { actual, required, other: ItemRef::Pad(*other) },
                    item: ItemRef::Pad(*id),
                    at: board.pad_position(*id),
                });
            }
        }
    }
    out
}
