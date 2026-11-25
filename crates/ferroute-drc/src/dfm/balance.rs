// SPDX-License-Identifier: AGPL-3.0-only
//! Copper balance: per-layer copper area fraction and warping risk between layers that
//! are symmetric in the stack-up (layer `i` vs `n - 1 - i`).
//!
//! **Approximate by design**: tracks are capsules, pads and vias their convex hulls, fills
//! their exact polygon area; overlaps between items are *not* subtracted (the fraction is
//! clamped at 100 %), and drills and unfilled zones are ignored. All arithmetic is integer
//! (`i128`), so the result is a deterministic function of the board.

use super::DfmConfig;
use ferroute_board::{Board, LayerId};
use ferroute_geom::Hull;
use serde::{Deserialize, Serialize};

const PPM: i128 = 1_000_000;

/// Copper content of one layer.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LayerCopper {
    pub layer: LayerId,
    /// Approximate copper area (nm²).
    pub area: i128,
    /// Copper area over the board area in parts per million (clamped to 1 000 000).
    pub fraction_ppm: u32,
}

/// Two symmetric layers whose copper fractions differ by more than the threshold.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BalanceNote {
    pub a: LayerId,
    pub b: LayerId,
    pub a_ppm: u32,
    pub b_ppm: u32,
    pub threshold_ppm: u32,
}

impl BalanceNote {
    /// One-line description with layer names from `board`.
    #[must_use]
    pub fn describe(&self, board: &Board) -> String {
        let name = |l: LayerId| {
            board.copper_layers.get(usize::from(l.0)).map_or("?", |c| c.name.as_str()).to_owned()
        };
        let pct = |p: u32| format!("{}.{}%", p / 10_000, p % 10_000 / 1_000);
        format!(
            "copper balance: {} {} vs {} {} differ by more than {} points (warping risk, approximate)",
            name(self.a),
            pct(self.a_ppm),
            name(self.b),
            pct(self.b_ppm),
            self.threshold_ppm / 10_000
        )
    }
}

/// Per-layer copper fractions and the imbalance notes.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct CopperBalance {
    pub layers: Vec<LayerCopper>,
    pub notes: Vec<BalanceNote>,
}

/// Integer square root (floor).
fn isqrt(v: i128) -> i128 {
    if v <= 0 {
        return 0;
    }
    let mut x = v;
    let mut y = (x + 1) / 2;
    while y < x {
        x = y;
        y = (x + v / x) / 2;
    }
    x
}

/// Area (nm²) of a convex hull = polygon + perimeter · r + π r² (π ≈ 3.141593).
fn hull_area(h: &Hull) -> i128 {
    let n = h.pts.len();
    let (mut a2, mut perim) = (0_i128, 0_i128);
    if n > 1 {
        for i in 0..n {
            let (p, q) = (h.pts[i], h.pts[(i + 1) % n]);
            a2 += p.cross(q);
            let (dx, dy) = (i128::from(q.x - p.x), i128::from(q.y - p.y));
            perim += isqrt(dx * dx + dy * dy);
        }
    }
    let r = i128::from(h.radius);
    a2.abs() / 2 + perim * r + 3_141_593 * r * r / PPM
}

/// Computes the copper balance of `board`.
#[must_use]
pub fn copper_balance(board: &Board, cfg: &DfmConfig) -> CopperBalance {
    let n = usize::from(board.layer_count());
    let board_area = (board.outline.outer.area2().abs()
        - board.outline.holes.iter().map(|h| h.area2().abs()).sum::<i128>())
        / 2;
    if n == 0 || board_area <= 0 {
        return CopperBalance::default();
    }
    let mut area = vec![0_i128; n];
    let mut add = |l: LayerId, a: i128| {
        if let Some(s) = area.get_mut(usize::from(l.0)) {
            *s += a;
        }
    };
    for id in board.pad_ids() {
        let a = hull_area(&board.pad_hull(id));
        for l in board.pad(id).layers.iter() {
            add(l, a);
        }
    }
    for t in &board.tracks {
        add(t.layer, hull_area(&t.hull()));
    }
    for arc in &board.arcs {
        add(arc.layer, arc.hulls().iter().map(hull_area).sum());
    }
    for v in &board.vias {
        let a = hull_area(&v.hull());
        for l in v.layers.iter() {
            add(l, a);
        }
    }
    for z in &board.zones {
        for (l, p) in z.fill_polys() {
            add(l, p.area2().abs() / 2);
        }
    }
    let layers: Vec<LayerCopper> = area
        .iter()
        .enumerate()
        .map(|(i, &a)| LayerCopper {
            layer: LayerId(u8::try_from(i).unwrap_or(u8::MAX)),
            area: a,
            fraction_ppm: u32::try_from((a * PPM / board_area).min(PPM)).unwrap_or(0),
        })
        .collect();
    let notes = (0..n / 2)
        .filter_map(|i| {
            let (a, b) = (layers[i], layers[n - 1 - i]);
            (a.fraction_ppm.abs_diff(b.fraction_ppm) > cfg.balance_threshold_ppm).then_some(
                BalanceNote {
                    a: a.layer,
                    b: b.layer,
                    a_ppm: a.fraction_ppm,
                    b_ppm: b.fraction_ppm,
                    threshold_ppm: cfg.balance_threshold_ppm,
                },
            )
        })
        .collect();
    CopperBalance { layers, notes }
}
