// SPDX-License-Identifier: AGPL-3.0-only
//! Design-for-manufacturing checks (task #22). Like [`crate::rules`] these are
//! *constraint-style* reports, not hard DRC violations: they never gate a result, they
//! tell the designer what a fab may reject.
//!
//! * annular ring of plated pads and vias against the minimum ring (the larger of the
//!   configured minimum and any KiCad `annular_width` custom rule),
//! * drill aspect ratio (`board thickness / drill`, scaled to the layers a via spans),
//! * minimum drill size of plated holes,
//! * copper-to-hole clearance of non-plated (NPTH) holes,
//! * solder-mask slivers between adjacent pad openings (`mask`),
//! * per-layer copper balance and warping risk (`balance`, reported separately).
//!
//! All decisions use integer arithmetic (INV-GEOM-1); the NPTH clearance uses the exact
//! hull predicate of `ferroute-geom`.

mod balance;
mod mask;

pub use balance::{BalanceNote, CopperBalance, LayerCopper, copper_balance};

use crate::{BucketGrid, CopperItem, ItemRef, copper_items, idx};
use ferroute_board::custom::{ItemCtx, ItemType};
use ferroute_board::{Board, NetId, PadShape, ViaKind};
use ferroute_geom::units::format_mm;
use ferroute_geom::{Coord, Hull, Point, Rect};
use serde::{Deserialize, Serialize};

/// Thresholds of the DFM checks.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DfmConfig {
    /// Minimum annular ring (nm); a custom `annular_width` rule may raise it per item.
    pub min_annular: Coord,
    /// Maximum drill aspect ratio in thousandths (10 000 = 10:1).
    pub max_aspect_milli: u32,
    /// Minimum plated drill diameter (nm).
    pub min_drill: Coord,
    /// Maximum aspect ratio of a microvia in thousandths (1 000 = 1:1; the dielectric it
    /// crosses divided by its drill).
    pub max_aspect_micro_milli: u32,
    /// Minimum microvia (laser) drill diameter (nm).
    pub min_micro_drill: Coord,
    /// Minimum copper edge to NPTH hole edge distance (nm).
    pub npth_clearance: Coord,
    /// Minimum solder-mask web between pad openings (nm). `None` = the board's
    /// `solder_mask_min_width` when set, else 0.1 mm.
    pub min_mask_web: Option<Coord>,
    /// Copper-fraction difference (ppm of the board area) between stack-up-symmetric
    /// layers above which a warping note is raised (300 000 = 30 percentage points).
    pub balance_threshold_ppm: u32,
}

impl Default for DfmConfig {
    fn default() -> Self {
        Self {
            min_annular: 130_000,
            max_aspect_milli: 10_000,
            min_drill: 200_000,
            max_aspect_micro_milli: 1_000,
            min_micro_drill: 100_000,
            npth_clearance: 250_000,
            min_mask_web: None,
            balance_threshold_ppm: 300_000,
        }
    }
}

/// Which manufacturing limit is exceeded.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum DfmKind {
    /// Annular ring (nm) narrower than required.
    AnnularRing { actual: Coord, required: Coord },
    /// Drill aspect ratio above the maximum (both in thousandths).
    AspectRatio { actual_milli: u64, max_milli: u32 },
    /// Plated drill smaller than the fab minimum.
    MinDrill { actual: Coord, required: Coord },
    /// Copper closer than allowed to a non-plated hole (nm, edge to edge).
    NpthCopper { actual: Coord, required: Coord, copper: ItemRef },
    /// Solder-mask web (nm) between this pad's opening and `other`'s narrower than the
    /// fab minimum.
    MaskSliver { actual: Coord, required: Coord, other: ItemRef },
}

impl DfmKind {
    /// Short stable label (reports, JSON consumers).
    #[must_use]
    pub const fn label(&self) -> &'static str {
        match self {
            Self::AnnularRing { .. } => "annular-ring",
            Self::AspectRatio { .. } => "aspect-ratio",
            Self::MinDrill { .. } => "min-drill",
            Self::NpthCopper { .. } => "npth-copper",
            Self::MaskSliver { .. } => "mask-sliver",
        }
    }
}

/// One DFM finding: the offending item and where it is.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DfmIssue {
    pub kind: DfmKind,
    pub item: ItemRef,
    pub at: Point,
}

impl DfmIssue {
    /// One-line description with the item named by `label` (mm for lengths).
    #[must_use]
    pub fn describe(&self, item_label: &str) -> String {
        let (x, y) = (format_mm(self.at.x), format_mm(self.at.y));
        let what = match &self.kind {
            DfmKind::AnnularRing { actual, required } => format!(
                "annular ring {} mm < required {} mm",
                format_mm(*actual),
                format_mm(*required)
            ),
            DfmKind::AspectRatio { actual_milli, max_milli } => format!(
                "drill aspect ratio {}.{:02}:1 > allowed {}.{:02}:1",
                actual_milli / 1000,
                actual_milli % 1000 / 10,
                max_milli / 1000,
                max_milli % 1000 / 10
            ),
            DfmKind::MinDrill { actual, required } => {
                format!("drill {} mm < minimum {} mm", format_mm(*actual), format_mm(*required))
            }
            DfmKind::NpthCopper { actual, required, .. } => format!(
                "copper {} mm from an NPTH hole < required {} mm",
                format_mm(*actual),
                format_mm(*required)
            ),
            DfmKind::MaskSliver { actual, required, .. } => format!(
                "mask sliver: web {} mm between pad openings < minimum {} mm",
                format_mm(*actual),
                format_mm(*required)
            ),
        };
        format!("{item_label} at ({x}, {y}) mm: {what}")
    }
}

/// Smallest copper dimension of a pad (the ring is measured across it); `None` for
/// custom polygons, whose inscribed size is not derivable from a convex superset.
pub(crate) fn pad_min_dimension(shape: &PadShape) -> Option<Coord> {
    match shape {
        PadShape::Circle { diameter } => Some(*diameter),
        PadShape::Rect { w, h } | PadShape::Oval { w, h } | PadShape::RoundRect { w, h, .. } => {
            Some((*w).min(*h))
        }
        PadShape::Convex { .. } => None,
    }
}

/// `true` when `thickness / drill` exceeds `max_milli / 1000` (exact, no floats).
fn aspect_exceeds(thickness: Coord, drill: Coord, max_milli: u32) -> bool {
    drill > 0 && i128::from(thickness) * 1000 > i128::from(max_milli) * i128::from(drill)
}

fn aspect_milli(thickness: Coord, drill: Coord) -> u64 {
    u64::try_from(i128::from(thickness) * 1000 / i128::from(drill.max(1))).unwrap_or(u64::MAX)
}

/// Runs every DFM check. Deterministic: pads, then vias, then NPTH holes, each in board
/// order.
#[must_use]
pub fn check(board: &Board, cfg: &DfmConfig) -> Vec<DfmIssue> {
    let mut out = Vec::new();
    let custom = !board.rules.custom.is_empty();
    let full = board.stackup.board_thickness;
    let required_ring = |net: NetId, kind: ItemType| {
        if custom {
            cfg.min_annular.max(board.rules.min_annular_for(board, &ItemCtx::new(net, kind)))
        } else {
            cfg.min_annular
        }
    };
    let mut hole =
        |item: ItemRef, at: Point, drill: Coord, ring: Option<(Coord, Coord)>, t, micro: bool| {
            let (min_drill, max_aspect) = if micro {
                (cfg.min_micro_drill, cfg.max_aspect_micro_milli)
            } else {
                (cfg.min_drill, cfg.max_aspect_milli)
            };
            if let Some((actual, required)) = ring.filter(|&(a, r)| a < r) {
                out.push(DfmIssue { kind: DfmKind::AnnularRing { actual, required }, item, at });
            }
            if drill < min_drill {
                out.push(DfmIssue {
                    kind: DfmKind::MinDrill { actual: drill, required: min_drill },
                    item,
                    at,
                });
            }
            if aspect_exceeds(t, drill, max_aspect) {
                out.push(DfmIssue {
                    kind: DfmKind::AspectRatio {
                        actual_milli: aspect_milli(t, drill),
                        max_milli: max_aspect,
                    },
                    item,
                    at,
                });
            }
        };
    for id in board.pad_ids() {
        let pad = board.pad(id);
        let Some(d) = pad.drill.filter(|d| d.plated) else { continue };
        // A slot's ring is measured against its width, not its length.
        let dim = d.min_dimension();
        let ring = pad_min_dimension(&pad.shape)
            .map(|m| ((m - dim) / 2, required_ring(pad.net, ItemType::Pad)));
        hole(ItemRef::Pad(id), board.pad_position(id), dim, ring, full, false);
    }
    let spans = i64::from(board.layer_count().saturating_sub(1)).max(1);
    for (i, v) in board.vias.iter().enumerate() {
        let n = i64::try_from(v.layers.iter().count()).unwrap_or(1).saturating_sub(1).max(1);
        let ring = ((v.diameter - v.drill) / 2, required_ring(v.net, ItemType::Via));
        // Through-style vias scale the board thickness with the gaps they span; typed
        // blind / buried / micro vias use the real dielectrics they cross when known.
        let depth = if v.kind == ViaKind::Through {
            full * n / spans
        } else {
            span_depth(board, v).unwrap_or(full * n / spans)
        };
        hole(ItemRef::Via(idx(i)), v.pos, v.drill, Some(ring), depth, v.kind == ViaKind::Micro);
    }
    out.extend(npth_copper(board, cfg));
    out.extend(mask::mask_slivers(board, cfg));
    out
}

/// Dielectric (plus inner copper) thickness a non-through via crosses, from the stack-up
/// detail; `None` when the board file gave none.
fn span_depth(board: &Board, v: &ferroute_board::Via) -> Option<Coord> {
    let ids: Vec<_> = v.layers.iter().collect();
    let (lo, hi) = (ids.first()?.0, ids.last()?.0);
    let st = &board.stackup;
    if st.dielectrics.is_empty() {
        return None;
    }
    let diel: Coord = st
        .dielectrics
        .iter()
        .filter(|d| d.between.0.0 >= lo && d.between.1.0 <= hi)
        .map(|d| d.thickness)
        .sum();
    let inner: Coord =
        (lo + 1..hi).filter_map(|l| st.copper_thickness.get(usize::from(l)).copied()).sum();
    Some(diel + inner)
}

fn npth_copper(board: &Board, cfg: &DfmConfig) -> Vec<DfmIssue> {
    let items: Vec<CopperItem> = copper_items(board);
    let boxes: Vec<Rect> = items.iter().map(|it| it.bbox).collect();
    let grid = BucketGrid::new(&boxes, 2_000_000);
    let mut out = Vec::new();
    for id in board.pad_ids() {
        let pad = board.pad(id);
        let Some(d) = pad.drill.filter(|d| !d.plated) else { continue };
        let at = board.pad_position(id);
        let hull = Hull::disk(at, d.diameter / 2);
        for k in grid.query(hull.bbox().inflate(cfg.npth_clearance + 1), &boxes) {
            let it = &items[k as usize];
            if it.item == ItemRef::Pad(id) || !it.hull.clearance_violated(&hull, cfg.npth_clearance)
            {
                continue;
            }
            #[allow(clippy::cast_possible_truncation)]
            let actual = it.hull.distance(&hull).floor() as Coord;
            out.push(DfmIssue {
                kind: DfmKind::NpthCopper { actual, required: cfg.npth_clearance, copper: it.item },
                item: ItemRef::Pad(id),
                at,
            });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use ferroute_board::builder::{BoardBuilder, mm, pt10};
    use ferroute_board::custom::{Cond, ConstraintKind, CustomRule, Limits, Severity};
    use ferroute_board::{Drill, Origin};

    fn via_board(diameter: Coord, drill: Coord) -> Board {
        let mut b = BoardBuilder::new(mm(20), mm(20), 2);
        let n = b.net("N");
        b.via(pt10(100, 100), diameter, drill, n, Origin::Routed);
        b.build()
    }

    fn kinds(board: &Board, cfg: &DfmConfig) -> Vec<DfmKind> {
        check(board, cfg).into_iter().map(|i| i.kind).collect()
    }

    #[test]
    fn annular_ring_hit_and_miss() {
        let cfg = DfmConfig { min_drill: 0, ..DfmConfig::default() };
        // ring (0.5 - 0.3) / 2 = 0.1 mm < 0.13 mm
        assert_eq!(
            kinds(&via_board(500_000, 300_000), &cfg),
            vec![DfmKind::AnnularRing { actual: 100_000, required: 130_000 }]
        );
        // ring 0.15 mm passes
        assert!(kinds(&via_board(600_000, 300_000), &cfg).is_empty());
    }

    #[test]
    fn custom_annular_rule_raises_the_minimum() {
        let mut board = via_board(600_000, 300_000);
        let cfg = DfmConfig::default();
        assert!(kinds(&board, &cfg).is_empty());
        board.rules.custom.push(CustomRule {
            name: "ring".into(),
            condition: Cond::True,
            layer: None,
            constraints: vec![(
                ConstraintKind::AnnularWidth,
                Limits { min: Some(200_000), ..Limits::default() },
            )],
            severity: Severity::Error,
        });
        assert_eq!(
            kinds(&board, &cfg),
            vec![DfmKind::AnnularRing { actual: 150_000, required: 200_000 }]
        );
    }

    #[test]
    fn slot_ring_uses_the_slot_width_not_its_length() {
        let mut b = BoardBuilder::new(mm(20), mm(20), 2);
        let n = b.net("N");
        b.header("J1", pt10(100, 100), 0.0, &[n, n]);
        let mut board = b.build();
        // Oval 1.0 x 2.4 mm pad with a 0.6 x 1.8 mm slot: ring = (1.0 - 0.6) / 2 = 0.2 mm.
        board.pads[1].shape = PadShape::Oval { w: 1_000_000, h: 2_400_000 };
        board.pads[1].drill =
            Some(Drill { diameter: 1_800_000, plated: true, slot: Some([600_000, 1_800_000]) });
        let cfg = DfmConfig::default();
        assert!(kinds(&board, &cfg).is_empty(), "{:?}", check(&board, &cfg));
        // A 0.9 mm wide slot leaves only 0.05 mm.
        board.pads[1].drill =
            Some(Drill { diameter: 1_800_000, plated: true, slot: Some([1_800_000, 900_000]) });
        let got = check(&board, &cfg);
        assert_eq!(got[0].kind, DfmKind::AnnularRing { actual: 50_000, required: 130_000 });
    }

    #[test]
    fn pad_annular_ring_uses_the_smallest_copper_dimension() {
        let mut b = BoardBuilder::new(mm(20), mm(20), 2);
        let n = b.net("N");
        b.header("J1", pt10(100, 100), 0.0, &[n, n]);
        let mut board = b.build();
        let cfg = DfmConfig::default();
        assert!(kinds(&board, &cfg).is_empty(), "1.7 mm pad, 1.0 mm drill");
        // Pad 2 is a 1.7 mm circle; a 1.5 mm drill leaves 0.1 mm of ring.
        board.pads[1].drill = Some(Drill::round(1_500_000, true));
        let got = check(&board, &cfg);
        assert_eq!(got.len(), 1, "{got:?}");
        assert_eq!(got[0].kind, DfmKind::AnnularRing { actual: 100_000, required: 130_000 });
        assert_eq!(got[0].item, ItemRef::Pad(ferroute_board::PadId(1)));
    }

    #[test]
    fn aspect_ratio_hit_and_miss() {
        let cfg = DfmConfig { min_annular: 0, ..DfmConfig::default() };
        let mut board = via_board(500_000, 200_000);
        board.stackup.board_thickness = 2_000_000; // 10.0:1 is allowed
        assert!(kinds(&board, &cfg).is_empty());
        board.stackup.board_thickness = 2_100_000; // 10.5:1
        assert_eq!(
            kinds(&board, &cfg),
            vec![DfmKind::AspectRatio { actual_milli: 10_500, max_milli: 10_000 }]
        );
        let loose = DfmConfig { max_aspect_milli: 12_000, ..cfg };
        assert!(kinds(&board, &loose).is_empty());
    }

    #[test]
    fn blind_via_aspect_ratio_scales_with_the_layers_it_spans() {
        let mut b = BoardBuilder::new(mm(20), mm(20), 4);
        let n = b.net("N");
        b.via(pt10(100, 100), 500_000, 150_000, n, Origin::Routed);
        let mut board = b.build();
        board.stackup.board_thickness = 3_000_000;
        board.vias[0].layers =
            ferroute_board::LayerSet::span(ferroute_board::LayerId(0), ferroute_board::LayerId(1));
        let cfg = DfmConfig { min_drill: 0, min_annular: 0, ..DfmConfig::default() };
        // spans one of three gaps: 1 mm / 0.15 mm = 6.67:1
        assert!(kinds(&board, &cfg).is_empty());
        board.vias[0].layers = board.all_layers(); // 3 mm / 0.15 mm = 20:1
        assert_eq!(
            kinds(&board, &cfg),
            vec![DfmKind::AspectRatio { actual_milli: 20_000, max_milli: 10_000 }]
        );
    }

    #[test]
    fn microvia_has_its_own_drill_minimum_and_aspect_limit() {
        use ferroute_board::{LayerId, LayerSet, ViaKind};
        let mut b = BoardBuilder::new(mm(20), mm(20), 4);
        let n = b.net("N");
        b.via(pt10(100, 100), 300_000, 100_000, n, Origin::Routed);
        let mut board = b.build();
        board.stackup.board_thickness = 1_600_000;
        board.vias[0].kind = ViaKind::Micro;
        board.vias[0].layers = LayerSet::span(LayerId(0), LayerId(1));
        let cfg = DfmConfig { min_annular: 0, ..DfmConfig::default() };
        // 0.1 mm laser drill is fine for a microvia; one gap = 1.6/3 mm → 5.33:1 > 1:1
        // without real dielectric data.
        assert_eq!(
            kinds(&board, &cfg),
            vec![DfmKind::AspectRatio { actual_milli: 5_333, max_milli: 1_000 }]
        );
        // With the dielectric known (0.1 mm prepreg) the microvia is 1:1.
        board.stackup.dielectrics = vec![ferroute_board::constraints::Dielectric {
            thickness: 100_000,
            epsilon_r_milli: 4_000,
            between: (LayerId(0), LayerId(1)),
        }];
        assert!(kinds(&board, &cfg).is_empty());
        let strict = DfmConfig { max_aspect_micro_milli: 800, ..cfg };
        assert_eq!(
            kinds(&board, &strict),
            vec![DfmKind::AspectRatio { actual_milli: 1_000, max_milli: 800 }]
        );
        board.vias[0].drill = 80_000;
        assert!(kinds(&board, &strict).iter().any(|k| matches!(k, DfmKind::MinDrill { .. })));
    }

    #[test]
    fn minimum_drill_hit_and_miss() {
        let cfg = DfmConfig { min_annular: 0, max_aspect_milli: 20_000, ..DfmConfig::default() };
        assert_eq!(
            kinds(&via_board(500_000, 150_000), &cfg),
            vec![DfmKind::MinDrill { actual: 150_000, required: 200_000 }]
        );
        assert!(kinds(&via_board(500_000, 200_000), &cfg).is_empty());
    }

    #[test]
    fn npth_copper_hit_and_miss() {
        let mut b = BoardBuilder::new(mm(20), mm(20), 2);
        let n = b.net("N");
        b.header("J1", pt10(100, 100), 0.0, &[n]);
        b.via(pt10(130, 100), 600_000, 300_000, n, Origin::Existing);
        let mut board = b.build();
        // Turn the header pad into a 3.2 mm NPTH mounting hole with no copper.
        board.pads[0].drill = Some(Drill::round(3_200_000, false));
        board.pads[0].layers = ferroute_board::LayerSet::EMPTY;
        let cfg = DfmConfig { min_annular: 0, ..DfmConfig::default() };
        // Via at 3.0 mm: its edge is 3.0 - 0.3 - 1.6 = 1.1 mm from the hole edge.
        assert!(kinds(&board, &cfg).is_empty());
        let tight = DfmConfig { npth_clearance: 1_200_000, ..cfg };
        let got = check(&board, &tight);
        assert_eq!(got.len(), 1, "{got:?}");
        assert!(matches!(
            got[0].kind,
            DfmKind::NpthCopper { required: 1_200_000, copper: ItemRef::Via(0), .. }
        ));
        assert_eq!(got[0].item, ItemRef::Pad(ferroute_board::PadId(0)));
    }

    #[test]
    fn describe_names_item_location_and_limits() {
        let issues = check(&via_board(500_000, 300_000), &DfmConfig::default());
        let s = issues[0].describe("via#0");
        assert!(s.contains("via#0") && s.contains("annular ring 0.1 mm < required 0.13 mm"), "{s}");
    }
    // ---- solder-mask slivers ----

    /// Two 0.5 mm square SMD pads whose copper gap is `gap` nm.
    fn two_pads(gap: Coord) -> Board {
        use ferroute_board::builder::PadSpec;
        let mut b = BoardBuilder::new(mm(20), mm(20), 2);
        let n = b.net("N");
        let spec = |num: &str, x: Coord| PadSpec {
            number: num.into(),
            local_pos: Point::new(x, 0),
            shape: PadShape::Rect { w: 500_000, h: 500_000 },
            drill: None,
            net: n,
        };
        let pitch = 500_000 + gap;
        b.footprint(
            "U1",
            pt10(100, 100),
            0.0,
            ferroute_board::Side::Front,
            &[spec("1", 0), spec("2", pitch)],
        );
        b.build()
    }

    fn slivers(board: &Board) -> Vec<DfmKind> {
        kinds(board, &DfmConfig::default())
            .into_iter()
            .filter(|k| matches!(k, DfmKind::MaskSliver { .. }))
            .collect()
    }

    #[test]
    fn mask_sliver_hit_and_miss() {
        assert_eq!(
            slivers(&two_pads(50_000)),
            vec![DfmKind::MaskSliver {
                actual: 50_000,
                required: 100_000,
                other: ItemRef::Pad(ferroute_board::PadId(1))
            }]
        );
        assert!(slivers(&two_pads(200_000)).is_empty());
    }

    #[test]
    fn mask_margin_shrinks_the_web() {
        let mut b = two_pads(300_000);
        assert!(slivers(&b).is_empty());
        b.mask.margin = 120_000; // web = 0.3 - 2 * 0.12 = 0.06 mm
        let got = slivers(&b);
        assert!(matches!(got[..], [DfmKind::MaskSliver { actual: 60_000, .. }]), "{got:?}");
        // A per-pad override on one pad only grows its side: 0.3 - 0.12 - 0.0 = 0.18 mm.
        b.mask.pad_margins = vec![Some(0), None];
        assert!(slivers(&b).is_empty());
    }

    #[test]
    fn board_min_width_sets_the_default_and_config_wins() {
        let mut b = two_pads(150_000);
        assert!(slivers(&b).is_empty());
        b.mask.min_width = 200_000;
        assert_eq!(slivers(&b).len(), 1);
        let cfg = DfmConfig { min_mask_web: Some(100_000), ..DfmConfig::default() };
        assert!(kinds(&b, &cfg).is_empty());
    }

    #[test]
    fn touching_openings_merge_and_have_no_web() {
        assert!(slivers(&two_pads(0)).is_empty());
        let mut b = two_pads(100_000);
        b.mask.margin = 50_000; // openings touch
        assert!(slivers(&b).is_empty());
    }

    #[test]
    fn mask_sliver_describe_and_determinism() {
        let b = two_pads(50_000);
        let a = check(&b, &DfmConfig::default());
        assert_eq!(a, check(&b, &DfmConfig::default()));
        let s = a[0].describe("U1.1");
        assert!(s.contains("mask sliver") && s.contains("0.05 mm"), "{s}");
        assert_eq!(a[0].kind.label(), "mask-sliver");
    }

    // ---- copper balance ----

    fn four_layer(f: bool, back: bool) -> Board {
        let mut b = BoardBuilder::new(mm(20), mm(20), 4);
        let n = b.net("N");
        // 15 mm x 10 mm track = 150 mm2 of 400 mm2 + round end caps ≈ 57 %.
        if f {
            b.track(pt10(25, 100), pt10(175, 100), mm(10), 0, n, Origin::Existing);
        }
        if back {
            b.track(pt10(25, 100), pt10(175, 100), mm(10), 3, n, Origin::Existing);
        }
        b.build()
    }

    #[test]
    fn unbalanced_four_layer_board_is_noted() {
        let board = four_layer(true, false);
        let bal = copper_balance(&board, &DfmConfig::default());
        assert_eq!(bal.layers.len(), 4);
        assert!(bal.layers[0].fraction_ppm > 560_000 && bal.layers[0].fraction_ppm < 580_000);
        assert_eq!(bal.layers[3].fraction_ppm, 0);
        assert_eq!(bal.notes.len(), 1);
        assert_eq!(
            (bal.notes[0].a, bal.notes[0].b),
            (ferroute_board::LayerId(0), ferroute_board::LayerId(3))
        );
        let s = bal.notes[0].describe(&board);
        assert!(s.contains("F.Cu") && s.contains("B.Cu") && s.contains("warping"), "{s}");
    }

    #[test]
    fn balanced_or_empty_boards_have_no_note() {
        assert!(copper_balance(&four_layer(true, true), &DfmConfig::default()).notes.is_empty());
        assert!(copper_balance(&four_layer(false, false), &DfmConfig::default()).notes.is_empty());
        // A looser threshold silences the unbalanced board.
        let loose = DfmConfig { balance_threshold_ppm: 600_000, ..DfmConfig::default() };
        assert!(copper_balance(&four_layer(true, false), &loose).notes.is_empty());
    }

    #[test]
    fn copper_balance_counts_fills_exactly_and_is_deterministic() {
        use ferroute_board::{PadConnection, Zone, ZoneFill};
        use ferroute_geom::Polygon;
        let mut board = four_layer(false, false);
        let fill = Polygon::from_rect(Rect::new(Point::ORIGIN, Point::new(mm(20), mm(10))));
        board.zones.push(Zone {
            net: ferroute_board::NetId(1),
            layers: ferroute_board::LayerSet::single(ferroute_board::LayerId(1)),
            outline: fill.clone(),
            fills: vec![ZoneFill { layer: ferroute_board::LayerId(1), polys: vec![fill] }],
            clearance: 0,
            min_thickness: 0,
            priority: 0,
            pad_connection: PadConnection::default(),
            islands: Default::default(),
            yields: false,
        });
        let a = copper_balance(&board, &DfmConfig::default());
        assert_eq!(a.layers[1].fraction_ppm, 500_000);
        assert_eq!(a.layers[1].area, 200_000_000_000_000);
        assert_eq!(a.notes.len(), 1); // In1 50 % vs In2 0 %
        assert_eq!(a, copper_balance(&board, &DfmConfig::default()));
    }
}
