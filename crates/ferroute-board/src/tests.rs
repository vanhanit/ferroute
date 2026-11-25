// SPDX-License-Identifier: AGPL-3.0-only
use super::*;
use ferroute_geom::{Angle, Point, Polygon, Rect};

/// Minimal two-pad board used by several unit tests.
pub(crate) fn tiny_board() -> Board {
    let mut b = Board {
        copper_layers: vec![
            CopperLayer { name: "F.Cu".into(), routable: true },
            CopperLayer { name: "B.Cu".into(), routable: true },
        ],
        nets: vec![Net::new(String::new(), 0), Net::new("SIG", 0)],
        outline: Outline {
            outer: Polygon::from_rect(Rect::new(
                Point::new(0, 0),
                Point::new(20_000_000, 10_000_000),
            )),
            holes: vec![],
        },
        ..Board::default()
    };
    let pl = Placement { pos: Point::new(5_000_000, 5_000_000), rot: Angle::ZERO };
    b.footprints.push(Footprint {
        reference: "R1".into(),
        lib_id: "R_0603".into(),
        placement: pl,
        original: pl,
        side: Side::Front,
        locked: false,
        mobility: Mobility::Fixed,
        courtyard_local: Polygon::from_rect(Rect::new(
            Point::new(-1_500_000, -800_000),
            Point::new(1_500_000, 800_000),
        )),
        has_courtyard: true,
        pads: vec![PadId(0), PadId(1)],
    });
    for (i, x) in [-800_000, 800_000].into_iter().enumerate() {
        b.pads.push(Pad {
            footprint: FootprintId(0),
            number: format!("{}", i + 1),
            net: if i == 0 { NetId(1) } else { NetId::NONE },
            local_pos: Point::new(x, 0),
            local_rot: Angle::ZERO,
            shape: PadShape::RoundRect { w: 800_000, h: 950_000, radius: 200_000 },
            layers: LayerSet::single(LayerId(0)),
            drill: None,
        });
    }
    b
}

#[test]
fn tiny_board_validates() {
    tiny_board().validate().expect("valid");
}

#[test]
fn pad_hull_follows_rotation() {
    let mut b = tiny_board();
    let before = b.pad_position(PadId(0));
    assert_eq!(before, Point::new(4_200_000, 5_000_000));
    b.footprints[0].placement.rot = Angle::from_degrees(90.0);
    // Local (-0.8, 0) rotated +90° (KiCad) = (0, +0.8) → below the anchor in y-down.
    assert_eq!(b.pad_position(PadId(0)), Point::new(5_000_000, 5_800_000));
    let h = b.pad_hull(PadId(0));
    let bb = h.bbox();
    // Rotated pad: 0.95 wide in x, 0.8 tall in y.
    assert_eq!(bb.width(), 950_000);
    assert_eq!(bb.height(), 800_000);
}

#[test]
fn layer_set_ops() {
    let s = LayerSet::span(LayerId(3), LayerId(1));
    assert_eq!(s.iter().map(|l| l.0).collect::<Vec<_>>(), vec![1, 2, 3]);
    assert!(LayerSet::all(2).contains(LayerId(1)));
    assert!(!LayerSet::all(2).contains(LayerId(2)));
    assert_eq!(LayerSet::all(64).0, u64::MAX);
}

#[test]
fn edit_invariants_reject_locked_moves() {
    let mut b = tiny_board();
    b.footprints[0].mobility = Mobility::Locked;
    let fp = &b.footprints[0];
    let edit = BoardEdit {
        moves: vec![FootprintMove {
            footprint: FootprintId(0),
            from: fp.placement,
            to: Placement { pos: fp.placement.pos + Point::new(1, 0), rot: fp.placement.rot },
        }],
        ..BoardEdit::default()
    };
    assert!(edit.check_invariants(&b).is_err());
    b.footprints[0].mobility = Mobility::Movable { max_displacement: 10, allow_rotation: false };
    assert!(edit.check_invariants(&b).is_ok());
    b.footprints[0].mobility = Mobility::Movable { max_displacement: 0, allow_rotation: false };
    assert!(edit.check_invariants(&b).is_err());
}

#[test]
fn outline_admits_with_edge_clearance() {
    let b = tiny_board();
    let h = ferroute_geom::Hull::disk(Point::new(300_000, 5_000_000), 100_000);
    assert!(b.outline.admits(&h, 200_000));
    assert!(!b.outline.admits(&h, 200_001));
}

#[test]
fn arcs_are_applied_covered_and_measured() {
    let mut b = tiny_board();
    let arc = ArcTrack {
        start: Point::new(6_000_000, 5_000_000),
        mid: Point::new(6_707_107, 5_292_893),
        end: Point::new(7_000_000, 6_000_000),
        width: 250_000,
        layer: LayerId(0),
        net: NetId(1),
        locked: false,
        origin: Origin::Routed,
    };
    let edit = BoardEdit { new_arcs: vec![arc.clone()], ..BoardEdit::default() };
    edit.check_invariants(&b).expect("routed arcs are additive");
    b.apply(&edit);
    assert_eq!(b.arcs, vec![arc.clone()]);
    // Quarter circle of radius 1 mm.
    assert!((arc.length() - 1e6 * std::f64::consts::FRAC_PI_2).abs() < 10.0);
    let hulls = arc.hulls();
    assert!(hulls.len() >= 2);
    for p in [arc.start, arc.mid, arc.end] {
        assert!(hulls.iter().any(|h| h.clearance_violated(&ferroute_geom::Hull::disk(p, 0), 1)));
    }
    let bad = BoardEdit {
        new_arcs: vec![ArcTrack { origin: Origin::Existing, ..arc }],
        ..BoardEdit::default()
    };
    assert!(bad.check_invariants(&b).is_err());
}
