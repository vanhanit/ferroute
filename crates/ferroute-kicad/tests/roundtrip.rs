// SPDX-License-Identifier: AGPL-3.0-only
mod common;

use common::{load, mm, near};
use ferroute_board::{Board, Drill, LayerId, Origin, Pad, PadId, PadShape, Placement, Track, Via};
use ferroute_geom::{Angle, Point, Polygon, convex_hull};
use ferroute_kicad::project::write_project_json;
use ferroute_kicad::{ReadOptions, read_board, write_new_board};
use proptest::prelude::*;

fn reread(board: &Board, seed: u64) -> Board {
    let text = write_new_board(board, seed);
    let opts =
        ReadOptions { project_json: Some(write_project_json(board)), ..ReadOptions::default() };
    read_board(&text, &opts).unwrap_or_else(|e| panic!("{e}\n{text}")).board
}

fn norm_poly(p: &Polygon) -> Vec<Point> {
    let mut v = p.pts.clone();
    if p.area2() < 0 {
        v.reverse();
    }
    let i = (0..v.len()).min_by_key(|&i| v[i]).unwrap();
    v.rotate_left(i);
    v
}

/// A board with every feature the writer supports.
fn rich_board() -> Board {
    let mut b = load("demo_2layer.kicad_pcb").board;
    let gnd = b.net_by_name("GND").unwrap();
    let sda = b.net_by_name("SDA").unwrap();
    b.tracks.push(Track {
        start: Point::new(mm(110.0), mm(110.0)),
        end: Point::new(mm(120.5), mm(112.25)),
        width: mm(0.25),
        layer: LayerId(0),
        net: sda,
        locked: false,
        origin: Origin::Existing,
    });
    b.tracks.push(Track {
        start: Point::new(mm(120.5), mm(112.25)),
        end: Point::new(mm(120.5), mm(130.0)),
        width: mm(0.5),
        layer: LayerId(1),
        net: gnd,
        locked: true,
        origin: Origin::Existing,
    });
    b.vias.push(Via {
        pos: Point::new(mm(120.5), mm(112.25)),
        diameter: mm(0.6),
        drill: mm(0.3),
        layers: ferroute_board::LayerSet::span(LayerId(0), LayerId(1)),
        net: sda,
        locked: false,
        origin: Origin::Existing,
    });
    b.outline.holes.push(Polygon::new(vec![
        Point::new(mm(110.0), mm(125.0)),
        Point::new(mm(112.0), mm(125.0)),
        Point::new(mm(112.0), mm(127.0)),
        Point::new(mm(110.0), mm(127.0)),
    ]));
    // A non-plated mounting hole attached to J2 (the last footprint keeps pad ids stable).
    let j1 = common::fp_id(&b, "J2");
    let pid = PadId(u32::try_from(b.pads.len()).unwrap());
    b.pads.push(Pad {
        footprint: j1,
        number: String::new(),
        net: ferroute_board::NetId::NONE,
        local_pos: Point::new(mm(3.0), mm(-1.0)),
        local_rot: Angle::ZERO,
        shape: PadShape::Circle { diameter: mm(2.2) },
        layers: ferroute_board::LayerSet::EMPTY,
        drill: Some(Drill { diameter: mm(2.2), plated: false }),
    });
    b.footprints[j1.idx()].pads.push(pid);
    b.keepouts.push(ferroute_board::Keepout {
        area: Polygon::new(vec![
            Point::new(mm(101.0), mm(101.0)),
            Point::new(mm(104.0), mm(101.0)),
            Point::new(mm(104.0), mm(104.0)),
        ]),
        layers: ferroute_board::LayerSet::single(LayerId(1)),
        no_tracks: true,
        no_vias: false,
        no_pads: true,
        no_footprints: true,
    });
    // Slightly different classes
    b.rules.classes.push(ferroute_board::NetClass {
        name: "Fine".into(),
        clearance: mm(0.127),
        track_width: mm(0.127),
        via_diameter: mm(0.45),
        via_drill: mm(0.2),
    });
    b.nets[sda.idx()].class = 2;
    b.rules.edge_clearance = mm(0.35);
    b.validate().unwrap();
    b
}

fn assert_same_essentials(a: &Board, b: &Board) {
    assert_eq!(a.copper_layers, b.copper_layers);
    let names = |x: &Board| x.nets.iter().map(|n| n.name.clone()).collect::<Vec<_>>();
    assert_eq!(names(a), names(b));
    assert_eq!(a.footprints.len(), b.footprints.len());
    for (fa, fb) in a.footprints.iter().zip(&b.footprints) {
        assert_eq!(
            (&fa.reference, &fa.lib_id, fa.side, fa.locked),
            (&fb.reference, &fb.lib_id, fb.side, fb.locked)
        );
        assert_eq!(fa.placement.pos, fb.placement.pos);
        assert!((fa.placement.rot.degrees() - fb.placement.rot.degrees()).abs() < 1e-9);
        assert_eq!(fa.pads.len(), fb.pads.len());
        assert_eq!(convex_hull(&fa.courtyard_local.pts), convex_hull(&fb.courtyard_local.pts));
    }
    assert_eq!(a.pads.len(), b.pads.len());
    for id in a.pad_ids() {
        let (pa, pb) = (a.pad(id), b.pad(id));
        assert_eq!(
            (&pa.number, pa.net, pa.local_pos, &pa.shape, pa.layers, pa.drill),
            (&pb.number, pb.net, pb.local_pos, &pb.shape, pb.layers, pb.drill),
            "pad {id:?}"
        );
        assert_eq!(pa.footprint, pb.footprint);
        let (ha, hb) = (a.pad_hull(id), b.pad_hull(id));
        assert!(ha.pts.iter().zip(&hb.pts).all(|(p, q)| near(*p, *q, 1)));
        assert_eq!(ha.radius, hb.radius);
    }
    assert_eq!(a.tracks, b.tracks);
    assert_eq!(a.vias, b.vias);
    assert_eq!(a.keepouts.len(), b.keepouts.len());
    for (ka, kb) in a.keepouts.iter().zip(&b.keepouts) {
        assert_eq!(
            (ka.layers, ka.no_tracks, ka.no_vias, ka.no_pads, ka.no_footprints),
            (kb.layers, kb.no_tracks, kb.no_vias, kb.no_pads, kb.no_footprints)
        );
        assert_eq!(norm_poly(&ka.area), norm_poly(&kb.area));
    }
    assert_eq!(norm_poly(&a.outline.outer), norm_poly(&b.outline.outer));
    assert_eq!(a.outline.holes.len(), b.outline.holes.len());
    for (ha, hb) in a.outline.holes.iter().zip(&b.outline.holes) {
        assert_eq!(norm_poly(ha), norm_poly(hb));
    }
    assert_eq!(a.rules, b.rules);
    for (na, nb) in a.nets.iter().zip(&b.nets) {
        assert_eq!(na.class, nb.class, "class of {}", na.name);
    }
}

#[test]
fn write_then_read_preserves_the_model() {
    let b = rich_board();
    let back = reread(&b, 1);
    assert_same_essentials(&b, &back);
    // Also stable when going around twice.
    let again = reread(&back, 2);
    assert_same_essentials(&back, &again);
}

#[test]
fn write_is_deterministic_and_kicad10_shaped() {
    let b = rich_board();
    let t1 = write_new_board(&b, 99);
    assert_eq!(t1, write_new_board(&b, 99));
    assert_ne!(t1, write_new_board(&b, 100));
    assert!(t1.starts_with("(kicad_pcb\n\t(version 20260206)\n\t(generator \"ferroute\")"));
    // KiCad 10: nets by name, no net table, KiCad 9+ layer ids
    assert!(t1.contains("(net \"GND\")"));
    assert!(!t1.contains("(net 0 \"\")") && !t1.contains("(net 1 \"GND\")"));
    assert!(t1.contains("(0 \"F.Cu\" signal)") && t1.contains("(2 \"B.Cu\" signal)"));
    assert!(t1.contains("(layer \"Edge.Cuts\")"));
    assert!(t1.contains("(keepout"));
    assert!(t1.ends_with(")\n"));
}

#[test]
fn four_layer_ids_and_names() {
    let mut b = rich_board();
    b.copper_layers = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]
        .iter()
        .map(|n| ferroute_board::CopperLayer { name: (*n).into(), routable: true })
        .collect();
    // keep every layer reference valid: pads on all copper layers must now cover 4
    for p in &mut b.pads {
        if p.layers == ferroute_board::LayerSet::all(2) {
            p.layers = ferroute_board::LayerSet::all(4);
        } else if p.layers == ferroute_board::LayerSet::single(LayerId(1)) {
            p.layers = ferroute_board::LayerSet::single(LayerId(3));
        }
    }
    for t in &mut b.tracks {
        if t.layer == LayerId(1) {
            t.layer = LayerId(3);
        }
    }
    for v in &mut b.vias {
        v.layers = ferroute_board::LayerSet::all(4);
    }
    for k in &mut b.keepouts {
        k.layers = ferroute_board::LayerSet::single(LayerId(3));
    }
    let text = write_new_board(&b, 1);
    // KiCad 9+: F.Cu=0, B.Cu=2, In1.Cu=4, In2.Cu=6, listed front to back
    let f = text.find("(0 \"F.Cu\"").unwrap();
    let i1 = text.find("(4 \"In1.Cu\"").unwrap();
    let i2 = text.find("(6 \"In2.Cu\"").unwrap();
    let bc = text.find("(2 \"B.Cu\"").unwrap();
    assert!(f < i1 && i1 < i2 && i2 < bc);
    let back = reread(&b, 1);
    assert_same_essentials(&b, &back);
    assert_eq!(back.copper_layers[3].name, "B.Cu");
}

fn arb_rot() -> impl Strategy<Value = f64> {
    prop_oneof![
        Just(0.0),
        Just(90.0),
        Just(180.0),
        Just(270.0),
        (0u32..4).prop_map(|k| f64::from(k) * 90.0),
        0.0f64..360.0,
        (0i64..36_000_000).prop_map(|k| k as f64 / 100_000.0),
    ]
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(48))]

    #[test]
    fn random_placements_survive_write_and_read(
        placements in proptest::collection::vec(
            ((-200_000_000i64..300_000_000), (-200_000_000i64..300_000_000), arb_rot()), 9)
    ) {
        let mut b = load("demo_2layer.kicad_pcb").board;
        for (i, (x, y, r)) in placements.iter().enumerate() {
            b.footprints[i].placement = Placement { pos: Point::new(*x, *y), rot: Angle::from_degrees(*r) };
        }
        let back = reread(&b, 3);
        for id in b.pad_ids() {
            let (p, q) = (b.pad_position(id), back.pad_position(id));
            prop_assert!(near(p, q, 1), "pad {:?}: {:?} vs {:?}", id, p, q);
            let (ha, hb) = (b.pad_hull(id), back.pad_hull(id));
            prop_assert!(ha.pts.iter().zip(&hb.pts).all(|(a, c)| near(*a, *c, 2)));
        }
    }
}

#[test]
fn copper_zones_with_fills_round_trip() {
    use ferroute_board::{IslandRemoval, LayerSet, PadConnection, Zone, ZoneFill};
    let mut b = load("demo_2layer.kicad_pcb").board;
    b.zones.clear();
    let net = b.net_by_name("GND").expect("GND");
    let outline = Polygon::new(vec![
        Point::new(mm(101.0), mm(101.0)),
        Point::new(mm(129.0), mm(101.0)),
        Point::new(mm(129.0), mm(119.0)),
        Point::new(mm(101.0), mm(119.0)),
    ]);
    b.zones.push(Zone {
        net,
        layers: LayerSet::single(LayerId(1)),
        outline: outline.clone(),
        fills: vec![ZoneFill { layer: LayerId(1), polys: vec![outline.clone()] }],
        clearance: 200_000,
        min_thickness: 250_000,
        priority: 0,
        pad_connection: PadConnection::ThermalRelief,
        islands: IslandRemoval::Always,
        yields: false,
    });
    let back = reread(&b, 5);
    assert_eq!(back.zones.len(), 1);
    let z = &back.zones[0];
    assert_eq!((z.net, z.layers, z.clearance), (net, LayerSet::single(LayerId(1)), 200_000));
    assert_eq!(z.fills.len(), 1);
    assert_eq!(norm_poly(&z.fills[0].polys[0]), norm_poly(&outline));
}
