// SPDX-License-Identifier: AGPL-3.0-only
mod common;

use common::{fixture, fp_id, load, mm, net_name, pad_pos};
use ferroute_board::{LayerId, LayerSet, Mobility, PadShape, Side};
use ferroute_geom::Point;
use ferroute_kicad::sexpr;

#[test]
fn fixtures_are_canonical() {
    // Set FERROUTE_REGEN_FIXTURES=1 to rewrite the fixtures in canonical form.
    for name in ["demo_2layer.kicad_pcb", "legacy_v9_numeric.kicad_pcb"] {
        let path = fixture(name);
        let src = std::fs::read_to_string(&path).unwrap();
        let canon = sexpr::parse(&src).unwrap().serialize();
        if std::env::var_os("FERROUTE_REGEN_FIXTURES").is_some() {
            std::fs::write(&path, &canon).unwrap();
        } else {
            assert_eq!(src, canon, "{name} is not in canonical KiCad formatting");
        }
    }
}

#[test]
fn demo_counts_and_rules() {
    let kb = load("demo_2layer.kicad_pcb");
    let b = &kb.board;
    assert_eq!(kb.doc.version(), 20_260_206);
    assert!(kb.doc.uses_net_names());
    assert_eq!(
        b.copper_layers.iter().map(|l| l.name.as_str()).collect::<Vec<_>>(),
        ["F.Cu", "B.Cu"]
    );
    assert_eq!(b.footprints.len(), 9);
    assert_eq!(b.pads.len(), 8 + 8 + 4 + 4 + 2);
    assert!(b.tracks.is_empty() && b.vias.is_empty());
    let mut names: Vec<&str> = b.nets.iter().map(|n| n.name.as_str()).collect();
    assert_eq!(names[0], "");
    names.sort_unstable();
    assert_eq!(names, ["", "+3V3", "ADDR", "EN", "GND", "INT", "OUT", "SCL", "SDA"]);
    // no copper pours in the demo, no unexpected warnings
    assert!(kb.warnings.iter().all(|w| w.contains(".kicad_pro")), "{:?}", kb.warnings);

    // net classes from the .kicad_pro
    let r = &b.rules;
    assert_eq!(r.classes.len(), 2);
    assert_eq!(r.classes[0].name, "Default");
    assert_eq!(
        (
            r.classes[0].clearance,
            r.classes[0].track_width,
            r.classes[0].via_diameter,
            r.classes[0].via_drill
        ),
        (mm(0.2), mm(0.25), mm(0.6), mm(0.3))
    );
    assert_eq!(r.classes[1].name, "Power");
    assert_eq!(
        (
            r.classes[1].clearance,
            r.classes[1].track_width,
            r.classes[1].via_diameter,
            r.classes[1].via_drill
        ),
        (mm(0.3), mm(0.5), mm(0.8), mm(0.4))
    );
    assert_eq!(r.edge_clearance, mm(0.3));
    assert_eq!(r.min_track_width, mm(0.15));
    for n in &b.nets {
        let want = u16::from(n.name == "GND" || n.name == "+3V3");
        assert_eq!(n.class, want, "class of {:?}", n.name);
    }
    assert_eq!(
        b.clearance_between(b.net_by_name("GND").unwrap(), b.net_by_name("SDA").unwrap()),
        mm(0.3)
    );
}

#[test]
fn demo_footprints_and_pads() {
    let b = load("demo_2layer.kicad_pcb").board;
    // locked flags
    for (r, locked) in [("U1", true), ("J2", true), ("R1", false), ("R4", false), ("J1", false)] {
        let f = b.footprint(fp_id(&b, r));
        assert_eq!(f.locked, locked, "{r}");
        assert_eq!(f.mobility, if locked { Mobility::Locked } else { Mobility::Fixed }, "{r}");
        assert_eq!(f.original, f.placement);
    }
    // plain SMD footprint at the origin of its frame
    assert_eq!(pad_pos(&b, "R1", "1"), Point::new(mm(107.175), mm(108.0)));
    assert_eq!(pad_pos(&b, "R1", "2"), Point::new(mm(108.825), mm(108.0)));

    // rotated by 90 degrees: (x, y) -> (y, -x); pad angle is absolute (90) => local 0
    let r3 = b.footprint(fp_id(&b, "R3"));
    assert_eq!(r3.placement.rot.degrees().to_bits(), 90.0f64.to_bits());
    assert_eq!(pad_pos(&b, "R3", "1"), Point::new(mm(108.0), mm(126.825)));
    assert_eq!(pad_pos(&b, "R3", "2"), Point::new(mm(108.0), mm(125.175)));
    let p = b.pad(common::pad_id(&b, "R3", "1"));
    assert_eq!(p.local_rot.degrees().to_bits(), 0.0f64.to_bits());
    assert_eq!(p.local_pos, Point::new(mm(-0.825), 0));

    // back side, rotated 180
    let r4 = b.footprint(fp_id(&b, "R4"));
    assert_eq!(r4.side, Side::Back);
    assert_eq!(pad_pos(&b, "R4", "1"), Point::new(mm(140.825), mm(128.0)));
    assert_eq!(pad_pos(&b, "R4", "2"), Point::new(mm(139.175), mm(128.0)));
    assert_eq!(b.pad(common::pad_id(&b, "R4", "1")).layers, LayerSet::single(LayerId(1)));
    assert_eq!(net_name(&b, b.pad(common::pad_id(&b, "R4", "2")).net), "INT");
    assert_eq!(b.pad(common::pad_id(&b, "R1", "1")).layers, LayerSet::single(LayerId(0)));

    // SOIC pad: roundrect with radius = floor(0.25 * 0.6) and net by name
    let u1p1 = b.pad(common::pad_id(&b, "U1", "1"));
    assert_eq!(u1p1.shape, PadShape::RoundRect { w: mm(1.95), h: mm(0.6), radius: mm(0.15) });
    assert_eq!(net_name(&b, u1p1.net), "INT");
    assert_eq!(pad_pos(&b, "U1", "5"), Point::new(mm(127.475), mm(121.905)));

    // through-hole header
    let j1p1 = b.pad(common::pad_id(&b, "J1", "1"));
    assert_eq!(j1p1.shape, PadShape::Rect { w: mm(1.7), h: mm(1.7) });
    assert_eq!(j1p1.drill.map(|d| (d.diameter, d.plated)), Some((mm(1.0), true)));
    assert_eq!(j1p1.layers, b.all_layers());
    assert!(matches!(b.pad(common::pad_id(&b, "J1", "2")).shape, PadShape::Oval { .. }));
    assert!(matches!(b.pad(common::pad_id(&b, "J2", "2")).shape, PadShape::Circle { .. }));
    assert_eq!(pad_pos(&b, "J1", "4"), Point::new(mm(143.0), mm(111.62)));

    // courtyards: fp_rect on F.CrtYd / B.CrtYd and four fp_line on J2
    let c = b.footprint(fp_id(&b, "R1")).courtyard_local.bbox().unwrap();
    assert_eq!((c.min, c.max), (Point::new(mm(-1.48), mm(-0.73)), Point::new(mm(1.48), mm(0.73))));
    let c = b.footprint(fp_id(&b, "R4")).courtyard_local.bbox().unwrap();
    assert_eq!(c.max, Point::new(mm(1.48), mm(0.73)));
    let c = b.footprint(fp_id(&b, "J2")).courtyard().bbox().unwrap();
    assert_eq!(
        (c.min, c.max),
        (Point::new(mm(133.5), mm(130.5)), Point::new(mm(143.58), mm(137.5)))
    );
}

#[test]
fn demo_outline_and_keepout() {
    let b = load("demo_2layer.kicad_pcb").board;
    let bb = b.outline.bbox().unwrap();
    assert_eq!(
        (bb.min, bb.max),
        (Point::new(mm(100.0), mm(100.0)), Point::new(mm(150.0), mm(140.0)))
    );
    assert_eq!(b.outline.outer.pts.len(), 4);
    assert_eq!(b.outline.outer.area2(), 2 * i128::from(mm(50.0)) * i128::from(mm(40.0)));
    assert!(b.outline.holes.is_empty());
    assert_eq!(b.keepouts.len(), 1);
    let k = &b.keepouts[0];
    assert!(k.no_tracks && k.no_vias && !k.no_pads && !k.no_footprints);
    assert_eq!(k.layers, b.all_layers());
    let kb = k.area.bbox().unwrap();
    assert_eq!(
        (kb.min, kb.max),
        (Point::new(mm(128.0), mm(102.0)), Point::new(mm(136.0), mm(110.0)))
    );
    b.validate().unwrap();
}

#[test]
fn legacy_numeric_dialect() {
    let kb = load("legacy_v9_numeric.kicad_pcb");
    let b = &kb.board;
    assert_eq!(kb.doc.version(), 20_241_229);
    assert!(!kb.doc.uses_net_names());
    assert_eq!(b.copper_layers.len(), 2);
    assert_eq!(b.copper_layers[1].name, "B.Cu");
    let names: Vec<&str> = b.nets.iter().map(|n| n.name.as_str()).collect();
    assert_eq!(names, ["", "GND", "VCC", "SIG"]);
    assert_eq!(b.footprints.len(), 2);
    assert!(!b.footprint(fp_id(b, "R1")).locked);
    assert!(b.footprint(fp_id(b, "J1")).locked);
    assert_eq!(b.pads.len(), 4);
    // R1 at (110,105) rot 90, pad angle absolute 90
    assert_eq!(pad_pos(b, "R1", "1"), Point::new(mm(110.0), mm(105.825)));
    assert_eq!(net_name(b, b.pad(common::pad_id(b, "R1", "1")).net), "VCC");
    assert_eq!(net_name(b, b.pad(common::pad_id(b, "J1", "2")).net), "GND");

    // two plain segments + an arc flattened to chords
    assert!(b.tracks.len() > 3, "arc must be flattened: {}", b.tracks.len());
    let t0 = &b.tracks[0];
    assert_eq!(
        (t0.start, t0.end, t0.width),
        (Point::new(mm(110.0), mm(105.825)), Point::new(mm(112.0), mm(105.825)), mm(0.25))
    );
    assert_eq!(net_name(b, t0.net), "VCC");
    assert!(!t0.locked && b.tracks[1].locked);
    assert_eq!(b.tracks[1].width, mm(0.2));
    let arc: Vec<_> = b.tracks.iter().filter(|t| t.layer == LayerId(1)).collect();
    assert!(arc.len() > 2);
    assert_eq!(arc[0].start, Point::new(mm(112.0), mm(105.825)));
    assert_eq!(arc[arc.len() - 1].end, Point::new(mm(115.0), mm(108.825)));
    assert!(arc.windows(2).all(|w| w[0].end == w[1].start));
    assert!(arc.iter().all(|t| t.width > mm(0.25) && t.width <= mm(0.25) + 4_000));
    assert_eq!(b.vias.len(), 1);
    let v = &b.vias[0];
    assert_eq!(
        (v.pos, v.diameter, v.drill),
        (Point::new(mm(112.0), mm(105.825)), mm(0.6), mm(0.3))
    );
    assert_eq!(v.layers, LayerSet::span(LayerId(0), LayerId(1)));
    assert_eq!(net_name(b, v.net), "VCC");

    // copper pour is reported, not modelled
    assert!(
        kb.warnings.iter().any(|w| w.contains("copper zone") && w.contains("GND")),
        "{:?}",
        kb.warnings
    );
    assert!(b.keepouts.is_empty());

    // outline: 30x20 with one r=3 rounded corner => chained from 4 lines + 1 arc
    let bb = b.outline.bbox().unwrap();
    assert_eq!(
        (bb.min, bb.max),
        (Point::new(mm(100.0), mm(100.0)), Point::new(mm(130.0), mm(120.0)))
    );
    #[allow(clippy::cast_precision_loss)]
    let area = (b.outline.outer.area2() as f64) / 2.0 / 1e12;
    let expect = 600.0 - 9.0 * (1.0 - std::f64::consts::FRAC_PI_4);
    assert!((area - expect).abs() < 0.01, "area {area} vs {expect}");
    assert!(b.outline.outer.area2() > 0);
}
