// SPDX-License-Identifier: AGPL-3.0-only
use super::*;
use ferroute_board::{
    CopperLayer, Drill, Footprint, FootprintId, Keepout, LayerId, Mobility, Net, Outline, Pad,
    PadId, PadShape, Placement, Side, Track, Via,
};
use ferroute_geom::{Angle, Point, Polygon, Rect};

/// Two-layer, 20 x 10 mm board with one footprint (two SMD pads and one THT pad), one track
/// per layer, a via, a keep-out and a hole in the outline.
fn test_board() -> Board {
    let mut b = Board {
        copper_layers: vec![
            CopperLayer { name: "F.Cu".into(), routable: true },
            CopperLayer { name: "B.Cu".into(), routable: true },
        ],
        nets: vec![Net::new(String::new(), 0), Net::new("SIG&<>", 0)],
        outline: Outline {
            outer: Polygon::from_rect(Rect::new(
                Point::new(0, 0),
                Point::new(20_000_000, 10_000_000),
            )),
            holes: vec![Polygon::from_rect(Rect::new(
                Point::new(2_000_000, 2_000_000),
                Point::new(3_000_000, 3_000_000),
            ))],
        },
        ..Board::default()
    };
    let pl = Placement { pos: Point::new(10_000_000, 5_000_000), rot: Angle::ZERO };
    b.footprints.push(Footprint {
        reference: "R1".into(),
        lib_id: "R_0603".into(),
        placement: pl,
        original: pl,
        side: Side::Front,
        locked: false,
        mobility: Mobility::Movable { max_displacement: 1_000_000, allow_rotation: false },
        courtyard_local: Polygon::from_rect(Rect::new(
            Point::new(-1_500_000, -2_500_000),
            Point::new(1_500_000, 2_500_000),
        )),
        pads: vec![PadId(0), PadId(1), PadId(2)],
    });
    let smd = |x: i64, net: NetId| Pad {
        footprint: FootprintId(0),
        number: String::new(),
        net,
        local_pos: Point::new(x, 0),
        local_rot: Angle::ZERO,
        shape: PadShape::RoundRect { w: 800_000, h: 950_000, radius: 200_000 },
        layers: LayerSet::single(LayerId(0)),
        drill: None,
    };
    b.pads.push(Pad { number: "1".into(), ..smd(-800_000, NetId(1)) });
    b.pads.push(Pad { number: "2".into(), ..smd(800_000, NetId::NONE) });
    b.pads.push(Pad {
        footprint: FootprintId(0),
        number: "3".into(),
        net: NetId(1),
        local_pos: Point::new(0, 2_000_000),
        local_rot: Angle::ZERO,
        shape: PadShape::Circle { diameter: 1_600_000 },
        layers: LayerSet::all(2),
        drill: Some(Drill { diameter: 800_000, plated: true }),
    });
    b.tracks.push(Track {
        start: Point::new(5_000_000, 5_000_000),
        end: Point::new(15_000_000, 5_000_000),
        width: 250_000,
        layer: LayerId(0),
        net: NetId(1),
        locked: false,
        origin: Origin::Routed,
    });
    b.tracks.push(Track {
        start: Point::new(5_000_000, 7_000_000),
        end: Point::new(15_000_000, 7_000_000),
        width: 250_000,
        layer: LayerId(1),
        net: NetId(1),
        locked: false,
        origin: Origin::Existing,
    });
    b.vias.push(Via {
        pos: Point::new(12_000_000, 8_000_000),
        diameter: 600_000,
        drill: 300_000,
        kind: ferroute_board::ViaKind::Through,
        layers: LayerSet::all(2),
        net: NetId(1),
        locked: false,
        origin: Origin::Routed,
    });
    b.keepouts.push(Keepout {
        area: Polygon::from_rect(Rect::new(
            Point::new(16_000_000, 1_000_000),
            Point::new(19_000_000, 3_000_000),
        )),
        layers: LayerSet::all(2),
        no_tracks: true,
        no_vias: true,
        no_pads: false,
        no_footprints: false,
    });
    b
}

fn parse(svg: &str) -> roxmltree::Document<'_> {
    roxmltree::Document::parse(svg).expect("renderer output must be well-formed XML")
}

fn has_group(doc: &roxmltree::Document<'_>, id: &str) -> bool {
    doc.descendants().any(|n| n.tag_name().name() == "g" && n.attribute("id") == Some(id))
}

#[test]
fn test_board_is_valid() {
    test_board().validate().expect("test board must be structurally valid");
}

#[test]
fn mm_formatting_is_exact() {
    let cases: [(Coord, &str); 11] = [
        (0, "0.0000"),
        (1, "0.0000"),
        (49, "0.0000"),
        (50, "0.0001"),
        (-50, "-0.0001"),
        (-149, "-0.0001"),
        (-1, "0.0000"),
        (1_234_567, "1.2346"),
        (-1_000_000, "-1.0000"),
        (20_000_000, "20.0000"),
        (150, "0.0002"),
    ];
    for (nm, want) in cases {
        assert_eq!(mm(nm), want, "mm({nm})");
    }
    // Must not panic at the extreme.
    assert!(mm(Coord::MIN).starts_with('-'));
}

#[test]
fn escapes_xml_specials() {
    assert_eq!(esc(r#"a&b<c>"d'"#), "a&amp;b&lt;c&gt;&quot;d&apos;");
}

#[test]
fn svg_is_well_formed_with_layer_groups_and_viewbox() {
    let svg = render_svg(&test_board(), &RenderOptions::default());
    let doc = parse(&svg);
    assert!(has_group(&doc, "layer-F.Cu"));
    assert!(has_group(&doc, "layer-B.Cu"));
    assert!(has_group(&doc, "vias"));
    assert!(has_group(&doc, "courtyards"));
    // 20 x 10 mm outline plus a 2 mm margin on each side, at 20 px/mm.
    let root = doc.root_element();
    assert_eq!(root.attribute("viewBox"), Some("-2.0000 -2.0000 24.0000 14.0000"));
    assert_eq!(root.attribute("width"), Some("480"));
    assert_eq!(root.attribute("height"), Some("280"));
}

#[test]
fn front_copper_is_drawn_last() {
    let svg = render_svg(&test_board(), &RenderOptions::default());
    let b = svg.find(r#"id="layer-B.Cu""#).expect("B.Cu group");
    let f = svg.find(r#"id="layer-F.Cu""#).expect("F.Cu group");
    assert!(b < f, "bottom layer must precede top layer in document order");
}

#[test]
fn net_names_are_escaped_and_present() {
    let svg = render_svg(&test_board(), &RenderOptions::default());
    let doc = parse(&svg);
    let n = doc.descendants().filter(|n| n.attribute("data-net") == Some("SIG&<>")).count();
    // Two tracks, one via, two SMD pads and one THT pad carry the net.
    assert!(n >= 3, "expected copper elements tagged with the net, found {n}");
    assert!(svg.contains("data-net=\"SIG&amp;&lt;&gt;\""));
}

#[test]
fn marker_title_carries_label() {
    let label = "clearance < 0.2 & 'SIG'";
    let opts = RenderOptions {
        markers: vec![Marker { at: Point::new(5_000_000, 5_000_000), label: label.into() }],
        ..RenderOptions::default()
    };
    let svg = render_svg(&test_board(), &opts);
    let doc = parse(&svg);
    assert!(
        doc.descendants().any(|n| n.tag_name().name() == "title" && n.text() == Some(label)),
        "marker <title> must hold the label"
    );
}

#[test]
fn rendering_is_deterministic() {
    let opts = RenderOptions {
        ratsnest: vec![(Point::new(0, 0), Point::new(10_000_000, 1_000_000))],
        moves: vec![(Point::new(1_000_000, 1_000_000), Point::new(2_000_000, 1_000_000))],
        markers: vec![Marker { at: Point::new(5_000_000, 5_000_000), label: "x".into() }],
        title: Some("Board & title".into()),
        highlight_routed: true,
        ..RenderOptions::default()
    };
    let a = render_svg(&test_board(), &opts);
    let b = render_svg(&test_board(), &opts);
    assert_eq!(a, b);
    assert!(a.contains("Board &amp; title"));
    assert!(a.contains(r#"opacity="1""#), "routed copper should be highlighted");
}

#[test]
fn layer_selection_filters_groups() {
    let opts =
        RenderOptions { layers: Some(LayerSet::single(LayerId(1))), ..RenderOptions::default() };
    let svg = render_svg(&test_board(), &opts);
    let doc = parse(&svg);
    assert!(has_group(&doc, "layer-B.Cu"));
    assert!(!has_group(&doc, "layer-F.Cu"));
}

#[test]
fn courtyards_can_be_hidden_and_movable_ones_are_dashed() {
    let shown = render_svg(&test_board(), &RenderOptions::default());
    assert!(shown.contains(r#"stroke-dasharray="0.2000 0.1000""#));
    let hidden = render_svg(
        &test_board(),
        &RenderOptions { show_courtyards: false, ..RenderOptions::default() },
    );
    assert!(!hidden.contains(r#"id="courtyards""#));
}

#[test]
fn empty_board_renders_without_panicking() {
    let svg = render_svg(&Board::default(), &RenderOptions::default());
    parse(&svg);
}

#[test]
fn write_svg_writes_the_same_bytes() {
    let path =
        std::env::temp_dir().join(format!("ferroute-render-test-{}.svg", std::process::id()));
    let opts = RenderOptions::default();
    write_svg(&path, &test_board(), &opts).expect("write");
    let written = std::fs::read_to_string(&path).expect("read back");
    std::fs::remove_file(&path).ok();
    assert_eq!(written, render_svg(&test_board(), &opts));
}

fn quarter_arc(ccw_on_screen: bool, layer: LayerId) -> ferroute_board::ArcTrack {
    // Quarter circle of radius 5 mm centred at (10, 5) mm, from (15,5) to (10,10) or (10,0).
    let h = 5_000_000.0 * std::f64::consts::FRAC_1_SQRT_2;
    let (sy, ey) = if ccw_on_screen { (-1, -1) } else { (1, 1) };
    ferroute_board::ArcTrack {
        start: Point::new(15_000_000, 5_000_000),
        mid: Point::new(10_000_000 + h as i64, 5_000_000 + sy * h as i64),
        end: Point::new(10_000_000, 5_000_000 + ey * 5_000_000),
        width: 250_000,
        layer,
        net: NetId(1),
        locked: false,
        origin: Origin::Routed,
    }
}

fn arc_paths(svg: &str) -> Vec<String> {
    svg.lines().filter(|l| l.contains(" A")).map(str::to_owned).collect()
}

#[test]
fn arc_emits_one_arc_command_with_flags() {
    // Down on screen (y grows): sweep positive -> SVG sweep flag 1.
    let mut b = test_board();
    b.arcs.push(quarter_arc(false, LayerId(1)));
    let svg = render_svg(&b, &RenderOptions::default());
    let l = arc_paths(&svg);
    assert_eq!(l.len(), 1, "{svg}");
    assert!(
        l[0].contains(r#"d="M15.0000 5.0000 A5.0000 5.0000 0 0 1 10.0000 10.0000""#),
        "{}",
        l[0]
    );
    assert!(l[0].contains(r#"fill="none""#) && l[0].contains(r#"stroke-width="0.2500""#));
    assert!(l[0].contains(r#"data-net="SIG&amp;&lt;&gt;""#));
    let doc = parse(&svg);
    let g = doc
        .descendants()
        .find(|n| n.tag_name().name() == "g" && n.attribute("id") == Some("layer-B.Cu"))
        .unwrap();
    assert!(g.children().any(|c| c.attribute("d").is_some_and(|d| d.contains(" A5.0000"))));

    // Opposite orientation: sweep flag 0.
    let mut b = test_board();
    b.arcs.push(quarter_arc(true, LayerId(0)));
    let l = arc_paths(&render_svg(&b, &RenderOptions::default()));
    assert_eq!(l.len(), 1);
    assert!(
        l[0].contains(r#"d="M15.0000 5.0000 A5.0000 5.0000 0 0 0 10.0000 0.0000""#),
        "{}",
        l[0]
    );
}

#[test]
fn arc_large_flag_and_degenerate_fallback() {
    let mut b = test_board();
    let mut a = quarter_arc(false, LayerId(0));
    // Three-quarter arc: start (15,5), mid (5,5), end (10,0)... clockwise on screen via bottom.
    a.mid = Point::new(5_000_000, 5_000_000);
    a.end = Point::new(10_000_000, 0);
    b.arcs.push(a);
    let l = arc_paths(&render_svg(&b, &RenderOptions::default()));
    assert!(l[0].contains("A5.0000 5.0000 0 1 1 10.0000 0.0000"), "{}", l[0]);

    let mut b = test_board();
    let mut a = quarter_arc(false, LayerId(0));
    a.mid = Point::new(10_000_000, 5_000_000);
    a.end = Point::new(5_000_000, 5_000_000);
    b.arcs.push(a);
    let svg = render_svg(&b, &RenderOptions::default());
    assert!(arc_paths(&svg).is_empty());
    assert!(svg.contains("M15.0000 5.0000 L"));
}

#[test]
fn boards_without_arcs_have_no_arc_paths_and_arcs_extend_bbox() {
    let svg = render_svg(&test_board(), &RenderOptions::default());
    assert!(arc_paths(&svg).is_empty());
    let mut b = Board {
        copper_layers: test_board().copper_layers,
        nets: test_board().nets,
        ..Board::default()
    };
    b.arcs.push(quarter_arc(false, LayerId(0)));
    let svg = render_svg(&b, &RenderOptions::default());
    assert_eq!(arc_paths(&svg).len(), 1);
    assert!(svg.contains("viewBox=\"7.8735 2.8735 "), "{svg}");
}
