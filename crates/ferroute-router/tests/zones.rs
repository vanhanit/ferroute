// SPDX-License-Identifier: AGPL-3.0-only
//! Copper pours are obstacles for other nets and connect pads of their own net.

use ferroute_board::builder::{BoardBuilder, mm, pt10};
use ferroute_board::{Board, LayerId, LayerSet, PadConnection, Zone, ZoneFill};
use ferroute_drc::{DrcConfig, ItemRef, check};
use ferroute_geom::Polygon;
use ferroute_router::{Accel, RouterConfig, pad_islands, route};

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Polygon {
    Polygon::new(vec![pt10(x0, y0), pt10(x1, y0), pt10(x1, y1), pt10(x0, y1)])
}

/// `S` must go from R1 (left) to R2 (right) past a GND pour (x 12..18, y 6..19.5 mm on
/// both layers) that already connects the two GND pads inside it.
fn board(layers: &[u8]) -> Board {
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let (s, q, q2, gnd) = (b.net("S"), b.net("Q"), b.net("Q2"), b.net("GND"));
    b.smd2("R1", pt10(50, 100), 0.0, q, s);
    b.smd2("R2", pt10(250, 100), 0.0, s, q2);
    b.smd2("G1", pt10(135, 100), 0.0, gnd, gnd);
    b.smd2("G2", pt10(165, 150), 0.0, gnd, gnd);
    let mut board = b.build();
    let fill = |l: u8| ZoneFill { layer: LayerId(l), polys: vec![rect(120, 60, 180, 195)] };
    board.zones.push(Zone {
        net: gnd,
        layers: LayerSet::all(2),
        outline: rect(120, 60, 180, 195),
        fills: layers.iter().map(|&l| fill(l)).collect(),
        clearance: 300_000,
        min_thickness: 250_000,
        priority: 0,
        pad_connection: PadConnection::ThermalRelief,
        islands: Default::default(),
        yields: false,
    });
    board
}

fn cfg() -> RouterConfig {
    RouterConfig { accel: Accel::None, ..RouterConfig::default() }
}

#[test]
fn pad_islands_include_pads_joined_by_a_fill() {
    let b = board(&[0]);
    let gnd = b.net_by_name("GND").unwrap();
    assert_eq!(pad_islands(&b)[gnd.idx()].len(), 1);
    let mut none = b;
    none.zones[0].pad_connection = PadConnection::None;
    assert_eq!(pad_islands(&none)[gnd.idx()].len(), 4);
}

#[test]
fn signal_routes_around_the_pour_and_gnd_needs_nothing() {
    let b = board(&[0, 1]);
    let out = route(&b, &cfg()).expect("route");
    assert!(out.failed.is_empty(), "{:?} {:?}", out.failed, out.stats);
    assert_eq!(out.stats.drc_gate_rips, 0);
    let mut after = b.clone();
    after.apply(&out.edit);
    let gnd = b.net_by_name("GND").unwrap();
    assert!(after.tracks.iter().all(|t| t.net != gnd) && after.vias.iter().all(|v| v.net != gnd));
    assert!(!after.tracks.is_empty());
    let rep = check(&after, &DrcConfig::default());
    assert!(rep.introduced().next().is_none(), "{:#?}", rep.violations);
    assert!(rep.violations.iter().all(|v| v.b != Some(ItemRef::Zone(0))), "{:#?}", rep.violations);
    assert_eq!(rep.connectivity.connections_missing, 0, "{:?}", rep.connectivity);
}

#[test]
fn a_fill_on_one_layer_only_blocks_that_layer() {
    let b = board(&[0]);
    let out = route(&b, &cfg()).expect("route");
    assert!(out.failed.is_empty());
    let mut after = b;
    after.apply(&out.edit);
    let rep = check(&after, &DrcConfig::default());
    assert!(rep.introduced().next().is_none(), "{:#?}", rep.violations);
    assert_eq!(rep.connectivity.connections_missing, 0);
}

/// Two GND SMD pads on F.Cu, a GND pour on B.Cu only: each pad drops a short stub and a
/// via that lands wholly inside the pour (no via in a pad), without a trace between pads.
#[test]
fn plane_nets_reach_their_pour_with_dog_bone_vias() {
    let mut b = BoardBuilder::new(mm(30), mm(20), 2);
    let (q, gnd) = (b.net("Q"), b.net("GND"));
    b.smd2("G1", pt10(60, 100), 0.0, gnd, q);
    b.smd2("G2", pt10(240, 100), 0.0, gnd, q);
    let mut board = b.build();
    let poly = rect(30, 30, 270, 170);
    board.zones.push(Zone {
        net: gnd,
        layers: LayerSet::all(2),
        outline: poly.clone(),
        fills: vec![ZoneFill { layer: LayerId(1), polys: vec![poly.clone()] }],
        clearance: 300_000,
        min_thickness: 250_000,
        priority: 0,
        pad_connection: PadConnection::ThermalRelief,
        islands: Default::default(),
        yields: false,
    });
    let out = route(&board, &cfg()).expect("route");
    assert!(out.failed.is_empty(), "{:?} {:?}", out.failed, out.stats);
    let mut after = board.clone();
    after.apply(&out.edit);
    let gvias: Vec<_> = after.vias.iter().filter(|v| v.net == gnd).collect();
    assert_eq!(gvias.len(), 2, "{gvias:?}");
    for v in &gvias {
        assert!(poly.contains_disk(v.pos, v.diameter / 2), "via copper inside the pour");
        assert!(after.via_in_pad(v).is_none(), "no via in a pad");
    }
    let rep = check(&after, &DrcConfig::default());
    assert!(rep.introduced().next().is_none(), "{:#?}", rep.violations);
    assert_eq!(rep.connectivity.connections_missing, 0, "{:?}", rep.connectivity);
}
