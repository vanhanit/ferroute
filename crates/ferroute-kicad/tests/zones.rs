// SPDX-License-Identifier: AGPL-3.0-only
//! Copper zones (pours): stored fills are parsed, keep-out zones stay keep-outs.
mod common;

use common::mm;
use ferroute_board::{LayerId, LayerSet, PadConnection as P};
use ferroute_kicad::{KicadBoard, ReadOptions, read_board};

const LAYERS: &str = r#"(layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))"#;
const RECT: &str = r#"(gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts") (width 0.1))"#;

fn read(extra: &str) -> KicadBoard {
    let src = format!(
        "(kicad_pcb (version 20211014) (generator pcbnew) (general (thickness 1.6)) (paper \"A4\") {LAYERS} (net 0 \"\") (net 1 \"N1\") {extra} {RECT})"
    );
    read_board(&src, &ReadOptions::default()).unwrap()
}

const ZONE_FILLED: &str = r#"(zone (net 1) (net_name "N1") (layers "F.Cu" "B.Cu") (priority 2)
    (connect_pads no (clearance 0.3)) (min_thickness 0.2)
    (fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5))
    (polygon (pts (xy 1 1) (xy 9 1) (xy 9 9) (xy 1 9)))
    (filled_polygon (layer "F.Cu") (pts (xy 1 1) (xy 9 1) (xy 9 5) (xy 5 5) (xy 5 9) (xy 1 9)))
    (filled_polygon (layer "B.Cu") (island) (pts (xy 2 2) (xy 3 2) (xy 3 3))))"#;

#[test]
fn zone_fills_are_parsed() {
    let kb = read(ZONE_FILLED);
    let b = &kb.board;
    assert!(kb.warnings.iter().all(|w| !w.contains("ignored")), "{:?}", kb.warnings);
    assert_eq!(b.zones.len(), 1);
    let z = &b.zones[0];
    assert_eq!(z.net, b.net_by_name("N1").unwrap());
    assert_eq!(z.layers, LayerSet::span(LayerId(0), LayerId(1)));
    assert_eq!((z.priority, z.clearance, z.min_thickness), (2, mm(0.3), mm(0.2)));
    assert_eq!(z.pad_connection, P::None);
    assert!(!z.is_unfilled());
    assert_eq!(z.fills.len(), 2);
    assert_eq!(z.fills[0].layer, LayerId(0));
    assert_eq!(z.fills[0].polys[0].pts.len(), 6);
    assert_eq!(z.fills[1].layer, LayerId(1));
    assert!(b.keepouts.is_empty());
}

#[test]
fn zone_pad_connection_modes_and_kicad10_net_names() {
    let mode = |m: &str| {
        let z = format!(
            r#"(zone (net 1) (net_name "N1") (layer "F.Cu") (connect_pads {m} (clearance 0.2)) (polygon (pts (xy 1 1) (xy 9 1) (xy 9 9))))"#
        );
        read(&z).board.zones[0].pad_connection
    };
    assert_eq!(mode("yes"), P::Solid);
    assert_eq!(mode("thru_hole_only"), P::ThruHoleOnly);
    assert_eq!(mode("no"), P::None);
    let z = read(
        r#"(zone (net 1) (layer "F.Cu") (connect_pads (clearance 0.2)) (polygon (pts (xy 1 1) (xy 9 1) (xy 9 9))))"#,
    );
    assert_eq!(z.board.zones[0].pad_connection, P::ThermalRelief);
    let src = format!(
        r#"(kicad_pcb (version 20251028) (generator pcbnew) (general (thickness 1.6)) (paper "A4") {LAYERS} (zone (net "GND") (layer "F.Cu") (polygon (pts (xy 1 1) (xy 9 1) (xy 9 9)))) {RECT})"#
    );
    let kb = read_board(&src, &ReadOptions::default()).unwrap();
    assert_eq!(kb.board.nets[kb.board.zones[0].net.idx()].name, "GND");
}

#[test]
fn keepout_zones_stay_keepouts() {
    let kb = read(
        r#"(zone (net 0) (net_name "") (layer "F.Cu") (keepout (tracks not_allowed) (vias allowed) (pads allowed) (copperpour allowed) (footprints allowed))
            (polygon (pts (xy 1 1) (xy 9 1) (xy 9 9) (xy 1 9))))"#,
    );
    assert_eq!(kb.board.keepouts.len(), 1);
    assert!(kb.board.zones.is_empty());
}

#[test]
fn zones_do_not_change_untouched_output() {
    let kb = read(ZONE_FILLED);
    let text = kb.doc.to_kicad_string();
    assert!(text.contains("filled_polygon"));
    let again = read_board(&text, &ReadOptions::default()).unwrap();
    assert_eq!(again.doc.to_kicad_string(), text);
}
