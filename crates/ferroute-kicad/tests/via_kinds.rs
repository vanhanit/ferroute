// SPDX-License-Identifier: AGPL-3.0-only
//! Blind, buried and micro vias: written with KiCad 10's keywords and span, read back
//! identically; through vias stay keyword-free.
mod common;

use common::{load, mm};
use ferroute_board::{BoardEdit, LayerId, LayerSet, NetId, Origin, Via, ViaKind};
use ferroute_geom::Point;
use ferroute_kicad::{ReadOptions, read_board};

fn via(x: f64, kind: ViaKind, layers: LayerSet, net: NetId, d: f64, drill: f64) -> Via {
    Via {
        pos: Point::new(mm(x), mm(100.0)),
        diameter: mm(d),
        drill: mm(drill),
        layers,
        kind,
        net,
        locked: false,
        origin: Origin::Routed,
    }
}

#[test]
fn via_kinds_roundtrip_through_kicad_text() {
    let kb = load("bga_planes.kicad_pcb");
    let board = kb.board.clone();
    assert_eq!(board.layer_count(), 4, "fixture is a 4-layer board");
    let net = NetId(1);
    let span = |a, b| LayerSet::span(LayerId(a), LayerId(b));
    let vias = vec![
        via(10.0, ViaKind::Through, span(0, 3), net, 0.6, 0.3),
        via(11.0, ViaKind::Blind, span(0, 2), net, 0.6, 0.3),
        via(12.0, ViaKind::Micro, span(0, 1), net, 0.3, 0.1),
        via(13.0, ViaKind::Micro, span(2, 3), net, 0.3, 0.1),
        via(14.0, ViaKind::Buried, span(1, 2), net, 0.6, 0.3),
    ];
    let edit = BoardEdit { new_vias: vias.clone(), ..BoardEdit::default() };
    let mut doc = kb.doc;
    doc.apply_edit(&board, &edit, 7).unwrap();
    let text = doc.to_kicad_string();

    // Each `(via ...)` block flattened to one line (KiCad pretty-prints one token per line).
    let flat: Vec<String> = text
        .split("\n\t(via")
        .skip(1)
        .map(|b| format!("(via{}", b.split("\n\t)").next().unwrap_or("")))
        .map(|b| b.split_whitespace().collect::<Vec<_>>().join(" "))
        .collect();
    let new = &flat[flat.len() - 5..];
    assert!(new[0].starts_with("(via (at 10 100)"), "{}", new[0]);
    assert!(new[1].starts_with("(via blind (at 11 100)"), "{}", new[1]);
    assert!(new[2].starts_with("(via micro (at 12 100)"), "{}", new[2]);
    assert!(new[3].starts_with("(via micro (at 13 100)"), "{}", new[3]);
    assert!(new[4].starts_with("(via buried (at 14 100)"), "{}", new[4]);
    assert!(new[1].contains("(layers \"F.Cu\" \"In2.Cu\")"), "{}", new[1]);
    assert!(new[3].contains("(layers \"In2.Cu\" \"B.Cu\")"), "{}", new[3]);
    assert!(new[2].contains("(size 0.3) (drill 0.1)"), "{}", new[2]);

    let after = read_board(&text, &ReadOptions::default()).unwrap().board;
    let read: Vec<&Via> = after.vias.iter().filter(|v| v.pos.y == mm(100.0)).collect();
    assert_eq!(read.len(), vias.len());
    for (a, b) in read.iter().zip(&vias) {
        assert_eq!(
            (a.pos, a.diameter, a.drill, a.layers, a.kind),
            (b.pos, b.diameter, b.drill, b.layers, b.kind)
        );
    }
    // Writing what was read changes nothing (INV-IO-1).
    let again = read_board(&text, &ReadOptions::default()).unwrap();
    assert_eq!(again.doc.to_kicad_string(), text);
}

#[test]
fn existing_vias_keep_their_kind_and_bytes() {
    let kb = load("demo_2layer.kicad_pcb");
    let text = kb.doc.to_kicad_string();
    let after = read_board(&text, &ReadOptions::default()).unwrap();
    assert!(after.board.vias.iter().all(|v| v.kind == ViaKind::Through));
    assert_eq!(after.doc.to_kicad_string(), text);
}
