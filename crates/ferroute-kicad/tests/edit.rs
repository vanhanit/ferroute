// SPDX-License-Identifier: AGPL-3.0-only
mod common;

use common::{fp_id, load, mm, near};
use ferroute_board::{
    Board, BoardEdit, FootprintMove, LayerId, LayerSet, Mobility, NetId, Origin, Placement, Track,
    Via,
};
use ferroute_geom::{Angle, Point};
use ferroute_kicad::{ReadOptions, read_board};

fn movable(b: &mut Board, refs: &[&str]) {
    for r in refs {
        let id = fp_id(b, r);
        b.footprints[id.idx()].mobility =
            Mobility::Movable { max_displacement: mm(20.0), allow_rotation: true };
    }
}

fn demo_edit(b: &Board) -> BoardEdit {
    let mv = |r: &str, dx: f64, dy: f64, rot: f64| {
        let id = fp_id(b, r);
        let from = b.footprint(id).placement;
        FootprintMove {
            footprint: id,
            from,
            to: Placement {
                pos: Point::new(from.pos.x + mm(dx), from.pos.y + mm(dy)),
                rot: Angle::from_degrees(rot),
            },
        }
    };
    let sda = b.net_by_name("SDA").unwrap();
    let gnd = b.net_by_name("GND").unwrap();
    BoardEdit {
        new_arcs: Vec::new(),
        zone_fills: Vec::new(),
        moves: vec![
            mv("R1", 2.5, -1.25, 45.0), // arbitrary angle
            mv("R3", 0.0, 0.0, 180.0),  // 90 -> 180, same place
            mv("R4", -3.0, 1.0, 270.0), // back side
            mv("C1", 1.0, 0.0, 0.0),    // translate only
        ],
        new_tracks: vec![
            Track {
                start: Point::new(mm(110.0), mm(110.0)),
                end: Point::new(mm(120.5), mm(112.25)),
                width: mm(0.25),
                layer: LayerId(0),
                net: sda,
                locked: false,
                origin: Origin::Routed,
            },
            Track {
                start: Point::new(mm(120.5), mm(112.25)),
                end: Point::new(mm(120.5), mm(130.0)),
                width: mm(0.5),
                layer: LayerId(1),
                net: gnd,
                locked: false,
                origin: Origin::Routed,
            },
        ],
        new_vias: vec![Via {
            pos: Point::new(mm(120.5), mm(112.25)),
            diameter: mm(0.6),
            drill: mm(0.3),
            kind: ferroute_board::ViaKind::Through,
            layers: LayerSet::span(LayerId(0), LayerId(1)),
            net: sda,
            locked: false,
            origin: Origin::Routed,
        }],
    }
}

fn opts() -> ReadOptions {
    ReadOptions {
        project_json: Some(
            std::fs::read_to_string(common::fixture("demo_2layer.kicad_pro")).unwrap(),
        ),
        ..ReadOptions::default()
    }
}

#[test]
fn edit_roundtrip_matches_model() {
    let kb = load("demo_2layer.kicad_pcb");
    let mut board = kb.board.clone();
    movable(&mut board, &["R1", "R3", "R4", "C1"]);
    let edit = demo_edit(&board);
    edit.check_invariants(&board).unwrap();
    let mut doc = kb.doc;
    doc.apply_edit(&board, &edit, 42).unwrap();
    let text = doc.to_kicad_string();

    let after = read_board(&text, &opts()).unwrap();
    let mut expect = board.clone();
    expect.apply(&edit);

    // every pad: absolute position and hull agree within 1 nm
    assert_eq!(after.board.pads.len(), expect.pads.len());
    for id in expect.pad_ids() {
        let (a, b) = (expect.pad_position(id), after.board.pad_position(id));
        assert!(near(a, b, 1), "pad {id:?}: {a:?} vs {b:?}");
        let (ha, hb) = (expect.pad_hull(id), after.board.pad_hull(id));
        assert_eq!(ha.pts.len(), hb.pts.len());
        assert!(ha.pts.iter().zip(&hb.pts).all(|(p, q)| near(*p, *q, 2)), "hull of {id:?}");
        assert_eq!(ha.radius, hb.radius);
        assert_eq!(expect.pad(id).net, after.board.pad(id).net);
    }
    // footprints
    for (i, (a, b)) in expect.footprints.iter().zip(&after.board.footprints).enumerate() {
        assert_eq!(a.reference, b.reference);
        assert!(near(a.placement.pos, b.placement.pos, 0), "fp {i}");
        let d = (a.placement.rot.degrees() - b.placement.rot.degrees()).abs();
        assert!(d < 1e-9 || (360.0 - d) < 1e-9, "fp {i} rot");
        assert_eq!(a.locked, b.locked);
        assert_eq!(a.side, b.side);
    }
    // new copper
    assert_eq!(after.board.tracks.len(), 2);
    for (a, b) in expect.tracks.iter().zip(&after.board.tracks) {
        assert_eq!(
            (a.start, a.end, a.width, a.layer, a.net),
            (b.start, b.end, b.width, b.layer, b.net)
        );
    }
    assert_eq!(after.board.nets[after.board.tracks[0].net.idx()].name, "SDA");
    assert_eq!(after.board.nets[after.board.tracks[1].net.idx()].name, "GND");
    assert_eq!(after.board.tracks[1].layer, LayerId(1));
    assert_eq!(after.board.vias.len(), 1);
    let (va, vb) = (&expect.vias[0], &after.board.vias[0]);
    assert_eq!(
        (va.pos, va.diameter, va.drill, va.layers, va.net),
        (vb.pos, vb.diameter, vb.drill, vb.layers, vb.net)
    );
    assert!(text.contains("(net \"SDA\")"));
    assert!(!text.contains("(net 2)"));
}

#[test]
fn untouched_nodes_are_byte_identical() {
    let kb = load("demo_2layer.kicad_pcb");
    let mut board = kb.board.clone();
    movable(&mut board, &["R1", "R3", "R4", "C1"]);
    let edit = demo_edit(&board);
    let mut doc = kb.doc.clone();
    doc.apply_edit(&board, &edit, 7).unwrap();
    let moved: Vec<usize> = edit.moves.iter().map(|m| m.footprint.idx()).collect();
    for i in 0..board.footprints.len() {
        let (a, b) = (kb.doc.footprint_text(i).unwrap(), doc.footprint_text(i).unwrap());
        if moved.contains(&i) {
            assert_ne!(a, b, "footprint {i} should have changed");
        } else {
            assert_eq!(a, b, "footprint {i} must be untouched");
        }
    }
    // Everything outside footprints and the inserted copper is unchanged too.
    let strip = |d: &ferroute_kicad::KicadDocument| -> Vec<String> {
        d.root()
            .items()
            .unwrap()
            .iter()
            .filter(|n| !matches!(n.head(), Some("footprint" | "segment" | "via")))
            .map(|n| n.serialize())
            .collect()
    };
    assert_eq!(strip(&kb.doc), strip(&doc));
}

#[test]
fn rotation_rewrites_pad_and_text_angles_like_kicad() {
    let kb = load("demo_2layer.kicad_pcb");
    let mut board = kb.board.clone();
    movable(&mut board, &["R3"]);
    let id = fp_id(&board, "R3");
    let to_zero = BoardEdit {
        moves: vec![FootprintMove {
            footprint: id,
            from: board.footprint(id).placement,
            to: Placement { pos: board.footprint(id).placement.pos, rot: Angle::ZERO },
        }],
        ..BoardEdit::default()
    };
    let mut doc = kb.doc.clone();
    doc.apply_edit(&board, &to_zero, 1).unwrap();
    let t = doc.footprint_text(id.idx()).unwrap();
    // fp (at) loses its angle; pad angles vanish; text angles stay explicit (0)
    assert!(t.contains("(at 108 126)"), "{t}");
    assert!(t.contains("(at -0.825 0)") && t.contains("(at 0.825 0)"), "{t}");
    assert!(t.contains("(at 0 -1.43 0)"), "{t}");
    assert!(!t.contains(" 90)"), "{t}");

    // 0 -> 90 on a different footprint: pad angle appears, text gets 90
    let mut b2 = kb.board.clone();
    movable(&mut b2, &["R2"]);
    let r2 = fp_id(&b2, "R2");
    let e = BoardEdit {
        moves: vec![FootprintMove {
            footprint: r2,
            from: b2.footprint(r2).placement,
            to: Placement { pos: b2.footprint(r2).placement.pos, rot: Angle::from_degrees(90.0) },
        }],
        ..BoardEdit::default()
    };
    let mut d2 = kb.doc;
    d2.apply_edit(&b2, &e, 1).unwrap();
    let t = d2.footprint_text(r2.idx()).unwrap();
    assert!(
        t.contains("(at 108 114 90)")
            && t.contains("(at -0.825 0 90)")
            && t.contains("(at 0 -1.43 90)"),
        "{t}"
    );
}

#[test]
fn deterministic_and_seeded() {
    let kb = load("demo_2layer.kicad_pcb");
    let mut board = kb.board.clone();
    movable(&mut board, &["R1", "R3", "R4", "C1"]);
    let edit = demo_edit(&board);
    let run = |seed| {
        let mut d = kb.doc.clone();
        d.apply_edit(&board, &edit, seed).unwrap();
        d.to_kicad_string()
    };
    assert_eq!(run(5), run(5));
    assert_ne!(run(5), run(6));
    // No edit: bytes identical to a plain re-serialization.
    let mut d = kb.doc.clone();
    d.apply_edit(&board, &BoardEdit::default(), 1).unwrap();
    assert_eq!(d.to_kicad_string(), kb.doc.to_kicad_string());
    // Successive edits with one seed never reuse a uuid.
    let mut d = kb.doc.clone();
    d.apply_edit(&board, &edit, 5).unwrap();
    let only_copper = BoardEdit { moves: vec![], ..edit.clone() };
    let mut board2 = board.clone();
    board2.apply(&edit);
    d.apply_edit(&board2, &only_copper, 5).unwrap();
    let text = d.to_kicad_string();
    let mut uuids: Vec<&str> = text.lines().filter(|l| l.contains("(uuid ")).collect();
    let n = uuids.len();
    uuids.sort_unstable();
    uuids.dedup();
    assert_eq!(uuids.len(), n);
}

#[test]
fn rejects_invalid_edits_without_changing_the_document() {
    let kb = load("demo_2layer.kicad_pcb");
    let board = kb.board.clone();
    let u1 = fp_id(&board, "U1");
    let bad = BoardEdit {
        moves: vec![FootprintMove {
            footprint: u1,
            from: board.footprint(u1).placement,
            to: Placement { pos: Point::new(0, 0), rot: Angle::ZERO },
        }],
        ..BoardEdit::default()
    };
    let mut d = kb.doc.clone();
    assert!(d.apply_edit(&board, &bad, 1).is_err());
    assert_eq!(d.to_kicad_string(), kb.doc.to_kicad_string());

    // unknown net
    let bad_net = BoardEdit {
        new_tracks: vec![Track {
            start: Point::new(0, 0),
            end: Point::new(mm(1.0), 0),
            width: mm(0.2),
            layer: LayerId(0),
            net: NetId(999),
            locked: false,
            origin: Origin::Routed,
        }],
        ..BoardEdit::default()
    };
    assert!(d.apply_edit(&board, &bad_net, 1).is_err());
    assert_eq!(d.to_kicad_string(), kb.doc.to_kicad_string());
}

#[test]
fn legacy_numeric_dialect_edit() {
    let kb = load("legacy_v9_numeric.kicad_pcb");
    let mut board = kb.board.clone();
    movable(&mut board, &["R1"]);
    let vcc = board.net_by_name("VCC").unwrap();
    let r1 = fp_id(&board, "R1");
    let edit = BoardEdit {
        new_arcs: Vec::new(),
        zone_fills: Vec::new(),
        moves: vec![FootprintMove {
            footprint: r1,
            from: board.footprint(r1).placement,
            to: Placement { pos: Point::new(mm(112.0), mm(103.0)), rot: Angle::from_degrees(90.0) },
        }],
        new_tracks: vec![Track {
            start: Point::new(mm(112.0), mm(103.0)),
            end: Point::new(mm(118.0), mm(103.0)),
            width: mm(0.25),
            layer: LayerId(0),
            net: vcc,
            locked: false,
            origin: Origin::Routed,
        }],
        new_vias: vec![Via {
            pos: Point::new(mm(118.0), mm(103.0)),
            diameter: mm(0.6),
            drill: mm(0.3),
            kind: ferroute_board::ViaKind::Through,
            layers: LayerSet::span(LayerId(0), LayerId(1)),
            net: vcc,
            locked: false,
            origin: Origin::Routed,
        }],
    };
    let mut doc = kb.doc;
    doc.apply_edit(&board, &edit, 3).unwrap();
    let text = doc.to_kicad_string();
    assert!(text.contains("(net 2)") && !text.contains("(net \"VCC\")"));
    // new copper sits right after the existing copper, before the zone
    let order: Vec<&str> = doc.root().items().unwrap().iter().filter_map(|n| n.head()).collect();
    let last_cu = order.iter().rposition(|h| matches!(*h, "segment" | "via" | "arc")).unwrap();
    let zone = order.iter().position(|h| *h == "zone").unwrap();
    assert_eq!(last_cu + 1, zone);
    let after = read_board(&text, &ReadOptions::default()).unwrap();
    assert_eq!(after.board.vias.len(), 2);
    let new_via = &after.board.vias[1];
    assert_eq!(after.board.nets[new_via.net.idx()].name, "VCC");
    let n = after.board.tracks.len();
    assert_eq!(after.board.tracks[n - 1].end, Point::new(mm(118.0), mm(103.0)));
    let mut expect = board.clone();
    expect.apply(&edit);
    for id in expect.pad_ids() {
        assert!(near(expect.pad_position(id), after.board.pad_position(id), 1));
    }
}

fn arc_edit(net: NetId) -> BoardEdit {
    BoardEdit {
        new_arcs: vec![ferroute_board::ArcTrack {
            start: Point::new(mm(110.0), mm(110.0)),
            mid: Point::new(mm(111.0), mm(110.25)),
            end: Point::new(mm(112.0), mm(111.0)),
            width: mm(0.25),
            layer: LayerId(0),
            net,
            locked: false,
            origin: Origin::Routed,
        }],
        ..BoardEdit::default()
    }
}

/// The `(arc ...)` block of `text` and `text` without it.
fn split_arc(text: &str) -> (String, String) {
    let a = text.rfind("\t(arc\n").expect("arc node");
    let b = a + text[a..].find("\n\t)\n").unwrap() + 4;
    let flat = text[a..b]
        .split_whitespace()
        .collect::<Vec<_>>()
        .join(" ")
        .replace("( ", "(")
        .replace(" )", ")");
    (flat, format!("{}{}", &text[..a], &text[b..]))
}

fn arc_flat(text: &str) -> String {
    split_arc(text).0
}

fn arc_edit_roundtrip(name: &str, net_name: &str) -> (String, String) {
    let kb = load(name);
    let board = kb.board.clone();
    let net = board.net_by_name(net_name).unwrap();
    let edit = arc_edit(net);
    let before = kb.doc.to_kicad_string();
    let mut doc = kb.doc;
    doc.apply_edit(&board, &edit, 7).unwrap();
    let text = doc.to_kicad_string();
    // untouched text is byte-identical: removing the arc node restores the input
    let line = arc_flat(&text);
    let again = {
        let mut d2 = load(name).doc;
        d2.apply_edit(&board, &edit, 7).unwrap();
        d2.to_kicad_string()
    };
    assert_eq!(text, again, "deterministic");
    assert_ne!(text, before);
    let b = read_board(&text, &ReadOptions::default()).unwrap().board;
    assert!(!b.tracks.is_empty());
    let chords: Vec<_> =
        b.tracks.iter().filter(|t| t.net == net && t.layer == LayerId(0)).collect();
    let (s, e) = (Point::new(mm(110.0), mm(110.0)), Point::new(mm(112.0), mm(111.0)));
    assert!(chords.iter().any(|t| t.start == s || t.end == s));
    assert!(chords.iter().any(|t| t.start == e || t.end == e));
    (text, line.trim().to_string())
}

#[test]
fn arc_written_kicad10_names() {
    let (text, line) = arc_edit_roundtrip("demo_2layer.kicad_pcb", "SDA");
    assert!(line.starts_with("(arc (start 110 110) (mid 111 110.25) (end 112 111) (width 0.25)"));
    assert!(line.contains("(layer \"F.Cu\") (net \"SDA\") (uuid \""), "{line}");
    // removing the arc line gives the original byte-for-byte
    let orig = load("demo_2layer.kicad_pcb").doc.to_kicad_string();
    assert_eq!(split_arc(&text).1, orig);
}

#[test]
fn arc_written_kicad9_numeric() {
    let (text, line) = arc_edit_roundtrip("legacy_v9_numeric.kicad_pcb", "SIG");
    assert!(line.contains("(layer \"F.Cu\") (net 3) (uuid \""), "{line}");
    let orig = load("legacy_v9_numeric.kicad_pcb").doc.to_kicad_string();
    assert_eq!(split_arc(&text).1, orig);
}

#[test]
fn empty_edit_leaves_file_unchanged() {
    let kb = load("demo_2layer.kicad_pcb");
    let board = kb.board.clone();
    let orig = kb.doc.to_kicad_string();
    let mut doc = kb.doc;
    doc.apply_edit(&board, &BoardEdit::default(), 1).unwrap();
    assert_eq!(doc.to_kicad_string(), orig);
}

#[test]
fn new_board_writes_arcs() {
    let mut b = load("demo_2layer.kicad_pcb").board;
    let net = b.net_by_name("SDA").unwrap();
    b.arcs = arc_edit(net).new_arcs;
    let text = ferroute_kicad::write_new_board(&b, 3);
    assert!(arc_flat(&text).starts_with("(arc (start 110 110) (mid 111 110.25) (end 112 111)"));
}

/// A via at the centre of the first connected SMD pad that fits it, and one far away.
fn via_in_pad_edit(b: &Board) -> BoardEdit {
    let via = |pos: Point, net: NetId| Via {
        pos,
        diameter: 600_000,
        drill: 300_000,
        kind: ferroute_board::ViaKind::Through,
        layers: b.all_layers(),
        net,
        locked: false,
        origin: Origin::Routed,
    };
    let pad = b
        .pad_ids()
        .find(|&id| {
            let p = b.pad(id);
            p.net != NetId(0)
                && p.drill.is_none()
                && b.pad_hull(id).contains_disk(b.pad_position(id), 300_000)
        })
        .expect("SMD pad");
    let (pos, net) = (b.pad_position(pad), b.pad(pad).net);
    assert!(b.via_in_pad(&via(pos, net)).is_some());
    BoardEdit {
        new_vias: vec![via(pos, net), via(Point::new(mm(1.0), mm(1.0)), net)],
        ..BoardEdit::default()
    }
}

#[test]
fn via_in_pad_is_written_filled_and_capped_in_kicad10() {
    let kb = load("demo_2layer.kicad_pcb");
    let board = kb.board.clone();
    let edit = via_in_pad_edit(&board);
    let mut doc = kb.doc;
    doc.apply_edit(&board, &edit, 7).unwrap();
    let text = doc.to_kicad_string();
    // Exactly one new via is in a pad (the board-level `(setup …)` defaults are `no`).
    assert_eq!(text.matches("(capping yes)").count(), 1, "{text}");
    assert_eq!(text.matches("(filling yes)").count(), 1);
    let again = read_board(&text, &ReadOptions::default()).unwrap().board;
    assert_eq!(again.vias.len(), board.vias.len() + 2);
}

#[test]
fn via_in_pad_attributes_are_not_written_for_older_dialects() {
    let kb = load("legacy_v9_numeric.kicad_pcb");
    let board = kb.board.clone();
    let edit = via_in_pad_edit(&board);
    let mut doc = kb.doc;
    doc.apply_edit(&board, &edit, 7).unwrap();
    let text = doc.to_kicad_string();
    assert_eq!(text.matches("(capping yes)").count(), 0);
    assert_eq!(text.matches("(filling yes)").count(), 0);
}

#[test]
fn via_in_pad_follows_moved_footprints() {
    let kb = load("demo_2layer.kicad_pcb");
    let mut board = kb.board.clone();
    let mut edit = via_in_pad_edit(&board);
    edit.new_vias.truncate(1);
    // Move the part owning that pad; the via sits at the pad's *new* centre.
    let pad = board.pad_ids().find(|&p| board.pad_position(p) == edit.new_vias[0].pos).unwrap();
    let fp = board.pad(pad).footprint;
    board.footprints[fp.idx()].mobility =
        Mobility::Movable { max_displacement: mm(20.0), allow_rotation: true };
    let from = board.footprint(fp).placement;
    let to = Placement { pos: Point::new(from.pos.x + mm(1.0), from.pos.y), rot: from.rot };
    edit.moves = vec![FootprintMove { footprint: fp, from, to }];
    edit.new_vias[0].pos = Point::new(edit.new_vias[0].pos.x + mm(1.0), edit.new_vias[0].pos.y);
    let mut doc = kb.doc;
    doc.apply_edit(&board, &edit, 7).unwrap();
    assert_eq!(doc.to_kicad_string().matches("(capping yes)").count(), 1);
}
