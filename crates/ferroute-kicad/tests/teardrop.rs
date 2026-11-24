// SPDX-License-Identifier: AGPL-3.0-only
//! Opt-in native KiCad teardrops: only the pads/vias new copper connects to change.
mod common;

use common::{load, mm, pad_id, pad_pos};
use ferroute_board::{BoardEdit, LayerId, LayerSet, Origin, Track, Via};
use ferroute_geom::Point;
use ferroute_kicad::read_board;

const TD: &str = "(teardrops (best_length_ratio 0.5) (max_length 1) (best_width_ratio 1) \
(max_width 2) (curved_edges no) (filter_ratio 0.9) (enabled yes) (allow_two_segments yes) \
(prefer_zone_connections yes))";

/// One new track from the centre of R1 pad 1 to a new via beside it.
fn fixture_edit() -> (ferroute_board::Board, BoardEdit, ferroute_kicad::KicadDocument) {
    let kb = load("demo_2layer.kicad_pcb");
    let board = kb.board.clone();
    let p = pad_pos(&board, "R1", "1");
    let net = board.pad(pad_id(&board, "R1", "1")).net;
    let far = Point::new(p.x - mm(2.0), p.y);
    let edit = BoardEdit {
        new_tracks: vec![Track {
            start: p,
            end: far,
            width: mm(0.25),
            layer: LayerId(0),
            net,
            locked: false,
            origin: Origin::Routed,
        }],
        new_vias: vec![Via {
            pos: far,
            diameter: mm(0.6),
            drill: mm(0.3),
            layers: LayerSet::span(LayerId(0), LayerId(1)),
            net,
            locked: false,
            origin: Origin::Routed,
        }],
        ..BoardEdit::default()
    };
    (board, edit, kb.doc)
}

fn without_teardrops(text: &str) -> String {
    // Each node sits on its own line in KiCad-style output; drop the lines that hold it.
    let mut out = String::new();
    let mut depth = 0i32;
    for line in text.lines() {
        if depth == 0 && line.trim_start().starts_with("(teardrops") {
            depth += line.matches('(').count() as i32 - line.matches(')').count() as i32;
            continue;
        }
        if depth > 0 {
            depth += line.matches('(').count() as i32 - line.matches(')').count() as i32;
            continue;
        }
        out.push_str(line);
        out.push('\n');
    }
    out
}

#[test]
fn default_off_writes_no_teardrop_property() {
    let (board, edit, mut doc) = fixture_edit();
    doc.apply_edit(&board, &edit, 7).unwrap();
    assert!(!doc.to_kicad_string().contains("(teardrops"));
}

#[test]
fn enables_exactly_the_connected_pad_and_via() {
    let (board, edit, mut doc) = fixture_edit();
    doc.apply_edit(&board, &edit, 7).unwrap();
    let plain = doc.to_kicad_string();
    assert_eq!(doc.enable_teardrops(&board, &edit), 2);
    let text = doc.to_kicad_string();
    assert_eq!(text.matches("(teardrops").count(), 2, "one pad and one via");
    let squashed: String = text.split_whitespace().collect();
    let want: String = TD.split_whitespace().collect();
    assert_eq!(squashed.matches(&want).count(), 2, "KiCad defaults with enabled yes");
    // Every other byte is untouched.
    assert_eq!(without_teardrops(&text), without_teardrops(&plain));
    // In the pad it sits before the uuid, in the via before the net.
    let squashed_pad = text.find("(teardrops").map(|i| &text[i..]).unwrap();
    assert!(squashed_pad.find("(uuid").unwrap() > 0);
    let via_at = text.rfind("\n\t(via\n").unwrap();
    let via = &text[via_at..];
    assert!(via.find("(teardrops").unwrap() < via.find("(net").unwrap());
    // The file still parses and keeps its geometry.
    let again = read_board(&text, &ferroute_kicad::ReadOptions::default()).unwrap();
    assert_eq!(again.board.vias.len(), 1);
    assert_eq!(again.board.tracks.len(), 1);
}

#[test]
fn enabling_is_idempotent() {
    let (board, edit, mut doc) = fixture_edit();
    doc.apply_edit(&board, &edit, 7).unwrap();
    doc.enable_teardrops(&board, &edit);
    let once = doc.to_kicad_string();
    assert_eq!(doc.enable_teardrops(&board, &edit), 0);
    assert_eq!(doc.to_kicad_string(), once);
}

#[test]
fn untouched_pads_and_other_nets_stay_unchanged() {
    let (board, mut edit, mut doc) = fixture_edit();
    // A track that merely ends in free space connects to nothing.
    edit.new_vias.clear();
    edit.new_tracks[0].end =
        Point::new(edit.new_tracks[0].start.x - mm(3.0), edit.new_tracks[0].start.y);
    edit.new_tracks[0].net = board
        .nets
        .iter()
        .position(|n| n.name == "ADDR")
        .map_or(edit.new_tracks[0].net, |i| ferroute_board::NetId(u32::try_from(i).unwrap()));
    doc.apply_edit(&board, &edit, 7).unwrap();
    let plain = doc.to_kicad_string();
    // Wrong net at the pad centre: the pad is not a connection target.
    assert_eq!(doc.enable_teardrops(&board, &edit), 0);
    assert_eq!(doc.to_kicad_string(), plain);
}
