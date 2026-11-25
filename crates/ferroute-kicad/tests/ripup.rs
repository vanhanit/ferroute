// SPDX-License-Identifier: AGPL-3.0-only
//! Writer side of the explicit user rip-up (INV-EDIT-1 exception, INV-IO-1).

use ferroute_board::{BoardEdit, NetId, Origin, RipupScope, Track, ripup};
use ferroute_geom::Point;
use ferroute_kicad::{KicadBoard, ReadOptions, Removal, read_board};

/// Layout (root children): footprint, then segments/arc/vias of nets A and B (one segment on
/// a non-copper layer that the reader skips, one locked), then a second footprint.
const SRC: &str = r#"(kicad_pcb (version 20260206) (generator "pcbnew") (paper "A4")
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user) (37 "F.SilkS" user))
  (footprint "L:X" (layer "F.Cu") (at 3 3)
    (property "Reference" "U1" (at 0 0 0) (layer "F.SilkS"))
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net "A")))
  (segment (start 1 1) (end 3 1) (width 0.2) (layer "F.Cu") (net "A") (uuid "s0"))
  (segment (start 1 2) (end 3 2) (width 0.2) (layer "Edge.Cuts") (net "A") (uuid "skipped"))
  (arc (start 3 1) (mid 4 2) (end 3 3) (width 0.2) (layer "F.Cu") (net "A") (uuid "a0"))
  (segment (start 1 4) (end 3 4) (width 0.2) (layer "F.Cu") (net "B") (uuid "s1"))
  (segment (start 1 5) (end 3 5) (width 0.2) (layer "F.Cu") (locked yes) (net "A") (uuid "locked"))
  (via (at 3 4) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net "B") (uuid "v1"))
  (via (at 3 1) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net "A") (uuid "v0"))
  (footprint "L:Y" (layer "F.Cu") (at 8 8)
    (property "Reference" "U2" (at 0 0 0) (layer "F.SilkS"))
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net "B")))
  (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts") (width 0.1))
)"#;

fn rm<'a>(
    original: &'a ferroute_board::Board,
    removed: &'a ferroute_board::RemovedItems,
    scope: &'a RipupScope,
) -> Removal<'a> {
    Removal { original, removed, scope }
}

fn load() -> KicadBoard {
    read_board(SRC, &ReadOptions::default()).unwrap()
}

fn children(kb: &KicadBoard) -> Vec<String> {
    kb.doc.root().items().unwrap().iter().map(ferroute_kicad::sexpr::SExpr::serialize).collect()
}

fn uuid_of(kb: &KicadBoard, head: &str) -> Vec<String> {
    kb.doc
        .root()
        .items()
        .unwrap()
        .iter()
        .filter(|c| c.head() == Some(head))
        .map(|c| c.child("uuid").and_then(|u| u.atom(1)).unwrap().to_owned())
        .collect()
}

#[test]
fn no_scope_is_byte_identical_to_a_plain_edit() {
    let mut a = load();
    let mut b = load();
    let r = ripup(&a.board, &RipupScope::None);
    assert!(r.removed.is_empty());
    let before = a.doc.to_kicad_string();
    a.doc.apply_edit(&a.board, &BoardEdit::default(), 7).unwrap();
    b.doc
        .apply_edit_removing(
            &b.board,
            &BoardEdit::default(),
            &rm(&b.board, &r.removed, &r.scope),
            7,
        )
        .unwrap();
    assert_eq!(a.doc.to_kicad_string(), before);
    assert_eq!(b.doc.to_kicad_string(), before);
}

#[test]
fn removed_nodes_are_gone_and_everything_else_is_byte_identical() {
    let mut kb = load();
    let before = children(&kb);
    let r = ripup(&kb.board, &RipupScope::All);
    // s0, the arc's chords, s1, v1, v0 (the locked segment stays).
    assert_eq!(r.report.skipped_locked, 1);
    kb.doc
        .apply_edit_removing(
            &kb.board,
            &BoardEdit::default(),
            &rm(&kb.board, &r.removed, &r.scope),
            1,
        )
        .unwrap();
    let after = children(&kb);
    let kept: Vec<String> = before
        .iter()
        .filter(|c| {
            !(c.starts_with("(via")
                || c.starts_with("(arc")
                || (c.starts_with("(segment") && (c.contains("\"s0\"") || c.contains("\"s1\""))))
        })
        .cloned()
        .collect();
    assert_eq!(after, kept);
    assert_eq!(uuid_of(&kb, "segment"), ["skipped", "locked"]);
    assert!(uuid_of(&kb, "via").is_empty() && uuid_of(&kb, "arc").is_empty());
    // The document still reads back, footprints intact and addressable after the shift.
    let again = read_board(&kb.doc.to_kicad_string(), &ReadOptions::default()).unwrap();
    assert_eq!(again.board.footprints.len(), 2);
    assert_eq!(again.board.tracks.len(), 1, "only the locked segment is copper");
    assert!(kb.doc.footprint_text(1).unwrap().contains("U2"));
}

#[test]
fn net_scope_removes_only_that_net_and_new_copper_is_appended() {
    let mut kb = load();
    let r = ripup(&kb.board, &RipupScope::Nets(vec!["B".into()]));
    let b = kb.board.net_by_name("B").unwrap();
    let new = Track {
        start: Point::new(1_000_000, 9_000_000),
        end: Point::new(3_000_000, 9_000_000),
        width: 200_000,
        layer: ferroute_board::LayerId(0),
        net: b,
        locked: false,
        origin: Origin::Routed,
    };
    let edit = BoardEdit { new_tracks: vec![new], ..BoardEdit::default() };
    kb.doc.apply_edit_removing(&r.board, &edit, &rm(&kb.board, &r.removed, &r.scope), 1).unwrap();
    assert_eq!(uuid_of(&kb, "via"), ["v0"]);
    assert_eq!(uuid_of(&kb, "arc"), ["a0"]);
    let segs = uuid_of(&kb, "segment");
    assert_eq!(segs.len(), 4, "{segs:?}");
    assert!(!segs.contains(&"s1".to_owned()) && segs.contains(&"s0".to_owned()));
}

#[test]
fn same_input_and_scope_give_identical_output() {
    let run = || {
        let mut kb = load();
        let r = ripup(&kb.board, &RipupScope::All);
        kb.doc
            .apply_edit_removing(
                &kb.board,
                &BoardEdit::default(),
                &rm(&kb.board, &r.removed, &r.scope),
                3,
            )
            .unwrap();
        kb.doc.to_kicad_string()
    };
    assert_eq!(run(), run());
}

#[test]
fn invariant_violations_are_rejected_and_leave_the_document_unchanged() {
    let mut kb = load();
    let before = kb.doc.to_kicad_string();
    let r = ripup(&kb.board, &RipupScope::All);
    // Removals without a scope.
    assert!(
        kb.doc
            .apply_edit_removing(
                &kb.board,
                &BoardEdit::default(),
                &rm(&kb.board, &r.removed, &RipupScope::None),
                1
            )
            .is_err()
    );
    // A locked track smuggled in.
    let mut bad = r.removed.clone();
    let locked = kb.board.tracks.iter().position(|t| t.locked).unwrap();
    bad.tracks.push(ferroute_board::TrackId(u32::try_from(locked).unwrap()));
    assert!(
        kb.doc
            .apply_edit_removing(
                &kb.board,
                &BoardEdit::default(),
                &rm(&kb.board, &bad, &r.scope),
                1
            )
            .is_err()
    );
    assert_eq!(kb.doc.to_kicad_string(), before);
    // A second rip-up on the same document is refused (indices are stale).
    kb.doc
        .apply_edit_removing(
            &kb.board,
            &BoardEdit::default(),
            &rm(&kb.board, &r.removed, &r.scope),
            1,
        )
        .unwrap();
    assert!(
        kb.doc
            .apply_edit_removing(
                &kb.board,
                &BoardEdit::default(),
                &rm(&kb.board, &r.removed, &r.scope),
                1
            )
            .is_err()
    );
    let _ = NetId::NONE;
}
