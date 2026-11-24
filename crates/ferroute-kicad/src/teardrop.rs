// SPDX-License-Identifier: AGPL-3.0-only
//! Native KiCad teardrops (opt-in). KiCad stores teardrop *parameters* on pads and vias
//! and generates the copper itself when zones are refilled, so ferroute only switches the
//! property on for the pads and vias it connected new copper to; it never draws teardrop
//! copper.
//!
//! Verified against the KiCad 10 sources (`pcb_io_kicad_sexpr.cpp`, `formatTeardropParameters`
//! and `parseTEARDROP_PARAMETERS`): the node is written only when the parameters differ
//! from the defaults, and the parser starts from `enabled no`, `allow_two_segments no`,
//! `prefer_zone_connections no`, so every token is written explicitly. In a pad it follows
//! the custom-shape primitives and precedes `(tenting …)` / `(uuid …)`; in a via it follows
//! the padstack and precedes `(net …)`.

use crate::doc::KicadDocument;
use crate::sexpr::SExpr;
use crate::util::{kv, mm, node};
use ferroute_board::{Board, BoardEdit, NetId};
use ferroute_geom::Point;
use std::collections::HashSet;

/// KiCad's default teardrop parameters with `enabled yes`, in KiCad's token order.
fn default_enabled() -> SExpr {
    node(
        "teardrops",
        vec![
            kv("best_length_ratio", "0.5"),
            kv("max_length", "1"),
            kv("best_width_ratio", "1"),
            kv("max_width", "2"),
            kv("curved_edges", "no"),
            kv("filter_ratio", "0.9"),
            kv("enabled", "yes"),
            kv("allow_two_segments", "yes"),
            kv("prefer_zone_connections", "yes"),
        ],
    )
}

/// Sets `(enabled yes)` on a pad or via node; returns `true` when the node changed.
/// An existing `(teardrops …)` keeps its other parameters. `before` lists the heads the
/// new node must precede.
fn enable_on(item: &mut SExpr, before: &[&str]) -> bool {
    if let Some(td) = item.child_mut("teardrops") {
        if td.child("enabled").and_then(|e| e.atom(1)) == Some("yes") {
            return false;
        }
        let Some(items) = td.items_mut() else { return false };
        match items.iter_mut().find(|c| c.head() == Some("enabled")) {
            Some(e) => *e = kv("enabled", "yes"),
            None => items.push(kv("enabled", "yes")),
        }
        return true;
    }
    let Some(items) = item.items_mut() else { return false };
    let at = items
        .iter()
        .position(|c| c.head().is_some_and(|h| before.contains(&h)))
        .unwrap_or(items.len());
    items.insert(at, default_enabled());
    true
}

fn via_at(v: &SExpr) -> Option<Point> {
    let at = v.child("at")?;
    Some(Point::new(mm(at.atom(1)?).ok()?, mm(at.atom(2)?).ok()?))
}

impl KicadDocument {
    /// Enables KiCad teardrops on every pad and via that `edit` connects new copper to: a
    /// pad whose centre is an endpoint of a new track or arc of its net, and any via
    /// (existing or new) located at such an endpoint or created by the edit. Call it after
    /// [`KicadDocument::apply_edit`] with the same `board_before` and `edit`. Nothing else
    /// is touched (INV-IO-1), the call is idempotent, and it returns the number of nodes
    /// it changed.
    pub fn enable_teardrops(&mut self, board_before: &Board, edit: &BoardEdit) -> usize {
        let mut ends: HashSet<(Point, NetId)> = HashSet::new();
        let mut points: HashSet<Point> = HashSet::new();
        for t in &edit.new_tracks {
            ends.extend([(t.start, t.net), (t.end, t.net)]);
        }
        for a in &edit.new_arcs {
            ends.extend([(a.start, a.net), (a.end, a.net)]);
        }
        points.extend(ends.iter().map(|&(p, _)| p));
        points.extend(edit.new_vias.iter().map(|v| v.pos));
        let placed;
        let board = if edit.moves.is_empty() {
            board_before
        } else {
            let mut b = board_before.clone();
            b.apply(&BoardEdit { moves: edit.moves.clone(), ..BoardEdit::default() });
            placed = b;
            &placed
        };
        let mut changed = 0;
        for (fi, fp) in board.footprints.iter().enumerate() {
            let Some(&node_idx) = self.fp_nodes.get(fi) else { continue };
            let Some(fp_node) = self.root.items_mut().and_then(|v| v.get_mut(node_idx)) else {
                continue;
            };
            let Some(children) = fp_node.items_mut() else { continue };
            let pad_nodes: Vec<usize> = children
                .iter()
                .enumerate()
                .filter(|(_, c)| c.head() == Some("pad"))
                .map(|(i, _)| i)
                .collect();
            for (k, &pid) in fp.pads.iter().enumerate() {
                let pad = board.pad(pid);
                if pad.net == NetId::NONE || !ends.contains(&(board.pad_position(pid), pad.net)) {
                    continue;
                }
                if let Some(&ci) = pad_nodes.get(k)
                    && enable_on(&mut children[ci], &["tenting", "uuid", "padstack"])
                {
                    changed += 1;
                }
            }
        }
        if let Some(items) = self.root.items_mut() {
            for v in items.iter_mut().filter(|c| c.head() == Some("via")) {
                if via_at(v).is_some_and(|p| points.contains(&p)) && enable_on(v, &["net", "uuid"])
                {
                    changed += 1;
                }
            }
        }
        changed
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sexpr::parse;

    #[test]
    fn existing_parameters_are_kept_and_only_enabled_flips() {
        let mut pad = parse(
            "(pad \"1\" smd rect (teardrops (best_length_ratio 0.7) (enabled no)) (uuid \"u\"))",
        )
        .unwrap();
        assert!(enable_on(&mut pad, &["tenting", "uuid"]));
        assert_eq!(
            pad.child("teardrops").unwrap().serialize().split_whitespace().collect::<String>(),
            "(teardrops(best_length_ratio0.7)(enabledyes))"
        );
        assert!(!enable_on(&mut pad, &["tenting", "uuid"]), "second call is a no-op");
    }

    #[test]
    fn missing_node_is_inserted_before_the_anchor_heads() {
        let mut via = parse("(via (at 1 2) (size 0.6) (net \"N\") (uuid \"u\"))").unwrap();
        assert!(enable_on(&mut via, &["net", "uuid"]));
        let heads: Vec<_> =
            via.items().unwrap().iter().filter_map(|c| c.head().map(str::to_owned)).collect();
        assert_eq!(heads, ["at", "size", "teardrops", "net", "uuid"]);
    }
}
