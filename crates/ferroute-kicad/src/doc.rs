// SPDX-License-Identifier: AGPL-3.0-only
//! The lossless document: the parsed tree plus enough bookkeeping to apply a
//! [`BoardEdit`] by touching only the nodes that change.

use crate::KicadError;
use crate::read::NetDialect;
use crate::sexpr::SExpr;
use crate::util::{float, format_angle, kq, kv, node, norm180, same_angle, uuid_for, xy_node};
use crate::write::KICAD10_VERSION;
use ferroute_board::{
    ArcTrack, Board, BoardEdit, Placement, RemovedItems, RipupScope, Track, Via, ViaKind,
};
use ferroute_geom::units::{format_mm, parse_mm};
use std::collections::BTreeSet;

/// UUID kinds (domain separation for [`uuid_for`]).
pub(crate) const KIND_TRACK: u64 = 1;
pub(crate) const KIND_VIA: u64 = 2;
pub(crate) const KIND_ARC: u64 = 3;

/// A parsed `.kicad_pcb` that can be edited in place and written back.
#[derive(Clone, Debug)]
pub struct KicadDocument {
    pub(crate) root: SExpr,
    pub(crate) version: u32,
    pub(crate) dialect: NetDialect,
    /// Root-list index of every footprint, in `Board::footprints` order.
    pub(crate) fp_nodes: Vec<usize>,
    /// Root-list index of the source node of every `Board::tracks` entry (arcs: one per
    /// chord) and `Board::vias` entry, as read. Emptied by a rip-up (indices go stale).
    pub(crate) track_nodes: Vec<usize>,
    pub(crate) via_nodes: Vec<usize>,
    /// Number of tracks / vias this document has appended (keeps UUIDs unique across
    /// successive edits with the same seed).
    pub(crate) added_tracks: u64,
    pub(crate) added_arcs: u64,
    pub(crate) added_vias: u64,
}

fn at_number(at: &SExpr, i: usize) -> Result<f64, KicadError> {
    at.atom(i).map(float).transpose().map(|v| v.unwrap_or(0.0))
}

/// Adds `delta` degrees to the angle (element 3) of an `(at x y [a])` node. With
/// `omit_zero`, a zero result removes the angle like KiCad's footprint/pad writers.
fn rotate_at(at: &mut SExpr, delta: f64, omit_zero: bool) -> Result<(), KicadError> {
    let old = at_number(at, 3)?;
    let new = norm180(old + delta);
    let Some(items) = at.items_mut() else { return Ok(()) };
    items.truncate(3);
    if !(omit_zero && new.abs() < 1e-9) {
        items.push(SExpr::sym(format_angle(new)));
    }
    Ok(())
}

fn move_footprint(fp: &mut SExpr, to: Placement) -> Result<(), KicadError> {
    let at = fp.child("at").ok_or_else(|| KicadError::Format("footprint without (at)".into()))?;
    let ox = parse_mm(at.atom(1).unwrap_or("0")).map_err(|e| KicadError::Format(e.to_string()))?;
    let oy = parse_mm(at.atom(2).unwrap_or("0")).map_err(|e| KicadError::Format(e.to_string()))?;
    let old_rot = at_number(at, 3)?;
    let new_rot = to.rot.degrees();
    let delta = norm180(new_rot - old_rot);
    let rotated = !same_angle(new_rot, old_rot);
    let moved = (ox, oy) != (to.pos.x, to.pos.y);
    if moved || rotated {
        let at = fp
            .child_mut("at")
            .ok_or_else(|| KicadError::Format("footprint without (at)".into()))?;
        let items = at.items_mut().ok_or_else(|| KicadError::Format("malformed (at)".into()))?;
        if items.len() < 3 {
            return Err(KicadError::Format("footprint (at) needs x and y".into()));
        }
        if moved {
            items[1] = SExpr::sym(format_mm(to.pos.x));
            items[2] = SExpr::sym(format_mm(to.pos.y));
        }
        if rotated {
            items.truncate(3);
            let a = norm180(new_rot);
            if a.abs() > 1e-9 {
                items.push(SExpr::sym(format_angle(a)));
            }
        }
    }
    if !rotated {
        return Ok(());
    }
    // KiCad's FOOTPRINT::SetOrientation rotates every child by the same delta: pad and text
    // angles are absolute, positions of children are stored in the unrotated frame.
    let Some(items) = fp.items_mut() else { return Ok(()) };
    for child in items.iter_mut().skip(1) {
        match child.head() {
            Some("pad") => {
                if let Some(at) = child.child_mut("at") {
                    rotate_at(at, delta, true)?;
                }
            }
            Some("property" | "fp_text") => {
                if let Some(at) = child.child_mut("at") {
                    rotate_at(at, delta, false)?;
                }
                // Outline-font render caches are stale after rotation; KiCad rebuilds them.
                if let Some(list) = child.items_mut() {
                    list.retain(|c| c.head() != Some("render_cache"));
                }
            }
            Some("fp_text_box") => {
                let new = {
                    let old =
                        child.child("angle").map(|a| at_number(a, 1)).transpose()?.unwrap_or(0.0);
                    norm180(old + delta)
                };
                if let Some(list) = child.items_mut() {
                    list.retain(|c| c.head() != Some("angle") && c.head() != Some("render_cache"));
                    if new.abs() > 1e-9 {
                        // KiCad writes (angle ..) right after the text and before (start).
                        let pos = list
                            .iter()
                            .position(|c| c.head() == Some("start"))
                            .unwrap_or(list.len());
                        list.insert(pos, kv("angle", format_angle(new)));
                    }
                }
            }
            _ => {}
        }
    }
    Ok(())
}

/// `(net ...)` node for `net` in the given dialect.
pub(crate) fn net_node(
    dialect: &NetDialect,
    board: &Board,
    net: ferroute_board::NetId,
) -> Result<SExpr, KicadError> {
    if net == ferroute_board::NetId::NONE {
        // Unconnected: legacy files carry (net 0); KiCad 10 writes the empty net name.
        return Ok(match dialect {
            NetDialect::Names => kq("net", ""),
            NetDialect::Codes(_) => kv("net", "0"),
        });
    }
    let name = &board
        .nets
        .get(net.idx())
        .ok_or_else(|| KicadError::Format(format!("edit references unknown net {}", net.0)))?
        .name;
    Ok(match dialect {
        NetDialect::Names => kq("net", name.as_str()),
        NetDialect::Codes(codes) => {
            let code = codes.get(name).ok_or_else(|| {
                KicadError::Format(format!("net {name:?} is not in the file's net table"))
            })?;
            kv("net", code.to_string())
        }
    })
}

fn layer_name(board: &Board, l: ferroute_board::LayerId) -> Result<String, KicadError> {
    board
        .copper_layers
        .get(l.0 as usize)
        .map(|c| c.name.clone())
        .ok_or_else(|| KicadError::Format(format!("edit references unknown layer {}", l.0)))
}

/// `(segment ...)` node in KiCad 10 attribute order.
pub(crate) fn track_node(
    dialect: &NetDialect,
    board: &Board,
    t: &Track,
    seed: u64,
    index: u64,
) -> Result<SExpr, KicadError> {
    let mut items =
        vec![xy_node("start", t.start), xy_node("end", t.end), kv("width", format_mm(t.width))];
    if t.locked {
        items.push(kv("locked", "yes"));
    }
    items.push(kq("layer", layer_name(board, t.layer)?));
    items.push(net_node(dialect, board, t.net)?);
    items.push(kq("uuid", uuid_for(seed, KIND_TRACK, index)));
    Ok(node("segment", items))
}

/// `(arc ...)` track node in KiCad 10 attribute order.
pub(crate) fn arc_node(
    dialect: &NetDialect,
    board: &Board,
    a: &ArcTrack,
    seed: u64,
    index: u64,
) -> Result<SExpr, KicadError> {
    let mut items = vec![
        xy_node("start", a.start),
        xy_node("mid", a.mid),
        xy_node("end", a.end),
        kv("width", format_mm(a.width)),
    ];
    if a.locked {
        items.push(kv("locked", "yes"));
    }
    items.push(kq("layer", layer_name(board, a.layer)?));
    items.push(net_node(dialect, board, a.net)?);
    items.push(kq("uuid", uuid_for(seed, KIND_ARC, index)));
    Ok(node("arc", items))
}

/// `(via ...)` node in KiCad 10 attribute order.
pub(crate) fn via_node(
    dialect: &NetDialect,
    board: &Board,
    v: &Via,
    seed: u64,
    index: u64,
    version: u32,
) -> Result<SExpr, KicadError> {
    let ids: Vec<_> = v.layers.iter().collect();
    let (Some(&lo), Some(&hi)) = (ids.first(), ids.last()) else {
        return Err(KicadError::Format("via without layers".into()));
    };
    let n = board.copper_layers.len();
    let mut items = Vec::new();
    // Through vias carry no type keyword; partial spans keep KiCad's keywords (`micro`
    // when the via says so, else `blind` when one end is an outer layer, else `buried`).
    let through = lo.0 == 0 && usize::from(hi.0) + 1 == n;
    if !through {
        let outer = lo.0 == 0 || usize::from(hi.0) + 1 == n;
        items.push(SExpr::sym(match v.kind {
            ViaKind::Micro => "micro",
            _ if outer => "blind",
            _ => "buried",
        }));
    }
    items.push(xy_node("at", v.pos));
    items.push(kv("size", format_mm(v.diameter)));
    items.push(kv("drill", format_mm(v.drill)));
    items.push(node(
        "layers",
        vec![SExpr::quoted(layer_name(board, lo)?), SExpr::quoted(layer_name(board, hi)?)],
    ));
    if v.locked {
        items.push(kv("locked", "yes"));
    }
    // A via inside an SMD pad must be filled and capped (IPC-4761 type VII). The per-via
    // `(capping …)` / `(filling …)` attributes exist in the KiCad 10 format (the
    // padstack drill flags `is_capped` / `is_filled`); older dialects have no such token.
    if version >= KICAD10_VERSION && board.via_in_pad(v).is_some() {
        items.push(kv("capping", "yes"));
        items.push(kv("filling", "yes"));
    }
    items.push(net_node(dialect, board, v.net)?);
    items.push(kq("uuid", uuid_for(seed, KIND_VIA, index)));
    Ok(node("via", items))
}

/// Replaces the `(filled_polygon ...)` children of the zones named by `edit.zone_fills`
/// (indices into `Board::zones`, i.e. top-level copper zones in file order). Nothing else in
/// the zone nodes is touched.
fn apply_zone_fills(root: &mut SExpr, board: &Board, edit: &BoardEdit) -> Result<(), KicadError> {
    if edit.zone_fills.is_empty() {
        return Ok(());
    }
    let nodes: Vec<usize> = root
        .items()
        .unwrap_or(&[])
        .iter()
        .enumerate()
        .filter(|(_, c)| c.head() == Some("zone") && c.child("keepout").is_none())
        .map(|(i, _)| i)
        .collect();
    if nodes.len() != board.zones.len() {
        return Err(KicadError::Format(
            "board does not match this document (copper zone count differs)".into(),
        ));
    }
    for (zi, fills) in &edit.zone_fills {
        let at_node = *nodes
            .get(*zi as usize)
            .ok_or_else(|| KicadError::Format("fill edit of an unknown zone".into()))?;
        let mut fresh = Vec::new();
        for f in fills {
            let name = layer_name(board, f.layer)?;
            for p in &f.polys {
                let pts = p.pts.iter().map(|&q| xy_node("xy", q)).collect();
                fresh.push(node(
                    "filled_polygon",
                    vec![kq("layer", name.clone()), node("pts", pts)],
                ));
            }
        }
        let zone = root
            .items_mut()
            .and_then(|v| v.get_mut(at_node))
            .and_then(SExpr::items_mut)
            .ok_or_else(|| KicadError::Format("zone node index out of range".into()))?;
        let first = zone.iter().position(|c| c.head() == Some("filled_polygon"));
        zone.retain(|c| c.head() != Some("filled_polygon"));
        let at = first.unwrap_or(zone.len());
        zone.splice(at..at, fresh);
    }
    Ok(())
}

/// Root-list positions of the nodes a [`Board`] was read from.
pub(crate) struct SourceNodes {
    pub footprints: Vec<usize>,
    pub tracks: Vec<usize>,
    pub vias: Vec<usize>,
}

/// An explicit user rip-up to apply together with an edit (see
/// [`KicadDocument::apply_edit_removing`]).
#[derive(Clone, Copy, Debug)]
pub struct Removal<'a> {
    /// The board as read from this document (the removed indices refer to it).
    pub original: &'a Board,
    pub removed: &'a RemovedItems,
    pub scope: &'a RipupScope,
}

impl KicadDocument {
    pub(crate) fn new(root: SExpr, version: u32, dialect: NetDialect, src: SourceNodes) -> Self {
        Self {
            root,
            version,
            dialect,
            fp_nodes: src.footprints,
            track_nodes: src.tracks,
            via_nodes: src.vias,
            added_tracks: 0,
            added_arcs: 0,
            added_vias: 0,
        }
    }

    /// Root-list indices of the nodes `removed` refers to (an arc node counts once).
    fn removal_nodes(
        &self,
        board: &Board,
        removed: &RemovedItems,
    ) -> Result<BTreeSet<usize>, KicadError> {
        if removed.is_empty() {
            return Ok(BTreeSet::new());
        }
        if self.track_nodes.len() != board.tracks.len() || self.via_nodes.len() != board.vias.len()
        {
            return Err(KicadError::Format(
                "board does not match this document for a rip-up (copper changed since reading)"
                    .into(),
            ));
        }
        if !removed.arcs.is_empty() {
            return Err(KicadError::Format(
                "rip-up of pre-existing arc items is not supported (the reader produces none)"
                    .into(),
            ));
        }
        let mut nodes = BTreeSet::new();
        for t in &removed.tracks {
            nodes.insert(self.track_nodes[t.idx()]);
        }
        for v in &removed.vias {
            nodes.insert(self.via_nodes[v.idx()]);
        }
        // An arc node is read as several chords: all of them must be removed together.
        for (i, n) in self.track_nodes.iter().enumerate() {
            if nodes.contains(n) && !removed.tracks.iter().any(|t| t.idx() == i) {
                return Err(KicadError::Format("rip-up removes only part of an arc track".into()));
            }
        }
        Ok(nodes)
    }

    /// The `(version N)` of the file (KiCad 10 writes 20260206).
    #[must_use]
    pub fn version(&self) -> u32 {
        self.version
    }

    /// The whole file as KiCad-style text.
    #[must_use]
    pub fn to_kicad_string(&self) -> String {
        self.root.serialize()
    }

    /// The parsed tree (read-only).
    #[must_use]
    pub fn root(&self) -> &SExpr {
        &self.root
    }

    /// Serialized text of footprint `idx` (for byte-identity checks).
    #[must_use]
    pub fn footprint_text(&self, idx: usize) -> Option<String> {
        let n = *self.fp_nodes.get(idx)?;
        Some(self.root.items()?.get(n)?.serialize())
    }

    /// `true` when nets are referenced by name (KiCad 10) rather than by numeric code.
    #[must_use]
    pub fn uses_net_names(&self) -> bool {
        self.dialect == NetDialect::Names
    }

    /// Where new tracks/vias go: after the last existing segment/arc/via, else before the
    /// first zone/group/generator/embedded section, else at the end.
    fn insertion_index(&self) -> usize {
        let items = self.root.items().unwrap_or(&[]);
        let last = items.iter().rposition(|c| matches!(c.head(), Some("segment" | "arc" | "via")));
        if let Some(i) = last {
            return i + 1;
        }
        items
            .iter()
            .position(|c| {
                matches!(
                    c.head(),
                    Some("zone" | "group" | "generator" | "embedded_fonts" | "embedded_files")
                )
            })
            .unwrap_or(items.len())
    }

    /// Applies `edit` (computed against `board_before`) to the tree: rewrites the `(at)` of
    /// moved footprints (and rotates pad/text angles by the same delta as KiCad does) and
    /// appends new segments, arcs and vias (in that order) and replaces the
    /// `(filled_polygon ...)` children of zones in `edit.zone_fills`. No other node is modified. Output is deterministic in
    /// `(document, edit, seed)`.
    ///
    /// # Errors
    /// [`KicadError::Format`] when the edit violates `INV-EDIT-1`, does not match this
    /// document, or references nets/layers the file cannot express. On error the document
    /// is left unchanged.
    pub fn apply_edit(
        &mut self,
        board_before: &Board,
        edit: &BoardEdit,
        seed: u64,
    ) -> Result<(), KicadError> {
        self.apply_edit_inner(board_before, edit, None, seed)
    }

    /// As [`Self::apply_edit`], and additionally deletes exactly the nodes of the
    /// explicitly ripped-up copper (`removal.removed`, indices into `removal.original`,
    /// produced by [`ferroute_board::ripup`]). `board_before` is the board the edit was
    /// computed on (normally the ripped board, with placement policy applied). Nothing else
    /// is touched; with empty `removed` the result is byte-identical to [`Self::apply_edit`].
    /// A document accepts one rip-up (node positions go stale afterwards).
    ///
    /// # Errors
    /// As [`Self::apply_edit`], plus [`KicadError::Format`] when `removed` violates
    /// INV-EDIT-1 (no scope, locked or out-of-scope copper) or does not match this document.
    pub fn apply_edit_removing(
        &mut self,
        board_before: &Board,
        edit: &BoardEdit,
        removal: &Removal<'_>,
        seed: u64,
    ) -> Result<(), KicadError> {
        self.apply_edit_inner(board_before, edit, Some(removal), seed)
    }

    fn apply_edit_inner(
        &mut self,
        board_before: &Board,
        edit: &BoardEdit,
        removal: Option<&Removal<'_>>,
        seed: u64,
    ) -> Result<(), KicadError> {
        edit.check_invariants(board_before).map_err(KicadError::Format)?;
        let gone = match removal {
            Some(r) => {
                r.removed.check_invariants(r.original, r.scope).map_err(KicadError::Format)?;
                self.removal_nodes(r.original, r.removed)?
            }
            None => BTreeSet::new(),
        };
        if board_before.footprints.len() != self.fp_nodes.len() {
            return Err(KicadError::Format(
                "board does not match this document (footprint count differs)".into(),
            ));
        }
        // Build everything first so a failure leaves the document untouched.
        // New copper order: segments, then arcs, then vias.
        let mut new_nodes =
            Vec::with_capacity(edit.new_tracks.len() + edit.new_arcs.len() + edit.new_vias.len());
        for (i, t) in edit.new_tracks.iter().enumerate() {
            let idx = self.added_tracks + i as u64;
            new_nodes.push(track_node(&self.dialect, board_before, t, seed, idx)?);
        }
        for (i, a) in edit.new_arcs.iter().enumerate() {
            let idx = self.added_arcs + i as u64;
            new_nodes.push(arc_node(&self.dialect, board_before, a, seed, idx)?);
        }
        // New vias are judged against the pads where the moved parts end up.
        let placed;
        let after = if edit.moves.is_empty() || edit.new_vias.is_empty() {
            board_before
        } else {
            let mut b = board_before.clone();
            b.apply(&BoardEdit { moves: edit.moves.clone(), ..BoardEdit::default() });
            placed = b;
            &placed
        };
        for (i, v) in edit.new_vias.iter().enumerate() {
            let idx = self.added_vias + i as u64;
            new_nodes.push(via_node(&self.dialect, after, v, seed, idx, self.version)?);
        }
        let mut edited = self.root.clone();
        for m in &edit.moves {
            let idx = *self
                .fp_nodes
                .get(m.footprint.idx())
                .ok_or_else(|| KicadError::Format("move of unknown footprint".into()))?;
            let fp = edited
                .items_mut()
                .and_then(|v| v.get_mut(idx))
                .ok_or_else(|| KicadError::Format("footprint node index out of range".into()))?;
            move_footprint(fp, m.to)?;
        }
        apply_zone_fills(&mut edited, board_before, edit)?;
        if !gone.is_empty() {
            if let Some(items) = edited.items_mut() {
                let mut i = 0usize;
                items.retain(|_| {
                    let keep = !gone.contains(&i);
                    i += 1;
                    keep
                });
            }
            for f in &mut self.fp_nodes {
                *f -= gone.range(..*f).count();
            }
            self.track_nodes.clear();
            self.via_nodes.clear();
        }
        self.root = edited;
        if !new_nodes.is_empty() {
            let at = self.insertion_index();
            let n = new_nodes.len();
            if let Some(items) = self.root.items_mut() {
                items.splice(at..at, new_nodes);
            }
            for f in &mut self.fp_nodes {
                if *f >= at {
                    *f += n;
                }
            }
        }
        self.added_tracks += edit.new_tracks.len() as u64;
        self.added_arcs += edit.new_arcs.len() as u64;
        self.added_vias += edit.new_vias.len() as u64;
        Ok(())
    }
}
