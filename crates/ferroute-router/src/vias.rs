// SPDX-License-Identifier: AGPL-3.0-only
//! Via relocation.
//!
//! The lattice search puts vias wherever the cheapest layer change was, and the later passes
//! ([`crate::straighten`], [`crate::chamfer`]) treat them as fixed anchors, so traces jog to
//! reach them. This pass moves each routed via that jogs (see
//! [`ferroute_drc::quality::via_jogs`]) onto a position where both of its traces run
//! straight: candidates are the intersections of the octilinear lines through the far
//! anchors of its two traces and the points where the via can slide onto such a line. A
//! candidate needs a legal via (exactly the oracle's clearance, hole, edge and keep-out
//! rules, never inside a pad or touching same-net copper) and exactly legal rebuilt chains
//! (the admissibility test of `straighten`, including pad entry axes). The best candidate is
//! taken only when strictly better by
//! `(slivers, near-pad bends, via jogs, bends, 90° bends, length)`.
//!
//! Via count never changes; vias of `Origin::Existing`, locked vias, T-junction vias (more
//! than one track per layer) and vias touching foreign same-net copper are left alone. A final
//! oracle run guards the pass: if the result is worse than the input in any measured way,
//! the input is returned unchanged.

use crate::access::octilinear_paths;
use crate::straighten::{
    Chain, Dynamic, Statics, moved_board, on_interior, quality, rebuild, score,
};
use ferroute_board::custom::{ItemCtx, ItemType};
use ferroute_board::{Board, BoardEdit, NetId, Origin, PadId, PadShape, Track, Via};
use ferroute_drc::ItemRef;
use ferroute_drc::quality::{ViaTrace, is_sliver, polylines_jog_weight};
use ferroute_geom::{Coord, Hull, Point, Rect};
use std::collections::BTreeSet;

const MAX_PASSES: usize = 4;
/// Candidates farther than this from the current via are not considered.
const MAX_MOVE: Coord = 3_000_000;
/// How many trailing vertices of a chain may be rebuilt (besides the whole chain).
const REBUILD_TAIL: usize = 3;

/// The four octilinear line directions.
const DIRS: [Point; 4] = [Point::new(1, 0), Point::new(0, 1), Point::new(1, 1), Point::new(1, -1)];

/// `(slivers, near-pad bends, via jogs, bends, bends of 90° or more, length)`, lexicographic.
type Total = (u64, u64, u64, u64, u64, u64);

/// Intersection of the lines `p + t·da` and `q + s·db`, when it is an integer point.
fn intersect(p: Point, da: Point, q: Point, db: Point) -> Option<Point> {
    let den = da.cross(db);
    if den == 0 {
        return None;
    }
    let num = (q - p).cross(db);
    if num % den != 0 {
        return None;
    }
    let t = num / den;
    let x = i128::from(p.x) + i128::from(da.x) * t;
    let y = i128::from(p.y) + i128::from(da.y) * t;
    Some(Point::new(Coord::try_from(x).ok()?, Coord::try_from(y).ok()?))
}

/// Candidate positions for a via now at `v` whose traces end at the anchors `fs`: the
/// intersections of the octilinear lines through the anchors, and the points where `v`
/// slides along an octilinear direction onto such a line. Sorted, deduplicated, within
/// [`MAX_MOVE`] of `v`, never `v` itself.
fn candidates(v: Point, fs: &[Point]) -> Vec<Point> {
    let mut out = BTreeSet::new();
    for (i, &f) in fs.iter().enumerate() {
        for &g in &fs[i + 1..] {
            for da in DIRS {
                for db in DIRS {
                    out.extend(intersect(f, da, g, db));
                }
            }
        }
        for da in DIRS {
            for db in DIRS {
                out.extend(intersect(f, da, v, db));
            }
        }
    }
    let limit = u128::try_from(MAX_MOVE).unwrap_or(0).pow(2);
    out.into_iter().filter(|&p| p != v && (p - v).norm2() <= limit).collect()
}

/// Same-net SMD pads centred exactly on one of `fs` whose copper fully contains a via of
/// `diameter` centred on them (the disc of that diameter fits the pad's smallest dimension).
fn pads_holding(board: &Board, net: NetId, diameter: Coord, fs: &[Point]) -> Vec<(Point, PadId)> {
    let mut out = Vec::new();
    for id in board.pad_ids() {
        let pad = board.pad(id);
        let c = board.pad_position(id);
        let fits = match pad.shape {
            PadShape::Circle { diameter: d } => diameter <= d,
            PadShape::Rect { w, h }
            | PadShape::Oval { w, h }
            | PadShape::RoundRect { w, h, .. } => diameter <= w.min(h),
            PadShape::Convex { .. } => false,
        };
        if pad.net == net
            && pad.drill.is_none()
            && !pad.layers.is_empty()
            && fits
            && fs.contains(&c)
        {
            out.push((c, id));
        }
    }
    out
}

/// Would a via like `via` (at its `pos`) be legal? Exactly the oracle's rules: copper
/// clearance on shared layers, hole-to-hole, copper-to-hole both ways, board edge and
/// no-via keep-outs; additionally it may neither touch nor sit within the clearance of
/// same-net copper outside `own` (the segments being rebuilt), and never covers a pad.
fn via_legal(
    st: &Statics<'_>,
    dynamic: &Dynamic,
    via: &Via,
    own: &[u32],
    inside: Option<ItemRef>,
) -> bool {
    let base = st.base;
    let hull = via.hull();
    let hole = Hull::disk(via.pos, via.drill / 2);
    let rules = &base.rules;
    if !base.outline.admits(&hull, rules.edge_clearance)
        || base
            .keepouts
            .iter()
            .any(|k| k.no_vias && k.layers.intersects(via.layers) && k.area.hull_violates(&hull, 0))
    {
        return false;
    }
    if ferroute_drc::zones::conflicts(base, &st.fills, &hull, via.layers, via.net) {
        return false;
    }
    let same = |n: NetId| n == via.net && n != NetId::NONE;
    let clr_same = base.clearance_between(via.net, via.net);
    let reach = st.reach.max(rules.effective_hole_to_hole()).max(clr_same) + 1;
    let window = Rect::new(via.pos, via.pos).inflate(via.diameter / 2 + reach);
    for k in st.query_items(window) {
        let it = &st.items[k as usize];
        if same(it.net) {
            if Some(it.item) == inside {
                continue;
            }
            if it.layers.intersects(via.layers) && hull.clearance_violated(&it.hull, clr_same) {
                return false;
            }
            continue;
        }
        if it.layers.intersects(via.layers)
            && hull.clearance_violated(&it.hull, base.clearance_between(via.net, it.net))
        {
            return false;
        }
        if hole.clearance_violated(&it.hull, rules.hole_clearance) {
            return false;
        }
    }
    let mut segs = Vec::new();
    dynamic.query(window, &mut segs);
    for id in segs {
        let s = &dynamic.segs[id as usize];
        let shape = s.hull();
        if same(s.net) {
            if !own.contains(&id)
                && via.layers.contains(s.layer)
                && hull.clearance_violated(&shape, clr_same)
            {
                return false;
            }
            continue;
        }
        if via.layers.contains(s.layer)
            && hull.clearance_violated(&shape, base.clearance_between(via.net, s.net))
        {
            return false;
        }
        if hole.clearance_violated(&shape, rules.hole_clearance) {
            return false;
        }
    }
    let wide = Rect::new(via.pos, via.pos).inflate(via.diameter / 2 + reach);
    for k in st.query_holes(wide) {
        let h = &st.holes[k as usize];
        let h2h = if rules.custom.is_empty() {
            rules.hole_to_hole
        } else {
            rules.hole_to_hole_for(
                base,
                &ItemCtx::new(via.net, ItemType::Via),
                &ItemCtx { net: Some(h.net), kind: None, layer: None },
            )
        };
        if hole.clearance_violated(&h.hull, h2h)
            || (!same(h.net) && hull.clearance_violated(&h.hull, rules.hole_clearance))
        {
            return false;
        }
    }
    true
}

/// One way to rebuild a chain: replace its vertices `i..` by `cand` (ending at the via).
#[derive(Clone)]
struct Rebuild {
    i: usize,
    cand: Vec<Point>,
    full: Vec<Point>,
    /// `(slivers, near, bends, right, length)` of the rebuilt chain.
    local: (u64, u64, u64, u64, u64),
}

fn sliver_count(st: &Statics<'_>, ch: &Chain, via: &Hull, pts: &[Point]) -> u32 {
    let clr = st.base.clearance_between(ch.net, ch.net);
    pts.windows(2)
        .filter(|s| is_sliver(via, &Hull::capsule(s[0], s[1], ch.template.width / 2), clr))
        .count()
        .try_into()
        .unwrap_or(u32::MAX)
}

/// Every admissible rebuild of `ch` (far anchor first, via last) ending at `p`.
fn rebuilds(
    st: &Statics<'_>,
    dynamic: &Dynamic,
    ch: &Chain,
    p: Point,
    via_hull: &Hull,
    collapse: bool,
) -> Vec<Rebuild> {
    let n = ch.pts.len() - 1;
    if collapse && ch.pts[0] == p {
        // The via sits in the chain's pad: nothing is left to route on this layer.
        return vec![Rebuild { i: 0, cand: vec![p], full: vec![p], local: (0, 0, 0, 0, 0) }];
    }
    let mut starts: BTreeSet<usize> = (n.saturating_sub(REBUILD_TAIL)..n).collect();
    starts.insert(0);
    let mut out = Vec::new();
    for i in starts {
        let a = ch.pts[i];
        if a == p {
            continue;
        }
        for cand in octilinear_paths(a, p) {
            // A bend exactly on a pad centre would enter the pad off its axes.
            if cand[1..cand.len() - 1].iter().any(|q| st.pads_at.contains_key(q))
                || !st.admissible(dynamic, ch, i, n, &cand)
            {
                continue;
            }
            let mut full = ch.pts[..i].to_vec();
            full.extend_from_slice(&cand);
            let sl = st.slivers(dynamic, ch, i, n, &full) + sliver_count(st, ch, via_hull, &full);
            let s = score(st.base, ch, None, &full, None, sl);
            out.push(Rebuild {
                i,
                cand,
                full,
                local: (u64::from(s.0), u64::from(s.1), u64::from(s.2), u64::from(s.3), s.4),
            });
        }
    }
    out
}

fn total(
    chains: [&Chain; 2],
    pts: [&[Point]; 2],
    local: [(u64, u64, u64, u64, u64); 2],
    at: Point,
    diameter: Coord,
) -> Total {
    let trace = |k: usize| ViaTrace {
        layer: chains[k].layer,
        width: chains[k].template.width,
        pts: pts[k],
    };
    let jog = u64::from(polylines_jog_weight(at, diameter, &[trace(0), trace(1)]));
    (
        local[0].0 + local[1].0,
        local[0].1 + local[1].1,
        jog,
        local[0].2 + local[1].2,
        local[0].3 + local[1].3,
        local[0].4 + local[1].4,
    )
}

/// The two single-per-layer chains ending at the via, far anchor first and via last.
fn incident_chains(st: &Statics<'_>, cur: &BoardEdit, via: &Via) -> Option<[Chain; 2]> {
    // Exactly one track end per layer on exactly two layers, no foreign same-net copper
    // touching the via, no same-net track passing through its centre.
    let mut ends = 0;
    let mut sub: Vec<Track> = Vec::new();
    let mut idx: Vec<usize> = Vec::new();
    for (k, t) in cur.new_tracks.iter().enumerate() {
        if t.net != via.net {
            continue;
        }
        if on_interior(t.start, t.end, via.pos) {
            return None;
        }
        ends += usize::from(t.start == via.pos) + usize::from(t.end == via.pos);
        sub.push(t.clone());
        idx.push(k);
    }
    if ends != 2 || via.layers.iter().any(|l| st.static_anchor(via.net, l, via.pos)) {
        return None;
    }
    let mut found: Vec<Chain> = Vec::new();
    for mut ch in st.chains_min(&sub, 1) {
        if ch.pts[0] != via.pos && ch.pts[ch.pts.len() - 1] != via.pos {
            continue;
        }
        if ch.pts[0] == ch.pts[ch.pts.len() - 1]
            || ch.template.origin != Origin::Routed
            || ch.template.locked
        {
            return None;
        }
        ch.ids =
            ch.ids.iter().map(|&i| u32::try_from(idx[i as usize]).unwrap_or(u32::MAX)).collect();
        ch.orig = ch.orig.iter().map(|&i| idx[i]).collect();
        if ch.pts[0] == via.pos {
            ch.pts.reverse();
            ch.ids.reverse();
            ch.orig.reverse();
        }
        found.push(ch);
    }
    let [a, b] = <[Chain; 2]>::try_from(found).ok()?;
    (a.layer != b.layer).then_some([a, b])
}

/// Tries to move via `k` of `cur`; returns the improved edit. `st` is built from the whole
/// of `cur`; via `k` is hidden from it for the duration of the call.
fn relocate_one(
    board: &Board,
    cur: &BoardEdit,
    k: usize,
    via_in_pad: bool,
    st: &mut Statics<'_>,
) -> Option<BoardEdit> {
    let via = &cur.new_vias[k];
    if via.origin != Origin::Routed || via.locked || via.net == NetId::NONE {
        return None;
    }
    st.hide_via(k);
    let r = relocate_hidden(board, cur, k, via_in_pad, st);
    st.unhide();
    r
}

fn relocate_hidden(
    board: &Board,
    cur: &BoardEdit,
    k: usize,
    via_in_pad: bool,
    st: &Statics<'_>,
) -> Option<BoardEdit> {
    let via = &cur.new_vias[k];
    let mut dynamic = Dynamic::of(&cur.new_tracks);
    let [mut c0, mut c1] = incident_chains(st, cur, via)?;
    let own: Vec<u32> = c0.ids.iter().chain(&c1.ids).copied().collect();
    let now = |c: &Chain| score(board, c, None, &c.pts, None, 0);
    let current = {
        let hull = via.hull();
        let loc = |c: &Chain| {
            let s = now(c);
            let sl = st.slivers(&dynamic, c, c.ids.len(), c.ids.len(), &c.pts)
                + sliver_count(st, c, &hull, &c.pts);
            (u64::from(sl), u64::from(s.1), u64::from(s.2), u64::from(s.3), s.4)
        };
        total([&c0, &c1], [&c0.pts, &c1.pts], [loc(&c0), loc(&c1)], via.pos, via.diameter)
    };
    if current.2 == 0 {
        return None; // does not jog
    }
    let fs = [c0.pts[0], c1.pts[0]];
    let mut best: Option<(Total, u128, Point, Rebuild, Rebuild)> = None;
    let pads =
        if via_in_pad { pads_holding(board, via.net, via.diameter, &fs) } else { Vec::new() };
    let limit = u128::try_from(MAX_MOVE).unwrap_or(0).pow(2);
    let mut cands: Vec<(Point, Option<PadId>)> =
        candidates(via.pos, &fs).into_iter().map(|p| (p, None)).collect();
    for &(c, id) in &pads {
        if c != via.pos && (c - via.pos).norm2() <= limit {
            cands.push((c, Some(id)));
        }
    }
    cands.sort_by_key(|&(p, id)| (p, id.is_none()));
    for (p, pad) in cands {
        let moved = Via { pos: p, ..via.clone() };
        if !via_legal(st, &dynamic, &moved, &own, pad.map(ItemRef::Pad)) {
            continue;
        }
        let hull = moved.hull();
        // A chain ending in the pad collapses (the pad must be on its layer).
        let collapse = |c: &Chain| {
            pad.is_some_and(|id| c.pts[0] == p && board.pad(id).layers.contains(c.layer))
        };
        let (o0, o1) = (
            rebuilds(st, &dynamic, &c0, p, &hull, collapse(&c0)),
            rebuilds(st, &dynamic, &c1, p, &hull, collapse(&c1)),
        );
        for a in &o0 {
            for b in &o1 {
                let t = total([&c0, &c1], [&a.full, &b.full], [a.local, b.local], p, via.diameter);
                let key = (p - via.pos).norm2();
                if t < current && best.as_ref().is_none_or(|x| (t, key) < (x.0, x.1)) {
                    best = Some((t, key, p, a.clone(), b.clone()));
                }
            }
        }
    }
    let (_, _, p, a, b) = best?;
    let n0 = c0.ids.len();
    c0.splice(&mut dynamic, a.i, n0, a.cand);
    let n1 = c1.ids.len();
    c1.splice(&mut dynamic, b.i, n1, b.cand);
    let mut out = rebuild(cur, &[c0, c1]);
    out.new_vias[k].pos = p;
    Some(out)
}

/// Moves routed vias so that their traces run straight into them. `base` is the board BEFORE
/// `edit`. Returns an edit that differs only in via positions and `new_tracks`.
#[must_use]
pub fn relocate_vias(base: &Board, edit: &BoardEdit, via_in_pad: bool) -> BoardEdit {
    let moved = moved_board(base, edit);
    let board = moved.as_ref();
    let mut cur = edit.clone();
    let mut any = false;
    let jogs_of = |e: &BoardEdit| {
        let mut after = base.clone();
        after.apply(e);
        ferroute_drc::quality::via_jogs(&after)
    };
    for _ in 0..MAX_PASSES {
        let jogging = jogs_of(&cur);
        let mut count = jogging.len();
        let jogging: BTreeSet<Point> = jogging.into_iter().map(|j| j.at).collect();
        let mut changed = false;
        // The static index is rebuilt only when a via actually moves.
        let mut shared: Option<Statics<'_>> = None;
        for k in 0..cur.new_vias.len() {
            if !jogging.contains(&cur.new_vias[k].pos) {
                continue;
            }
            let st = shared.get_or_insert_with(|| Statics::new(board, &cur));
            if let Some(next) = relocate_one(board, &cur, k, via_in_pad, st) {
                // A move may also disturb the via at the far end of a chain: keep it only
                // if the board as a whole jogs less.
                let now = jogs_of(&next).len();
                if now <= count {
                    cur = next;
                    count = now;
                    changed = true;
                    shared = None;
                }
            }
        }
        any |= changed;
        if !changed {
            break;
        }
    }
    if !any {
        return edit.clone();
    }
    // Safety net: never return anything the oracle likes less than the input.
    let (before, after) = (quality(base, edit), quality(base, &cur));
    let slivers = |e: &BoardEdit| {
        let mut b = base.clone();
        b.apply(e);
        ferroute_drc::quality::slivers(&b).len()
    };
    if after.0 > before.0
        || after.1 > before.1
        || after.2 > before.2
        || slivers(&cur) > slivers(edit)
    {
        tracing::error!(?before, ?after, "via relocation made the result worse; keeping the input");
        return edit.clone();
    }
    cur
}

#[cfg(test)]
mod tests {
    use super::*;
    use ferroute_board::builder::{BoardBuilder, mm};
    use ferroute_board::{LayerId, Track};
    use ferroute_drc::quality::{acute_angles, via_jogs};
    use ferroute_drc::{DrcConfig, check};

    const W: i64 = 250_000;

    #[allow(clippy::cast_possible_truncation)]
    fn p(x: f64, y: f64) -> Point {
        Point::new((x * 1e6).round() as i64, (y * 1e6).round() as i64)
    }

    fn trk(net: NetId, layer: u8, pts: &[Point], origin: Origin) -> Vec<Track> {
        pts.windows(2)
            .map(|w| Track {
                start: w[0],
                end: w[1],
                width: W,
                layer: LayerId(layer),
                net,
                locked: false,
                origin,
            })
            .collect()
    }

    fn via_at(pos: Point, net: NetId) -> Via {
        Via {
            pos,
            diameter: 600_000,
            drill: 300_000,
            kind: ferroute_board::ViaKind::Through,
            layers: ferroute_board::LayerSet(0b11),
            net,
            locked: false,
            origin: Origin::Routed,
        }
    }

    /// Pad A (5, 5.13) leaving along +x; existing layer-1 copper of the net from (15, 5.13).
    fn board() -> (Board, NetId, NetId, Point, Point) {
        let mut b = BoardBuilder::new(mm(30), mm(20), 2);
        let n = b.net("N");
        let m = b.net("M");
        b.smd2("R1", p(4.2, 5.13), 180.0, n, NetId::NONE);
        let mut board = b.build();
        board.tracks.extend(trk(n, 1, &[p(15.0, 5.13), p(25.0, 5.13)], Origin::Existing));
        (board, n, m, p(5.0, 5.13), p(15.0, 5.13))
    }

    /// Layer 0: A, jog, via 0.3 mm off the pad axis; layer 1: on to the existing copper.
    fn jogging_edit(n: NetId, a: Point, f: Point) -> BoardEdit {
        let v = p(10.0, 5.43);
        let mut tracks = trk(n, 0, &[a, p(9.7, 5.13), v], Origin::Routed);
        tracks.extend(trk(n, 1, &[v, p(14.7, 5.43), f], Origin::Routed));
        BoardEdit { new_tracks: tracks, new_vias: vec![via_at(v, n)], ..BoardEdit::default() }
    }

    fn assert_clean(base: &Board, edit: &BoardEdit) {
        let mut after = base.clone();
        after.apply(edit);
        let rep = check(&after, &DrcConfig { courtyards: false, ..DrcConfig::default() });
        let intro: Vec<_> = rep.introduced().collect();
        assert!(intro.is_empty(), "introduced violations: {intro:#?}");
        assert_eq!(rep.connectivity.connections_missing, 0, "{:?}", rep.connectivity);
        assert!(acute_angles(&after).is_empty(), "acid traps");
    }

    fn jogs(base: &Board, edit: &BoardEdit) -> usize {
        let mut after = base.clone();
        after.apply(edit);
        via_jogs(&after).len()
    }

    #[test]
    fn offset_via_moves_onto_the_pad_axis_and_both_layers_run_straight() {
        let (base, n, _m, a, f) = board();
        let edit = jogging_edit(n, a, f);
        assert_clean(&base, &edit);
        assert_eq!(jogs(&base, &edit), 1);
        let out = relocate_vias(&base, &edit, false);
        assert_clean(&base, &out);
        assert_eq!(out.new_vias.len(), 1);
        assert_eq!(out.new_vias[0].pos, p(10.0, 5.13), "via sits on the pad axis");
        assert_eq!(out.new_tracks.len(), 2, "{:#?}", out.new_tracks);
        for t in &out.new_tracks {
            assert_eq!(t.start.y, p(0.0, 5.13).y);
            assert_eq!(t.end.y, p(0.0, 5.13).y);
        }
        assert_eq!(jogs(&base, &out), 0);
        assert!(out.moves.is_empty());
    }

    #[test]
    fn blocked_axis_position_leaves_the_via_in_place() {
        let (mut base, n, m, a, f) = board();
        // A foreign via 0.73 mm below the axis position: every nearby candidate is illegal.
        base.vias.push(Via { origin: Origin::Existing, ..via_at(p(10.0, 4.4), m) });
        let edit = jogging_edit(n, a, f);
        assert_clean(&base, &edit);
        let out = relocate_vias(&base, &edit, false);
        assert_clean(&base, &out);
        assert_eq!(out, edit);
    }

    /// Via 1.5 mm from pad A with its axis positions blocked by a foreign via.
    fn near_pad_case() -> (Board, BoardEdit, Point) {
        let (mut base, n, m, a, f) = board();
        base.vias.push(Via { origin: Origin::Existing, ..via_at(p(6.5, 4.4), m) });
        let v = p(6.5, 5.43);
        let mut tracks = trk(n, 0, &[a, p(6.2, 5.13), v], Origin::Routed);
        tracks.extend(trk(n, 1, &[v, p(14.7, 5.43), f], Origin::Routed));
        let edit =
            BoardEdit { new_tracks: tracks, new_vias: vec![via_at(v, n)], ..BoardEdit::default() };
        (base, edit, a)
    }

    #[test]
    fn via_in_pad_off_never_enters_the_pad() {
        let (base, edit, a) = near_pad_case();
        assert_clean(&base, &edit);
        let out = relocate_vias(&base, &edit, false);
        assert_clean(&base, &out);
        let pad = base.pad_ids().find(|&id| base.pad_position(id) == a).expect("pad A");
        let via = out.new_vias[0].hull();
        assert!(!via.clearance_violated(
            &base.pad_hull(pad),
            base.rules.class_of(&base, edit.new_vias[0].net).clearance
        ));
    }

    #[test]
    fn via_in_pad_on_moves_the_via_into_the_pad_centre() {
        let (base, edit, a) = near_pad_case();
        let out = relocate_vias(&base, &edit, true);
        assert_clean(&base, &out);
        assert_eq!(out.new_vias[0].pos, a);
        assert!(out.new_tracks.iter().all(|t| t.layer == LayerId(1)), "{:#?}", out.new_tracks);
        assert_eq!(out.new_tracks.len(), 1);
        assert_eq!(jogs(&base, &out), 0);
        assert_eq!(out, relocate_vias(&base, &edit, true), "deterministic");
    }

    #[test]
    fn t_junction_and_existing_vias_are_untouched() {
        let (base, n, _m, a, f) = board();
        let mut edit = jogging_edit(n, a, f);
        // A branch on layer 0 makes the via a T junction.
        edit.new_tracks.extend(trk(n, 0, &[p(10.0, 5.43), p(10.0, 8.0)], Origin::Routed));
        assert_eq!(relocate_vias(&base, &edit, false), edit);
        // Existing vias never move.
        let mut ex = jogging_edit(n, a, f);
        ex.new_vias[0].origin = Origin::Existing;
        assert_eq!(relocate_vias(&base, &ex, false), ex);
    }

    #[test]
    fn deterministic_and_idempotent() {
        let (base, n, _m, a, f) = board();
        let edit = jogging_edit(n, a, f);
        let o1 = relocate_vias(&base, &edit, false);
        assert_eq!(o1, relocate_vias(&base, &edit, false));
        assert_eq!(o1, relocate_vias(&base, &o1, false));
    }

    #[test]
    fn candidates_are_integer_octilinear_alignments() {
        let c = candidates(p(10.0, 5.43), &[p(5.0, 5.13), p(15.0, 5.13)]);
        assert!(c.contains(&p(10.0, 5.13)));
        assert!(c.iter().all(|&q| (q - p(10.0, 5.43)).norm2() <= 9_000_000_000_000));
        assert!(!c.contains(&p(10.0, 5.43)));
        assert_eq!(
            intersect(Point::new(0, 0), Point::new(1, 1), Point::new(1, 0), Point::new(1, -1)),
            None
        );
    }
}
