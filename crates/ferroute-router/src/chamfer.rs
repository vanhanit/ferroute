// SPDX-License-Identifier: AGPL-3.0-only
//! 90° → 45° chamfer pass.
//!
//! A routed trace never bends 90°: every right-angle corner of the emitted copper is
//! replaced by two 45° bends (`P→K→Q` becomes `P→T1→T2→Q`, with `T1`/`T2` a distance `c`
//! before/after the corner). The largest `c` (at most half of the shorter leg, halved until
//! it is at least one trace width) whose three segments are *exactly* legal wins; corners
//! that cannot be chamfered are kept. Legality reuses the exact checks of
//! [`straighten`](crate::straighten): net-pair clearance, board edge, keep-outs, holes,
//! same-net overlap, pad entry axes. Where a leg ends at a pad centre the new first
//! point must lie outside that pad's copper so the axial segment still crosses the pad
//! edge (INV-ROUTE-2). A final oracle run guards the pass: if the result is worse than
//! the input the input is returned unchanged.

use crate::straighten::{Chain, Dynamic, Statics, covers, moved_board, quality, rebuild, unit};
use ferroute_board::{Board, BoardEdit};
use ferroute_drc::quality::avoidable_pad_bend;
use ferroute_geom::{Coord, Point};

const MAX_PASSES: usize = 4;

/// Extent of an octilinear vector in lattice steps (`None` if it is not octilinear).
fn extent(v: Point) -> Option<Coord> {
    v.is_octilinear().then(|| v.x.abs().max(v.y.abs()))
}

impl Statics<'_> {
    /// Is `t` inside the copper of a same-net pad centred on `centre`?
    fn inside_pad_at(&self, chain: &Chain, centre: Point, t: Point) -> bool {
        self.pads_at.get(&centre).is_some_and(|ps| {
            ps.iter().any(|&id| {
                let pad = self.base.pad(id);
                pad.net == chain.net
                    && pad.layers.contains(chain.layer)
                    && covers(&self.base.pad_hull(id), t)
            })
        })
    }

    /// Tries to chamfer the corner `ch.pts[i]`; returns the replacement polyline for the
    /// vertices `i-1..=i+1` on success.
    fn chamfer_corner(&self, dynamic: &Dynamic, ch: &Chain, i: usize) -> Option<Vec<Point>> {
        let (p, k, q) = (ch.pts[i - 1], ch.pts[i], ch.pts[i + 1]);
        let (vin, vout) = (k - p, q - k);
        if vin.dot(vout) != 0 || vin.cross(vout) == 0 {
            return None;
        }
        let (lin, lout) = (extent(vin)?, extent(vout)?);
        let (din, dout) = (unit(vin), unit(vout));
        // A leg shared with another vertex may give up half its length (that neighbour
        // might chamfer too); a leg ending at the chain's end (pad, via) keeps one width.
        let w = ch.template.width.max(1);
        // Only a neighbouring 90° corner (which chamfers too) takes half of the shared leg.
        let corner = |a: Point, b: Point, c: Point| (b - a).dot(c - b) == 0;
        // A leg ending at another (non-90°) bend may vanish entirely: the chamfer then
        // merges into the next leg. A leg ending at the chain's end keeps one width.
        let cap = |len: Coord, shared: bool, end: bool| {
            if shared {
                len / 2
            } else if end {
                len - w
            } else {
                len
            }
        };
        let key = (ch.net, ch.layer, w);
        let shared_in = i >= 2 && corner(ch.pts[i - 2], p, k);
        let shared_out = i + 2 < ch.pts.len() && corner(k, q, ch.pts[i + 2]);
        let mut c = cap(lin, shared_in, i == 1).min(cap(lout, shared_out, i + 2 == ch.pts.len()));
        let min_c = w / 4;
        while c >= min_c {
            let t1 = Point::new(k.x - din.0 * c, k.y - din.1 * c);
            let t2 = Point::new(k.x + dout.0 * c, k.y + dout.1 * c);
            let near = |a, b, c| u32::from(avoidable_pad_bend(self.base, key, (a, b, c)));
            // Never trade a corner for more avoidable bends next to a pad.
            let mut cand = vec![p, t1, t2, q];
            cand.dedup();
            if near(p, t1, t2) + near(t1, t2, q) <= near(p, k, q)
                && self.slivers(dynamic, ch, i - 1, i + 1, &cand)
                    <= self.slivers(dynamic, ch, i - 1, i + 1, &[p, k, q])
                && !self.inside_pad_at(ch, p, t1)
                && !self.inside_pad_at(ch, q, t2)
                && self.admissible(dynamic, ch, i - 1, i + 1, &cand)
            {
                return Some(cand);
            }
            c /= 2;
        }
        None
    }
}

/// Merges collinear runs so a corner's legs reach the next real bend.
fn merge_collinear(dynamic: &mut Dynamic, ch: &mut Chain) {
    let mut i = 1;
    while i + 1 < ch.pts.len() {
        let (p, k, q) = (ch.pts[i - 1], ch.pts[i], ch.pts[i + 1]);
        let (u, v) = (k - p, q - k);
        if u.cross(v) == 0 && u.dot(v) > 0 {
            ch.splice(dynamic, i - 1, i + 1, vec![p, q]);
        } else {
            i += 1;
        }
    }
}

/// Replaces every 90° bend of routed tracks by a 45° chamfer when exactly legal.
/// `base` is the board BEFORE `edit`; only `new_tracks` of the result differ.
#[must_use]
pub fn chamfer(base: &Board, edit: &BoardEdit) -> BoardEdit {
    let moved = moved_board(base, edit);
    let st = Statics::new(moved.as_ref(), edit);
    let mut dynamic = Dynamic::of(&edit.new_tracks);
    let mut chains = st.chains(&edit.new_tracks);
    for _ in 0..MAX_PASSES {
        let mut changed = false;
        for ch in &mut chains {
            merge_collinear(&mut dynamic, ch);
        }
        for ch in &mut chains {
            let mut i = 1;
            while i + 1 < ch.pts.len() {
                if let Some(cand) = st.chamfer_corner(&dynamic, ch, i) {
                    ch.splice(&mut dynamic, i - 1, i + 1, cand);
                    changed = true;
                    i += 1;
                } else {
                    i += 1;
                }
            }
        }
        if !changed {
            break;
        }
    }
    for ch in &mut chains {
        merge_collinear(&mut dynamic, ch);
    }
    if !chains.iter().any(|c| c.changed) {
        return edit.clone();
    }
    let out = rebuild(edit, &chains);
    let (before, after) = (quality(base, edit), quality(base, &out));
    if after.0 > before.0 || after.1 > before.1 || after.2 > before.2 {
        tracing::error!(?before, ?after, "chamfer made the result worse; keeping the input");
        return edit.clone();
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use ferroute_board::builder::{BoardBuilder, mm};
    use ferroute_board::{NetId, Origin, Track};
    use ferroute_drc::quality::{acute_angles, right_angles};
    use ferroute_drc::{DrcConfig, check};

    const W: i64 = 250_000;

    #[allow(clippy::cast_possible_truncation)]
    fn p(x: f64, y: f64) -> Point {
        Point::new((x * 1e6).round() as i64, (y * 1e6).round() as i64)
    }

    fn trk(net: NetId, pts: &[Point]) -> Vec<Track> {
        pts.windows(2)
            .map(|w| Track {
                start: w[0],
                end: w[1],
                width: W,
                layer: ferroute_board::LayerId(0),
                net,
                locked: false,
                origin: Origin::Routed,
            })
            .collect()
    }

    fn applied(base: &Board, edit: &BoardEdit) -> Board {
        let mut b = base.clone();
        b.apply(edit);
        b
    }

    fn assert_clean(base: &Board, edit: &BoardEdit) {
        let after = applied(base, edit);
        let rep = check(&after, &DrcConfig { courtyards: false, ..DrcConfig::default() });
        let intro: Vec<_> = rep.introduced().collect();
        assert!(intro.is_empty(), "introduced violations: {intro:#?}");
        assert!(acute_angles(&after).is_empty(), "acid traps");
    }

    fn length(tracks: &[Track]) -> f64 {
        tracks.iter().map(|t| t.start.dist(t.end)).sum()
    }

    fn open_board() -> (Board, NetId) {
        let mut b = BoardBuilder::new(mm(30), mm(20), 2);
        let n = b.net("N");
        (b.build(), n)
    }

    #[test]
    fn l_corner_becomes_two_45_degree_bends() {
        let (base, n) = open_board();
        let edit = BoardEdit {
            new_tracks: trk(n, &[p(5.0, 5.0), p(15.0, 5.0), p(15.0, 12.0)]),
            ..BoardEdit::default()
        };
        assert_eq!(right_angles(&applied(&base, &edit)).len(), 1);
        let out = chamfer(&base, &edit);
        assert_eq!(out.new_tracks.len(), 3);
        assert!(right_angles(&applied(&base, &out)).is_empty());
        assert!(length(&out.new_tracks) < length(&edit.new_tracks));
        assert_clean(&base, &out);
        assert_eq!(chamfer(&base, &edit).new_tracks, out.new_tracks, "deterministic");
    }

    #[test]
    fn diagonal_to_diagonal_corner_is_chamfered() {
        let (base, n) = open_board();
        let edit = BoardEdit {
            new_tracks: trk(n, &[p(5.0, 5.0), p(10.0, 10.0), p(15.0, 5.0)]),
            ..BoardEdit::default()
        };
        let out = chamfer(&base, &edit);
        assert_eq!(out.new_tracks.len(), 3);
        assert!(right_angles(&applied(&base, &out)).is_empty());
        assert_clean(&base, &out);
    }

    #[test]
    fn t_junction_is_untouched() {
        let (base, n) = open_board();
        let mut t = trk(n, &[p(5.0, 5.0), p(15.0, 5.0), p(15.0, 12.0)]);
        t.extend(trk(n, &[p(15.0, 5.0), p(25.0, 5.0)]));
        let edit = BoardEdit { new_tracks: t, ..BoardEdit::default() };
        assert_eq!(chamfer(&base, &edit).new_tracks, edit.new_tracks);
    }

    #[test]
    fn blocked_corner_uses_smaller_chamfer_or_stays_clean() {
        let mut b = BoardBuilder::new(mm(30), mm(20), 2);
        let n = b.net("N");
        let m = b.net("M");
        // Foreign pad hugging the inside of the corner: a large chamfer would hit it.
        b.smd2("R9", p(13.2, 7.0), 0.0, m, m);
        let base = b.build();
        let edit = BoardEdit {
            new_tracks: trk(n, &[p(5.0, 5.0), p(15.0, 5.0), p(15.0, 12.0)]),
            ..BoardEdit::default()
        };
        assert_clean(&base, &edit);
        let out = chamfer(&base, &edit);
        assert_clean(&base, &out);
    }

    #[test]
    fn corner_next_to_pad_keeps_axial_pad_crossing() {
        let mut b = BoardBuilder::new(mm(30), mm(20), 2);
        let n = b.net("N");
        b.smd2("R1", p(4.2, 5.13), 180.0, n, NetId::NONE);
        let base = b.build();
        let a = p(5.0, 5.13);
        let edit = BoardEdit {
            new_tracks: trk(n, &[a, p(7.0, 5.13), p(7.0, 12.0)]),
            ..BoardEdit::default()
        };
        let out = chamfer(&base, &edit);
        assert_clean(&base, &out);
        let first = out.new_tracks.iter().find(|t| t.start == a || t.end == a).unwrap();
        let far = if first.start == a { first.end } else { first.start };
        assert_eq!(far.y, a.y, "axial approach");
        let pad = base.pad_ids().next().unwrap();
        assert!(!covers(&base.pad_hull(pad), far), "first segment must cross the pad edge");
    }
}
