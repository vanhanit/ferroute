// SPDX-License-Identifier: AGPL-3.0-only
//! Length tuning: lengthens routed nets with accordion meanders ([`crate::meander`]) so
//! `length_min` and length-matching (`match_group` + `skew_max`) rules are *met*.
//!
//! Targets: a net below `length_min` is raised to it; within a matching group, every net
//! shorter than `longest − skew_max` is raised to `longest − skew_max / 2`. No target ever
//! exceeds the net's `length_max`. Only copper ferroute routed in this run is touched,
//! widths never change (INV-RULE-1), and every meander is accepted only if the exact oracle
//! finds no introduced violation, connectivity does not drop (INV-CONN-1), no acid trap
//! appears (INV-ROUTE-3) and the bumps keep the net's clearance to its own other copper. Every
//! meander corner is a true circular arc (`ArcTrack`), never a mitre.
//! Rules still unmet afterwards are reported by `ferroute_drc::rules`.

use crate::meander::{MeanderSpec, Piece, pieces_length, run_frame};
use ferroute_board::{ArcTrack, Board, BoardEdit, NetId, Origin, Track};
use ferroute_drc::rules::net_lengths;
use ferroute_drc::{DrcConfig, quality};
use ferroute_geom::{Coord, Hull, units::nm_to_mm};
use std::collections::BTreeMap;

/// Length targets (nm) of nets that must grow, in net order.
#[must_use]
pub fn length_targets(board: &Board) -> BTreeMap<NetId, Coord> {
    let lengths = net_lengths(board);
    let mut want: BTreeMap<NetId, Coord> = BTreeMap::new();
    let mut groups: BTreeMap<&str, (Coord, Vec<usize>)> = BTreeMap::new();
    for (i, net) in board.nets.iter().enumerate().skip(1) {
        let id = NetId(u32::try_from(i).unwrap_or(u32::MAX));
        let c = &net.constraints;
        if let Some(min) = c.length_min
            && lengths[i] < min
        {
            // A little over the limit so float summation can never land just below it.
            want.insert(id, min + 2_000);
        }
        if let (Some(g), Some(skew)) = (c.match_group.as_deref(), c.skew_max) {
            let e = groups.entry(g).or_insert((skew, Vec::new()));
            e.0 = e.0.min(skew);
            e.1.push(i);
        }
    }
    for (skew, nets) in groups.into_values() {
        let longest = nets.iter().map(|&i| lengths[i]).max().unwrap_or(0);
        for i in nets {
            if lengths[i] < longest - skew {
                let id = NetId(u32::try_from(i).unwrap_or(u32::MAX));
                let t = want.entry(id).or_insert(0);
                *t = (*t).max(longest - skew / 2);
            }
        }
    }
    want.retain(|&id, t| {
        if let Some(max) = board.nets[id.idx()].constraints.length_max {
            *t = (*t).min(max);
        }
        *t > lengths[id.idx()] + 1_000
    });
    want
}

fn after(base: &Board, edit: &BoardEdit) -> Board {
    let mut b = base.clone();
    b.apply(edit);
    b
}

/// What a candidate must not make worse.
struct Gate {
    introduced: usize,
    missing: usize,
    acid: usize,
}

impl Gate {
    fn of(board: &Board) -> Self {
        let drc = ferroute_drc::check(board, &DrcConfig::default());
        Self {
            introduced: drc.introduced().count(),
            missing: drc.connectivity.connections_missing,
            acid: quality::acute_angles(board).len(),
        }
    }

    fn admits(&self, board: &Board) -> bool {
        let g = Self::of(board);
        g.introduced <= self.introduced && g.missing <= self.missing && g.acid <= self.acid
    }
}

/// `true` if any bump piece comes closer than the net's clearance to the net's *other*
/// copper (pads, vias, tracks and arcs not part of this meander) — no crowding or overlap.
fn crowds_own_copper(
    board: &Board,
    bump_tracks: &[Track],
    bump_arcs: &[ArcTrack],
    meander_tracks: &[Track],
    meander_arcs: &[ArcTrack],
) -> bool {
    let (net, layer) = match (bump_tracks.first(), bump_arcs.first()) {
        (Some(t), _) => (t.net, t.layer),
        (None, Some(a)) => (a.net, a.layer),
        (None, None) => return false,
    };
    let cl = board.rules.class_of(board, net).clearance;
    let mut others: Vec<Hull> = board
        .tracks
        .iter()
        .filter(|t| t.net == net && t.layer == layer && !meander_tracks.contains(t))
        .map(Track::hull)
        .collect();
    others.extend(
        board
            .arcs
            .iter()
            .filter(|t| t.net == net && t.layer == layer && !meander_arcs.contains(t))
            .flat_map(ArcTrack::hulls),
    );
    others.extend(
        board.vias.iter().filter(|v| v.net == net && v.layers.contains(layer)).map(|v| v.hull()),
    );
    others.extend(
        board
            .pad_ids()
            .filter(|&p| board.pad(p).net == net && board.pad(p).layers.contains(layer))
            .map(|p| board.pad_hull(p)),
    );
    let hulls =
        bump_tracks.iter().map(|t| vec![t.hull()]).chain(bump_arcs.iter().map(ArcTrack::hulls));
    hulls.into_iter().flatten().any(|h| others.iter().any(|o| h.clearance_violated(o, cl)))
}

/// Candidate runs of `net` in `edit`: routed, axis-aligned or 45°, longest (real length)
/// first, ties by index.
fn runs(edit: &BoardEdit, net: NetId) -> Vec<usize> {
    let len = |i: usize| {
        let t = &edit.new_tracks[i];
        run_frame(t.start, t.end).map(|(l, unit)| {
            #[allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]
            let real = (l as f64 * unit).round() as Coord;
            real
        })
    };
    let mut v: Vec<(Coord, usize)> = (0..edit.new_tracks.len())
        .filter(|&i| edit.new_tracks[i].net == net && edit.new_tracks[i].origin == Origin::Routed)
        .filter_map(|i| len(i).map(|l| (l, i)))
        .collect();
    v.sort_by_key(|&(l, i)| (std::cmp::Reverse(l), i));
    v.into_iter().map(|(_, i)| i).collect()
}

/// Tries to replace run `i` by a meander gaining up to `need`; returns the gain.
fn tune_run(base: &Board, edit: &mut BoardEdit, i: usize, need: f64, gate: &Gate) -> f64 {
    let seg = edit.new_tracks[i].clone();
    let current = after(base, edit);
    let cl = current.rules.class_of(&current, seg.net).clearance;
    let spacing = seg.width + cl.max(2 * seg.width);
    // Lowest amplitude that meets the whole need on this run first (a long, low accordion
    // looks hand-made); only then taller bumps for a partial gain on a short run.
    let amps = [spacing, spacing * 3 / 2, 2 * spacing, 3 * spacing];
    let full = amps.iter().map(|&a| (a, true));
    let partial = amps.iter().rev().map(|&a| (a, false));
    // Fillets keep the copper's inner edge at a positive radius.
    let min_radius = seg.width / 2 + 1;
    for (amp, must_be_full) in full.chain(partial) {
        for side in [1i8, -1] {
            let Some(spec) = MeanderSpec::plan_run(
                seg.start, seg.end, need, spacing, amp, spacing, min_radius, side,
            ) else {
                continue;
            };
            let Some(pieces) = spec.pieces(seg.start, seg.end) else { continue };
            let gain = pieces_length(&pieces) - seg.start.dist(seg.end);
            if must_be_full && gain < need - 1_000.0 {
                continue;
            }
            let mut meander: Vec<Track> = Vec::new();
            let mut meander_arcs: Vec<ArcTrack> = Vec::new();
            for p in &pieces {
                match *p {
                    Piece::Line(a, b) => meander.push(Track { start: a, end: b, ..seg.clone() }),
                    Piece::Arc(arc) => meander_arcs.push(ArcTrack {
                        start: arc.start,
                        mid: arc.mid,
                        end: arc.end,
                        width: seg.width,
                        layer: seg.layer,
                        net: seg.net,
                        locked: seg.locked,
                        origin: seg.origin,
                    }),
                }
            }
            let on_base = |t: &Track| {
                let d = seg.end - seg.start;
                let cross = |p: ferroute_geom::Point| {
                    let q = p - seg.start;
                    i128::from(q.x) * i128::from(d.y) - i128::from(q.y) * i128::from(d.x)
                };
                cross(t.start) == 0 && cross(t.end) == 0
            };
            let bumps: Vec<Track> = meander.iter().filter(|t| !on_base(t)).cloned().collect();
            let mut cand = edit.clone();
            cand.new_tracks.splice(i..=i, meander.iter().cloned());
            cand.new_arcs.extend(meander_arcs.iter().cloned());
            let board = after(base, &cand);
            if crowds_own_copper(&board, &bumps, &meander_arcs, &meander, &meander_arcs)
                || !gate.admits(&board)
            {
                continue;
            }
            *edit = cand;
            return gain;
        }
    }
    0.0
}

/// Lengthens nets in `edit` (relative to `base`) towards their length targets. Returns one
/// note per tuned net.
pub fn tune_lengths(base: &Board, edit: &mut BoardEdit) -> Vec<String> {
    let mut notes = Vec::new();
    let mut gained: BTreeMap<NetId, (f64, u32)> = BTreeMap::new();
    let start = net_lengths(&after(base, edit));
    // Two rounds: raising a net to its `length_min` can lengthen a group's longest member.
    for _ in 0..2 {
        let targets = length_targets(&after(base, edit));
        if targets.is_empty() {
            break;
        }
        let gate = Gate::of(&after(base, edit));
        for (net, target) in targets {
            let mut have = net_lengths(&after(base, edit))[net.idx()];
            let mut tried: Vec<(ferroute_geom::Point, ferroute_geom::Point)> = Vec::new();
            loop {
                #[allow(clippy::cast_precision_loss)]
                let need = (target - have) as f64;
                if need < 1_000.0 {
                    break;
                }
                let Some(i) = runs(edit, net).into_iter().find(|&i| {
                    let t = &edit.new_tracks[i];
                    !tried.contains(&(t.start, t.end))
                }) else {
                    break;
                };
                let t = &edit.new_tracks[i];
                tried.push((t.start, t.end));
                let g = tune_run(base, edit, i, need, &gate);
                if g > 0.0 {
                    let e = gained.entry(net).or_insert((0.0, 0));
                    e.0 += g;
                    e.1 += 1;
                    have = net_lengths(&after(base, edit))[net.idx()];
                }
            }
        }
    }
    let end = net_lengths(&after(base, edit));
    for (net, (_, meanders)) in gained {
        notes.push(format!(
            "net {}: length tuned {:.2} mm → {:.2} mm ({meanders} meander run(s))",
            base.nets[net.idx()].name,
            nm_to_mm(start[net.idx()]),
            nm_to_mm(end[net.idx()])
        ));
    }
    notes
}

#[cfg(test)]
mod tests {
    use super::*;
    use ferroute_board::builder::{BoardBuilder, mm, pt10};
    use ferroute_board::{LayerId, Track};
    use ferroute_drc::rules::{IssueKind, check_constraints};
    use ferroute_geom::{Point, Rect};

    /// Two nets routed as straight F.Cu runs between facing inner pads:
    /// P 28.4 mm long (y = 5 mm), N 18.4 mm (y = 12 mm).
    fn pair(skew: Coord) -> (Board, BoardEdit, NetId, NetId) {
        let mut b = BoardBuilder::new(mm(40), mm(20), 2);
        let p = b.net("D_P");
        let n = b.net("D_N");
        b.smd2("R1", pt10(50, 50), 0.0, NetId::NONE, p);
        b.smd2("R2", pt10(350, 50), 0.0, p, NetId::NONE);
        b.smd2("R3", pt10(50, 120), 0.0, NetId::NONE, n);
        b.smd2("R4", pt10(250, 120), 0.0, n, NetId::NONE);
        let mut board = b.build();
        for id in [p, n] {
            let c = &mut board.nets[id.idx()].constraints;
            c.match_group = Some("pair".into());
            c.skew_max = Some(skew);
        }
        let track = |a: Point, z: Point, net| Track {
            start: a,
            end: z,
            width: 250_000,
            layer: LayerId(0),
            net,
            locked: false,
            origin: Origin::Routed,
        };
        let edit = BoardEdit {
            new_tracks: vec![
                track(pt10(58, 50), pt10(342, 50), p),
                track(pt10(58, 120), pt10(242, 120), n),
            ],
            ..BoardEdit::default()
        };
        (board, edit, p, n)
    }

    /// Every corner of the net's copper is rounded: no two tracks meet at a shared endpoint
    /// unless they are collinear, and the net's arcs and tracks all have the given width.
    fn assert_rounded_corners(out: &Board, net: NetId, width: Coord) {
        let ts: Vec<&Track> = out.tracks.iter().filter(|t| t.net == net).collect();
        for (i, a) in ts.iter().enumerate() {
            for b in &ts[i + 1..] {
                for (p, q) in
                    [(a.start, b.start), (a.start, b.end), (a.end, b.start), (a.end, b.end)]
                {
                    if p != q {
                        continue;
                    }
                    let (u, v) = (a.end - a.start, b.end - b.start);
                    let cross =
                        i128::from(u.x) * i128::from(v.y) - i128::from(u.y) * i128::from(v.x);
                    assert_eq!(cross, 0, "mitred corner at {p:?}");
                }
            }
        }
        assert!(ts.iter().all(|t| t.width == width));
        assert!(out.arcs.iter().filter(|a| a.net == net).all(|a| a.width == width));
    }

    #[test]
    fn targets_follow_group_skew_min_and_max() {
        let (board, edit, p, n) = pair(mm(1));
        let after = after(&board, &edit);
        let t = length_targets(&after);
        assert_eq!(t.keys().copied().collect::<Vec<_>>(), vec![n]);
        assert_eq!(t[&n], mm(284) / 10 - mm(1) / 2, "longest − skew/2");
        let mut capped = after.clone();
        capped.nets[n.idx()].constraints.length_max = Some(mm(25));
        assert_eq!(length_targets(&capped)[&n], mm(25), "never beyond length_max");
        let mut min = after;
        min.nets[p.idx()].constraints.length_min = Some(mm(40));
        assert_eq!(length_targets(&min)[&p], mm(40) + 2_000);
    }

    #[test]
    fn meanders_match_a_pair_within_skew_and_stay_clean() {
        let (board, mut edit, p, n) = pair(mm(1) / 2);
        let before = net_lengths(&after(&board, &edit));
        let notes = tune_lengths(&board, &mut edit);
        assert_eq!(notes.len(), 1, "{notes:?}");
        let out = after(&board, &edit);
        let l = net_lengths(&out);
        assert_eq!(l[p.idx()], before[p.idx()], "the longest net is untouched");
        assert!(l[p.idx()] - l[n.idx()] <= mm(1) / 2, "spread {}", l[p.idx()] - l[n.idx()]);
        assert!(check_constraints(&out).is_empty(), "{:?}", check_constraints(&out));
        let drc = ferroute_drc::check(&out, &DrcConfig::default());
        assert_eq!(drc.introduced().count(), 0, "{:?}", drc.violations);
        assert_eq!(drc.connectivity.connections_missing, 0);
        assert!(quality::acute_angles(&out).is_empty());
        assert!(out.tracks.iter().all(|t| (t.end - t.start).is_octilinear()));
        assert!(out.tracks.iter().all(|t| t.width == 250_000), "never narrowed");
        assert!(out.arcs.iter().all(|t| t.width == 250_000), "arcs never narrowed");
        assert!(!edit.new_arcs.is_empty(), "meander corners are true arcs");
        assert!(edit.new_arcs.iter().all(|a| a.net == n && a.origin == Origin::Routed));
        assert_rounded_corners(&out, n, 250_000);
        // A long run gets a low accordion, not tall fingers: bumps stay within 1.5 spacings
        // (1.125 mm) of N's original line at y = 12 mm.
        let dev =
            out.tracks.iter().filter(|t| t.net == n).flat_map(|t| [t.start.y, t.end.y]).chain(
                out.arcs.iter().filter(|a| a.net == n).flat_map(|a| [a.start.y, a.mid.y, a.end.y]),
            );
        assert!(dev.map(|y| (y - mm(12)).abs()).max() <= Some(1_125_000));
    }

    #[test]
    fn diagonal_runs_get_diagonal_meanders() {
        let (mut board, mut edit, _, n) = pair(mm(1) / 2);
        // N becomes one 45° run R3.2 (5.8, 12) → (11.8, 18) with a 12 mm minimum length.
        for net in &mut board.nets {
            net.constraints = ferroute_board::constraints::NetConstraints::default();
        }
        board.nets[n.idx()].constraints.length_min = Some(mm(12));
        let n_track = &mut edit.new_tracks[1];
        n_track.end = n_track.start + Point::new(mm(6), mm(6));
        let end = n_track.end;
        board.footprints[3].placement.pos = end + Point::new(800_000, 0);
        board.footprints[3].original = board.footprints[3].placement;
        let notes = tune_lengths(&board, &mut edit);
        assert_eq!(notes.len(), 1, "{notes:?}");
        let out = after(&board, &edit);
        assert!(net_lengths(&out)[n.idx()] >= mm(12));
        let drc = ferroute_drc::check(&out, &DrcConfig::default());
        assert_eq!(drc.introduced().count(), 0, "{:?}", drc.violations);
        assert_eq!(drc.connectivity.connections_missing, 0);
        assert!(quality::acute_angles(&out).is_empty());
        let n_tracks: Vec<_> = out.tracks.iter().filter(|t| t.net == n).collect();
        assert!(n_tracks.iter().all(|t| (t.end - t.start).is_octilinear()));
        assert!(out.arcs.iter().filter(|a| a.net == n).count() > 5, "meandered with arcs");
        assert!(!edit.new_arcs.is_empty());
        assert_rounded_corners(&out, n, 250_000);
        assert!(check_constraints(&out).is_empty(), "{:?}", check_constraints(&out));
    }

    #[test]
    fn blocked_runs_are_left_alone_and_the_rule_stays_reported() {
        let (mut board, mut edit, _, n) = pair(mm(1) / 2);
        // Keep-outs hugging both sides of N's run: no room for any bump.
        for (y0, y1) in [(mm(10), 11_700_000), (12_300_000, mm(14))] {
            board.keepouts.push(ferroute_board::Keepout {
                area: ferroute_geom::Polygon::from_rect(Rect::new(
                    Point::new(mm(6), y0),
                    Point::new(mm(24), y1),
                )),
                layers: board.all_layers(),
                no_tracks: true,
                no_vias: true,
                no_pads: false,
                no_footprints: false,
            });
        }
        let original = edit.clone();
        assert!(tune_lengths(&board, &mut edit).is_empty());
        assert_eq!(edit, original);
        let issues = check_constraints(&after(&board, &edit));
        assert!(issues.iter().any(|i| matches!(i.kind, IssueKind::Skew { .. })), "{issues:?}");
        let _ = n;
    }
}
