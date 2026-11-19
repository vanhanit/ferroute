// SPDX-License-Identifier: AGPL-3.0-only
//! Lattice routes → board copper (tracks merged along straight runs, through vias, stubs).

use crate::access::Stub;
use crate::grid::Grid;
use crate::legality::ClassGeom;
use crate::negotiate::NetRoute;
use ferroute_board::{Board, NetId, Origin, Track, Via};
use ferroute_geom::Point;

/// Converts one net's lattice route into tracks and vias.
#[must_use]
pub fn emit(
    board: &Board,
    grid: &Grid,
    stubs: &[Stub],
    net: NetId,
    cg: &ClassGeom,
    r: &NetRoute,
) -> (Vec<Track>, Vec<Via>) {
    let mut tracks = Vec::new();
    let track = |a: Point, b: Point, layer| Track {
        start: a,
        end: b,
        width: cg.track_width,
        layer,
        net,
        locked: false,
        origin: Origin::Routed,
    };
    for path in &r.paths {
        let mut run_start = 0usize;
        for k in 1..=path.len() {
            let end_run = if k == path.len() {
                true
            } else {
                let (a, b) = (grid.coords(path[k - 1]), grid.coords(path[k]));
                let layer_change = a.2 != b.2;
                let dir_change = k >= run_start + 2 && {
                    let p = grid.coords(path[k - 2]);
                    let d0 = (i64::from(a.0) - i64::from(p.0), i64::from(a.1) - i64::from(p.1));
                    let d1 = (i64::from(b.0) - i64::from(a.0), i64::from(b.1) - i64::from(a.1));
                    d0 != d1
                };
                layer_change || dir_change
            };
            if end_run {
                let (s, e) = (path[run_start], path[k - 1]);
                if s != e {
                    let l = grid.layers[grid.coords(s).2 as usize];
                    tracks.push(track(grid.pos(s), grid.pos(e), l));
                }
                // A layer change starts the next run at the via's far side.
                run_start = k - 1;
                if k < path.len() && grid.coords(path[k - 1]).2 != grid.coords(path[k]).2 {
                    run_start = k;
                }
            }
        }
    }
    for &s in &r.stubs {
        let st = &stubs[s as usize];
        if st.from != st.to {
            tracks.push(track(st.from, st.to, st.layer));
        }
    }
    let layers = board.all_layers();
    let vias = r
        .vias
        .iter()
        .map(|&v| Via {
            pos: grid.pos_ij(v % grid.nx, v / grid.nx),
            diameter: cg.via_diameter,
            drill: cg.via_drill,
            layers,
            net,
            locked: false,
            origin: Origin::Routed,
        })
        .collect();
    (tracks, vias)
}
