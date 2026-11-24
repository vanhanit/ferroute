// SPDX-License-Identifier: AGPL-3.0-only
//! Plane terminals: lattice sites from which a net reaches its own copper pour.
//!
//! A site qualifies when a via of the net's class is legal there and the via's copper disk
//! lies **entirely** inside one stored fill polygon of the net (exact containment, see
//! `Polygon::contains_disk`). A route that reaches such a site on the pour's layer is
//! connected to the pour (the via, or the track ending inside the fill, touches it).

use crate::access::Terminal;
use crate::grid::{Grid, NodeIdx, forward_slot};
use crate::legality::{ClassGeom, ClassLegality, allows};
use ferroute_board::{LayerId, NetId};
use ferroute_drc::zones::FillPoly;
use ferroute_geom::{Dir8, Point, Rect};

/// Lattice steps around a pad searched for plane sites (covers the BGA interstices).
pub const WINDOW: i64 = 8;

/// Plane terminals of `net` within [`WINDOW`] lattice steps of any of `centres`, in
/// deterministic (layer, row, column) order. Only fills on layers of the lattice count.
#[must_use]
pub fn plane_terminals(
    grid: &Grid,
    legal: &ClassLegality,
    geom: &ClassGeom,
    net: NetId,
    fills: &[FillPoly<'_>],
    centres: &[Point],
) -> Vec<Terminal> {
    let mine: Vec<(u32, &FillPoly<'_>)> = fills
        .iter()
        .filter(|f| f.net == net)
        .filter_map(|f| layer_index(grid, f.layer).map(|l| (l, f)))
        .collect();
    if mine.is_empty() {
        return Vec::new();
    }
    let pitch = grid.pitch;
    let mut sites: Vec<(u32, u32)> = Vec::new();
    for c in centres {
        let ci = (c.x - grid.origin.x).div_euclid(pitch);
        let cj = (c.y - grid.origin.y).div_euclid(pitch);
        for j in (cj - WINDOW).max(0)..=(cj + WINDOW).min(i64::from(grid.ny) - 1) {
            for i in (ci - WINDOW).max(0)..=(ci + WINDOW).min(i64::from(grid.nx) - 1) {
                sites.push((u32::try_from(i).unwrap_or(0), u32::try_from(j).unwrap_or(0)));
            }
        }
    }
    sites.sort_unstable_by_key(|&(i, j)| (j, i));
    sites.dedup();
    let mut layers: Vec<u32> = mine.iter().map(|e| e.0).collect();
    layers.sort_unstable();
    layers.dedup();
    let mut per_layer: Vec<Vec<Terminal>> = vec![Vec::new(); layers.len()];
    for &(i, j) in &sites {
        if !allows(legal.via[j as usize * grid.nx as usize + i as usize], net) {
            continue;
        }
        let p = grid.pos_ij(i, j);
        let disk = Rect::new(p, p).inflate(geom.via_radius + 2);
        for (k, &l) in layers.iter().enumerate() {
            let n = grid.idx(i, j, l);
            if !allows(legal.node[n as usize], net) {
                continue;
            }
            let inside = mine.iter().any(|(fl, f)| {
                *fl == l && f.bbox.intersects(disk) && f.poly.contains_disk(p, geom.via_radius)
            });
            if inside {
                per_layer[k].push(Terminal { node: n, stub: None, heading: None });
            }
        }
    }
    per_layer.into_iter().flatten().collect()
}

fn layer_index(grid: &Grid, l: LayerId) -> Option<u32> {
    grid.layers.iter().position(|&x| x == l).and_then(|p| u32::try_from(p).ok())
}

/// `true` when some terminal in `to` can be reached from a node in `from` over legal lattice
/// nodes, edges and via sites alone (other nets' routes ignored): the pad has a dog-bone
/// at all. Flood fill inside the window of [`WINDOW`] steps (plus a margin) around `from`.
#[must_use]
pub fn reaches(
    grid: &Grid,
    legal: &ClassLegality,
    net: NetId,
    from: &[NodeIdx],
    to: &[Terminal],
) -> bool {
    use rustc_hash::FxHashSet;
    let goal: FxHashSet<NodeIdx> = to.iter().map(|t| t.node).collect();
    let (mut i0, mut j0, mut i1, mut j1) = (u32::MAX, u32::MAX, 0, 0);
    for &n in from {
        let (i, j, _) = grid.coords(n);
        i0 = i0.min(i);
        j0 = j0.min(j);
        i1 = i1.max(i);
        j1 = j1.max(j);
    }
    let m = u32::try_from(WINDOW).unwrap_or(8) + 4;
    let (i0, j0) = (i0.saturating_sub(m), j0.saturating_sub(m));
    let (i1, j1) = ((i1 + m).min(grid.nx - 1), (j1 + m).min(grid.ny - 1));
    let mut seen: FxHashSet<NodeIdx> = from.iter().copied().collect();
    let mut stack: Vec<NodeIdx> = from.to_vec();
    let edge_ok = |a: NodeIdx, b: NodeIdx, d: Dir8| {
        let st = match forward_slot(d) {
            Some(s) => legal.edge_state(a, s),
            None => match forward_slot(d.opposite()) {
                Some(s) => legal.edge_state(b, s),
                None => return false,
            },
        };
        allows(st, net)
    };
    while let Some(n) = stack.pop() {
        if goal.contains(&n) {
            return true;
        }
        let (i, j, l) = grid.coords(n);
        let mut next: Vec<NodeIdx> = Vec::with_capacity(12);
        for d in Dir8::ALL {
            let Some(q) = grid.step(n, d) else { continue };
            let (qi, qj, _) = grid.coords(q);
            if qi < i0 || qi > i1 || qj < j0 || qj > j1 {
                continue;
            }
            if allows(legal.node[q as usize], net) && edge_ok(n, q, d) {
                next.push(q);
            }
        }
        if allows(legal.via[j as usize * grid.nx as usize + i as usize], net) {
            for ll in (0..grid.nl()).filter(|&x| x != l) {
                let q = grid.idx(i, j, ll);
                if allows(legal.node[q as usize], net) {
                    next.push(q);
                }
            }
        }
        for q in next {
            if seen.insert(q) {
                stack.push(q);
            }
        }
    }
    false
}
