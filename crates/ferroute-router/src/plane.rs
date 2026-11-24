// SPDX-License-Identifier: AGPL-3.0-only
//! Plane terminals: lattice sites from which a net reaches its own copper pour.
//!
//! A site qualifies when a via of the net's class is legal there and the via's copper disk
//! lies **entirely** inside one stored fill polygon of the net (exact containment, see
//! `Polygon::contains_disk`). A route that reaches such a site on the pour's layer is
//! connected to the pour (the via, or the track ending inside the fill, touches it).

use crate::access::Terminal;
use crate::grid::Grid;
use crate::legality::{ClassGeom, ClassLegality, allows};
use ferroute_board::{LayerId, NetId};
use ferroute_drc::zones::FillPoly;
use ferroute_geom::{Point, Rect};

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
