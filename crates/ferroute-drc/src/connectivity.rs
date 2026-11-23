// SPDX-License-Identifier: AGPL-3.0-only
//! Connectivity analysis: union-find over same-net copper that touches on a shared
//! layer (vias / through-hole pads join all their layers), then a minimum spanning
//! "ratsnest" between the islands that contain pads.
//!
//! INV-CONN-1 (connectivity conservation): ferroute only adds copper, so the set of
//! connected pad pairs after a run is a superset of the set before it.

use crate::{BucketGrid, CopperItem, ItemRef};
use ferroute_board::{Board, NetId};
use ferroute_geom::{Point, Rect};
use serde::{Deserialize, Serialize};

/// Two items are considered electrically joined when their copper is closer than this
/// (shapes that touch or overlap). 1 nm absorbs nothing but rounding of exact contacts.
pub const CONTACT_TOLERANCE: i64 = 1;

/// One missing connection (ratsnest line) of a net.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Unconnected {
    pub net: NetId,
    pub a: Point,
    pub b: Point,
}

/// Connectivity summary.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Connectivity {
    /// Nets with at least two pads.
    pub nets_total: usize,
    /// Of those, nets whose pads are all in one island.
    pub nets_complete: usize,
    /// Σ (pads − 1) over nets with ≥ 2 pads: connections needed for completion.
    pub connections_total: usize,
    /// Σ (islands − 1): connections still missing.
    pub connections_missing: usize,
    pub unconnected: Vec<Unconnected>,
}

impl Connectivity {
    /// Routing completion in `[0, 1]` (1.0 when nothing needs routing).
    #[must_use]
    pub fn completion(&self) -> f64 {
        if self.connections_total == 0 {
            return 1.0;
        }
        #[allow(clippy::cast_precision_loss)]
        let r = 1.0 - self.connections_missing as f64 / self.connections_total as f64;
        r
    }
}

/// Disjoint-set forest with path halving and union by size.
#[derive(Debug, Clone)]
pub struct UnionFind {
    parent: Vec<u32>,
    size: Vec<u32>,
}

impl UnionFind {
    #[must_use]
    pub fn new(n: usize) -> Self {
        Self { parent: (0..u32::try_from(n).unwrap_or(u32::MAX)).collect(), size: vec![1; n] }
    }

    pub fn find(&mut self, mut x: u32) -> u32 {
        while self.parent[x as usize] != x {
            let p = self.parent[x as usize];
            self.parent[x as usize] = self.parent[p as usize];
            x = self.parent[x as usize];
        }
        x
    }

    pub fn union(&mut self, a: u32, b: u32) -> bool {
        let (mut ra, mut rb) = (self.find(a), self.find(b));
        if ra == rb {
            return false;
        }
        if self.size[ra as usize] < self.size[rb as usize] {
            std::mem::swap(&mut ra, &mut rb);
        }
        self.parent[rb as usize] = ra;
        self.size[ra as usize] += self.size[rb as usize];
        true
    }
}

/// Computes connectivity from flattened copper items.
#[must_use]
pub fn analyse(board: &Board, items: &[CopperItem]) -> Connectivity {
    let boxes: Vec<Rect> = items.iter().map(|it| it.bbox.inflate(CONTACT_TOLERANCE)).collect();
    let grid = BucketGrid::new(&boxes, 2_000_000);
    let mut uf = UnionFind::new(items.len() + crate::zones::fill_count(board));
    for (i, j) in grid.pairs(&boxes) {
        let (a, b) = (&items[i as usize], &items[j as usize]);
        if a.net != b.net || a.net == NetId::NONE || !a.layers.intersects(b.layers) {
            continue;
        }
        if a.hull.clearance_violated(&b.hull, CONTACT_TOLERANCE) {
            uf.union(i, j);
        }
    }
    for (a, b) in crate::zones::fill_links(board, items) {
        uf.union(a, b);
    }
    // Pads of the same footprint with the same number are internally connected in
    // KiCad (e.g. thermal pads split into several pads). Join them.
    let mut by_fp_num: Vec<((u32, &str, NetId), u32)> = items
        .iter()
        .enumerate()
        .filter_map(|(k, it)| match it.item {
            ItemRef::Pad(p) => {
                let pad = board.pad(p);
                Some((
                    (pad.footprint.0, pad.number.as_str(), pad.net),
                    u32::try_from(k).unwrap_or(u32::MAX),
                ))
            }
            _ => None,
        })
        .collect();
    by_fp_num.sort_unstable();
    for w in by_fp_num.windows(2) {
        if w[0].0 == w[1].0 && w[0].0.2 != NetId::NONE && !w[0].0.1.is_empty() {
            uf.union(w[0].1, w[1].1);
        }
    }

    let mut out = Connectivity::default();
    let mut pads_by_net: Vec<Vec<u32>> = vec![Vec::new(); board.nets.len()];
    for (k, it) in items.iter().enumerate() {
        if matches!(it.item, ItemRef::Pad(_)) && it.net != NetId::NONE {
            pads_by_net[it.net.idx()].push(u32::try_from(k).unwrap_or(u32::MAX));
        }
    }
    for (net_idx, pads) in pads_by_net.iter().enumerate() {
        if pads.len() < 2 {
            continue;
        }
        out.nets_total += 1;
        out.connections_total += pads.len() - 1;
        // Islands: representative → member pad items.
        let mut islands: Vec<(u32, Vec<u32>)> = Vec::new();
        for &p in pads {
            let r = uf.find(p);
            match islands.iter_mut().find(|(root, _)| *root == r) {
                Some((_, v)) => v.push(p),
                None => islands.push((r, vec![p])),
            }
        }
        if islands.len() == 1 {
            out.nets_complete += 1;
            continue;
        }
        out.connections_missing += islands.len() - 1;
        let net = NetId(u32::try_from(net_idx).unwrap_or(u32::MAX));
        out.unconnected.extend(island_mst(items, &islands, net));
    }
    out
}

/// Prim's MST over islands with edge weight = closest pad-centre pair between islands.
fn island_mst(items: &[CopperItem], islands: &[(u32, Vec<u32>)], net: NetId) -> Vec<Unconnected> {
    let centre = |k: u32| items[k as usize].hull.bbox().center();
    let closest = |a: &[u32], b: &[u32]| {
        let mut best = (u128::MAX, Point::ORIGIN, Point::ORIGIN);
        for &x in a {
            for &y in b {
                let (px, py) = (centre(x), centre(y));
                let d = (px - py).norm2();
                if d < best.0 {
                    best = (d, px, py);
                }
            }
        }
        best
    };
    let n = islands.len();
    let mut in_tree = vec![false; n];
    in_tree[0] = true;
    let mut edges = Vec::with_capacity(n - 1);
    for _ in 1..n {
        let mut best: Option<(u128, usize, Point, Point)> = None;
        for i in (0..n).filter(|&i| in_tree[i]) {
            for j in (0..n).filter(|&j| !in_tree[j]) {
                let (d, a, b) = closest(&islands[i].1, &islands[j].1);
                if best.is_none_or(|(bd, ..)| d < bd) {
                    best = Some((d, j, a, b));
                }
            }
        }
        if let Some((_, j, a, b)) = best {
            in_tree[j] = true;
            edges.push(Unconnected { net, a, b });
        }
    }
    edges
}
