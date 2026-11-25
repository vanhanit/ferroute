// SPDX-License-Identifier: AGPL-3.0-only
//! Pour-continuity costs (INV-CONN-1 upstream of the refill).
//!
//! Copper of other nets on a layer that carries a *yielding* pour (one the pipeline refills
//! around the routed copper) narrows the pour; enough of it (a long track, a wall of vias
//! closer than the pour's neck width) splits it into islands and the refill then
//! disconnects pour pads. Two soft costs steer the search away from that:
//!
//! * a base `track` cost on every node of a pour layer inside the pour's fill, paid by
//!   every net except the pour's own (inner planes are for planes, signals prefer the
//!   outer layers);
//! * `avoid` regions (from a diagnosed split, `ferroute_drc::refill::pour_splits`) that
//!   cost extra for tracks *and* for vias crossing the layer.
//!
//! Costs are multiplicative factors added to `1 + history` in the search; they never make a
//! node illegal, so legality and the exact oracle are unaffected.

use crate::grid::Grid;
use ferroute_board::{Board, LayerId, NetId};
use ferroute_geom::{Point, Rect};
use serde::{Deserialize, Serialize};

/// A region of a pour layer the router should avoid (soft).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PourAvoid {
    pub layer: LayerId,
    pub area: Rect,
    /// Extra cost factor per lattice step (tracks) or per via crossing (vias).
    pub cost: f32,
    /// The nets that pay (empty: every net except the pour's own).
    #[serde(default)]
    pub nets: Vec<NetId>,
}

/// An avoid region of specific nets in lattice indices.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct NetBox {
    pub layer: u32,
    pub i0: u32,
    pub j0: u32,
    pub i1: u32,
    pub j1: u32,
    pub cost: f32,
}

impl NetBox {
    fn covers(&self, l: u32, i: u32, j: u32) -> bool {
        self.layer == l && i >= self.i0 && i <= self.i1 && j >= self.j0 && j <= self.j1
    }
}

/// Per-node pour costs, indexed like the lattice.
#[derive(Clone, Debug, Default)]
pub struct PourCosts {
    /// Extra factor for a track node of a foreign net.
    pub track: Vec<f32>,
    /// Extra factor for a via crossing the node's layer (avoid regions only).
    pub via: Vec<f32>,
    /// Per lattice layer: the net of the pour on it (`NetId::NONE` for none / several).
    pub owner: Vec<NetId>,
    /// Net-specific avoid regions, sorted by net.
    pub by_net: Vec<(NetId, Vec<NetBox>)>,
}

/// The costs as seen by one net's searches.
#[derive(Copy, Clone, Debug)]
pub struct PourView<'a> {
    pub costs: &'a PourCosts,
    pub net: NetId,
    pub boxes: &'a [NetBox],
}

impl<'a> PourView<'a> {
    /// View of `costs` for `net`.
    #[must_use]
    pub fn new(costs: &'a PourCosts, net: NetId) -> Self {
        let boxes = costs
            .by_net
            .binary_search_by_key(&net, |e| e.0)
            .map_or(&[][..], |k| costs.by_net[k].1.as_slice());
        Self { costs, net, boxes }
    }

    fn boxed(&self, l: u32, i: u32, j: u32) -> f32 {
        self.boxes.iter().filter(|b| b.covers(l, i, j)).map(|b| b.cost).sum()
    }

    /// Track-node penalty at node `n` (layer `l`, column `i`, row `j`).
    #[inline]
    #[must_use]
    pub fn track_pen(&self, n: u32, l: u32, i: u32, j: u32) -> f32 {
        let own = self.costs.owner[l as usize] == self.net;
        let base = if own { 0.0 } else { self.costs.track[n as usize] };
        base + self.boxed(l, i, j)
    }

    /// Penalty for a via crossing layer `l` at `(i, j)` (node `n`).
    #[inline]
    #[must_use]
    pub fn via_pen(&self, n: u32, l: u32, i: u32, j: u32) -> f32 {
        let own = self.costs.owner[l as usize] == self.net;
        let base = if own { 0.0 } else { self.costs.via[n as usize] };
        base + self.boxed(l, i, j)
    }
}

/// Builds the costs; `None` when there is nothing to price (no yielding pour and no avoid
/// region), which keeps boards without refillable pours bit-identical.
#[must_use]
pub fn build(
    grid: &Grid,
    board: &Board,
    track_cost: f32,
    avoid: &[PourAvoid],
) -> Option<PourCosts> {
    let nl = grid.layers.len();
    let mut owner = vec![NetId::NONE; nl];
    let mut seen = vec![false; nl];
    let mut track = vec![0.0f32; grid.len()];
    let mut via = vec![0.0f32; grid.len()];
    let mut any = false;
    for z in board.zones.iter().filter(|z| z.yields && !z.is_unfilled()) {
        for f in &z.fills {
            let Some(l) = grid.layers.iter().position(|&x| x == f.layer) else { continue };
            let lu = u32::try_from(l).unwrap_or(0);
            owner[l] = if seen[l] && owner[l] != z.net { NetId::NONE } else { z.net };
            seen[l] = true;
            if track_cost <= 0.0 {
                continue;
            }
            for p in &f.polys {
                let Some(bb) = p.bbox() else { continue };
                let Some((i0, j0, i1, j1)) = grid.index_range(bb) else { continue };
                for j in j0..=j1 {
                    for i in i0..=i1 {
                        let pos: Point = grid.pos_ij(i, j);
                        if p.contains(pos) {
                            track[grid.idx(i, j, lu) as usize] = track_cost;
                            any = true;
                        }
                    }
                }
            }
        }
    }
    let mut by_net: Vec<(NetId, Vec<NetBox>)> = Vec::new();
    for a in avoid {
        let Some(l) = grid.layers.iter().position(|&x| x == a.layer) else { continue };
        let lu = u32::try_from(l).unwrap_or(0);
        let Some((i0, j0, i1, j1)) = grid.index_range(a.area) else { continue };
        if !a.nets.is_empty() {
            for &net in &a.nets {
                let b = NetBox { layer: lu, i0, j0, i1, j1, cost: a.cost };
                match by_net.binary_search_by_key(&net, |e| e.0) {
                    Ok(k) => by_net[k].1.push(b),
                    Err(k) => by_net.insert(k, (net, vec![b])),
                }
            }
            any = true;
            continue;
        }
        for j in j0..=j1 {
            for i in i0..=i1 {
                let n = grid.idx(i, j, lu) as usize;
                track[n] += a.cost;
                via[n] += a.cost;
                any = true;
            }
        }
    }
    any.then_some(PourCosts { track, via, owner, by_net })
}

#[cfg(test)]
mod tests {
    use super::*;
    use ferroute_board::{IslandRemoval, LayerSet, PadConnection, Zone, ZoneFill};
    use ferroute_geom::Polygon;

    fn grid() -> Grid {
        Grid::covering(
            Rect::new(Point::new(0, 0), Point::new(10_000, 10_000)),
            1_000,
            vec![LayerId(0), LayerId(1)],
        )
    }

    fn pour(net: NetId, layer: u8, yields: bool) -> Zone {
        let poly =
            Polygon::from_rect(Rect::new(Point::new(2_000, 2_000), Point::new(8_000, 8_000)));
        Zone {
            net,
            layers: LayerSet::single(LayerId(layer)),
            outline: poly.clone(),
            fills: vec![ZoneFill { layer: LayerId(layer), polys: vec![poly] }],
            clearance: 200,
            min_thickness: 200,
            priority: 0,
            pad_connection: PadConnection::ThermalRelief,
            islands: IslandRemoval::Always,
            yields,
        }
    }

    #[test]
    fn nothing_to_price_without_a_yielding_pour() {
        let mut b = Board::default();
        b.zones.push(pour(NetId(1), 1, false));
        assert!(build(&grid(), &b, 0.5, &[]).is_none());
    }

    #[test]
    fn foreign_nets_pay_on_the_pour_layer_only_inside_the_fill() {
        let g = grid();
        let mut b = Board::default();
        b.zones.push(pour(NetId(1), 1, true));
        let c = build(&g, &b, 0.5, &[]).unwrap();
        let inside = g.idx(5, 5, 1);
        let outside = g.idx(0, 0, 1);
        assert!((PourView::new(&c, NetId(2)).track_pen(inside, 1, 5, 5) - 0.5).abs() < 1e-6);
        assert_eq!(
            PourView::new(&c, NetId(1)).track_pen(inside, 1, 5, 5),
            0.0,
            "the pour's own net is exempt"
        );
        assert_eq!(PourView::new(&c, NetId(2)).track_pen(outside, 1, 0, 0), 0.0);
        assert_eq!(
            PourView::new(&c, NetId(2)).track_pen(g.idx(5, 5, 0), 0, 5, 5),
            0.0,
            "other layers are free"
        );
        assert_eq!(
            PourView::new(&c, NetId(2)).via_pen(inside, 1, 5, 5),
            0.0,
            "no avoid region: vias are not priced"
        );
    }

    #[test]
    fn avoid_regions_price_tracks_and_vias() {
        let g = grid();
        let mut b = Board::default();
        b.zones.push(pour(NetId(1), 1, true));
        let avoid = PourAvoid {
            layer: LayerId(1),
            area: Rect::new(Point::new(4_000, 4_000), Point::new(6_000, 6_000)),
            cost: 3.0,
            nets: Vec::new(),
        };
        let c = build(&g, &b, 0.5, &[avoid]).unwrap();
        let n = g.idx(5, 5, 1);
        assert!((PourView::new(&c, NetId(2)).track_pen(n, 1, 5, 5) - 3.5).abs() < 1e-6);
        assert!((PourView::new(&c, NetId(2)).via_pen(n, 1, 5, 5) - 3.0).abs() < 1e-6);
        assert_eq!(PourView::new(&c, NetId(1)).via_pen(n, 1, 5, 5), 0.0);
    }

    #[test]
    fn net_specific_regions_price_only_those_nets() {
        let g = grid();
        let mut b = Board::default();
        b.zones.push(pour(NetId(1), 1, true));
        let avoid = PourAvoid {
            layer: LayerId(1),
            area: Rect::new(Point::new(0, 0), Point::new(10_000, 10_000)),
            cost: 4.0,
            nets: vec![NetId(7)],
        };
        let c = build(&g, &b, 0.0, &[avoid]).unwrap();
        let n = g.idx(3, 3, 1);
        assert!((PourView::new(&c, NetId(7)).track_pen(n, 1, 3, 3) - 4.0).abs() < 1e-6);
        assert!((PourView::new(&c, NetId(7)).via_pen(n, 1, 3, 3) - 4.0).abs() < 1e-6);
        assert_eq!(PourView::new(&c, NetId(8)).track_pen(n, 1, 3, 3), 0.0);
        assert_eq!(PourView::new(&c, NetId(7)).track_pen(g.idx(3, 3, 0), 0, 3, 3), 0.0);
    }
}
