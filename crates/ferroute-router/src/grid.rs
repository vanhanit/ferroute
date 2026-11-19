// SPDX-License-Identifier: AGPL-3.0-only
//! The routing lattice: an `nx × ny` octilinear grid replicated on every routable layer.

use ferroute_board::LayerId;
use ferroute_geom::{Coord, Dir8, Point, Rect};

/// Node index into the flattened `(layer, j, i)` lattice.
pub type NodeIdx = u32;

/// The routing lattice.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Grid {
    /// Board position of node `(0, 0)`.
    pub origin: Point,
    pub pitch: Coord,
    pub nx: u32,
    pub ny: u32,
    /// Board copper layer of each grid layer (grid layer `g` ↔ `layers[g]`).
    pub layers: Vec<LayerId>,
}

impl Grid {
    /// Lattice covering `bbox` (inclusive) with the given pitch.
    #[must_use]
    pub fn covering(bbox: Rect, pitch: Coord, layers: Vec<LayerId>) -> Self {
        let pitch = pitch.max(1);
        let nx = u32::try_from(bbox.width() / pitch + 1).unwrap_or(u32::MAX);
        let ny = u32::try_from(bbox.height() / pitch + 1).unwrap_or(u32::MAX);
        Self { origin: bbox.min, pitch, nx, ny, layers }
    }

    #[must_use]
    pub fn nl(&self) -> u32 {
        u32::try_from(self.layers.len()).unwrap_or(u32::MAX)
    }

    /// Nodes per layer.
    #[must_use]
    pub fn plane(&self) -> usize {
        self.nx as usize * self.ny as usize
    }

    /// Total node count.
    #[must_use]
    pub fn len(&self) -> usize {
        self.plane() * self.layers.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    #[must_use]
    pub fn idx(&self, i: u32, j: u32, l: u32) -> NodeIdx {
        (l * self.ny + j) * self.nx + i
    }

    /// `(i, j, l)` of a node.
    #[must_use]
    pub fn coords(&self, n: NodeIdx) -> (u32, u32, u32) {
        let i = n % self.nx;
        let r = n / self.nx;
        (i, r % self.ny, r / self.ny)
    }

    /// Planar index `j * nx + i` of a node (layer dropped).
    #[must_use]
    pub fn planar(&self, n: NodeIdx) -> u32 {
        n % u32::try_from(self.plane()).unwrap_or(u32::MAX)
    }

    #[must_use]
    pub fn pos_ij(&self, i: u32, j: u32) -> Point {
        Point::new(
            self.origin.x + Coord::from(i) * self.pitch,
            self.origin.y + Coord::from(j) * self.pitch,
        )
    }

    #[must_use]
    pub fn pos(&self, n: NodeIdx) -> Point {
        let (i, j, _) = self.coords(n);
        self.pos_ij(i, j)
    }

    /// Neighbour in direction `d` on the same layer.
    #[must_use]
    pub fn step(&self, n: NodeIdx, d: Dir8) -> Option<NodeIdx> {
        let (i, j, l) = self.coords(n);
        let (di, dj) = d.delta();
        let ni = i64::from(i) + i64::from(di);
        let nj = i64::from(j) + i64::from(dj);
        if ni < 0 || nj < 0 || ni >= i64::from(self.nx) || nj >= i64::from(self.ny) {
            return None;
        }
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        Some(self.idx(ni as u32, nj as u32, l))
    }

    /// Same `(i, j)` on grid layer `l`.
    #[must_use]
    pub fn on_layer(&self, n: NodeIdx, l: u32) -> NodeIdx {
        let (i, j, _) = self.coords(n);
        self.idx(i, j, l)
    }

    /// Grid cell `(i, j)` (floor) containing `p`, clamped to the lattice.
    #[must_use]
    pub fn cell_floor(&self, p: Point) -> (i64, i64) {
        ((p.x - self.origin.x).div_euclid(self.pitch), (p.y - self.origin.y).div_euclid(self.pitch))
    }

    /// Grid layer of a board layer.
    #[must_use]
    pub fn grid_layer(&self, l: LayerId) -> Option<u32> {
        self.layers.iter().position(|&x| x == l).and_then(|g| u32::try_from(g).ok())
    }

    /// Inclusive `(i0, j0, i1, j1)` index range of nodes whose position lies in `r`.
    #[must_use]
    pub fn index_range(&self, r: Rect) -> Option<(u32, u32, u32, u32)> {
        let lo = |v: Coord, o: Coord| (v - o + self.pitch - 1).div_euclid(self.pitch);
        let hi = |v: Coord, o: Coord| (v - o).div_euclid(self.pitch);
        let i0 = lo(r.min.x, self.origin.x).max(0);
        let j0 = lo(r.min.y, self.origin.y).max(0);
        let i1 = hi(r.max.x, self.origin.x).min(i64::from(self.nx) - 1);
        let j1 = hi(r.max.y, self.origin.y).min(i64::from(self.ny) - 1);
        if i0 > i1 || j0 > j1 {
            return None;
        }
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        Some((i0 as u32, j0 as u32, i1 as u32, j1 as u32))
    }
}

/// Forward edge slot of direction `d` when stored at its *source* node, or `None` if the
/// edge must be looked up at the destination node with the opposite direction.
#[must_use]
pub const fn forward_slot(d: Dir8) -> Option<usize> {
    match d {
        Dir8::E => Some(0),
        Dir8::SE => Some(1),
        Dir8::S => Some(2),
        Dir8::SW => Some(3),
        _ => None,
    }
}

/// The four forward directions in slot order.
pub const FORWARD: [Dir8; 4] = [Dir8::E, Dir8::SE, Dir8::S, Dir8::SW];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn indexing_roundtrip() {
        let g = Grid::covering(
            Rect::new(Point::new(0, 0), Point::new(10_000, 5_000)),
            1_000,
            vec![LayerId(0), LayerId(1)],
        );
        assert_eq!((g.nx, g.ny), (11, 6));
        for n in 0..u32::try_from(g.len()).unwrap() {
            let (i, j, l) = g.coords(n);
            assert_eq!(g.idx(i, j, l), n);
        }
        let n = g.idx(0, 0, 1);
        assert_eq!(g.step(n, Dir8::W), None);
        assert_eq!(g.step(n, Dir8::SE), Some(g.idx(1, 1, 1)));
        assert_eq!(
            g.index_range(Rect::new(Point::new(500, 500), Point::new(2_500, 1_000))),
            Some((1, 1, 2, 1))
        );
    }
}
