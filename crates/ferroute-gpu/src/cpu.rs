// SPDX-License-Identifier: AGPL-3.0-only
//! Deterministic rayon CPU backend.
//!
//! # Error bound
//!
//! Everything is evaluated in f64 on coordinates relative to the grid origin
//! (exact integers below 2^53). The f64 arithmetic error (a handful of
//! roundings of magnitude <= 2^32, each 2^-53 relative) is below 1e-5 nm, far
//! under the `+1 nm` slack. The only material error is the final
//! f64 -> f32 round-to-nearest, whose error is at most `|d| * 2^-24`
//! (half an ulp, relative `2^-24`). Any returned value has
//! `|d| <= D := max(cutoff, max_radius)` (distances are saturated at `cutoff`
//! and are `>= -radius`), so
//! `error_bound_nm = D * 2^-24 + 1`. Culled obstacles (those whose inflated
//! bounding box is further than `cutoff` from the node) have true distance
//! `>= cutoff`, which saturates to `cutoff`, consistent with the contract.

use rayon::prelude::*;

use crate::{
    ClearanceBackend, ComputeError, F32_EPS, Field, FieldRequest, trivial_field, validate,
};

/// Always-available CPU backend. Each node is computed independently, so the
/// output is bit-identical regardless of rayon thread count.
#[derive(Debug, Clone, Copy, Default)]
pub struct CpuBackend;

impl CpuBackend {
    #[must_use]
    pub fn new() -> Self {
        Self
    }
}

/// Obstacle precomputed in f64, relative to the grid origin.
#[derive(Clone, Copy)]
struct Prep {
    ax: f64,
    ay: f64,
    dx: f64,
    dy: f64,
    len2: f64,
    r: f64,
}

impl Prep {
    #[inline]
    fn dist(&self, px: f64, py: f64) -> f64 {
        let wx = px - self.ax;
        let wy = py - self.ay;
        let t = if self.len2 > 0.0 {
            ((wx * self.dx + wy * self.dy) / self.len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let qx = wx - t * self.dx;
        let qy = wy - t * self.dy;
        (qx * qx + qy * qy).sqrt() - self.r
    }
}

/// CSR bucket grid in node-index space.
struct Buckets {
    bs: usize,
    nbx: usize,
    start: Vec<u32>,
    items: Vec<u32>,
}

const MAX_BUCKETS: usize = 1 << 20;
const MAX_ENTRIES: u64 = 1 << 26;

fn ceil_div(a: i64, b: i64) -> i64 {
    -((-a).div_euclid(b))
}

/// Node-index rectangle `[ilo, ihi] x [jlo, jhi]` whose nodes may lie within
/// `cutoff` of the obstacle (clipped to the grid), if any.
fn node_rect(
    rel_a: [i64; 2],
    rel_b: [i64; 2],
    r: i64,
    cutoff: i64,
    pitch: i64,
    nx: u32,
    ny: u32,
) -> Option<[usize; 4]> {
    let m = r + cutoff;
    let mut out = [0usize; 4];
    for (axis, n) in [(0usize, nx), (1, ny)] {
        let lo = rel_a[axis].min(rel_b[axis]) - m;
        let hi = rel_a[axis].max(rel_b[axis]) + m;
        let ilo = ceil_div(lo, pitch).max(0);
        let ihi = hi.div_euclid(pitch).min(i64::from(n) - 1);
        if ilo > ihi {
            return None;
        }
        out[axis * 2] = usize::try_from(ilo).ok()?;
        out[axis * 2 + 1] = usize::try_from(ihi).ok()?;
    }
    Some(out)
}

impl Buckets {
    fn build(req: &FieldRequest<'_>, rects: &[Option<[usize; 4]>]) -> Self {
        let nx = req.grid.nx as usize;
        let ny = req.grid.ny as usize;
        let maxn = nx.max(ny);
        let want = usize::try_from(ceil_div(req.cutoff, req.grid.pitch)).unwrap_or(maxn);
        let mut bs = want.clamp(1, maxn);
        loop {
            let nbx = nx.div_ceil(bs);
            let nby = ny.div_ceil(bs);
            if nbx * nby > MAX_BUCKETS && bs < maxn {
                bs *= 2;
                continue;
            }
            let mut counts = vec![0u32; nbx * nby + 1];
            let mut total = 0u64;
            for r in rects.iter().flatten() {
                for by in r[2] / bs..=r[3] / bs {
                    for bx in r[0] / bs..=r[1] / bs {
                        counts[by * nbx + bx] += 1;
                        total += 1;
                    }
                }
            }
            if total > MAX_ENTRIES && bs < maxn {
                bs *= 2;
                continue;
            }
            // Exclusive prefix sum into `start`.
            let mut start = vec![0u32; nbx * nby + 1];
            let mut acc = 0u32;
            for (s, c) in start.iter_mut().zip(&counts) {
                *s = acc;
                acc += *c;
            }
            let mut cursor = start.clone();
            let mut items = vec![0u32; acc as usize];
            for (k, r) in rects.iter().enumerate() {
                let Some(r) = r else { continue };
                for by in r[2] / bs..=r[3] / bs {
                    for bx in r[0] / bs..=r[1] / bs {
                        let c = &mut cursor[by * nbx + bx];
                        items[*c as usize] = k as u32;
                        *c += 1;
                    }
                }
            }
            return Self { bs, nbx, start, items };
        }
    }

    #[inline]
    fn bucket(&self, b: usize) -> &[u32] {
        &self.items[self.start[b] as usize..self.start[b + 1] as usize]
    }
}

/// Rigorous error bound documented in the module header.
pub(crate) fn cpu_error_bound(req: &FieldRequest<'_>) -> f64 {
    let max_r = req.obstacles.iter().map(|c| c.radius).max().unwrap_or(0);
    #[allow(clippy::cast_precision_loss)]
    let d = req.cutoff.max(max_r) as f64;
    d * F32_EPS + 1.0
}

impl ClearanceBackend for CpuBackend {
    fn name(&self) -> &'static str {
        "cpu"
    }

    fn clearance_field(&self, req: &FieldRequest<'_>) -> Result<Field, ComputeError> {
        validate(req)?;
        let g = req.grid;
        let nx = g.nx as usize;
        let ny = g.ny as usize;
        if req.obstacles.is_empty() || nx == 0 || ny == 0 {
            return Ok(trivial_field(req, "cpu"));
        }
        let o = g.origin;
        let rel = |p: [i64; 2]| [p[0] - o[0], p[1] - o[1]];
        let rects: Vec<Option<[usize; 4]>> = req
            .obstacles
            .iter()
            .map(|c| node_rect(rel(c.a), rel(c.b), c.radius, req.cutoff, g.pitch, g.nx, g.ny))
            .collect();
        #[allow(clippy::cast_precision_loss)]
        let prep: Vec<Prep> = req
            .obstacles
            .iter()
            .map(|c| {
                let a = rel(c.a);
                let b = rel(c.b);
                let dx = (b[0] - a[0]) as f64;
                let dy = (b[1] - a[1]) as f64;
                Prep {
                    ax: a[0] as f64,
                    ay: a[1] as f64,
                    dx,
                    dy,
                    len2: dx * dx + dy * dy,
                    r: c.radius as f64,
                }
            })
            .collect();
        let buckets = Buckets::build(req, &rects);
        #[allow(clippy::cast_precision_loss)]
        let cutoff = req.cutoff as f64;
        let pitch = g.pitch;

        let mut values = vec![0f32; nx * ny];
        values.par_chunks_mut(nx).enumerate().for_each(|(j, row)| {
            let bj = j / buckets.bs;
            #[allow(clippy::cast_precision_loss)]
            let py = (j as i64 * pitch) as f64;
            for (i, out) in row.iter_mut().enumerate() {
                #[allow(clippy::cast_precision_loss)]
                let px = (i as i64 * pitch) as f64;
                let mut best = cutoff;
                for &k in buckets.bucket(bj * buckets.nbx + i / buckets.bs) {
                    best = best.min(prep[k as usize].dist(px, py));
                }
                *out = best as f32;
            }
        });
        Ok(Field { values, error_bound_nm: cpu_error_bound(req), backend: "cpu" })
    }
}
