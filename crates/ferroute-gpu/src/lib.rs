// SPDX-License-Identifier: AGPL-3.0-only
//! Tiered compute backends (wgpu GPU / rayon CPU) for ferroute clearance fields.
//!
//! Architectural rule: **accelerators propose, the exact oracle disposes.**
//! A backend returns an *approximate* clearance field together with a
//! guaranteed absolute error bound (`Field::error_bound_nm`). Callers treat
//! nodes with `|d - threshold| <= bound` as uncertain and re-check them with
//! exact integer predicates.
//!
//! # Semantics
//!
//! Node `(i, j)` sits at `(origin[0] + i*pitch, origin[1] + j*pitch)` (nm) and
//! its output slot is `j*nx + i`. For every node
//! `d = min over obstacles of (euclid_dist(node, segment) - radius)`,
//! saturated to `cutoff`: if the true `d >= cutoff` the backend may return any
//! value in `[cutoff - bound, cutoff]`. With no obstacles every value is
//! `cutoff as f32`.
//!
//! # Request limits
//!
//! `pitch > 0`, `nx*ny <= 2^26`, every obstacle coordinate, the grid origin and
//! the far grid corner within `+-2^30` nm, `0 <= radius <= 2^30`,
//! `0 < cutoff <= 2^40`, at most `u32::MAX` obstacles. Violations yield
//! [`ComputeError::Invalid`].

mod cpu;
#[cfg(feature = "gpu")]
mod gpu;

pub use cpu::CpuBackend;
#[cfg(feature = "gpu")]
pub use gpu::{GpuBackend, adapter_info};

/// Maximum number of grid nodes (`nx * ny`).
pub const MAX_NODES: u64 = 1 << 26;
/// Maximum absolute coordinate (and radius) in nanometres.
pub const MAX_COORD: i64 = 1 << 30;
/// Maximum cutoff in nanometres.
pub const MAX_CUTOFF: i64 = 1 << 40;

/// Node `(i, j)` sits at `(origin[0] + i*pitch, origin[1] + j*pitch)`, nanometres.
/// Output index is `j*nx + i`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GridSpec {
    pub origin: [i64; 2],
    pub pitch: i64,
    pub nx: u32,
    pub ny: u32,
}

/// Stadium/capsule obstacle: segment `a -> b` (`a == b` allowed, giving a disk)
/// inflated by `radius` (nm, `>= 0`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Capsule {
    pub a: [i64; 2],
    pub b: [i64; 2],
    pub radius: i64,
}

/// A clearance-field request.
#[derive(Debug, Clone)]
pub struct FieldRequest<'a> {
    pub grid: GridSpec,
    pub obstacles: &'a [Capsule],
    pub cutoff: i64,
}

/// Result of a clearance computation. See the crate docs for the semantics of
/// `values`; every value is within `error_bound_nm` of the exact (saturated)
/// answer.
#[derive(Debug, Clone)]
pub struct Field {
    pub values: Vec<f32>,
    pub error_bound_nm: f64,
    pub backend: &'static str,
}

/// Errors from compute backends.
#[derive(Debug, thiserror::Error)]
pub enum ComputeError {
    #[error("no GPU adapter: {0}")]
    NoAdapter(String),
    #[error("gpu: {0}")]
    Gpu(String),
    #[error("invalid request: {0}")]
    Invalid(String),
}

/// A backend able to compute clearance fields.
pub trait ClearanceBackend: Send + Sync + std::fmt::Debug {
    fn name(&self) -> &'static str;
    fn clearance_field(&self, req: &FieldRequest<'_>) -> Result<Field, ComputeError>;
}

/// Which backend [`select_backend`] should pick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BackendPreference {
    /// GPU if the `gpu` feature is on and an adapter is found, else CPU.
    Auto,
    Cpu,
    /// GPU or error.
    Gpu,
}

/// Choose a backend. Logs nothing; inspect [`ClearanceBackend::name`].
pub fn select_backend(pref: BackendPreference) -> Result<Box<dyn ClearanceBackend>, ComputeError> {
    match pref {
        BackendPreference::Cpu => Ok(Box::new(CpuBackend::new())),
        BackendPreference::Gpu => gpu_backend(),
        BackendPreference::Auto => {
            Ok(gpu_backend().unwrap_or_else(|_| Box::new(CpuBackend::new())))
        }
    }
}

#[cfg(feature = "gpu")]
fn gpu_backend() -> Result<Box<dyn ClearanceBackend>, ComputeError> {
    // Adapter discovery can in principle block on broken drivers; give it a
    // few seconds on a helper thread.
    use std::time::Duration;
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let _ = tx.send(GpuBackend::new());
    });
    match rx.recv_timeout(Duration::from_secs(5)) {
        Ok(Ok(b)) => Ok(Box::new(b)),
        Ok(Err(e)) => Err(e),
        Err(_) => Err(ComputeError::NoAdapter("timed out waiting for adapter".into())),
    }
}

#[cfg(not(feature = "gpu"))]
fn gpu_backend() -> Result<Box<dyn ClearanceBackend>, ComputeError> {
    Err(ComputeError::NoAdapter("ferroute-gpu built without the `gpu` feature".into()))
}

/// Validate a request against the documented limits.
pub(crate) fn validate(req: &FieldRequest<'_>) -> Result<(), ComputeError> {
    let bad = |m: String| Err(ComputeError::Invalid(m));
    let g = &req.grid;
    if g.pitch <= 0 {
        return bad(format!("pitch must be > 0, got {}", g.pitch));
    }
    if u64::from(g.nx) * u64::from(g.ny) > MAX_NODES {
        return bad(format!("nx*ny = {}*{} exceeds 2^26", g.nx, g.ny));
    }
    if req.cutoff <= 0 || req.cutoff > MAX_CUTOFF {
        return bad(format!("cutoff must be in (0, 2^40], got {}", req.cutoff));
    }
    if u32::try_from(req.obstacles.len()).is_err() {
        return bad("too many obstacles".into());
    }
    if g.pitch > MAX_COORD * 2 {
        return bad("pitch too large".into());
    }
    for (axis, n) in [(0usize, g.nx), (1, g.ny)] {
        let o = g.origin[axis];
        let far = i128::from(o) + i128::from(n.saturating_sub(1)) * i128::from(g.pitch);
        if o.abs() > MAX_COORD || far.abs() > i128::from(MAX_COORD) {
            return bad(format!("grid extent on axis {axis} exceeds +-2^30 nm"));
        }
    }
    for (k, c) in req.obstacles.iter().enumerate() {
        if c.radius < 0 || c.radius > MAX_COORD {
            return bad(format!("obstacle {k}: radius {} out of range", c.radius));
        }
        if c.a.iter().chain(c.b.iter()).any(|v| v.abs() > MAX_COORD) {
            return bad(format!("obstacle {k}: coordinate exceeds +-2^30 nm"));
        }
    }
    Ok(())
}

/// Result for the degenerate cases (no obstacles / empty grid).
pub(crate) fn trivial_field(req: &FieldRequest<'_>, backend: &'static str) -> Field {
    let n = req.grid.nx as usize * req.grid.ny as usize;
    #[allow(clippy::cast_precision_loss)]
    let cutoff = req.cutoff as f64;
    Field {
        values: vec![req.cutoff as f32; n],
        // f32 rounding of the cutoff only.
        error_bound_nm: cutoff * F32_EPS + 1.0,
        backend,
    }
}

/// Half-ulp relative rounding error of f32 round-to-nearest, `2^-24`.
pub(crate) const F32_EPS: f64 = 1.0 / 16_777_216.0;

/// Naive O(N*M) reference, saturated to `cutoff` (`min(d, cutoff)`).
///
/// Independent of the production formulas: the squared distance numerators and
/// the projection-parameter tests use exact `i128` arithmetic on absolute
/// coordinates; only the final division/sqrt is in f64 (relative error
/// about 1e-15, i.e. far below a nanometre for in-range inputs).
///
/// Does not validate; callers pass in-range requests.
pub fn exact_reference(req: &FieldRequest<'_>) -> Vec<f64> {
    let g = &req.grid;
    let mut out = Vec::with_capacity(g.nx as usize * g.ny as usize);
    #[allow(clippy::cast_precision_loss)]
    let cutoff = req.cutoff as f64;
    for j in 0..i64::from(g.ny) {
        for i in 0..i64::from(g.nx) {
            let p = [g.origin[0] + i * g.pitch, g.origin[1] + j * g.pitch];
            let mut best = cutoff;
            for c in req.obstacles {
                best = best.min(ref_dist(p, c));
            }
            out.push(best);
        }
    }
    out
}

#[allow(clippy::cast_precision_loss)]
fn ref_dist(p: [i64; 2], c: &Capsule) -> f64 {
    let w = [i128::from(p[0] - c.a[0]), i128::from(p[1] - c.a[1])];
    let d = [i128::from(c.b[0] - c.a[0]), i128::from(c.b[1] - c.a[1])];
    let len2 = d[0] * d[0] + d[1] * d[1];
    let dot = w[0] * d[0] + w[1] * d[1];
    let dist = if len2 == 0 || dot <= 0 {
        ((w[0] * w[0] + w[1] * w[1]) as f64).sqrt()
    } else if dot >= len2 {
        let v = [w[0] - d[0], w[1] - d[1]];
        ((v[0] * v[0] + v[1] * v[1]) as f64).sqrt()
    } else {
        let cross = w[0] * d[1] - w[1] * d[0];
        (cross.abs() as f64) / (len2 as f64).sqrt()
    };
    dist - c.radius as f64
}
