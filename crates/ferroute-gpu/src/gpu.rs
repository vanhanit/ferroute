// SPDX-License-Identifier: AGPL-3.0-only
//! wgpu compute backend (feature `gpu`).
//!
//! # Error bound
//!
//! All shader arithmetic is f32. Let
//! `E = max(|rel. coord| of every obstacle endpoint, (nx-1)*pitch, (ny-1)*pitch)
//!      + cutoff + max_radius`
//! where "rel. coord" is relative to the grid origin (so every f32 quantity the
//! shader touches has magnitude at most about `2E`, and every intermediate is
//! a difference/sum of such quantities).
//! Each f32 operation introduces relative error `<= 2^-24` (`u`); the WebGPU
//! spec allows a few ulp for `sqrt`/`length`/division (<= 3 ulp is assumed).
//! Contributions, in units of `u * E`:
//!
//! * rounding the endpoints and radius to f32 on the host: 3 (node->point
//!   distance is 1-Lipschitz in each input position);
//! * node position `f32(i) * f32(pitch)`: 3 (two input roundings + product);
//! * `pa = p - a`, `ba = b - a`: 2 each, and the projection point `a + t*ba`
//!   moves by at most `(error of t) * |ba|`. Writing `t = (pa.ba)/(ba.ba)`,
//!   the abs error of the dot products is `~ 4u |pa||ba|`, giving a
//!   projection-point error of `~ 8u |pa| <= 16 u E`-ish; clamping to `[0, 1]`
//!   is non-expansive, so it never increases that error. Degenerate
//!   segments (`ba.ba == 0`) are handled explicitly (`t = 0`);
//! * `q = pa - t*ba`: 4, `length(q)`: 3 ulp, `- r` and `min` with the
//!   previous pass: 2, final readback is exact.
//!
//! A straightforward sum is well below `40 u E`; we publish
//! `error_bound_nm = 64 * E * 2^-24 + 1` (`+1 nm` absorbs `E` rounding and any
//! sub-nm effects), leaving a comfortable margin. For a 500 mm board
//! (`E ~ 5e8 nm`) this is about 2 um, versus typical 100 um clearances.
//! Saturation (`min` with the f32 cutoff) keeps true `d >= cutoff` inside
//! `[cutoff - bound, cutoff]`.
//!
//! Note: the bound is derived analytically; it is validated against the CPU
//! backend in `tests/gpu.rs` whenever an adapter is present (CI uses lavapipe).

use std::sync::mpsc;

use crate::{
    ClearanceBackend, ComputeError, F32_EPS, Field, FieldRequest, trivial_field, validate,
};

/// WGSL kernel. Obstacle `k` occupies `obs[2k]` = `(ax, ay, bx, by)` and
/// `obs[2k+1] = (radius, 0, 0, 0)`, all relative to the grid origin (nm, f32).
pub(crate) const SHADER: &str = r"
struct Params {
    nx: u32,
    rows: u32,
    row0: u32,
    count: u32,
    groups_x: u32,
    first: u32,
    pitch: f32,
    cutoff: f32,
};

@group(0) @binding(0) var<uniform> p: Params;
@group(0) @binding(1) var<storage, read> obs: array<vec4<f32>>;
@group(0) @binding(2) var<storage, read_write> out: array<f32>;

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let idx = gid.y * (p.groups_x * 64u) + gid.x;
    if (idx >= p.nx * p.rows) {
        return;
    }
    let i = idx % p.nx;
    let j = idx / p.nx + p.row0;
    let pt = vec2<f32>(f32(i), f32(j)) * p.pitch;
    var best = p.cutoff;
    if (p.first == 0u) {
        best = min(best, out[idx]);
    }
    for (var k = 0u; k < p.count; k = k + 1u) {
        let g = obs[2u * k];
        let r = obs[2u * k + 1u].x;
        let a = g.xy;
        let ba = g.zw - a;
        let pa = pt - a;
        let l2 = dot(ba, ba);
        var t = 0.0;
        if (l2 > 0.0) {
            t = clamp(dot(pa, ba) / l2, 0.0, 1.0);
        }
        let q = pa - t * ba;
        best = min(best, length(q) - r);
    }
    out[idx] = best;
}
";

const OBSTACLE_BYTES: u64 = 32;
/// Obstacles per dispatch (keeps single submissions short, avoiding driver
/// watchdog timeouts on huge problems).
const MAX_OBSTACLES_PER_PASS: u64 = 16_384;
const MAX_OUT_BYTES: u64 = 256 << 20;

/// wgpu-based backend. Device, queue and pipeline are created once.
#[derive(Debug)]
pub struct GpuBackend {
    device: wgpu::Device,
    queue: wgpu::Queue,
    pipeline: wgpu::ComputePipeline,
    limits: wgpu::Limits,
    info: String,
}

fn gpu_err(e: impl std::fmt::Display) -> ComputeError {
    ComputeError::Gpu(e.to_string())
}

fn request_adapter(instance: &wgpu::Instance) -> Result<wgpu::Adapter, ComputeError> {
    let mut last = String::new();
    for fallback in [false, true] {
        let opts =
            wgpu::RequestAdapterOptions { force_fallback_adapter: fallback, ..Default::default() };
        match pollster::block_on(instance.request_adapter(&opts)) {
            Ok(a) => return Ok(a),
            Err(e) => last = e.to_string(),
        }
    }
    Err(ComputeError::NoAdapter(last))
}

fn describe(adapter: &wgpu::Adapter) -> String {
    let i = adapter.get_info();
    format!("{} ({:?}, {:?})", i.name, i.device_type, i.backend)
}

/// Human-readable description of the adapter that [`GpuBackend::new`] would
/// use, or `None` if there is none.
pub fn adapter_info() -> Option<String> {
    let instance = wgpu::Instance::default();
    request_adapter(&instance).ok().map(|a| describe(&a))
}

impl GpuBackend {
    /// Create the device and compile the pipeline.
    pub fn new() -> Result<Self, ComputeError> {
        let instance = wgpu::Instance::default();
        let adapter = request_adapter(&instance)?;
        let info = describe(&adapter);
        let limits = adapter.limits();
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("ferroute-gpu"),
            required_limits: limits.clone(),
            ..Default::default()
        }))
        .map_err(gpu_err)?;

        let scope = device.push_error_scope(wgpu::ErrorFilter::Validation);
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("clearance"),
            source: wgpu::ShaderSource::Wgsl(SHADER.into()),
        });
        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("clearance"),
            layout: None,
            module: &module,
            entry_point: Some("main"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            cache: None,
        });
        if let Some(e) = pollster::block_on(scope.pop()) {
            return Err(gpu_err(e));
        }
        Ok(Self { device, queue, pipeline, limits, info })
    }

    /// Adapter description.
    #[must_use]
    pub fn info(&self) -> &str {
        &self.info
    }

    fn run(&self, req: &FieldRequest<'_>) -> Result<Vec<f32>, ComputeError> {
        let g = req.grid;
        let nx = u64::from(g.nx);
        let ny = u64::from(g.ny);
        let o = g.origin;
        #[allow(clippy::cast_precision_loss)]
        let f = |v: i64| v as f32;

        // Obstacle data, packed little-endian by hand (no unsafe / bytemuck).
        let mut bytes: Vec<u8> = Vec::with_capacity(req.obstacles.len() * 32);
        for c in req.obstacles {
            for v in [
                f(c.a[0] - o[0]),
                f(c.a[1] - o[1]),
                f(c.b[0] - o[0]),
                f(c.b[1] - o[1]),
                f(c.radius),
                0.0,
                0.0,
                0.0,
            ] {
                bytes.extend_from_slice(&v.to_le_bytes());
            }
        }

        let max_bind = self.limits.max_storage_buffer_binding_size.min(self.limits.max_buffer_size);
        let per_pass = (max_bind / OBSTACLE_BYTES).clamp(1, MAX_OBSTACLES_PER_PASS);
        let max_out = max_bind.min(MAX_OUT_BYTES);
        if nx * 4 > max_out {
            return Err(ComputeError::Gpu("grid row exceeds device buffer limits".into()));
        }
        let rows_per_band = (max_out / (nx * 4)).min(ny).max(1);
        let max_groups = u64::from(self.limits.max_compute_workgroups_per_dimension);

        let dev = &self.device;
        let layout = self.pipeline.get_bind_group_layout(0);

        // Obstacle chunk buffers (shared by every band).
        let chunk_bytes = (per_pass * OBSTACLE_BYTES) as usize;
        let chunks: Vec<(wgpu::Buffer, u32)> = bytes
            .chunks(chunk_bytes)
            .map(|c| {
                let buf = dev.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("obstacles"),
                    size: c.len() as u64,
                    usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });
                self.queue.write_buffer(&buf, 0, c);
                (buf, (c.len() as u64 / OBSTACLE_BYTES) as u32)
            })
            .collect();

        let mut values = Vec::with_capacity((nx * ny) as usize);
        let mut row0 = 0u64;
        while row0 < ny {
            let rows = rows_per_band.min(ny - row0);
            let n = nx * rows;
            let size = n * 4;
            let out = dev.create_buffer(&wgpu::BufferDescriptor {
                label: Some("out"),
                size,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            });
            let groups = n.div_ceil(64);
            let gx = groups.min(max_groups).max(1);
            let gy = groups.div_ceil(gx);
            if gy > max_groups {
                return Err(ComputeError::Gpu("dispatch too large for device".into()));
            }
            for (ci, (obs, count)) in chunks.iter().enumerate() {
                let mut pb = Vec::with_capacity(32);
                for u in [g.nx, rows as u32, row0 as u32, *count, gx as u32, u32::from(ci == 0)] {
                    pb.extend_from_slice(&u.to_le_bytes());
                }
                pb.extend_from_slice(&f(g.pitch).to_le_bytes());
                pb.extend_from_slice(&f(req.cutoff).to_le_bytes());
                let params = dev.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("params"),
                    size: pb.len() as u64,
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });
                self.queue.write_buffer(&params, 0, &pb);
                let bg = dev.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("clearance"),
                    layout: &layout,
                    entries: &[
                        wgpu::BindGroupEntry { binding: 0, resource: params.as_entire_binding() },
                        wgpu::BindGroupEntry { binding: 1, resource: obs.as_entire_binding() },
                        wgpu::BindGroupEntry { binding: 2, resource: out.as_entire_binding() },
                    ],
                });
                let mut enc = dev.create_command_encoder(&wgpu::CommandEncoderDescriptor {
                    label: Some("clearance"),
                });
                {
                    let mut pass = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
                        label: Some("clearance"),
                        timestamp_writes: None,
                    });
                    pass.set_pipeline(&self.pipeline);
                    pass.set_bind_group(0, &bg, &[]);
                    pass.dispatch_workgroups(gx as u32, gy as u32, 1);
                }
                self.queue.submit([enc.finish()]);
            }

            let readback = dev.create_buffer(&wgpu::BufferDescriptor {
                label: Some("readback"),
                size,
                usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            let mut enc = dev.create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("readback"),
            });
            enc.copy_buffer_to_buffer(&out, 0, &readback, 0, size);
            self.queue.submit([enc.finish()]);
            let (tx, rx) = mpsc::channel();
            readback.slice(..).map_async(wgpu::MapMode::Read, move |r| {
                let _ = tx.send(r);
            });
            dev.poll(wgpu::PollType::wait_indefinitely()).map_err(gpu_err)?;
            rx.recv().map_err(gpu_err)?.map_err(gpu_err)?;
            {
                let view = readback.slice(..).get_mapped_range().map_err(gpu_err)?;
                values.extend(view.as_chunks::<4>().0.iter().map(|b| f32::from_le_bytes(*b)));
            }
            readback.unmap();
            row0 += rows;
        }
        Ok(values)
    }
}

/// Documented analytic bound, see the module header.
pub(crate) fn gpu_error_bound(req: &FieldRequest<'_>) -> f64 {
    let g = &req.grid;
    let mut e: i64 = 0;
    for c in req.obstacles {
        for k in 0..2 {
            e = e.max((c.a[k] - g.origin[k]).abs()).max((c.b[k] - g.origin[k]).abs());
        }
    }
    let ext = g.pitch * i64::from(g.nx.max(g.ny).saturating_sub(1));
    let max_r = req.obstacles.iter().map(|c| c.radius).max().unwrap_or(0);
    #[allow(clippy::cast_precision_loss)]
    let e = (e.max(ext) + req.cutoff + max_r) as f64;
    64.0 * e * F32_EPS + 1.0
}

impl ClearanceBackend for GpuBackend {
    fn name(&self) -> &'static str {
        "gpu"
    }

    fn clearance_field(&self, req: &FieldRequest<'_>) -> Result<Field, ComputeError> {
        validate(req)?;
        if req.obstacles.is_empty() || req.grid.nx == 0 || req.grid.ny == 0 {
            return Ok(trivial_field(req, "gpu"));
        }
        // Error scopes are thread-local; everything below is synchronous.
        let oom = self.device.push_error_scope(wgpu::ErrorFilter::OutOfMemory);
        let val = self.device.push_error_scope(wgpu::ErrorFilter::Validation);
        let res = self.run(req);
        let verr = pollster::block_on(val.pop());
        let merr = pollster::block_on(oom.pop());
        if let Some(e) = verr.or(merr) {
            return Err(gpu_err(e));
        }
        Ok(Field { values: res?, error_bound_nm: gpu_error_bound(req), backend: "gpu" })
    }
}

#[cfg(test)]
mod tests {
    use super::SHADER;

    /// Validates the shader offline (no adapter needed).
    #[test]
    fn shader_is_valid_wgsl() {
        let module = naga::front::wgsl::parse_str(SHADER).expect("wgsl parse");
        naga::valid::Validator::new(
            naga::valid::ValidationFlags::all(),
            naga::valid::Capabilities::empty(),
        )
        .validate(&module)
        .expect("wgsl validate");
    }
}
