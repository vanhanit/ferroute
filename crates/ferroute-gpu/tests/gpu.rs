// SPDX-License-Identifier: AGPL-3.0-only
#![cfg(feature = "gpu")]
mod common;

use common::{Case, case};
use ferroute_gpu::{
    Capsule, ClearanceBackend, ComputeError, CpuBackend, GpuBackend, GridSpec, exact_reference,
};
use proptest::prelude::*;

fn backend() -> Option<GpuBackend> {
    match GpuBackend::new() {
        Ok(b) => {
            eprintln!("using adapter: {}", b.info());
            Some(b)
        }
        Err(ComputeError::NoAdapter(_)) => {
            eprintln!("skipped: no GPU adapter");
            None
        }
        Err(e) => panic!("unexpected GPU init failure: {e}"),
    }
}

fn compare(gpu: &GpuBackend, c: &Case) {
    let g = gpu.clearance_field(&c.req()).unwrap();
    let cpu = CpuBackend::new().clearance_field(&c.req()).unwrap();
    let r = exact_reference(&c.req());
    let tol = g.error_bound_nm + cpu.error_bound_nm;
    assert_eq!(g.values.len(), cpu.values.len());
    for (k, ((a, b), r)) in g.values.iter().zip(&cpu.values).zip(&r).enumerate() {
        assert!((f64::from(*a) - f64::from(*b)).abs() <= tol, "node {k}: gpu {a} cpu {b}");
        assert!((f64::from(*a) - r).abs() <= g.error_bound_nm, "node {k}: gpu {a} ref {r}");
    }
}

#[test]
fn gpu_adapter_info_does_not_panic() {
    let _ = ferroute_gpu::adapter_info();
}

#[test]
fn gpu_matches_cpu_random() {
    let Some(gpu) = backend() else { return };
    let mut runner = proptest::test_runner::TestRunner::new(ProptestConfig::with_cases(40));
    runner
        .run(&case(), |c| {
            compare(&gpu, &c);
            Ok(())
        })
        .unwrap();
}

fn obstacles(n: i64, extent: i64) -> Vec<Capsule> {
    (0..n)
        .map(|k| Capsule {
            a: [(k * 7919) % extent, (k * 104_729) % extent],
            b: [(k * 7919) % extent + 300, (k * 104_729) % extent],
            radius: 20,
        })
        .collect()
}

// Kept cheap on purpose: CI runs these on lavapipe (software Vulkan on the CPU), where
// nodes × obstacles evaluations cost real CPU time.
#[test]
fn gpu_large_grid_uses_2d_dispatch() {
    let Some(gpu) = backend() else { return };
    // > 65535*64 nodes forces the 2D dispatch path.
    let c = Case {
        grid: GridSpec { origin: [0, 0], pitch: 100, nx: 2200, ny: 2000 },
        obstacles: obstacles(64, 220_000),
        cutoff: 2_000,
    };
    assert!(u64::from(c.grid.nx) * u64::from(c.grid.ny) > 65_535 * 64);
    compare(&gpu, &c);
}

#[test]
fn gpu_many_obstacles_use_multiple_passes() {
    let Some(gpu) = backend() else { return };
    // > 16384 obstacles forces several obstacle passes (min-combined).
    let c = Case {
        grid: GridSpec { origin: [0, 0], pitch: 100, nx: 64, ny: 64 },
        obstacles: obstacles(20_000, 6_400),
        cutoff: 2_000,
    };
    compare(&gpu, &c);
}
