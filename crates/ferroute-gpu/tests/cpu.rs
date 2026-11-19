// SPDX-License-Identifier: AGPL-3.0-only
mod common;

use common::{Case, case};
use ferroute_gpu::{
    BackendPreference, Capsule, ClearanceBackend, ComputeError, CpuBackend, FieldRequest, GridSpec,
    exact_reference, select_backend,
};
use proptest::prelude::*;

fn check_against_reference(c: &Case) {
    let f = CpuBackend::new().clearance_field(&c.req()).unwrap();
    let r = exact_reference(&c.req());
    assert_eq!(f.values.len(), r.len());
    assert!(f.error_bound_nm >= 1.0 && f.error_bound_nm.is_finite());
    let cutoff = c.cutoff as f64;
    for (k, (v, r)) in f.values.iter().zip(&r).enumerate() {
        let v = f64::from(*v);
        assert!(v <= cutoff + f.error_bound_nm, "node {k}: {v} above cutoff");
        assert!(
            (v - r).abs() <= f.error_bound_nm,
            "node {k}: cpu {v} vs ref {r}, bound {}",
            f.error_bound_nm
        );
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(300))]

    #[test]
    fn cpu_matches_reference(c in case()) {
        check_against_reference(&c);
    }

    #[test]
    fn cpu_is_thread_count_independent(c in case()) {
        let run = |n: usize| {
            rayon::ThreadPoolBuilder::new().num_threads(n).build().unwrap()
                .install(|| CpuBackend::new().clearance_field(&c.req()).unwrap())
        };
        let a = run(1);
        let b = run(7);
        let ab: Vec<u32> = a.values.iter().map(|v| v.to_bits()).collect();
        let bb: Vec<u32> = b.values.iter().map(|v| v.to_bits()).collect();
        prop_assert_eq!(ab, bb);
    }
}

fn grid(n: u32) -> GridSpec {
    GridSpec { origin: [0, 0], pitch: 1000, nx: n, ny: n }
}

#[test]
fn empty_obstacles_gives_cutoff() {
    let obs = [];
    let req = FieldRequest { grid: grid(5), obstacles: &obs, cutoff: 12_345 };
    let f = CpuBackend::new().clearance_field(&req).unwrap();
    assert_eq!(f.values.len(), 25);
    assert!(f.values.iter().all(|&v| v.to_bits() == 12_345f32.to_bits()));
}

#[test]
fn zero_length_and_on_segment_and_inside() {
    let obs = [
        Capsule { a: [2000, 2000], b: [2000, 2000], radius: 0 },
        Capsule { a: [0, 4000], b: [4000, 4000], radius: 300 },
        Capsule { a: [0, 0], b: [0, 0], radius: 5000 },
    ];
    let c = Case { grid: grid(5), obstacles: obs.to_vec(), cutoff: 20_000 };
    check_against_reference(&c);
    let f = CpuBackend::new().clearance_field(&c.req()).unwrap();
    // Node (2,2) is exactly on the point obstacle but inside the big disk
    // at origin only if within 5000: distance is 2828 -> d = -2172.
    assert!((f64::from(f.values[2 * 5 + 2]) + 2171.57).abs() < 1.0);
    // Node (3,4) lies on the horizontal segment: d = -300.
    assert!((f64::from(f.values[4 * 5 + 3]) + 300.0).abs() < 1.0);
}

#[test]
fn obstacles_outside_grid() {
    let obs = [
        Capsule { a: [-1_000_000, -1_000_000], b: [-900_000, -1_000_000], radius: 10 },
        Capsule { a: [9_000, 2_000], b: [9_000, 3_000], radius: 1_000 },
    ];
    let c = Case { grid: grid(5), obstacles: obs.to_vec(), cutoff: 3_000 };
    check_against_reference(&c);
}

#[test]
fn many_obstacles_small_cutoff_fine_pitch() {
    // Exercises bucket culling with a long capsule spanning many buckets.
    let mut obs = vec![Capsule { a: [0, 5_000], b: [200_000, 5_000], radius: 50 }];
    for k in 0..200 {
        obs.push(Capsule {
            a: [k * 997, k * 631 % 90_000],
            b: [k * 997 + 400, k * 631 % 90_000],
            radius: 25,
        });
    }
    let c = Case {
        grid: GridSpec { origin: [-1234, 777], pitch: 100, nx: 300, ny: 200 },
        obstacles: obs,
        cutoff: 1_000,
    };
    check_against_reference(&c);
}

#[test]
fn validation() {
    let b = CpuBackend::new();
    let obs = [Capsule { a: [0, 0], b: [1, 1], radius: 1 }];
    let ok = FieldRequest { grid: grid(2), obstacles: &obs, cutoff: 10 };
    assert!(b.clearance_field(&ok).is_ok());
    let mut r = ok.clone();
    r.grid.pitch = 0;
    assert!(matches!(b.clearance_field(&r), Err(ComputeError::Invalid(_))));
    let mut r = ok.clone();
    r.cutoff = 0;
    assert!(matches!(b.clearance_field(&r), Err(ComputeError::Invalid(_))));
    let mut r = ok.clone();
    r.grid.nx = 1 << 14;
    r.grid.ny = (1 << 12) + 1;
    assert!(matches!(b.clearance_field(&r), Err(ComputeError::Invalid(_))));
    let bad = [Capsule { a: [1 << 31, 0], b: [0, 0], radius: 0 }];
    let r = FieldRequest { grid: grid(2), obstacles: &bad, cutoff: 10 };
    assert!(matches!(b.clearance_field(&r), Err(ComputeError::Invalid(_))));
    let bad = [Capsule { a: [0, 0], b: [0, 0], radius: -1 }];
    let r = FieldRequest { grid: grid(2), obstacles: &bad, cutoff: 10 };
    assert!(matches!(b.clearance_field(&r), Err(ComputeError::Invalid(_))));
}

#[test]
fn select_cpu_and_auto() {
    assert_eq!(select_backend(BackendPreference::Cpu).unwrap().name(), "cpu");
    let auto = select_backend(BackendPreference::Auto).unwrap();
    assert!(auto.name() == "cpu" || auto.name() == "gpu");
    #[cfg(not(feature = "gpu"))]
    assert!(select_backend(BackendPreference::Gpu).is_err());
}
