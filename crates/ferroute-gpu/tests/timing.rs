// SPDX-License-Identifier: AGPL-3.0-only
//! Micro-timing smoke test; run with `cargo test --release -- --ignored --nocapture`.
use std::time::Instant;

use ferroute_gpu::{BackendPreference, Capsule, FieldRequest, GridSpec, select_backend};

#[test]
#[ignore = "timing smoke test"]
fn timing_smoke() {
    let obs: Vec<Capsule> = (0..5_000i64)
        .map(|k| Capsule {
            a: [(k * 7919) % 100_000_000, (k * 104_729) % 100_000_000],
            b: [(k * 7919) % 100_000_000 + 2_000_000, (k * 104_729) % 100_000_000],
            radius: 100_000,
        })
        .collect();
    let req = FieldRequest {
        grid: GridSpec { origin: [0, 0], pitch: 100_000, nx: 1000, ny: 1000 },
        obstacles: &obs,
        cutoff: 1_000_000,
    };
    for pref in [BackendPreference::Cpu, BackendPreference::Auto] {
        let b = select_backend(pref).unwrap();
        let t = Instant::now();
        let f = b.clearance_field(&req).unwrap();
        eprintln!(
            "{}: {} nodes in {:?}, bound {} nm",
            f.backend,
            f.values.len(),
            t.elapsed(),
            f.error_bound_nm
        );
    }
}
