// SPDX-License-Identifier: AGPL-3.0-only
//! Criterion benchmarks of the full pipeline on the synthetic CI suite.

use criterion::{Criterion, criterion_group, criterion_main};
use ferroute_engine::{PipelineConfig, run, synth};
use std::hint::black_box;

fn bench(c: &mut Criterion) {
    let mut g = c.benchmark_group("pipeline");
    g.sample_size(10);
    for spec in synth::suite("ci") {
        let board = synth::generate(&spec);
        g.bench_function(&spec.name, |b| {
            b.iter(|| run(black_box(&board), &PipelineConfig::default()).expect("run"));
        });
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
