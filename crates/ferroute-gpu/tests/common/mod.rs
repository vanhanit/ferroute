// SPDX-License-Identifier: AGPL-3.0-only
#![allow(dead_code)]
use ferroute_gpu::{Capsule, FieldRequest, GridSpec};
use proptest::prelude::*;

#[derive(Debug, Clone)]
pub struct Case {
    pub grid: GridSpec,
    pub obstacles: Vec<Capsule>,
    pub cutoff: i64,
}

impl Case {
    pub fn req(&self) -> FieldRequest<'_> {
        FieldRequest { grid: self.grid, obstacles: &self.obstacles, cutoff: self.cutoff }
    }
}

pub fn capsule() -> impl Strategy<Value = Capsule> {
    let c = -2_000_000i64..2_000_000;
    (c.clone(), c.clone(), c.clone(), c, 0i64..60_000, 0u8..4).prop_map(
        |(ax, ay, bx, by, r, kind)| {
            // kind 0 => zero-length capsule (disk)
            let b = if kind == 0 { [ax, ay] } else { [bx, by] };
            Capsule { a: [ax, ay], b, radius: r }
        },
    )
}

pub fn case() -> impl Strategy<Value = Case> {
    (
        (-1_000_000i64..1_000_000, -1_000_000i64..1_000_000),
        1i64..8_000,
        1u32..28,
        1u32..28,
        prop::collection::vec(capsule(), 0..14),
        1i64..300_000,
    )
        .prop_map(|((ox, oy), pitch, nx, ny, obstacles, cutoff)| Case {
            grid: GridSpec { origin: [ox, oy], pitch, nx, ny },
            obstacles,
            cutoff,
        })
}
