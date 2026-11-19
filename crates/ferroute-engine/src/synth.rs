// SPDX-License-Identifier: AGPL-3.0-only
//! Seeded synthetic board generator for benchmarks and fuzzing.
//!
//! Parts are dropped on a jittered placement grid (so courtyards never overlap), nets
//! connect pins of *nearby* parts (realistic locality, so boards are routable), a fraction
//! of parts is locked, and an optional power net spans the board.

use ferroute_board::builder::{BoardBuilder, mm};
use ferroute_board::{Board, Mobility, NetClass, NetId};
use ferroute_geom::Point;
use ferroute_geom::rng::Rng;
use serde::{Deserialize, Serialize};

/// Parameters of a synthetic board.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SynthSpec {
    pub name: String,
    pub width_mm: i64,
    pub height_mm: i64,
    pub layers: u8,
    /// Two-pin SMD parts.
    pub parts: usize,
    /// 1×4 THT headers.
    pub headers: usize,
    /// Signal nets (each joins 2–4 pins of nearby parts).
    pub nets: usize,
    /// Fraction of parts that are locked.
    pub locked_fraction: f64,
    /// Placement grid spacing (mm).
    pub spacing_mm: i64,
    /// Add a wide-class GND net touching many parts.
    pub power_net: bool,
    pub seed: u64,
}

/// Named suites. `ci` is small enough for every PR; `full` is for nightly runs.
#[must_use]
pub fn suite(name: &str) -> Vec<SynthSpec> {
    let base = |name: &str, w, h, layers, parts, headers, nets, seed| SynthSpec {
        name: name.into(),
        width_mm: w,
        height_mm: h,
        layers,
        parts,
        headers,
        nets,
        locked_fraction: 0.25,
        spacing_mm: 5,
        power_net: true,
        seed,
    };
    match name {
        "full" => vec![
            base("s2-small", 40, 30, 2, 18, 1, 14, 11),
            base("s2-medium", 60, 45, 2, 40, 2, 32, 12),
            base("s2-dense", 50, 40, 2, 60, 2, 50, 13),
            base("s4-medium", 60, 45, 4, 50, 2, 45, 14),
            base("s4-large", 100, 80, 4, 150, 4, 130, 15),
        ],
        _ => vec![
            base("ci-s2-small", 40, 30, 2, 14, 1, 10, 1),
            base("ci-s2-medium", 55, 40, 2, 30, 2, 24, 2),
            base("ci-s4-medium", 55, 40, 4, 36, 2, 32, 3),
        ],
    }
}

/// Generates the board described by `spec`.
#[must_use]
pub fn generate(spec: &SynthSpec) -> Board {
    let mut rng = Rng::new(spec.seed);
    let mut b = BoardBuilder::new(mm(spec.width_mm), mm(spec.height_mm), spec.layers.max(1));
    b.rules_mut().edge_clearance = 300_000;
    let sp = mm(spec.spacing_mm);
    let cols = ((mm(spec.width_mm) - sp) / sp).max(1);
    let rows = ((mm(spec.height_mm) - sp) / sp).max(1);
    let mut cells: Vec<(i64, i64)> =
        (0..rows).flat_map(|r| (0..cols).map(move |c| (c, r))).collect();
    // Fisher–Yates shuffle (seeded).
    for i in (1..cells.len()).rev() {
        let j = usize::try_from(rng.below(i as u64 + 1)).unwrap_or(0);
        cells.swap(i, j);
    }
    // Headers span three cells vertically: reserve the cells above and below them.
    let mut used = std::collections::BTreeSet::new();
    let mut header_cells = Vec::new();
    for &(c, r) in &cells {
        if header_cells.len() >= spec.headers {
            break;
        }
        if r >= 1
            && r + 1 < rows
            && [(c, r - 1), (c, r), (c, r + 1)].iter().all(|x| !used.contains(x))
        {
            used.extend([(c, r - 1), (c, r), (c, r + 1)]);
            header_cells.push((c, r));
        }
    }
    let part_cells: Vec<(i64, i64)> =
        cells.iter().copied().filter(|x| !used.contains(x)).take(spec.parts).collect();
    let gnd = if spec.power_net { Some(b.net("GND")) } else { None };
    let nets: Vec<NetId> = (0..spec.nets).map(|i| b.net(&format!("N{i}"))).collect();
    let mut centres: Vec<Point> = Vec::new();
    let mut pin_count: Vec<usize> = Vec::new();
    for (k, &(c, r)) in header_cells.iter().enumerate() {
        let pos = Point::new(sp + c * sp, sp + r * sp);
        centres.push(pos);
        b.header(&format!("J{}", k + 1), pos - Point::new(0, 3_810_000), 0.0, &[NetId::NONE; 4]);
        pin_count.push(4);
    }
    for (k, &(c, r)) in part_cells.iter().enumerate() {
        let jitter = rng.range_i64(-4, 4) * 100_000;
        let pos = Point::new(sp + c * sp + jitter, sp + r * sp + jitter / 2);
        centres.push(pos);
        let rot = if rng.below(2) == 0 { 0.0 } else { 90.0 };
        b.smd2(&format!("R{}", k + 1), pos, rot, NetId::NONE, NetId::NONE);
        pin_count.push(2);
    }
    let mut board = b.build();
    // Free pins (footprint, pad slot).
    let mut free: Vec<(usize, usize)> =
        pin_count.iter().enumerate().flat_map(|(f, &n)| (0..n).map(move |p| (f, p))).collect();
    let take_near = |free: &mut Vec<(usize, usize)>,
                     rng: &mut Rng,
                     around: Point,
                     max_d: f64|
     -> Option<(usize, usize)> {
        let cands: Vec<usize> =
            (0..free.len()).filter(|&i| centres[free[i].0].dist(around) <= max_d).collect();
        if cands.is_empty() {
            return None;
        }
        let pick = cands[usize::try_from(rng.below(cands.len() as u64)).unwrap_or(0)];
        Some(free.swap_remove(pick))
    };
    #[allow(clippy::cast_precision_loss)]
    let reach = (sp * 3) as f64;
    for &net in &nets {
        if free.len() < 2 {
            break;
        }
        let seed_pin = free.swap_remove(usize::try_from(rng.below(free.len() as u64)).unwrap_or(0));
        let mut pins = vec![seed_pin];
        let want = 2 + usize::try_from(rng.below(3)).unwrap_or(0);
        let mut rejected = Vec::new();
        for _ in 0..16 {
            if pins.len() >= want {
                break;
            }
            match take_near(&mut free, &mut rng, centres[seed_pin.0], reach) {
                // Avoid nets that only join the two pads of one part.
                Some(p) if p.0 != seed_pin.0 || pins.len() > 1 => pins.push(p),
                Some(p) => rejected.push(p),
                None => break,
            }
        }
        free.extend(rejected);
        if pins.len() < 2 {
            free.extend(pins);
            continue;
        }
        for (f, p) in pins {
            let pad = board.footprints[f].pads[p];
            board.pads[pad.idx()].net = net;
        }
    }
    if let Some(g) = gnd {
        board.rules.classes.push(NetClass {
            name: "Power".into(),
            clearance: 250_000,
            track_width: 400_000,
            via_diameter: 700_000,
            via_drill: 350_000,
        });
        board.nets[g.idx()].class = 1;
        // GND on up to a quarter of the remaining free pins.
        let n = free.len() / 4;
        for _ in 0..n.max(2).min(free.len()) {
            let (f, p) =
                free.swap_remove(usize::try_from(rng.below(free.len() as u64)).unwrap_or(0));
            let pad = board.footprints[f].pads[p];
            board.pads[pad.idx()].net = g;
        }
    }
    for fp in &mut board.footprints {
        if rng.next_f64() < spec.locked_fraction {
            fp.locked = true;
            fp.mobility = Mobility::Locked;
        }
    }
    board
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generator_is_deterministic_and_valid() {
        for spec in suite("full") {
            let a = generate(&spec);
            a.validate().expect("valid");
            assert_eq!(a, generate(&spec), "{} not deterministic", spec.name);
            let rep = ferroute_drc::check(&a, &ferroute_drc::DrcConfig::default());
            assert!(
                rep.is_clean(),
                "{}: unrouted synthetic board must be DRC-clean: {:#?}",
                spec.name,
                rep.violations
            );
        }
    }
}
