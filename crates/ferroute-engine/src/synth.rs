// SPDX-License-Identifier: AGPL-3.0-only
//! Seeded synthetic board generator for benchmarks and fuzzing.
//!
//! Parts are dropped on a jittered placement grid (so courtyards never overlap), nets
//! connect pins of *nearby* parts (realistic locality, so boards are routable), a fraction
//! of parts is locked, and an optional power net spans the board.

use ferroute_board::builder::{BoardBuilder, PadSpec, mm};
use ferroute_board::{
    Board, IslandRemoval, LayerId, LayerSet, Mobility, NetClass, NetId, PadConnection, PadShape,
    Side, Zone, ZoneFill,
};
use ferroute_geom::rng::Rng;
use ferroute_geom::{Point, Polygon};
use serde::{Deserialize, Serialize};

mod dense;
pub use dense::dense_suite;

/// A generated benchmark board plus the rules file needed to reproduce its constraints
/// (differential pairs and matching groups exist only through rules in KiCad files).
#[derive(Clone, Debug)]
pub struct SynthBoard {
    pub name: String,
    pub seed: u64,
    pub board: Board,
    /// `[[pair]]`/`[[match]]` rules equivalent to the board's in-memory constraints.
    pub rules_toml: Option<String>,
}

/// Boards of the named suite: `ci`/`full` (via [`suite`] + [`generate`]) or `dense`.
#[must_use]
pub fn suite_boards(name: &str) -> Vec<SynthBoard> {
    if name == "dense" {
        return dense_suite();
    }
    suite(name)
        .into_iter()
        .map(|spec| SynthBoard {
            board: generate(&spec),
            name: spec.name,
            seed: spec.seed,
            rules_toml: None,
        })
        .collect()
}

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
    /// Bottom strip (mm) of the board kept free of generated parts (for hand-placed extras).
    #[serde(default)]
    pub strip_mm: i64,
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
        strip_mm: 0,
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
    generate_with(spec, &|_| Extras::default())
}

/// Constraints created by a [`generate_with`] hook.
#[derive(Clone, Debug, Default)]
pub struct Extras {
    pub diff_pairs: Vec<ferroute_board::pairs::DiffPair>,
    pub match_groups: Vec<ferroute_board::constraints::MatchGroup>,
}

/// [`generate`] plus a hook that adds hand-placed parts (in the `strip_mm` area) to the
/// builder after the random parts; it returns the constraints on its nets.
#[must_use]
pub fn generate_with(spec: &SynthSpec, extras: &dyn Fn(&mut BoardBuilder) -> Extras) -> Board {
    let mut rng = Rng::new(spec.seed);
    let mut b = BoardBuilder::new(mm(spec.width_mm), mm(spec.height_mm), spec.layers.max(1));
    b.rules_mut().edge_clearance = 300_000;
    let sp = mm(spec.spacing_mm);
    let cols = ((mm(spec.width_mm) - sp) / sp).max(1);
    let rows = ((mm(spec.height_mm - spec.strip_mm) - sp) / sp).max(1);
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
    let extra = extras(&mut b);
    let mut board = b.build();
    board.diff_pairs = extra.diff_pairs;
    board.match_groups = extra.match_groups;
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

/// Hand-designed fixture for nested length matching: a locked 1×6 header fans out to
/// resistors at deliberately different distances. Differential pairs `D0_*` (near) and
/// `D1_*` (far) are matched within 0.1 mm each, and `CLK` (middle) must match both pairs
/// within 0.5 mm (rules in `testdata/rules/bus_match.toml`), so tuning must nest.
#[must_use]
pub fn bus_match() -> Board {
    let mut b = BoardBuilder::new(mm(50), mm(35), 2);
    let names = ["D0_P", "D0_N", "D1_P", "D1_N", "CLK", "GND"];
    let nets: Vec<NetId> = names.iter().map(|n| b.net(n)).collect();
    let j1 = b.header("J1", Point::new(mm(5), mm(8)), 0.0, &nets);
    b.mobility(j1, Mobility::Locked);
    // (net index, x mm, y mm)
    let parts = [(0, 22, 5), (1, 22, 10), (4, 31, 16), (2, 40, 22), (3, 40, 28)];
    for (k, &(n, x, y)) in parts.iter().enumerate() {
        b.smd2(&format!("R{}", k + 1), Point::new(mm(x), mm(y)), 0.0, nets[n], nets[5]);
    }
    b.build()
}

/// Hand-designed fixture for coupled differential pairs: a locked 1×3 header (GND, `USB_D+`,
/// `USB_D-`) feeds an SOIC-8-like part 30 mm away whose pair pins sit 1.27 mm apart, 12 mm
/// lower, behind a no-track keep-out, so the pair needs two 45° bends and a pad-spacing
/// change at each end (rules in `testdata/rules/diff_pair.toml`: gap 0.2 mm, width 0.25 mm,
/// skew 0.1 mm).
#[must_use]
pub fn diff_pair() -> Board {
    let mut b = BoardBuilder::new(mm(50), mm(34), 2);
    let names = ["GND", "USB_D+", "USB_D-", "VCC"];
    let n: Vec<NetId> = names.iter().map(|x| b.net(x)).collect();
    let j1 = b.header("J1", Point::new(mm(5), mm(8)), 0.0, &[n[0], n[1], n[2]]);
    b.mobility(j1, Mobility::Locked);
    let pad = |num: usize, x: i64, y: i64, net: NetId| PadSpec {
        number: num.to_string(),
        local_pos: Point::new(x, y),
        shape: PadShape::Rect { w: 1_550_000, h: 600_000 },
        drill: None,
        net,
    };
    let rows = [-1_905_000, -635_000, 635_000, 1_905_000];
    let left = [n[3], n[1], n[2], n[0]];
    let mut pads: Vec<PadSpec> = rows
        .iter()
        .zip(left)
        .enumerate()
        .map(|(i, (&y, net))| pad(i + 1, -2_700_000, y, net))
        .collect();
    pads.extend(rows.iter().enumerate().map(|(i, &y)| pad(i + 5, 2_700_000, y, NetId::NONE)));
    b.footprint("U1", Point::new(mm(38), mm(24)), 0.0, Side::Front, &pads);
    b.keepout(
        ferroute_geom::Rect::new(Point::new(mm(16), mm(5)), Point::new(mm(24), mm(17))),
        true,
        true,
    );
    b.build()
}

/// 4-layer 12×12 ball BGA (1 mm pitch, 0.5 mm balls) with `GND` and `VCC` pours on the two
/// inner layers: the 50 power balls (every other inner row/column, the rest unconnected) are GND/VCC and must reach their plane through
/// dog-bone vias, the 44 outer balls are signals to 0603 parts around the package.
#[must_use]
pub fn bga_planes() -> Board {
    let mut b = BoardBuilder::new(mm(40), mm(40), 4);
    {
        // Fine-pitch BGA rules: 0.15 mm tracks, 0.1 mm clearance, 0.45/0.2 mm vias.
        let r = b.rules_mut();
        r.min_track_width = 100_000;
        r.min_via_diameter = 400_000;
        r.min_via_drill = 200_000;
        r.hole_clearance = 200_000;
        r.hole_to_hole = 250_000;
        let c = &mut r.classes[0];
        c.clearance = 100_000;
        c.track_width = 150_000;
        c.via_diameter = 450_000;
        c.via_drill = 200_000;
    }
    let gnd = b.net("GND");
    let vcc = b.net("VCC");
    let n = 12i64;
    let pitch = mm(1);
    let centre = Point::new(mm(20), mm(20));
    let at = |r: i64, c: i64| {
        Point::new(
            centre.x + (c * 2 - (n - 1)) * pitch / 2,
            centre.y + (r * 2 - (n - 1)) * pitch / 2,
        )
    };
    let mut pads = Vec::new();
    let mut parts: Vec<(Point, NetId, u8)> = Vec::new();
    let mut sig = 0usize;
    for r in 0..n {
        for c in 0..n {
            let edge = r == 0 || c == 0 || r == n - 1 || c == n - 1;
            let net = if edge {
                let id = b.net(&format!("S{sig}"));
                sig += 1;
                id
            } else if r % 2 == 1 && c % 2 == 1 {
                gnd
            } else if r % 2 == 0 && c % 2 == 0 {
                vcc
            } else {
                NetId::NONE
            };
            let local = Point::new(at(r, c).x - centre.x, at(r, c).y - centre.y);
            if edge {
                let side = if r == 0 && c < n - 1 {
                    0
                } else if c == n - 1 && r < n - 1 {
                    1
                } else if r == n - 1 && c > 0 {
                    2
                } else {
                    3
                };
                parts.push((at(r, c), net, side));
            }
            pads.push(PadSpec {
                number: format!("{}{}", char::from(b'A' + u8::try_from(r).unwrap_or(0)), c + 1),
                local_pos: local,
                shape: PadShape::Circle { diameter: 500_000 },
                drill: None,
                net,
            });
        }
    }
    let u1 = b.footprint("U1", centre, 0.0, Side::Front, &pads);
    b.mobility(u1, Mobility::Locked);
    for (k, &(p, net, side)) in parts.iter().enumerate() {
        // Three staggered rows per side keep the 0603 parts 3 mm apart laterally.
        let row = mm(9 + 4 * i64::try_from(k % 3).unwrap_or(0));
        let (dx, dy) = (p.x - centre.x, p.y - centre.y);
        let (x, y, rot) = match side {
            0 => (centre.x + dx * 18 / 10, centre.y - row, 90.0),
            1 => (centre.x + row, centre.y + dy * 125 / 100, 0.0),
            2 => (centre.x + dx * 18 / 10, centre.y + row, 90.0),
            _ => (centre.x - row, centre.y + dy * 125 / 100, 0.0),
        };
        let fp = b.smd2(&format!("R{k}"), Point::new(x, y), rot, net, NetId::NONE);
        b.mobility(fp, Mobility::Locked);
    }
    let mut board = b.build();
    let poly = Polygon::from_rect(ferroute_geom::Rect::new(
        Point::new(mm(1), mm(1)),
        Point::new(mm(39), mm(39)),
    ));
    for (net, layer) in [(gnd, 1u8), (vcc, 2u8)] {
        board.zones.push(Zone {
            net,
            layers: LayerSet::single(LayerId(layer)),
            outline: poly.clone(),
            fills: vec![ZoneFill { layer: LayerId(layer), polys: vec![poly.clone()] }],
            clearance: 100_000,
            min_thickness: 150_000,
            priority: 0,
            pad_connection: PadConnection::ThermalRelief,
            islands: IslandRemoval::Always,
            yields: false,
        });
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
