// SPDX-License-Identifier: AGPL-3.0-only
//! The `dense` benchmark suite: BGA escape, fine-pitch QFN and a mixed 8-layer board.
//!
//! Every board is a pure function of its seed (frozen [`Rng`]); parts are locked where a
//! real designer would not move them (BGA, ICs, connectors) and plain passives stay
//! movable. Inner-layer planes are filled copper zones (the pipeline refills them).

use super::{Extras, SynthBoard, SynthSpec, generate_with};
use ferroute_board::builder::{BoardBuilder, PadSpec, mm};
use ferroute_board::constraints::{MatchBy, MatchGroup};
use ferroute_board::pairs::DiffPair;
use ferroute_board::{
    Board, IslandRemoval, LayerId, LayerSet, Mobility, NetId, PadConnection, PadShape, Side, Zone,
    ZoneFill,
};
use ferroute_geom::rng::Rng;
use ferroute_geom::{Coord, Point, Polygon, Rect};

/// The dense suite, in a fixed order.
#[must_use]
pub fn dense_suite() -> Vec<SynthBoard> {
    let plain = |name: &str, seed: u64, board: Board| SynthBoard {
        name: name.into(),
        seed,
        board,
        rules_toml: None,
    };
    let (mixed, toml) = mixed_8l(24);
    vec![
        plain("bga_256_4l", 21, bga_256_4l(21)),
        plain("bga_484_6l", 22, bga_484_6l(22)),
        plain("qfn_dense_2l", 23, qfn_dense_2l(23)),
        SynthBoard { name: "mixed_8l".into(), seed: 24, board: mixed, rules_toml: Some(toml) },
    ]
}

// ---------------------------------------------------------------------------------------
// Footprint pad generators (local coordinates, no nets)
// ---------------------------------------------------------------------------------------

fn pad(num: String, x: Coord, y: Coord, shape: PadShape) -> PadSpec {
    PadSpec { number: num, local_pos: Point::new(x, y), shape, drill: None, net: NetId::NONE }
}

/// 0402 pads: 0.5 × 0.6 mm at ±0.485 mm.
fn pads_0402() -> Vec<PadSpec> {
    let s = PadShape::RoundRect { w: 500_000, h: 600_000, radius: 125_000 };
    vec![pad("1".into(), -485_000, 0, s.clone()), pad("2".into(), 485_000, 0, s)]
}

/// SOIC-8, 1.27 mm pitch; pins 1–4 left (top to bottom), 5–8 right (bottom to top).
fn pads_soic8() -> Vec<PadSpec> {
    let s = PadShape::Rect { w: 1_550_000, h: 600_000 };
    let rows = [-1_905_000, -635_000, 635_000, 1_905_000];
    let mut v: Vec<PadSpec> = rows
        .iter()
        .enumerate()
        .map(|(i, &y)| pad((i + 1).to_string(), -2_700_000, y, s.clone()))
        .collect();
    v.extend(
        rows.iter()
            .rev()
            .enumerate()
            .map(|(i, &y)| pad((i + 5).to_string(), 2_700_000, y, s.clone())),
    );
    v
}

/// QFN with `per_side` pins per side at `pitch`, counter-clockwise from the top of the left
/// side, plus an exposed pad as the last entry.
fn pads_qfn(per_side: i64, pitch: Coord) -> Vec<PadSpec> {
    let body = per_side * pitch + 1_000_000;
    let d = body / 2 - 100_000;
    let (long, short) = (800_000, 250_000);
    let horiz = PadShape::Rect { w: long, h: short };
    let vert = PadShape::Rect { w: short, h: long };
    let s = |k: i64| (2 * k - (per_side - 1)) * pitch / 2;
    let mut v = Vec::new();
    let mut num = 1;
    for k in 0..per_side {
        v.push(pad(num.to_string(), -d, s(k), horiz.clone()));
        num += 1;
    }
    for k in 0..per_side {
        v.push(pad(num.to_string(), s(k), d, vert.clone()));
        num += 1;
    }
    for k in 0..per_side {
        v.push(pad(num.to_string(), d, -s(k), horiz.clone()));
        num += 1;
    }
    for k in 0..per_side {
        v.push(pad(num.to_string(), -s(k), -d, vert.clone()));
        num += 1;
    }
    let ep = body - 1_800_000;
    v.push(pad(num.to_string(), 0, 0, PadShape::Rect { w: ep, h: ep }));
    v
}

const LETTERS: &[u8] = b"ABCDEFGHJKLMNPRTUVWY";

fn ball_name(r: usize, c: usize) -> String {
    let row = if r < LETTERS.len() {
        (LETTERS[r] as char).to_string()
    } else {
        format!("A{}", LETTERS[r - LETTERS.len()] as char)
    };
    format!("{row}{}", c + 1)
}

fn ball_offset(i: usize, n: usize, pitch: Coord) -> Coord {
    (2 * i as i64 - (n as i64 - 1)) * pitch / 2
}

/// `n × n` BGA, row-major pads named `A1`…
fn pads_bga(n: usize, pitch: Coord, ball: Coord) -> Vec<PadSpec> {
    let shape = PadShape::Circle { diameter: ball };
    (0..n)
        .flat_map(|r| (0..n).map(move |c| (r, c)))
        .map(|(r, c)| {
            pad(ball_name(r, c), ball_offset(c, n, pitch), ball_offset(r, n, pitch), shape.clone())
        })
        .collect()
}

// ---------------------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------------------

fn set_rules(b: &mut BoardBuilder, clearance: Coord, width: Coord, via: Coord, drill: Coord) {
    let r = b.rules_mut();
    r.edge_clearance = 300_000;
    // Board-level minimums follow the (small) class values so vias are legal at all.
    r.min_track_width = width;
    r.min_via_diameter = via;
    r.min_via_drill = drill;
    let c = &mut r.classes[0];
    c.clearance = clearance;
    c.track_width = width;
    c.via_diameter = via;
    c.via_drill = drill;
}

/// A filled rectangular pour on one layer, inset 0.5 mm from the board outline.
fn add_plane(board: &mut Board, net: NetId, layer: u8) {
    let bb = board.outline.outer.bbox().unwrap_or(Rect::new(Point::ORIGIN, Point::ORIGIN));
    let m = 500_000;
    let r =
        Rect::new(Point::new(bb.min.x + m, bb.min.y + m), Point::new(bb.max.x - m, bb.max.y - m));
    let outline = Polygon::from_rect(r);
    let l = LayerId(layer);
    board.zones.push(Zone {
        net,
        layers: LayerSet::single(l),
        outline: outline.clone(),
        fills: vec![ZoneFill { layer: l, polys: vec![outline] }],
        clearance: board.rules.classes[0].clearance.max(200_000),
        min_thickness: 200_000,
        priority: 0,
        pad_connection: PadConnection::ThermalRelief,
        islands: IslandRemoval::Always,
        yields: false,
    });
}

/// Recomputes every pour around the other-net copper (through-hole pads), as KiCad would.
fn refill(mut board: Board) -> Board {
    let r = ferroute_drc::refill::refill_zones(&board);
    board.apply(&ferroute_board::BoardEdit { zone_fills: r.fills, ..Default::default() });
    board
}

fn overlaps(a: Rect, b: Rect) -> bool {
    a.min.x < b.max.x && b.min.x < a.max.x && a.min.y < b.max.y && b.min.y < a.max.y
}

fn around(c: Point, hx: Coord, hy: Coord) -> Rect {
    Rect::new(Point::new(c.x - hx, c.y - hy), Point::new(c.x + hx, c.y + hy))
}

fn d2(a: Point, b: Point) -> i128 {
    let (dx, dy) = (i128::from(a.x - b.x), i128::from(a.y - b.y));
    dx * dx + dy * dy
}

/// Adds a passive (0402, or 0603 when `big`) with pad 1 on `a` and pad 2 on `c`.
fn passive(b: &mut BoardBuilder, name: &str, pos: Point, rot: f64, big: bool, a: NetId, c: NetId) {
    if big {
        b.smd2(name, pos, rot, a, c);
    } else {
        let mut pads = pads_0402();
        pads[0].net = a;
        pads[1].net = c;
        b.footprint(name, pos, rot, Side::Front, &pads);
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Ic {
    Soic8,
    Qfn24,
}

impl Ic {
    fn pads(self) -> Vec<PadSpec> {
        match self {
            Ic::Soic8 => pads_soic8(),
            Ic::Qfn24 => pads_qfn(6, 500_000),
        }
    }

    /// `(gnd pins, vcc pins, signal pins)` as 0-based pad indices.
    fn roles(self) -> (Vec<usize>, Vec<usize>, Vec<usize>) {
        match self {
            Ic::Soic8 => (vec![3], vec![7], vec![0, 1, 2, 4, 5, 6]),
            // Exposed pad (index 24) is ground; pins 0 and 12 are the supplies.
            Ic::Qfn24 => (vec![24], vec![0, 12], vec![2, 3, 4, 8, 9, 10, 14, 15]),
        }
    }

    /// Half extents of the area reserved around the part.
    fn reserve(self) -> (Coord, Coord) {
        match self {
            Ic::Soic8 => (4_500_000, 3_500_000),
            Ic::Qfn24 => (4_000_000, 4_000_000),
        }
    }
}

// ---------------------------------------------------------------------------------------
// BGA boards
// ---------------------------------------------------------------------------------------

struct BgaSpec {
    n: usize,
    pitch: Coord,
    ball: Coord,
    layers: u8,
    w: i64,
    h: i64,
    /// Outer rings that carry signals; the inside is a GND/VCC row stripes (a checkerboard makes the router cross GND and VCC diagonals).
    sig_rings: usize,
    /// Inner layers holding the GND and VCC planes.
    planes: [u8; 2],
    clearance: Coord,
    width: Coord,
    via: Coord,
    drill: Coord,
    seed: u64,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Ball {
    Nc,
    Gnd,
    Vcc,
    Sig,
}

fn bga_board(s: &BgaSpec) -> Board {
    let mut rng = Rng::new(s.seed);
    let mut b = BoardBuilder::new(mm(s.w), mm(s.h), s.layers);
    set_rules(&mut b, s.clearance, s.width, s.via, s.drill);
    let (gnd, vcc) = (b.net("GND"), b.net("VCC"));
    let centre = Point::new(mm(s.w) / 2, mm(s.h) / 2);
    let n = s.n;
    let ball_pos = |r: usize, c: usize| {
        Point::new(centre.x + ball_offset(c, n, s.pitch), centre.y + ball_offset(r, n, s.pitch))
    };
    // Ball roles.
    let mut kind = vec![Ball::Nc; n * n];
    for r in 0..n {
        for c in 0..n {
            let ring = r.min(c).min(n - 1 - r).min(n - 1 - c);
            kind[r * n + c] = if ring < s.sig_rings {
                if ring + 1 == s.sig_rings && rng.below(10) >= 7 { Ball::Nc } else { Ball::Sig }
            } else if r % 2 == 0 {
                Ball::Gnd
            } else {
                Ball::Vcc
            };
        }
    }
    let ring_of = |i: usize| {
        let (r, c) = (i / n, i % n);
        r.min(c).min(n - 1 - r).min(n - 1 - c)
    };
    let mut ball_net = vec![NetId::NONE; n * n];
    for (i, k) in kind.iter().enumerate() {
        ball_net[i] = match k {
            Ball::Gnd => gnd,
            Ball::Vcc => vcc,
            _ => NetId::NONE,
        };
    }
    let mut free_balls: Vec<usize> = (0..n * n).filter(|&i| kind[i] == Ball::Sig).collect();
    let mut sig_counter = 0usize;
    let mut next_sig = |b: &mut BoardBuilder| {
        let id = b.net(&format!("S{sig_counter}"));
        sig_counter += 1;
        id
    };
    // Keep-out of the BGA courtyard plus a fan-out margin.
    let half = (n as i64 - 1) * s.pitch / 2 + s.ball / 2 + 250_000;
    let excl = around(centre, half + 1_500_000, half + 1_500_000);
    // ICs in three corners.
    let ics = [
        (Ic::Soic8, Point::new(mm(7), mm(s.h - 7))),
        (Ic::Soic8, Point::new(mm(s.w - 7), mm(7))),
        (Ic::Qfn24, Point::new(mm(s.w - 8), mm(s.h - 8))),
    ];
    let mut reserved = vec![excl];
    let mut ic_parts: Vec<(Ic, Point, Vec<PadSpec>)> = Vec::new();
    for &(ic, pos) in &ics {
        let (hx, hy) = ic.reserve();
        reserved.push(around(pos, hx, hy));
        let mut pads = ic.pads();
        let (g, v, sg) = ic.roles();
        for i in g {
            pads[i].net = gnd;
        }
        for i in v {
            pads[i].net = vcc;
        }
        // Take the signal balls on the outer ring closest to the part, pair them with the
        // part's signal pins in coordinate order.
        let mut cand: Vec<usize> =
            free_balls.iter().copied().filter(|&i| ring_of(i) == 0).collect();
        cand.sort_by_key(|&i| (d2(ball_pos(i / n, i % n), pos), i));
        let mut chosen: Vec<usize> = cand.into_iter().take(sg.len()).collect();
        chosen.sort_by_key(|&i| {
            let p = ball_pos(i / n, i % n);
            (p.x, p.y, i)
        });
        let mut pins = sg;
        pins.sort_by_key(|&i| (pads[i].local_pos.x, pads[i].local_pos.y, i));
        for (&ball, &pin) in chosen.iter().zip(&pins) {
            let net = next_sig(&mut b);
            ball_net[ball] = net;
            pads[pin].net = net;
            free_balls.retain(|&x| x != ball);
        }
        ic_parts.push((ic, pos, pads));
    }
    // Passive slot lattice outside every reserved area.
    let mut slots: Vec<Point> = Vec::new();
    let mut y = mm(2) + 500_000;
    while y <= mm(s.h) - mm(2) - 500_000 {
        let mut x = mm(3);
        while x <= mm(s.w) - mm(3) {
            let p = Point::new(x, y);
            let r = around(p, 1_600_000, 800_000);
            if !reserved.iter().any(|&q| overlaps(r, q)) {
                slots.push(p);
            }
            x += 3_200_000;
        }
        y += 1_600_000;
    }
    // Outer balls first, nearest free slot each; a passive ties the net to GND or VCC.
    free_balls.sort_by_key(|&i| (ring_of(i), i));
    let mut passives: Vec<(Point, f64, bool, NetId, NetId)> = Vec::new();
    for &i in &free_balls {
        let bp = ball_pos(i / n, i % n);
        let Some(k) = (0..slots.len()).min_by_key(|&k| (d2(slots[k], bp), k)) else { break };
        let slot = slots.remove(k);
        let net = next_sig(&mut b);
        ball_net[i] = net;
        let other = if rng.below(2) == 0 { gnd } else { vcc };
        let big = rng.below(4) == 0;
        let rot = if bp.x > slot.x { 180.0 } else { 0.0 };
        passives.push((slot, rot, big, net, other));
    }
    let mut pads = pads_bga(n, s.pitch, s.ball);
    for (p, &net) in pads.iter_mut().zip(&ball_net) {
        p.net = net;
    }
    let u1 = b.footprint("U1", centre, 0.0, Side::Front, &pads);
    b.mobility(u1, Mobility::Locked);
    for (k, (_, pos, pads)) in ic_parts.iter().enumerate() {
        let id = b.footprint(&format!("U{}", k + 2), *pos, 0.0, Side::Front, pads);
        b.mobility(id, Mobility::Locked);
    }
    for (k, &(pos, rot, big, a, c)) in passives.iter().enumerate() {
        passive(&mut b, &format!("R{}", k + 1), pos, rot, big, a, c);
    }
    let mut board = b.build();
    add_plane(&mut board, gnd, s.planes[0]);
    add_plane(&mut board, vcc, s.planes[1]);
    refill(board)
}

/// 16 × 16 BGA, 1.0 mm pitch, 0.5 mm balls, 4 layers (GND and VCC planes on In1/In2).
#[must_use]
pub fn bga_256_4l(seed: u64) -> Board {
    bga_board(&BgaSpec {
        n: 16,
        pitch: 1_000_000,
        ball: 500_000,
        layers: 4,
        w: 48,
        h: 48,
        sig_rings: 3,
        planes: [1, 2],
        clearance: 100_000,
        width: 100_000,
        via: 450_000,
        drill: 200_000,
        seed,
    })
}

/// 22 × 22 BGA, 0.8 mm pitch, 0.4 mm balls, 6 layers (GND on In1, VCC on In4). Only the two
/// outer ball rings carry signals (three or four rings did not finish within 25 min).
#[must_use]
pub fn bga_484_6l(seed: u64) -> Board {
    bga_board(&BgaSpec {
        n: 22,
        pitch: 800_000,
        ball: 400_000,
        layers: 6,
        w: 48,
        h: 48,
        sig_rings: 2,
        planes: [1, 4],
        clearance: 100_000,
        width: 100_000,
        via: 400_000,
        drill: 200_000,
        seed,
    })
}

// ---------------------------------------------------------------------------------------
// Four QFN-48 on a tight two-layer board
// ---------------------------------------------------------------------------------------

/// Four 0.5 mm-pitch QFN-48 (7 × 7 mm) in a 2 × 2 array on a 34 × 34 mm two-layer board.
/// Facing sides carry 10-wide buses between neighbours, outer sides break out through
/// 0402 passives; a GND pour covers the back.
#[must_use]
pub fn qfn_dense_2l(seed: u64) -> Board {
    let mut rng = Rng::new(seed);
    let mut b = BoardBuilder::new(mm(34), mm(34), 2);
    set_rules(&mut b, 100_000, 150_000, 400_000, 200_000);
    let (gnd, vcc) = (b.net("GND"), b.net("VCC"));
    let centres = [
        Point::new(mm(10), mm(10)),
        Point::new(mm(24), mm(10)),
        Point::new(mm(10), mm(24)),
        Point::new(mm(24), mm(24)),
    ];
    // Side indices: 0 left, 1 bottom, 2 right, 3 top; pin `side*12 + k`.
    let mut nets = vec![vec![NetId::NONE; 49]; 4];
    for q in &mut nets {
        q[48] = gnd;
        for side in 0..4 {
            q[side * 12] = gnd;
            q[side * 12 + 11] = vcc;
        }
    }
    // (qfn a, side a, qfn b, side b): pin k of a joins pin 11-k of b (mirrored).
    let links = [(0, 2, 1, 0), (2, 2, 3, 0), (0, 1, 2, 3), (1, 1, 3, 3)];
    let mut count = 0;
    for &(a, sa, c, sc) in &links {
        for k in 1..11 {
            let net = b.net(&format!("B{count}"));
            count += 1;
            nets[a][sa * 12 + k] = net;
            nets[c][sc * 12 + (11 - k)] = net;
        }
    }
    // Outer sides: every third pin goes to a passive column.
    let mut passives: Vec<(Point, f64, NetId, NetId)> = Vec::new();
    let mut outer = 0;
    for (qi, &c) in centres.iter().enumerate() {
        let left = qi % 2 == 0;
        let top = qi < 2;
        for (side, horizontal) in
            [(if left { 0 } else { 2 }, true), (if top { 3 } else { 1 }, false)]
        {
            for k in [1usize, 4, 7, 10] {
                let net = b.net(&format!("P{outer}"));
                outer += 1;
                nets[qi][side * 12 + k] = net;
                let s = (2 * k as i64 - 11) * 500_000 / 2;
                let reach = 5_500_000;
                let (pos, rot) = if horizontal {
                    let dx = if left { -reach } else { reach };
                    (Point::new(c.x + dx, c.y + if side == 0 { s } else { -s }), 0.0)
                } else {
                    let dy = if top { -reach } else { reach };
                    // Vertical column of passives: pads stack along y.
                    let sx = if side == 1 { s } else { -s };
                    (Point::new(c.x + sx, c.y + dy), 90.0)
                };
                let other = if rng.below(2) == 0 { gnd } else { vcc };
                passives.push((pos, rot, net, other));
            }
        }
    }
    for (qi, &c) in centres.iter().enumerate() {
        let mut pads = pads_qfn(12, 500_000);
        for (p, &net) in pads.iter_mut().zip(&nets[qi]) {
            p.net = net;
        }
        let id = b.footprint(&format!("U{}", qi + 1), c, 0.0, Side::Front, &pads);
        b.mobility(id, Mobility::Locked);
    }
    for (k, &(pos, rot, a, c)) in passives.iter().enumerate() {
        passive(&mut b, &format!("R{}", k + 1), pos, rot, false, a, c);
    }
    let mut board = b.build();
    add_plane(&mut board, gnd, 1);
    refill(board)
}

// ---------------------------------------------------------------------------------------
// Mixed 8-layer board
// ---------------------------------------------------------------------------------------

/// 120 × 86 mm, 8 layers, ~300 nets: random 2–4 pin nets over ~400 parts, three
/// differential pairs and a five-net length-matched bus in a bottom strip. Returns the
/// board and the equivalent rules file.
#[must_use]
pub fn mixed_8l(seed: u64) -> (Board, String) {
    let spec = SynthSpec {
        name: "mixed_8l".into(),
        width_mm: 120,
        height_mm: 86,
        layers: 8,
        parts: 440,
        headers: 4,
        nets: 300,
        locked_fraction: 0.15,
        spacing_mm: 4,
        power_net: true,
        seed,
        strip_mm: 18,
    };
    let extras = |b: &mut BoardBuilder| -> Extras {
        set_rules(b, 150_000, 150_000, 500_000, 250_000);
        let net = |b: &mut BoardBuilder, n: &str| b.net(n);
        let pair_nets: Vec<NetId> = ["USB0_P", "USB0_N", "USB1_P", "USB1_N", "USB2_P", "USB2_N"]
            .iter()
            .map(|n| net(b, n))
            .collect();
        let bus: Vec<NetId> = ["B0", "B1", "B2", "B3", "CLK"].iter().map(|n| net(b, n)).collect();
        let j1 = b.header("J1", Point::new(mm(8), mm(70)), 0.0, &pair_nets);
        let mut u1 = pads_soic8();
        for (i, p) in pair_nets[..4].iter().enumerate() {
            u1[i].net = *p;
        }
        let id = b.footprint("U1", Point::new(mm(36), mm(74)), 0.0, Side::Front, &u1);
        b.mobility(id, Mobility::Locked);
        let mut u2 = pads_soic8();
        u2[0].net = pair_nets[4];
        u2[1].net = pair_nets[5];
        let id = b.footprint("U2", Point::new(mm(36), mm(82)), 0.0, Side::Front, &u2);
        b.mobility(id, Mobility::Locked);
        let j2 = b.header("J2", Point::new(mm(70), mm(70)), 0.0, &bus);
        let permuted = [bus[1], bus[0], bus[2], bus[4], bus[3]];
        let j3 = b.header("J3", Point::new(mm(100), mm(72)), 0.0, &permuted);
        for h in [j1, j2, j3] {
            b.mobility(h, Mobility::Locked);
        }
        let pairs: Vec<DiffPair> = pair_nets
            .chunks(2)
            .map(|c| DiffPair {
                p: c[0],
                n: c[1],
                gap: 200_000,
                width: Some(150_000),
                max_uncoupled: None,
                skew_max: Some(100_000),
            })
            .collect();
        let mut groups: Vec<MatchGroup> = pairs
            .iter()
            .enumerate()
            .map(|(i, d)| MatchGroup {
                name: format!("diffpair:USB{i}"),
                skew_max: 100_000,
                by: MatchBy::Length,
                nets: vec![d.p.min(d.n), d.p.max(d.n)],
            })
            .collect();
        let mut members = bus.clone();
        members.sort();
        groups.push(MatchGroup {
            name: "bus".into(),
            skew_max: 500_000,
            by: MatchBy::Length,
            nets: members,
        });
        Extras { diff_pairs: pairs, match_groups: groups }
    };
    let mut board = generate_with(&spec, &extras);
    let gnd = board.net_by_name("GND").unwrap_or(NetId::NONE);
    add_plane(&mut board, gnd, 2);
    add_plane(&mut board, gnd, 5);
    let board = refill(board);
    let mut toml = String::new();
    for i in 0..3 {
        toml.push_str(&format!(
            "[[pair]]\nnets = [\"USB{i}_P\", \"USB{i}_N\"]\ngap_mm = 0.2\nwidth_mm = 0.15\nskew_mm = 0.1\n"
        ));
    }
    toml.push_str("[[match]]\nname = \"bus\"\nnets = [\"B0\", \"B1\", \"B2\", \"B3\", \"CLK\"]\nskew_mm = 0.5\n");
    (board, toml)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stats(b: &Board) -> (usize, usize, usize) {
        let nets = (1..b.nets.len())
            .filter(|&i| b.pads.iter().filter(|p| p.net.idx() == i).count() >= 2)
            .count();
        (b.footprints.len(), b.pads.iter().filter(|p| p.net != NetId::NONE).count(), nets)
    }

    #[test]
    fn dense_boards_are_deterministic_valid_and_drc_clean() {
        for sb in dense_suite() {
            sb.board.validate().expect("valid");
            let again = dense_suite().into_iter().find(|x| x.name == sb.name).expect("same suite");
            assert_eq!(sb.board, again.board, "{} not deterministic", sb.name);
            assert_eq!(sb.rules_toml, again.rules_toml);
            let rep = ferroute_drc::check(&sb.board, &ferroute_drc::DrcConfig::default());
            assert!(
                rep.is_clean(),
                "{}: {:#?}",
                sb.name,
                &rep.violations[..rep.violations.len().min(8)]
            );
        }
    }

    #[test]
    fn seeds_change_the_boards() {
        assert_ne!(bga_256_4l(1), bga_256_4l(2));
        assert_ne!(qfn_dense_2l(1), qfn_dense_2l(2));
    }

    #[test]
    fn dense_boards_have_the_specified_structure() {
        let bga = |b: &Board| b.footprints[0].pads.len();
        let b256 = bga_256_4l(21);
        assert_eq!(b256.layer_count(), 4);
        assert_eq!(bga(&b256), 256);
        let signal_balls = b256.footprints[0]
            .pads
            .iter()
            .filter(|&&p| b256.nets[b256.pads[p.idx()].net.idx()].name.starts_with('S'))
            .count();
        assert!((140..=170).contains(&signal_balls), "{signal_balls} signal balls (~60 %)");
        assert_eq!(b256.zones.len(), 2);
        let b484 = bga_484_6l(22);
        assert_eq!((b484.layer_count(), bga(&b484), b484.zones.len()), (6, 484, 2));
        let q = qfn_dense_2l(23);
        assert_eq!(q.layer_count(), 2);
        assert_eq!(q.footprints.iter().filter(|f| f.pads.len() == 49).count(), 4);
        let (m, toml) = mixed_8l(24);
        assert_eq!(m.layer_count(), 8);
        assert_eq!(m.diff_pairs.len(), 3);
        assert!(m.match_groups.iter().any(|g| g.name == "bus" && g.nets.len() == 5));
        assert!(toml.contains("[[pair]]") && toml.contains("[[match]]"));
        // (footprints, connected pads, nets with >= 2 pads): frozen with the generator.
        assert_eq!(stats(&b256), (127, 516, 145));
        assert_eq!(stats(&b484), (121, 722, 139));
        assert_eq!(stats(&q), (36, 212, 74));
        assert_eq!(stats(&m), (449, 893, 305));
    }
}
