// SPDX-License-Identifier: AGPL-3.0-only
//! Plane nets give every ball its own dog-bone via; balls are never chained on signal layers.

use ferroute_board::builder::{BoardBuilder, PadSpec, mm};
use ferroute_board::{
    Board, IslandRemoval, LayerId, LayerSet, Mobility, NetId, PadConnection, PadShape, Side, Zone,
    ZoneFill,
};
use ferroute_engine::{PipelineConfig, PipelineResult, run, synth};
use ferroute_geom::{Dir8, Point, Polygon, Rect};
use ferroute_router::{Accel, RouterConfig};
use std::collections::BTreeMap;

fn cfg() -> PipelineConfig {
    PipelineConfig {
        router: RouterConfig { accel: Accel::Cpu, max_iterations: 20, ..RouterConfig::default() },
        lns_rounds: 0,
        refill_zones: true,
        ..PipelineConfig::default()
    }
}

/// 4×4 BGA (1 mm pitch) whose balls with `(row + col)` even are GND, over a GND pour on In1.
fn small_bga() -> Board {
    let mut b = BoardBuilder::new(mm(16), mm(16), 4);
    {
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
    let centre = Point::new(mm(8), mm(8));
    let mut pads = Vec::new();
    for r in 0..4i64 {
        for c in 0..4i64 {
            pads.push(PadSpec {
                number: format!("{}{}", char::from(b'A' + u8::try_from(r).unwrap_or(0)), c + 1),
                local_pos: Point::new((2 * c - 3) * mm(1) / 2, (2 * r - 3) * mm(1) / 2),
                shape: PadShape::Circle { diameter: 500_000 },
                drill: None,
                net: if (r + c) % 2 == 0 { gnd } else { NetId::NONE },
            });
        }
    }
    let u1 = b.footprint("U1", centre, 0.0, Side::Front, &pads);
    b.mobility(u1, Mobility::Locked);
    let mut board = b.build();
    let poly = Polygon::from_rect(Rect::new(Point::new(mm(1), mm(1)), Point::new(mm(15), mm(15))));
    board.zones.push(Zone {
        net: gnd,
        layers: LayerSet::single(LayerId(1)),
        outline: poly.clone(),
        fills: vec![ZoneFill { layer: LayerId(1), polys: vec![poly] }],
        clearance: 100_000,
        min_thickness: 150_000,
        priority: 0,
        pad_connection: PadConnection::ThermalRelief,
        islands: IslandRemoval::Always,
        yields: false,
    });
    board
}

fn find(parent: &mut [usize], mut x: usize) -> usize {
    while parent[x] != x {
        parent[x] = parent[parent[x]];
        x = parent[x];
    }
    x
}

/// Number of `net` pads in each cluster of pads joined by `net` tracks (not via a pour).
fn pad_clusters(board: &Board, net: NetId) -> Vec<usize> {
    let mut ids: BTreeMap<(i64, i64), usize> = BTreeMap::new();
    let mut parent: Vec<usize> = Vec::new();
    let mut key = |p: Point| -> usize {
        let next = parent.len();
        let k = *ids.entry((p.x, p.y)).or_insert(next);
        if k == next {
            parent.push(next);
        }
        k
    };
    let mut edges: Vec<(usize, usize)> = Vec::new();
    for t in board.tracks.iter().filter(|t| t.net == net) {
        edges.push((key(t.start), key(t.end)));
    }
    let pad_keys: Vec<usize> =
        board.pads_by_net()[net.idx()].iter().map(|&p| key(board.pad_position(p))).collect();
    for (a, b) in edges {
        let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
        parent[ra] = rb;
    }
    let mut count: BTreeMap<usize, usize> = BTreeMap::new();
    for k in pad_keys {
        *count.entry(find(&mut parent, k)).or_insert(0) += 1;
    }
    count.into_values().collect()
}

fn assert_dogbones(board: &Board, r: &PipelineResult, net: NetId) {
    let after = &r.board_after;
    assert!(pad_clusters(after, net).iter().all(|&n| n == 1), "a track chains two plane pads");
    let pads = &board.pads_by_net()[net.idx()];
    let vias: Vec<_> = after.vias.iter().filter(|v| v.net == net).collect();
    assert_eq!(vias.len(), pads.len(), "one dog-bone via per ball");
    let fill = &board.zones.iter().find(|z| z.net == net).expect("zone").fills[0].polys[0];
    for v in &vias {
        assert!(fill.contains_disk(v.pos, v.diameter / 2), "via inside the pour");
    }
}

#[test]
fn small_bga_gives_every_ball_its_own_via() {
    let board = small_bga();
    let r = run(&board, &cfg()).expect("run");
    assert!(r.metrics.drc_clean, "{:?}", r.drc.introduced().collect::<Vec<_>>());
    assert_eq!(r.metrics.connections_missing, 0, "{:?}", r.drc.connectivity.unconnected);
    let gnd = board.net_by_name("GND").expect("net");
    assert_dogbones(&board, &r, gnd);
    for t in r.board_after.tracks.iter().filter(|t| t.net == gnd) {
        assert!(Dir8::from_vector(t.end - t.start).is_some(), "octilinear");
    }
}

#[test]
fn bga_planes_balls_are_never_chained() {
    let board = synth::bga_planes();
    let r = run(&board, &cfg()).expect("run");
    assert!(r.metrics.drc_clean);
    assert_eq!(r.metrics.connections_missing, 0);
    for name in ["GND", "VCC"] {
        let net = board.net_by_name(name).expect("net");
        assert_dogbones(&board, &r, net);
    }
}
