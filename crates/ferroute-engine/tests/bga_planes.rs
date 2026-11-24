// SPDX-License-Identifier: AGPL-3.0-only
//! Plane nets of a BGA reach their inner-layer pours through dog-bone vias.

use ferroute_engine::{PipelineConfig, run, synth};
use ferroute_router::{Accel, RouterConfig};

#[test]
fn bga_power_balls_connect_to_their_planes() {
    let board = synth::bga_planes();
    let cfg = PipelineConfig {
        router: RouterConfig { accel: Accel::Cpu, max_iterations: 20, ..RouterConfig::default() },
        lns_rounds: 0,
        refill_zones: true,
        ..PipelineConfig::default()
    };
    let r = run(&board, &cfg).expect("run");
    assert!(r.metrics.drc_clean, "{:?}", r.drc.introduced().collect::<Vec<_>>());
    assert_eq!(r.metrics.connections_missing, 0, "{:?}", r.drc.connectivity.unconnected);
    for name in ["GND", "VCC"] {
        let net = board.net_by_name(name).expect("net");
        let vias: Vec<_> = r.board_after.vias.iter().filter(|v| v.net == net).collect();
        assert!(!vias.is_empty(), "{name} reaches its plane through a via");
        let zone = board.zones.iter().find(|z| z.net == net).expect("zone");
        let fill = &zone.fills[0].polys[0];
        for v in vias {
            assert!(fill.contains_disk(v.pos, v.diameter / 2), "{name} via inside its pour");
            assert!(r.board_after.via_in_pad(v).is_none(), "no via in a pad");
        }
    }
}
