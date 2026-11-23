// SPDX-License-Identifier: AGPL-3.0-only
//! Detour-driven net-order refinement on the bus_match fixture (fixed placement).

use ferroute_router::{RouterConfig, detour, route};

#[test]
fn bus_match_order_refinement_reduces_detours() {
    let kb = ferroute_kicad::read_board_file(std::path::Path::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../testdata/boards/bus_match.kicad_pcb"
    )))
    .expect("read");
    let mut board = kb.board;
    let rules = std::fs::read_to_string(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../testdata/rules/bus_match.toml"
    ))
    .expect("rules file");
    let rs: ferroute_board::netrules::RuleSet = toml::from_str(&rules).expect("rules");
    rs.apply(&mut board).expect("apply");
    let run = |order_rounds: u32| {
        // The order refinement is measured in isolation (no neighbourhood re-route).
        let lns = ferroute_router::lns::LnsConfig { rounds: 0, ..Default::default() };
        let out = route(&board, &RouterConfig { order_rounds, lns, ..RouterConfig::default() })
            .expect("route");
        let mut after = board.clone();
        after.apply(&out.edit);
        let rep = ferroute_drc::check(&after, &ferroute_drc::DrcConfig::default());
        assert_eq!(rep.introduced().count(), 0);
        assert_eq!(out.stats.drc_gate_rips, 0);
        assert!(out.failed.is_empty(), "completion must stay 100 %");
        out
    };
    let (off, on) = (run(0), run(3));
    assert!(on.stats.order_rounds_run > 0);
    assert!(detour::quality(&board, &on) < detour::quality(&board, &off));
    assert!(on.stats.detour_excess_mm <= off.stats.detour_excess_mm);
}
