// SPDX-License-Identifier: AGPL-3.0-only
//! # ferroute-engine
//!
//! Transport-agnostic orchestration of a full ferroute run:
//!
//! 1. apply the movability policy (locked parts are untouchable),
//! 2. placement-aware nudging of movable parts (seeded multi-start annealing),
//! 3. PathFinder routing,
//! 4. large-neighbourhood rounds: re-nudge with failed nets up-weighted and re-route,
//!    keeping the best candidate,
//! 5. length tuning (meanders) for `length_min` / length-matching rules,
//! 6. copper-pour refill: stored zone fills are recomputed around the new copper
//!    ([`ferroute_drc::refill`]); until then other-net fills yield to the router,
//! 7. the deterministic DRC oracle as the final acceptance gate.
//!
//! Also hosts the synthetic board generator ([`synth`]) and benchmark scoring
//! ([`score`]) used by the CI regression gate.

pub mod score;
pub mod synth;

use ferroute_board::{Board, BoardEdit, FootprintMove, NetId};
use ferroute_drc::rules::ConstraintIssue;
use ferroute_drc::{DrcConfig, DrcReport};
use ferroute_place::{MovePolicy, PlaceConfig, PlaceResult};
use ferroute_router::{RouteError, RouteOutcome, RouteStats, RouterConfig};
use serde::{Deserialize, Serialize};
use std::time::Instant;

/// Pipeline configuration.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PipelineConfig {
    pub seed: u64,
    pub router: RouterConfig,
    /// `Some` enables placement co-optimisation.
    pub nudge: Option<(MovePolicy, PlaceConfig)>,
    /// Extra nudge → re-route rounds when nets remain unrouted.
    pub lns_rounds: u32,
    /// HPWL weight multiplier for nets that failed in the previous round.
    pub failed_net_weight: f64,
    /// Refill copper pours after routing: other-net fills then yield to routed copper
    /// instead of being obstacles (`--no-refill-zones` keeps them as obstacles).
    #[serde(default = "default_true")]
    pub refill_zones: bool,
}

const fn default_true() -> bool {
    true
}

impl Default for PipelineConfig {
    fn default() -> Self {
        Self {
            seed: 1,
            router: RouterConfig::default(),
            nudge: None,
            lns_rounds: 2,
            failed_net_weight: 6.0,
            refill_zones: true,
        }
    }
}

/// Quality metrics of a run (all deterministic except `runtime_ms`).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Metrics {
    /// No DRC violation introduced by ferroute (the hard gate).
    pub drc_clean: bool,
    pub introduced_violations: usize,
    /// All violations on the output board, including pre-existing ones.
    pub total_violations: usize,
    pub completion: f64,
    pub connections_total: usize,
    pub connections_missing: usize,
    pub wirelength_mm: f64,
    pub vias: usize,
    pub moved_footprints: usize,
    pub displacement_mm: f64,
    /// Per-net electrical rules the result does not meet (reported, never hidden).
    #[serde(default)]
    pub constraint_issues: usize,
    pub runtime_ms: f64,
}

/// Full pipeline result.
#[derive(Clone, Debug)]
pub struct PipelineResult {
    /// Internal: the refill split a pour net (the pipeline then re-runs without refill).
    pub refill_disconnects: bool,
    /// The input board with the movability policy applied (edit invariants are checked
    /// against this board).
    pub policy_board: Board,
    /// Footprint moves + new copper relative to `policy_board`.
    pub edit: BoardEdit,
    pub board_after: Board,
    pub route: RouteStats,
    pub place: Option<PlaceResult>,
    pub drc: DrcReport,
    pub metrics: Metrics,
    pub failed_nets: Vec<NetId>,
    pub rounds: u32,
    /// How per-net rules changed derived widths before routing.
    pub rule_notes: Vec<String>,
    /// Per-net electrical constraints the routed board does not meet.
    pub constraint_issues: Vec<ConstraintIssue>,
}

struct Candidate {
    placed: Board,
    moves: Vec<FootprintMove>,
    out: RouteOutcome,
    place: Option<PlaceResult>,
}

/// Weight of one via in the candidate cost, as equivalent trace length (µm).
const VIA_COST_UM: u64 = 4_000;
/// Weight of one routing-quality defect (sliver, right angle, via jog, pad-edge bend).
const DEFECT_COST_UM: u64 = 2_000;

fn quality(c: &Candidate) -> (usize, u64) {
    // Fewer failed nets first; then one weighted cost an expert would use: wire length plus
    // a few millimetres per via and per quality defect (vias and defects are not worth any
    // amount of extra wire).
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let wl = (c.out.stats.wirelength_nm / 1e3) as u64;
    let st = &c.out.stats;
    let defects = (st.slivers + st.right_angle_bends + st.via_jogs + st.near_pad_bends) as u64;
    let cost = wl + st.vias as u64 * VIA_COST_UM + defects * DEFECT_COST_UM;
    (c.out.failed.len(), cost)
}

fn moves_between(policy_board: &Board, placed: &Board) -> Vec<FootprintMove> {
    policy_board
        .footprints
        .iter()
        .zip(&placed.footprints)
        .enumerate()
        .filter(|(_, (a, b))| a.placement != b.placement)
        .map(|(i, (a, b))| FootprintMove {
            footprint: ferroute_board::FootprintId(u32::try_from(i).unwrap_or(u32::MAX)),
            from: a.placement,
            to: b.placement,
        })
        .collect()
}

/// Runs the pipeline.
///
/// # Errors
/// Router errors (invalid board, impossible lattice).
pub fn run(board: &Board, cfg: &PipelineConfig) -> Result<PipelineResult, RouteError> {
    let first = run_once(board, cfg)?;
    if !first.refill_disconnects {
        return Ok(first);
    }
    // INV-CONN-1: refilling around the new copper would split a pour and disconnect pads
    // of its net. Keep the pours intact (obstacle mode) instead, and say so.
    let mut res = run_once(board, &PipelineConfig { refill_zones: false, ..cfg.clone() })?;
    res.rule_notes.push(
        "copper pours kept as obstacles: refill would disconnect a pour net (INV-CONN-1)"
            .to_owned(),
    );
    Ok(res)
}

fn missing_per_net(c: &ferroute_drc::Connectivity) -> std::collections::BTreeMap<NetId, usize> {
    let mut m = std::collections::BTreeMap::new();
    for u in &c.unconnected {
        *m.entry(u.net).or_insert(0) += 1;
    }
    m
}

fn run_once(board: &Board, cfg: &PipelineConfig) -> Result<PipelineResult, RouteError> {
    let t0 = Instant::now();
    let mut policy_board = board.clone();
    if let Some((policy, _)) = &cfg.nudge {
        ferroute_place::apply_policy(&mut policy_board, policy);
    } else {
        for fp in &mut policy_board.footprints {
            fp.mobility = if fp.locked {
                ferroute_board::Mobility::Locked
            } else {
                ferroute_board::Mobility::Fixed
            };
        }
    }
    for z in &mut policy_board.zones {
        z.yields = cfg.refill_zones && !z.is_unfilled();
    }
    // Per-net electrical rules → derived widths, before anything routes (INV-RULE-1).
    let rule_notes = ferroute_board::constraints::resolve(&mut policy_board);
    let router = RouterConfig { seed: cfg.seed, ..cfg.router.clone() };
    let nudge =
        |from: &Board, weights: Vec<(NetId, f64)>, round: u32| -> (Board, Option<PlaceResult>) {
            let Some((_, pcfg)) = &cfg.nudge else { return (from.clone(), None) };
            let pc = PlaceConfig {
                seed: ferroute_geom::rng::splitmix64(cfg.seed ^ u64::from(round)),
                net_weights: weights,
                ..pcfg.clone()
            };
            let r = ferroute_place::optimize(from, &pc);
            let mut placed = from.clone();
            for m in &r.moves {
                placed.footprints[m.footprint.idx()].placement = m.to;
            }
            (placed, Some(r))
        };
    let (placed, place) = nudge(&policy_board, Vec::new(), 0);
    let out = ferroute_router::route(&placed, &router)?;
    let mut best = Candidate { moves: moves_between(&policy_board, &placed), placed, out, place };
    let mut rounds = 1;
    if cfg.nudge.is_some() {
        for round in 1..=cfg.lns_rounds {
            if best.out.failed.is_empty() {
                break;
            }
            rounds += 1;
            let weights = best.out.failed.iter().map(|&n| (n, cfg.failed_net_weight)).collect();
            let (placed, place) = nudge(&best.placed, weights, round);
            let out = ferroute_router::route(
                &placed,
                &RouterConfig { seed: cfg.seed.wrapping_add(u64::from(round)), ..router.clone() },
            )?;
            let cand =
                Candidate { moves: moves_between(&policy_board, &placed), placed, out, place };
            if quality(&cand) < quality(&best) {
                best = cand;
            }
        }
    }
    if cfg.nudge.is_some() && !best.moves.is_empty() {
        // Nudging may only help: after the LNS rounds, the un-nudged placement competes too.
        let out = ferroute_router::route(&policy_board, &router)?;
        let fixed = Candidate { moves: Vec::new(), placed: policy_board.clone(), out, place: None };
        if quality(&fixed) < quality(&best) {
            best = fixed;
        }
    }
    let mut edit = BoardEdit {
        new_arcs: Vec::new(),
        moves: best.moves.clone(),
        new_tracks: best.out.edit.new_tracks.clone(),
        new_vias: best.out.edit.new_vias.clone(),
        zone_fills: Vec::new(),
    };
    // Length rules (length_min, match groups): meanders, each verified by the oracle.
    let mut rule_notes = rule_notes;
    rule_notes.extend(ferroute_router::tune::tune_lengths(&policy_board, &mut edit));
    for w in &best.out.stats.uncoupled_pairs {
        rule_notes.push(format!("pair {w} — routed as two independent nets"));
    }
    let mut board_after = policy_board.clone();
    board_after.apply(&edit);
    let mut refill_disconnects = false;
    if cfg.refill_zones {
        let only_conn = DrcConfig {
            copper: false,
            edge: false,
            holes: false,
            keepouts: false,
            courtyards: false,
            sizes: false,
            ..DrcConfig::default()
        };
        let before = missing_per_net(&ferroute_drc::check(&board_after, &only_conn).connectivity);
        let refill = ferroute_drc::refill::refill_zones(&board_after);
        rule_notes.extend(refill.notes);
        edit.zone_fills = refill.fills;
        board_after
            .apply(&BoardEdit { zone_fills: edit.zone_fills.clone(), ..BoardEdit::default() });
        for z in &mut board_after.zones {
            z.yields = false;
        }
        let after = missing_per_net(&ferroute_drc::check(&board_after, &only_conn).connectivity);
        refill_disconnects = after.iter().any(|(n, &c)| c > before.get(n).copied().unwrap_or(0));
    }
    debug_assert!(edit.check_invariants(&policy_board).is_ok());
    let drc = ferroute_drc::check(&board_after, &DrcConfig::default());
    let introduced = drc.introduced().count();
    let constraint_issues = ferroute_drc::rules::check_constraints(&board_after);
    let metrics = Metrics {
        drc_clean: introduced == 0,
        introduced_violations: introduced,
        total_violations: drc.violations.len(),
        completion: drc.connectivity.completion(),
        connections_total: drc.connectivity.connections_total,
        connections_missing: drc.connectivity.connections_missing,
        wirelength_mm: best.out.stats.wirelength_nm / 1e6,
        vias: best.out.stats.vias,
        moved_footprints: edit.moves.len(),
        displacement_mm: edit
            .moves
            .iter()
            .map(|m| m.to.pos.dist(policy_board.footprints[m.footprint.idx()].original.pos))
            .sum::<f64>()
            / 1e6
            + 0.0, // normalise -0.0 (empty f64 sums are -0.0)
        constraint_issues: constraint_issues.len(),
        runtime_ms: t0.elapsed().as_secs_f64() * 1e3,
    };
    Ok(PipelineResult {
        refill_disconnects,
        policy_board,
        edit,
        board_after,
        route: best.out.stats.clone(),
        place: best.place,
        drc,
        metrics,
        failed_nets: best.out.failed,
        rounds,
        rule_notes,
        constraint_issues,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn synthetic_suite_is_clean_and_deterministic() {
        for spec in synth::suite("ci").into_iter().take(2) {
            let board = synth::generate(&spec);
            board.validate().expect("valid synthetic board");
            let cfg = PipelineConfig {
                router: RouterConfig {
                    accel: ferroute_router::Accel::Cpu,
                    ..RouterConfig::default()
                },
                nudge: Some((
                    MovePolicy::default(),
                    PlaceConfig { moves_per_fp: 100, ..PlaceConfig::default() },
                )),
                ..PipelineConfig::default()
            };
            let a = run(&board, &cfg).expect("run");
            assert!(
                a.metrics.drc_clean,
                "{}: {:#?}",
                spec.name,
                a.drc.introduced().collect::<Vec<_>>()
            );
            a.edit.check_invariants(&a.policy_board).expect("INV-EDIT-1");
            let b = run(&board, &cfg).expect("run");
            assert_eq!(a.edit, b.edit, "non-deterministic pipeline on {}", spec.name);
        }
    }
}

#[cfg(test)]
mod rule_tests {
    use super::*;
    use ferroute_board::builder::{BoardBuilder, mm, pt10};
    use ferroute_board::constraints::{required_width, width_for_current};
    use ferroute_drc::rules::IssueKind;

    fn two_parts() -> (Board, NetId) {
        let mut b = BoardBuilder::new(mm(30), mm(20), 2);
        let p = b.net("VBUS");
        let g = b.net("GND");
        b.smd2("R1", pt10(80, 100), 0.0, p, g);
        b.smd2("R2", pt10(220, 100), 0.0, p, g);
        (b.build(), p)
    }

    fn cfg() -> PipelineConfig {
        PipelineConfig {
            router: RouterConfig { accel: ferroute_router::Accel::Cpu, ..RouterConfig::default() },
            ..PipelineConfig::default()
        }
    }

    #[test]
    fn current_rules_widen_routed_traces_never_narrow() {
        let (mut board, p) = two_parts();
        board.nets[p.idx()].constraints.max_current_ma = Some(2000);
        let r = run(&board, &cfg()).expect("run");
        assert!(r.metrics.drc_clean);
        assert_eq!(r.metrics.completion, 1.0);
        let need = width_for_current(2000, 35_000, true, 10.0);
        assert_eq!(required_width(&r.board_after, p), need);
        let vbus: Vec<_> = r.edit.new_tracks.iter().filter(|t| t.net == p).collect();
        assert!(!vbus.is_empty());
        assert!(vbus.iter().all(|t| t.width >= need), "INV-RULE-1");
        assert_eq!(r.rule_notes.len(), 1, "{:?}", r.rule_notes);
        assert!(r.constraint_issues.is_empty(), "{:?}", r.constraint_issues);
    }

    #[test]
    fn unmet_rules_are_reported_not_silently_violated() {
        let (mut board, p) = two_parts();
        board.nets[p.idx()].constraints.length_max = Some(mm(1));
        let r = run(&board, &cfg()).expect("run");
        assert_eq!(r.metrics.constraint_issues, 1);
        assert!(matches!(
            r.constraint_issues[0].kind,
            IssueKind::Length { max: Some(m), .. } if m == mm(1)
        ));
    }
}

#[cfg(test)]
mod nudge_tests {
    use super::*;
    use ferroute_board::Mobility;
    use ferroute_board::builder::{BoardBuilder, mm, pt10};
    use ferroute_geom::{Point, Rect};

    /// R1's net-A pad sits inside a no-track keep-out: unroutable where it is, routable
    /// once the unlocked passive is nudged (≤ 2 mm) out of it. J1 is locked.
    fn must_move_board() -> Board {
        let mut b = BoardBuilder::new(mm(30), mm(20), 2);
        let a = b.net("A");
        let j1 = b.header("J1", pt10(50, 100), 0.0, &[a]);
        b.mobility(j1, Mobility::Locked);
        b.smd2("R1", pt10(200, 100), 0.0, a, NetId::NONE);
        b.keepout(
            Rect::new(Point::new(mm(18) + 500_000, 0), Point::new(mm(30), mm(20))),
            true,
            true,
        );
        b.build()
    }

    fn cfg(nudge: bool) -> PipelineConfig {
        PipelineConfig {
            router: RouterConfig { accel: ferroute_router::Accel::Cpu, ..RouterConfig::default() },
            nudge: nudge.then(|| (MovePolicy::default(), PlaceConfig::default())),
            ..PipelineConfig::default()
        }
    }

    #[test]
    fn connection_routes_only_after_nudging_an_unlocked_passive() {
        let board = must_move_board();
        board.validate().expect("valid");
        let fixed = run(&board, &cfg(false)).expect("run");
        assert!(fixed.metrics.drc_clean);
        assert_eq!(fixed.metrics.connections_missing, 1, "blocked without moving");
        let moved = run(&board, &cfg(true)).expect("run");
        assert!(moved.metrics.drc_clean, "{:?}", moved.drc.introduced().collect::<Vec<_>>());
        assert_eq!(moved.metrics.connections_missing, 0, "routes once R1 moves");
        moved.edit.check_invariants(&moved.policy_board).expect("INV-EDIT-1");
        assert!(moved.edit.moves.iter().all(|m| m.footprint.idx() == 1), "J1 locked");
        assert!(!moved.edit.moves.is_empty());
        let grid = PlaceConfig::default().grid;
        for m in &moved.edit.moves {
            assert_eq!(m.to.pos.x % grid, 0, "INV-PLACE-3");
            assert_eq!(m.to.pos.y % grid, 0, "INV-PLACE-3");
        }
    }
}

#[cfg(test)]
mod tune_tests {
    use super::*;
    use ferroute_board::builder::{BoardBuilder, mm, pt10};
    use ferroute_drc::rules::net_lengths;

    /// Two signals from a locked connector to two resistors at different distances,
    /// matched within 0.5 mm: the pipeline must meander the shorter one.
    #[test]
    fn pipeline_meets_length_matching_with_meanders() {
        let mut b = BoardBuilder::new(mm(45), mm(25), 2);
        let p = b.net("D_P");
        let n = b.net("D_N");
        let j = b.header("J1", pt10(40, 80), 0.0, &[p, n]);
        b.mobility(j, ferroute_board::Mobility::Locked);
        b.smd2("R1", pt10(400, 80), 0.0, p, NetId::NONE);
        b.smd2("R2", pt10(250, 105), 0.0, n, NetId::NONE);
        let mut board = b.build();
        for id in [p, n] {
            let c = &mut board.nets[id.idx()].constraints;
            c.match_group = Some("pair".into());
            c.skew_max = Some(mm(1) / 2);
        }
        let cfg = PipelineConfig {
            router: RouterConfig { accel: ferroute_router::Accel::Cpu, ..RouterConfig::default() },
            ..PipelineConfig::default()
        };
        let r = run(&board, &cfg).expect("run");
        assert!(r.metrics.drc_clean, "{:?}", r.drc.introduced().collect::<Vec<_>>());
        assert_eq!(r.metrics.connections_missing, 0);
        assert!(r.constraint_issues.is_empty(), "{:?}", r.constraint_issues);
        assert!(r.rule_notes.iter().any(|n| n.contains("length tuned")), "{:?}", r.rule_notes);
        assert!(!r.edit.new_arcs.is_empty(), "meander corners are arcs");
        assert!(ferroute_drc::quality::acute_angles(&r.board_after).is_empty());
        let l = net_lengths(&r.board_after);
        assert!((l[p.idx()] - l[n.idx()]).abs() <= mm(1) / 2);
        r.edit.check_invariants(&r.policy_board).expect("INV-EDIT-1");
        let again = run(&board, &cfg).expect("run");
        assert_eq!(r.edit, again.edit, "deterministic");
    }
}

#[cfg(test)]
mod bus_match_tests {
    use super::*;
    use ferroute_board::constraints::MatchGroup;
    use ferroute_drc::rules::net_lengths;

    /// Nested groups (two pairs within 0.1 mm, CLK + both pairs within 0.5 mm) on the
    /// `bus_match` fixture, with the groups set programmatically.
    #[test]
    fn nested_bus_matching_is_met_in_fixed_and_nudging_mode() {
        let mut board = synth::bus_match();
        let id = |n: &str| board.net_by_name(n).expect("net");
        let (d0p, d0n, d1p, d1n, clk) = (id("D0_P"), id("D0_N"), id("D1_P"), id("D1_N"), id("CLK"));
        let mm = ferroute_board::builder::mm;
        let g = |name: &str, skew, nets: &[NetId]| MatchGroup {
            name: name.into(),
            skew_max: skew,
            by: ferroute_board::constraints::MatchBy::Length,
            nets: nets.to_vec(),
        };
        board.match_groups = vec![
            g("pair0", mm(1) / 10, &[d0p, d0n]),
            g("pair1", mm(1) / 10, &[d1p, d1n]),
            g("bus", mm(1) / 2, &[d0p, d0n, d1p, d1n, clk]),
        ];
        for nudge in [false, true] {
            let cfg = PipelineConfig {
                router: RouterConfig {
                    accel: ferroute_router::Accel::Cpu,
                    ..RouterConfig::default()
                },
                nudge: nudge.then(|| (MovePolicy::default(), PlaceConfig::default())),
                ..PipelineConfig::default()
            };
            let r = run(&board, &cfg).expect("run");
            let tag = format!("nudge={nudge}");
            assert!(r.metrics.drc_clean, "{tag}: {:?}", r.drc.introduced().collect::<Vec<_>>());
            assert!((r.metrics.completion - 1.0).abs() < 1e-9, "{tag}");
            assert!(r.constraint_issues.is_empty(), "{tag}: {:?}", r.constraint_issues);
            assert!(!r.edit.new_arcs.is_empty(), "{tag}: meander corners are arcs");
            let l = net_lengths(&r.board_after);
            let spread = |n: &[NetId]| {
                let v: Vec<_> = n.iter().map(|x| l[x.idx()]).collect();
                v.iter().max().unwrap() - v.iter().min().unwrap()
            };
            assert!(spread(&[d0p, d0n]) <= mm(1) / 10, "{tag}");
            assert!(spread(&[d1p, d1n]) <= mm(1) / 10, "{tag}");
            assert!(spread(&[d0p, d0n, d1p, d1n, clk]) <= mm(1) / 2, "{tag}");
        }
    }
}

#[cfg(test)]
mod refill_tests {
    use super::*;
    use ferroute_board::builder::{BoardBuilder, mm, pt10};
    use ferroute_board::{IslandRemoval, LayerId, LayerSet, PadConnection, Zone, ZoneFill};
    use ferroute_geom::Polygon;

    fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Polygon {
        Polygon::new(vec![pt10(x0, y0), pt10(x1, y0), pt10(x1, y1), pt10(x0, y1)])
    }

    /// `S` joins R1 (x 5 mm) to R2 (x 25 mm) on F.Cu only, but a GND pour fills the whole
    /// board height between x 11 and 19 mm: the only corridor.
    fn blocked_board() -> Board {
        let mut b = BoardBuilder::new(mm(30), mm(20), 2);
        let (s, q, q2, gnd) = (b.net("S"), b.net("Q"), b.net("Q2"), b.net("GND"));
        b.smd2("R1", pt10(50, 100), 0.0, q, s);
        b.smd2("R2", pt10(250, 100), 0.0, s, q2);
        b.smd2("G1", pt10(150, 30), 0.0, gnd, NetId::NONE);
        let mut board = b.build();
        let poly = rect(110, 5, 190, 195);
        board.zones.push(Zone {
            net: gnd,
            layers: LayerSet::single(LayerId(0)),
            outline: poly.clone(),
            fills: vec![ZoneFill { layer: LayerId(0), polys: vec![poly] }],
            clearance: 300_000,
            min_thickness: 250_000,
            priority: 0,
            pad_connection: PadConnection::Solid,
            islands: IslandRemoval::Always,
            yields: false,
        });
        board
    }

    fn cfg(refill: bool) -> PipelineConfig {
        PipelineConfig {
            refill_zones: refill,
            router: RouterConfig {
                accel: ferroute_router::Accel::Cpu,
                layers: Some(LayerSet::single(LayerId(0))),
                ..RouterConfig::default()
            },
            ..PipelineConfig::default()
        }
    }

    #[test]
    fn refill_never_disconnects_a_pour_net() {
        // A second GND pad at the bottom: the pour is what joins the two GND pads, so a
        // signal cut across it would split GND (INV-CONN-1). The engine must keep GND
        // connected (pour as obstacle) and report the fallback.
        let mut board = blocked_board();
        let gnd = board.net_by_name("GND").expect("gnd");
        let fp = {
            let mut bb = BoardBuilder::new(mm(30), mm(20), 2);
            for n in ["S", "Q", "Q2", "GND"] {
                bb.net(n);
            }
            bb.smd2("G2", pt10(150, 170), 0.0, NetId(4), NetId::NONE);
            bb.build()
        };
        let mut pad = fp.pads[0].clone();
        pad.footprint =
            ferroute_board::FootprintId(u32::try_from(board.footprints.len()).expect("fp"));
        pad.net = gnd;
        let mut foot = fp.footprints[0].clone();
        foot.pads = vec![ferroute_board::PadId(u32::try_from(board.pads.len()).expect("pad"))];
        board.pads.push(pad);
        board.footprints.push(foot);
        let missing = |r: &PipelineResult| {
            r.drc.connectivity.unconnected.iter().filter(|u| u.net == gnd).count()
        };
        let input = ferroute_drc::check(&board, &DrcConfig::default());
        assert_eq!(input.connectivity.unconnected.iter().filter(|u| u.net == gnd).count(), 0);
        let r = run(&board, &cfg(true)).expect("run");
        assert_eq!(missing(&r), 0, "GND stays joined through its pour");
        assert!(r.metrics.drc_clean);
        assert!(
            r.rule_notes.iter().any(|n| n.contains("refill would disconnect")),
            "{:?}",
            r.rule_notes
        );
    }

    #[test]
    fn a_pour_over_the_only_corridor_yields_when_refill_is_enabled() {
        let board = blocked_board();
        let keep = run(&board, &cfg(false)).expect("run");
        assert!(keep.metrics.drc_clean);
        assert!(keep.metrics.connections_missing > 0, "pour blocks the corridor");
        assert!(keep.edit.zone_fills.is_empty());
        let r = run(&board, &cfg(true)).expect("run");
        assert!(r.metrics.drc_clean, "{:?}", r.drc.introduced().collect::<Vec<_>>());
        assert_eq!(r.metrics.connections_missing, 0, "routes through the pour");
        assert!(r.metrics.completion > keep.metrics.completion);
        assert_eq!(r.edit.zone_fills.len(), 1);
        assert!(r.board_after.zones.iter().all(|z| !z.yields));
        r.edit.check_invariants(&r.policy_board).expect("INV-EDIT-1");
        // The pour was cut in two; the half without the GND pad is gone.
        let regions = ferroute_geom::boolean::regions_of(&r.board_after.zones[0].fills[0].polys);
        assert_eq!(regions.len(), 1, "{regions:?}");
        assert!(r.drc.is_clean(), "{:#?}", r.drc.violations);
        let again = run(&board, &cfg(true)).expect("run");
        assert_eq!(r.edit, again.edit, "deterministic");
    }
}
