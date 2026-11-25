// SPDX-License-Identifier: AGPL-3.0-only
//! `ferroute route`.

use super::{item_label, load, out, render_opts};
use crate::length::parse_length;
use anyhow::{Context, Result, bail};
use ferroute_board::netrules::RuleSet;
use ferroute_board::{LayerSet, PourNetPolicy, RipupScope, ViaKind, ViaPolicy, ripup_with};
use ferroute_engine::{PipelineConfig, run as run_pipeline};
use ferroute_geom::Coord;
use ferroute_kicad::Removal;
use ferroute_place::{MovePolicy, PlaceConfig};
use ferroute_router::{Accel, RouterConfig};
use serde_json::json;
use std::path::PathBuf;
use std::process::ExitCode;

#[derive(clap::ValueEnum, Clone, Copy, Debug, PartialEq, Eq)]
pub enum AccelArg {
    Auto,
    Cpu,
    Gpu,
    None,
}

impl From<AccelArg> for Accel {
    fn from(a: AccelArg) -> Self {
        match a {
            AccelArg::Auto => Self::Auto,
            AccelArg::Cpu => Self::Cpu,
            AccelArg::Gpu => Self::Gpu,
            AccelArg::None => Self::None,
        }
    }
}

#[derive(clap::Args, Debug, Clone)]
pub struct RouteArgs {
    /// Input board (.kicad_pcb). A sibling .kicad_pro supplies net classes / rules.
    pub input: PathBuf,
    /// Output board path (may equal the input for in-place routing).
    #[arg(short, long)]
    pub output: PathBuf,
    /// Seed for every stochastic decision (replays are bit-identical).
    #[arg(long, default_value_t = 1)]
    pub seed: u64,
    /// Allow nudging unlocked footprints that match --movable.
    #[arg(long)]
    pub allow_move: bool,
    /// Reference globs of movable footprints (default: R*,C*,L*,FB*).
    #[arg(long, value_delimiter = ',')]
    pub movable: Vec<String>,
    /// Reference globs that must never move.
    #[arg(long, value_delimiter = ',')]
    pub exclude: Vec<String>,
    /// Maximum displacement of a movable footprint.
    #[arg(long, default_value = "2mm", value_parser = parse_length)]
    pub max_move: Coord,
    /// Placement grid for nudged footprints: moved anchors land on multiples of this step.
    #[arg(long, default_value = "0.1mm", value_parser = parse_length)]
    pub move_grid: Coord,
    /// Forbid 90° rotations of movable footprints.
    #[arg(long)]
    pub no_rotate: bool,
    /// Routing lattice pitch (default: default-class track width + clearance).
    #[arg(long, value_parser = parse_length)]
    pub pitch: Option<Coord>,
    /// Restrict routing to these copper layers (e.g. F.Cu,B.Cu).
    #[arg(long, value_delimiter = ',')]
    pub layers: Vec<String>,
    /// Clearance-field accelerator.
    #[arg(long, value_enum, default_value_t = AccelArg::Auto)]
    pub accel: AccelArg,
    /// PathFinder negotiation iterations.
    #[arg(long, default_value_t = 50)]
    pub iterations: u32,
    /// Nudge → re-route rounds for nets that failed.
    #[arg(long, default_value_t = 2)]
    pub lns_rounds: u32,
    /// Write a JSON report.
    #[arg(long)]
    pub report: Option<PathBuf>,
    /// Write an SVG rendering of the result.
    #[arg(long)]
    pub svg: Option<PathBuf>,
    /// Exit with status 1 if any connection remains unrouted.
    #[arg(long)]
    pub fail_on_incomplete: bool,
    /// Per-net electrical rules (TOML): current, frequency, impedance, length, skew, vias.
    #[arg(long)]
    pub rules: Option<PathBuf>,
    /// Exit with status 2 if any per-net electrical rule is not met.
    #[arg(long)]
    pub strict_rules: bool,
    /// Allow vias inside SMD pads (also `[routing] via_in_pad = true` in --rules). Such vias
    /// must be filled and capped (IPC-4761 type VII): an extra fabrication cost.
    #[arg(long)]
    pub allow_via_in_pad: bool,
    /// Via kinds the router may create, comma separated: through, blind, buried, micro
    /// (default through; also `[routing] via_types` in --rules). Blind, buried and micro
    /// vias need HDI fabrication: an extra cost.
    #[arg(long, value_delimiter = ',')]
    pub via_types: Vec<String>,
    /// Allow micro vias inside same-net SMD pads (also `[routing] micro_in_pad = true`).
    #[arg(long)]
    pub micro_in_pad: bool,
    /// Enable KiCad's native teardrops on every pad and via that gets new copper (also
    /// `[routing] teardrops = true` in --rules). Only the teardrop property of those pads and
    /// vias is written; KiCad creates the copper when zones are refilled.
    #[arg(long)]
    pub teardrops: bool,
    /// Keep copper pours as obstacles and never refill them (default: other-net pours yield
    /// to routed copper and their stored fill is recomputed around it).
    #[arg(long)]
    pub no_refill_zones: bool,
    /// Delete every unlocked existing track, arc and via before routing, then route the
    /// board from scratch. Locked copper, pours, pads and no-net copper are kept. The
    /// deleted copper is gone from the output file (also `[routing] ripup = "all"`).
    /// Connectivity/completion is reported against the ripped board.
    #[arg(long, conflicts_with = "ripup_nets")]
    pub ripup_all: bool,
    /// Delete unlocked existing copper of the nets matching these globs (`*`, `?`,
    /// case-sensitive) before routing; repeatable and/or comma separated, e.g.
    /// `--ripup-nets 'USB_*' --ripup-nets GND` (also `[routing] ripup = ["USB_*"]`).
    #[arg(long, value_delimiter = ',', value_name = "PATTERN", conflicts_with = "ripup_all")]
    pub ripup_nets: Vec<String>,
    /// With `--ripup-all` / `--ripup-nets`: also rip up nets that own a copper pour (their
    /// stitching vias and support tracks). Off by default: pour nets keep their copper so
    /// pour connectivity survives the refill (also `[routing] ripup_pour_nets = true`).
    #[arg(long)]
    pub ripup_pour_nets: bool,
}

/// The rip-up scope: command-line flags win over the rules file.
fn resolve_ripup<S: AsRef<str>>(all: bool, nets: &[S], from_rules: &RipupScope) -> RipupScope {
    match RipupScope::from_patterns(all, nets) {
        RipupScope::None => from_rules.clone(),
        cli => cli,
    }
}

/// The pour-net policy: the flag or the rules file switches to [`PourNetPolicy::Rip`].
fn resolve_pour_policy(flag: bool, from_rules: bool) -> PourNetPolicy {
    if flag || from_rules { PourNetPolicy::Rip } else { PourNetPolicy::Keep }
}

/// Number of new vias per [`ViaKind`] (in `ViaKind::ALL` order).
fn via_kind_counts(vias: &[ferroute_board::Via]) -> [usize; 4] {
    let mut c = [0usize; 4];
    for v in vias {
        c[ViaKind::ALL.iter().position(|&k| k == v.kind).unwrap_or(0)] += 1;
    }
    c
}

fn via_kinds_json(c: &[usize; 4]) -> serde_json::Value {
    let mut m = serde_json::Map::new();
    for (k, n) in ViaKind::ALL.iter().zip(c) {
        m.insert(k.name().to_owned(), serde_json::json!(n));
    }
    serde_json::Value::Object(m)
}

/// Parses a rules file (`[stackup]` + `[[net]]` tables).
///
/// # Errors
/// TOML syntax, unknown keys or invalid values.
pub fn parse_rules(text: &str) -> Result<RuleSet> {
    let rs: RuleSet = toml::from_str(text).context("parsing rules")?;
    rs.validate()?;
    Ok(rs)
}

/// Executes `ferroute route`.
pub fn run(a: &RouteArgs) -> Result<ExitCode> {
    let kb = load(&a.input)?;
    let mut doc = kb.doc;
    let mut board = kb.board;
    let mut via_in_pad = a.allow_via_in_pad;
    let mut teardrops = a.teardrops;
    let mut rules_ripup = RipupScope::None;
    let mut rules_pour_nets = false;
    let mut via_policy = ViaPolicy::default();
    if !a.via_types.is_empty() {
        via_policy = ViaPolicy::from_names(a.via_types.iter().map(String::as_str))
            .map_err(|n| anyhow::anyhow!("--via-types: unknown via kind {n:?}"))?;
    }
    via_policy.micro_in_pad |= a.micro_in_pad;
    if let Some(path) = &a.rules {
        let text =
            std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
        let rules = parse_rules(&text).with_context(|| format!("in {}", path.display()))?;
        via_in_pad |= rules.via_in_pad();
        if a.via_types.is_empty()
            && let Some(p) = rules.via_policy()
        {
            via_policy = ViaPolicy { micro_in_pad: via_policy.micro_in_pad || p.micro_in_pad, ..p };
        }
        teardrops |= rules.teardrops();
        rules_ripup = rules.ripup_scope();
        rules_pour_nets = rules.ripup_pour_nets();
        for w in rules.apply(&mut board)? {
            out(&format!("rules warning: {w}"));
        }
    }
    if a.ripup_pour_nets && !a.ripup_all && a.ripup_nets.is_empty() {
        anyhow::bail!("--ripup-pour-nets requires --ripup-all or --ripup-nets");
    }
    let policy = resolve_pour_policy(a.ripup_pour_nets, rules_pour_nets);
    let rip = ripup_with(&board, &resolve_ripup(a.ripup_all, &a.ripup_nets, &rules_ripup), policy);
    if !rip.scope.is_none() {
        out(&format!(
            "rip-up: removed {} track segment(s), {} arc(s), {} via(s) on {} net(s){}; kept {} pour net(s){}",
            rip.report.tracks,
            rip.report.arcs,
            rip.report.vias,
            rip.report.nets.len(),
            if rip.report.nets.is_empty() {
                String::new()
            } else {
                format!(" ({})", rip.report.nets.join(", "))
            },
            rip.report.kept_pour_nets.len(),
            if rip.report.kept_pour_nets.is_empty() {
                String::new()
            } else {
                format!(" ({})", rip.report.kept_pour_nets.join(", "))
            }
        ));
        if policy == PourNetPolicy::Rip {
            out(
                "rip-up warning: pour nets are ripped too; pour connectivity may now depend on the router's plane fan-out",
            );
        }
        for w in rip.report.warnings() {
            out(&format!("rip-up warning: {w}"));
        }
        out(
            "warning: ripped copper is deleted from the output file; completion is reported against the ripped board",
        );
    }
    let layers = if a.layers.is_empty() {
        None
    } else {
        let mut s = LayerSet::EMPTY;
        for name in &a.layers {
            let Some(i) = board.copper_layers.iter().position(|l| &l.name == name) else {
                bail!("unknown copper layer {name:?}");
            };
            s = s.with(ferroute_board::LayerId(u8::try_from(i)?));
        }
        Some(s)
    };
    let policy = MovePolicy {
        include: if a.movable.is_empty() {
            MovePolicy::default().include
        } else {
            a.movable.clone()
        },
        exclude: a.exclude.clone(),
        max_displacement: a.max_move,
        allow_rotation: !a.no_rotate,
    };
    let cfg = PipelineConfig {
        seed: a.seed,
        router: RouterConfig {
            pitch: a.pitch,
            layers,
            max_iterations: a.iterations,
            accel: a.accel.into(),
            via_in_pad,
            via_policy,
            ..RouterConfig::default()
        },
        nudge: a.allow_move.then(|| {
            (
                policy,
                PlaceConfig { seed: a.seed, grid: a.move_grid.max(1), ..PlaceConfig::default() },
            )
        }),
        lns_rounds: a.lns_rounds,
        refill_zones: !a.no_refill_zones,
        ..PipelineConfig::default()
    };
    let res = run_pipeline(&rip.board, &cfg).context("routing")?;
    if !res.metrics.drc_clean {
        bail!(
            "internal error: DRC gate reported {} introduced violations; refusing to write output",
            res.metrics.introduced_violations
        );
    }
    doc.apply_edit_removing(
        &res.policy_board,
        &res.edit,
        &Removal { original: &board, removed: &rip.removed, scope: &rip.scope },
        a.seed,
    )
    .context("applying edit to the KiCad document")?;
    let teardrop_nodes =
        if teardrops { doc.enable_teardrops(&res.policy_board, &res.edit) } else { 0 };
    if a.output == a.input {
        let bak = a.input.with_extension("kicad_pcb.ferroute-bak");
        std::fs::copy(&a.input, &bak)
            .with_context(|| format!("backing up to {}", bak.display()))?;
    }
    if let Some(dir) = a.output.parent()
        && !dir.as_os_str().is_empty()
    {
        std::fs::create_dir_all(dir)?;
    }
    std::fs::write(&a.output, doc.to_kicad_string())
        .with_context(|| format!("writing {}", a.output.display()))?;
    // Keep the rules next to the output so KiCad / kicad-cli DRC see the same classes.
    let pro_in = a.input.with_extension("kicad_pro");
    let pro_out = a.output.with_extension("kicad_pro");
    if pro_in.exists() && pro_in != pro_out {
        std::fs::copy(&pro_in, &pro_out)?;
    }
    let m = &res.metrics;
    let failed: Vec<&str> =
        res.failed_nets.iter().map(|n| res.board_after.nets[n.idx()].name.as_str()).collect();
    out(&format!(
        "{}: completion {:.1}% ({}/{} connections), {} tracks, {} vias, wirelength {:.1} mm, {} part(s) moved ({:.2} mm), DRC clean, {:.0} ms [{}]",
        a.output.display(),
        m.completion * 100.0,
        m.connections_total - m.connections_missing,
        m.connections_total,
        res.edit.new_tracks.len(),
        m.vias,
        m.wirelength_mm,
        m.moved_footprints,
        m.displacement_mm,
        m.runtime_ms,
        res.route.accel_backend
    ));
    let via_in_pad_count =
        res.edit.new_vias.iter().filter(|v| res.board_after.via_in_pad(v).is_some()).count();
    if via_in_pad_count > 0 {
        out(&format!(
            "via-in-pad: {via_in_pad_count} via(s) — require filled & capped vias (extra fabrication cost)"
        ));
    }
    let kinds = via_kind_counts(&res.edit.new_vias);
    let hdi: usize = kinds[1..].iter().sum();
    if hdi > 0 {
        out(&format!(
            "blind/buried/micro vias: {hdi} ({} blind, {} buried, {} micro) — require HDI fabrication (extra cost)",
            kinds[1], kinds[2], kinds[3]
        ));
    }
    if teardrops {
        out(&format!(
            "teardrops: enabled on {teardrop_nodes} pad(s)/via(s); KiCad creates the copper on refill"
        ));
    }
    if !failed.is_empty() {
        out(&format!("unrouted nets: {}", failed.join(", ")));
    }
    for n in &res.rule_notes {
        out(&format!("rules: {n}"));
    }
    let issues: Vec<String> =
        res.constraint_issues.iter().map(|i| i.describe(&res.board_after)).collect();
    for i in &issues {
        out(&format!("rule not met: {i}"));
    }
    let dfm = ferroute_drc::dfm::check(&res.board_after, &ferroute_drc::dfm::DfmConfig::default());
    for i in &dfm {
        out(&format!("dfm: {}", i.describe(&item_label(&res.board_after, i.item))));
    }
    if let Some(p) = &a.report {
        let report = json!({
            "ferroute_version": env!("CARGO_PKG_VERSION"),
            "input": a.input, "output": a.output, "seed": a.seed,
            "metrics": m, "route": res.route, "place": res.place, "failed_nets": failed,
            "lns_rounds_run": res.rounds, "via_in_pad": via_in_pad_count,
            "via_kinds": via_kinds_json(&kinds),
            "rule_notes": res.rule_notes, "rule_issues": issues,
            "teardrops": teardrops.then_some(teardrop_nodes), "dfm_issues": dfm,
            "ripup": (!rip.scope.is_none()).then_some(&rip.report),
            "drc": { "violations": res.drc.violations.len(), "connectivity": res.drc.connectivity },
        });
        std::fs::write(p, serde_json::to_string_pretty(&report)?)?;
    }
    if let Some(p) = &a.svg {
        let title = format!(
            "ferroute {} — {} (seed {})",
            env!("CARGO_PKG_VERSION"),
            a.input.display(),
            a.seed
        );
        ferroute_render::write_svg(
            p,
            &res.board_after,
            &render_opts(&res.board_after, &res.drc, title),
        )?;
    }
    Ok(if a.fail_on_incomplete && m.connections_missing > 0 {
        ExitCode::from(1)
    } else if a.strict_rules && !issues.is_empty() {
        ExitCode::from(2)
    } else {
        ExitCode::SUCCESS
    })
}

#[cfg(test)]
mod tests {
    use super::parse_rules;

    #[test]
    fn routing_table_selects_via_kinds() {
        let rs =
            parse_rules("[routing]\nvia_types = [\"through\", \"micro\"]\nmicro_in_pad = true\n")
                .expect("rules");
        let p = rs.via_policy().expect("policy");
        assert!(p.through && p.micro && !p.blind && !p.buried && p.micro_in_pad);
        assert_eq!(parse_rules("[routing]\nvia_in_pad = true").expect("rules").via_policy(), None);
        let err = parse_rules("[routing]\nvia_types = [\"stacked\"]").expect_err("unknown kind");
        assert!(format!("{err:#}").contains("stacked"), "{err:#}");
        assert!(parse_rules("[routing]\nvia_types = []").is_err(), "empty list rejected");
    }

    #[test]
    fn routing_table_selects_ripup() {
        use ferroute_board::RipupScope;
        assert_eq!(parse_rules("").expect("empty").ripup_scope(), RipupScope::None);
        let all = parse_rules("[routing]\nripup = \"all\"").expect("all");
        assert_eq!(all.ripup_scope(), RipupScope::All);
        let nets = parse_rules("[routing]\nripup = [\"USB_*\", \"GND\"]").expect("nets");
        assert_eq!(nets.ripup_scope(), RipupScope::Nets(vec!["USB_*".into(), "GND".into()]));
        assert_eq!(
            parse_rules("[routing]\nripup = \"none\"").expect("none").ripup_scope(),
            RipupScope::None
        );
        assert!(parse_rules("[routing]\nripup = \"everything\"").is_err());
    }

    #[test]
    fn pour_policy_from_flag_or_rules() {
        use ferroute_board::PourNetPolicy::{Keep, Rip};
        assert_eq!(super::resolve_pour_policy(false, false), Keep);
        assert_eq!(super::resolve_pour_policy(true, false), Rip);
        assert_eq!(super::resolve_pour_policy(false, true), Rip);
        let rs = parse_rules("[routing]\nripup_pour_nets = true").expect("rules");
        assert!(rs.ripup_pour_nets());
        assert!(!parse_rules("").expect("empty").ripup_pour_nets());
    }

    #[test]
    fn cli_ripup_wins_over_the_rules_file() {
        use ferroute_board::RipupScope;
        let rules = RipupScope::Nets(vec!["A".into()]);
        let pick = |all, nets: &[&str]| super::resolve_ripup(all, nets, &rules);
        assert_eq!(pick(false, &[]), rules, "rules file applies without flags");
        assert_eq!(pick(true, &[]), RipupScope::All);
        assert_eq!(pick(false, &["B, C"]), RipupScope::Nets(vec!["B".into(), "C".into()]));
    }

    #[test]
    fn via_kind_counts_follow_the_kind_order() {
        use ferroute_board::{LayerSet, NetId, Origin, Via, ViaKind};
        let via = |kind| Via {
            pos: ferroute_geom::Point::ORIGIN,
            diameter: 1,
            drill: 1,
            layers: LayerSet::EMPTY,
            kind,
            net: NetId(1),
            locked: false,
            origin: Origin::Routed,
        };
        let vias = [ViaKind::Micro, ViaKind::Through, ViaKind::Micro, ViaKind::Buried].map(via);
        assert_eq!(super::via_kind_counts(&vias), [1, 0, 1, 2]);
        let json = super::via_kinds_json(&[1, 0, 1, 2]);
        assert_eq!(json["micro"], 2);
        assert_eq!(json["blind"], 0);
    }

    #[test]
    fn rules_file_round_trip() {
        let rs = parse_rules(
            r#"
            [stackup]
            copper_um = 35
            outer_dielectric_mm = 0.2

            [[net]]
            nets = ["VBUS", "+5V"]
            max_current_a = 2.0

            [[net]]
            nets = ["USB_D?"]
            impedance_ohm = 45
            match_group = "usb"
            skew_max_mm = 0.15

            [[match]]
            name = "pair0"
            nets = ["D0_P", "D0_N"]
            skew_mm = 0.1

            [[match]]
            name = "bus"
            groups = ["pair*"]
            nets = ["CLK"]
            skew_mm = 0.5
            "#,
        )
        .expect("valid");
        assert_eq!(rs.rules.len(), 2);
        assert_eq!(rs.rules[1].impedance_ohm, Some(45.0));
        assert_eq!(rs.matches.len(), 2);
        assert_eq!(rs.matches[1].groups, vec!["pair*".to_owned()]);
        assert_eq!(rs.matches[1].skew_mm, Some(0.5));
        assert!(parse_rules("[[match]]\nname=\"x\"\nskew_mm=0.1\nbogus=1").is_err());
        assert_eq!(rs.stackup.and_then(|s| s.outer_dielectric_mm), Some(0.2));
    }

    #[test]
    fn routing_table_switches_via_in_pad_and_defaults_off() {
        assert!(!parse_rules("").expect("empty").via_in_pad());
        assert!(!parse_rules("[routing]").expect("empty table").via_in_pad());
        assert!(parse_rules("[routing]\nvia_in_pad = true").expect("on").via_in_pad());
        assert!(!parse_rules("[routing]\nvia_in_pad = false").expect("off").via_in_pad());
        assert!(parse_rules("[routing]\nvia_in_pads = true").is_err(), "typo rejected");
        assert!(!parse_rules("[routing]").expect("empty table").teardrops());
        assert!(parse_rules("[routing]\nteardrops = true").expect("on").teardrops());
    }

    #[test]
    fn delay_match_rules_parse() {
        let rs = parse_rules(
            "[stackup]\nboard_thickness_mm = 1.2\n[[match]]\nname = \"d\"\nnets = [\"A\", \"B\"]\nby = \"delay\"\nskew_ps = 2.5",
        )
        .expect("valid");
        assert_eq!(rs.matches[0].by, ferroute_board::constraints::MatchBy::Delay);
        assert_eq!(rs.matches[0].skew_ps, Some(2.5));
        assert_eq!(rs.stackup.and_then(|s| s.board_thickness_mm), Some(1.2));
        assert!(parse_rules("[[match]]\nname=\"x\"\nby=\"time\"\nskew_ps=1").is_err());
        let both = "[[match]]\nname=\"x\"\nby=\"delay\"\nskew_ps=1\nskew_mm=1";
        assert!(parse_rules(both).is_err(), "both skew keys");
    }

    #[test]
    fn rules_file_rejects_typos_and_bad_values() {
        assert!(parse_rules("[[net]]\nnets=[\"A\"]\nmax_curent_a = 1").is_err());
        assert!(parse_rules("[[net]]\nnets=[\"A\"]\nmax_current_a = -1").is_err());
    }
}
