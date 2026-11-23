// SPDX-License-Identifier: AGPL-3.0-only
//! Per-net electrical constraints and the geometry they imply.
//!
//! Constraints are stored in exact integer units (mA, Hz, mΩ, nm) so the model stays
//! comparable and deterministic. Derived widths use only IEEE-portable math
//! (`ferroute_geom::detmath`) and are rounded **up** to a 10 µm fabrication step, so a
//! derived width is never narrower than the formula requires (INV-RULE-1).

use crate::{Board, NetClass, NetId};
use ferroute_geom::Coord;
use ferroute_geom::detmath::{exp, pow};
use ferroute_geom::units::format_mm;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

/// Frequency at or above which a net is routed with the high-speed policy.
pub const HIGH_SPEED_HZ: u64 = 50_000_000;
/// Fabrication rounding step for derived widths.
pub const WIDTH_STEP: Coord = 10_000;
/// Default allowed temperature rise for current-carrying traces (°C).
pub const DEFAULT_DELTA_T_C: f64 = 10.0;

/// Electrical constraints of one net (all optional).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct NetConstraints {
    /// Maximum continuous current (mA) → minimum trace width (IPC-2221).
    pub max_current_ma: Option<u32>,
    /// Highest signal frequency (Hz); ≥ [`HIGH_SPEED_HZ`] selects the high-speed policy.
    pub frequency_hz: Option<u64>,
    /// Target single-ended characteristic impedance (mΩ, e.g. 50 000 = 50 Ω).
    pub impedance_mohm: Option<u32>,
    /// Routed length limits (nm).
    pub length_min: Option<Coord>,
    pub length_max: Option<Coord>,
    /// Length-matching group and the allowed length spread within it (nm).
    pub match_group: Option<String>,
    pub skew_max: Option<Coord>,
    /// Maximum number of vias on the net.
    pub max_vias: Option<u32>,
}

impl NetConstraints {
    #[must_use]
    pub fn is_high_speed(&self) -> bool {
        self.frequency_hz.is_some_and(|f| f >= HIGH_SPEED_HZ) || self.impedance_mohm.is_some()
    }
}

/// A length-matching group: every member's routed length must lie within `skew_max` of the
/// others. Groups may share nets, so a pair can sit inside a bus (nesting).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MatchGroup {
    pub name: String,
    pub skew_max: Coord,
    /// Member nets, sorted and de-duplicated.
    pub nets: Vec<NetId>,
}

/// Explicit `board.match_groups` plus legacy per-net groups (`match_group` + `skew_max`),
/// merged by name (union of nets, smaller skew). Sorted by name; groups with fewer than two
/// nets are dropped.
#[must_use]
pub fn match_groups(board: &Board) -> Vec<MatchGroup> {
    let mut by_name: BTreeMap<String, (Coord, BTreeSet<NetId>)> = BTreeMap::new();
    let mut add = |name: &str, skew: Coord, nets: &mut dyn Iterator<Item = NetId>| {
        let e = by_name.entry(name.to_owned()).or_insert((skew, BTreeSet::new()));
        e.0 = e.0.min(skew);
        e.1.extend(nets);
    };
    for g in &board.match_groups {
        add(&g.name, g.skew_max, &mut g.nets.iter().copied());
    }
    for (i, net) in board.nets.iter().enumerate().skip(1) {
        let c = &net.constraints;
        if let (Some(g), Some(skew)) = (c.match_group.as_deref(), c.skew_max) {
            let id = NetId(u32::try_from(i).unwrap_or(u32::MAX));
            add(g, skew, &mut std::iter::once(id));
        }
    }
    by_name
        .into_iter()
        .filter(|(_, (_, nets))| nets.len() >= 2)
        .map(|(name, (skew_max, nets))| MatchGroup {
            name,
            skew_max,
            nets: nets.into_iter().collect(),
        })
        .collect()
}

/// Per-net target lengths (index = net id), never below the current `lengths`. Nets below
/// `length_min` rise to `length_min + 2 µm`; then, until stable, every group whose spread
/// exceeds its skew raises members below `max − skew` to `max − skew/2`. A target never
/// exceeds the net's `length_max` (the rule then stays unmet and is reported by the DRC).
#[must_use]
pub fn match_targets(board: &Board, lengths: &[Coord]) -> Vec<Coord> {
    let mut t = lengths.to_vec();
    let cap = |i: usize, v: Coord| board.nets[i].constraints.length_max.map_or(v, |m| v.min(m));
    for i in 1..t.len().min(board.nets.len()) {
        if let Some(min) = board.nets[i].constraints.length_min
            && t[i] < min
        {
            // A little over the limit so float summation can never land just below it.
            t[i] = t[i].max(cap(i, min + 2_000));
        }
    }
    let groups = match_groups(board);
    for _ in 0..64 * groups.len().max(1) {
        let mut changed = false;
        for g in &groups {
            let ids: Vec<usize> = g.nets.iter().map(|n| n.idx()).filter(|&i| i < t.len()).collect();
            let (Some(&hi), Some(&lo)) =
                (ids.iter().map(|&i| &t[i]).max(), ids.iter().map(|&i| &t[i]).min())
            else {
                continue;
            };
            if hi - lo <= g.skew_max {
                continue;
            }
            for &i in &ids {
                if t[i] < hi - g.skew_max {
                    let v = cap(i, hi - g.skew_max / 2);
                    if v > t[i] {
                        t[i] = v;
                        changed = true;
                    }
                }
            }
        }
        if !changed {
            break;
        }
    }
    t
}

/// Physical stack-up facts used by the width formulas.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Stackup {
    /// Copper thickness of every copper layer (nm), top to bottom.
    pub copper_thickness: Vec<Coord>,
    /// Dielectric thickness between an outer layer and its nearest reference plane (nm).
    pub outer_dielectric: Coord,
    /// Relative permittivity × 1000 (e.g. 4300 for FR-4).
    pub epsilon_r_milli: u32,
}

impl Default for Stackup {
    /// 1.6 mm FR-4, 35 µm (1 oz) copper, 2 layers (reference plane on the far side).
    fn default() -> Self {
        Self {
            copper_thickness: vec![35_000, 35_000],
            outer_dielectric: 1_510_000,
            epsilon_r_milli: 4300,
        }
    }
}

fn round_up(v: f64) -> Coord {
    #[allow(clippy::cast_possible_truncation)]
    let nm = v.ceil() as Coord;
    (nm + WIDTH_STEP - 1) / WIDTH_STEP * WIDTH_STEP
}

/// Minimum trace width (nm) for `current_ma` per IPC-2221:
/// `I = k·ΔT^0.44·A^0.725` (A in mil², k = 0.048 external / 0.024 internal).
#[must_use]
pub fn width_for_current(
    current_ma: u32,
    copper_thickness: Coord,
    external: bool,
    delta_t_c: f64,
) -> Coord {
    if current_ma == 0 || copper_thickness <= 0 {
        return 0;
    }
    let k = if external { 0.048 } else { 0.024 };
    let i = f64::from(current_ma) / 1000.0;
    let area_mil2 = pow(i / (k * pow(delta_t_c, 0.44)), 1.0 / 0.725);
    #[allow(clippy::cast_precision_loss)]
    let thickness_mil = copper_thickness as f64 / 25_400.0;
    round_up(area_mil2 / thickness_mil * 25_400.0)
}

/// Microstrip trace width (nm) for impedance `z0_ohm` over a reference plane `h` away
/// (IPC-2141: `Z0 = 87/√(εr+1.41)·ln(5.98h/(0.8w+t))`). `None` if unattainable.
#[must_use]
pub fn microstrip_width(z0_ohm: f64, h: Coord, t: Coord, epsilon_r: f64) -> Option<Coord> {
    #[allow(clippy::cast_precision_loss)]
    let (h, t) = (h as f64, t as f64);
    let w = (5.98 * h / exp(z0_ohm * (epsilon_r + 1.41).sqrt() / 87.0) - t) / 0.8;
    (w > 0.0).then(|| round_up(w))
}

/// Symmetric stripline width (nm) between planes `b` apart
/// (IPC-2141: `Z0 = 60/√εr·ln(4b/(0.67π(0.8w+t)))`). `None` if unattainable.
#[must_use]
pub fn stripline_width(z0_ohm: f64, b: Coord, t: Coord, epsilon_r: f64) -> Option<Coord> {
    #[allow(clippy::cast_precision_loss)]
    let (b, t) = (b as f64, t as f64);
    let w =
        (4.0 * b / (0.67 * std::f64::consts::PI * exp(z0_ohm * epsilon_r.sqrt() / 60.0)) - t) / 0.8;
    (w > 0.0).then(|| round_up(w))
}

/// Width (nm) net `net` must have on every layer: the widest of its net-class width, the
/// current-derived width (internal-layer formula when the board has inner layers — the
/// conservative case) and the impedance-derived outer-layer width.
#[must_use]
pub fn required_width(board: &Board, net: NetId) -> Coord {
    let class_w = board.rules.class_of(board, net).track_width;
    let Some(n) = board.nets.get(net.idx()) else { return class_w };
    let c = &n.constraints;
    let s = &board.stackup;
    let thin = s.copper_thickness.iter().copied().filter(|&t| t > 0).min().unwrap_or(35_000);
    let external_only = board.layer_count() <= 2;
    let current_w = c
        .max_current_ma
        .map_or(0, |i| width_for_current(i, thin, external_only, DEFAULT_DELTA_T_C));
    let t_outer = s.copper_thickness.first().copied().unwrap_or(35_000);
    let z_w = c
        .impedance_mohm
        .and_then(|z| {
            microstrip_width(
                f64::from(z) / 1000.0,
                s.outer_dielectric,
                t_outer,
                f64::from(s.epsilon_r_milli) / 1000.0,
            )
        })
        .unwrap_or(0);
    class_w.max(current_w).max(z_w)
}

/// Applies derived widths: every net whose required width exceeds its class width moves to
/// a derived class (`<class>/<net>`) with that width — never narrower (INV-RULE-1).
/// Returns human-readable notes describing each change.
pub fn resolve(board: &mut Board) -> Vec<String> {
    let mut notes = Vec::new();
    for i in 1..board.nets.len() {
        let net = NetId(u32::try_from(i).unwrap_or(u32::MAX));
        let need = required_width(board, net);
        let base = board.rules.class_of(board, net).clone();
        if need <= base.track_width {
            continue;
        }
        let old = base.track_width;
        let name = format!("{}/{}", base.name, board.nets[i].name);
        board.rules.classes.push(NetClass { name, track_width: need, ..base });
        board.nets[i].class = u16::try_from(board.rules.classes.len() - 1).unwrap_or(u16::MAX);
        notes.push(format!(
            "net {}: width {} mm → {} mm (current/impedance rules)",
            board.nets[i].name,
            format_mm(old),
            format_mm(need)
        ));
    }
    notes
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builder::{BoardBuilder, mm};

    #[test]
    fn ipc2221_widths_match_reference_values() {
        let ext = width_for_current(1000, 35_000, true, 10.0);
        assert!((290_000..=310_000).contains(&ext), "external 1 A: {ext}");
        let int = width_for_current(1000, 35_000, false, 10.0);
        assert!((770_000..=800_000).contains(&int), "internal 1 A: {int}");
        assert!(width_for_current(2000, 35_000, true, 10.0) > ext);
        assert_eq!(width_for_current(0, 35_000, true, 10.0), 0);
        assert_eq!(ext % WIDTH_STEP, 0, "rounded to the fabrication step");
    }

    #[test]
    fn ipc2141_microstrip_and_stripline() {
        let w = microstrip_width(50.0, 200_000, 35_000, 4.3).expect("attainable");
        assert!((330_000..=350_000).contains(&w), "50 Ω microstrip: {w}");
        // Higher impedance → narrower trace.
        assert!(microstrip_width(75.0, 200_000, 35_000, 4.3).expect("w") < w);
        assert!(stripline_width(50.0, 400_000, 17_500, 4.3).expect("w") > 0);
        // Unattainable: 150 Ω over a 50 µm dielectric would need a negative width.
        assert!(microstrip_width(150.0, 50_000, 35_000, 4.3).is_none());
    }

    #[test]
    fn resolve_widens_but_never_narrows() {
        let mut b = BoardBuilder::new(mm(30), mm(20), 2);
        let p = b.net("VBUS");
        let s = b.net("SIG");
        let mut board = b.build();
        board.nets[p.idx()].constraints.max_current_ma = Some(2000);
        board.nets[s.idx()].constraints.max_current_ma = Some(10); // tiny: class width wins
        let notes = resolve(&mut board);
        assert_eq!(notes.len(), 1);
        let w = board.rules.class_of(&board, p).track_width;
        assert!(w >= width_for_current(2000, 35_000, true, 10.0));
        assert_eq!(board.rules.class_of(&board, s).track_width, 250_000);
        assert_eq!(required_width(&board, s), 250_000);
    }

    #[test]
    fn high_speed_detection() {
        let mut c = NetConstraints::default();
        assert!(!c.is_high_speed());
        c.frequency_hz = Some(100_000_000);
        assert!(c.is_high_speed());
        c.frequency_hz = Some(1_000_000);
        assert!(!c.is_high_speed());
        c.impedance_mohm = Some(50_000);
        assert!(c.is_high_speed());
    }

    fn match_board(names: &[&str]) -> (Board, Vec<NetId>) {
        let mut b = BoardBuilder::new(mm(30), mm(20), 2);
        let ids = names.iter().map(|n| b.net(n)).collect();
        (b.build(), ids)
    }

    fn group(name: &str, skew: Coord, nets: &[NetId]) -> MatchGroup {
        MatchGroup { name: name.into(), skew_max: skew, nets: nets.to_vec() }
    }

    fn spread(t: &[Coord], nets: &[NetId]) -> Coord {
        let v: Vec<Coord> = nets.iter().map(|n| t[n.idx()]).collect();
        v.iter().max().unwrap() - v.iter().min().unwrap()
    }

    #[test]
    fn nested_groups_match_pairs_and_bus() {
        let (mut b, n) = match_board(&["A", "B", "C", "D"]);
        let (p0, p1) = ([n[0], n[1]], [n[2], n[3]]);
        b.match_groups = vec![
            group("p0", mm(1) / 10, &p0),
            group("p1", mm(1) / 10, &p1),
            group("bus", mm(1) / 2, &n),
        ];
        let l = [0, mm(10), mm(105) / 10, mm(20), mm(202) / 10];
        let t = match_targets(&b, &l);
        assert_eq!(t, match_targets(&b, &l), "deterministic");
        for g in &b.match_groups {
            assert!(spread(&t, &g.nets) <= g.skew_max, "{} {t:?}", g.name);
        }
        assert!(t.iter().all(|&v| v <= mm(202) / 10), "nothing above the longest: {t:?}");
        assert!(t.iter().zip(&l).all(|(a, b)| a >= b), "never shrinks");
    }

    #[test]
    fn net_in_two_groups_and_length_cap() {
        let (mut b, n) = match_board(&["A", "B", "C"]);
        b.match_groups =
            vec![group("x", mm(1) / 10, &[n[0], n[1]]), group("y", mm(1) / 10, &[n[1], n[2]])];
        let l = [0, mm(10), mm(12), mm(14)];
        let t = match_targets(&b, &l);
        assert!(spread(&t, &n) <= mm(1) / 5, "{t:?}");
        b.nets[n[0].idx()].constraints.length_max = Some(mm(11));
        let t = match_targets(&b, &l);
        assert_eq!(t[n[0].idx()], mm(11), "capped");
        assert!(spread(&t, &[n[0], n[1]]) > mm(1) / 10, "group stays unmet");
        b.nets[n[0].idx()].constraints.length_max = None;
        b.nets[n[2].idx()].constraints.length_min = Some(mm(20));
        assert_eq!(match_targets(&b, &l)[n[2].idx()], mm(20) + 2_000);
    }

    #[test]
    fn legacy_groups_and_merge_with_explicit() {
        let (mut b, n) = match_board(&["A", "B", "C"]);
        for (i, skew) in [(0, mm(1)), (1, mm(1) / 2)] {
            let c = &mut b.nets[n[i].idx()].constraints;
            c.match_group = Some("g".into());
            c.skew_max = Some(skew);
        }
        assert_eq!(match_groups(&b), vec![group("g", mm(1) / 2, &[n[0], n[1]])]);
        b.match_groups = vec![group("g", mm(1) / 4, &[n[2], n[1]]), group("solo", 1, &[n[0]])];
        assert_eq!(match_groups(&b), vec![group("g", mm(1) / 4, &n)], "union, smaller skew");
    }
}
