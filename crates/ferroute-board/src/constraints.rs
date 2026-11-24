// SPDX-License-Identifier: AGPL-3.0-only
//! Per-net electrical constraints and the geometry they imply.
//!
//! Constraints are stored in exact integer units (mA, Hz, mΩ, nm) so the model stays
//! comparable and deterministic. Derived widths use only IEEE-portable math
//! (`ferroute_geom::detmath`) and are rounded **up** to a 10 µm fabrication step, so a
//! derived width is never narrower than the formula requires (INV-RULE-1).

use crate::{Board, LayerId, NetClass, NetId};
use ferroute_geom::Coord;
use ferroute_geom::detmath::{exp, ln, pow};
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

/// Which quantity a [`MatchGroup`] equalises.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum MatchBy {
    /// Routed physical length; `skew_max` is in nm.
    #[default]
    Length,
    /// Propagation delay (layer- and via-aware, see [`net_delays_fs`]); `skew_max` is in fs.
    Delay,
}

/// A matching group: every member's routed length (or delay, see [`MatchBy`]) must lie
/// within `skew_max` of the others. Groups may share nets, so a pair can sit inside a bus
/// (nesting).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MatchGroup {
    pub name: String,
    /// Allowed spread: nm for [`MatchBy::Length`], femtoseconds for [`MatchBy::Delay`].
    pub skew_max: Coord,
    #[serde(default)]
    pub by: MatchBy,
    /// Member nets, sorted and de-duplicated.
    pub nets: Vec<NetId>,
}

impl MatchGroup {
    /// Spread (max − min) of the members in this group's own unit: `lengths` (nm) or
    /// `delays_fs`, indexed by net id.
    #[must_use]
    pub fn spread(&self, lengths: &[Coord], delays_fs: &[i64]) -> i64 {
        let v = match self.by {
            MatchBy::Length => lengths,
            MatchBy::Delay => delays_fs,
        };
        let it = self.nets.iter().filter_map(|n| v.get(n.idx()).copied());
        it.clone().max().unwrap_or(0) - it.min().unwrap_or(0)
    }
}

/// Explicit `board.match_groups` plus legacy per-net groups (`match_group` + `skew_max`),
/// merged by name (union of nets, smaller skew). Sorted by name; groups with fewer than two
/// nets are dropped.
#[must_use]
pub fn match_groups(board: &Board) -> Vec<MatchGroup> {
    type Entry = (Coord, MatchBy, BTreeSet<NetId>);
    let mut by_name: BTreeMap<String, Entry> = BTreeMap::new();
    let mut add = |name: &str, skew: Coord, by: MatchBy, nets: &mut dyn Iterator<Item = NetId>| {
        let e = by_name.entry(name.to_owned()).or_insert((skew, by, BTreeSet::new()));
        e.0 = e.0.min(skew);
        e.2.extend(nets);
    };
    for g in &board.match_groups {
        add(&g.name, g.skew_max, g.by, &mut g.nets.iter().copied());
    }
    for (i, net) in board.nets.iter().enumerate().skip(1) {
        let c = &net.constraints;
        if let (Some(g), Some(skew)) = (c.match_group.as_deref(), c.skew_max) {
            let id = NetId(u32::try_from(i).unwrap_or(u32::MAX));
            add(g, skew, MatchBy::Length, &mut std::iter::once(id));
        }
    }
    by_name
        .into_iter()
        .filter(|(_, (_, _, nets))| nets.len() >= 2)
        .map(|(name, (skew_max, by, nets))| MatchGroup {
            name,
            skew_max,
            by,
            nets: nets.into_iter().collect(),
        })
        .collect()
}

/// Raises, until stable, every member of a `by` group whose spread exceeds its skew and
/// that is below `max − skew` to `max − skew/2` (capped per net by `cap`). `t` is in the
/// groups' unit.
fn raise_groups(board: &Board, t: &mut [i64], by: MatchBy, cap: &dyn Fn(usize, i64) -> i64) {
    let groups: Vec<MatchGroup> = match_groups(board).into_iter().filter(|g| g.by == by).collect();
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
}

/// Per-net target lengths (index = net id), never below the current `lengths`. Nets below
/// `length_min` rise to `length_min + 2 µm`; then, until stable, every *length* group whose
/// spread exceeds its skew raises members below `max − skew` to `max − skew/2`. A target
/// never exceeds the net's `length_max` (the rule then stays unmet and is reported by the
/// DRC). Delay groups are handled by [`match_delay_targets`].
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
    raise_groups(board, &mut t, MatchBy::Length, &cap);
    t
}

/// Per-net target delays (fs, index = net id), never below the current `delays_fs`: the
/// same raising rule as [`match_targets`], applied to the board's delay groups in delay
/// space. The tuner converts the extra delay to length per run (`layer_delay_ps_per_mm` of
/// the run's layer and width).
#[must_use]
pub fn match_delay_targets(board: &Board, delays_fs: &[i64]) -> Vec<i64> {
    let mut t = delays_fs.to_vec();
    raise_groups(board, &mut t, MatchBy::Delay, &|_, v| v);
    t
}

/// Delay (ps/mm) of a trace of `width` on copper layer `layer`: outer layers are
/// microstrip over the outer dielectric, inner layers stripline in `εr`. Falls back to
/// `√εr / c` if the microstrip model is out of range.
#[must_use]
pub fn layer_delay_ps_per_mm(board: &Board, layer: LayerId, width: Coord) -> f64 {
    let s = &board.stackup;
    let n = board.layer_count();
    let last = n.saturating_sub(1);
    let inner = layer.0 != 0 && layer.0 < last;
    let kind = if inner { LineKind::Stripline } else { LineKind::Microstrip };
    let t = s.copper_thickness.get(usize::from(layer.0)).copied().unwrap_or(35_000);
    let (h, er) = if inner {
        let er = s.stripline_b_er(layer, n).map_or(f64::from(s.epsilon_r_milli) / 1000.0, |v| v.1);
        (s.outer_dielectric, er)
    } else {
        s.outer_h_er(layer, n)
    };
    delay_ps_per_mm(kind, width, h, t, er).unwrap_or_else(|| er.sqrt() / C_MM_PER_PS)
}

/// Routed propagation delay of every net in femtoseconds (index = net id): the sum over
/// tracks and arcs of length × [`layer_delay_ps_per_mm`], plus each via's barrel
/// (`board_thickness × spanned-layer fraction × √εr / c`). Rounded once per net;
/// IEEE-only maths (INV-DET-3).
#[must_use]
pub fn net_delays_fs(board: &Board) -> Vec<i64> {
    let mut d = vec![0.0f64; board.nets.len()];
    #[allow(clippy::cast_precision_loss)]
    let mm = |nm: f64| nm / 1_000_000.0;
    for t in &board.tracks {
        if let Some(e) = d.get_mut(t.net.idx()) {
            *e += mm(t.start.dist(t.end)) * layer_delay_ps_per_mm(board, t.layer, t.width);
        }
    }
    for a in &board.arcs {
        if let Some(e) = d.get_mut(a.net.idx()) {
            *e += mm(a.length()) * layer_delay_ps_per_mm(board, a.layer, a.width);
        }
    }
    let er = f64::from(board.stackup.epsilon_r_milli) / 1000.0;
    let n = board.layer_count();
    #[allow(clippy::cast_precision_loss)]
    let barrel_ps_per_full = mm(board.stackup.board_thickness as f64) * er.sqrt() / C_MM_PER_PS;
    for v in &board.vias {
        let lo = v.layers.iter().map(|l| l.0).min();
        let hi = v.layers.iter().map(|l| l.0).max();
        let (Some(lo), Some(hi)) = (lo, hi) else { continue };
        if n >= 2
            && let Some(e) = d.get_mut(v.net.idx())
        {
            *e += barrel_ps_per_full * f64::from(hi - lo) / f64::from(n - 1);
        }
    }
    #[allow(clippy::cast_possible_truncation)]
    d.into_iter().map(|ps| (ps * 1000.0).round() as i64).collect()
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
    /// Plane-to-plane spacing `b` of an inner (stripline) layer (nm), when the stack-up
    /// has inner dielectrics; `None` disables the stripline width model.
    #[serde(default)]
    pub inner_dielectric: Option<Coord>,
    /// Overall board thickness (nm); sets the barrel length of vias for delay matching.
    #[serde(default = "default_board_thickness")]
    pub board_thickness: Coord,
    /// Per-dielectric detail read from the board file (empty = only the scalar fields
    /// above are known). When non-empty, [`Stackup::outer_h_er`] and
    /// [`Stackup::stripline_b_er`] use it in preference to the scalars.
    #[serde(default)]
    pub dielectrics: Vec<Dielectric>,
}

/// One dielectric (prepreg/core, all its sub-layers merged) between two adjacent copper
/// layers.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Dielectric {
    /// Total thickness (nm) of all sub-layers.
    pub thickness: Coord,
    /// Relative permittivity × 1000: the thickness-weighted mean of its sub-layers.
    pub epsilon_r_milli: u32,
    /// The copper layers it separates (adjacent: `.1 == .0 + 1`).
    pub between: (LayerId, LayerId),
}

const fn default_board_thickness() -> Coord {
    1_600_000
}

impl Default for Stackup {
    /// 1.6 mm FR-4, 35 µm (1 oz) copper, 2 layers (reference plane on the far side).
    fn default() -> Self {
        Self {
            copper_thickness: vec![35_000, 35_000],
            outer_dielectric: 1_510_000,
            epsilon_r_milli: 4300,
            inner_dielectric: None,
            board_thickness: default_board_thickness(),
            dielectrics: Vec::new(),
        }
    }
}

impl Stackup {
    /// Builds a stack-up from the file's copper thicknesses (top to bottom) and its
    /// dielectrics (top to bottom; `dielectrics[k]` separates copper `k` and `k + 1`,
    /// `between` is rewritten accordingly). Derived scalars:
    /// * `outer_dielectric` = thickness of the dielectric adjacent to `F.Cu`;
    /// * `epsilon_r_milli` = thickness-weighted mean over all dielectrics;
    /// * `inner_dielectric` (for ≥ 3 copper layers) = plane spacing `b` of the first inner
    ///   layer: the two dielectrics around it plus its copper thickness (adjacent copper
    ///   layers act as references);
    /// * `board_thickness` = `board_thickness` if given, else copper + dielectrics.
    ///
    /// Returns `None` unless there are exactly `copper.len() - 1` dielectrics, all with
    /// positive thickness and `εr ≥ 1`.
    #[must_use]
    pub fn from_layers(
        copper: Vec<Coord>,
        mut dielectrics: Vec<Dielectric>,
        board_thickness: Option<Coord>,
    ) -> Option<Self> {
        if copper.len() < 2
            || dielectrics.len() + 1 != copper.len()
            || copper.iter().any(|&t| t <= 0)
            || dielectrics.iter().any(|d| d.thickness <= 0 || d.epsilon_r_milli < 1000)
        {
            return None;
        }
        for (k, d) in dielectrics.iter_mut().enumerate() {
            let k = u8::try_from(k).ok()?;
            d.between = (LayerId(k), LayerId(k + 1));
        }
        let total: Coord = dielectrics.iter().map(|d| d.thickness).sum();
        let weighted: i128 = dielectrics
            .iter()
            .map(|d| i128::from(d.thickness) * i128::from(d.epsilon_r_milli))
            .sum();
        let er = u32::try_from((weighted + i128::from(total) / 2) / i128::from(total)).ok()?;
        let mut s = Self {
            outer_dielectric: dielectrics[0].thickness,
            epsilon_r_milli: er,
            inner_dielectric: None,
            board_thickness: board_thickness
                .unwrap_or_else(|| total + copper.iter().copied().sum::<Coord>()),
            copper_thickness: copper,
            dielectrics,
        };
        let n = u8::try_from(s.copper_thickness.len()).ok()?;
        if n > 2 {
            s.inner_dielectric = s.stripline_b_er(LayerId(1), n).map(|v| v.0);
        }
        Some(s)
    }

    fn dielectric_of(&self, a: u8, b: u8) -> Option<&Dielectric> {
        self.dielectrics.iter().find(|d| d.between == (LayerId(a), LayerId(b)))
    }

    /// `(h, εr)` for a microstrip on outer layer `layer` of an `n`-layer board: the
    /// dielectric between that layer and its neighbour, falling back to the scalar
    /// `outer_dielectric` / `epsilon_r_milli` when the file gave no per-dielectric data.
    #[must_use]
    pub fn outer_h_er(&self, layer: LayerId, n: u8) -> (Coord, f64) {
        let neighbour = if layer.0 == 0 { 1 } else { layer.0.saturating_sub(1) };
        let (a, b) = (layer.0.min(neighbour), layer.0.max(neighbour));
        let outer = layer.0 == 0 || layer.0 + 1 == n;
        match self.dielectric_of(a, b) {
            Some(d) if outer => (d.thickness, f64::from(d.epsilon_r_milli) / 1000.0),
            _ => (self.outer_dielectric, f64::from(self.epsilon_r_milli) / 1000.0),
        }
    }

    /// `(b, εr)` for a stripline on inner layer `layer` of an `n`-layer board: `b` is the
    /// two adjacent dielectrics plus the layer's copper thickness, `εr` their
    /// thickness-weighted mean. Without per-dielectric data: `inner_dielectric` and the
    /// scalar `εr`. `None` for outer layers or when no inner spacing is known.
    #[must_use]
    pub fn stripline_b_er(&self, layer: LayerId, n: u8) -> Option<(Coord, f64)> {
        if layer.0 == 0 || layer.0 + 1 >= n {
            return None;
        }
        let l = layer.0;
        if let (Some(u), Some(d)) = (self.dielectric_of(l - 1, l), self.dielectric_of(l, l + 1)) {
            let t = self.copper_thickness.get(usize::from(l)).copied().unwrap_or(0);
            let h = u.thickness + d.thickness;
            let w = i128::from(u.thickness) * i128::from(u.epsilon_r_milli)
                + i128::from(d.thickness) * i128::from(d.epsilon_r_milli);
            #[allow(clippy::cast_precision_loss)]
            let er = (w as f64) / (h as f64) / 1000.0;
            return Some((h + t, er));
        }
        self.inner_dielectric.map(|b| (b, f64::from(self.epsilon_r_milli) / 1000.0))
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

// ---------------------------------------------------------------------------------------
// Transmission-line models. Everything below uses only `+ − × ÷ √` (IEEE-exact) and
// `detmath::{exp, ln, pow}` (INV-DET-3). Lengths are nm in the API, dimensionless ratios
// inside. IPC-2152 (current vs. width with planes/airflow) is *not* implemented: its charts
// are proprietary; IPC-2221 above stays the current model.
// ---------------------------------------------------------------------------------------

/// Free-space wave impedance η0 (Ω).
const ETA0: f64 = 376.730_313_668;
/// Speed of light in mm/ps.
const C_MM_PER_PS: f64 = 0.299_792_458;
/// Euler's number (used by the Hammerstad–Bekkadal thickness term).
const E: f64 = std::f64::consts::E;

/// Which kind of line a model describes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LineKind {
    /// Outer-layer trace over one reference plane.
    Microstrip,
    /// Inner-layer trace centred between two reference planes.
    Stripline,
}

/// Which dimension of an edge-coupled pair is held fixed by [`diff_pair_geometry`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PairFixed {
    /// The edge-to-edge gap `s` (nm) is given; the width is solved.
    Gap(Coord),
    /// The trace width `w` (nm) is given; the gap is solved.
    Width(Coord),
}

#[allow(clippy::cast_precision_loss)]
fn f(v: Coord) -> f64 {
    v as f64
}

fn coth(x: f64) -> f64 {
    let q = exp(-2.0 * x);
    (1.0 + q) / (1.0 - q)
}

fn tanh(x: f64) -> f64 {
    let q = exp(-2.0 * x);
    (1.0 - q) / (1.0 + q)
}

/// Hammerstad–Jensen air-filled microstrip impedance `Z01(u)` (Ω), `u = w/h`:
/// `(η0/2π)·ln(f(u)/u + √(1+(2/u)²))`, `f(u) = 6+(2π−6)·exp(−(30.666/u)^0.7528)`.
fn z01(u: f64) -> f64 {
    let tau = std::f64::consts::TAU;
    let fu = 6.0 + (tau - 6.0) * exp(-pow(30.666 / u, 0.7528));
    ETA0 / tau * ln(fu / u + (1.0 + (2.0 / u) * (2.0 / u)).sqrt())
}

/// Hammerstad–Jensen effective permittivity of a zero-thickness microstrip.
fn eps_eff_u(u: f64, er: f64) -> f64 {
    let u4 = u * u * u * u;
    let a = 1.0
        + ln((u4 + (u / 52.0) * (u / 52.0)) / (u4 + 0.432)) / 49.0
        + ln(1.0 + pow(u / 18.1, 3.0)) / 18.7;
    let b = 0.564 * pow((er - 0.9) / (er + 3.0), 0.053);
    (er + 1.0) / 2.0 + (er - 1.0) / 2.0 * pow(1.0 + 10.0 / u, -a * b)
}

/// Microstrip impedance (Ω) and effective permittivity of a trace of width `w`, copper
/// thickness `t`, over a dielectric of height `h` and relative permittivity `er`.
///
/// Hammerstad & Jensen, "Accurate Models for Microstrip Computer-Aided Design", IEEE MTT-S
/// Int. Microwave Symp. Dig., 1980 (Z01 error < 0.03 %, εeff < 0.2 %), with the
/// finite-thickness correction of Hammerstad & Bekkadal (1975):
/// `Δu1 = (t/πh)·ln(1 + 4e/((t/h)·coth²√(6.517u)))`,
/// `Δur = ½(1+1/cosh√(εr−1))·Δu1`, `Z0 = Z01(u+Δur)/√εeff(u+Δur, εr)`,
/// `εeff,t = εeff(u+Δur)·(Z01(u+Δu1)/Z01(u+Δur))²`.
/// Returns `None` for non-positive/non-finite inputs.
#[must_use]
pub fn microstrip_hj(w: Coord, h: Coord, t: Coord, er: f64) -> Option<(f64, f64)> {
    if w <= 0 || h <= 0 || t < 0 || !(er.is_finite() && er >= 1.0) {
        return None;
    }
    let (u, th) = (f(w) / f(h), f(t) / f(h));
    let (u1, ur) = if t == 0 {
        (u, u)
    } else {
        let c = coth((6.517 * u).sqrt());
        let d1 = th / std::f64::consts::PI * ln(1.0 + 4.0 * E / (th * c * c));
        let x = (er - 1.0).sqrt();
        let sech = 2.0 / (exp(x) + exp(-x));
        (u + d1, u + 0.5 * (1.0 + sech) * d1)
    };
    let eur = eps_eff_u(ur, er);
    let ratio = z01(u1) / z01(ur);
    Some((z01(ur) / eur.sqrt(), eur * ratio * ratio))
}

/// Smallest `x` in `[lo, hi]` (nm) with `pred(x)`, for a `pred` that is false then true
/// (monotone). Pure integer bisection: deterministic with nm resolution.
fn bisect(lo: Coord, hi: Coord, pred: impl Fn(Coord) -> bool) -> Option<Coord> {
    if !pred(hi) {
        return None;
    }
    if pred(lo) {
        return Some(lo);
    }
    let (mut lo, mut hi) = (lo, hi);
    while hi - lo > 1 {
        let mid = lo + (hi - lo) / 2;
        if pred(mid) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    Some(hi)
}

fn round_up_nm(nm: Coord) -> Coord {
    (nm + WIDTH_STEP - 1) / WIDTH_STEP * WIDTH_STEP
}

/// Unrounded (nm-resolution) microstrip width whose [`microstrip_hj`] impedance is ≤ `z0`.
fn microstrip_width_hj_nm(z0_ohm: f64, h: Coord, t: Coord, er: f64) -> Option<Coord> {
    if !(z0_ohm.is_finite() && z0_ohm > 0.0) || h <= 0 {
        return None;
    }
    bisect(1_000, 100 * h, |w| microstrip_hj(w, h, t, er).is_some_and(|(z, _)| z <= z0_ohm))
        .filter(|&w| w > 1_000)
}

/// Microstrip width (nm) for impedance `z0_ohm` by monotone bisection on the
/// Hammerstad–Jensen model ([`microstrip_hj`]), rounded **up** to [`WIDTH_STEP`].
/// `None` if no width in `1 µm … 100·h` reaches the impedance.
#[must_use]
pub fn microstrip_width_hj(z0_ohm: f64, h: Coord, t: Coord, er: f64) -> Option<Coord> {
    microstrip_width_hj_nm(z0_ohm, h, t, er).map(round_up_nm)
}

/// Symmetric stripline impedance (Ω) of a strip of width `w`, thickness `t`, centred between
/// planes `b` apart (dielectric `er`). Cohn (1955) / Wadell, *Transmission Line Design
/// Handbook* (1991) §4.3 form with finite thickness:
/// `Z0 = η0/(4√εr) / (w/(b−t) + Cf)`, `x = 1/(1−t/b)`,
/// `Cf = (1/π)[2x·ln(x+1) − (x−1)·ln(x²−1)]` (→ 2 ln2/π = 0.441 at t = 0, Cohn's wide-strip
/// limit). Valid for `t/b < 0.25` and `w/(b−t) ≥ 0.35`; `None` outside that range.
#[must_use]
pub fn stripline_z0(w: Coord, b: Coord, t: Coord, er: f64) -> Option<f64> {
    if w <= 0 || b <= 0 || t < 0 || !(er.is_finite() && er >= 1.0) || 4 * t >= b {
        return None;
    }
    let (wf, bt) = (f(w), f(b - t));
    if wf / bt < 0.35 {
        return None;
    }
    let x = 1.0 / (1.0 - f(t) / f(b));
    let second = if t == 0 { 0.0 } else { (x - 1.0) * ln(x * x - 1.0) };
    let cf = (2.0 * x * ln(x + 1.0) - second) / std::f64::consts::PI;
    Some(ETA0 / (4.0 * er.sqrt()) / (wf / bt + cf))
}

/// Stripline width (nm) for `z0_ohm` between planes `b` apart, by bisection on
/// [`stripline_z0`], rounded **up** to [`WIDTH_STEP`]. `None` if outside its validity range.
#[must_use]
pub fn stripline_width_cohn(z0_ohm: f64, b: Coord, t: Coord, er: f64) -> Option<Coord> {
    if !(z0_ohm.is_finite() && z0_ohm > 0.0) || b <= 0 || t < 0 || 4 * t >= b {
        return None;
    }
    // Smallest width inside the model's validity range (w/(b−t) ≥ 0.35).
    let lo = (f(b - t) * 0.35).ceil() as Coord;
    if stripline_z0(lo, b, t, er)? < z0_ohm {
        return None; // would need a narrower strip than the model is valid for
    }
    bisect(lo, 100 * b, |w| stripline_z0(w, b, t, er).is_some_and(|z| z <= z0_ohm)).map(round_up_nm)
}

/// Arithmetic–geometric mean (fixed 40 iterations: converged, deterministic).
fn agm(mut a: f64, mut b: f64) -> f64 {
    for _ in 0..40 {
        (a, b) = ((a + b) / 2.0, (a * b).sqrt());
    }
    a
}

/// `K(k')/K(k)` of the complete elliptic integral from the modulus `k` and complement
/// `kp = √(1−k²)`, via `K(k) = π/(2·AGM(1, k'))`.
fn k_ratio(k: f64, kp: f64) -> f64 {
    agm(1.0, kp) / agm(1.0, k)
}

/// Impedance (Ω) of a **zero-thickness** symmetric stripline, exact (Cohn 1955):
/// `Z0 = 30π/√εr · K(k)/K(k')`, `k = sech(πw/2b)`. Reference for [`stripline_z0`].
#[must_use]
pub fn stripline_z0_cohn_exact(w: Coord, b: Coord, er: f64) -> Option<f64> {
    if w <= 0 || b <= 0 || !(er.is_finite() && er >= 1.0) {
        return None;
    }
    let a = std::f64::consts::PI * f(w) / (2.0 * f(b));
    let k = 2.0 / (exp(a) + exp(-a));
    let kp = tanh(a);
    Some(30.0 * std::f64::consts::PI / er.sqrt() / k_ratio(k, kp))
}

/// Even/odd-mode impedances (Ω) of a **zero-thickness** edge-coupled symmetric stripline
/// (strip width `w`, edge gap `s`, plane spacing `b`). Cohn, "Shielded Coupled-Strip
/// Transmission Line", IRE Trans. MTT-3, 1955:
/// `Z0e = 30π/√εr · K(k_e')/K(k_e)`, `k_e = tanh(πw/2b)·tanh(π(w+s)/2b)`,
/// `Z0o = 30π/√εr · K(k_o')/K(k_o)`, `k_o = tanh(πw/2b)·coth(π(w+s)/2b)`.
/// Thickness is not modelled (no citable closed form); real traces are thicker, so the true
/// impedance is slightly lower. Returns `(Z0e, Z0o)`.
#[must_use]
pub fn stripline_coupled_cohn(w: Coord, s: Coord, b: Coord, er: f64) -> Option<(f64, f64)> {
    if w <= 0 || s <= 0 || b <= 0 || !(er.is_finite() && er >= 1.0) {
        return None;
    }
    let pi = std::f64::consts::PI;
    let a = pi * f(w) / (2.0 * f(b));
    let c = pi * f(w + s) / (2.0 * f(b));
    let (ke, ko) = (tanh(a) * tanh(c), tanh(a) * coth(c));
    let pref = 30.0 * pi / er.sqrt();
    Some((pref * k_ratio(ke, (1.0 - ke * ke).sqrt()), pref * k_ratio(ko, (1.0 - ko * ko).sqrt())))
}

/// Differential impedance (Ω) of an edge-coupled **microstrip** pair (width `w`, gap `s`,
/// dielectric height `h`): IPC-2141A, `Zdiff = 2·Z0·(1 − 0.48·exp(−0.96·s/h))` with `Z0` the
/// thickness-corrected Hammerstad–Jensen single-line impedance ([`microstrip_hj`]).
/// This is an engineering approximation (IPC-2141A quotes it for loosely to moderately
/// coupled pairs, roughly `0.5 ≤ s/h`, `0.1 ≤ w/h ≤ 3`); the exact Kirschning–Jansen
/// odd-mode equations are intentionally not implemented (their coefficient set could not be
/// cited verbatim). It is monotone: wider trace → lower, larger gap → higher, and
/// `Zdiff → 2·Z0` as `s → ∞`.
#[must_use]
pub fn microstrip_diff_ipc2141a(w: Coord, s: Coord, h: Coord, t: Coord, er: f64) -> Option<f64> {
    if s <= 0 {
        return None;
    }
    let z0 = microstrip_hj(w, h, t, er)?.0;
    Some(2.0 * z0 * (1.0 - 0.48 * exp(-0.96 * f(s) / f(h))))
}

/// Differential impedance (Ω) of an edge-coupled pair (width `w`, gap `s`): microstrip
/// ([`microstrip_diff_ipc2141a`], `h` = dielectric height) or stripline (`2·Z0o`, Cohn,
/// [`stripline_coupled_cohn`], `h` = plane spacing `b`, thickness ignored).
#[must_use]
pub fn diff_impedance(
    kind: LineKind,
    w: Coord,
    s: Coord,
    h: Coord,
    t: Coord,
    er: f64,
) -> Option<f64> {
    match kind {
        LineKind::Microstrip => microstrip_diff_ipc2141a(w, s, h, t, er),
        LineKind::Stripline => stripline_coupled_cohn(w, s, h, er).map(|r| 2.0 * r.1),
    }
}

/// Width and gap `(w, s)` (nm) of an edge-coupled differential pair with differential
/// impedance `zdiff_ohm` (see [`diff_impedance`] for the models). Exactly one dimension is fixed
/// by `fixed`; the other is solved by monotone bisection (a wider trace lowers Z, a larger
/// gap raises it) and rounded **up** to [`WIDTH_STEP`]. `h` is the dielectric height
/// (microstrip) or plane spacing `b` (stripline). `None` if the target is unreachable
/// within `h/20 … 20h`.
#[must_use]
pub fn diff_pair_geometry(
    zdiff_ohm: f64,
    kind: LineKind,
    fixed: PairFixed,
    h: Coord,
    t: Coord,
    er: f64,
) -> Option<(Coord, Coord)> {
    if !(zdiff_ohm.is_finite() && zdiff_ohm > 0.0) || h <= 0 {
        return None;
    }
    match fixed {
        PairFixed::Gap(s) => {
            let w = bisect(h / 20, 20 * h, |w| {
                diff_impedance(kind, w, s, h, t, er).is_some_and(|z| z <= zdiff_ohm)
            })?;
            Some((round_up_nm(w), s))
        }
        PairFixed::Width(w) => {
            let s = bisect(h / 20, 20 * h, |s| {
                diff_impedance(kind, w, s, h, t, er).is_some_and(|z| z >= zdiff_ohm)
            })?;
            Some((w, round_up_nm(s)))
        }
    }
}

/// Signal propagation delay (ps/mm) `= √εeff / c`. Microstrip uses the Hammerstad–Jensen
/// `εeff` of a trace of width `w` over dielectric height `h`; stripline is homogeneous, so
/// `εeff = εr` (`w`, `h`, `t` are ignored).
#[must_use]
pub fn delay_ps_per_mm(kind: LineKind, w: Coord, h: Coord, t: Coord, er: f64) -> Option<f64> {
    let eeff = match kind {
        LineKind::Microstrip => microstrip_hj(w, h, t, er)?.1,
        LineKind::Stripline => {
            if !(er.is_finite() && er >= 1.0) {
                return None;
            }
            er
        }
    };
    Some(eeff.sqrt() / C_MM_PER_PS)
}

/// `(microstrip, stripline)` widths (nm) for impedance `z_ohm`: the widest over all outer
/// layers (microstrip over their own dielectric) and over all inner layers (stripline in
/// their own plane spacing; 0 on boards without inner layers or inner spacing).
fn impedance_widths(board: &Board, z_ohm: f64) -> (Coord, Coord) {
    let s = &board.stackup;
    let n = board.layer_count();
    let t_outer = s.copper_thickness.first().copied().unwrap_or(35_000);
    let thick = |l: u8| s.copper_thickness.get(usize::from(l)).copied().unwrap_or(t_outer);
    let micro = [0, n.saturating_sub(1)]
        .into_iter()
        .map(|l| {
            let (h, er) = s.outer_h_er(LayerId(l), n);
            microstrip_width_hj(z_ohm, h, thick(l), er).unwrap_or(0)
        })
        .max()
        .unwrap_or(0);
    let strip = if n > 2 {
        (1..n - 1)
            .filter_map(|l| {
                let (b, er) = s.stripline_b_er(LayerId(l), n)?;
                let t = s.copper_thickness.get(usize::from(l)).copied().unwrap_or(t_outer);
                Some(stripline_width_cohn(z_ohm, b, t, er).unwrap_or(0))
            })
            .max()
            .unwrap_or(0)
    } else {
        0
    };
    (micro, strip)
}

/// Width (nm) net `net` must have on every layer: the widest of its net-class width, the
/// current-derived width (internal-layer formula when the board has inner layers — the
/// conservative case), the Hammerstad–Jensen microstrip width (outer layers) and, on
/// multilayer boards with `Stackup::inner_dielectric`, the Cohn stripline width.
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
    let z_ohm = c.impedance_mohm.map(|z| f64::from(z) / 1000.0);
    let (micro_w, strip_w) = z_ohm.map_or((0, 0), |z| impedance_widths(board, z));
    class_w.max(current_w).max(micro_w).max(strip_w).max(board.rules.min_width_for_net(board, net))
}

/// `true` when the net has an impedance target that no trace width can reach with this
/// stack-up on its outer layers, or on inner layers when the board has them and declares
/// an inner dielectric. Such a rule cannot be met by width and must be reported.
#[must_use]
pub fn impedance_unattainable(board: &Board, net: NetId) -> bool {
    let Some(z) = board.nets.get(net.idx()).and_then(|n| n.constraints.impedance_mohm) else {
        return false;
    };
    let z = f64::from(z) / 1000.0;
    let s = &board.stackup;
    let n = board.layer_count();
    let t_outer = s.copper_thickness.first().copied().unwrap_or(35_000);
    let outer = [LayerId(0), LayerId(n.saturating_sub(1))].into_iter().any(|l| {
        let (h, er) = s.outer_h_er(l, n);
        let t = s.copper_thickness.get(usize::from(l.0)).copied().unwrap_or(t_outer);
        microstrip_width_hj(z, h, t, er).is_none()
    });
    let inner = n > 2
        && (1..n - 1).any(|l| {
            s.stripline_b_er(LayerId(l), n).is_some_and(|(b, er)| {
                let t = s.copper_thickness.get(usize::from(l)).copied().unwrap_or(t_outer);
                stripline_width_cohn(z, b, t, er).is_none()
            })
        });
    outer || inner
}

/// Applies derived widths and custom-rule bounds: every net whose required width,
/// clearance, via diameter or drill (per-net electrical rules, KiCad custom rules) exceeds
/// its class moves to a derived class (`<class>/<net>`) with those values — never narrower
/// or looser (INV-RULE-1). Custom length limits are merged into the net constraints and
/// custom edge/hole clearances raise the board-wide values. Returns notes per change.
pub fn resolve(board: &mut Board) -> Vec<String> {
    let mut notes = Vec::new();
    if board.rules.has_custom(crate::custom::ConstraintKind::EdgeClearance) {
        let v = board.rules.max_custom_min(crate::custom::ConstraintKind::EdgeClearance);
        board.rules.edge_clearance = board.rules.edge_clearance.max(v);
    }
    if board.rules.has_custom(crate::custom::ConstraintKind::HoleClearance) {
        let v = board.rules.max_custom_min(crate::custom::ConstraintKind::HoleClearance);
        board.rules.hole_clearance = board.rules.hole_clearance.max(v);
    }
    // KiCad custom `diff_pair_gap` rules raise the target gap of matching pairs.
    for k in 0..board.diff_pairs.len() {
        let (p, n) = (board.diff_pairs[k].p, board.diff_pairs[k].n);
        let g = board
            .rules
            .diff_pair_gap_for_net(board, p)
            .max(board.rules.diff_pair_gap_for_net(board, n));
        board.diff_pairs[k].gap = board.diff_pairs[k].gap.max(g);
    }
    for i in 1..board.nets.len() {
        let net = NetId(u32::try_from(i).unwrap_or(u32::MAX));
        if !board.rules.custom.is_empty() {
            let (lo, hi) = board.rules.length_limits_for_net(board, net);
            let c = &mut board.nets[i].constraints;
            c.length_min = lo.map_or(c.length_min, |m| Some(c.length_min.map_or(m, |o| o.max(m))));
            c.length_max = hi.map_or(c.length_max, |m| Some(c.length_max.map_or(m, |o| o.min(m))));
        }
        let base = board.rules.class_of(board, net).clone();
        let need_w = required_width(board, net)
            .max(board.rules.min_width_for_net(board, net))
            .max(crate::pairs::pair_min_width(board, net));
        let need_c = board.rules.max_clearance_for_net(board, net);
        let need_drill = base.via_drill.max(board.rules.min_hole_for_net(board, net));
        let need_via =
            base.via_diameter.max(board.rules.min_via_diameter_for_net(board, net, need_drill));
        if need_w <= base.track_width
            && need_c <= base.clearance
            && need_via <= base.via_diameter
            && need_drill <= base.via_drill
        {
            continue;
        }
        let name = format!("{}/{}", base.name, board.nets[i].name);
        let derived = NetClass {
            name,
            track_width: need_w.max(base.track_width),
            clearance: need_c.max(base.clearance),
            via_diameter: need_via.max(base.via_diameter),
            via_drill: need_drill.max(base.via_drill),
        };
        let old_w = base.track_width;
        if derived.track_width > old_w {
            notes.push(format!(
                "net {}: width {} mm → {} mm ({})",
                board.nets[i].name,
                format_mm(old_w),
                format_mm(derived.track_width),
                if board.rules.custom.is_empty() {
                    "current/impedance rules"
                } else {
                    "current/impedance/custom rules"
                }
            ));
        }
        if derived.clearance > base.clearance {
            notes.push(format!(
                "net {}: clearance {} mm → {} mm (custom rules)",
                board.nets[i].name,
                format_mm(base.clearance),
                format_mm(derived.clearance)
            ));
        }
        board.rules.classes.push(derived);
        board.nets[i].class = u16::try_from(board.rules.classes.len() - 1).unwrap_or(u16::MAX);
    }
    notes
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unreachable_impedance_is_flagged_not_silently_dropped() {
        let mut b = crate::builder::BoardBuilder::new(30_000_000, 20_000_000, 4);
        let n = b.net("RF");
        let mut board = b.build();
        board.nets[n.idx()].constraints.impedance_mohm = Some(50_000);
        assert!(!impedance_unattainable(&board, n), "50 Ω outer microstrip is reachable");
        // A 50 µm plane spacing cannot host a 50 Ω stripline of 35 µm copper.
        board.stackup.inner_dielectric = Some(50_000);
        board.stackup.copper_thickness = vec![35_000; 4];
        assert!(impedance_unattainable(&board, n));
        board.nets[n.idx()].constraints.impedance_mohm = None;
        assert!(!impedance_unattainable(&board, n));
    }
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
        MatchGroup { name: name.into(), skew_max: skew, by: MatchBy::Length, nets: nets.to_vec() }
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

    // ---- transmission-line models (task #20) ------------------------------------------

    fn rel(a: f64, b: f64) -> f64 {
        ((a - b) / b).abs()
    }

    #[test]
    fn hj_microstrip_matches_published_values() {
        // Hammerstad–Jensen 1980 / Pozar (Microwave Engineering, §3.8): a 50 Ω line on
        // εr = 2.2 (RT/duroid 5880) has W/h ≈ 3.08 and εeff ≈ 1.87–1.88.
        let (z, e) = microstrip_hj(308_000, 100_000, 0, 2.2).expect("valid");
        assert!(rel(z, 50.0) < 0.02, "Z0 {z}");
        assert!(rel(e, 1.88) < 0.02, "εeff {e}");
        // Alumina εr = 9.8, W/h = 1: ≈ 50 Ω, εeff ≈ 6.6 (H&J 1980, Fig. 2 / Gupta et al.,
        // Microstrip Lines and Slotlines, Table 2.1 style values).
        let (z, e) = microstrip_hj(100_000, 100_000, 0, 9.8).expect("valid");
        assert!(rel(z, 50.0) < 0.02, "Z0 {z}");
        assert!(rel(e, 6.6) < 0.02, "εeff {e}");
        // Air line limit: εr = 1 gives εeff = 1 and Z0 = Z01.
        let (_, e1) = microstrip_hj(100_000, 100_000, 0, 1.0).expect("valid");
        assert!((e1 - 1.0).abs() < 1e-9);
    }

    #[test]
    fn hj_fr4_50_ohm_width() {
        let w = microstrip_width_hj(50.0, 200_000, 35_000, 4.3).expect("attainable");
        assert!((350_000..=380_000).contains(&w), "50 Ω on 0.2 mm FR-4: {w}");
        assert_eq!(w % WIDTH_STEP, 0);
        let exact = microstrip_width_hj_nm(50.0, 200_000, 35_000, 4.3).expect("w");
        let (z, e) = microstrip_hj(exact, 200_000, 35_000, 4.3).expect("valid");
        assert!(rel(z, 50.0) < 0.005, "round trip {z}");
        assert!((3.0..=3.4).contains(&e), "εeff {e}");
        // Finite thickness widens the effective trace → a thicker trace needs less width.
        let thin = microstrip_width_hj_nm(50.0, 200_000, 0, 4.3).expect("w");
        assert!(exact < thin, "{exact} < {thin}");
    }

    #[test]
    fn hj_solver_is_monotone_deterministic_and_bounded() {
        let widths: Vec<Coord> = [30.0, 40.0, 50.0, 65.0, 80.0, 100.0]
            .iter()
            .map(|&z| microstrip_width_hj(z, 200_000, 35_000, 4.3).expect("w"))
            .collect();
        assert!(widths.windows(2).all(|p| p[0] >= p[1]), "higher Z → narrower: {widths:?}");
        for z in [35.0, 50.0, 75.0] {
            let w = microstrip_width_hj_nm(z, 150_000, 18_000, 3.66).expect("w");
            let (got, _) = microstrip_hj(w, 150_000, 18_000, 3.66).expect("z");
            assert!(rel(got, z) < 0.01, "{z}: {got}");
            let (below, _) = microstrip_hj(w - 1, 150_000, 18_000, 3.66).expect("z");
            assert!(below > z, "smallest nm width reaching {z}: {below}");
        }
        assert_eq!(
            microstrip_width_hj(50.0, 200_000, 35_000, 4.3),
            microstrip_width_hj(50.0, 200_000, 35_000, 4.3)
        );
        assert!(microstrip_width_hj(0.0, 200_000, 35_000, 4.3).is_none());
        assert!(microstrip_width_hj(f64::NAN, 200_000, 35_000, 4.3).is_none());
        assert!(microstrip_width_hj(50.0, 0, 35_000, 4.3).is_none());
        assert!(microstrip_hj(100_000, 100_000, 0, 0.5).is_none());
        // 1 Ω needs more than 100·h of width.
        assert!(microstrip_width_hj(1.0, 200_000, 35_000, 4.3).is_none());
    }

    #[test]
    fn stripline_cohn_matches_exact_and_reference() {
        // Wide-strip thickness form at t = 0 vs Cohn's exact elliptic result: < 1.5 %.
        for w in [140_000, 200_000, 300_000, 600_000] {
            let ex = stripline_z0_cohn_exact(w, 400_000, 4.3).expect("z");
            let ap = stripline_z0(w, 400_000, 0, 4.3).expect("z");
            assert!(rel(ap, ex) < 0.015, "w {w}: {ap} vs {ex}");
        }
        // Cohn: Z0 = 94.15/√εr /(w/b + 0.441) in the wide-strip limit (t = 0).
        let z = stripline_z0(400_000, 400_000, 0, 4.3).expect("z");
        assert!(rel(z, 94.15 / 4.3_f64.sqrt() / (1.0 + 0.441)) < 0.01, "{z}");
        // 50 Ω, b = 0.4 mm, t = 17.5 µm, εr = 4.3 (Wadell, Transmission Line Design Handbook
        // §4.3 thickness form): ≈ 0.16 mm.
        let w = stripline_width_cohn(50.0, 400_000, 17_500, 4.3).expect("w");
        assert!((150_000..=180_000).contains(&w), "{w}");
        assert_eq!(w % WIDTH_STEP, 0);
        let z = stripline_z0(w, 400_000, 17_500, 4.3).expect("z");
        assert!(z <= 50.0 && rel(z, 50.0) < 0.03, "round trip {z}");
        // Thicker copper → lower Z0 at equal width.
        assert!(
            stripline_z0(200_000, 400_000, 35_000, 4.3).expect("z")
                < stripline_z0(200_000, 400_000, 0, 4.3).expect("z")
        );
        // Out of validity range / bad input.
        assert!(stripline_z0(50_000, 400_000, 0, 4.3).is_none(), "w/b < 0.35");
        assert!(stripline_z0(200_000, 400_000, 120_000, 4.3).is_none(), "t/b ≥ 0.25");
        assert!(stripline_width_cohn(150.0, 400_000, 17_500, 4.3).is_none(), "needs narrow strip");
        assert!(stripline_width_cohn(0.0, 400_000, 17_500, 4.3).is_none());
        let wide = |z| stripline_width_cohn(z, 400_000, 17_500, 4.3).expect("w");
        assert!(wide(40.0) > wide(45.0) && wide(45.0) >= wide(50.0));
    }

    #[test]
    fn coupled_stripline_limits_and_ordering() {
        let (w, b) = (150_000, 400_000);
        let single = stripline_z0_cohn_exact(w, b, 4.3).expect("z");
        let (ze, zo) = stripline_coupled_cohn(w, 20_000_000, b, 4.3).expect("z");
        assert!(rel(ze, single) < 0.005 && rel(zo, single) < 0.005, "{ze} {zo} vs {single}");
        let mut last = 0.0;
        for s in [50_000, 100_000, 200_000, 400_000, 800_000] {
            let (ze, zo) = stripline_coupled_cohn(w, s, b, 4.3).expect("z");
            assert!(ze > zo && zo > last && ze > single && zo < single, "s {s}: {ze} {zo}");
            last = zo;
        }
    }

    #[test]
    fn microstrip_diff_follows_ipc2141a() {
        let z0 = microstrip_hj(200_000, 200_000, 35_000, 4.3).expect("z").0;
        let zd = microstrip_diff_ipc2141a(200_000, 200_000, 200_000, 35_000, 4.3).expect("z");
        assert!(rel(zd, 2.0 * z0 * (1.0 - 0.48 * (-0.96_f64).exp())) < 1e-9);
        let far = microstrip_diff_ipc2141a(200_000, 50_000_000, 200_000, 35_000, 4.3).expect("z");
        assert!(rel(far, 2.0 * z0) < 1e-6, "uncoupled limit");
        assert!(zd < far);
    }

    #[test]
    fn diff_pair_geometry_roundtrips() {
        for (kind, h, t) in
            [(LineKind::Microstrip, 200_000, 35_000), (LineKind::Stripline, 400_000, 0)]
        {
            // Gap fixed → width solved.
            let (w, s) =
                diff_pair_geometry(100.0, kind, PairFixed::Gap(150_000), h, t, 4.3).expect("g");
            assert_eq!(s, 150_000);
            assert_eq!(w % WIDTH_STEP, 0);
            let z = diff_impedance(kind, w, s, h, t, 4.3).expect("z");
            assert!(z <= 100.0 && rel(z, 100.0) < 0.03, "{kind:?} gap-fixed {z} (w {w})");
            // Width fixed → gap solved.
            let (w2, s2) =
                diff_pair_geometry(100.0, kind, PairFixed::Width(w), h, t, 4.3).expect("w");
            assert_eq!(w2, w);
            let z = diff_impedance(kind, w2, s2, h, t, 4.3).expect("z");
            assert!(z >= 100.0 && rel(z, 100.0) < 0.03, "{kind:?} width-fixed {z} (s {s2})");
            // Determinism and monotonicity: higher Zdiff → narrower at equal gap.
            let a = diff_pair_geometry(100.0, kind, PairFixed::Gap(150_000), h, t, 4.3);
            assert_eq!(a, diff_pair_geometry(100.0, kind, PairFixed::Gap(150_000), h, t, 4.3));
            let (w_hi, _) =
                diff_pair_geometry(120.0, kind, PairFixed::Gap(150_000), h, t, 4.3).expect("g");
            assert!(w_hi <= w, "{w_hi} <= {w}");
            // Unreachable: a pair cannot exceed 2·Z0 of its uncoupled trace within range.
            assert!(
                diff_pair_geometry(1000.0, kind, PairFixed::Width(w), h, t, 4.3).is_none()
                    || kind == LineKind::Stripline
            );
        }
        assert!(
            diff_pair_geometry(0.0, LineKind::Microstrip, PairFixed::Gap(1), 1, 0, 4.3).is_none()
        );
    }

    #[test]
    fn propagation_delay() {
        let w = microstrip_width_hj_nm(50.0, 200_000, 35_000, 4.3).expect("w");
        let d = delay_ps_per_mm(LineKind::Microstrip, w, 200_000, 35_000, 4.3).expect("d");
        assert!((5.8..=6.1).contains(&d), "microstrip {d} ps/mm");
        let (_, e) = microstrip_hj(w, 200_000, 35_000, 4.3).expect("z");
        assert!(rel(d, e.sqrt() / 0.299_792_458) < 1e-12);
        let s = delay_ps_per_mm(LineKind::Stripline, 0, 0, 0, 4.3).expect("d");
        assert!((6.85..=6.95).contains(&s), "stripline {s} ps/mm");
        assert!(s > d, "stripline is slower than microstrip");
        assert!(delay_ps_per_mm(LineKind::Stripline, 0, 0, 0, f64::NAN).is_none());
        assert!(delay_ps_per_mm(LineKind::Microstrip, 0, 200_000, 0, 4.3).is_none());
    }

    #[test]
    fn required_width_uses_hj_and_stripline_and_never_narrows() {
        let mut b = BoardBuilder::new(mm(30), mm(20), 2);
        let n = b.net("CLK");
        let mut board = b.build();
        board.stackup.outer_dielectric = 200_000;
        board.nets[n.idx()].constraints.impedance_mohm = Some(50_000);
        let class_w = board.rules.class_of(&board, n).track_width;
        let hj = microstrip_width_hj(50.0, 200_000, 35_000, 4.3).expect("w");
        assert_eq!(required_width(&board, n), hj.max(class_w));
        // 2-layer boards ignore the inner dielectric.
        board.stackup.inner_dielectric = Some(400_000);
        assert_eq!(required_width(&board, n), hj.max(class_w));
        // 4-layer: widest of microstrip and stripline.
        let mut b4 = BoardBuilder::new(mm(30), mm(20), 4);
        let n4 = b4.net("CLK");
        let mut board4 = b4.build();
        board4.stackup.outer_dielectric = 200_000;
        board4.stackup.inner_dielectric = Some(400_000);
        board4.nets[n4.idx()].constraints.impedance_mohm = Some(50_000);
        board4.stackup.copper_thickness = vec![35_000, 17_500, 17_500, 35_000];
        let sl = stripline_width_cohn(50.0, 400_000, 17_500, 4.3).expect("w");
        let class4 = board4.rules.class_of(&board4, n4).track_width;
        assert_eq!(required_width(&board4, n4), hj.max(sl).max(class4));
        assert!(required_width(&board4, n4) >= class4, "never narrower than the class");
    }
    fn delay_board(layers: u8) -> (Board, NetId) {
        let mut b = BoardBuilder::new(mm(60), mm(30), layers);
        let n = b.net("S");
        let mut board = b.build();
        board.stackup.outer_dielectric = 200_000;
        board.stackup.epsilon_r_milli = 4300;
        (board, n)
    }

    fn run(board: &mut Board, n: NetId, layer: u8, len_mm: i64) {
        board.tracks.push(crate::Track {
            start: ferroute_geom::Point::new(0, 0),
            end: ferroute_geom::Point::new(mm(len_mm), 0),
            width: 300_000,
            layer: LayerId(layer),
            net: n,
            locked: false,
            origin: crate::Origin::Routed,
        });
    }

    fn via(board: &mut Board, n: NetId, layers: crate::LayerSet) {
        board.vias.push(crate::Via {
            pos: ferroute_geom::Point::new(0, 0),
            diameter: 600_000,
            drill: 300_000,
            layers,
            net: n,
            locked: false,
            origin: crate::Origin::Routed,
        });
    }

    #[test]
    fn delay_depends_on_layer_kind_and_vias() {
        let (mut b, n) = delay_board(4);
        run(&mut b, n, 0, 10);
        let outer = net_delays_fs(&b)[n.idx()];
        assert!((57_000..=62_000).contains(&outer), "microstrip 10 mm ≈ 59 ps: {outer}");
        let (mut b, n) = delay_board(4);
        run(&mut b, n, 1, 10);
        let inner = net_delays_fs(&b)[n.idx()];
        assert!((inner - 69_170).abs() < 200, "stripline 10 mm εr 4.3 ≈ 69 ps: {inner}");
        let (mut b, n) = delay_board(4);
        run(&mut b, n, 1, 10);
        let all = b.all_layers();
        via(&mut b, n, all);
        let through = net_delays_fs(&b)[n.idx()] - inner;
        assert!((through - 11_070).abs() < 100, "1.6 mm barrel ≈ 11 ps: {through}");
        let (mut b, n) = delay_board(4);
        via(&mut b, n, crate::LayerSet(0b0011));
        let blind = net_delays_fs(&b)[n.idx()];
        assert!((blind - 11_070 / 3).abs() < 100, "L1–L2 spans a third of the stack: {blind}");
        // Arcs count like tracks.
        let (mut b, n) = delay_board(2);
        b.arcs.push(crate::ArcTrack {
            start: ferroute_geom::Point::new(0, 0),
            mid: ferroute_geom::Point::new(mm(1), mm(1)),
            end: ferroute_geom::Point::new(mm(2), 0),
            width: 300_000,
            layer: LayerId(0),
            net: n,
            locked: false,
            origin: crate::Origin::Routed,
        });
        assert!(net_delays_fs(&b)[n.idx()] > 0);
    }

    #[test]
    fn delay_groups_use_their_own_unit_and_leave_length_groups_alone() {
        let (mut b, ids) = match_board(&["A", "B", "C"]);
        b.match_groups = vec![
            MatchGroup {
                name: "t".into(),
                skew_max: 10_000,
                by: MatchBy::Delay,
                nets: ids.clone(),
            },
            group("l", mm(1), &[ids[0], ids[1]]),
        ];
        let d = [0, 100_000, 60_000, 99_000];
        assert_eq!(match_delay_targets(&b, &d), vec![0, 100_000, 95_000, 99_000]);
        let l = [0, mm(5), mm(1), mm(5)];
        assert_eq!(match_targets(&b, &l), vec![0, mm(5), mm(5) - mm(1) / 2, mm(5)]);
        assert_eq!(b.match_groups[0].spread(&l, &d), 40_000, "delay unit");
        assert_eq!(b.match_groups[1].spread(&l, &d), mm(4), "length unit");
    }
}

#[cfg(test)]
mod stackup_tests {
    use super::*;
    use crate::builder::BoardBuilder;
    use ferroute_geom::units::mm_to_nm;

    fn diel(t: f64, er: u32) -> Dielectric {
        Dielectric {
            thickness: mm_to_nm(t),
            epsilon_r_milli: er,
            between: (LayerId(0), LayerId(0)),
        }
    }

    fn four(outer: f64) -> Stackup {
        Stackup::from_layers(
            vec![35_000, 17_500, 17_500, 35_000],
            vec![diel(outer, 4500), diel(0.8, 4000), diel(outer, 4500)],
            None,
        )
        .expect("valid")
    }

    #[test]
    fn from_layers_derives_scalars_and_rejects_bad_input() {
        let s = four(0.2);
        assert_eq!(s.outer_dielectric, 200_000);
        assert_eq!(s.board_thickness, 35_000 * 2 + 17_500 * 2 + 1_200_000);
        assert_eq!(s.dielectrics[2].between, (LayerId(2), LayerId(3)));
        assert_eq!(s.outer_h_er(LayerId(3), 4), (200_000, 4.5));
        assert_eq!(s.stripline_b_er(LayerId(1), 4).map(|v| v.0), Some(1_017_500));
        assert!(s.stripline_b_er(LayerId(0), 4).is_none());
        assert!(Stackup::from_layers(vec![35_000; 4], vec![diel(0.2, 4500)], None).is_none());
        assert!(Stackup::from_layers(vec![35_000; 2], vec![diel(0.2, 900)], None).is_none());
    }

    #[test]
    fn impedance_width_follows_the_real_dielectric() {
        let width = |outer: f64| {
            let mut b = BoardBuilder::new(mm_to_nm(30.0), mm_to_nm(20.0), 4);
            let n = b.net("CLK");
            let mut board = b.build();
            board.stackup = four(outer);
            board.nets[n.idx()].constraints.impedance_mohm = Some(50_000);
            required_width(&board, n)
        };
        let (thin, thick) = (width(0.1), width(0.2));
        assert!(thick > thin, "thicker dielectric needs a wider trace: {thin} vs {thick}");
        let hj = microstrip_width_hj(50.0, 200_000, 35_000, 4.5).expect("w");
        assert!(thick >= hj);
    }

    #[test]
    fn delay_uses_per_layer_er() {
        let mut b = BoardBuilder::new(mm_to_nm(30.0), mm_to_nm(20.0), 4);
        let _ = b.net("S");
        let mut board = b.build();
        board.stackup = four(0.2);
        let inner = layer_delay_ps_per_mm(&board, LayerId(1), 150_000);
        // Stripline delay = √εr / c with the weighted εr of the two neighbours.
        let er: f64 = (0.2 * 4.5 + 0.8 * 4.0) / 1.0;
        assert!((inner - er.sqrt() / C_MM_PER_PS).abs() < 1e-9, "{inner}");
        let outer = layer_delay_ps_per_mm(&board, LayerId(0), 150_000);
        assert!(outer < inner);
    }
}
