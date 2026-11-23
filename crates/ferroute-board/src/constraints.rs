// SPDX-License-Identifier: AGPL-3.0-only
//! Per-net electrical constraints and the geometry they imply.
//!
//! Constraints are stored in exact integer units (mA, Hz, mΩ, nm) so the model stays
//! comparable and deterministic. Derived widths use only IEEE-portable math
//! (`ferroute_geom::detmath`) and are rounded **up** to a 10 µm fabrication step, so a
//! derived width is never narrower than the formula requires (INV-RULE-1).

use crate::{Board, NetClass, NetId};
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
    /// Plane-to-plane spacing `b` of an inner (stripline) layer (nm), when the stack-up
    /// has inner dielectrics; `None` disables the stripline width model.
    #[serde(default)]
    pub inner_dielectric: Option<Coord>,
}

impl Default for Stackup {
    /// 1.6 mm FR-4, 35 µm (1 oz) copper, 2 layers (reference plane on the far side).
    fn default() -> Self {
        Self {
            copper_thickness: vec![35_000, 35_000],
            outer_dielectric: 1_510_000,
            epsilon_r_milli: 4300,
            inner_dielectric: None,
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
    let t_outer = s.copper_thickness.first().copied().unwrap_or(35_000);
    let er = f64::from(s.epsilon_r_milli) / 1000.0;
    let z_ohm = c.impedance_mohm.map(|z| f64::from(z) / 1000.0);
    let micro_w =
        z_ohm.and_then(|z| microstrip_width_hj(z, s.outer_dielectric, t_outer, er)).unwrap_or(0);
    // Inner layers: stripline model when the stack-up declares an inner dielectric.
    let t_inner = s.copper_thickness.get(1).copied().unwrap_or(t_outer);
    let strip_w = match (z_ohm, s.inner_dielectric) {
        (Some(z), Some(b)) if board.layer_count() > 2 => {
            stripline_width_cohn(z, b, t_inner, er).unwrap_or(0)
        }
        _ => 0,
    };
    class_w.max(current_w).max(micro_w).max(strip_w)
}

/// `true` when the net has an impedance target that no trace width can reach with this
/// stack-up on its outer layers, or on inner layers when the board has them and declares
/// an inner dielectric. Such a rule cannot be met by width and must be reported.
#[must_use]
pub fn impedance_unattainable(board: &Board, net: NetId) -> bool {
    let Some(z) = board.nets.get(net.idx()).and_then(|n| n.constraints.impedance_mohm) else {
        return false;
    };
    let s = &board.stackup;
    let (z, er) = (f64::from(z) / 1000.0, f64::from(s.epsilon_r_milli) / 1000.0);
    let t_outer = s.copper_thickness.first().copied().unwrap_or(35_000);
    let outer = microstrip_width_hj(z, s.outer_dielectric, t_outer, er).is_none();
    let inner = match s.inner_dielectric {
        Some(b) if board.layer_count() > 2 => {
            let t = s.copper_thickness.get(1).copied().unwrap_or(t_outer);
            stripline_width_cohn(z, b, t, er).is_none()
        }
        _ => false,
    };
    outer || inner
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
}
