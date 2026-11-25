// SPDX-License-Identifier: AGPL-3.0-only
//! Edge-coupled microstrip (static even/odd modes) and the solder-mask estimate.

use crate::constraints::{ETA0, coth, eps_eff_u, microstrip_hj, z01};
use ferroute_geom::Coord;
use ferroute_geom::detmath::{exp, ln, pow};

#[allow(clippy::cast_precision_loss)]
fn f(v: Coord) -> f64 {
    v as f64
}

/// Which edge-coupled microstrip model derives differential impedance (`coupled_model` in a
/// `[[pair]]` rule). Default [`CoupledModel::Ipc2141a`], so existing outputs do not change.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum CoupledModel {
    /// IPC-2141A approximation (default).
    #[default]
    Ipc2141a,
    /// Kirschning–Jansen static even/odd modes: unverified, see CLAUDE.md #20b.
    Kj,
}

/// Static even/odd-mode parameters of an edge-coupled microstrip pair.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CoupledMicrostrip {
    /// Even-mode impedance (Ω).
    pub z0e: f64,
    /// Odd-mode impedance (Ω).
    pub z0o: f64,
    /// Even-mode effective permittivity.
    pub eeff_e: f64,
    /// Odd-mode effective permittivity.
    pub eeff_o: f64,
}

impl CoupledMicrostrip {
    /// Differential impedance `2·Z0o` (Ω).
    #[must_use]
    pub fn zdiff(&self) -> f64 {
        2.0 * self.z0o
    }
}

/// Static even/odd-mode impedances and effective permittivities of two identical coupled
/// microstrips (width `w`, edge gap `s`, dielectric height `h`, copper thickness `t`,
/// permittivity `er`).
///
/// **Unverified, see CLAUDE.md #20b** (opt-in via [`CoupledModel::Kj`]).
///
/// Equations: Kirschning & Jansen, "Accurate Wide-Range Design Equations for the
/// Frequency-Dependent Characteristic of Parallel Coupled Microstrip Lines", IEEE Trans.
/// MTT-32(1), 1984, static (`f → 0`) part, which restates Hammerstad & Jensen (1980):
/// with `u = w/h`, `g = s/h`, `v = u(20+g²)/(10+g²) + g·e^−g`,
/// `εe = (εr+1)/2 + (εr−1)/2·(1 + 10/v)^(−ae·be)`,
/// `εo = ε(u) + [(εr+1)/2 + ao − ε(u)]·exp(−co·g^do)`, and the `Q1…Q10` impedance
/// functions, `Z0e = ZL√(ε(u)/εe)/(1 − ZL√ε(u)·Q4/η0)`,
/// `Z0o = ZL√(ε(u)/εo)/(1 − ZL√ε(u)·Q10/η0)` with `ZL`, `ε(u)` the single-line
/// Hammerstad–Jensen values. Published accuracy ≲ 1 % for `0.1 ≤ u, g ≤ 10`, `1 ≤ εr ≤ 18`;
/// the formulas stay smooth outside, which the width/gap solvers rely on, but accuracy
/// degrades there. Thickness: the Hammerstad–Bekkadal shift `Δ` of
/// [`microstrip_hj`] widens each trace to `w + Δ` and narrows the gap to `s − Δ` (an
/// engineering approximation; exact at `t = 0`). `None` for invalid input or `εr ∉ [1, 20]`.
#[must_use]
pub fn coupled_microstrip_kj(
    w: Coord,
    s: Coord,
    h: Coord,
    t: Coord,
    er: f64,
) -> Option<CoupledMicrostrip> {
    if w <= 0 || s <= 0 || h <= 0 || t < 0 || !(er.is_finite() && (1.0..=20.0).contains(&er)) {
        return None;
    }
    let hf = f(h);
    let (mut u, mut g) = (f(w) / hf, f(s) / hf);
    if t > 0 {
        // Same effective-width shift as `microstrip_hj` (Δur), applied to both edges.
        let th = f(t) / hf;
        let c = coth((6.517 * u).sqrt());
        let d1 = th / std::f64::consts::PI * ln(1.0 + 4.0 * std::f64::consts::E / (th * c * c));
        let x = (er - 1.0).sqrt();
        let sech = 2.0 / (exp(x) + exp(-x));
        let dr = 0.5 * (1.0 + sech) * d1;
        u += dr;
        g = (g - dr).max(0.01);
    }
    let eu = eps_eff_u(u, er);
    let zl = z01(u) / eu.sqrt();

    // Even mode.
    let g2 = g * g;
    let v = u * (20.0 + g2) / (10.0 + g2) + g * exp(-g);
    let v4 = v * v * v * v;
    let ae = 1.0
        + ln((v4 + (v / 52.0) * (v / 52.0)) / (v4 + 0.432)) / 49.0
        + ln(1.0 + pow(v / 18.1, 3.0)) / 18.7;
    let be = 0.564 * pow((er - 0.9) / (er + 3.0), 0.053);
    let ee = (er + 1.0) / 2.0 + (er - 1.0) / 2.0 * pow(1.0 + 10.0 / v, -ae * be);

    // Odd mode.
    let ao = 0.7287 * (eu - (er + 1.0) / 2.0) * (1.0 - exp(-0.179 * u));
    let bo = 0.747 * er / (0.15 + er);
    let co = bo - (bo - 0.207) * exp(-0.414 * u);
    let d_o = 0.593 + 0.694 * exp(-0.562 * u);
    let eo = ((er + 1.0) / 2.0 + ao - eu) * exp(-co * pow(g, d_o)) + eu;

    // Impedance functions.
    let q1 = 0.8695 * pow(u, 0.194);
    let q2 = 1.0 + 0.7519 * g + 0.189 * pow(g, 2.31);
    let q3 = 0.1975
        + pow(16.6 + pow(8.4 / g, 6.0), -0.387)
        + ln(pow(g, 10.0) / (1.0 + pow(g / 3.4, 10.0))) / 241.0;
    let q4 = 2.0 * q1 / q2 / (exp(-g) * pow(u, q3) + (2.0 - exp(-g)) * pow(u, -q3));
    let q5 = 1.794 + 1.14 * ln(1.0 + 0.638 / (g + 0.517 * pow(g, 2.43)));
    let q6 = 0.2305
        + ln(pow(g, 10.0) / (1.0 + pow(g / 5.8, 10.0))) / 281.3
        + ln(1.0 + 0.598 * pow(g, 1.154)) / 5.1;
    let q7 = (10.0 + 190.0 * g2) / (1.0 + 82.3 * g2 * g);
    let q8 = exp(-6.5 - 0.95 * ln(g) - pow(g / 0.15, 5.0));
    let q9 = ln(q7) * (q8 + 1.0 / 16.5);
    let q10 = (q2 * q4 - q5 * exp(ln(u) * q6 * pow(u, -q9))) / q2;

    let z0e = zl * (eu / ee).sqrt() / (1.0 - zl * eu.sqrt() * q4 / ETA0);
    let z0o = zl * (eu / eo).sqrt() / (1.0 - zl * eu.sqrt() * q10 / ETA0);
    (z0e.is_finite() && z0o.is_finite() && z0e > 0.0 && z0o > 0.0).then_some(CoupledMicrostrip {
        z0e,
        z0o,
        eeff_e: ee,
        eeff_o: eo,
    })
}

/// **Reported approximation** of a solder-mask-coated microstrip: impedance (Ω) of a single
/// trace under a mask of thickness `hm` and permittivity `er_mask`. The mask replaces part
/// of the air above the trace, raising `εeff` by
/// `Δε = (εm − 1)·qair·(1 − exp(−2·hm/h))`, `qair = (εr − εeff)/(εr − 1)` (the fraction of
/// the field in air), and `Z = Z0·√(εeff/(εeff + Δε))`. This is a heuristic calibrated to
/// the commonly observed 1–3 Ω drop for 20–30 µm masks; it is **not** a published closed
/// form, is never used for width synthesis, and only serves to report the expected shift.
#[must_use]
pub fn microstrip_mask_z0(
    w: Coord,
    h: Coord,
    t: Coord,
    er: f64,
    hm: Coord,
    er_mask: f64,
) -> Option<f64> {
    let (z0, eeff) = microstrip_hj(w, h, t, er)?;
    if hm < 0 || !(er_mask.is_finite() && er_mask >= 1.0) || er <= 1.0 {
        return None;
    }
    let qair = ((er - eeff) / (er - 1.0)).clamp(0.0, 1.0);
    let de = (er_mask - 1.0) * qair * (1.0 - exp(-2.0 * f(hm) / f(h)));
    Some(z0 * (eeff / (eeff + de)).sqrt())
}
