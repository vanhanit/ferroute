// SPDX-License-Identifier: AGPL-3.0-only
//! Coplanar waveguide, with and without a lower ground plane.

use crate::constraints::{bisect, k_ratio, round_up_nm, tanh};
use ferroute_geom::Coord;
use ferroute_geom::detmath::ln;

#[allow(clippy::cast_precision_loss)]
fn f(v: Coord) -> f64 {
    v as f64
}

/// `K(k)/K(k')` for modulus `k ∈ (0, 1)`.
fn kk(k: f64) -> f64 {
    1.0 / k_ratio(k, (1.0 - k * k).sqrt())
}

/// Impedance (Ω) and effective permittivity of a coplanar waveguide: centre conductor `w`,
/// gap `gap` to the coplanar grounds, copper thickness `t`, permittivity `er`; `h` is the
/// substrate height to a lower ground plane (`Some`, conductor-backed CPW) or `None`
/// (CPW on an infinite substrate).
///
/// Conformal-mapping solution (Wadell, *Transmission Line Design Handbook*, 1991, §3.4;
/// Ghione & Naldi 1984), `a = w/2`, `b = a + gap`, `k = a/b`,
/// `k3 = tanh(πa/2h)/tanh(πb/2h)`, `q = K(k3)/K(k3')·K(k')/K(k)`:
/// `εeff = (1 + εr·q)/(1 + q)`, `Z0 = 60π/√εeff / (K(k)/K(k') + K(k3)/K(k3'))`
/// (without a lower plane `εeff = (εr+1)/2`, `Z0 = 30π/√εeff·K(k')/K(k)`).
/// Thickness: the strip edge shifts by `Δ = (1.25t/π)(1 + ln(4πw/t))`
/// (Gupta, Garg & Bahl, *Microstrip Lines and Slotlines*, 1996), `w → w+Δ`, `gap → gap−Δ`.
/// `None` for invalid input or a gap eaten by the thickness shift.
#[must_use]
pub fn cpw_z0(w: Coord, gap: Coord, h: Option<Coord>, t: Coord, er: f64) -> Option<(f64, f64)> {
    if w <= 0 || gap <= 0 || t < 0 || h.is_some_and(|h| h <= 0) || !(er.is_finite() && er >= 1.0) {
        return None;
    }
    let (mut wf, mut gf) = (f(w), f(gap));
    if t > 0 {
        let d =
            1.25 * f(t) / std::f64::consts::PI * (1.0 + ln(4.0 * std::f64::consts::PI * wf / f(t)));
        wf += d;
        gf -= d;
        if gf <= 0.0 {
            return None;
        }
    }
    let (a, b) = (wf / 2.0, wf / 2.0 + gf);
    let k = a / b;
    let pi = std::f64::consts::PI;
    let kr = kk(k); // K(k)/K(k')
    let Some(h) = h else {
        let ee = (er + 1.0) / 2.0;
        return Some((30.0 * pi / ee.sqrt() / kr, ee));
    };
    let hf = f(h);
    let k3 = tanh(pi * a / (2.0 * hf)) / tanh(pi * b / (2.0 * hf));
    let k3r = kk(k3); // K(k3)/K(k3')
    let q = k3r / kr;
    let ee = (1.0 + er * q) / (1.0 + q);
    Some((60.0 * pi / ee.sqrt() / (kr + k3r), ee))
}

/// Centre-conductor width (nm, rounded **up** to the width step) giving `z0_ohm` for a CPW
/// with fixed `gap` (see [`cpw_z0`]); integer bisection on the monotone model
/// (wider centre conductor → lower Z). `None` if unreachable in `1 µm … 100·gap`.
#[must_use]
pub fn cpw_width(z0_ohm: f64, gap: Coord, h: Option<Coord>, t: Coord, er: f64) -> Option<Coord> {
    if !(z0_ohm.is_finite() && z0_ohm > 0.0) || gap <= 0 {
        return None;
    }
    bisect(1_000, 100 * gap, |w| cpw_z0(w, gap, h, t, er).is_some_and(|(z, _)| z <= z0_ohm))
        .filter(|&w| w > 1_000)
        .map(round_up_nm)
}
