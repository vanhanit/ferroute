// SPDX-License-Identifier: AGPL-3.0-only
//! Narrow-strip and offset (asymmetric) stripline.

use crate::constraints::stripline_z0;
use ferroute_geom::Coord;
use ferroute_geom::detmath::ln;

#[allow(clippy::cast_precision_loss)]
fn f(v: Coord) -> f64 {
    v as f64
}

/// Symmetric stripline impedance (Ω) for a **narrow** strip, `w/(b−t) < 0.35`: the round-wire
/// equivalent of Cohn/Wheeler as given in Wadell, *Transmission Line Design Handbook*
/// (1991) §4.3: `Z0 = 60/√εr · ln(4b/(π·d0))`,
/// `d0 = (w/2)·[1 + (t/πw)(1 + ln(4πw/t) + 0.551(t/w)²)]` (`d0 = w/2` at `t = 0`).
/// Joins the wide-strip formula of [`stripline_z0`] at `w/(b−t) = 0.35` (< 1 % step, tested).
/// The round-wire equivalence loses monotonicity once the strip is thinner than it is thick,
/// so `None` is returned for `w < t` as well as for `t/b ≥ 0.25` or invalid input.
#[must_use]
pub fn stripline_z0_narrow(w: Coord, b: Coord, t: Coord, er: f64) -> Option<f64> {
    if w <= 0 || b <= 0 || t < 0 || !(er.is_finite() && er >= 1.0) || 4 * t >= b {
        return None;
    }
    if w < t {
        return None;
    }
    let (wf, bf, tf) = (f(w), f(b), f(t));
    let d0 = if t == 0 {
        wf / 2.0
    } else {
        let r = tf / wf;
        wf / 2.0
            * (1.0
                + r / std::f64::consts::PI
                    * (1.0 + ln(4.0 * std::f64::consts::PI * wf / tf) + 0.551 * r * r))
    };
    let arg = 4.0 * bf / (std::f64::consts::PI * d0);
    (arg > 1.0).then(|| 60.0 / er.sqrt() * ln(arg))
}

/// Impedance (Ω) of an **offset** stripline: strip `w` × `t` with dielectric thicknesses
/// `h_a` and `h_b` between its two faces and the two reference planes. Cohn/Wadell
/// capacitance-additivity form: each side contributes half the capacitance of a centred
/// stripline whose plane spacing is `2h + t`, so
/// `Z0 = 2·Za·Zb/(Za + Zb)` with `Za = Z0sym(w, 2h_a + t)`, `Zb = Z0sym(w, 2h_b + t)`.
/// Exact in the symmetric case (`h_a = h_b`) and for wide strips; an approximation (a few
/// per cent, not validated against a field solver here) for strong offsets. `None` outside
/// the validity of [`stripline_z0`] for either half.
#[must_use]
pub fn offset_stripline_z0(w: Coord, h_a: Coord, h_b: Coord, t: Coord, er: f64) -> Option<f64> {
    let za = stripline_z0(w, 2 * h_a + t, t, er)?;
    let zb = stripline_z0(w, 2 * h_b + t, t, er)?;
    Some(2.0 * za * zb / (za + zb))
}
