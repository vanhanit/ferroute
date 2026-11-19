// SPDX-License-Identifier: AGPL-3.0-only
//! Platform-independent transcendental functions for solver *decisions*.
//!
//! INV-DET-3: solver decisions depend only on IEEE-754 basic operations (`+ − × ÷ √`),
//! which are correctly rounded on every platform, never on the host `libm` (whose
//! `exp`/`pow`/`hypot` may differ in the last ulp between glibc, musl, macOS and MSVC).
//! The functions here are accurate to ~1e-15 relative error — plenty for annealing
//! acceptance tests — and bit-identical everywhere.

/// `e^x` for `x ∈ [-700, 700]` (clamped outside), using range reduction
/// `e^x = 2^k · e^r` with `|r| ≤ ln2/2` and a degree-13 Taylor polynomial.
#[must_use]
pub fn exp(x: f64) -> f64 {
    if x.is_nan() {
        return f64::NAN;
    }
    let x = x.clamp(-700.0, 700.0);
    const LN2: f64 = std::f64::consts::LN_2;
    let k = (x / LN2).round();
    let r = x - k * LN2;
    // Horner evaluation of Σ r^n / n!, n = 0..=13.
    let mut p = 1.0 / 6_227_020_800.0; // 1/13!
    let inv_fact = [
        1.0 / 479_001_600.0, // 1/12!
        1.0 / 39_916_800.0,
        1.0 / 3_628_800.0,
        1.0 / 362_880.0,
        1.0 / 40_320.0,
        1.0 / 5_040.0,
        1.0 / 720.0,
        1.0 / 120.0,
        1.0 / 24.0,
        1.0 / 6.0,
        0.5,
        1.0,
        1.0,
    ];
    for c in inv_fact {
        p = p * r + c;
    }
    // Multiply by 2^k exactly via repeated scaling (k is an integer in [-1010, 1010]).
    #[allow(clippy::cast_possible_truncation)]
    let mut k = k as i32;
    let mut v = p;
    while k > 0 {
        let s = k.min(512);
        v *= f64::from(2.0f32).powi(s);
        k -= s;
    }
    while k < 0 {
        let s = (-k).min(512);
        v /= f64::from(2.0f32).powi(s);
        k += s;
    }
    v
}

/// `sqrt(dx² + dy²)` with only correctly-rounded IEEE operations.
#[must_use]
pub fn hypot(dx: f64, dy: f64) -> f64 {
    (dx * dx + dy * dy).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exp_matches_std_closely() {
        let mut x = -50.0;
        while x <= 50.0 {
            let (a, b) = (exp(x), x.exp());
            assert!(((a - b) / b).abs() < 1e-13, "x={x} {a} vs {b}");
            x += 0.37;
        }
        assert_eq!(exp(0.0), 1.0);
        assert!(exp(-1000.0) > 0.0);
    }
}
