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

/// Natural logarithm for `x > 0` (`-∞` at 0, NaN below), IEEE-only: `x = m·2^k` with
/// `m ∈ [√½, √2)` taken from the bit pattern, then `ln m = 2·atanh(s)`, `s = (m−1)/(m+1)`,
/// summed to 24 odd terms (|s| ≤ 0.172, so the tail is far below f64 precision).
#[must_use]
pub fn ln(x: f64) -> f64 {
    if x.is_nan() || x < 0.0 {
        return f64::NAN;
    }
    if x == 0.0 {
        return f64::NEG_INFINITY;
    }
    if x.is_infinite() {
        return x;
    }
    // Normalise subnormals first.
    let (mut x, mut k) = (x, 0i32);
    if x < f64::MIN_POSITIVE {
        x *= 2f64.powi(64);
        k -= 64;
    }
    let bits = x.to_bits();
    #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
    let e = ((bits >> 52) & 0x7ff) as i32 - 1023;
    let mut m = f64::from_bits((bits & !(0x7ff << 52)) | (1023u64 << 52)); // m ∈ [1, 2)
    k += e;
    if m > std::f64::consts::SQRT_2 {
        m /= 2.0;
        k += 1;
    }
    let s = (m - 1.0) / (m + 1.0);
    let s2 = s * s;
    let mut term = s;
    let mut sum = 0.0;
    let mut n = 1.0;
    for _ in 0..24 {
        sum += term / n;
        term *= s2;
        n += 2.0;
    }
    f64::from(k) * std::f64::consts::LN_2 + 2.0 * sum
}

/// `x^y` for `x > 0` via [`exp`]/[`ln`] (IEEE-only).
#[must_use]
pub fn pow(x: f64, y: f64) -> f64 {
    exp(y * ln(x))
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
    fn ln_matches_std_and_inverts_exp() {
        for &x in &[1e-9, 0.001, 0.5, 1.0, 2.0, std::f64::consts::E, 10.0, 7.565, 1e6, 1e12] {
            let (a, b) = (ln(x), x.ln());
            assert!((a - b).abs() <= 1e-13 * b.abs().max(1.0), "x={x} {a} vs {b}");
            assert!((exp(ln(x)) - x).abs() <= 1e-12 * x, "x={x}");
        }
        assert!(ln(0.0).is_infinite() && ln(-1.0).is_nan());
    }

    #[test]
    fn pow_matches_std() {
        assert!((pow(7.565, 1.0 / 0.725) - 7.565f64.powf(1.0 / 0.725)).abs() < 1e-11);
        assert!((pow(10.0, 0.44) - 10f64.powf(0.44)).abs() < 1e-12);
    }

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
