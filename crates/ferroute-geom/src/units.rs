// SPDX-License-Identifier: AGPL-3.0-only
//! Exact conversion between KiCad's millimetre decimal strings and nanometres.
//!
//! KiCad writes coordinates as decimal millimetres with at most 6 fractional digits
//! (1 nm resolution). Parsing via `f64` would introduce representation error
//! (`0.1` is not representable), so we parse the decimal digits directly.

use crate::Coord;

/// Nanometres per millimetre.
pub const NM_PER_MM: Coord = 1_000_000;

/// Error returned by [`parse_mm`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParseMmError(pub String);

impl std::fmt::Display for ParseMmError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "invalid millimetre value {:?}", self.0)
    }
}

impl std::error::Error for ParseMmError {}

/// Parses a KiCad decimal millimetre string (`"-12.7"`, `"0.000001"`, `"3"`, `"1e-3"`)
/// into nanometres. Digits beyond the 6th decimal are rounded half away from zero.
/// Exponent notation (rare, but legal in KiCad's lexer) is supported.
///
/// # Errors
/// Returns [`ParseMmError`] for malformed input or values outside ±9.2e9 mm.
pub fn parse_mm(s: &str) -> Result<Coord, ParseMmError> {
    let err = || ParseMmError(s.to_owned());
    let t = s.trim();
    let (neg, body) = match t.as_bytes().first() {
        Some(b'-') => (true, &t[1..]),
        Some(b'+') => (false, &t[1..]),
        _ => (false, t),
    };
    let (mantissa, exp) = match body.find(['e', 'E']) {
        Some(i) => (&body[..i], body[i + 1..].parse::<i32>().map_err(|_| err())?),
        None => (body, 0),
    };
    let (int_part, frac_part) = match mantissa.find('.') {
        Some(i) => (&mantissa[..i], &mantissa[i + 1..]),
        None => (mantissa, ""),
    };
    if (int_part.is_empty() && frac_part.is_empty())
        || !int_part.bytes().all(|b| b.is_ascii_digit())
        || !frac_part.bytes().all(|b| b.is_ascii_digit())
    {
        return Err(err());
    }
    // value = digits * 10^(exp - frac_len) mm = digits * 10^(exp - frac_len + 6) nm
    let digits: String = format!("{int_part}{frac_part}");
    let digits = digits.trim_start_matches('0');
    #[allow(clippy::cast_possible_wrap, clippy::cast_possible_truncation)]
    let shift = exp - frac_part.len() as i32 + 6;
    let mut value: i128 = 0;
    let mut round_up = false;
    if shift >= 0 {
        for b in digits.bytes() {
            value = value
                .checked_mul(10)
                .and_then(|v| v.checked_add(i128::from(b - b'0')))
                .ok_or_else(err)?;
        }
        for _ in 0..shift {
            value = value.checked_mul(10).ok_or_else(err)?;
        }
    } else {
        #[allow(clippy::cast_sign_loss)]
        let drop = (-shift) as usize;
        let keep = digits.len().saturating_sub(drop);
        for b in digits[..keep].bytes() {
            value = value
                .checked_mul(10)
                .and_then(|v| v.checked_add(i128::from(b - b'0')))
                .ok_or_else(err)?;
        }
        // First dropped digit decides rounding (half away from zero).
        let first_dropped = if digits.len() >= drop { digits.as_bytes().get(keep) } else { None };
        round_up = first_dropped.is_some_and(|&d| d >= b'5');
    }
    if round_up {
        value += 1;
    }
    if neg {
        value = -value;
    }
    Coord::try_from(value).map_err(|_| err())
}

/// Formats nanometres as KiCad does: decimal millimetres, up to 6 fractional digits,
/// trailing zeros (and a trailing '.') removed, `-0` normalised to `0`.
#[must_use]
pub fn format_mm(nm: Coord) -> String {
    let neg = nm < 0;
    let a = nm.unsigned_abs();
    let int = a / NM_PER_MM.unsigned_abs();
    let frac = a % NM_PER_MM.unsigned_abs();
    let mut s = if frac == 0 {
        format!("{int}")
    } else {
        let f = format!("{frac:06}");
        format!("{int}.{}", f.trim_end_matches('0'))
    };
    if neg && a != 0 {
        s.insert(0, '-');
    }
    s
}

/// Converts nanometres to millimetres as `f64` (display only).
#[must_use]
pub fn nm_to_mm(nm: Coord) -> f64 {
    #[allow(clippy::cast_precision_loss)]
    let v = nm as f64;
    v / 1e6
}

/// Converts millimetres to nanometres, rounding half away from zero.
#[must_use]
pub fn mm_to_nm(mm: f64) -> Coord {
    #[allow(clippy::cast_possible_truncation)]
    let v = (mm * 1e6).round() as Coord;
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_common_values() {
        assert_eq!(parse_mm("0"), Ok(0));
        assert_eq!(parse_mm("1.27"), Ok(1_270_000));
        assert_eq!(parse_mm("-12.7"), Ok(-12_700_000));
        assert_eq!(parse_mm("0.000001"), Ok(1));
        assert_eq!(parse_mm("0.0000005"), Ok(1));
        assert_eq!(parse_mm("-0.0000005"), Ok(-1));
        assert_eq!(parse_mm("0.0000004"), Ok(0));
        assert_eq!(parse_mm("100"), Ok(100_000_000));
        assert_eq!(parse_mm(".5"), Ok(500_000));
        assert_eq!(parse_mm("5."), Ok(5_000_000));
        assert_eq!(parse_mm("1e-3"), Ok(1_000));
        assert_eq!(parse_mm("2.5E1"), Ok(25_000_000));
        assert_eq!(parse_mm("+3.000"), Ok(3_000_000));
    }

    #[test]
    fn rejects_garbage() {
        for s in ["", "-", ".", "1.2.3", "abc", "1,5", "--1", "1e", "0x10"] {
            assert!(parse_mm(s).is_err(), "{s:?} should fail");
        }
    }

    #[test]
    fn formats_like_kicad() {
        assert_eq!(format_mm(0), "0");
        assert_eq!(format_mm(1_270_000), "1.27");
        assert_eq!(format_mm(-12_700_000), "-12.7");
        assert_eq!(format_mm(1), "0.000001");
        assert_eq!(format_mm(-1), "-0.000001");
        assert_eq!(format_mm(100_000_000), "100");
    }

    proptest::proptest! {
        #[test]
        fn roundtrip(nm in -(1i64 << 40)..(1i64 << 40)) {
            proptest::prop_assert_eq!(parse_mm(&format_mm(nm)), Ok(nm));
        }
    }
}
