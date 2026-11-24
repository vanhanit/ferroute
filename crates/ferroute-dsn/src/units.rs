// SPDX-License-Identifier: AGPL-3.0-only
//! Exact integer unit conversion (no floating point).

use crate::DsnError;
use ferroute_geom::Coord;

/// Nanometres per Specctra length unit name, `None` when unknown.
pub(crate) fn unit_nm(name: &str) -> Option<i128> {
    Some(match name {
        "nm" => 1,
        "um" => 1_000,
        "mm" => 1_000_000,
        "cm" => 10_000_000,
        "inch" => 25_400_000,
        "mil" => 25_400,
        _ => return None,
    })
}

/// Converts the decimal literal `tok` (e.g. `-12.375`) times `num / den` to an integer,
/// rounding half away from zero. Pure `i128` arithmetic.
pub(crate) fn scale_decimal(tok: &str, num: i128, den: i128) -> Result<Coord, DsnError> {
    let bad = || DsnError::Session(format!("bad number {tok:?}"));
    let (neg, body) = match tok.strip_prefix('-') {
        Some(r) => (true, r),
        None => (false, tok.strip_prefix('+').unwrap_or(tok)),
    };
    let (int, frac) = body.split_once('.').unwrap_or((body, ""));
    if (int.is_empty() && frac.is_empty())
        || frac.len() > 18
        || !int.bytes().chain(frac.bytes()).all(|b| b.is_ascii_digit())
    {
        return Err(bad());
    }
    let digits: i128 = format!("{int}{frac}").parse().map_err(|_| bad())?;
    let n = digits.checked_mul(num).ok_or_else(bad)?;
    let pow = 10_i128.pow(u32::try_from(frac.len()).map_err(|_| bad())?);
    let d = den.checked_mul(pow).ok_or_else(bad)?;
    if d <= 0 {
        return Err(bad());
    }
    let q = (2 * n + d) / (2 * d);
    Coord::try_from(if neg { -q } else { q }).map_err(|_| bad())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn integer_and_decimal_scaling_is_exact() {
        // um 10: one unit = 100 nm.
        assert_eq!(scale_decimal("2500", 1000, 10).unwrap(), 250_000);
        assert_eq!(scale_decimal("-1234.5", 1000, 10).unwrap(), -123_450);
        assert_eq!(scale_decimal("0.005", 1000, 1).unwrap(), 5);
        // Half rounds away from zero.
        assert_eq!(scale_decimal("0.5", 1, 1).unwrap(), 1);
        assert_eq!(scale_decimal("-0.5", 1, 1).unwrap(), -1);
        assert_eq!(scale_decimal("100", 25_400, 1).unwrap(), 2_540_000);
    }

    #[test]
    fn rejects_garbage() {
        for t in ["", "-", "1e3", "1.2.3", "abc", "."] {
            assert!(scale_decimal(t, 1, 1).is_err(), "{t}");
        }
    }
}
