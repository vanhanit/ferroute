// SPDX-License-Identifier: AGPL-3.0-only
//! Length arguments with units: `2mm`, `0.45`, `500um`, `10mil`, `0.1in`, `1500nm`.

use ferroute_geom::Coord;
use ferroute_geom::units::parse_mm;

/// Parses a length (default unit: millimetres) into nanometres.
///
/// # Errors
/// Malformed numbers or unknown units.
pub fn parse_length(s: &str) -> Result<Coord, String> {
    let t = s.trim();
    let split = t.find(|c: char| c.is_ascii_alphabetic()).unwrap_or(t.len());
    let (num, unit) = t.split_at(split);
    let mm = parse_mm(num).map_err(|e| e.to_string())?;
    // `mm` holds the value as if it were millimetres (in nm).
    let v = match unit.to_ascii_lowercase().as_str() {
        "" | "mm" => mm,
        "um" | "µm" => mm / 1_000,
        "nm" => mm / 1_000_000,
        "mil" | "mils" | "thou" => mm * 254 / 10_000,
        "in" | "inch" => mm * 254 / 10,
        other => return Err(format!("unknown unit {other:?} (use mm, um, nm, mil, in)")),
    };
    Ok(v)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn units() {
        assert_eq!(parse_length("2mm"), Ok(2_000_000));
        assert_eq!(parse_length("0.45"), Ok(450_000));
        assert_eq!(parse_length("500um"), Ok(500_000));
        assert_eq!(parse_length("10mil"), Ok(254_000));
        assert_eq!(parse_length("0.1in"), Ok(2_540_000));
        assert_eq!(parse_length("1500nm"), Ok(1_500));
        assert!(parse_length("3furlong").is_err());
    }
}
