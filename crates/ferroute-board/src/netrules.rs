// SPDX-License-Identifier: AGPL-3.0-only
//! User-facing per-net electrical rules (`--rules FILE`), in engineering units.
//!
//! A [`RuleSet`] is a list of [`NetRule`]s, each selecting nets by name and/or net-class
//! glob. Rules apply in order; a later rule overrides only the fields it sets. Values are
//! converted once, at this edge, into the exact integer units of
//! [`NetConstraints`](crate::constraints::NetConstraints).

use crate::constraints::NetConstraints;
use crate::{Board, glob};
use ferroute_geom::Coord;
use ferroute_geom::units::mm_to_nm;
use serde::{Deserialize, Serialize};

/// Stack-up overrides (all optional).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StackupSpec {
    /// Copper thickness of every copper layer (µm).
    pub copper_um: Option<f64>,
    /// Outer-layer dielectric height to the nearest reference plane (mm).
    pub outer_dielectric_mm: Option<f64>,
    /// Relative permittivity.
    pub epsilon_r: Option<f64>,
}

/// One rule: selector globs plus the constraints it sets.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NetRule {
    /// Net-name globs (`*`, `?`). Empty with empty `classes` = matches nothing.
    #[serde(default)]
    pub nets: Vec<String>,
    /// Net-class-name globs.
    #[serde(default)]
    pub classes: Vec<String>,
    pub max_current_a: Option<f64>,
    pub frequency_mhz: Option<f64>,
    pub impedance_ohm: Option<f64>,
    pub length_min_mm: Option<f64>,
    pub length_max_mm: Option<f64>,
    pub match_group: Option<String>,
    pub skew_max_mm: Option<f64>,
    pub max_vias: Option<u32>,
}

/// A complete rules file.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RuleSet {
    #[serde(default)]
    pub stackup: Option<StackupSpec>,
    #[serde(default, rename = "net")]
    pub rules: Vec<NetRule>,
}

/// Rejected rule value.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum RuleError {
    #[error("rule {index}: {field} must be a finite positive number")]
    NotPositive { index: usize, field: &'static str },
    #[error("rule {index}: length_min_mm exceeds length_max_mm")]
    LengthRange { index: usize },
    #[error("stackup: {field} must be a finite positive number")]
    Stackup { field: &'static str },
}

fn positive(v: Option<f64>, index: usize, field: &'static str) -> Result<(), RuleError> {
    match v {
        Some(x) if !(x.is_finite() && x > 0.0) => Err(RuleError::NotPositive { index, field }),
        _ => Ok(()),
    }
}

#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn scaled<T: TryFrom<u64>>(v: f64, scale: f64) -> Option<T> {
    T::try_from((v * scale).round() as u64).ok()
}

impl NetRule {
    fn validate(&self, index: usize) -> Result<(), RuleError> {
        positive(self.max_current_a, index, "max_current_a")?;
        positive(self.frequency_mhz, index, "frequency_mhz")?;
        positive(self.impedance_ohm, index, "impedance_ohm")?;
        positive(self.length_min_mm, index, "length_min_mm")?;
        positive(self.length_max_mm, index, "length_max_mm")?;
        positive(self.skew_max_mm, index, "skew_max_mm")?;
        if let (Some(lo), Some(hi)) = (self.length_min_mm, self.length_max_mm)
            && lo > hi
        {
            return Err(RuleError::LengthRange { index });
        }
        Ok(())
    }

    fn matches(&self, net: &str, class: &str) -> bool {
        self.nets.iter().any(|p| glob(p, net)) || self.classes.iter().any(|p| glob(p, class))
    }

    /// Overlays the fields this rule sets onto `c`.
    fn overlay(&self, c: &mut NetConstraints) {
        if let Some(a) = self.max_current_a {
            c.max_current_ma = scaled(a, 1000.0);
        }
        if let Some(f) = self.frequency_mhz {
            c.frequency_hz = scaled(f, 1e6);
        }
        if let Some(z) = self.impedance_ohm {
            c.impedance_mohm = scaled(z, 1000.0);
        }
        if let Some(l) = self.length_min_mm {
            c.length_min = Some(mm_to_nm(l));
        }
        if let Some(l) = self.length_max_mm {
            c.length_max = Some(mm_to_nm(l));
        }
        if let Some(g) = &self.match_group {
            c.match_group = Some(g.clone());
        }
        if let Some(s) = self.skew_max_mm {
            c.skew_max = Some(mm_to_nm(s));
        }
        if let Some(v) = self.max_vias {
            c.max_vias = Some(v);
        }
    }
}

impl RuleSet {
    /// Checks every value; nothing is applied if any is invalid.
    ///
    /// # Errors
    /// The first invalid value.
    pub fn validate(&self) -> Result<(), RuleError> {
        if let Some(s) = &self.stackup {
            for (v, field) in [
                (s.copper_um, "copper_um"),
                (s.outer_dielectric_mm, "outer_dielectric_mm"),
                (s.epsilon_r, "epsilon_r"),
            ] {
                if v.is_some_and(|x| !(x.is_finite() && x > 0.0)) {
                    return Err(RuleError::Stackup { field });
                }
            }
        }
        self.rules.iter().enumerate().try_for_each(|(i, r)| r.validate(i))
    }

    /// Validates, then applies the stack-up and every rule to `board`. Returns warnings
    /// (e.g. rules that matched no net) — never silently ignored.
    ///
    /// # Errors
    /// See [`RuleSet::validate`].
    pub fn apply(&self, board: &mut Board) -> Result<Vec<String>, RuleError> {
        self.validate()?;
        if let Some(s) = &self.stackup {
            if let Some(t) = s.copper_um {
                let nm: Coord = mm_to_nm(t / 1000.0);
                board.stackup.copper_thickness = vec![nm; usize::from(board.layer_count().max(1))];
            }
            if let Some(h) = s.outer_dielectric_mm {
                board.stackup.outer_dielectric = mm_to_nm(h);
            }
            if let Some(e) = s.epsilon_r {
                board.stackup.epsilon_r_milli = scaled(e, 1000.0).unwrap_or(u32::MAX);
            }
        }
        let mut warnings = Vec::new();
        for (i, rule) in self.rules.iter().enumerate() {
            let mut hits = 0usize;
            for n in 1..board.nets.len() {
                let class = board.rules.classes.get(usize::from(board.nets[n].class));
                let class = class.map_or("Default", |c| c.name.as_str()).to_owned();
                if rule.matches(&board.nets[n].name, &class) {
                    rule.overlay(&mut board.nets[n].constraints);
                    hits += 1;
                }
            }
            if hits == 0 {
                warnings.push(format!("rule {i} matched no net"));
            }
        }
        Ok(warnings)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builder::{BoardBuilder, mm};

    fn board() -> Board {
        let mut b = BoardBuilder::new(mm(30), mm(20), 2);
        for n in ["VBUS", "USB_D+", "USB_D-", "SIG"] {
            b.net(n);
        }
        b.build()
    }

    fn c<'a>(b: &'a Board, n: &str) -> &'a NetConstraints {
        &b.nets[b.net_by_name(n).expect("net").idx()].constraints
    }

    #[test]
    fn rules_select_by_glob_and_convert_units_exactly() {
        let mut b = board();
        let rs = RuleSet {
            stackup: None,
            rules: vec![
                NetRule {
                    nets: vec!["VBUS".into()],
                    max_current_a: Some(1.5),
                    ..NetRule::default()
                },
                NetRule {
                    nets: vec!["USB_D?".into()],
                    frequency_mhz: Some(240.0),
                    impedance_ohm: Some(45.0),
                    match_group: Some("usb".into()),
                    skew_max_mm: Some(0.15),
                    length_max_mm: Some(50.0),
                    max_vias: Some(2),
                    ..NetRule::default()
                },
            ],
        };
        assert!(rs.apply(&mut b).expect("valid").is_empty());
        assert_eq!(c(&b, "VBUS").max_current_ma, Some(1500));
        let d = c(&b, "USB_D+");
        assert_eq!(d.frequency_hz, Some(240_000_000));
        assert_eq!(d.impedance_mohm, Some(45_000));
        assert_eq!(d.skew_max, Some(150_000));
        assert_eq!(d.length_max, Some(mm(50)));
        assert_eq!(d.match_group.as_deref(), Some("usb"));
        assert_eq!(c(&b, "USB_D-"), d);
        assert_eq!(c(&b, "SIG"), &NetConstraints::default());
    }

    #[test]
    fn later_rules_override_only_their_fields() {
        let mut b = board();
        let rs = RuleSet {
            stackup: None,
            rules: vec![
                NetRule {
                    classes: vec!["*".into()],
                    max_vias: Some(4),
                    max_current_a: Some(0.1),
                    ..NetRule::default()
                },
                NetRule { nets: vec!["SIG".into()], max_vias: Some(1), ..NetRule::default() },
            ],
        };
        rs.apply(&mut b).expect("valid");
        assert_eq!(c(&b, "SIG").max_vias, Some(1));
        assert_eq!(c(&b, "SIG").max_current_ma, Some(100));
        assert_eq!(c(&b, "VBUS").max_vias, Some(4));
    }

    #[test]
    fn unmatched_rules_warn_and_bad_values_reject_without_applying() {
        let mut b = board();
        let rs = RuleSet {
            stackup: None,
            rules: vec![NetRule { nets: vec!["NOPE*".into()], ..NetRule::default() }],
        };
        assert_eq!(rs.apply(&mut b).expect("valid"), vec!["rule 0 matched no net".to_owned()]);
        let bad = RuleSet {
            stackup: None,
            rules: vec![
                NetRule { nets: vec!["VBUS".into()], max_vias: Some(1), ..NetRule::default() },
                NetRule { nets: vec!["*".into()], max_current_a: Some(-1.0), ..NetRule::default() },
            ],
        };
        assert_eq!(
            bad.apply(&mut b),
            Err(RuleError::NotPositive { index: 1, field: "max_current_a" })
        );
        assert_eq!(c(&b, "VBUS").max_vias, None, "nothing applied on error");
        let range = RuleSet {
            stackup: None,
            rules: vec![NetRule {
                length_min_mm: Some(5.0),
                length_max_mm: Some(1.0),
                ..NetRule::default()
            }],
        };
        assert_eq!(range.validate(), Err(RuleError::LengthRange { index: 0 }));
        let nan = RuleSet {
            stackup: Some(StackupSpec { epsilon_r: Some(f64::NAN), ..StackupSpec::default() }),
            rules: vec![],
        };
        assert_eq!(nan.validate(), Err(RuleError::Stackup { field: "epsilon_r" }));
    }

    #[test]
    fn stackup_overrides() {
        let mut b = board();
        let rs = RuleSet {
            stackup: Some(StackupSpec {
                copper_um: Some(70.0),
                outer_dielectric_mm: Some(0.2),
                epsilon_r: Some(3.66),
            }),
            rules: vec![],
        };
        rs.apply(&mut b).expect("valid");
        assert_eq!(b.stackup.copper_thickness, vec![70_000, 70_000]);
        assert_eq!(b.stackup.outer_dielectric, 200_000);
        assert_eq!(b.stackup.epsilon_r_milli, 3660);
    }
}
