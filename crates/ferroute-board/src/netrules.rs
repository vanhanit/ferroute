// SPDX-License-Identifier: AGPL-3.0-only
//! User-facing per-net electrical rules (`--rules FILE`), in engineering units.
//!
//! A [`RuleSet`] is a list of [`NetRule`]s, each selecting nets by name and/or net-class
//! glob. Rules apply in order; a later rule overrides only the fields it sets. Values are
//! converted once, at this edge, into the exact integer units of
//! [`NetConstraints`](crate::constraints::NetConstraints).

use crate::constraints::{MatchGroup, NetConstraints};
use crate::{Board, NetId, glob};
use ferroute_geom::Coord;
use ferroute_geom::units::mm_to_nm;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

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

/// A length-matching group. Members are the nets/classes it selects plus every member of
/// the groups it references (`groups`: globs over other match names), so groups nest.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MatchRule {
    pub name: String,
    /// Net-name globs.
    #[serde(default)]
    pub nets: Vec<String>,
    /// Net-class-name globs.
    #[serde(default)]
    pub classes: Vec<String>,
    /// Globs over the names of other match rules.
    #[serde(default)]
    pub groups: Vec<String>,
    /// Allowed length spread within the group (mm).
    pub skew_mm: f64,
}

/// A complete rules file.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RuleSet {
    #[serde(default)]
    pub stackup: Option<StackupSpec>,
    #[serde(default, rename = "net")]
    pub rules: Vec<NetRule>,
    #[serde(default, rename = "match")]
    pub matches: Vec<MatchRule>,
}

/// Rejected rule value.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum RuleError {
    #[error("rule {index}: {field} must be a finite positive number")]
    NotPositive { index: usize, field: &'static str },
    #[error("rule {index}: length_min_mm exceeds length_max_mm")]
    LengthRange { index: usize },
    #[error("match rule {index}: skew_mm must be a finite positive number")]
    MatchSkew { index: usize },
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
        for (index, m) in self.matches.iter().enumerate() {
            if !(m.skew_mm.is_finite() && m.skew_mm > 0.0) {
                return Err(RuleError::MatchSkew { index });
            }
        }
        self.rules.iter().enumerate().try_for_each(|(i, r)| r.validate(i))
    }

    /// Resolves the match rules (direct nets, then group references to a fixpoint) and
    /// appends them to `board.match_groups`; rules ending with < 2 nets are warned about.
    fn apply_matches(&self, board: &mut Board, warnings: &mut Vec<String>) {
        let class_of = |board: &Board, n: usize| {
            board
                .rules
                .classes
                .get(usize::from(board.nets[n].class))
                .map_or_else(|| "Default".to_owned(), |c| c.name.clone())
        };
        let mut members: Vec<BTreeSet<usize>> = self
            .matches
            .iter()
            .map(|m| {
                (1..board.nets.len())
                    .filter(|&n| {
                        let class = class_of(board, n);
                        m.nets.iter().any(|p| glob(p, &board.nets[n].name))
                            || m.classes.iter().any(|p| glob(p, &class))
                    })
                    .collect()
            })
            .collect();
        loop {
            let mut changed = false;
            for i in 0..self.matches.len() {
                for j in 0..self.matches.len() {
                    if i == j
                        || !self.matches[i].groups.iter().any(|p| glob(p, &self.matches[j].name))
                    {
                        continue;
                    }
                    let add: Vec<usize> = members[j].difference(&members[i]).copied().collect();
                    changed |= !add.is_empty();
                    members[i].extend(add);
                }
            }
            if !changed {
                break;
            }
        }
        for (m, nets) in self.matches.iter().zip(members) {
            if nets.len() < 2 {
                warnings.push(format!("match rule {} has fewer than 2 nets", m.name));
                continue;
            }
            board.match_groups.push(MatchGroup {
                name: m.name.clone(),
                skew_max: mm_to_nm(m.skew_mm),
                nets: nets
                    .into_iter()
                    .map(|n| NetId(u32::try_from(n).unwrap_or(u32::MAX)))
                    .collect(),
            });
        }
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
        self.apply_matches(board, &mut warnings);
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
            ..RuleSet::default()
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
            rules: vec![
                NetRule {
                    classes: vec!["*".into()],
                    max_vias: Some(4),
                    max_current_a: Some(0.1),
                    ..NetRule::default()
                },
                NetRule { nets: vec!["SIG".into()], max_vias: Some(1), ..NetRule::default() },
            ],
            ..RuleSet::default()
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
            rules: vec![NetRule { nets: vec!["NOPE*".into()], ..NetRule::default() }],
            ..RuleSet::default()
        };
        assert_eq!(rs.apply(&mut b).expect("valid"), vec!["rule 0 matched no net".to_owned()]);
        let bad = RuleSet {
            rules: vec![
                NetRule { nets: vec!["VBUS".into()], max_vias: Some(1), ..NetRule::default() },
                NetRule { nets: vec!["*".into()], max_current_a: Some(-1.0), ..NetRule::default() },
            ],
            ..RuleSet::default()
        };
        assert_eq!(
            bad.apply(&mut b),
            Err(RuleError::NotPositive { index: 1, field: "max_current_a" })
        );
        assert_eq!(c(&b, "VBUS").max_vias, None, "nothing applied on error");
        let range = RuleSet {
            rules: vec![NetRule {
                length_min_mm: Some(5.0),
                length_max_mm: Some(1.0),
                ..NetRule::default()
            }],
            ..RuleSet::default()
        };
        assert_eq!(range.validate(), Err(RuleError::LengthRange { index: 0 }));
        let nan = RuleSet {
            stackup: Some(StackupSpec { epsilon_r: Some(f64::NAN), ..StackupSpec::default() }),
            rules: vec![],
            ..RuleSet::default()
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
            ..RuleSet::default()
        };
        rs.apply(&mut b).expect("valid");
        assert_eq!(b.stackup.copper_thickness, vec![70_000, 70_000]);
        assert_eq!(b.stackup.outer_dielectric, 200_000);
        assert_eq!(b.stackup.epsilon_r_milli, 3660);
    }

    fn mr(name: &str, nets: &[&str], groups: &[&str], skew: f64) -> MatchRule {
        MatchRule {
            name: name.into(),
            nets: nets.iter().map(|s| (*s).into()).collect(),
            groups: groups.iter().map(|s| (*s).into()).collect(),
            skew_mm: skew,
            ..MatchRule::default()
        }
    }

    fn nets_of(b: &Board, g: &str) -> Vec<String> {
        let g = b.match_groups.iter().find(|m| m.name == g).expect("group");
        g.nets.iter().map(|n| b.nets[n.idx()].name.clone()).collect()
    }

    #[test]
    fn match_rules_nest() {
        let rs = RuleSet {
            matches: vec![
                mr("pair0", &["D0_P", "D0_N"], &[], 0.1),
                mr("pair1", &["D1_P", "D1_N"], &[], 0.1),
                mr("bus", &["CLK"], &["pair*"], 0.5),
                mr("top", &[], &["bus"], 1.0),
            ],
            ..RuleSet::default()
        };
        let mut b = BoardBuilder::new(mm(30), mm(20), 2);
        for n in ["D0_P", "D0_N", "D1_P", "D1_N", "CLK", "X"] {
            b.net(n);
        }
        let mut b = b.build();
        assert!(rs.apply(&mut b).expect("valid").is_empty());
        assert_eq!(b.match_groups.len(), 4);
        assert_eq!(nets_of(&b, "pair0"), ["D0_P", "D0_N"]);
        assert_eq!(nets_of(&b, "bus").len(), 5);
        assert_eq!(nets_of(&b, "top").len(), 5, "group of groups of groups");
        let bus = b.match_groups.iter().find(|m| m.name == "bus").expect("bus");
        assert_eq!(bus.skew_max, 500_000);
    }

    #[test]
    fn match_rules_self_reference_cycles_bad_skew_and_small_groups() {
        let mut b = board();
        let rs = RuleSet {
            matches: vec![
                mr("a", &["USB_D+"], &["a", "b"], 0.1),
                mr("b", &["USB_D-"], &["a"], 0.1),
                mr("lonely", &["SIG"], &[], 0.1),
            ],
            ..RuleSet::default()
        };
        let w = rs.apply(&mut b).expect("valid");
        assert_eq!(w, vec!["match rule lonely has fewer than 2 nets".to_owned()]);
        assert_eq!(nets_of(&b, "a"), nets_of(&b, "b"));
        assert_eq!(nets_of(&b, "a").len(), 2);
        let neg = RuleSet { matches: vec![mr("x", &[], &[], -1.0)], ..RuleSet::default() };
        assert_eq!(neg.validate(), Err(RuleError::MatchSkew { index: 0 }));
    }
}
