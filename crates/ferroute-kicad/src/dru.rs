// SPDX-License-Identifier: AGPL-3.0-only
//! `.kicad_dru` custom design rules (KiCad 10 `pcbnew/drc/drc_rule_parser.cpp`).
//!
//! Supported: `(version N)`, `(rule NAME (condition "..") (layer ..) (severity ..)
//! (constraint TYPE (min ..) (max ..) (opt ..)))` for clearance, track_width,
//! via_diameter, hole_size, hole_to_hole, hole_clearance, edge_clearance, annular_width, diff_pair_gap,
//! length and skew. Conditions: `A.`/`B.` `NetClass`, `NetName`, `Type`, `Layer` compared
//! with `==`/`!=` (KiCad `*`/`?` wildcards), combined with `&&`, `||`, `!`, parentheses.
//! `(severity error|warning|ignore|exclusion)` is honoured: warnings are reported as
//! non-error violations, `ignore`/`exclusion` suppress; an unknown severity is an error.
//! Anything else (`insideArea`, `Pad_Type`, ...) becomes [`Cond::Unsupported`] and is
//! reported in the returned warnings; unsupported constraint types are skipped with a
//! warning.

use crate::KicadError;
use crate::sexpr::{SExpr, parse};
use ferroute_board::custom::{
    CmpOp, Cond, ConstraintKind, CustomRule, Field, LayerFilter, Limits, RuleSide, Severity,
};
use ferroute_geom::Coord;
use ferroute_geom::units::mm_to_nm;

/// Parsed rules plus human-readable warnings.
#[derive(Debug, Default, PartialEq)]
pub struct DruRules {
    pub rules: Vec<CustomRule>,
    pub warnings: Vec<String>,
}

/// Parses the text of a `.kicad_dru` file.
///
/// # Errors
/// [`KicadError`] when the S-expression structure is broken (rules must not be silently
/// dropped).
pub fn parse_dru(src: &str) -> Result<DruRules, KicadError> {
    let body: String =
        src.lines().filter(|l| !l.trim_start().starts_with('#')).collect::<Vec<_>>().join("\n");
    let root = parse(&format!("(dru {body}\n)"))?;
    let mut out = DruRules::default();
    for item in root.items().unwrap_or_default().iter().skip(1) {
        match item.head() {
            Some("version") => {}
            Some("rule") => parse_rule(item, &mut out),
            other => out
                .warnings
                .push(format!("unrecognised top-level item '{}' ignored", other.unwrap_or("?"))),
        }
    }
    Ok(out)
}

fn parse_rule(node: &SExpr, out: &mut DruRules) {
    let name = node.atom(1).unwrap_or("").to_owned();
    let mut rule = CustomRule {
        name: name.clone(),
        condition: Cond::True,
        layer: None,
        constraints: Vec::new(),
        severity: Severity::Error,
    };
    let warn = |out: &mut DruRules, m: String| out.warnings.push(format!("rule '{name}': {m}"));
    for c in node.items().unwrap_or_default().iter().skip(2) {
        match c.head() {
            Some("condition") => {
                let (cond, unsupported) = parse_condition(c.atom(1).unwrap_or(""));
                for u in unsupported {
                    warn(out, format!("unsupported condition term `{u}` (treated conservatively)"));
                }
                rule.condition = cond;
            }
            Some("layer") => {
                rule.layer = Some(match c.atom(1).unwrap_or("") {
                    "outer" => LayerFilter::Outer,
                    "inner" => LayerFilter::Inner,
                    n => LayerFilter::Named(n.to_owned()),
                });
            }
            Some("severity") => {
                let s = c.atom(1).unwrap_or("");
                if let Some(sev) = Severity::from_keyword(s) {
                    rule.severity = sev;
                } else {
                    warn(out, format!("unknown severity '{s}': enforced as error"));
                }
            }
            Some("constraint") => {
                let ty = c.atom(1).unwrap_or("");
                let Some(kind) = ConstraintKind::from_keyword(ty) else {
                    warn(out, format!("constraint '{ty}' is not supported and is not enforced"));
                    continue;
                };
                match parse_limits(c) {
                    Ok(l) => rule.constraints.push((kind, l)),
                    Err(e) => warn(out, format!("constraint '{ty}' skipped: {e}")),
                }
            }
            other => warn(out, format!("unrecognised item '{}' ignored", other.unwrap_or("?"))),
        }
    }
    if rule.constraints.is_empty() {
        warn(out, "no supported constraint; rule dropped".into());
    } else {
        out.rules.push(rule);
    }
}

fn parse_limits(c: &SExpr) -> Result<Limits, String> {
    let mut l = Limits::default();
    for p in c.items().unwrap_or_default().iter().skip(2) {
        let slot = match p.head() {
            Some("min") => &mut l.min,
            Some("max") => &mut l.max,
            Some("opt") => &mut l.opt,
            _ => continue,
        };
        *slot = Some(parse_length(p.atom(1).unwrap_or(""))?);
    }
    Ok(l)
}

/// `0.5mm`, `20mil`, `0.01in`, `0.2` (mm), `-1.5mm`.
fn parse_length(s: &str) -> Result<Coord, String> {
    let t = s.trim();
    let split = t.find(|c: char| c.is_ascii_alphabetic() || c == '"').unwrap_or(t.len());
    let (num, unit) = t.split_at(split);
    let v: f64 = num.trim().parse().map_err(|_| format!("bad number '{s}'"))?;
    let mm = match unit.trim() {
        "" | "mm" => v,
        "in" | "\"" => v * 25.4,
        "mil" => v * 0.0254,
        u => return Err(format!("unsupported unit '{u}'")),
    };
    Ok(mm_to_nm(mm))
}

// ---- condition expressions -----------------------------------------------------------

#[derive(Clone, Debug, PartialEq)]
enum Tok {
    Ident(String),
    Str(String),
    Eq,
    Ne,
    And,
    Or,
    Not,
    LParen,
    RParen,
    Other(String),
}

fn lex(s: &str) -> Vec<Tok> {
    let cs: Vec<char> = s.chars().collect();
    let mut i = 0;
    let mut out = Vec::new();
    while i < cs.len() {
        let c = cs[i];
        let next = cs.get(i + 1).copied();
        match c {
            _ if c.is_whitespace() => i += 1,
            '=' if next == Some('=') => {
                out.push(Tok::Eq);
                i += 2;
            }
            '!' if next == Some('=') => {
                out.push(Tok::Ne);
                i += 2;
            }
            '&' if next == Some('&') => {
                out.push(Tok::And);
                i += 2;
            }
            '|' if next == Some('|') => {
                out.push(Tok::Or);
                i += 2;
            }
            '!' => {
                out.push(Tok::Not);
                i += 1;
            }
            '(' => {
                out.push(Tok::LParen);
                i += 1;
            }
            ')' => {
                out.push(Tok::RParen);
                i += 1;
            }
            '\'' | '"' => {
                let q = c;
                let mut j = i + 1;
                let mut t = String::new();
                while j < cs.len() && cs[j] != q {
                    t.push(cs[j]);
                    j += 1;
                }
                out.push(Tok::Str(t));
                i = j + 1;
            }
            _ if c.is_alphanumeric() || c == '_' || c == '.' => {
                let mut j = i;
                while j < cs.len() && (cs[j].is_alphanumeric() || cs[j] == '_' || cs[j] == '.') {
                    j += 1;
                }
                out.push(Tok::Ident(cs[i..j].iter().collect()));
                i = j;
            }
            _ => {
                out.push(Tok::Other(c.to_string()));
                i += 1;
            }
        }
    }
    out
}

struct Parser {
    t: Vec<Tok>,
    i: usize,
    unsupported: Vec<String>,
}

fn field(name: &str) -> Option<(RuleSide, Field)> {
    let (s, f) = name.split_once('.')?;
    let side = match s {
        "A" => RuleSide::A,
        "B" => RuleSide::B,
        _ => return None,
    };
    let f = match f.to_ascii_lowercase().as_str() {
        "netclass" => Field::NetClass,
        "netname" => Field::NetName,
        "type" => Field::Type,
        "layer" => Field::Layer,
        _ => return None,
    };
    Some((side, f))
}

impl Parser {
    fn peek(&self) -> Option<&Tok> {
        self.t.get(self.i)
    }

    fn or(&mut self) -> Cond {
        let mut v = vec![self.and()];
        while self.peek() == Some(&Tok::Or) {
            self.i += 1;
            v.push(self.and());
        }
        if v.len() == 1 { v.remove(0) } else { Cond::Or(v) }
    }

    fn and(&mut self) -> Cond {
        let mut v = vec![self.unary()];
        while self.peek() == Some(&Tok::And) {
            self.i += 1;
            v.push(self.unary());
        }
        if v.len() == 1 { v.remove(0) } else { Cond::And(v) }
    }

    fn unary(&mut self) -> Cond {
        if self.peek() == Some(&Tok::Not) {
            self.i += 1;
            return Cond::Not(Box::new(self.unary()));
        }
        if self.peek() == Some(&Tok::LParen) {
            self.i += 1;
            let c = self.or();
            if self.peek() == Some(&Tok::RParen) {
                self.i += 1;
            }
            return c;
        }
        self.term()
    }

    /// Consumes tokens up to the next boolean operator at paren depth 0 as one opaque term.
    fn skip_term(&mut self, start: usize) -> Cond {
        let mut depth = 0i32;
        while let Some(t) = self.peek() {
            match t {
                Tok::LParen => depth += 1,
                Tok::RParen if depth == 0 => break,
                Tok::RParen => depth -= 1,
                Tok::And | Tok::Or if depth == 0 => break,
                _ => {}
            }
            self.i += 1;
        }
        let text = self.t[start..self.i].iter().map(tok_text).collect::<Vec<_>>().join(" ");
        self.unsupported.push(text.clone());
        Cond::Unsupported(text)
    }

    fn term(&mut self) -> Cond {
        let start = self.i;
        if let (Some(Tok::Ident(id)), Some(Tok::LParen), Some(Tok::Str(v)), Some(Tok::RParen)) =
            (self.t.get(start), self.t.get(start + 1), self.t.get(start + 2), self.t.get(start + 3))
            && let Some(side) = id.split_once('.').and_then(|(s, f)| {
                let side = match s {
                    "A" => RuleSide::A,
                    "B" => RuleSide::B,
                    _ => return None,
                };
                f.eq_ignore_ascii_case("memberOfFootprint").then_some(side)
            })
            && matches!(self.t.get(start + 4), None | Some(Tok::And | Tok::Or | Tok::RParen))
        {
            self.i = start + 4;
            return Cond::MemberOfFootprint { side, pattern: v.clone() };
        }
        if let (Some(Tok::Ident(id)), Some(op), Some(Tok::Str(v))) =
            (self.t.get(start), self.t.get(start + 1), self.t.get(start + 2))
        {
            let cmp = match op {
                Tok::Eq => Some(CmpOp::Eq),
                Tok::Ne => Some(CmpOp::Ne),
                _ => None,
            };
            if let (Some((side, f)), Some(op)) = (field(id), cmp) {
                let end = start + 3;
                // Only a clean term (followed by a boolean operator / end) is understood.
                if matches!(self.t.get(end), None | Some(Tok::And | Tok::Or | Tok::RParen)) {
                    self.i = end;
                    return Cond::Cmp { side, field: f, op, value: v.clone() };
                }
            }
        }
        self.skip_term(start)
    }
}

fn tok_text(t: &Tok) -> String {
    match t {
        Tok::Ident(s) | Tok::Other(s) => s.clone(),
        Tok::Str(s) => format!("'{s}'"),
        Tok::Eq => "==".into(),
        Tok::Ne => "!=".into(),
        Tok::And => "&&".into(),
        Tok::Or => "||".into(),
        Tok::Not => "!".into(),
        Tok::LParen => "(".into(),
        Tok::RParen => ")".into(),
    }
}

/// Parses a condition; the second value lists the unsupported terms.
fn parse_condition(s: &str) -> (Cond, Vec<String>) {
    if s.trim().is_empty() {
        return (Cond::True, Vec::new());
    }
    let mut p = Parser { t: lex(s), i: 0, unsupported: Vec::new() };
    let c = p.or();
    if p.i < p.t.len() {
        // Trailing garbage (e.g. unbalanced parentheses): the whole expression is opaque.
        return (Cond::Unsupported(s.to_owned()), vec![s.to_owned()]);
    }
    (c, p.unsupported)
}

#[cfg(test)]
mod tests {
    use super::*;

    const DOC: &str = r#"
(version 1)
# netclass to netclass
(rule "HV clearance"
  (condition "A.NetClass == 'HV' && B.NetClass == 'Default'")
  (constraint clearance (min 1.5mm)))
(rule "hv names" (condition "A.NetName == '/HV*'") (constraint clearance (min 40mil)))
(rule outer_w (layer outer) (constraint track_width (min 0.3) (opt 0.4mm)))
(rule area (condition "A.insideArea('X') && A.NetName != 'GND'")
  (constraint clearance (min 1mm)))
(rule weird (constraint courtyard_clearance (min 1mm)))
(rule len (condition "A.Type == 'track' || !(A.Layer == 'inner')") (constraint length (max 100mm)))
"#;

    #[test]
    fn parses_documented_examples() {
        let d = parse_dru(DOC).unwrap();
        assert_eq!(d.rules.len(), 5, "{:?}", d.warnings);
        assert_eq!(d.rules[0].name, "HV clearance");
        assert!(matches!(&d.rules[0].condition, Cond::And(v) if v.len() == 2));
        let l = d.rules[0].limits(ConstraintKind::Clearance).unwrap();
        assert_eq!(l.min, Some(1_500_000));
        assert_eq!(d.rules[1].limits(ConstraintKind::Clearance).unwrap().min, Some(1_016_000));
        assert_eq!(d.rules[2].layer, Some(LayerFilter::Outer));
        let w = d.rules[2].limits(ConstraintKind::TrackWidth).unwrap();
        assert_eq!((w.min, w.opt), (Some(300_000), Some(400_000)));
        assert_eq!(d.rules[4].limits(ConstraintKind::Length).unwrap().max, Some(100_000_000));
    }

    #[test]
    fn unsupported_terms_warn_and_stay_maybe() {
        let d = parse_dru(DOC).unwrap();
        let area = &d.rules[3];
        assert!(matches!(&area.condition, Cond::And(v)
            if matches!(v[0], Cond::Unsupported(ref s) if s.contains("insideArea"))
            && matches!(v[1], Cond::Cmp { op: CmpOp::Ne, .. })));
        assert!(d.warnings.iter().any(|w| w.contains("insideArea")), "{:?}", d.warnings);
        assert!(d.warnings.iter().any(|w| w.contains("courtyard_clearance")));
    }

    #[test]
    fn member_of_footprint_conditions_parse() {
        let d = parse_dru(
            r#"(version 1)
(rule "RJ45 shell tabs at locating posts"
	(condition "A.memberOfFootprint('RJ*') && B.memberOfFootprint('RJ*')")
	(constraint hole_clearance (min 0mm)))
(rule "USB-C pads to locating posts"
	(condition "A.memberOfFootprint('USB1') && B.memberOfFootprint('USB1')")
	(constraint hole_clearance (min 0.2mm)))"#,
        )
        .unwrap();
        assert!(d.warnings.is_empty(), "{:?}", d.warnings);
        let fp = |side, p: &str| Cond::MemberOfFootprint { side, pattern: p.into() };
        assert_eq!(
            d.rules[0].condition,
            Cond::And(vec![fp(RuleSide::A, "RJ*"), fp(RuleSide::B, "RJ*")])
        );
        assert_eq!(
            d.rules[1].condition,
            Cond::And(vec![fp(RuleSide::A, "USB1"), fp(RuleSide::B, "USB1")])
        );
        assert_eq!(d.rules[0].limits(ConstraintKind::HoleClearance).unwrap().min, Some(0));
    }

    #[test]
    fn broken_structure_is_an_error_not_silence() {
        assert!(parse_dru("(rule x (constraint clearance (min 1mm))").is_err());
    }

    #[test]
    fn severity_is_parsed_and_unknown_stays_error() {
        let d = parse_dru(
            r#"(version 1)
(rule a (severity warning) (constraint clearance (min 1mm)))
(rule b (severity ignore) (constraint clearance (min 1mm)))
(rule c (severity exclusion) (constraint clearance (min 1mm)))
(rule d (severity error) (constraint clearance (min 1mm)))
(rule e (severity bogus) (constraint clearance (min 1mm)))
(rule f (constraint clearance (min 1mm)))"#,
        )
        .unwrap();
        let sev: Vec<_> = d.rules.iter().map(|r| r.severity).collect();
        assert_eq!(
            sev,
            [
                Severity::Warning,
                Severity::Ignore,
                Severity::Exclusion,
                Severity::Error,
                Severity::Error,
                Severity::Error
            ]
        );
        assert_eq!(d.warnings.len(), 1, "{:?}", d.warnings);
        assert!(d.warnings[0].contains("bogus"));
    }

    #[test]
    fn hole_to_hole_and_max_limits_parse() {
        let d = parse_dru(
            r#"(rule h (constraint hole_to_hole (min 0.4mm)))
(rule w (constraint track_width (min 0.1mm) (max 1mm) (opt 0.2mm)))
(rule v (constraint via_diameter (max 0.8mm)))
(rule s (constraint hole_size (max 0.5mm)))
(rule r (constraint annular_width (max 0.3mm)))"#,
        )
        .unwrap();
        assert_eq!(d.rules.len(), 5, "{:?}", d.warnings);
        assert_eq!(d.rules[0].limits(ConstraintKind::HoleToHole).unwrap().min, Some(400_000));
        let w = d.rules[1].limits(ConstraintKind::TrackWidth).unwrap();
        assert_eq!((w.min, w.max, w.opt), (Some(100_000), Some(1_000_000), Some(200_000)));
        assert_eq!(d.rules[2].limits(ConstraintKind::ViaDiameter).unwrap().max, Some(800_000));
        assert_eq!(d.rules[3].limits(ConstraintKind::HoleSize).unwrap().max, Some(500_000));
        assert_eq!(d.rules[4].limits(ConstraintKind::AnnularWidth).unwrap().max, Some(300_000));
    }

    #[test]
    fn units() {
        assert_eq!(parse_length("0.5mm"), Ok(500_000));
        assert_eq!(parse_length("20mil"), Ok(508_000));
        assert_eq!(parse_length("0.01in"), Ok(254_000));
        assert_eq!(parse_length("0.2"), Ok(200_000));
        assert!(parse_length("5ps").is_err());
    }
}
