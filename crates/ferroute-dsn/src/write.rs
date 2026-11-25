// SPDX-License-Identifier: AGPL-3.0-only
//! Board to Specctra DSN.

use crate::DsnError;
use ferroute_board::constraints::required_width;
use ferroute_board::{Board, LayerId, LayerSet, NetId, Origin, PadShape};
use ferroute_geom::{Angle, Coord, Point, Polygon};
use std::collections::BTreeMap;
use std::fmt::Write as _;

/// Design units per micrometre (`(resolution um 1)`: one unit = 1 um). Freerouting 2.5.0
/// overflows its coordinates at nanometre resolution (diameters read -1, keep-outs
/// "degenerate", nothing routes) and writes SES numbers multiplied by the resolution twice
/// for any resolution other than `um 1` (`um 10` gives 10x too large coordinates), so
/// `um 1` is the only resolution that round-trips.
pub(crate) const RESOLUTION: i64 = 1;
/// Nanometres per design unit.
pub(crate) const UNIT_NM: i64 = 1000 / RESOLUTION;

/// Position in design units, rounded half away from zero.
pub(crate) fn q(c: Coord) -> Coord {
    let h = UNIT_NM / 2;
    if c >= 0 { (c + h) / UNIT_NM } else { -((-c + h) / UNIT_NM) }
}

/// Size in design units, rounded up (never narrower/smaller than the true value).
pub(crate) fn qc(c: Coord) -> Coord {
    (c + UNIT_NM - 1) / UNIT_NM
}

/// A through/blind/buried via padstack identity.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct ViaKey {
    pub first: u8,
    pub last: u8,
    pub diameter: Coord,
    pub drill: Coord,
}

impl ViaKey {
    pub(crate) fn name(self) -> String {
        format!("Via[{}-{}]_{}:{}_nm", self.first, self.last, self.diameter, self.drill)
    }

    /// Parses `Via[a-b]_D:H_<unit>` (our own names and KiCad-style `um`/`mm` names).
    pub(crate) fn parse(name: &str) -> Option<Self> {
        let rest = name.strip_prefix("Via[")?;
        let (span, rest) = rest.split_once("]_")?;
        let (a, b) = span.split_once('-')?;
        let (dims, unit) = rest.rsplit_once('_')?;
        let (d, h) = dims.split_once(':')?;
        let u = crate::units::unit_nm(unit)?;
        Some(Self {
            first: a.parse().ok()?,
            last: b.parse().ok()?,
            diameter: crate::units::scale_decimal(d, u, 1).ok()?,
            drill: crate::units::scale_decimal(h, u, 1).ok()?,
        })
    }
}

/// Layer span `(first, last)` of a set of copper layers.
fn span(layers: LayerSet) -> (u8, u8) {
    let mut it = layers.iter();
    let first = it.next().map_or(0, |l| l.0);
    (first, it.last().map_or(first, |l| l.0))
}

fn tok(s: &str) -> Result<String, DsnError> {
    if s.contains('"') {
        return Err(DsnError::Unrepresentable(s.to_owned()));
    }
    let plain = !s.is_empty() && !s.chars().any(|c| c.is_whitespace() || c == '(' || c == ')');
    Ok(if plain { s.to_owned() } else { format!("\"{s}\"") })
}

fn pt(p: Point) -> String {
    format!("{} {}", q(p.x), q(-p.y))
}

fn pts(ps: &[Point]) -> String {
    ps.iter().map(|&p| pt(p)).collect::<Vec<_>>().join(" ")
}

/// Specctra items (one per layer) describing a pad's copper around its own origin.
fn pad_items(shape: &PadShape, rot: Angle, layers: LayerSet, names: &[String]) -> String {
    let quarter = rot.quadrant().map(|q| q % 2 == 1);
    let rect_pts = |w: Coord, h: Coord| {
        let (hw, hh) = ((w + 1) / 2, (h + 1) / 2);
        [Point::new(-hw, -hh), Point::new(hw, -hh), Point::new(hw, hh), Point::new(-hw, hh)]
    };
    let mut out = Vec::new();
    for l in layers.iter() {
        let ln = &names[l.0 as usize];
        let item = match (shape, quarter) {
            (PadShape::Circle { diameter }, _) => format!("(circle {ln} {} 0 0)", qc(*diameter)),
            (PadShape::Rect { w, h } | PadShape::RoundRect { w, h, .. }, Some(odd)) => {
                let (w, h) = if odd { (*h, *w) } else { (*w, *h) };
                let (hw, hh) = (qc((w + 1) / 2), qc((h + 1) / 2));
                format!("(rect {ln} {} {} {hw} {hh})", -hw, -hh)
            }
            (PadShape::Oval { w, h }, Some(odd)) => {
                let (w, h) = if odd { (*h, *w) } else { (*w, *h) };
                if w >= h {
                    let d = (w - h) / 2;
                    format!("(path {ln} {} {} 0 {} 0)", qc(h), -qc(d), qc(d))
                } else {
                    let d = (h - w) / 2;
                    format!("(path {ln} {} 0 {} 0 {})", qc(w), -qc(d), qc(d))
                }
            }
            (PadShape::Rect { w, h } | PadShape::RoundRect { w, h, .. }, None) => {
                let p: Vec<Point> = rect_pts(*w, *h).iter().map(|q| q.rotated(rot)).collect();
                format!("(polygon {ln} 0 {})", pts(&p))
            }
            (PadShape::Oval { w, h }, None) => {
                let p: Vec<Point> = rect_pts(*w, *h).iter().map(|q| q.rotated(rot)).collect();
                format!("(polygon {ln} 0 {})", pts(&p))
            }
            (PadShape::Convex { pts: cp }, _) => {
                let p: Vec<Point> = cp.iter().map(|q| q.rotated(rot)).collect();
                format!("(polygon {ln} 0 {})", pts(&p))
            }
        };
        out.push(format!("(shape {item})"));
    }
    out.join(" ")
}

/// Writes `board` as a Specctra DSN design file.
///
/// Deterministic: the output is a pure function of `board`.
///
/// # Errors
/// [`DsnError::Io`] on write failure, [`DsnError::Unrepresentable`] for names containing `"`.
#[allow(clippy::too_many_lines)]
pub fn write_dsn(board: &Board, out: &mut dyn std::io::Write) -> Result<(), DsnError> {
    let layer_names: Vec<String> = board.copper_layers.iter().map(|l| l.name.clone()).collect();
    let mut s = String::new();
    let n_layers = board.layer_count();
    let all = board.all_layers();
    let rules = &board.rules;
    let class0 = &rules.classes[0];

    // --- padstacks -------------------------------------------------------------------
    // Pad padstacks keyed by their shape text, numbered in key order.
    let mut pad_keys: BTreeMap<String, usize> = BTreeMap::new();
    let mut pad_key_of: Vec<String> = Vec::with_capacity(board.pads.len());
    for fp in &board.footprints {
        for &pid in &fp.pads {
            let pad = board.pad(pid);
            let rot = fp.placement.rot.add(pad.local_rot);
            let key = pad_items(&pad.shape, rot, pad.layers, &layer_names);
            pad_keys.insert(key.clone(), 0);
            pad_key_of.push(key);
        }
    }
    for (i, v) in pad_keys.values_mut().enumerate() {
        *v = i + 1;
    }
    // Vias: classes' through vias plus every existing via.
    let mut via_keys: Vec<ViaKey> = rules
        .classes
        .iter()
        .map(|c| ViaKey {
            first: 0,
            last: n_layers - 1,
            diameter: c.via_diameter,
            drill: c.via_drill,
        })
        .chain(board.vias.iter().map(|v| {
            let (first, last) = span(v.layers);
            ViaKey { first, last, diameter: v.diameter, drill: v.drill }
        }))
        .collect();
    via_keys.sort();
    via_keys.dedup();

    // --- header ----------------------------------------------------------------------
    writeln!(s, "(pcb ferroute").unwrap_or(());
    writeln!(s, "  (parser").unwrap_or(());
    writeln!(s, "    (string_quote \")").unwrap_or(());
    writeln!(s, "    (space_in_quoted_tokens on)").unwrap_or(());
    writeln!(s, "    (host_cad ferroute)").unwrap_or(());
    writeln!(s, "    (host_version {})", env!("CARGO_PKG_VERSION")).unwrap_or(());
    writeln!(s, "  )").unwrap_or(());
    writeln!(s, "  (resolution um {RESOLUTION})").unwrap_or(());

    // --- structure -------------------------------------------------------------------
    writeln!(s, "  (structure").unwrap_or(());
    for (i, l) in layer_names.iter().enumerate() {
        writeln!(s, "    (layer {} (type signal) (property (index {i})))", tok(l)?).unwrap_or(());
    }
    let ring = |p: &Polygon| -> Vec<Point> { p.pts.clone() };
    let mut outer = ring(&board.outline.outer);
    if let Some(&first) = outer.first() {
        outer.push(first);
    }
    writeln!(s, "    (boundary (path pcb 0 {}))", pts(&outer)).unwrap_or(());
    let mut ko = 0usize;
    for hole in &board.outline.holes {
        for l in all.iter() {
            ko += 1;
            writeln!(
                s,
                "    (keepout KO{ko} (polygon {} 0 {}))",
                tok(&layer_names[l.0 as usize])?,
                pts(&hole.pts)
            )
            .unwrap_or(());
        }
    }
    for k in &board.keepouts {
        let kind = match (k.no_tracks, k.no_vias) {
            (true, true) => "keepout",
            (true, false) => "wire_keepout",
            (false, true) => "via_keepout",
            (false, false) => continue,
        };
        for l in k.layers.iter() {
            ko += 1;
            writeln!(
                s,
                "    ({kind} KO{ko} (polygon {} 0 {}))",
                tok(&layer_names[l.0 as usize])?,
                pts(&k.area.pts)
            )
            .unwrap_or(());
        }
    }
    // Copper pours: KiCad's own export uses `(plane net polygon)` on the zone outline, so the
    // pour is a conduction area (own-net pins connect, other nets keep clear) instead of
    // invisible copper that Freerouting routes straight through.
    for z in board.zones.iter().filter(|z| z.net != NetId::NONE) {
        for l in z.layers.iter() {
            writeln!(
                s,
                "    (plane {} (polygon {} 0 {}))",
                tok(&board.nets[z.net.idx()].name)?,
                tok(&layer_names[l.0 as usize])?,
                pts(&z.outline.pts)
            )
            .unwrap_or(());
        }
    }
    let via_list: Vec<String> =
        via_keys.iter().map(|k| tok(&k.name())).collect::<Result<_, _>>()?;
    writeln!(s, "    (via {})", via_list.join(" ")).unwrap_or(());
    writeln!(
        s,
        "    (rule (width {}) (clearance {}))",
        qc(class0.track_width),
        qc(class0.clearance)
    )
    .unwrap_or(());
    writeln!(s, "  )").unwrap_or(());

    // --- placement -------------------------------------------------------------------
    let mut image_names: Vec<String> = Vec::with_capacity(board.footprints.len());
    for (i, fp) in board.footprints.iter().enumerate() {
        let mut n = if fp.reference.is_empty() { format!("FP{i}") } else { fp.reference.clone() };
        while image_names.contains(&n) {
            n = format!("{n}_{i}");
        }
        image_names.push(n);
    }
    writeln!(s, "  (placement").unwrap_or(());
    for (fp, img) in board.footprints.iter().zip(&image_names) {
        let lock = if fp.locked || fp.mobility == ferroute_board::Mobility::Locked {
            " (lock_type position)"
        } else {
            ""
        };
        writeln!(
            s,
            "    (component {} (place {} {} front 0{lock}))",
            tok(img)?,
            tok(img)?,
            pt(fp.placement.pos)
        )
        .unwrap_or(());
    }
    writeln!(s, "  )").unwrap_or(());

    // --- library ---------------------------------------------------------------------
    writeln!(s, "  (library").unwrap_or(());
    let mut pin_ids: Vec<Vec<String>> = Vec::with_capacity(board.footprints.len());
    let mut pad_cursor = 0usize;
    for (fp, img) in board.footprints.iter().zip(&image_names) {
        writeln!(s, "    (image {}", tok(img)?).unwrap_or(());
        let mut used: Vec<String> = Vec::new();
        for (j, &pid) in fp.pads.iter().enumerate() {
            let pad = board.pad(pid);
            let mut name = if pad.number.is_empty() { format!("P{j}") } else { pad.number.clone() };
            let mut k = 2;
            while used.contains(&name) {
                name = format!("{}_{k}", pad.number);
                k += 1;
            }
            used.push(name.clone());
            let ps = pad_keys[&pad_key_of[pad_cursor]];
            pad_cursor += 1;
            let off = pad.position(fp.placement) - fp.placement.pos;
            writeln!(s, "      (pin Pad{ps} {} {})", tok(&name)?, pt(off)).unwrap_or(());
        }
        pin_ids.push(used);
        writeln!(s, "    )").unwrap_or(());
    }
    for (key, ps) in &pad_keys {
        writeln!(s, "    (padstack Pad{ps} {key} (attach off))").unwrap_or(());
    }
    for k in &via_keys {
        let layers = LayerSet::span(LayerId(k.first), LayerId(k.last));
        let shapes: Vec<String> = layers
            .iter()
            .map(|l| {
                Ok(format!(
                    "(shape (circle {} {} 0 0))",
                    tok(&layer_names[l.0 as usize])?,
                    qc(k.diameter)
                ))
            })
            .collect::<Result<_, DsnError>>()?;
        writeln!(s, "    (padstack {} {} (attach off))", tok(&k.name())?, shapes.join(" "))
            .unwrap_or(());
    }
    writeln!(s, "  )").unwrap_or(());

    // --- network ---------------------------------------------------------------------
    writeln!(s, "  (network").unwrap_or(());
    let mut net_pins: Vec<Vec<String>> = vec![Vec::new(); board.nets.len()];
    for (fi, fp) in board.footprints.iter().enumerate() {
        for (j, &pid) in fp.pads.iter().enumerate() {
            let net = board.pad(pid).net;
            if net != NetId::NONE {
                net_pins[net.idx()].push(tok(&format!("{}-{}", image_names[fi], pin_ids[fi][j]))?);
            }
        }
    }
    for id in board.net_ids().filter(|&n| n != NetId::NONE) {
        writeln!(
            s,
            "    (net {} (pins {}))",
            tok(&board.nets[id.idx()].name)?,
            net_pins[id.idx()].join(" ")
        )
        .unwrap_or(());
    }
    // Classes keyed by (class index, effective width).
    let mut classes: BTreeMap<(u16, Coord), Vec<NetId>> = BTreeMap::new();
    for id in board.net_ids().filter(|&n| n != NetId::NONE) {
        classes
            .entry((board.nets[id.idx()].class, required_width(board, id)))
            .or_default()
            .push(id);
    }
    for ((ci, width), nets) in &classes {
        let c = rules.classes.get(usize::from(*ci)).unwrap_or(class0);
        let name =
            if *width == c.track_width { c.name.clone() } else { format!("{}_w{width}", c.name) };
        let names: Vec<String> =
            nets.iter().map(|n| tok(&board.nets[n.idx()].name)).collect::<Result<_, _>>()?;
        let via =
            ViaKey { first: 0, last: n_layers - 1, diameter: c.via_diameter, drill: c.via_drill };
        writeln!(
            s,
            "    (class {} {} (circuit (use_via {})) (rule (width {}) (clearance {})))",
            tok(&name)?,
            names.join(" "),
            tok(&via.name())?,
            qc(*width),
            qc(c.clearance)
        )
        .unwrap_or(());
    }
    writeln!(s, "  )").unwrap_or(());

    // --- wiring ----------------------------------------------------------------------
    writeln!(s, "  (wiring").unwrap_or(());
    let net_clause = |n: NetId| -> Result<String, DsnError> {
        if n == NetId::NONE {
            Ok(String::new())
        } else {
            Ok(format!(" (net {})", tok(&board.nets[n.idx()].name)?))
        }
    };
    for t in board.tracks.iter().filter(|t| t.origin == Origin::Existing) {
        writeln!(
            s,
            "    (wire (path {} {} {}){} (type protect))",
            tok(&layer_names[t.layer.0 as usize])?,
            qc(t.width),
            pts(&[t.start, t.end]),
            net_clause(t.net)?
        )
        .unwrap_or(());
    }
    for a in board.arcs.iter().filter(|a| a.origin == Origin::Existing) {
        writeln!(
            s,
            "    (wire (path {} {} {}){} (type protect))",
            tok(&layer_names[a.layer.0 as usize])?,
            qc(a.width),
            pts(&[a.start, a.mid, a.end]),
            net_clause(a.net)?
        )
        .unwrap_or(());
    }
    for v in board.vias.iter().filter(|v| v.origin == Origin::Existing) {
        let (first, last) = span(v.layers);
        let key = ViaKey { first, last, diameter: v.diameter, drill: v.drill };
        writeln!(
            s,
            "    (via {} {}{} (type protect))",
            tok(&key.name())?,
            pt(v.pos),
            net_clause(v.net)?
        )
        .unwrap_or(());
    }
    writeln!(s, "  )").unwrap_or(());
    writeln!(s, ")").unwrap_or(());
    out.write_all(s.as_bytes())?;
    Ok(())
}
