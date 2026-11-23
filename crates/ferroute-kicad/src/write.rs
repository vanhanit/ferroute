// SPDX-License-Identifier: AGPL-3.0-only
//! Complete KiCad 10 `.kicad_pcb` generation from a [`Board`].

use crate::doc::{arc_node, track_node, via_node};
use crate::layers::kicad_cu_id;
use crate::read::NetDialect;
use crate::sexpr::SExpr;
use crate::util::{format_angle, kq, kv, node, norm180, uuid_for, xy_node};
use ferroute_board::{Board, Footprint, LayerSet, Pad, PadShape, Side};
use ferroute_geom::units::format_mm;
use ferroute_geom::{Coord, Point, Polygon, Rect};

/// File format version written by KiCad 10 (`SEXPR_BOARD_FILE_VERSION`).
pub const KICAD10_VERSION: u32 = 20_260_206;

/// Domain tag for sequentially numbered UUIDs (footprints, pads, text, graphics, zones).
const KIND_MISC: u64 = 99;

struct Ids {
    seed: u64,
    n: u64,
}

impl Ids {
    fn next(&mut self) -> SExpr {
        let i = self.n;
        self.n += 1;
        kq("uuid", uuid_for(self.seed, KIND_MISC, i))
    }
}

fn stroke(width: Coord) -> SExpr {
    node("stroke", vec![kv("width", format_mm(width)), kv("type", "solid")])
}

fn pts_node(pts: &[Point]) -> SExpr {
    node("pts", pts.iter().map(|&p| xy_node("xy", p)).collect())
}

fn layers_table(board: &Board) -> SExpr {
    let n = board.copper_layers.len();
    let mut items = Vec::new();
    for (i, l) in board.copper_layers.iter().enumerate() {
        items.push(SExpr::list(vec![
            SExpr::sym(kicad_cu_id(i, n).to_string()),
            SExpr::quoted(l.name.clone()),
            SExpr::sym(if l.routable { "signal" } else { "power" }),
        ]));
    }
    let tech: [(u32, &str, &str, Option<&str>); 20] = [
        (9, "F.Adhes", "user", Some("F.Adhesive")),
        (11, "B.Adhes", "user", Some("B.Adhesive")),
        (13, "F.Paste", "user", None),
        (15, "B.Paste", "user", None),
        (5, "F.SilkS", "user", Some("F.Silkscreen")),
        (7, "B.SilkS", "user", Some("B.Silkscreen")),
        (1, "F.Mask", "user", None),
        (3, "B.Mask", "user", None),
        (17, "Dwgs.User", "user", Some("User.Drawings")),
        (19, "Cmts.User", "user", Some("User.Comments")),
        (21, "Eco1.User", "user", Some("User.Eco1")),
        (23, "Eco2.User", "user", Some("User.Eco2")),
        (25, "Edge.Cuts", "user", None),
        (27, "Margin", "user", None),
        (31, "F.CrtYd", "user", Some("F.Courtyard")),
        (29, "B.CrtYd", "user", Some("B.Courtyard")),
        (35, "F.Fab", "user", None),
        (33, "B.Fab", "user", None),
        (39, "User.1", "user", None),
        (41, "User.2", "user", None),
    ];
    for (id, name, ty, user) in tech {
        let mut v = vec![SExpr::sym(id.to_string()), SExpr::quoted(name), SExpr::sym(ty)];
        if let Some(u) = user {
            v.push(SExpr::quoted(u));
        }
        items.push(SExpr::list(v));
    }
    node("layers", items)
}

fn setup_node() -> SExpr {
    let yn = |k: &str, a: &str, b: &str| node(k, vec![kv("front", a), kv("back", b)]);
    node(
        "setup",
        vec![
            kv("pad_to_mask_clearance", "0"),
            kv("allow_soldermask_bridges_in_footprints", "no"),
            yn("tenting", "yes", "yes"),
            yn("covering", "no", "no"),
            yn("plugging", "no", "no"),
            kv("capping", "no"),
            kv("filling", "no"),
        ],
    )
}

fn at_node(p: Point, angle: Option<f64>, always_angle: bool) -> SExpr {
    let mut v = vec![SExpr::sym("at"), SExpr::sym(format_mm(p.x)), SExpr::sym(format_mm(p.y))];
    match angle {
        Some(a) if always_angle || norm180(a).abs() > 1e-9 => {
            v.push(SExpr::sym(format_angle(norm180(a))));
        }
        _ => {}
    }
    SExpr::List(v)
}

fn text_property(
    ids: &mut Ids,
    key: &str,
    value: &str,
    pos: Point,
    abs_rot: f64,
    layer: &str,
    back: bool,
) -> SExpr {
    let mut effects = vec![node(
        "font",
        vec![node("size", vec![SExpr::sym("1"), SExpr::sym("1")]), kv("thickness", "0.15")],
    )];
    if back {
        effects.push(node("justify", vec![SExpr::sym("mirror")]));
    }
    node(
        "property",
        vec![
            SExpr::quoted(key),
            SExpr::quoted(value),
            at_node(pos, Some(abs_rot), true),
            kq("layer", layer),
            ids.next(),
            node("effects", effects),
        ],
    )
}

fn pad_layers(board: &Board, pad: &Pad) -> SExpr {
    let n = board.copper_layers.len();
    let all = board.all_layers();
    let mut toks: Vec<String> = Vec::new();
    let copper = pad.layers.0 & all.0;
    if copper == all.0 && copper != 0 {
        toks.push("*.Cu".into());
    } else {
        for l in LayerSet(copper).iter() {
            if let Some(c) = board.copper_layers.get(l.0 as usize) {
                toks.push(c.name.clone());
            }
        }
    }
    let smd = pad.drill.is_none();
    let front = copper & 1 != 0;
    let back = n > 0 && copper & (1u64 << (n - 1)) != 0;
    if copper == 0 || (front && back && !smd) {
        toks.push("*.Mask".into());
    } else {
        if front {
            if smd {
                toks.push("F.Paste".into());
            }
            toks.push("F.Mask".into());
        }
        if back {
            if smd {
                toks.push("B.Paste".into());
            }
            toks.push("B.Mask".into());
        }
    }
    node("layers", toks.into_iter().map(SExpr::quoted).collect())
}

fn pad_node(board: &Board, fp: &Footprint, pad: &Pad, ids: &mut Ids) -> SExpr {
    let kind = match pad.drill {
        Some(d) if d.plated => "thru_hole",
        Some(_) => "np_thru_hole",
        None => "smd",
    };
    let abs = fp.placement.rot.add(pad.local_rot).degrees();
    let two = |k: &str, a: Coord, b: Coord| {
        node(k, vec![SExpr::sym(format_mm(a)), SExpr::sym(format_mm(b))])
    };
    let mut extra_after: Vec<SExpr> = Vec::new();
    let (shape_name, size, rratio) = match &pad.shape {
        PadShape::Circle { diameter } => ("circle", (*diameter, *diameter), None),
        PadShape::Rect { w, h } => ("rect", (*w, *h), None),
        PadShape::Oval { w, h } => ("oval", (*w, *h), None),
        PadShape::RoundRect { w, h, radius } => {
            let m = (*w).min(*h);
            #[allow(clippy::cast_precision_loss)]
            let ratio = if m > 0 { *radius as f64 / m as f64 } else { 0.0 };
            ("roundrect", (*w, *h), Some(ratio))
        }
        PadShape::Convex { pts } => {
            // A custom pad with a ~1 um circular anchor and one filled polygon primitive.
            extra_after
                .push(node("options", vec![kv("clearance", "outline"), kv("anchor", "circle")]));
            extra_after.push(node(
                "primitives",
                vec![node("gr_poly", vec![pts_node(pts), kv("width", "0"), kv("fill", "yes")])],
            ));
            ("custom", (1000, 1000), None)
        }
    };
    let mut v = vec![
        SExpr::sym("pad"),
        SExpr::quoted(pad.number.clone()),
        SExpr::sym(kind),
        SExpr::sym(shape_name),
        at_node(pad.local_pos, Some(abs), false),
        two("size", size.0, size.1),
    ];
    if let Some(d) = pad.drill {
        v.push(kv("drill", format_mm(d.diameter)));
    }
    v.push(pad_layers(board, pad));
    if let Some(r) = rratio {
        v.push(kv("roundrect_rratio", format!("{r}")));
    }
    if pad.net != ferroute_board::NetId::NONE
        && let Some(n) = board.nets.get(pad.net.idx())
    {
        v.push(kq("net", n.name.clone()));
    }
    v.extend(extra_after);
    v.push(ids.next());
    SExpr::List(v)
}

fn footprint_node(board: &Board, fp: &Footprint, ids: &mut Ids) -> SExpr {
    let back = fp.side == Side::Back;
    let (cu, silk, fab, crt) = if back {
        ("B.Cu", "B.SilkS", "B.Fab", "B.CrtYd")
    } else {
        ("F.Cu", "F.SilkS", "F.Fab", "F.CrtYd")
    };
    let rot = fp.placement.rot.degrees();
    let mut v = vec![SExpr::sym("footprint"), SExpr::quoted(fp.lib_id.clone())];
    if fp.locked {
        v.push(kv("locked", "yes"));
    }
    v.push(kq("layer", cu));
    v.push(ids.next());
    v.push(at_node(fp.placement.pos, Some(rot), false));
    let cy = fp.courtyard_local.bbox().unwrap_or(Rect::new(Point::ORIGIN, Point::ORIGIN));
    v.push(text_property(
        ids,
        "Reference",
        &fp.reference,
        Point::new(0, cy.min.y - 1_000_000),
        rot,
        silk,
        back,
    ));
    v.push(text_property(ids, "Value", "", Point::new(0, cy.max.y + 1_000_000), rot, fab, back));
    if fp.courtyard_local.pts.len() >= 3 {
        v.push(node(
            "fp_poly",
            vec![
                pts_node(&fp.courtyard_local.pts),
                stroke(50_000),
                kv("fill", "no"),
                kq("layer", crt),
                ids.next(),
            ],
        ));
    }
    for &pid in &fp.pads {
        v.push(pad_node(board, fp, board.pad(pid), ids));
    }
    SExpr::List(v)
}

fn polygon_pts(poly: &Polygon) -> SExpr {
    pts_node(&poly.pts)
}

fn keepout_node(board: &Board, k: &ferroute_board::Keepout, i: usize, ids: &mut Ids) -> SExpr {
    let names: Vec<SExpr> = k
        .layers
        .iter()
        .filter_map(|l| board.copper_layers.get(l.0 as usize))
        .map(|c| SExpr::quoted(c.name.clone()))
        .collect();
    let layer = if names.len() == 1 { node("layer", names) } else { node("layers", names) };
    let ok = |b: bool| if b { "not_allowed" } else { "allowed" };
    node(
        "zone",
        vec![
            layer,
            ids.next(),
            kq("name", format!("keepout_{i}")),
            node("hatch", vec![SExpr::sym("edge"), SExpr::sym("0.5")]),
            node("connect_pads", vec![kv("clearance", "0")]),
            kv("min_thickness", "0.25"),
            node(
                "keepout",
                vec![
                    kv("tracks", ok(k.no_tracks)),
                    kv("vias", ok(k.no_vias)),
                    kv("pads", ok(k.no_pads)),
                    kv("copperpour", "allowed"),
                    kv("footprints", ok(k.no_footprints)),
                ],
            ),
            node("placement", vec![kv("enabled", "no")]),
            node("fill", vec![kv("thermal_gap", "0.5"), kv("thermal_bridge_width", "0.5")]),
            node("polygon", vec![polygon_pts(&k.area)]),
        ],
    )
}

fn outline_poly(poly: &Polygon, ids: &mut Ids) -> SExpr {
    node(
        "gr_poly",
        vec![
            polygon_pts(poly),
            stroke(50_000),
            kv("fill", "no"),
            kq("layer", "Edge.Cuts"),
            ids.next(),
        ],
    )
}

/// Builds the S-expression tree of a complete KiCad 10 board.
fn build(board: &Board, seed: u64) -> SExpr {
    let mut ids = Ids { seed, n: 0 };
    let mut root = vec![
        SExpr::sym("kicad_pcb"),
        kv("version", KICAD10_VERSION.to_string()),
        kq("generator", "ferroute"),
        kq("generator_version", env!("CARGO_PKG_VERSION")),
        node("general", vec![kv("thickness", "1.6"), kv("legacy_teardrops", "no")]),
        kq("paper", "A4"),
        layers_table(board),
        setup_node(),
    ];
    for fp in &board.footprints {
        root.push(footprint_node(board, fp, &mut ids));
    }
    root.push(outline_poly(&board.outline.outer, &mut ids));
    for h in &board.outline.holes {
        root.push(outline_poly(h, &mut ids));
    }
    let dialect = NetDialect::Names;
    for (i, t) in board.tracks.iter().enumerate() {
        if let Ok(n) = track_node(&dialect, board, t, seed, i as u64) {
            root.push(n);
        }
    }
    for (i, a) in board.arcs.iter().enumerate() {
        if let Ok(n) = arc_node(&dialect, board, a, seed, i as u64) {
            root.push(n);
        }
    }
    for (i, v) in board.vias.iter().enumerate() {
        if let Ok(n) = via_node(&dialect, board, v, seed, i as u64, KICAD10_VERSION) {
            root.push(n);
        }
    }
    for (i, k) in board.keepouts.iter().enumerate() {
        root.push(keepout_node(board, k, i, &mut ids));
    }
    root.push(kv("embedded_fonts", "no"));
    SExpr::List(root)
}

/// Writes a complete KiCad 10 board (`version 20260206`, generator `ferroute`) from
/// scratch: layer table with KiCad 9+ ids, footprints with courtyard, pads and nets by
/// name (KiCad 10 has no net table), Edge.Cuts outline as `gr_poly`, tracks, vias and
/// keepouts as rule-area zones. Deterministic in `(board, seed)`.
///
/// Net classes and design rules live in the `.kicad_pro`; see
/// [`crate::project::write_project_json`].
#[must_use]
pub fn write_new_board(board: &Board, seed: u64) -> String {
    build(board, seed).serialize()
}
