// SPDX-License-Identifier: AGPL-3.0-only
//! `.kicad_pcb` S-expression tree to [`Board`].

use crate::KicadError;
use crate::layers::Layers;
use crate::outline::{EdgeBuilder, Gfx, parse_gfx, point_child, poly_points};
use crate::pad::{build_shape, offset_shape, shape_offset};
use crate::sexpr::SExpr;
use crate::util::{circumscribed_polygon, float, mm, same_angle};
use ferroute_board::constraints::{Dielectric, Stackup};
use ferroute_board::{
    Board, Drill, Footprint, FootprintId, IslandRemoval, Keepout, LayerId, LayerSet, Mobility, Net,
    NetId, Origin, Pad, PadConnection, PadId, Placement, Side, Track, Via, ViaKind, Zone, ZoneFill,
};
use ferroute_geom::{Angle, Coord, Point, Polygon, Rect, convex_hull, units::mm_to_nm};
use std::collections::{BTreeMap, HashMap};

/// Courtyard margin around the pad bounding box when a footprint has no courtyard (nm).
const FALLBACK_COURTYARD_MARGIN: Coord = 250_000;

/// How nets are referenced from items.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum NetDialect {
    /// KiCad 10 (file version >= 20251028): items reference nets by name, no net table.
    Names,
    /// KiCad <= 9: top-level `(net N "name")` table; items carry the numeric code.
    Codes(BTreeMap<String, i64>),
}

pub(crate) struct Parsed {
    pub board: Board,
    pub version: u32,
    pub dialect: NetDialect,
    /// Index in the root list of every footprint, in `Board::footprints` order.
    pub fp_nodes: Vec<usize>,
    /// Root-list index of the source node of every `Board::tracks` entry (an arc node
    /// appears once per chord).
    pub track_nodes: Vec<usize>,
    /// Root-list index of the source node of every `Board::vias` entry.
    pub via_nodes: Vec<usize>,
    pub warnings: Vec<String>,
}

struct Reader {
    layers: Layers,
    nets: Vec<Net>,
    by_name: HashMap<String, NetId>,
    by_code: HashMap<i64, NetId>,
    warnings: Vec<String>,
    board: Board,
    edge: EdgeBuilder,
}

fn is_locked(node: &SExpr) -> bool {
    node.items().unwrap_or(&[]).iter().skip(1).any(|c| {
        c.is_sym("locked") || (c.head() == Some("locked") && c.atom(1).is_none_or(|v| v != "no"))
    })
}

fn num_at(node: &SExpr, i: usize) -> Result<Coord, KicadError> {
    node.atom(i)
        .ok_or_else(|| {
            KicadError::Format(format!("({}) is missing value #{i}", node.head().unwrap_or("?")))
        })
        .and_then(mm)
}

impl Reader {
    fn net_new(&mut self, name: &str) -> NetId {
        let id = NetId(u32::try_from(self.nets.len()).unwrap_or(u32::MAX));
        self.nets.push(Net::new(name.to_owned(), 0));
        self.by_name.insert(name.to_owned(), id);
        id
    }

    /// Resolves a `(net ...)` node in either dialect.
    fn resolve_net(&mut self, node: &SExpr) -> NetId {
        let items = node.items().unwrap_or(&[]);
        let first = items.get(1);
        let code = first.and_then(|f| match f {
            SExpr::Atom(a) if !a.quoted => a.text.parse::<i64>().ok(),
            _ => None,
        });
        if let Some(code) = code {
            if code == 0 {
                return NetId::NONE;
            }
            if let Some(&id) = self.by_code.get(&code) {
                return id;
            }
            if let Some(name) = items.get(2).and_then(SExpr::text).filter(|n| !n.is_empty()) {
                let name = name.to_owned();
                let id = self.by_name.get(&name).copied().unwrap_or_else(|| self.net_new(&name));
                self.by_code.insert(code, id);
                return id;
            }
            self.warnings
                .push(format!("reference to unknown net code {code}, treated as unconnected"));
            return NetId::NONE;
        }
        match first.and_then(SExpr::text) {
            None | Some("") => NetId::NONE,
            Some(name) => {
                let name = name.to_owned();
                self.by_name.get(&name).copied().unwrap_or_else(|| self.net_new(&name))
            }
        }
    }

    fn net_of(&mut self, node: &SExpr) -> NetId {
        match node.child("net") {
            Some(n) => self.resolve_net(n),
            None => NetId::NONE,
        }
    }

    fn read_pad(
        &mut self,
        fp: FootprintId,
        fp_rot: f64,
        reference: &str,
        node: &SExpr,
    ) -> Result<Pad, KicadError> {
        let number = node.atom(1).unwrap_or("").to_owned();
        let label = format!("{reference} pad {number}");
        let kind = node.atom(2).unwrap_or("smd");
        let at =
            node.child("at").ok_or_else(|| KicadError::Format(format!("{label} has no (at)")))?;
        let local_pos = Point::new(num_at(at, 1)?, num_at(at, 2)?);
        // KiCad stores the pad's ABSOLUTE orientation (footprint rotation included).
        let abs = at.atom(3).map(float).transpose()?.unwrap_or(0.0);
        let local_rot =
            if same_angle(abs, fp_rot) { Angle::ZERO } else { Angle::from_degrees(abs - fp_rot) };

        let shape = build_shape(node, &mut self.warnings, &label)?;
        let shape = offset_shape(&shape, shape_offset(node)?);

        let layers = self.layers.mask_of(node);
        let drill = match node.child("drill") {
            Some(d) => {
                let items = d.items().unwrap_or(&[]);
                let nums: Vec<Coord> = items
                    .iter()
                    .skip(1)
                    .filter(|n| n.text().is_some_and(|t| mm(t).is_ok()) && n.items().is_none())
                    .filter_map(|n| n.text().and_then(|t| mm(t).ok()))
                    .collect();
                let dia = nums.iter().copied().max().unwrap_or(0);
                (dia > 0).then_some(Drill { diameter: dia, plated: kind != "np_thru_hole" })
            }
            None => None,
        };
        let net = self.net_of(node);
        Ok(Pad { footprint: fp, number, net, local_pos, local_rot, shape, layers, drill })
    }

    fn read_zone(
        &mut self,
        node: &SExpr,
        pl: Option<Placement>,
        owner: &str,
    ) -> Result<(), KicadError> {
        let layers = self.layers.mask_of(node);
        let Some(ko) = node.child("keepout") else {
            return self.read_copper_zone(node, pl.is_some(), owner, layers);
        };
        let denied = |k: &str| ko.child(k).and_then(|c| c.atom(1)) == Some("not_allowed");
        let (no_tracks, no_vias, no_pads, no_footprints) =
            (denied("tracks"), denied("vias"), denied("pads"), denied("footprints"));
        if !(no_tracks || no_vias || no_pads || no_footprints) || layers.is_empty() {
            return Ok(());
        }
        let Some(poly) = node.child("polygon").and_then(|p| p.child("pts")) else {
            return Err(KicadError::Format("rule area without (polygon (pts ...))".into()));
        };
        let mut pts = poly_points(poly)?;
        if let Some(pl) = pl {
            pts = pts.into_iter().map(|p| pl.apply(p)).collect();
            self.warnings.push(format!(
                "rule area inside {owner} is treated as static at the footprint's original placement"
            ));
        }
        if pts.len() >= 3 {
            self.board.keepouts.push(Keepout {
                area: Polygon::new(pts),
                layers,
                no_tracks,
                no_vias,
                no_pads,
                no_footprints,
            });
        }
        Ok(())
    }

    /// A copper pour: outline, rules and the stored `(filled_polygon ...)` copper.
    fn read_copper_zone(
        &mut self,
        node: &SExpr,
        in_footprint: bool,
        owner: &str,
        layers: LayerSet,
    ) -> Result<(), KicadError> {
        if in_footprint {
            self.warnings.push(format!(
                "copper zone inside {owner} ignored for routing; footprint-embedded pours are not modelled"
            ));
            return Ok(());
        }
        let Some(outline) = node.child("polygon").and_then(|p| p.child("pts")) else {
            return Err(KicadError::Format("copper zone without (polygon (pts ...))".into()));
        };
        let outline = Polygon::new(poly_points(outline)?);
        let net = self.net_of(node);
        let connect = node.child("connect_pads");
        let pad_connection = match connect.and_then(|c| c.atom(1)) {
            Some("yes") => PadConnection::Solid,
            Some("no") => PadConnection::None,
            Some("thru_hole_only") => PadConnection::ThruHoleOnly,
            _ => PadConnection::ThermalRelief,
        };
        let clearance = match connect.and_then(|c| c.child("clearance")) {
            Some(c) => num_at(c, 1)?,
            None => 0,
        };
        let min_thickness = match node.child("min_thickness") {
            Some(c) => num_at(c, 1)?,
            None => 0,
        };
        let priority =
            node.child("priority").and_then(|c| c.atom(1)).map_or(0, |t| t.parse().unwrap_or(0));
        let mut fills: Vec<ZoneFill> = Vec::new();
        for f in node.children("filled_polygon") {
            let Some(layer) = self.layers.mask_of(f).iter().next() else { continue };
            let Some(pts) = f.child("pts") else { continue };
            let poly = Polygon::new(poly_points(pts)?);
            if poly.pts.len() < 3 {
                continue;
            }
            match fills.iter_mut().find(|z| z.layer == layer) {
                Some(z) => z.polys.push(poly),
                None => fills.push(ZoneFill { layer, polys: vec![poly] }),
            }
        }
        let islands = match node.child("fill") {
            Some(f) => match f.child("island_removal_mode").and_then(|c| c.atom(1)) {
                Some("1") => IslandRemoval::Never,
                Some("2") => {
                    let mm2: f64 = f
                        .child("island_area_min")
                        .and_then(|c| c.atom(1))
                        .and_then(|t| t.parse().ok())
                        .unwrap_or(10.0);
                    #[allow(clippy::cast_possible_truncation)]
                    IslandRemoval::BelowArea((mm2 * 1e12).round() as i128)
                }
                _ => IslandRemoval::Always,
            },
            None => IslandRemoval::Always,
        };
        let zone = Zone {
            net,
            layers,
            outline,
            fills,
            clearance,
            min_thickness,
            priority,
            pad_connection,
            islands,
            yields: false,
        };
        if zone.is_unfilled() {
            let name = self.nets.get(net.idx()).map_or("", |n| n.name.as_str());
            self.warnings
                .push(format!("zone {name:?} has no fill; refill in KiCad before routing"));
        }
        self.board.zones.push(zone);
        Ok(())
    }

    #[allow(clippy::too_many_lines)]
    fn read_footprint(&mut self, node: &SExpr) -> Result<(), KicadError> {
        let lib_id = node.atom(1).unwrap_or("").to_owned();
        let id = FootprintId(u32::try_from(self.board.footprints.len()).unwrap_or(u32::MAX));
        let locked = is_locked(node);
        let side = match node.child("layer").and_then(|l| l.atom(1)) {
            Some("B.Cu") => Side::Back,
            _ => Side::Front,
        };
        let at = node
            .child("at")
            .ok_or_else(|| KicadError::Format(format!("footprint {lib_id} has no (at)")))?;
        let pos = Point::new(num_at(at, 1)?, num_at(at, 2)?);
        let rot_deg = at.atom(3).map(float).transpose()?.unwrap_or(0.0);
        let placement = Placement { pos, rot: Angle::from_degrees(rot_deg) };

        let reference = node
            .children("property")
            .find(|p| p.atom(1) == Some("Reference"))
            .and_then(|p| p.atom(2))
            .or_else(|| {
                node.children("fp_text")
                    .find(|t| t.atom(1) == Some("reference"))
                    .and_then(|t| t.atom(2))
            })
            .unwrap_or("")
            .to_owned();

        let mut pad_ids: Vec<PadId> = Vec::new();
        for p in node.children("pad") {
            let pad = self.read_pad(id, rot_deg, &reference, p)?;
            pad_ids.push(PadId(u32::try_from(self.board.pads.len()).unwrap_or(u32::MAX)));
            self.board.pads.push(pad);
        }

        // Courtyard (any side) and Edge.Cuts shapes inside the footprint.
        let mut court: Vec<Point> = Vec::new();
        for g in node.items().unwrap_or(&[]).iter().skip(1) {
            let Some(h) = g.head() else { continue };
            if !h.starts_with("fp_") {
                continue;
            }
            let layer = g.child("layer").and_then(|l| l.atom(1)).unwrap_or("");
            match layer {
                "F.CrtYd" | "B.CrtYd" => collect_courtyard(g, &mut court)?,
                "Edge.Cuts" => {
                    if let Some(gfx) = parse_gfx(g)? {
                        self.edge.add(&gfx, Some(placement));
                    }
                }
                _ => {}
            }
        }
        let courtyard_local = if court.len() >= 3 {
            Polygon::new(convex_hull(&court))
        } else {
            let mut bb: Option<Rect> = None;
            for &pid in &pad_ids {
                let p = &self.board.pads[pid.idx()];
                let b = p
                    .hull(Placement { pos: Point::ORIGIN, rot: Angle::ZERO })
                    .bbox()
                    .translate(Point::ORIGIN);
                bb = Some(bb.map_or(b, |a| a.union(b)));
            }
            let r = bb.unwrap_or(Rect::new(Point::ORIGIN, Point::ORIGIN));
            Polygon::from_rect(r.inflate(FALLBACK_COURTYARD_MARGIN))
        };

        for z in node.children("zone") {
            self.read_zone(z, Some(placement), &reference)?;
        }

        self.board.footprints.push(Footprint {
            reference,
            lib_id,
            placement,
            original: placement,
            side,
            locked,
            mobility: if locked { Mobility::Locked } else { Mobility::Fixed },
            courtyard_local,
            pads: pad_ids,
        });
        Ok(())
    }

    fn read_track(&mut self, node: &SExpr) -> Result<(), KicadError> {
        let head = node.head().unwrap_or("");
        let width = num_at(
            node.child("width")
                .ok_or_else(|| KicadError::Format(format!("({head}) without width")))?,
            1,
        )?;
        let layer_name = node.child("layer").and_then(|l| l.atom(1)).unwrap_or("");
        let Some(layer) = self.layers.id(layer_name) else {
            self.warnings.push(format!("({head}) on non-copper layer {layer_name:?} ignored"));
            return Ok(());
        };
        let net = self.net_of(node);
        let locked = is_locked(node);
        let push = |board: &mut Board, a: Point, b: Point, w: Coord| {
            board.tracks.push(Track {
                start: a,
                end: b,
                width: w,
                layer,
                net,
                locked,
                origin: Origin::Existing,
            });
        };
        let (s, e) = (point_child(node, "start")?, point_child(node, "end")?);
        if head == "arc" {
            let m = point_child(node, "mid")?;
            let (pts, sag) = crate::outline::flatten_arc(s, m, e, 0.0);
            // Chords stay within `sag` of the true arc: widen the stroke accordingly.
            #[allow(clippy::cast_possible_truncation)]
            let w = width + 2 * (sag.ceil() as Coord + 1);
            for pair in pts.windows(2) {
                push(&mut self.board, pair[0], pair[1], if pts.len() > 2 { w } else { width });
            }
        } else {
            push(&mut self.board, s, e, width);
        }
        Ok(())
    }

    fn read_via(&mut self, node: &SExpr) -> Result<(), KicadError> {
        let at = point_child(node, "at")?;
        let mut diameter = num_at(
            node.child("size").ok_or_else(|| KicadError::Format("via without size".into()))?,
            1,
        )?;
        // KiCad 9+ padstack vias may have larger inner/back copper: take the maximum.
        for l in node.child("padstack").map(|p| p.children("layer")).into_iter().flatten() {
            if let Some(s) = l.child("size").and_then(|s| s.atom(1)) {
                diameter = diameter.max(mm(s)?);
            }
        }
        let drill = node.child("drill").and_then(|d| d.atom(1)).map(mm).transpose()?.unwrap_or(0);
        let names: Vec<&str> = node
            .child("layers")
            .map(|l| l.items().unwrap_or(&[]).iter().skip(1).filter_map(SExpr::text).collect())
            .unwrap_or_default();
        let ids: Vec<LayerId> = names.iter().filter_map(|n| self.layers.id(n)).collect();
        let (Some(&a), Some(&b)) = (ids.first(), ids.last()) else {
            self.warnings.push("via without copper layers ignored".into());
            return Ok(());
        };
        let net = self.net_of(node);
        // KiCad writes `blind` / `buried` / `micro` right after `(via`; through vias have none.
        let kind = node
            .items()
            .unwrap_or(&[])
            .iter()
            .skip(1)
            .find_map(|c| c.text().and_then(ViaKind::parse))
            .unwrap_or_default();
        self.board.vias.push(Via {
            pos: at,
            diameter,
            drill,
            layers: LayerSet::span(a, b),
            kind,
            net,
            locked: is_locked(node),
            origin: Origin::Existing,
        });
        Ok(())
    }
}

/// Points of a courtyard graphic (a superset of the drawn shape).
fn collect_courtyard(g: &SExpr, out: &mut Vec<Point>) -> Result<(), KicadError> {
    match parse_gfx(g)? {
        Some(Gfx::Line(a, b)) => out.extend([a, b]),
        Some(Gfx::Rect(a, b)) => out.extend(Rect::new(a, b).corners()),
        Some(Gfx::Poly(v)) => out.extend(v),
        Some(Gfx::Circle(c, e)) => {
            #[allow(clippy::cast_precision_loss)]
            out.extend(circumscribed_polygon(c.x as f64, c.y as f64, c.dist(e), 16));
        }
        Some(Gfx::Arc(s, m, e)) => out.extend(crate::outline::flatten_arc(s, m, e, 0.0).0),
        Some(Gfx::ArcLegacy(c, s)) => {
            #[allow(clippy::cast_precision_loss)]
            out.extend(circumscribed_polygon(c.x as f64, c.y as f64, c.dist(s), 16));
        }
        Some(Gfx::Curve(c)) => out.extend(c),
        None => {}
    }
    Ok(())
}

/// Legacy (pre-KiCad 6) `(net_class ...)` blocks inside the board file.
fn legacy_net_classes(root: &SExpr, board: &mut Board) {
    let mut classes: Vec<(ferroute_board::NetClass, Vec<String>)> = Vec::new();
    for nc in root.children("net_class") {
        let mut c = ferroute_board::NetClass {
            name: nc.atom(1).unwrap_or("Default").to_owned(),
            ..Default::default()
        };
        let val = |k: &str| nc.child(k).and_then(|n| n.atom(1)).and_then(|v| v.parse::<f64>().ok());
        if let Some(v) = val("clearance") {
            c.clearance = mm_to_nm(v);
        }
        if let Some(v) = val("trace_width") {
            c.track_width = mm_to_nm(v);
        }
        if let Some(v) = val("via_dia") {
            c.via_diameter = mm_to_nm(v);
        }
        if let Some(v) = val("via_drill") {
            c.via_drill = mm_to_nm(v);
        }
        let nets = nc.children("add_net").filter_map(|n| n.atom(1)).map(str::to_owned).collect();
        classes.push((c, nets));
    }
    if classes.is_empty() {
        return;
    }
    // The class called "Default" is class 0.
    if let Some(pos) = classes.iter().position(|(c, _)| c.name == "Default") {
        classes.swap(0, pos);
    }
    board.rules.classes = classes.iter().map(|(c, _)| c.clone()).collect();
    for (i, (_, nets)) in classes.iter().enumerate() {
        for n in nets {
            if let Some(id) = board.net_by_name(n) {
                board.nets[id.idx()].class = u16::try_from(i).unwrap_or(0);
            }
        }
    }
}

/// Parses `(setup (stackup …))` into a [`Stackup`]; `None` when absent. Sub-layers of one
/// dielectric (`addsublayer`) are merged (thickness summed, εr thickness-weighted); mask,
/// paste and silk entries are ignored. Anything inconsistent with `n_copper` copper layers
/// yields a warning and `None` (defaults stay).
fn read_stackup(
    root: &SExpr,
    n_copper: usize,
    board_thickness: Option<Coord>,
    warnings: &mut Vec<String>,
) -> Option<Stackup> {
    let node = root.child("setup")?.child("stackup")?;
    let mut copper = Vec::new();
    let mut dielectrics: Vec<Dielectric> = Vec::new();
    let mut after_copper = false;
    for layer in node.children("layer") {
        let name = layer.atom(1).unwrap_or("");
        let ty = layer.child("type").and_then(|t| t.atom(1)).unwrap_or("");
        // Items with sub-layers repeat `(thickness)`/`(epsilon_r)` after `addsublayer`.
        let mut parts: Vec<(Option<Coord>, Option<f64>)> = vec![(None, None)];
        for item in layer.items().unwrap_or(&[]).iter().skip(2) {
            if item.is_sym("addsublayer") {
                parts.push((None, None));
                continue;
            }
            let Some(cur) = parts.last_mut() else { continue };
            match item.head() {
                Some("thickness") => cur.0 = item.atom(1).and_then(|v| mm(v).ok()),
                Some("epsilon_r") => cur.1 = item.atom(1).and_then(|v| v.parse::<f64>().ok()),
                _ => {}
            }
        }
        if ty == "copper" || name.ends_with(".Cu") {
            if !after_copper && !copper.is_empty() {
                warnings.push("stackup: two copper layers without a dielectric; ignored".into());
                return None;
            }
            copper.push(parts[0].0.unwrap_or(0));
            after_copper = false;
        } else if matches!(ty, "core" | "prepreg") || name.starts_with("dielectric") {
            if after_copper || copper.is_empty() {
                warnings.push("stackup: consecutive dielectrics; ignored".into());
                return None;
            }
            let mut total: Coord = 0;
            let mut weighted = 0.0_f64;
            for (t, e) in &parts {
                let t = t.unwrap_or(0);
                total += t;
                #[allow(clippy::cast_precision_loss)]
                {
                    weighted += t as f64 * e.unwrap_or(4.3);
                }
            }
            #[allow(
                clippy::cast_precision_loss,
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss
            )]
            let er_milli =
                if total > 0 { (weighted / total as f64 * 1000.0).round() as u32 } else { 0 };
            dielectrics.push(Dielectric {
                thickness: total,
                epsilon_r_milli: er_milli,
                between: (LayerId(0), LayerId(0)),
            });
            after_copper = true;
        }
    }
    if copper.len() != n_copper {
        warnings.push(format!(
            "stackup lists {} copper layers but the board has {n_copper}; using defaults",
            copper.len()
        ));
        return None;
    }
    let s = Stackup::from_layers(copper, dielectrics, board_thickness);
    if s.is_none() {
        warnings.push("stackup is malformed (missing thickness or εr < 1); using defaults".into());
    }
    s
}

pub(crate) fn parse_board(root: &SExpr) -> Result<Parsed, KicadError> {
    if root.head() != Some("kicad_pcb") {
        return Err(KicadError::Format(
            "not a kicad_pcb file (top-level list is not kicad_pcb)".into(),
        ));
    }
    let version = root
        .child("version")
        .and_then(|v| v.atom(1))
        .and_then(|v| v.parse::<u32>().ok())
        .unwrap_or(0);
    let layers = Layers::from_root(root)?;
    let mut r = Reader {
        layers: layers.clone(),
        nets: vec![Net::new(String::new(), 0)],
        by_name: HashMap::new(),
        by_code: HashMap::new(),
        warnings: Vec::new(),
        board: Board::default(),
        edge: EdgeBuilder::default(),
    };
    // Net table (KiCad <= 9). Items reference it by code.
    let mut table: BTreeMap<i64, String> = BTreeMap::new();
    for n in root.children("net") {
        if let (Some(code), Some(name)) = (n.atom(1).and_then(|c| c.parse::<i64>().ok()), n.atom(2))
        {
            table.insert(code, name.to_owned());
        }
    }
    let dialect = if table.is_empty() {
        NetDialect::Names
    } else {
        let mut names = BTreeMap::new();
        r.by_code.insert(0, NetId::NONE);
        for (&code, name) in &table {
            names.insert(name.clone(), code);
            if code == 0 || name.is_empty() {
                continue;
            }
            let id = r.net_new(name);
            r.by_code.insert(code, id);
        }
        NetDialect::Codes(names)
    };

    let mut fp_nodes = Vec::new();
    for (i, child) in root.items().unwrap_or(&[]).iter().enumerate() {
        if child.head() == Some("footprint") && child.items().is_some() {
            fp_nodes.push(i);
            r.read_footprint(child)?;
        }
    }
    let (mut track_nodes, mut via_nodes) = (Vec::new(), Vec::new());
    for (i, child) in root.items().unwrap_or(&[]).iter().enumerate().skip(1) {
        match child.head() {
            Some("segment" | "arc") => {
                r.read_track(child)?;
                track_nodes.resize(r.board.tracks.len(), i);
            }
            Some("via") => {
                r.read_via(child)?;
                via_nodes.resize(r.board.vias.len(), i);
            }
            Some("zone") => r.read_zone(child, None, "")?,
            Some(h) if h.starts_with("gr_") => {
                if child.child("layer").and_then(|l| l.atom(1)) == Some("Edge.Cuts")
                    && let Some(g) = parse_gfx(child)?
                {
                    r.edge.add(&g, None);
                }
            }
            _ => {}
        }
    }

    let (outline, edge_warn) = std::mem::take(&mut r.edge).finish()?;
    r.warnings.extend(edge_warn);
    let mut board = r.board;
    board.copper_layers = layers.copper;
    board.nets = r.nets;
    board.outline = outline;
    legacy_net_classes(root, &mut board);
    let general_t = root
        .child("general")
        .and_then(|g| g.child("thickness"))
        .and_then(|t| t.atom(1))
        .and_then(|v| mm(v).ok())
        .filter(|&t| t > 0);
    if let Some(s) = read_stackup(root, board.copper_layers.len(), general_t, &mut r.warnings) {
        board.stackup = s;
    } else if let Some(t) = general_t {
        board.stackup.board_thickness = t;
    }
    board.validate().map_err(|e| KicadError::Format(e.to_string()))?;
    Ok(Parsed { board, version, dialect, fp_nodes, track_nodes, via_nodes, warnings: r.warnings })
}
