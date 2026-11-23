// SPDX-License-Identifier: AGPL-3.0-only
//! Programmatic board construction for tests, benchmarks and synthetic generators.

use crate::{
    Board, CopperLayer, Drill, Footprint, FootprintId, Keepout, LayerId, LayerSet, Mobility, Net,
    NetId, Origin, Outline, Pad, PadId, PadShape, Placement, Side, Track, Via,
};
use ferroute_geom::{Angle, Coord, Point, Polygon, Rect};

/// One pad of a footprint being built.
#[derive(Clone, Debug)]
pub struct PadSpec {
    pub number: String,
    pub local_pos: Point,
    pub shape: PadShape,
    /// `None` = SMD on the footprint's side; `Some(d)` = plated through-hole on all layers.
    pub drill: Option<Coord>,
    pub net: NetId,
}

/// Fluent builder for [`Board`].
#[derive(Debug)]
pub struct BoardBuilder {
    board: Board,
}

/// Canonical KiCad copper layer names for an `n`-layer stack.
#[must_use]
pub fn copper_layer_names(n: u8) -> Vec<String> {
    let n = n.max(1);
    let mut v = vec!["F.Cu".to_owned()];
    for i in 1..n.saturating_sub(1) {
        v.push(format!("In{i}.Cu"));
    }
    if n > 1 {
        v.push("B.Cu".to_owned());
    }
    v
}

impl BoardBuilder {
    /// Rectangular board `(0,0)–(w,h)` with `layers` copper layers and default rules.
    #[must_use]
    pub fn new(w: Coord, h: Coord, layers: u8) -> Self {
        let board = Board {
            copper_layers: copper_layer_names(layers)
                .into_iter()
                .map(|name| CopperLayer { name, routable: true })
                .collect(),
            nets: vec![Net::new(String::new(), 0)],
            outline: Outline {
                outer: Polygon::from_rect(Rect::new(Point::ORIGIN, Point::new(w, h))),
                holes: vec![],
            },
            ..Board::default()
        };
        Self { board }
    }

    /// Mutable access to the rules.
    pub fn rules_mut(&mut self) -> &mut crate::DesignRules {
        &mut self.board.rules
    }

    /// Returns the id of net `name`, creating it if needed.
    pub fn net(&mut self, name: &str) -> NetId {
        if let Some(id) = self.board.net_by_name(name) {
            return id;
        }
        self.board.nets.push(Net::new(name.to_owned(), 0));
        NetId(u32::try_from(self.board.nets.len() - 1).unwrap_or(u32::MAX))
    }

    /// Adds a footprint with the given pads. The courtyard is the pad bounding box
    /// inflated by 0.25 mm (KiCad's IPC-7351 nominal courtyard excess).
    pub fn footprint(
        &mut self,
        reference: &str,
        pos: Point,
        rot_deg: f64,
        side: Side,
        pads: &[PadSpec],
    ) -> FootprintId {
        let fid = FootprintId(u32::try_from(self.board.footprints.len()).unwrap_or(u32::MAX));
        let placement = Placement { pos, rot: Angle::from_degrees(rot_deg) };
        let n_layers = self.board.layer_count();
        let mut pad_ids = Vec::with_capacity(pads.len());
        let mut bbox: Option<Rect> = None;
        for spec in pads {
            let layers = match (spec.drill, side) {
                (Some(_), _) => LayerSet::all(n_layers),
                (None, Side::Front) => LayerSet::single(LayerId(0)),
                (None, Side::Back) => LayerSet::single(LayerId(n_layers - 1)),
            };
            let pad = Pad {
                footprint: fid,
                number: spec.number.clone(),
                net: spec.net,
                local_pos: spec.local_pos,
                local_rot: Angle::ZERO,
                shape: spec.shape.clone(),
                layers,
                drill: spec.drill.map(|d| Drill { diameter: d, plated: true }),
            };
            let local_bb = pad.shape.local_hull().bbox().translate(spec.local_pos);
            bbox = Some(bbox.map_or(local_bb, |b| b.union(local_bb)));
            pad_ids.push(PadId(u32::try_from(self.board.pads.len()).unwrap_or(u32::MAX)));
            self.board.pads.push(pad);
        }
        let court = bbox.unwrap_or(Rect::new(Point::ORIGIN, Point::ORIGIN)).inflate(250_000);
        self.board.footprints.push(Footprint {
            reference: reference.to_owned(),
            lib_id: String::new(),
            placement,
            original: placement,
            side,
            locked: false,
            mobility: Mobility::Fixed,
            courtyard_local: Polygon::from_rect(court),
            pads: pad_ids,
        });
        fid
    }

    /// A 0603-style two-terminal SMD part (pads 0.9 × 0.95 mm, 1.6 mm pitch).
    pub fn smd2(
        &mut self,
        reference: &str,
        pos: Point,
        rot_deg: f64,
        a: NetId,
        b: NetId,
    ) -> FootprintId {
        let shape = PadShape::RoundRect { w: 900_000, h: 950_000, radius: 225_000 };
        let pads = [
            PadSpec {
                number: "1".into(),
                local_pos: Point::new(-800_000, 0),
                shape: shape.clone(),
                drill: None,
                net: a,
            },
            PadSpec {
                number: "2".into(),
                local_pos: Point::new(800_000, 0),
                shape,
                drill: None,
                net: b,
            },
        ];
        self.footprint(reference, pos, rot_deg, Side::Front, &pads)
    }

    /// A 1×N 2.54 mm through-hole header (1.7 mm pads, 1.0 mm drill), pin 1 square.
    pub fn header(
        &mut self,
        reference: &str,
        pos: Point,
        rot_deg: f64,
        nets: &[NetId],
    ) -> FootprintId {
        let pads: Vec<PadSpec> = nets
            .iter()
            .enumerate()
            .map(|(i, &net)| PadSpec {
                number: format!("{}", i + 1),
                local_pos: Point::new(0, Coord::try_from(i).unwrap_or(0) * 2_540_000),
                shape: if i == 0 {
                    PadShape::Rect { w: 1_700_000, h: 1_700_000 }
                } else {
                    PadShape::Circle { diameter: 1_700_000 }
                },
                drill: Some(1_000_000),
                net,
            })
            .collect();
        self.footprint(reference, pos, rot_deg, Side::Front, &pads)
    }

    /// Marks a footprint's mobility (and KiCad lock flag for `Locked`).
    pub fn mobility(&mut self, fp: FootprintId, m: Mobility) -> &mut Self {
        let f = &mut self.board.footprints[fp.idx()];
        f.mobility = m;
        f.locked = m == Mobility::Locked;
        self
    }

    pub fn track(
        &mut self,
        a: Point,
        b: Point,
        width: Coord,
        layer: u8,
        net: NetId,
        origin: Origin,
    ) -> &mut Self {
        self.board.tracks.push(Track {
            start: a,
            end: b,
            width,
            layer: LayerId(layer),
            net,
            locked: false,
            origin,
        });
        self
    }

    pub fn via(
        &mut self,
        pos: Point,
        diameter: Coord,
        drill: Coord,
        net: NetId,
        origin: Origin,
    ) -> &mut Self {
        let layers = self.board.all_layers();
        self.board.vias.push(Via { pos, diameter, drill, layers, net, locked: false, origin });
        self
    }

    pub fn keepout(&mut self, area: Rect, no_tracks: bool, no_vias: bool) -> &mut Self {
        let layers = self.board.all_layers();
        self.board.keepouts.push(Keepout {
            area: Polygon::from_rect(area),
            layers,
            no_tracks,
            no_vias,
            no_pads: false,
            no_footprints: false,
        });
        self
    }

    #[must_use]
    pub fn build(self) -> Board {
        self.board
    }
}

/// Millimetres → nanometres for literal-heavy test code.
#[must_use]
pub const fn mm(v: i64) -> Coord {
    v * 1_000_000
}

/// Point from millimetre *tenths* (e.g. `pt10(125, 40)` = (12.5 mm, 4.0 mm)).
#[must_use]
pub const fn pt10(x: i64, y: i64) -> Point {
    Point::new(x * 100_000, y * 100_000)
}
