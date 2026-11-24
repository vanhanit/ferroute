// SPDX-License-Identifier: AGPL-3.0-only
//! Specctra SES (routes) to an additive [`BoardEdit`].

use crate::DsnError;
use crate::sexp::{Node, parse};
use crate::units::{scale_decimal, unit_nm};
use crate::write::ViaKey;
use ferroute_board::{Board, BoardEdit, LayerId, LayerSet, NetId, Origin, Track, Via, ViaKind};
use ferroute_geom::{Coord, Point};

/// Exact length converter for one `(resolution unit n)` declaration.
struct Scale {
    unit_nm: i128,
    res: i128,
}

impl Scale {
    fn from_resolution(node: &Node) -> Result<Self, DsnError> {
        let it = node.items();
        let unit = it.get(1).and_then(Node::atom).ok_or_else(|| bad("resolution without unit"))?;
        let res = it.get(2).and_then(Node::atom).ok_or_else(|| bad("resolution without value"))?;
        let unit_nm = unit_nm(unit).ok_or_else(|| bad(&format!("unknown unit {unit:?}")))?;
        let res: i128 = res.parse().map_err(|_| bad(&format!("bad resolution {res:?}")))?;
        if res <= 0 {
            return Err(bad("resolution must be positive"));
        }
        Ok(Self { unit_nm, res })
    }

    fn len(&self, n: &Node) -> Result<Coord, DsnError> {
        let t = n.atom().ok_or_else(|| bad("expected a number"))?;
        scale_decimal(t, self.unit_nm, self.res)
    }

    /// A coordinate pair; `y` is negated (Specctra is y-up).
    fn point(&self, x: &Node, y: &Node) -> Result<Point, DsnError> {
        Ok(Point::new(self.len(x)?, -self.len(y)?))
    }
}

fn bad(msg: &str) -> DsnError {
    DsnError::Session(msg.to_owned())
}

fn is_fixed(n: &Node) -> bool {
    n.child("type")
        .and_then(|t| t.items().get(1))
        .and_then(Node::atom)
        .is_some_and(|t| matches!(t, "protect" | "fix"))
}

fn layer_of(board: &Board, name: &str) -> Result<LayerId, DsnError> {
    board
        .copper_layers
        .iter()
        .position(|l| l.name == name)
        .and_then(|i| u8::try_from(i).ok())
        .map(LayerId)
        .ok_or_else(|| bad(&format!("unknown layer {name:?}")))
}

/// Parses a Specctra SES session into an additive edit against `board`.
///
/// Wires become [`Track`]s and vias become [`Via`]s tagged [`Origin::Routed`]. Items of
/// type `protect`/`fix` and exact copies of existing copper are skipped, so applying an
/// SES that echoes the exported wiring changes nothing. Coordinates are converted with
/// exact integer arithmetic from the SES `(resolution ...)`; `y` is negated.
///
/// # Errors
/// [`DsnError::Syntax`] for malformed text; [`DsnError::Session`] for unknown nets or layers,
/// malformed numbers or a missing `routes`/`resolution`.
pub fn read_ses(ses: &str, board: &Board) -> Result<BoardEdit, DsnError> {
    let root = parse(ses)?;
    if root.head() != Some("session") {
        return Err(bad("not a Specctra session (expected `(session ...)`)"));
    }
    let routes = root.child("routes").ok_or_else(|| bad("session has no `routes`"))?;
    let scale = Scale::from_resolution(
        routes.child("resolution").ok_or_else(|| bad("routes has no `resolution`"))?,
    )?;
    let mut edit = BoardEdit::default();
    let Some(net_out) = routes.child("network_out") else { return Ok(edit) };
    let n_layers = board.layer_count();
    for net in net_out.children("net") {
        let name =
            net.items().get(1).and_then(Node::atom).ok_or_else(|| bad("net without name"))?;
        let id: NetId =
            board.net_by_name(name).ok_or_else(|| bad(&format!("unknown net {name:?}")))?;
        for wire in net.children("wire") {
            if is_fixed(wire) {
                continue;
            }
            let Some(path) = wire.child("path") else { continue };
            let it = path.items();
            let layer = layer_of(
                board,
                it.get(1).and_then(Node::atom).ok_or_else(|| bad("path without layer"))?,
            )?;
            let width = scale.len(it.get(2).ok_or_else(|| bad("path without width"))?)?;
            let coords = it.get(3..).unwrap_or(&[]);
            if coords.len() % 2 != 0 {
                return Err(bad("path has an odd number of coordinates"));
            }
            let points: Vec<Point> = coords
                .as_chunks::<2>()
                .0
                .iter()
                .map(|[x, y]| scale.point(x, y))
                .collect::<Result<_, _>>()?;
            for w in points.windows(2) {
                let (start, end) = (w[0], w[1]);
                if start == end {
                    continue;
                }
                let dup = board.tracks.iter().any(|t| {
                    t.net == id
                        && t.layer == layer
                        && t.width == width
                        && ((t.start, t.end) == (start, end) || (t.start, t.end) == (end, start))
                });
                if !dup {
                    edit.new_tracks.push(Track {
                        start,
                        end,
                        width,
                        layer,
                        net: id,
                        locked: false,
                        origin: Origin::Routed,
                    });
                }
            }
        }
        for via in net.children("via") {
            if is_fixed(via) {
                continue;
            }
            let it = via.items();
            let ps = it.get(1).and_then(Node::atom).ok_or_else(|| bad("via without padstack"))?;
            let pos = scale.point(
                it.get(2).ok_or_else(|| bad("via without x"))?,
                it.get(3).ok_or_else(|| bad("via without y"))?,
            )?;
            let class = board.rules.class_of(board, id);
            let (layers, diameter, drill) = match ViaKey::parse(ps) {
                Some(k) if k.first <= k.last && k.last < n_layers => {
                    (LayerSet::span(LayerId(k.first), LayerId(k.last)), k.diameter, k.drill)
                }
                _ => (board.all_layers(), class.via_diameter, class.via_drill),
            };
            let (first, last) =
                layers.iter().fold((u8::MAX, 0), |(a, b), l| (a.min(l.0), b.max(l.0)));
            let kind = if layers == board.all_layers() {
                ViaKind::Through
            } else if first == 0 || last == n_layers - 1 {
                if last - first == 1 { ViaKind::Micro } else { ViaKind::Blind }
            } else {
                ViaKind::Buried
            };
            let dup = board.vias.iter().any(|v| v.net == id && v.pos == pos && v.layers == layers);
            if !dup {
                edit.new_vias.push(Via {
                    pos,
                    diameter,
                    drill,
                    layers,
                    kind,
                    net: id,
                    locked: false,
                    origin: Origin::Routed,
                });
            }
        }
    }
    Ok(edit)
}
