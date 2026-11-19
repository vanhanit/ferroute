// SPDX-License-Identifier: AGPL-3.0-only
//! Copper layer table handling. Stack order is derived from layer *names*, never ids
//! (ids changed between KiCad 8 and 9).

use crate::KicadError;
use crate::sexpr::SExpr;
use ferroute_board::{CopperLayer, LayerId, LayerSet};

/// Sort key of a copper layer name: `F.Cu` first, `In<n>.Cu` by `n`, `B.Cu` last.
pub(crate) fn copper_rank(name: &str) -> Option<u32> {
    match name {
        "F.Cu" => Some(0),
        "B.Cu" => Some(u32::MAX),
        _ => {
            let n = name.strip_prefix("In")?.strip_suffix(".Cu")?;
            n.parse::<u32>().ok().filter(|n| (1..u32::MAX).contains(n))
        }
    }
}

/// The board's copper stack in physical order.
#[derive(Clone, Debug, Default)]
pub(crate) struct Layers {
    pub copper: Vec<CopperLayer>,
}

impl Layers {
    /// Reads the top-level `(layers ...)` table.
    pub(crate) fn from_root(root: &SExpr) -> Result<Self, KicadError> {
        let table = root
            .child("layers")
            .ok_or_else(|| KicadError::Format("missing (layers) table".into()))?;
        let mut found: Vec<(u32, CopperLayer)> = Vec::new();
        for item in table.items().unwrap_or(&[]).iter().skip(1) {
            // (id "name" type ["user name"])
            let (Some(name), ty) = (item.atom(1), item.atom(2)) else { continue };
            let Some(rank) = copper_rank(name) else { continue };
            if found.iter().any(|(_, l)| l.name == name) {
                continue;
            }
            found
                .push((rank, CopperLayer { name: name.to_owned(), routable: ty != Some("power") }));
        }
        found.sort_by_key(|(r, _)| *r);
        if found.is_empty() {
            return Err(KicadError::Format("board has no copper layers".into()));
        }
        Ok(Self { copper: found.into_iter().map(|(_, l)| l).collect() })
    }

    pub(crate) fn id(&self, name: &str) -> Option<LayerId> {
        self.copper
            .iter()
            .position(|l| l.name == name)
            .and_then(|i| u8::try_from(i).ok())
            .map(LayerId)
    }

    pub(crate) fn all(&self) -> LayerSet {
        LayerSet::all(u8::try_from(self.copper.len()).unwrap_or(u8::MAX))
    }

    /// Copper layers selected by a list of layer tokens. `*.Cu` is every copper layer,
    /// `F&B.Cu` the two outer ones; non-copper names are ignored.
    pub(crate) fn mask<'a>(&self, names: impl IntoIterator<Item = &'a str>) -> LayerSet {
        let mut set = LayerSet::EMPTY;
        for n in names {
            match n {
                "*.Cu" => set = LayerSet(set.0 | self.all().0),
                "F&B.Cu" => {
                    set = set.with(LayerId(0));
                    set = set.with(LayerId(u8::try_from(self.copper.len() - 1).unwrap_or(0)));
                }
                _ => {
                    if let Some(id) = self.id(n) {
                        set = set.with(id);
                    }
                }
            }
        }
        set
    }

    /// Layer tokens of a `(layer "X")` or `(layers "X" "Y" ...)` child of `node`.
    pub(crate) fn mask_of(&self, node: &SExpr) -> LayerSet {
        let mut set = LayerSet::EMPTY;
        for key in ["layer", "layers"] {
            for c in node.children(key) {
                let names = c.items().unwrap_or(&[]).iter().skip(1).filter_map(SExpr::text);
                set = LayerSet(set.0 | self.mask(names).0);
            }
        }
        set
    }
}

/// KiCad 9+ numeric id of the copper layer at physical position `idx` of `count`.
pub(crate) fn kicad_cu_id(idx: usize, count: usize) -> usize {
    if idx == 0 {
        0
    } else if idx + 1 == count {
        2
    } else {
        2 + 2 * idx
    }
}

/// Canonical KiCad name for the copper layer at `idx` of `count`.
#[cfg(test)]
pub(crate) fn kicad_cu_name(idx: usize, count: usize) -> String {
    if idx == 0 {
        "F.Cu".into()
    } else if idx + 1 == count {
        "B.Cu".into()
    } else {
        format!("In{idx}.Cu")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ranks_and_ids() {
        assert_eq!(copper_rank("F.Cu"), Some(0));
        assert_eq!(copper_rank("In2.Cu"), Some(2));
        assert_eq!(copper_rank("B.Cu"), Some(u32::MAX));
        assert_eq!(copper_rank("F.Mask"), None);
        assert_eq!(copper_rank("In0.Cu"), None);
        // KiCad 9 ids: F.Cu=0 B.Cu=2 In1=4 In2=6
        assert_eq!(kicad_cu_id(0, 4), 0);
        assert_eq!(kicad_cu_id(1, 4), 4);
        assert_eq!(kicad_cu_id(2, 4), 6);
        assert_eq!(kicad_cu_id(3, 4), 2);
        assert_eq!(kicad_cu_name(2, 4), "In2.Cu");
    }
}
