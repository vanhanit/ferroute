// SPDX-License-Identifier: AGPL-3.0-only
//! `.kicad_pro` (JSON) design rules.
//!
//! Key names verified against KiCad 10 (`common/project/net_settings.cpp`,
//! `pcbnew/board_design_settings.cpp`):
//! * `net_settings.classes[]`: `name`, `priority`, `clearance`, `track_width`,
//!   `via_diameter`, `via_drill` (mm as JSON numbers; absent = inherited from `Default`);
//! * `net_settings.netclass_assignments`: net name to an *array* of class names (older
//!   files: a single string);
//! * `net_settings.netclass_patterns[]`: `{ "pattern", "netclass" }` with `*`/`?` globs;
//! * `board.design_settings.rules`: `min_copper_edge_clearance`, `min_hole_to_hole`,
//!   `min_hole_clearance`, `min_track_width`, `min_via_diameter`,
//!   `min_through_hole_diameter`.

use crate::KicadError;
use ferroute_board::rules::{DEFAULT_MICROVIA_DIAMETER, DEFAULT_MICROVIA_DRILL};
use ferroute_board::{Board, NetClass};
use ferroute_geom::Coord;
use ferroute_geom::units::{mm_to_nm, nm_to_mm};
use serde_json::{Map, Value, json};

fn mm_field(o: &Map<String, Value>, key: &str) -> Option<Coord> {
    o.get(key).and_then(Value::as_f64).map(mm_to_nm)
}

use ferroute_board::glob;

/// Fills `board.rules` (net classes, board-level minima) and each net's class from the
/// contents of a `.kicad_pro` file.
///
/// # Errors
/// [`KicadError::Json`] if the text is not a JSON object.
pub fn read_project_rules(json: &str, board: &mut Board) -> Result<(), KicadError> {
    let v: Value = serde_json::from_str(json).map_err(|e| KicadError::Json(e.to_string()))?;
    let Some(root) = v.as_object() else {
        return Err(KicadError::Json("project file is not a JSON object".into()));
    };

    if let Some(rules) = root
        .get("board")
        .and_then(|b| b.get("design_settings"))
        .and_then(|d| d.get("rules"))
        .and_then(Value::as_object)
    {
        let r = &mut board.rules;
        if let Some(x) = mm_field(rules, "min_copper_edge_clearance") {
            r.edge_clearance = x;
        }
        if let Some(x) = mm_field(rules, "min_hole_to_hole") {
            r.hole_to_hole = x;
        }
        if let Some(x) = mm_field(rules, "min_hole_clearance") {
            r.hole_clearance = x;
        }
        if let Some(x) = mm_field(rules, "min_track_width") {
            r.min_track_width = x;
        }
        if let Some(x) = mm_field(rules, "min_via_diameter") {
            r.min_via_diameter = x;
        }
        if let Some(x) = mm_field(rules, "min_through_hole_diameter") {
            r.min_via_drill = x;
        }
        if let Some(x) = mm_field(rules, "min_microvia_diameter") {
            r.min_microvia_diameter = x;
        }
        if let Some(x) = mm_field(rules, "min_microvia_drill") {
            r.min_microvia_drill = x;
        }
    }

    let Some(ns) = root.get("net_settings").and_then(Value::as_object) else {
        return Ok(());
    };

    // Classes: "Default" first (inheritance base), then the rest in file order.
    let base = NetClass::default();
    let entries: Vec<&Map<String, Value>> = ns
        .get("classes")
        .and_then(Value::as_array)
        .map(|a| a.iter().filter_map(Value::as_object).collect())
        .unwrap_or_default();
    let default_entry =
        entries.iter().find(|e| e.get("name").and_then(Value::as_str) == Some("Default"));
    let mut default = base;
    if let Some(e) = default_entry {
        default.clearance = mm_field(e, "clearance").unwrap_or(default.clearance);
        default.track_width = mm_field(e, "track_width").unwrap_or(default.track_width);
        default.via_diameter = mm_field(e, "via_diameter").unwrap_or(default.via_diameter);
        default.via_drill = mm_field(e, "via_drill").unwrap_or(default.via_drill);
        default.microvia_diameter =
            mm_field(e, "microvia_diameter").unwrap_or(default.microvia_diameter);
        default.microvia_drill = mm_field(e, "microvia_drill").unwrap_or(default.microvia_drill);
    }
    let mut classes = vec![default.clone()];
    let mut priority: Vec<i64> = vec![i64::MAX];
    for e in &entries {
        let Some(name) = e.get("name").and_then(Value::as_str) else { continue };
        if name == "Default" {
            continue;
        }
        classes.push(NetClass {
            name: name.to_owned(),
            clearance: mm_field(e, "clearance").unwrap_or(default.clearance),
            track_width: mm_field(e, "track_width").unwrap_or(default.track_width),
            via_diameter: mm_field(e, "via_diameter").unwrap_or(default.via_diameter),
            via_drill: mm_field(e, "via_drill").unwrap_or(default.via_drill),
            microvia_diameter: mm_field(e, "microvia_diameter")
                .unwrap_or(default.microvia_diameter),
            microvia_drill: mm_field(e, "microvia_drill").unwrap_or(default.microvia_drill),
        });
        priority.push(e.get("priority").and_then(Value::as_i64).unwrap_or(i64::MAX - 1));
    }
    let class_index = |name: &str| classes.iter().position(|c| c.name == name);

    // Candidate classes per net: explicit assignments, then every matching pattern.
    let patterns: Vec<(&str, &str)> = ns
        .get("netclass_patterns")
        .and_then(Value::as_array)
        .map(|a| {
            a.iter()
                .filter_map(|p| Some((p.get("pattern")?.as_str()?, p.get("netclass")?.as_str()?)))
                .collect()
        })
        .unwrap_or_default();
    let assignments = ns.get("netclass_assignments").and_then(Value::as_object);
    for net in board.nets.iter_mut().skip(1) {
        let mut cands: Vec<usize> = Vec::new();
        if let Some(a) = assignments.and_then(|a| a.get(&net.name)) {
            let names: Vec<&str> = match a {
                Value::String(s) => vec![s.as_str()],
                Value::Array(v) => v.iter().filter_map(Value::as_str).collect(),
                _ => Vec::new(),
            };
            cands.extend(names.into_iter().filter_map(class_index));
        }
        for (pat, class) in &patterns {
            if glob(pat, &net.name)
                && let Some(i) = class_index(class)
            {
                cands.push(i);
            }
        }
        // Lowest priority number wins; ties keep the first candidate.
        let best = cands.iter().copied().min_by_key(|&i| priority[i]).unwrap_or(0);
        net.class = u16::try_from(best).unwrap_or(0);
    }
    board.rules.classes = classes;
    Ok(())
}

/// Writes a minimal `.kicad_pro` carrying the board's net classes, net assignments and
/// board-level rules (the inverse of [`read_project_rules`]). Deterministic.
#[must_use]
pub fn write_project_json(board: &Board) -> String {
    let classes: Vec<Value> = board
        .rules
        .classes
        .iter()
        .enumerate()
        .map(|(i, c)| {
            let mut j = json!({
                "name": c.name,
                "priority": if i == 0 { i64::from(i32::MAX) } else { i64::try_from(i).unwrap_or(0) - 1 },
                "clearance": nm_to_mm(c.clearance),
                "track_width": nm_to_mm(c.track_width),
                "via_diameter": nm_to_mm(c.via_diameter),
                "via_drill": nm_to_mm(c.via_drill),
            });
            // KiCad's defaults are implied; only deviations are written.
            if c.microvia_diameter != DEFAULT_MICROVIA_DIAMETER {
                j["microvia_diameter"] = json!(nm_to_mm(c.microvia_diameter));
            }
            if c.microvia_drill != DEFAULT_MICROVIA_DRILL {
                j["microvia_drill"] = json!(nm_to_mm(c.microvia_drill));
            }
            j
        })
        .collect();
    let mut assign = Map::new();
    for n in board.nets.iter().skip(1) {
        if n.class != 0
            && let Some(c) = board.rules.classes.get(usize::from(n.class))
        {
            assign.insert(n.name.clone(), json!([c.name]));
        }
    }
    let r = &board.rules;
    let doc = json!({
        "board": { "design_settings": { "rules": {
            "min_copper_edge_clearance": nm_to_mm(r.edge_clearance),
            "min_hole_to_hole": nm_to_mm(r.hole_to_hole),
            "min_hole_clearance": nm_to_mm(r.hole_clearance),
            "min_track_width": nm_to_mm(r.min_track_width),
            "min_via_diameter": nm_to_mm(r.min_via_diameter),
            "min_through_hole_diameter": nm_to_mm(r.min_via_drill),
        } } },
        "meta": { "filename": "ferroute.kicad_pro", "version": 3 },
        "net_settings": {
            "classes": classes,
            "meta": { "version": 4 },
            "netclass_assignments": assign,
            "netclass_patterns": [],
        },
    });
    let mut s = serde_json::to_string_pretty(&doc).unwrap_or_default();
    s.push('\n');
    s
}
