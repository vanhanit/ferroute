// SPDX-License-Identifier: AGPL-3.0-only
//! # ferroute-render
//!
//! Deterministic SVG renderer for board snapshots, used for visual diffs in CI.
//!
//! Determinism: every collection is iterated in index order, every coordinate is printed
//! through an exact integer nanometre → millimetre formatter, and no floating-point value
//! is ever formatted directly. The same `(Board, RenderOptions)` always yields the same bytes.
//!
//! The SVG user unit is one millimetre; `width`/`height` are in pixels. Coordinates are
//! `+y` down, which matches both SVG and the nanometre model, so no flip is applied.

use std::fmt::Write as _;
use std::path::Path;

use ferroute_board::{Board, LayerId, LayerSet, NetId, Origin};
use ferroute_geom::{Coord, Hull, Point, Rect};

/// A DRC marker: a red ring with an X at `at`, carrying `label` as its `<title>`.
#[derive(Debug, Clone)]
pub struct Marker {
    pub at: Point,
    pub label: String,
}

/// Rendering options. [`RenderOptions::default`] gives a full copper view with courtyards.
#[derive(Debug, Clone)]
pub struct RenderOptions {
    /// Pixels per millimetre for the `width`/`height` attributes.
    pub px_per_mm: f64,
    /// Copper layers to draw; `None` draws every copper layer.
    pub layers: Option<LayerSet>,
    pub show_courtyards: bool,
    /// Unrouted connections, drawn as thin dashed lines.
    pub ratsnest: Vec<(Point, Point)>,
    /// DRC violations.
    pub markers: Vec<Marker>,
    /// Footprint displacement arrows (from → to).
    pub moves: Vec<(Point, Point)>,
    pub title: Option<String>,
    /// Draw `Origin::Routed` copper at full opacity, `Origin::Existing` at 0.85.
    pub highlight_routed: bool,
}

impl Default for RenderOptions {
    fn default() -> Self {
        Self {
            px_per_mm: DEFAULT_PX_PER_MM,
            layers: None,
            show_courtyards: true,
            ratsnest: Vec::new(),
            markers: Vec::new(),
            moves: Vec::new(),
            title: None,
            highlight_routed: false,
        }
    }
}

const DEFAULT_PX_PER_MM: f64 = 20.0;
const NM_PER_MM: Coord = 1_000_000;
const MARGIN: Coord = 2 * NM_PER_MM;
const LINE_THIN: Coord = NM_PER_MM / 20; // 0.05 mm
const LINE_MEDIUM: Coord = NM_PER_MM / 10; // 0.1 mm
const MIN_DOT_RADIUS: Coord = NM_PER_MM / 100; // 0.01 mm
const MARKER_RADIUS: Coord = NM_PER_MM / 2; // 0.5 mm
const MARKER_ARM: Coord = NM_PER_MM * 35 / 100; // 0.35 mm

const BACKGROUND: &str = "#0b0f14";
const OUTLINE_FILL: &str = "#14202a";
const OUTLINE_EDGE: &str = "#2e4454";
const KEEPOUT: &str = "#ff8c00";
const VIA_COPPER: &str = "#c8c8c8";
const COURTYARD: &str = "#888888";
const RATSNEST: &str = "#ffffff";
const MOVE: &str = "#ffd000";
const MARKER: &str = "#ff3040";
const LABEL: &str = "#dcdcdc";
const COPPER_OPACITY: &str = "0.85";
const ROUTED_OPACITY: &str = "1";
const INNER_COLOURS: [&str; 4] = ["#c2c23a", "#34b07a", "#b05ad0", "#e0803a"];
const FRONT_COLOUR: &str = "#c83434";
const BACK_COLOUR: &str = "#3a6fd8";

/// Renders `board` as a standalone SVG document.
#[must_use]
pub fn render_svg(board: &Board, opts: &RenderOptions) -> String {
    let content = board
        .outline
        .bbox()
        .or_else(|| content_bbox(board, opts))
        .unwrap_or(Rect::new(Point::ORIGIN, Point::new(NM_PER_MM, NM_PER_MM)));
    let vb = content.inflate(MARGIN);
    let (vw, vh) = (vb.max.x - vb.min.x, vb.max.y - vb.min.y);
    let ppm = if opts.px_per_mm.is_finite() && opts.px_per_mm > 0.0 {
        opts.px_per_mm
    } else {
        DEFAULT_PX_PER_MM
    };
    let sel = selected_layers(board, opts);
    let mut s = String::new();

    let _ = writeln!(
        s,
        r#"<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}" viewBox="{} {} {} {}">"#,
        px(vw, ppm),
        px(vh, ppm),
        mm(vb.min.x),
        mm(vb.min.y),
        mm(vw),
        mm(vh)
    );
    let _ = writeln!(
        s,
        concat!(
            r#"<defs>"#,
            r#"<pattern id="hatch" width="0.5000" height="0.5000" patternUnits="userSpaceOnUse">"#,
            r#"<path d="M 0.0000 0.0000 L 0.5000 0.5000" stroke="{keepout}" stroke-width="0.0500" fill="none"/>"#,
            r#"</pattern>"#,
            r#"<marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="4" markerHeight="4" orient="auto-start-reverse">"#,
            r#"<path d="M 0 0 L 10 5 L 0 10 z" fill="{moves}"/>"#,
            r#"</marker>"#,
            r#"</defs>"#
        ),
        keepout = KEEPOUT,
        moves = MOVE
    );
    let _ = writeln!(
        s,
        r#"<rect x="{}" y="{}" width="{}" height="{}" fill="{BACKGROUND}"/>"#,
        mm(vb.min.x),
        mm(vb.min.y),
        mm(vw),
        mm(vh)
    );

    // Board outline with holes (even-odd so cut-outs are holes regardless of winding).
    if board.outline.outer.pts.len() >= 3 {
        let mut d = String::new();
        push_path(&mut d, &board.outline.outer.pts, true);
        for hole in &board.outline.holes {
            if hole.pts.len() >= 3 {
                push_path(&mut d, &hole.pts, true);
            }
        }
        let _ = writeln!(
            s,
            r#"<path id="outline" d="{d}" fill="{OUTLINE_FILL}" fill-rule="evenodd" stroke="{OUTLINE_EDGE}" stroke-width="{}"/>"#,
            mm(LINE_THIN)
        );
    }

    for (i, k) in board.keepouts.iter().enumerate() {
        if k.area.pts.len() < 3 || (opts.layers.is_some() && !k.layers.intersects(sel)) {
            continue;
        }
        let mut d = String::new();
        push_path(&mut d, &k.area.pts, true);
        let _ = writeln!(
            s,
            r#"<path class="keepout" data-keepout="{i}" d="{d}" fill="url(#hatch)" stroke="{KEEPOUT}" stroke-width="{}" stroke-dasharray="0.2000 0.1000" opacity="0.7"/>"#,
            mm(LINE_THIN)
        );
    }

    // Copper, bottom layer first so the front layer paints on top.
    for li in (0..board.copper_layers.len()).rev() {
        let Ok(id) = u8::try_from(li) else { continue };
        let layer = LayerId(id);
        if !sel.contains(layer) {
            continue;
        }
        let colour = layer_colour(board, layer);
        let name = esc(&board.copper_layers[li].name);
        let _ = writeln!(s, r#"<g id="layer-{name}">"#);
        for t in board.tracks.iter().filter(|t| t.layer == layer) {
            let attrs = net_attr(board, t.net);
            let op = copper_opacity(t.origin, opts.highlight_routed);
            emit(&mut s, hull_element(&t.hull(), colour, op, &attrs));
        }
        for pid in board.pad_ids() {
            let pad = board.pad(pid);
            if !pad.layers.contains(layer) {
                continue;
            }
            let attrs = net_attr(board, pad.net);
            emit(&mut s, hull_element(&board.pad_hull(pid), colour, COPPER_OPACITY, &attrs));
        }
        s.push_str("</g>\n");
    }

    // Drill holes sit above all copper so no layer group can paint over them.
    s.push_str("<g id=\"drills\">\n");
    for pid in board.pad_ids() {
        let pad = board.pad(pid);
        let Some(drill) = pad.drill else { continue };
        if !pad.layers.intersects(sel) {
            continue;
        }
        let p = board.pad_position(pid);
        let _ = writeln!(
            s,
            r#"<circle cx="{}" cy="{}" r="{}" fill="{BACKGROUND}"/>"#,
            mm(p.x),
            mm(p.y),
            mm(drill.diameter / 2)
        );
    }
    s.push_str("</g>\n");

    s.push_str("<g id=\"vias\">\n");
    for v in &board.vias {
        if !v.layers.intersects(sel) {
            continue;
        }
        let attrs = net_attr(board, v.net);
        let op = copper_opacity(v.origin, opts.highlight_routed);
        emit(&mut s, hull_element(&v.hull(), VIA_COPPER, op, &attrs));
        let _ = writeln!(
            s,
            r#"<circle cx="{}" cy="{}" r="{}" fill="{BACKGROUND}"/>"#,
            mm(v.pos.x),
            mm(v.pos.y),
            mm(v.drill / 2)
        );
    }
    s.push_str("</g>\n");

    if opts.show_courtyards {
        let _ = writeln!(
            s,
            r#"<g id="courtyards" fill="none" stroke="{COURTYARD}" stroke-width="{}">"#,
            mm(LINE_THIN)
        );
        for fp in &board.footprints {
            let cy = fp.courtyard();
            if cy.pts.len() < 3 {
                continue;
            }
            let dash =
                if fp.mobility.is_movable() { r#" stroke-dasharray="0.2000 0.1000""# } else { "" };
            let mut d = String::new();
            push_path(&mut d, &cy.pts, true);
            let _ = writeln!(s, r#"<path data-ref="{}" d="{d}"{dash}/>"#, esc(&fp.reference));
        }
        s.push_str("</g>\n");
    }

    let _ = writeln!(
        s,
        r#"<g id="refs" font-family="monospace" font-size="0.8" text-anchor="middle" dominant-baseline="central" fill="{LABEL}">"#
    );
    for fp in &board.footprints {
        let _ = writeln!(
            s,
            r#"<text x="{}" y="{}">{}</text>"#,
            mm(fp.placement.pos.x),
            mm(fp.placement.pos.y),
            esc(&fp.reference)
        );
    }
    s.push_str("</g>\n");

    let _ = writeln!(
        s,
        r#"<g id="ratsnest" stroke="{RATSNEST}" stroke-opacity="0.5" stroke-width="{}" stroke-dasharray="0.2000 0.2000">"#,
        mm(LINE_THIN)
    );
    for &(a, b) in &opts.ratsnest {
        let _ = writeln!(
            s,
            r#"<line x1="{}" y1="{}" x2="{}" y2="{}"/>"#,
            mm(a.x),
            mm(a.y),
            mm(b.x),
            mm(b.y)
        );
    }
    s.push_str("</g>\n");

    let _ = writeln!(
        s,
        r#"<g id="moves" stroke="{MOVE}" stroke-width="{}" stroke-dasharray="0.3000 0.2000" marker-end="url(#arrow)">"#,
        mm(LINE_MEDIUM)
    );
    for &(a, b) in &opts.moves {
        let _ = writeln!(
            s,
            r#"<line x1="{}" y1="{}" x2="{}" y2="{}"/>"#,
            mm(a.x),
            mm(a.y),
            mm(b.x),
            mm(b.y)
        );
    }
    s.push_str("</g>\n");

    for (i, m) in opts.markers.iter().enumerate() {
        let (x, y) = (m.at.x, m.at.y);
        let _ = writeln!(
            s,
            r#"<g id="marker-{i}"><title>{label}</title><circle cx="{cx}" cy="{cy}" r="{r}" fill="none" stroke="{MARKER}" stroke-width="{w}"/><path d="M {x1} {y1} L {x2} {y2} M {x3} {y3} L {x4} {y4}" fill="none" stroke="{MARKER}" stroke-width="{w}"/></g>"#,
            label = esc(&m.label),
            cx = mm(x),
            cy = mm(y),
            r = mm(MARKER_RADIUS),
            w = mm(LINE_MEDIUM),
            x1 = mm(x - MARKER_ARM),
            y1 = mm(y - MARKER_ARM),
            x2 = mm(x + MARKER_ARM),
            y2 = mm(y + MARKER_ARM),
            x3 = mm(x + MARKER_ARM),
            y3 = mm(y - MARKER_ARM),
            x4 = mm(x - MARKER_ARM),
            y4 = mm(y + MARKER_ARM),
        );
    }

    if let Some(title) = &opts.title {
        let _ = writeln!(
            s,
            r#"<text x="{}" y="{}" font-family="sans-serif" font-size="1" fill="{LABEL}">{}</text>"#,
            mm(vb.min.x + MARGIN / 4),
            mm(vb.min.y + MARGIN * 3 / 5),
            esc(title)
        );
    }

    s.push_str("</svg>\n");
    s
}

/// Renders `board` with [`render_svg`] and writes the result to `path`.
///
/// # Errors
/// Any I/O error from writing the file.
pub fn write_svg(path: &Path, board: &Board, opts: &RenderOptions) -> std::io::Result<()> {
    std::fs::write(path, render_svg(board, opts))
}

/// Formats a nanometre value as millimetres with exactly four decimals, using integer
/// arithmetic only. Rounds half away from zero and never prints `-0.0000`.
fn mm(nm: Coord) -> String {
    let q = nm.unsigned_abs().saturating_add(50) / 100; // units of 0.0001 mm
    let sign = if nm < 0 && q != 0 { "-" } else { "" };
    format!("{sign}{}.{:04}", q / 10_000, q % 10_000)
}

/// Pixel length of a nanometre span at `px_per_mm`, at least one pixel.
fn px(nm: Coord, px_per_mm: f64) -> u64 {
    let v = nm as f64 / NM_PER_MM as f64 * px_per_mm;
    v.round().max(1.0) as u64
}

/// Escapes text and attribute content for XML.
fn esc(s: &str) -> String {
    let mut o = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '&' => o.push_str("&amp;"),
            '<' => o.push_str("&lt;"),
            '>' => o.push_str("&gt;"),
            '"' => o.push_str("&quot;"),
            '\'' => o.push_str("&apos;"),
            _ => o.push(c),
        }
    }
    o
}

/// Appends subpath data for `pts` to the path string `d` (subpaths are space-separated).
fn push_path(d: &mut String, pts: &[Point], closed: bool) {
    for (i, p) in pts.iter().enumerate() {
        if !d.is_empty() {
            d.push(' ');
        }
        let cmd = if i == 0 { 'M' } else { 'L' };
        let _ = write!(d, "{cmd}{} {}", mm(p.x), mm(p.y));
    }
    if closed && pts.len() >= 3 {
        d.push_str(" Z");
    }
}

/// A hull as an SVG element. Polygons and segments are stroked with width `2 * radius`
/// and round joins/caps, which draws exactly their Minkowski sum with a disk.
fn hull_element(h: &Hull, colour: &str, opacity: &str, attrs: &str) -> Option<String> {
    let pts: Vec<Point> = h.pts.iter().copied().collect();
    let r = h.radius.max(0);
    match pts.as_slice() {
        [] => None,
        [c] => Some(format!(
            r#"<circle cx="{}" cy="{}" r="{}" fill="{colour}" opacity="{opacity}"{attrs}/>"#,
            mm(c.x),
            mm(c.y),
            mm(r.max(MIN_DOT_RADIUS))
        )),
        _ => {
            let mut d = String::new();
            push_path(&mut d, &pts, true);
            Some(format!(
                r#"<path d="{d}" fill="{colour}" stroke="{colour}" stroke-width="{}" stroke-linejoin="round" stroke-linecap="round" opacity="{opacity}"{attrs}/>"#,
                mm(2 * r)
            ))
        }
    }
}

fn emit(out: &mut String, el: Option<String>) {
    if let Some(el) = el {
        out.push_str(&el);
        out.push('\n');
    }
}

fn net_attr(board: &Board, net: NetId) -> String {
    let name = board.nets.get(net.idx()).map_or("", |n| n.name.as_str());
    format!(r#" data-net="{}""#, esc(name))
}

fn copper_opacity(origin: Origin, highlight_routed: bool) -> &'static str {
    if highlight_routed && origin == Origin::Routed { ROUTED_OPACITY } else { COPPER_OPACITY }
}

/// Front copper is red, back copper blue, inner layers cycle through the palette.
fn layer_colour(board: &Board, l: LayerId) -> &'static str {
    let i = usize::from(l.0);
    if i == 0 {
        FRONT_COLOUR
    } else if i + 1 == board.copper_layers.len() {
        BACK_COLOUR
    } else {
        INNER_COLOURS[(i - 1) % INNER_COLOURS.len()]
    }
}

fn selected_layers(board: &Board, opts: &RenderOptions) -> LayerSet {
    opts.layers.unwrap_or_else(|| board.all_layers())
}

fn grow(acc: &mut Option<Rect>, r: Rect) {
    let cur = *acc;
    *acc = Some(cur.map_or(r, |a| a.union(r)));
}

fn point_rect(p: Point) -> Rect {
    Rect::new(p, p)
}

/// Bounding box of everything drawn; used only when the outline is missing.
fn content_bbox(board: &Board, opts: &RenderOptions) -> Option<Rect> {
    let mut acc = None;
    for pid in board.pad_ids() {
        grow(&mut acc, board.pad_hull(pid).bbox());
    }
    for t in &board.tracks {
        grow(&mut acc, t.hull().bbox());
    }
    for v in &board.vias {
        grow(&mut acc, v.hull().bbox());
    }
    for k in &board.keepouts {
        if let Some(r) = k.area.bbox() {
            grow(&mut acc, r);
        }
    }
    for fp in &board.footprints {
        if let Some(r) = fp.courtyard().bbox() {
            grow(&mut acc, r);
        }
    }
    for &(a, b) in opts.ratsnest.iter().chain(&opts.moves) {
        grow(&mut acc, point_rect(a));
        grow(&mut acc, point_rect(b));
    }
    for m in &opts.markers {
        grow(&mut acc, point_rect(m.at));
    }
    acc
}

#[cfg(test)]
mod tests;
