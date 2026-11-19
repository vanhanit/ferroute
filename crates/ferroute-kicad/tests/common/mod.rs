// SPDX-License-Identifier: AGPL-3.0-only
#![allow(dead_code)]

use ferroute_board::{Board, FootprintId, NetId, PadId};
use ferroute_geom::Point;
use ferroute_kicad::{KicadBoard, read_board_file};
use std::path::PathBuf;

pub fn fixture(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../testdata/boards").join(name)
}

pub fn load(name: &str) -> KicadBoard {
    read_board_file(&fixture(name)).unwrap_or_else(|e| panic!("{name}: {e}"))
}

pub fn mm(v: f64) -> i64 {
    ferroute_geom::units::mm_to_nm(v)
}

pub fn fp_id(b: &Board, reference: &str) -> FootprintId {
    let i = b.footprints.iter().position(|f| f.reference == reference).expect(reference);
    FootprintId(u32::try_from(i).unwrap())
}

pub fn pad_id(b: &Board, reference: &str, number: &str) -> PadId {
    let fp = b.footprint(fp_id(b, reference));
    *fp.pads.iter().find(|&&p| b.pad(p).number == number).expect(number)
}

pub fn pad_pos(b: &Board, reference: &str, number: &str) -> Point {
    b.pad_position(pad_id(b, reference, number))
}

pub fn net_name(b: &Board, n: NetId) -> &str {
    &b.nets[n.idx()].name
}

pub fn near(a: Point, b: Point, tol: i64) -> bool {
    (a.x - b.x).abs() <= tol && (a.y - b.y).abs() <= tol
}
