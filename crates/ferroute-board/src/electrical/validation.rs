// SPDX-License-Identifier: AGPL-3.0-only
//! Validation of the electrical models against published reference data. Every table names
//! its source and the tolerance it is held to. No network access: reference formulas are
//! evaluated with plain `std` math inside the tests (they are oracles, not decisions).

use super::current::area_mil2_ipc2152;
use super::*;
use crate::builder::{BoardBuilder, mm};
use crate::constraints::*;
use crate::netrules::{NetRule, RuleSet};

const MIL: f64 = 25_400.0;
const PI: f64 = std::f64::consts::PI;

fn nm(mils: f64) -> i64 {
    (mils * MIL).round() as i64
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

// ---- Microstrip (Hammerstad–Jensen) -------------------------------------------------

/// Wheeler's air-dielectric microstrip, `Z0 = 60·ln(8/u + u/4)` for `u ≤ 1`
/// (Wheeler 1965; Wadell §3.2). Tolerance ±0.5 % against `microstrip_hj` with `εr = 1`
/// (measured max 0.15 %).
#[test]
fn microstrip_air_vs_wheeler() {
    for u in [0.1_f64, 0.25, 0.5, 0.75, 1.0] {
        let wheeler = 60.0 * (8.0 / u + u / 4.0).ln();
        let z = microstrip_hj((u * 200_000.0) as i64, 200_000, 0, 1.0).expect("z").0;
        assert!(rel(z, wheeler) < 0.005, "u={u}: HJ {z} vs Wheeler {wheeler}");
    }
}

/// Pozar, *Microwave Engineering* 4e, eqs. 3.195–3.197 (Wheeler/Hammerstad synthesis):
/// `W/d` for a target `Z0` at zero thickness. Tolerance ±1.5 % on the synthesised width
/// (measured max 0.73 % over εr ∈ {2.2, 4.3, 9.8}, Z0 ∈ {30, 50, 75, 100} Ω).
fn pozar_w_over_d(z0: f64, er: f64) -> f64 {
    let a = z0 / 60.0 * ((er + 1.0) / 2.0).sqrt() + (er - 1.0) / (er + 1.0) * (0.23 + 0.11 / er);
    let wd = 8.0 * a.exp() / ((2.0 * a).exp() - 2.0);
    if wd < 2.0 {
        return wd;
    }
    let b = 377.0 * PI / (2.0 * z0 * er.sqrt());
    2.0 / PI
        * (b - 1.0 - (2.0 * b - 1.0).ln()
            + (er - 1.0) / (2.0 * er) * ((b - 1.0).ln() + 0.39 - 0.61 / er))
}

#[test]
fn microstrip_width_vs_pozar_synthesis() {
    let h = 200_000;
    // (Z0, εr): FR-4, PTFE, alumina and a spread of impedances.
    for er in [2.2, 4.3, 9.8] {
        for z0 in [30.0, 50.0, 75.0, 100.0] {
            let w = microstrip_width_hj_exact(z0, h, er);
            let pw = pozar_w_over_d(z0, er) * 200_000.0;
            assert!(rel(w, pw) < 0.015, "εr={er} Z0={z0}: HJ {w:.0} nm vs Pozar {pw:.0} nm");
        }
    }
}

fn microstrip_width_hj_exact(z0: f64, h: i64, er: f64) -> f64 {
    // Bisect on the public model (zero thickness) to avoid the 10 µm rounding step.
    let (mut lo, mut hi) = (1_000_i64, 50 * h);
    while hi - lo > 1 {
        let mid = lo + (hi - lo) / 2;
        if microstrip_hj(mid, h, 0, er).expect("z").0 <= z0 {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    hi as f64
}

// ---- Stripline (Cohn) ---------------------------------------------------------------

/// Abramowitz & Stegun Table 17.1: `K(m=0.25) = 1.685750`, `K(m=0.75) = 2.156516`
/// (`k = 0.5`, `k' = √0.75`), so `K(k')/K(k) = 1.279…`. Tolerance 1e-5.
#[test]
fn elliptic_ratio_vs_abramowitz_stegun() {
    let r = k_ratio(0.5, 0.75_f64.sqrt());
    assert!((r - 2.156_516 / 1.685_750).abs() < 1e-5, "{r}");
}

/// Cohn (1955) exact zero-thickness stripline (`stripline_z0_cohn_exact`, AGM) as the
/// oracle for the closed forms across the wide and narrow strip regimes. Tolerance ±1 %.
#[test]
fn stripline_closed_forms_vs_cohn_exact() {
    let b = 400_000;
    for (wb, tol) in [
        (0.05, 0.015),
        (0.1, 0.01),
        (0.2, 0.01),
        (0.3, 0.01),
        (0.5, 0.01),
        (1.0, 0.01),
        (2.0, 0.01),
    ] {
        let w = (wb * f64::from(b as i32)) as i64;
        let ex = stripline_z0_cohn_exact(w, b, 4.3).expect("exact");
        let ap = stripline_z0(w, b, 0, 4.3).expect("closed form");
        assert!(rel(ap, ex) < tol, "w/b={wb}: {ap} vs exact {ex}");
    }
}

/// Wide ↔ narrow formulas must join at `w/(b−t) = 0.35` (Wadell chose that boundary).
#[test]
fn stripline_narrow_wide_continuity() {
    let b = 400_000;
    for t in [0, 17_500, 35_000] {
        let w_edge = (0.35 * f64::from((b - t) as i32)) as i64;
        let wide = stripline_z0(w_edge + 100, b, t, 4.3).expect("wide");
        let narrow = stripline_z0_narrow(w_edge - 100, b, t, 4.3).expect("narrow");
        assert!(rel(wide, narrow) < 0.01, "t={t}: {wide} vs {narrow}");
    }
}

#[test]
fn stripline_narrow_synthesis_and_unattainable() {
    // 85 Ω in 0.4 mm spacing FR-4 (35 µm-class copper 17.5 µm) needs a ~30 µm narrow strip;
    // before the narrow-strip model this was reported unattainable (`None`).
    let w = stripline_width_cohn(85.0, 400_000, 17_500, 4.3).expect("narrow strip");
    let z = stripline_z0(w, 400_000, 17_500, 4.3).expect("z");
    assert!(z <= 85.0, "{z} at w={w}");
    assert!(stripline_z0(w - WIDTH_STEP, 400_000, 17_500, 4.3).expect("z") > 85.0);
    assert!(w < 140_000, "narrow regime: {w}");
    // Zero-thickness strips reach 150 Ω with a finite width.
    assert!(stripline_width_cohn(150.0, 400_000, 0, 4.3).is_some());
    // Still reported when out of the model's reach (strip would be thinner than thick).
    assert!(stripline_width_cohn(100.0, 400_000, 17_500, 4.3).is_none());
    assert!(stripline_width_cohn(1500.0, 400_000, 0, 4.3).is_none());
}

#[test]
fn offset_stripline_properties() {
    let (w, t, er) = (150_000, 17_500, 4.3);
    // Symmetric limit equals the centred stripline exactly.
    let h = 200_000;
    let sym = stripline_z0(w, 2 * h + t, t, er).expect("sym");
    let off = offset_stripline_z0(w, h, h, t, er).expect("off");
    assert!((sym - off).abs() < 1e-9, "{sym} vs {off}");
    // Moving the strip toward one plane (same total spacing) lowers Z.
    let near = offset_stripline_z0(w, h / 2, h + h / 2, t, er).expect("near");
    assert!(near < sym, "{near} < {sym}");
    // Far plane removed: tends to 2·Z0 of the near half (single-plane limit).
    let far = offset_stripline_z0(w, h, 40 * h, t, er).expect("far");
    let half = stripline_z0(w, 2 * h + t, t, er).expect("half");
    assert!(far > sym && far < 2.0 * half * 1.001, "{far}");
}

// ---- Edge-coupled microstrip (Kirschning–Jansen static) ------------------------------

/// Limits every correct set of KJ/HJ static equations must satisfy: both modes tend to
/// the single-line values as `s → ∞` (checks all `Q` coefficients' tails), `Z0o < Z0 < Z0e`,
/// and with `εr = 1` both modes are air-filled (`εeff = 1`). Tolerances: 0.1 % at
/// `s/h = 100`, 0.5 % at `s/h = 20`.
#[test]
fn coupled_microstrip_limits() {
    for er in [2.2, 4.3, 9.8] {
        for u in [0.3, 1.0, 2.0] {
            let w = (u * 200_000.0) as i64;
            let (z0, e0) = microstrip_hj(w, 200_000, 0, er).expect("single");
            for (g, tol) in [(100.0, 0.001), (20.0, 0.005)] {
                let c =
                    coupled_microstrip_kj(w, (g * 200_000.0) as i64, 200_000, 0, er).expect("kj");
                assert!(
                    rel(c.z0e, z0) < tol && rel(c.z0o, z0) < tol,
                    "εr={er} u={u} g={g}: {c:?} vs {z0}"
                );
                assert!(rel(c.eeff_e, e0) < tol && rel(c.eeff_o, e0) < tol);
            }
            let c = coupled_microstrip_kj(w, 100_000, 200_000, 0, er).expect("kj");
            assert!(c.z0o < z0 && z0 < c.z0e, "{c:?} vs {z0}");
            assert!(c.eeff_o < c.eeff_e);
        }
    }
    let air = coupled_microstrip_kj(200_000, 100_000, 200_000, 0, 1.0).expect("air");
    assert!((air.eeff_e - 1.0).abs() < 1e-9 && (air.eeff_o - 1.0).abs() < 1e-9);
}

/// Coupling grows as the gap closes; thickness lowers both modes.
#[test]
fn coupled_microstrip_monotone() {
    let z = |s: i64, t: i64| coupled_microstrip_kj(200_000, s, 200_000, t, 4.3).expect("kj");
    let mut prev = z(50_000, 0);
    for s in [100_000, 200_000, 400_000, 800_000] {
        let c = z(s, 0);
        assert!(c.z0o > prev.z0o && c.z0e < prev.z0e, "s={s}");
        prev = c;
    }
    let (thin, thick) = (z(200_000, 0), z(200_000, 35_000));
    assert!(thick.zdiff() < thin.zdiff() && thick.z0e < thin.z0e);
}

/// Saturn PCB Toolkit manual, p. 11 (edge-coupled external pair): W = 10, S = 5, H = 15,
/// T = 2.10 mil, εr = 4.6 → Z0 = 77.50, Zodd = 50.49, Zeven = 118.97, Zdiff = 100.98 Ω
/// (values as reproduced in the `pcb-toolkit` crate's test-suite). Those are the IPC-2141A
/// approximation, itself good to roughly ±10 %: our fallback `microstrip_diff_ipc2141a`
/// (which swaps the IPC single-line formula for Hammerstad–Jensen) must stay within 5 %,
/// and the Kirschning–Jansen model within 12 % of the same published numbers.
#[test]
fn coupled_microstrip_vs_saturn_page_11() {
    let (w, s, h, t) = (nm(10.0), nm(5.0), nm(15.0), nm(2.1));
    let ipc = microstrip_diff_ipc2141a(w, s, h, t, 4.6).expect("ipc");
    assert!(rel(ipc, 100.979) < 0.05, "IPC fallback {ipc}");
    let kj = coupled_microstrip_kj(w, s, h, t, 4.6).expect("kj");
    assert!(rel(kj.zdiff(), 100.979) < 0.12, "KJ Zdiff {}", kj.zdiff());
    assert!(rel(kj.z0o, 50.490) < 0.12, "KJ Zodd {}", kj.z0o);
    assert!(rel(kj.z0e, 118.971) < 0.15, "KJ Zeven {}", kj.z0e);
}

#[test]
fn diff_impedance_defaults_to_ipc2141a_and_kj_is_opt_in() {
    let (w, s, h, t) = (150_000, 150_000, 200_000, 35_000);
    let ipc = microstrip_diff_ipc2141a(w, s, h, t, 4.3).expect("ipc");
    let kj = coupled_microstrip_kj(w, s, h, t, 4.3).expect("kj").zdiff();
    let d = diff_impedance(LineKind::Microstrip, w, s, h, t, 4.3).expect("z");
    assert!((d - ipc).abs() < 1e-12, "default is the old IPC-2141A result");
    let with = |m| diff_impedance_with(m, LineKind::Microstrip, w, s, h, t, 4.3).expect("z");
    assert!((with(CoupledModel::Ipc2141a) - ipc).abs() < 1e-12);
    assert!((with(CoupledModel::Kj) - kj).abs() < 1e-12);
    assert!((ipc - kj).abs() > 1e-6, "models differ");
    // KJ with εr outside its range falls back to IPC-2141A.
    let f = diff_impedance_with(CoupledModel::Kj, LineKind::Microstrip, w, s, h, t, 25.0);
    assert_eq!(f, microstrip_diff_ipc2141a(w, s, h, t, 25.0));
    // The pair solver follows the model.
    let g =
        |m| diff_pair_geometry_with(m, 100.0, LineKind::Microstrip, PairFixed::Gap(s), h, t, 4.3);
    assert_eq!(
        diff_pair_geometry(100.0, LineKind::Microstrip, PairFixed::Gap(s), h, t, 4.3),
        g(CoupledModel::Ipc2141a)
    );
    assert_ne!(g(CoupledModel::Ipc2141a), g(CoupledModel::Kj));
}

// ---- Solder mask ---------------------------------------------------------------------

#[test]
fn solder_mask_lowers_impedance_by_a_few_ohms() {
    let (w, h, t) = (200_000, 110_000, 35_000);
    let bare = microstrip_hj(w, h, t, 4.3).expect("z").0;
    assert!((microstrip_mask_z0(w, h, t, 4.3, 0, 3.5).expect("z") - bare).abs() < 1e-12);
    let m20 = microstrip_mask_z0(w, h, t, 4.3, 20_000, 3.5).expect("z");
    let m50 = microstrip_mask_z0(w, h, t, 4.3, 50_000, 3.5).expect("z");
    assert!(m50 < m20 && m20 < bare);
    assert!((1.0..4.0).contains(&(bare - m20)), "20 µm mask shift {}", bare - m20);
}

// ---- Coplanar waveguide ----------------------------------------------------------------

/// Without a lower plane `Z0 = 30π/√((εr+1)/2)·K(k')/K(k)` with `k = w/(w+2g)`; for
/// `w = 2g` (`k = 0.5`) `K(k')/K(k) = 1.2792` (A&S Table 17.1). Tolerance 1e-4.
#[test]
fn cpw_vs_closed_form_and_limits() {
    let (z, ee) = cpw_z0(300_000, 150_000, None, 0, 4.3).expect("z");
    let want = 30.0 * PI / (2.65_f64).sqrt() * (2.156_516 / 1.685_750);
    assert!(rel(z, want) < 1e-4 && (ee - 2.65).abs() < 1e-12, "{z} vs {want}");
    // A very distant plane changes nothing (1e-3); a near plane lowers Z and raises εeff.
    let (zf, _) = cpw_z0(300_000, 150_000, Some(200_000_000), 0, 4.3).expect("z");
    assert!(rel(zf, z) < 1e-3, "{zf} vs {z}");
    let (zg, eg) = cpw_z0(300_000, 150_000, Some(200_000), 0, 4.3).expect("z");
    assert!(zg < z && eg > ee && eg < 4.3, "{zg} {eg}");
    // Narrower gap → lower Z; thickness → lower Z.
    assert!(cpw_z0(300_000, 75_000, Some(200_000), 0, 4.3).expect("z").0 < zg);
    assert!(cpw_z0(300_000, 150_000, Some(200_000), 35_000, 4.3).expect("z").0 < zg);
}

#[test]
fn cpw_width_roundtrip() {
    for h in [None, Some(200_000)] {
        let w = cpw_width(50.0, 150_000, h, 35_000, 4.3).expect("w");
        let z = cpw_z0(w, 150_000, h, 35_000, 4.3).expect("z").0;
        assert!(z <= 50.0 && z > 48.5, "{z} at {w}"); // 10 µm rounding step
        assert!(cpw_z0(w - 10_000, 150_000, h, 35_000, 4.3).expect("z").0 > 50.0);
        assert_eq!(cpw_width(50.0, 150_000, h, 35_000, 4.3), Some(w), "deterministic");
    }
    assert!(cpw_width(-1.0, 150_000, None, 0, 4.3).is_none());
}

// ---- Current ----------------------------------------------------------------------------

#[test]
fn ipc2152_properties_and_relation_to_ipc2221() {
    let t1 = 35_000; // 1 oz
    let t2 = 70_000; // 2 oz
    // Monotone in current and in allowed rise (more rise → narrower).
    let mut prev = 0;
    for i in [100, 500, 1000, 2000, 5000, 10_000] {
        let w = width_for_current_2152(i, t1, 10.0);
        assert!(w > prev, "i={i}");
        prev = w;
    }
    assert!(width_for_current_2152(2000, t1, 30.0) < width_for_current_2152(2000, t1, 10.0));
    // Copper weight enters through the cross-section: 2 oz is half as wide (within a step).
    let (w1, w2) = (width_for_current_2152(3000, t1, 20.0), width_for_current_2152(3000, t2, 20.0));
    assert!((w1 - 2 * w2).abs() <= 2 * WIDTH_STEP, "{w1} vs 2×{w2}");
    // At 1 A / 10 °C an external 1 oz trace: IPC-2221 and the IPC-2152 fit agree within 10 %
    // (≈ 0.3 mm), the internal IPC-2221 formula is ≥ 2× wider (the known IPC-2221 internal
    // conservatism that IPC-2152 corrected).
    let (a, e, i) = (
        width_for_current_2152(1000, t1, 10.0),
        width_for_current(1000, t1, true, 10.0),
        width_for_current(1000, t1, false, 10.0),
    );
    assert!(rel(a as f64, e as f64) < 0.10, "{a} vs {e}");
    assert!(i as f64 >= 2.0 * a as f64, "{i} vs {a}");
    // "max" takes the wider; zero current is zero.
    for (cur, dt) in [(1000, 10.0), (5000, 30.0)] {
        let m = width_for_current_model(CurrentModel::Max, cur, t1, true, dt);
        assert_eq!(
            m,
            width_for_current_2152(cur, t1, dt).max(width_for_current(cur, t1, true, dt))
        );
    }
    assert_eq!(width_for_current_2152(0, t1, 10.0), 0);
    assert!(area_mil2_ipc2152(-1.0, 10.0).is_none() && area_mil2_ipc2152(1.0, 0.0).is_none());
}

#[test]
fn current_model_is_selectable_per_rule_and_defaults_to_ipc2221() {
    let build = || {
        let mut b = BoardBuilder::new(mm(30), mm(20), 2);
        b.net("P");
        b.build()
    };
    let width = |model: Option<CurrentModel>| {
        let mut board = build();
        let rs = RuleSet {
            rules: vec![NetRule {
                nets: vec!["P".into()],
                max_current_a: Some(3.0),
                current_model: model,
                ..NetRule::default()
            }],
            ..RuleSet::default()
        };
        rs.apply(&mut board).expect("valid");
        let n = board.net_by_name("P").expect("net");
        required_width(&board, n)
    };
    let (d, a, b2152, m) = (
        width(None),
        width(Some(CurrentModel::Ipc2221)),
        width(Some(CurrentModel::Ipc2152)),
        width(Some(CurrentModel::Max)),
    );
    assert_eq!(d, a, "default is IPC-2221");
    assert_ne!(a, b2152);
    assert_eq!(m, a.max(b2152));
}
