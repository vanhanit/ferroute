// SPDX-License-Identifier: AGPL-3.0-only
//! Occupancy planning: how many lattice rings each kind of routed copper must reserve
//! so that **disjoint reservations imply DRC-clean geometry**.
//!
//! ## Theorem (lattice separation)
//! Every routed element `e` (a track node, the two *side nodes* of a diagonal step, a via
//! site on every layer) has a *base* node and reserves the *ring* = Chebyshev ball of
//! radius `ρ(e)` (lattice steps) around it. Rule: **no base node of one net may lie in a
//! ring of another net.** Then for elements `e`, `f` of different nets, every point of
//! `e`'s copper centre-line lies at Euclidean distance `≥ (max(ρ(e), ρ(f)) + 1) · pitch`
//! from every point of `f`'s centre-line.
//!
//! *Proof sketch.* The rule gives Chebyshev distance `≥ max(ρ(e), ρ(f)) + 1` between
//! base nodes of different nets. Centre-lines of lattice copper are unions of lattice
//! nodes, unit orthogonal segments and unit diagonals; each piece lies in a closed unit
//! cell (possibly degenerate) whose corners are all base nodes of its net (diagonals
//! include their side nodes in the base set). For two such cells the minimum Chebyshev
//! distance between their points equals the minimum Chebyshev distance between their
//! corners, and Euclidean distance dominates Chebyshev distance. ∎
//!
//! So it suffices to choose ρ such that `(max(ρ(e), ρ(f)) + 1)·pitch ≥ D(e,f)` for the
//! required centre distance `D` of every element-kind pair; [`Plan::new`] computes the
//! smallest such integers. Vias additionally use a net-independent *hole channel* so that
//! hole-to-hole spacing also holds between vias of the same net.

use crate::legality::ClassGeom;
use ferroute_board::DesignRules;
use ferroute_geom::Coord;

/// Reservation radii of one net class.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ClassPlan {
    pub geom: ClassGeom,
    /// Track reservation radius (lattice steps).
    pub k: u32,
    /// Via copper reservation radius (lattice steps, all layers).
    pub rv: u32,
}

/// Reservation plan for all classes.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Plan {
    pub pitch: Coord,
    pub classes: Vec<ClassPlan>,
    /// Hole-channel radius (lattice steps, planar), shared by all vias of all nets.
    pub rh: u32,
}

/// Geometry of every class of `rules`.
#[must_use]
pub fn class_geoms(rules: &DesignRules) -> Vec<ClassGeom> {
    rules
        .classes
        .iter()
        .map(|c| ClassGeom {
            track_width: c.track_width,
            track_radius: c.track_width / 2,
            clearance: c.clearance,
            via_diameter: c.via_diameter,
            via_radius: c.via_diameter / 2,
            via_drill: c.via_drill,
        })
        .collect()
}

fn steps_needed(dist: Coord, pitch: Coord) -> u32 {
    // smallest s with s * pitch >= dist
    u32::try_from(((dist + pitch - 1) / pitch).max(0)).unwrap_or(u32::MAX)
}

impl Plan {
    /// Smallest reservation radii satisfying every pairwise separation requirement.
    #[must_use]
    pub fn new(rules: &DesignRules, pitch: Coord) -> Self {
        let geoms = class_geoms(rules);
        let n = geoms.len();
        let mut k = vec![0u32; n];
        let mut rv = vec![0u32; n];
        let d_tt = |a: &ClassGeom, b: &ClassGeom| {
            a.track_radius + b.track_radius + a.clearance.max(b.clearance)
        };
        let d_vt = |v: &ClassGeom, t: &ClassGeom| {
            (v.via_radius + t.track_radius + v.clearance.max(t.clearance))
                .max(v.via_drill / 2 + t.track_radius + rules.hole_clearance)
        };
        let d_vv = |a: &ClassGeom, b: &ClassGeom| {
            (a.via_radius + b.via_radius + a.clearance.max(b.clearance))
                .max(a.via_drill / 2 + b.via_radius + rules.hole_clearance)
                .max(a.via_radius + b.via_drill / 2 + rules.hole_clearance)
        };
        // Requirement (max(ρa, ρb) + 1)·p ≥ D: satisfy each pair by growing the fatter
        // element to `need − 1` when neither side already reaches it.
        loop {
            let mut changed = false;
            for a in 0..n {
                for b in a..n {
                    let need = steps_needed(d_tt(&geoms[a], &geoms[b]), pitch).saturating_sub(1);
                    if k[a].max(k[b]) < need {
                        let fa = geoms[a].track_radius + geoms[a].clearance;
                        let fb = geoms[b].track_radius + geoms[b].clearance;
                        if fa >= fb {
                            k[a] = need;
                        } else {
                            k[b] = need;
                        }
                        changed = true;
                    }
                }
            }
            for a in 0..n {
                for b in 0..n {
                    let need = steps_needed(d_vt(&geoms[a], &geoms[b]), pitch).saturating_sub(1);
                    if rv[a].max(k[b]) < need {
                        rv[a] = need;
                        changed = true;
                    }
                    let need = steps_needed(d_vv(&geoms[a], &geoms[b]), pitch).saturating_sub(1);
                    if rv[a].max(rv[b]) < need {
                        if geoms[a].via_radius >= geoms[b].via_radius {
                            rv[a] = need;
                        } else {
                            rv[b] = need;
                        }
                        changed = true;
                    }
                }
            }
            if !changed {
                break;
            }
        }
        let max_drill = geoms.iter().map(|g| g.via_drill).max().unwrap_or(0);
        let need_h = steps_needed(max_drill + rules.hole_to_hole, pitch);
        let rh = need_h.saturating_sub(1).div_ceil(2);
        let classes = geoms
            .into_iter()
            .zip(k.iter().zip(rv.iter()))
            .map(|(geom, (&k, &rv))| ClassPlan { geom, k, rv })
            .collect();
        Self { pitch, classes, rh }
    }

    /// Largest reservation radius of any element (lattice steps).
    #[must_use]
    pub fn max_radius(&self) -> u32 {
        self.classes.iter().map(|c| c.k.max(c.rv)).max().unwrap_or(0).max(self.rh)
    }
}

/// Default lattice pitch: default-class track width + clearance (the tightest pitch at
/// which adjacent default-class tracks need no extra reservation).
#[must_use]
pub fn default_pitch(rules: &DesignRules) -> Coord {
    let c = &rules.classes[0];
    (c.track_width / 2 * 2 + c.clearance).max(1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use ferroute_board::NetClass;

    #[test]
    fn kicad_defaults_give_minimal_reservations() {
        let rules = DesignRules::default();
        let p = default_pitch(&rules);
        assert_eq!(p, 450_000);
        let plan = Plan::new(&rules, p);
        assert_eq!(plan.classes[0].k, 0);
        assert_eq!(plan.classes[0].rv, 1); // 2·0.45 = 0.9 ≥ 0.3 + 0.125 + 0.2
        assert_eq!(plan.rh, 1); // 3·0.45 ≥ 0.3 + 0.25
    }

    #[test]
    fn fat_class_reserves_more() {
        let mut rules = DesignRules::default();
        rules.classes.push(NetClass {
            name: "Power".into(),
            clearance: 300_000,
            track_width: 500_000,
            via_diameter: 800_000,
            via_drill: 400_000,
        });
        let plan = Plan::new(&rules, default_pitch(&rules));
        assert_eq!(plan.classes[0].k, 0);
        assert_eq!(plan.classes[1].k, 1);
        // Verify every separation requirement numerically.
        let g: Vec<_> = plan.classes.iter().map(|c| c.geom).collect();
        for (a, ca) in plan.classes.iter().enumerate() {
            for (b, cb) in plan.classes.iter().enumerate() {
                let p = i64::from(ca.k.max(cb.k) + 1) * plan.pitch;
                assert!(
                    p >= g[a].track_radius + g[b].track_radius + g[a].clearance.max(g[b].clearance)
                );
                let p = i64::from(ca.rv.max(cb.k) + 1) * plan.pitch;
                assert!(
                    p >= g[a].via_radius + g[b].track_radius + g[a].clearance.max(g[b].clearance)
                );
            }
        }
    }
}
