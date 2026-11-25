// SPDX-License-Identifier: AGPL-3.0-only
//! Current → minimum trace width.
//!
//! * **IPC-2221** (default, unchanged): `I = k·ΔT^0.44·A^0.725`, see
//!   [`crate::constraints::width_for_current`].
//! * **IPC-2152** (**unverified, see CLAUDE.md #20b**): IPC-2152 (2009) replaced the IPC-2221 nomographs with measured charts
//!   (the charts themselves are proprietary). The fit used here is the least-squares power
//!   curve through the IPC-2152 current / temperature-rise / area chart (Brooks-style chart
//!   extraction) that is widely used by calculators:
//!   `A[mil²] = (117.555·ΔT^−0.913 + 1.15)·I^(0.84·ΔT^−0.108 + 1.159)`, `I` in A (RMS/DC),
//!   `ΔT` in °C. The fit is explicit in the area, so no iteration is needed. Its authors quote
//!   about 3 % agreement with the chart at the reference points; **the coefficients were
//!   taken from that secondary source and could not be re-checked against the IPC document
//!   in this environment**, so treat the model as ±15 %. IPC-2152 reports no systematic
//!   internal/external difference for an unplaned board in still air, so one fit serves
//!   both, and copper weight enters through the cross-section only (no separate thickness
//!   modifier; plane/airflow credits are *not* taken, which is conservative).

use crate::constraints::{WIDTH_STEP, width_for_current};
use ferroute_geom::Coord;
use ferroute_geom::detmath::pow;
use serde::{Deserialize, Serialize};

/// Which current-capacity model derives a net's minimum width (`current_model` in the rules
/// file). Default [`CurrentModel::Ipc2221`], so existing outputs do not change.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum CurrentModel {
    /// IPC-2221 power law (historic, conservative on inner layers).
    #[default]
    Ipc2221,
    /// IPC-2152 chart fit (unverified, see CLAUDE.md #20b).
    Ipc2152,
    /// The wider of the two (`"max"`).
    Max,
}

/// Cross-section (mil²) that carries `current_a` amperes at a `delta_t_c` °C rise, per the
/// IPC-2152 chart fit above. `None` for non-positive/non-finite inputs.
#[must_use]
pub fn area_mil2_ipc2152(current_a: f64, delta_t_c: f64) -> Option<f64> {
    if !(current_a.is_finite() && current_a > 0.0 && delta_t_c.is_finite() && delta_t_c > 0.0) {
        return None;
    }
    let a = 117.555 * pow(delta_t_c, -0.913) + 1.15;
    let e = 0.84 * pow(delta_t_c, -0.108) + 1.159;
    Some(a * pow(current_a, e))
}

/// Minimum trace width (nm, rounded up to [`WIDTH_STEP`]) for `current_ma` at `delta_t_c`
/// on copper of `copper_thickness` nm, per the IPC-2152 fit.
#[must_use]
pub fn width_for_current_2152(current_ma: u32, copper_thickness: Coord, delta_t_c: f64) -> Coord {
    if current_ma == 0 || copper_thickness <= 0 {
        return 0;
    }
    let Some(area) = area_mil2_ipc2152(f64::from(current_ma) / 1000.0, delta_t_c) else {
        return 0;
    };
    #[allow(clippy::cast_precision_loss)]
    let thickness_mil = copper_thickness as f64 / 25_400.0;
    #[allow(clippy::cast_possible_truncation)]
    let nm = (area / thickness_mil * 25_400.0).ceil() as Coord;
    (nm + WIDTH_STEP - 1) / WIDTH_STEP * WIDTH_STEP
}

/// Width for `current_ma` under `model` (`external` only matters for IPC-2221).
#[must_use]
pub fn width_for_current_model(
    model: CurrentModel,
    current_ma: u32,
    copper_thickness: Coord,
    external: bool,
    delta_t_c: f64,
) -> Coord {
    let w2221 = || width_for_current(current_ma, copper_thickness, external, delta_t_c);
    let w2152 = || width_for_current_2152(current_ma, copper_thickness, delta_t_c);
    match model {
        CurrentModel::Ipc2221 => w2221(),
        CurrentModel::Ipc2152 => w2152(),
        CurrentModel::Max => w2221().max(w2152()),
    }
}
