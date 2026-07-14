#ifndef libslic3r_ThermalDwellGuard_hpp_
#define libslic3r_ThermalDwellGuard_hpp_

// Thermal dwell guard — future home of the dedicated G-code post-processing pass that
// pre-injects filament flushes during very-slow extrusion to prevent heat-creep jams.
// See docs/thermal_dwell_guard/SPEC.md.
//
// Stage 0 (config + scaffolding): this header intentionally contains no runtime logic — only
// the fallback flush position below. Nothing here is referenced by the slicing pipeline yet,
// so sliced G-code is unaffected. Detection / injection / scheduling arrive in later stages.

#include "libslic3r/Point.hpp"

namespace Slic3r {

// Fallback flush park position, used only when the selected printer's filament-change /
// machine-start G-code exposes no usable flush location. Default = Bambu X1C poop-chute park
// (from "change_filament_gcode": G1 X70 ... Y265). Placeholder for the derivation logic.
// TODO(Stage 2): resolve the real park position + wipe from the printer G-code; fall back to
// this constant only as a last resort.
static constexpr Vec2d THERMAL_DWELL_FALLBACK_FLUSH_POS{70.0, 265.0};

} // namespace Slic3r

#endif // libslic3r_ThermalDwellGuard_hpp_
