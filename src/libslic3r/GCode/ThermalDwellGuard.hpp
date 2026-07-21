#ifndef libslic3r_ThermalDwellGuard_hpp_
#define libslic3r_ThermalDwellGuard_hpp_

// Thermal dwell guard — the dedicated pass that pre-injects filament flushes during
// very-slow extrusion to prevent heat-creep jams. See docs/thermal_dwell_guard/SPEC.md.
//
// Stage 1 (detection only) adds the adapter below: it owns the pure accumulator, translates
// GCodeProcessor's resolved time blocks into accumulator moves, and records where a flush
// WOULD go. It emits comments only — no motion, extrusion, temperature or timing byte of the
// output changes. Injection / scheduling arrive in later stages.

#include "libslic3r/Point.hpp"
#include "libslic3r/GCode/ThermalDwellAccumulator.hpp"

#include <optional>
#include <string>
#include <vector>

namespace Slic3r {

class Print;

// Fallback flush park position, used only when the selected printer's filament-change /
// machine-start G-code exposes no usable flush location. Default = Bambu X1C poop-chute park
// (from "change_filament_gcode": G1 X70 ... Y265). Placeholder for the derivation logic.
// TODO(Stage 2): resolve the real park position + wipe from the printer G-code; fall back to
// this constant only as a last resort.
//
// `inline const`, not `constexpr`: Eigen's Vec2d constructor is not usable in a constant
// expression, so constexpr fails to compile (C2127 on MSVC). This went unnoticed in Stage 0
// because no translation unit included this header; it does now.
inline const Vec2d THERMAL_DWELL_FALLBACK_FLUSH_POS{70.0, 265.0};

// Stage 1 adapter around ThermalDwell::Accumulator.
//
// Lives between GCodeProcessor and the pure state machine: GCodeProcessor hands it one
// resolved move at a time (see TimeMachine::calculate_time), and it records an anchor for
// every flush point the accumulator reports. GCodeProcessor's post-process rewrite then
// turns those anchors into comments, riding the same pass that inserts M73.
class ThermalDwellGuard
{
public:
    struct FlushAnchor
    {
        // TimeBlock::g1_line_id, NOT move_id: move_id is remapped when actual-speed moves are
        // inserted, and process_VG1 never sets it at all. g1_line_id is monotonic and stable,
        // and the rewrite re-derives the identical counter, which is what lets the two phases
        // meet without any stored line table.
        unsigned int g1_line_id{ 0 };
        unsigned int id{ 0 };     // 1..N
        unsigned int layer{ 0 };
        float        z{ 0.0f };   // mm
        float        acc{ 0.0f }; // s
    };

    explicit ThermalDwellGuard(const ThermalDwell::Config &cfg) : m_acc(cfg) {}

    // Resolve the guard's settings from the print, or nullopt when no region enables it.
    //
    // Single source of truth, called by BOTH GCodeProcessor (to decide whether to build a
    // guard at all) and GCode.cpp (to emit the ;THERMAL_DWELL_CONFIG echo), so the echo can
    // never describe something other than what actually ran.
    static std::optional<ThermalDwell::Config> resolve_config(const Print &print);

    // The STAGE1 §5 config echo, including its trailing newline.
    static std::string format_config_comment(const ThermalDwell::Config &cfg);

    void on_move(bool is_extrusion, float duration, float volume, bool in_wave_region,
                 unsigned int g1_line_id, unsigned int layer, float z);

    // Non-decreasing in g1_line_id.
    const std::vector<FlushAnchor>& anchors() const { return m_anchors; }
    float total_slow_flow_seconds() const { return m_acc.total_slow_flow_seconds(); }
    const ThermalDwell::Config& config() const { return m_acc.config(); }

private:
    ThermalDwell::Accumulator m_acc;
    std::vector<FlushAnchor>  m_anchors;
};

} // namespace Slic3r

#endif // libslic3r_ThermalDwellGuard_hpp_
