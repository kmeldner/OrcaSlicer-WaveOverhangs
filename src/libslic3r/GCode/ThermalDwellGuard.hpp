#ifndef libslic3r_ThermalDwellGuard_hpp_
#define libslic3r_ThermalDwellGuard_hpp_

// Thermal dwell guard — the dedicated pass that pre-injects filament flushes during
// very-slow extrusion to prevent heat-creep jams. See docs/thermal_dwell_guard/SPEC.md.
//
// Standalone module (Stage 2): it catches slow extrusion from ANY cause — walls, infill,
// bridges, overhangs, wave overhangs — purely from each move's flow rate. It does not read or
// depend on the fork's wave-overhang markers or code at all, so it is portable to stock
// OrcaSlicer.
//
// The adapter below owns the pure accumulator, translates GCodeProcessor's resolved time
// blocks into accumulator moves, records where a flush must go together with the machine state
// at that anchor, and — Stage 2 — formats the real flush sequence that is spliced in during the
// post-process rewrite (the same pass that inserts M73). ForcedOnly semantics:
// thermal_dwell_schedule_mode / thermal_dwell_lookahead are still not read.

#include "libslic3r/Point.hpp"
#include "libslic3r/GCode/ThermalDwellAccumulator.hpp"

#include <optional>
#include <string>
#include <vector>

namespace Slic3r {

class Print;

// Fallback flush park position. Default = Bambu X1C poop-chute park, extracted from the bundled
// profile "resources/profiles/BBL/machine/Bambu Lab X1 Carbon 0.4 nozzle.json"
// (change_filament_gcode: G1 X70 ... / G1 Y245 -> Y265). Also the default of the
// thermal_dwell_flush_position config option; keep the two in sync.
//
// `inline const`, not `constexpr`: Eigen's Vec2d constructor is not usable in a constant
// expression, so constexpr fails to compile (C2127 on MSVC).
inline const Vec2d THERMAL_DWELL_FALLBACK_FLUSH_POS{70.0, 265.0};

// Everything the flush sequence needs beyond the accumulator config. Resolved once from the
// print (Group-3 keys + the reused retraction / travel / filament settings). v1 is
// single-extruder / single-material, so the reused per-extruder values are taken for the
// active tool of the guard-enabled region (extruder 0 in practice).
struct ThermalDwellFlushParams
{
    Vec2d flush_pos{ 70.0, 265.0 }; // thermal_dwell_flush_position (machine flush/wipe XY)
    float z_lift{ 1.0f };           // thermal_dwell_flush_z_lift            [mm]
    float purge_volume{ 25.0f };    // thermal_dwell_purge_volume            [mm^3]
    float purge_flow{ 12.0f };      // thermal_dwell_purge_flow              [mm^3/s]
    bool  wipe{ true };             // thermal_dwell_wipe
    float retract_length{ 0.8f };   // retraction_length[tool]               [mm]
    float retract_speed{ 30.0f };   // retraction_speed[tool]                [mm/s]
    float deretract_speed{ 30.0f }; // deretraction_speed[tool]              [mm/s]
    float travel_speed{ 150.0f };   // travel_speed                          [mm/s]
    float filament_area{ 2.405f };  // pi*(filament_diameter/2)^2            [mm^2]
};

// Stage 2 adapter around ThermalDwell::Accumulator.
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

        // STAGE2 §4 — machine state at the anchor line (the state downstream lines see). The
        // flush must restore all of it exactly (Invariant 2 — state neutrality).
        float        x{ 0.0f };          // mm, nozzle X at end of anchor move
        float        y{ 0.0f };          // mm, nozzle Y
        float        e{ 0.0f };          // logical E at anchor (filament mm)
        bool         e_relative{ true }; // true = M83 relative E, false = M82 absolute E
        bool         retracted{ false }; // filament retracted at the anchor
        float        feedrate{ 0.0f };   // modal feedrate F at anchor [mm/min]
        unsigned int tool{ 0 };          // active tool
    };

    ThermalDwellGuard(const ThermalDwell::Config &cfg, const ThermalDwellFlushParams &flush)
        : m_acc(cfg), m_flush(flush) {}

    // Resolve the guard's accumulator settings from the print, or nullopt when no region
    // enables it. Single source of truth, called by BOTH GCodeProcessor (to decide whether to
    // build a guard) and GCode.cpp (to emit the ;THERMAL_DWELL_CONFIG echo).
    static std::optional<ThermalDwell::Config> resolve_config(const Print &print);
    // The Group-3 + reused-config values for the flush, from the same first guard-enabled
    // region. Returns defaults filled from the print even when the guard is off (callers gate
    // on resolve_config()).
    static ThermalDwellFlushParams resolve_flush_params(const Print &print);

    // The STAGE1/§7 config echo (drops scope, adds the Group-3 values), including its newline.
    static std::string format_config_comment(const ThermalDwell::Config &cfg,
                                             const ThermalDwellFlushParams &flush);

    void on_move(bool is_extrusion, float duration, float volume, bool excluded,
                 unsigned int g1_line_id, unsigned int layer,
                 float x, float y, float z, float e, bool e_relative,
                 bool retracted, float feedrate_mmmin, unsigned int tool);

    // The full spliced block for one anchor: the kept ;THERMAL_DWELL_FLUSH_POINT line, then the
    // ;THERMAL_DWELL_FLUSH_START ... §5 sequence ... ;THERMAL_DWELL_FLUSH_END. Each returned
    // string is one line and ends in '\n', ready for ExportLines::append_line.
    std::vector<std::string> format_flush_lines(const FlushAnchor &a) const;
    // The end-of-file SUMMARY block (§7 totals), including per-point lines.
    std::string format_summary() const;

    // Non-decreasing in g1_line_id.
    const std::vector<FlushAnchor>& anchors() const { return m_anchors; }
    float total_slow_flow_seconds() const { return m_acc.total_slow_flow_seconds(); }
    const ThermalDwell::Config& config() const { return m_acc.config(); }
    const ThermalDwellFlushParams& flush_params() const { return m_flush; }

private:
    ThermalDwell::Accumulator m_acc;
    ThermalDwellFlushParams   m_flush;
    std::vector<FlushAnchor>  m_anchors;
};

} // namespace Slic3r

#endif // libslic3r_ThermalDwellGuard_hpp_
