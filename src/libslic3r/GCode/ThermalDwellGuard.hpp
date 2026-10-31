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
// The adapter below owns the pure scheduler (which owns the pure accumulator), translates
// GCodeProcessor's resolved time blocks into scheduler moves, records where a flush must go
// together with the machine state at that anchor, and formats the real flush sequence that is
// spliced in during the post-process rewrite (the same pass that inserts M73). Stage 3 made
// thermal_dwell_schedule_mode / thermal_dwell_lookahead live: a flush may be pulled early onto
// a layer change or an existing travel (docs/thermal_dwell_guard/STAGE3.md).

#include "libslic3r/Point.hpp"
#include "libslic3r/GCode/ThermalDwellAccumulator.hpp"
#include "libslic3r/GCode/ThermalDwellScheduler.hpp"

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

// Stage 3 adapter around ThermalDwell::Scheduler (which wraps ThermalDwell::Accumulator).
class ThermalDwellGuard
{
public:
    // One recorded flush: the STAGE2 §4 machine state at the anchor line (AnchorState — the
    // state downstream lines see, which the flush must restore exactly: Invariant 2) plus the
    // marker bookkeeping.
    struct FlushAnchor : public ThermalDwell::AnchorState
    {
        unsigned int            id{ 0 };     // 1..N
        float                   acc{ 0.0f }; // s, accumulator value at the anchor
        ThermalDwell::AnchorKind kind{ ThermalDwell::AnchorKind::Forced }; // STAGE3 §7 sched=
        float                   early{ 0.0f }; // s pulled before the crossing (0 when forced)
    };

    ThermalDwellGuard(const ThermalDwell::Config &cfg, const ThermalDwell::ScheduleConfig &sched,
                      const ThermalDwellFlushParams &flush)
        : m_sched(cfg, sched), m_flush(flush) {}
    // Convenience for callers (tests) that do not care about scheduling: ForcedOnly.
    ThermalDwellGuard(const ThermalDwell::Config &cfg, const ThermalDwellFlushParams &flush)
        : ThermalDwellGuard(cfg, forced_only(), flush) {}

    static ThermalDwell::ScheduleConfig forced_only()
    {
        ThermalDwell::ScheduleConfig s;
        s.mode = ThermalDwell::Schedule::ForcedOnly;
        return s;
    }

    // Resolve the guard's accumulator settings from the print, or nullopt when no region
    // enables it. Single source of truth, called by BOTH GCodeProcessor (to decide whether to
    // build a guard) and GCode.cpp (to emit the ;THERMAL_DWELL_CONFIG echo).
    static std::optional<ThermalDwell::Config> resolve_config(const Print &print);
    // The Group-4 scheduling keys from the same first guard-enabled region (STAGE3).
    static ThermalDwell::ScheduleConfig resolve_schedule(const Print &print);
    // The Group-3 + reused-config values for the flush, from the same first guard-enabled
    // region. Returns defaults filled from the print even when the guard is off (callers gate
    // on resolve_config()).
    static ThermalDwellFlushParams resolve_flush_params(const Print &print);

    // The STAGE1/§7 config echo (drops scope, adds the Group-3 values and — Stage 3 — the
    // Group-4 scheduling values), including its newline.
    static std::string format_config_comment(const ThermalDwell::Config &cfg,
                                             const ThermalDwell::ScheduleConfig &sched,
                                             const ThermalDwellFlushParams &flush);

    // `is_travel`: the move is a real XY travel (EMoveType::Travel) — the only kind that can be
    // a scheduling break (STAGE3 §3). Everything else is as in Stage 2.
    void on_move(bool is_extrusion, bool is_travel, float duration, float volume, bool excluded,
                 unsigned int g1_line_id, unsigned int layer,
                 float x, float y, float z, float e, bool e_relative,
                 bool retracted, float feedrate_mmmin, unsigned int tool);

    // The full spliced block for one anchor: the kept ;THERMAL_DWELL_FLUSH_POINT line, then the
    // ;THERMAL_DWELL_FLUSH_START ... §5 sequence ... ;THERMAL_DWELL_FLUSH_END. Each returned
    // string is one line and ends in '\n', ready for ExportLines::append_line.
    std::vector<std::string> format_flush_lines(const FlushAnchor &a) const;
    // The end-of-file SUMMARY block (§7 totals), including per-point lines.
    std::string format_summary() const;

    static const char* anchor_kind_name(ThermalDwell::AnchorKind kind);
    static const char* schedule_name(ThermalDwell::Schedule mode);

    // Non-decreasing in g1_line_id.
    const std::vector<FlushAnchor>& anchors() const { return m_anchors; }
    float total_slow_flow_seconds() const { return m_sched.accumulator().total_slow_flow_seconds(); }
    const ThermalDwell::Config& config() const { return m_sched.config(); }
    const ThermalDwell::ScheduleConfig& schedule_config() const { return m_sched.schedule_config(); }
    const ThermalDwellFlushParams& flush_params() const { return m_flush; }

private:
    ThermalDwell::Scheduler   m_sched;
    ThermalDwellFlushParams   m_flush;
    std::vector<FlushAnchor>  m_anchors;
};

} // namespace Slic3r

#endif // libslic3r_ThermalDwellGuard_hpp_
