#ifndef libslic3r_ThermalDwellAccumulator_hpp_
#define libslic3r_ThermalDwellAccumulator_hpp_

// Thermal dwell guard — the accumulator state machine.
// See docs/thermal_dwell_guard/STAGE1.md §4 for the governing table.
//
// This is a PURE state machine: it consumes per-move records and returns "a flush would go
// here" events. It deliberately depends on neither GCodeProcessor nor libslic3r's config
// headers, so it can be unit-tested against synthetic move sequences with nothing else linked.
// ThermalDwellGuard is the adapter that feeds it real slicer data.

#include <optional>

namespace Slic3r {
namespace ThermalDwell {

// Mirror of PrintConfig.hpp's ThermalDwellReset. Duplicated on purpose: it is what keeps this
// translation unit free of the config headers. ThermalDwellGuard::resolve_config() owns the
// translation and is the only place the two may drift.
enum class Reset : int {
    OnTravel,      // any non-extruding move resets
    OnFastSegment, // a single fast-flow segment resets (v1 default)
    OnVolume       // reset once fast-extruded volume >= melt zone volume (most physical)
};

// Standalone as of Stage 2: no `scope` field. The guard watches the whole print by flow rate
// alone, so slow extrusion always accumulates — there is no wave-region gating any more.
struct Config
{
    Reset reset{ Reset::OnFastSegment };
    float q_slow{ 2.0f };   // thermal_dwell_flow_threshold      [mm^3/s]
    float q_fast{ 5.0f };   // thermal_dwell_fast_flow_threshold [mm^3/s]
    float t_max{ 120.0f };  // thermal_dwell_max_time            [s]
    float v_melt{ 15.0f };  // thermal_dwell_melt_zone_volume    [mm^3]
};

// One move as the state machine sees it. `is_extrusion == false` covers travel, retract,
// unretract, pure-Z moves and firmware-synchronize dwells (G4 / M400 / tool change).
//
// `duration` must be the move's OWN accel-aware time. It must never carry a dwell that
// happened around the move — see ThermalDwellGuard's additional_time handling, which feeds
// such dwells as their own synthetic moves instead.
//
// `excluded` marks a move that lies inside an exclusion zone (toolchange / wipe-tower /
// filament-change / custom-gcode block, STAGE2 §6). Accumulation continues across it, but a
// flush may not be ANCHORED on it: a crossing there is deferred to the first non-excluded move.
struct Move
{
    bool  is_extrusion{ false };
    float duration{ 0.0f };      // s
    float volume{ 0.0f };        // mm^3 of filament; 0 unless is_extrusion
    bool  excluded{ false };
};

struct FlushEvent
{
    unsigned int id{ 0 };   // 1..N, global across the print
    float        acc{ 0.0f }; // s, the accumulator value at the crossing (before the reset)
};

class Accumulator
{
public:
    Accumulator() = default;
    explicit Accumulator(const Config &cfg) : m_cfg(cfg) {}

    // Feed one move. Returns an event iff this move pushed acc to or past t_max.
    // At most one event per move: STAGE1 §1 forbids move splitting, so a move that would
    // cross the limit several times over still yields a single flush point.
    std::optional<FlushEvent> step(const Move &move);

    void reset();

    const Config& config() const { return m_cfg; }
    float acc() const { return m_acc; }
    float fast_vol() const { return m_fast_vol; }
    unsigned int flush_count() const { return m_flush_count; }
    // Row-1 seconds only, for the STAGE1 §5 INFO line. Travel time extends an episode
    // (row 7) but is not itself slow-flow time, so it is deliberately not counted here.
    float total_slow_flow_seconds() const { return m_total_slow_flow_s; }

private:
    // `excluded` = the current move sits in an exclusion zone, so a crossing here may not be
    // anchored on it. When that happens the crossing is remembered (m_flush_pending) and fired
    // on the first subsequent non-excluded move instead (STAGE2 §6 deferral).
    std::optional<FlushEvent> check_threshold(bool excluded);

    Config m_cfg;
    float  m_acc{ 0.0f };       // s, >= 0
    float  m_fast_vol{ 0.0f };  // mm^3, used only by Reset::OnVolume
    bool   m_flush_pending{ false }; // a crossing landed in an exclusion zone, awaiting a safe line
    unsigned int m_flush_count{ 0 };
    float  m_total_slow_flow_s{ 0.0f };
};

} // namespace ThermalDwell
} // namespace Slic3r

#endif // libslic3r_ThermalDwellAccumulator_hpp_
