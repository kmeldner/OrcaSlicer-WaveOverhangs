#ifndef libslic3r_ThermalDwellScheduler_hpp_
#define libslic3r_ThermalDwellScheduler_hpp_

// Thermal dwell guard — Stage 3 scheduler.
// See docs/thermal_dwell_guard/STAGE3.md for the governing rules.
//
// Wraps the pure Accumulator and decides WHERE each flush is anchored. Stage 2 anchored every
// flush on the move that crossed t_max (ForcedOnly). The scheduler can instead pull the flush
// back onto a natural break — a layer change or any existing travel — that lies within
// `lookahead` seconds before the crossing (STAGE3 §2: backward look-ahead, so no future
// knowledge is needed), then replays the accumulator from that break (STAGE3 §5).
//
// Like the Accumulator this is a PURE state machine with no GCodeProcessor or config-header
// dependencies, so it is unit-testable against synthetic sequences. ThermalDwellGuard is the
// adapter that feeds it real slicer data and turns its decisions into flush anchors.

#include "libslic3r/GCode/ThermalDwellAccumulator.hpp"

#include <deque>
#include <vector>

namespace Slic3r {
namespace ThermalDwell {

// Mirror of PrintConfig.hpp's ThermalDwellSchedule (same duplication rationale as Reset).
enum class Schedule : int {
    ForcedOnly,        // flush exactly at the crossing (Stage 2 behaviour)
    PreferLayerChange, // snap to the latest layer-change travel within lookahead
    PreferAnyTravel    // snap to the latest layer-change travel, else the latest travel
};

// How a flush ended up where it is (STAGE3 §7 `sched=`).
enum class AnchorKind : int { Forced, LayerChange, Travel };

struct ScheduleConfig
{
    Schedule mode{ Schedule::PreferAnyTravel };
    float    lookahead{ 15.0f }; // s, thermal_dwell_lookahead
};

// The machine state at the end of a move that a flush anchored there must restore (STAGE2 §4),
// plus the bookkeeping the rewrite needs to find the line. Plain data: the scheduler stores it
// per buffered move and hands the chosen one back unchanged.
struct AnchorState
{
    // TimeBlock::g1_line_id, NOT move_id: move_id is remapped when actual-speed moves are
    // inserted, and process_VG1 never sets it at all. g1_line_id is monotonic and stable, and
    // the rewrite re-derives the identical counter, which is what lets the two phases meet
    // without any stored line table.
    unsigned int g1_line_id{ 0 };
    unsigned int layer{ 0 };
    float        z{ 0.0f };          // mm
    float        x{ 0.0f };          // mm, nozzle X at end of move
    float        y{ 0.0f };          // mm, nozzle Y
    float        e{ 0.0f };          // logical E (filament mm)
    bool         e_relative{ true }; // true = M83 relative E, false = M82 absolute E
    bool         retracted{ false }; // filament retracted after the move
    float        feedrate{ 0.0f };   // modal feedrate F [mm/min]
    unsigned int tool{ 0 };          // active tool
};

// One flush decision.
struct Scheduled
{
    AnchorState  at;                      // where the flush is anchored
    unsigned int id{ 0 };                 // 1..N, assigned by the scheduler
    float        acc{ 0.0f };             // accumulator value at the anchor [s]
    float        frac{ 0.0f };            // dwell fraction at the anchor (STAGE4 §2)
    AnchorKind   kind{ AnchorKind::Forced };
    float        early{ 0.0f };           // seconds the flush was pulled before the crossing
};

class Scheduler
{
public:
    Scheduler() = default;
    Scheduler(const Config &cfg, const ScheduleConfig &sched) : m_acc(cfg), m_sched(sched) {}

    // Feed one move. `is_travel` marks a non-extruding move with displacement (EMoveType::Travel);
    // the scheduler additionally requires an XY displacement for it to be a break (STAGE3 §3:
    // GCodeProcessor files pure-Z hops as Travel too). Returns the flushes decided by this move, in anchor order:
    // usually none, one at a crossing, and rarely two (a scheduled one plus a forced one from a
    // replay re-crossing, STAGE3 §5).
    std::vector<Scheduled> step(const Move &move, bool is_travel, const AnchorState &state);

    const Accumulator&    accumulator() const { return m_acc; }
    const Config&         config() const { return m_acc.config(); }
    const ScheduleConfig& schedule_config() const { return m_sched; }
    float                 now() const { return m_t; }
    unsigned int          flush_count() const { return m_count; }
    size_t                buffered() const { return m_buf.size(); }

private:
    struct Rec
    {
        Move        move;
        AnchorState st;
        AnchorKind  brk{ AnchorKind::Forced }; // Forced == "not a break"
        float       t_end{ 0.0f };            // print time at the end of the move
        float       acc_after{ 0.0f };        // accumulator value right after the move
        float       frac_after{ 0.0f };       // dwell fraction right after the move
    };

    AnchorKind classify_break(const Move &move, bool is_travel, const AnchorState &state);
    // Pick the break to pull the flush onto, or nullptr for a forced anchor (STAGE3 §4).
    int choose_break(float t_now) const;
    // STAGE3 §5: re-run the moves after `from` (exclusive) through a fresh accumulator state,
    // emitting forced anchors for any re-crossing, and install the result as the live state.
    void replay_after(size_t from_index, std::vector<Scheduled> &out);
    Scheduled make(const AnchorState &st, float acc, float frac, AnchorKind kind, float early);

    Accumulator     m_acc;
    ScheduleConfig  m_sched;
    float           m_t{ 0.0f };          // running print time [s]
    unsigned int    m_count{ 0 };         // flushes emitted
    std::deque<Rec> m_buf;                // current episode, newest at the back
    bool            m_have_layer{ false };
    unsigned int    m_layer{ 0 };         // layer of the previous move fed
    float           m_prev_x{ 0.0f };     // XY at the end of the previous move: a break must
    float           m_prev_y{ 0.0f };     // actually move in XY (pure-Z hops are not breaks)
    bool            m_layer_change_pending{ false }; // a new layer began; its first travel is a layer_change break
    bool            m_have_anchor{ false };
    unsigned int    m_last_anchor_g1{ 0 }; // anchors must stay non-decreasing in g1_line_id
};

} // namespace ThermalDwell
} // namespace Slic3r

#endif // libslic3r_ThermalDwellScheduler_hpp_
