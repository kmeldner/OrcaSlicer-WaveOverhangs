#include <catch2/catch_all.hpp>

#include "libslic3r/GCode/ThermalDwellAccumulator.hpp"
#include "libslic3r/GCode/ThermalDwellGuard.hpp"
#include "libslic3r/GCode/ThermalDwellScheduler.hpp"

#include <string>
#include <vector>

using namespace Slic3r::ThermalDwell;
using Slic3r::ThermalDwellGuard;
using Slic3r::ThermalDwellFlushParams;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::ContainsSubstring;

// Unit tests for the thermal dwell guard (docs/thermal_dwell_guard/STAGE1.md §4 +
// STAGE2.md §5/§9b). The accumulator is a pure, standalone state machine (no scope as of
// Stage 2); the flush sequence generator is a pure function of a synthetic anchor + params.

namespace {

// Small, explicit defaults so each test only states what it actually cares about.
Config make_config()
{
    Config cfg;
    cfg.reset = Reset::OnFastSegment;
    cfg.q_slow = 2.0f;
    cfg.q_fast = 5.0f;
    cfg.t_max = 10.0f;
    cfg.v_melt = 15.0f;
    return cfg;
}

// q = volume/duration = 1.0 mm^3/s, comfortably at or below q_slow.
Move slow_move(float duration = 1.0f, bool excluded = false)
{
    Move m;
    m.is_extrusion = true;
    m.duration = duration;
    m.volume = duration; // q = 1.0
    m.excluded = excluded;
    return m;
}

Move fast_move(float volume)
{
    Move m;
    m.is_extrusion = true;
    m.duration = 0.5f;
    m.volume = volume; // q = 2*volume, >= q_fast for volume >= 2.5
    return m;
}

Move travel_move(float duration, bool excluded = false)
{
    Move m;
    m.is_extrusion = false;
    m.duration = duration;
    m.volume = 0.0f;
    m.excluded = excluded;
    return m;
}

} // namespace

TEST_CASE("Thermal dwell: slow extrusion crosses the limit at t_max", "[ThermalDwell]")
{
    Accumulator acc(make_config());

    for (int i = 0; i < 9; ++i)
        REQUIRE_FALSE(acc.step(slow_move()).has_value());
    REQUIRE_THAT(acc.acc(), WithinAbs(9.0f, 1e-5f));

    const auto ev = acc.step(slow_move());
    REQUIRE(ev.has_value());
    REQUIRE(ev->id == 1);
    REQUIRE_THAT(ev->acc, WithinAbs(10.0f, 1e-5f));
}

TEST_CASE("Thermal dwell: slow extrusion always accumulates (standalone, no scope)", "[ThermalDwell]")
{
    // Stage 2 removed the scope gate: the guard watches the whole print, so every slow move
    // accumulates regardless of where it is. (Rows 1/2 of the STAGE1 table merged into one.)
    Accumulator acc(make_config());
    acc.step(slow_move());
    REQUIRE_THAT(acc.acc(), WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("Thermal dwell: a fast segment resets the accumulator (OnFastSegment)", "[ThermalDwell]")
{
    Accumulator acc(make_config());
    for (int i = 0; i < 5; ++i)
        acc.step(slow_move());
    REQUIRE_THAT(acc.acc(), WithinAbs(5.0f, 1e-5f));

    REQUIRE_FALSE(acc.step(fast_move(6.0f)).has_value());
    REQUIRE_THAT(acc.acc(), WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(acc.fast_vol(), WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Thermal dwell: OnVolume resets only once the melt zone is exchanged", "[ThermalDwell]")
{
    Config cfg = make_config();
    cfg.reset = Reset::OnVolume; // v_melt = 15
    Accumulator acc(cfg);

    for (int i = 0; i < 5; ++i)
        acc.step(slow_move());
    REQUIRE_THAT(acc.acc(), WithinAbs(5.0f, 1e-5f));

    acc.step(fast_move(6.0f));
    REQUIRE_THAT(acc.fast_vol(), WithinAbs(6.0f, 1e-5f));
    REQUIRE_THAT(acc.acc(), WithinAbs(5.0f, 1e-5f));

    acc.step(fast_move(6.0f));
    REQUIRE_THAT(acc.fast_vol(), WithinAbs(12.0f, 1e-5f));

    acc.step(fast_move(6.0f)); // 12 + 6 >= 15
    REQUIRE_THAT(acc.acc(), WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(acc.fast_vol(), WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Thermal dwell: a travel extends an existing episode (row 7)", "[ThermalDwell]")
{
    Accumulator acc(make_config());
    for (int i = 0; i < 5; ++i)
        acc.step(slow_move());
    acc.step(travel_move(3.0f));
    REQUIRE_THAT(acc.acc(), WithinAbs(8.0f, 1e-5f));
}

TEST_CASE("Thermal dwell: a travel never starts an episode (row 8)", "[ThermalDwell]")
{
    Accumulator acc(make_config());
    REQUIRE_FALSE(acc.step(travel_move(100.0f)).has_value());
    REQUIRE_THAT(acc.acc(), WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Thermal dwell: OnTravel resets on any non-extruding move (row 6)", "[ThermalDwell]")
{
    Config cfg = make_config();
    cfg.reset = Reset::OnTravel;
    Accumulator acc(cfg);
    for (int i = 0; i < 5; ++i)
        acc.step(slow_move());
    acc.step(travel_move(1.0f));
    REQUIRE_THAT(acc.acc(), WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Thermal dwell: hysteresis-band flow changes nothing (row 3)", "[ThermalDwell]")
{
    Config cfg = make_config();
    cfg.reset = Reset::OnVolume;
    Accumulator acc(cfg);

    for (int i = 0; i < 5; ++i)
        acc.step(slow_move());
    acc.step(fast_move(3.0f)); // q = 6 >= q_fast; partial exchange leaves fast_vol = 3
    REQUIRE_THAT(acc.acc(), WithinAbs(5.0f, 1e-5f));
    REQUIRE_THAT(acc.fast_vol(), WithinAbs(3.0f, 1e-5f));

    Move band;
    band.is_extrusion = true;
    band.duration = 1.0f;
    band.volume = 3.5f; // q = 3.5, strictly between q_slow and q_fast
    for (int i = 0; i < 10; ++i)
        REQUIRE_FALSE(acc.step(band).has_value());

    REQUIRE_THAT(acc.acc(), WithinAbs(5.0f, 1e-5f));
    REQUIRE_THAT(acc.fast_vol(), WithinAbs(3.0f, 1e-5f));
}

TEST_CASE("Thermal dwell: the accumulator resets after a flush point", "[ThermalDwell]")
{
    Accumulator acc(make_config());
    for (int i = 0; i < 9; ++i)
        acc.step(slow_move());
    const auto first = acc.step(slow_move());
    REQUIRE(first.has_value());
    REQUIRE(first->id == 1);
    REQUIRE_THAT(acc.acc(), WithinAbs(0.0f, 1e-5f));

    for (int i = 0; i < 9; ++i)
        REQUIRE_FALSE(acc.step(slow_move()).has_value());
    const auto second = acc.step(slow_move());
    REQUIRE(second.has_value());
    REQUIRE(second->id == 2);
    REQUIRE(acc.flush_count() == 2);
}

TEST_CASE("Thermal dwell: slow flow clears fast_vol", "[ThermalDwell]")
{
    Config cfg = make_config();
    cfg.reset = Reset::OnVolume;
    Accumulator acc(cfg);

    acc.step(fast_move(6.0f));
    REQUIRE_THAT(acc.fast_vol(), WithinAbs(6.0f, 1e-5f));

    acc.step(slow_move()); // interrupts the partial fast exchange
    REQUIRE_THAT(acc.fast_vol(), WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Thermal dwell: total slow-flow seconds counts only slow extrusion", "[ThermalDwell]")
{
    Config cfg = make_config();
    cfg.t_max = 100.0f;
    Accumulator acc(cfg);

    acc.step(slow_move(2.0f));
    acc.step(travel_move(5.0f)); // extends the episode, but is not slow-flow time
    acc.step(slow_move(3.0f));

    REQUIRE_THAT(acc.total_slow_flow_seconds(), WithinAbs(5.0f, 1e-5f));
    REQUIRE_THAT(acc.acc(), WithinAbs(10.0f, 1e-5f));
}

TEST_CASE("Thermal dwell: a crossing inside an exclusion zone is deferred (STAGE2 §6)", "[ThermalDwell]")
{
    Accumulator acc(make_config());

    // Reach the limit exactly, but on an EXCLUDED move: no event yet, and acc is not reset.
    for (int i = 0; i < 9; ++i)
        acc.step(slow_move());
    REQUIRE_FALSE(acc.step(slow_move(1.0f, /*excluded=*/true)).has_value());
    REQUIRE_THAT(acc.acc(), WithinAbs(10.0f, 1e-5f)); // held, not flushed

    // Still excluded on the next move -> still deferred.
    REQUIRE_FALSE(acc.step(travel_move(1.0f, /*excluded=*/true)).has_value());

    // First non-excluded move fires the pending flush and resets.
    const auto ev = acc.step(travel_move(1.0f, /*excluded=*/false));
    REQUIRE(ev.has_value());
    REQUIRE(ev->id == 1);
    REQUIRE_THAT(acc.acc(), WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Thermal dwell: a crossing outside any zone fires immediately", "[ThermalDwell]")
{
    Accumulator acc(make_config());
    for (int i = 0; i < 9; ++i)
        acc.step(slow_move());
    const auto ev = acc.step(slow_move(1.0f, /*excluded=*/false));
    REQUIRE(ev.has_value());
    REQUIRE(ev->id == 1);
}

// ---------------------------------------------------------------------------------------------
// Flush sequence generator (STAGE2 §5). Pure function of a synthetic anchor + params.

namespace {

ThermalDwellFlushParams make_flush_params()
{
    ThermalDwellFlushParams p;
    p.flush_pos = Slic3r::Vec2d(70.0, 265.0);
    p.z_lift = 1.0f;
    p.purge_volume = 25.0f;
    p.purge_flow = 12.0f;
    p.wipe = true;
    p.retract_length = 0.8f;
    p.retract_speed = 30.0f;
    p.deretract_speed = 30.0f;
    p.travel_speed = 150.0f;
    p.filament_area = float(M_PI) * 0.875f * 0.875f; // 1.75 mm filament
    return p;
}

ThermalDwellGuard::FlushAnchor make_anchor(bool e_relative, bool retracted)
{
    ThermalDwellGuard::FlushAnchor a;
    a.id = 1; a.layer = 5; a.z = 2.5f; a.acc = 10.0f;
    a.x = 100.0f; a.y = 90.0f;
    a.e = 123.456f;
    a.e_relative = e_relative;
    a.retracted = retracted;
    a.feedrate = 1800.0f; // mm/min
    a.tool = 0;
    return a;
}

// Count G0/G1 lines that lack an F word.
int moves_without_F(const std::vector<std::string>& lines)
{
    int n = 0;
    for (const std::string& l : lines) {
        if ((l.rfind("G1 ", 0) == 0 || l.rfind("G0 ", 0) == 0) && l.find('F') == std::string::npos)
            ++n;
    }
    return n;
}

bool any_contains(const std::vector<std::string>& lines, const std::string& needle)
{
    for (const std::string& l : lines)
        if (l.find(needle) != std::string::npos)
            return true;
    return false;
}

} // namespace

TEST_CASE("Thermal dwell flush: relative-E block structure and neutrality", "[ThermalDwell]")
{
    ThermalDwellGuard guard(make_config(), make_flush_params());
    const auto lines = guard.format_flush_lines(make_anchor(/*e_relative=*/true, /*retracted=*/false));

    REQUIRE_THAT(lines.front(), ContainsSubstring(";THERMAL_DWELL_FLUSH_POINT id=1"));
    REQUIRE_THAT(lines[1], ContainsSubstring(";THERMAL_DWELL_FLUSH_START id=1"));
    REQUIRE_THAT(lines.back(), ContainsSubstring(";THERMAL_DWELL_FLUSH_END id=1"));

    // Every inserted motion carries an explicit F.
    REQUIRE(moves_without_F(lines) == 0);

    // Travels to the chute XY and returns to the anchor XY.
    REQUIRE(any_contains(lines, "G1 X70.000"));
    REQUIRE(any_contains(lines, "G1 Y265.000"));
    REQUIRE(any_contains(lines, "G1 X100.000"));
    REQUIRE(any_contains(lines, "G1 Y90.000"));
    // Z-hop then restore.
    REQUIRE(any_contains(lines, "G1 Z3.500")); // z + lift
    REQUIRE(any_contains(lines, "G1 Z2.500")); // restore

    // Relative E: no G92 fixup, and the final line restores the modal feedrate.
    REQUIRE_FALSE(any_contains(lines, "G92"));
    REQUIRE(any_contains(lines, "G1 F1800"));

    // Wipe enabled -> at least one sweep line at the flush Y.
    REQUIRE(any_contains(lines, "Y265.000 F5000"));
}

TEST_CASE("Thermal dwell flush: absolute-E block carries the mandatory G92 fixup", "[ThermalDwell]")
{
    ThermalDwellGuard guard(make_config(), make_flush_params());
    const auto lines = guard.format_flush_lines(make_anchor(/*e_relative=*/false, /*retracted=*/false));

    REQUIRE(moves_without_F(lines) == 0);
    // Absolute E must be pinned back to the anchor's logical E.
    REQUIRE(any_contains(lines, "G92 E123.456"));
}

TEST_CASE("Thermal dwell flush: a retracted anchor skips the extra retract/unretract", "[ThermalDwell]")
{
    ThermalDwellGuard guard(make_config(), make_flush_params());

    const auto unretr = guard.format_flush_lines(make_anchor(true, /*retracted=*/false));
    const auto retr   = guard.format_flush_lines(make_anchor(true, /*retracted=*/true));

    // The retracted-anchor block omits step 1 (initial retract) and step 10 (final unretract),
    // so it has exactly two fewer E moves than the unretracted-anchor block.
    auto count_E = [](const std::vector<std::string>& ls) {
        int n = 0;
        for (const std::string& l : ls)
            if (l.rfind("G1 E", 0) == 0) ++n;
        return n;
    };
    REQUIRE(count_E(unretr) - count_E(retr) == 2);
}

TEST_CASE("Thermal dwell flush: wipe disabled emits no wiper sweep", "[ThermalDwell]")
{
    ThermalDwellFlushParams p = make_flush_params();
    p.wipe = false;
    ThermalDwellGuard guard(make_config(), p);
    const auto lines = guard.format_flush_lines(make_anchor(true, false));

    REQUIRE_FALSE(any_contains(lines, "Y265.000 F5000"));
    REQUIRE(moves_without_F(lines) == 0);
}

// ---------------------------------------------------------------------------------------------
// Stage 3 scheduler (STAGE3 §3–§6). Pure state machine over synthetic moves.

namespace {

using Slic3r::ThermalDwell::Scheduler;
using Slic3r::ThermalDwell::ScheduleConfig;
using Slic3r::ThermalDwell::Schedule;
using Slic3r::ThermalDwell::AnchorKind;
using Slic3r::ThermalDwell::AnchorState;
using Slic3r::ThermalDwell::Scheduled;

ScheduleConfig make_sched(Schedule mode, float lookahead = 5.0f)
{
    ScheduleConfig s;
    s.mode = mode;
    s.lookahead = lookahead;
    return s;
}

// Each fed move gets a fresh g1 line id and carries the layer, so anchors are distinguishable.
struct Feeder
{
    Scheduler sched;
    unsigned int g1{ 0 };
    unsigned int layer{ 1 };
    std::vector<Scheduled> fired;

    explicit Feeder(const ScheduleConfig &s) : sched(make_config(), s) {}

    AnchorState state()
    {
        AnchorState st;
        st.g1_line_id = ++g1;
        st.layer = layer;
        st.x = float(g1);
        return st;
    }
    void slow(float d = 1.0f)             { fire(sched.step(slow_move(d), false, state())); }
    void travel(float d = 1.0f, bool excluded = false)
    {
        fire(sched.step(travel_move(d, excluded), /*is_travel=*/true, state()));
    }
    void retract(float d = 0.1f)          { fire(sched.step(travel_move(d), /*is_travel=*/false, state())); }
    void fast()                           { fire(sched.step(fast_move(6.0f), false, state())); }
    void fire(std::vector<Scheduled> v)   { fired.insert(fired.end(), v.begin(), v.end()); }
};

} // namespace

TEST_CASE("Thermal dwell schedule: ForcedOnly anchors on the crossing move", "[ThermalDwell]")
{
    Feeder f(make_sched(Schedule::ForcedOnly));
    for (int i = 0; i < 5; ++i) f.slow();
    f.travel(); // in window, but ForcedOnly ignores it
    for (int i = 0; i < 4; ++i) f.slow(); // acc = 5 + 1 (travel extends) + 4 = 10 -> crossing

    REQUIRE(f.fired.size() == 1);
    REQUIRE(f.fired[0].kind == AnchorKind::Forced);
    REQUIRE(f.fired[0].at.g1_line_id == f.g1); // the crossing move itself
    REQUIRE_THAT(f.fired[0].acc, WithinAbs(10.0f, 1e-5f));
    REQUIRE_THAT(f.fired[0].early, WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(f.sched.accumulator().acc(), WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Thermal dwell schedule: a travel inside the window takes the flush, acc is replayed", "[ThermalDwell]")
{
    Feeder f(make_sched(Schedule::PreferAnyTravel, /*lookahead=*/5.0f));
    for (int i = 0; i < 5; ++i) f.slow();          // acc 5, t 5
    f.travel();                                     // acc 6, t 6  <- the break (g1 = 6)
    const unsigned int travel_g1 = f.g1;
    for (int i = 0; i < 4; ++i) f.slow();           // acc 10 at t 10 -> crossing, 4 s after the break

    REQUIRE(f.fired.size() == 1);
    const Scheduled &s = f.fired[0];
    REQUIRE(s.kind == AnchorKind::Travel);
    REQUIRE(s.at.g1_line_id == travel_g1);
    REQUIRE_THAT(s.acc, WithinAbs(6.0f, 1e-5f));   // accumulator value at the break
    REQUIRE_THAT(s.early, WithinAbs(4.0f, 1e-5f)); // pulled 4 s early, within lookahead 5
    // Replay: the 4 slow seconds after the break are now the live episode.
    REQUIRE_THAT(f.sched.accumulator().acc(), WithinAbs(4.0f, 1e-5f));
}

TEST_CASE("Thermal dwell schedule: a break older than the look-ahead is not used", "[ThermalDwell]")
{
    Feeder f(make_sched(Schedule::PreferAnyTravel, /*lookahead=*/3.0f));
    for (int i = 0; i < 5; ++i) f.slow();
    f.travel();                                     // t 6
    for (int i = 0; i < 4; ++i) f.slow();           // crossing at t 10, 4 s after -> outside 3 s window

    REQUIRE(f.fired.size() == 1);
    REQUIRE(f.fired[0].kind == AnchorKind::Forced);
    REQUIRE_THAT(f.fired[0].early, WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Thermal dwell schedule: the latest travel wins", "[ThermalDwell]")
{
    Feeder f(make_sched(Schedule::PreferAnyTravel, /*lookahead=*/10.0f));
    for (int i = 0; i < 3; ++i) f.slow();
    f.travel();                                     // older break
    f.slow(); f.slow();
    f.travel();                                     // latest break
    const unsigned int latest = f.g1;
    for (int i = 0; i < 3; ++i) f.slow();           // acc: 3+1+2+1+3 = 10 -> crossing

    REQUIRE(f.fired.size() == 1);
    REQUIRE(f.fired[0].at.g1_line_id == latest);
    REQUIRE_THAT(f.fired[0].early, WithinAbs(3.0f, 1e-5f));
}

TEST_CASE("Thermal dwell schedule: a layer change beats a later travel", "[ThermalDwell]")
{
    Feeder f(make_sched(Schedule::PreferAnyTravel, /*lookahead=*/10.0f));
    for (int i = 0; i < 3; ++i) f.slow();
    f.layer = 2;
    f.travel();                                     // first travel of the new layer = layer_change
    const unsigned int lc = f.g1;
    f.slow(); f.slow();
    f.travel();                                     // plain travel, later
    for (int i = 0; i < 3; ++i) f.slow();           // crossing

    REQUIRE(f.fired.size() == 1);
    REQUIRE(f.fired[0].kind == AnchorKind::LayerChange);
    REQUIRE(f.fired[0].at.g1_line_id == lc);
    REQUIRE(f.fired[0].at.layer == 2);
}

TEST_CASE("Thermal dwell schedule: only the FIRST travel of a new layer is a layer change", "[ThermalDwell]")
{
    Feeder f(make_sched(Schedule::PreferLayerChange, /*lookahead=*/10.0f));
    for (int i = 0; i < 3; ++i) f.slow();
    f.layer = 2;
    f.travel();                                     // layer_change
    const unsigned int lc = f.g1;
    f.slow();                                       // the layer has started extruding
    f.travel();                                     // just a travel now
    for (int i = 0; i < 4; ++i) f.slow();           // acc 3+1+1+1+4 = 10 -> crossing

    REQUIRE(f.fired.size() == 1);
    REQUIRE(f.fired[0].kind == AnchorKind::LayerChange);
    REQUIRE(f.fired[0].at.g1_line_id == lc);
}

TEST_CASE("Thermal dwell schedule: PreferLayerChange ignores plain travels", "[ThermalDwell]")
{
    Feeder f(make_sched(Schedule::PreferLayerChange, /*lookahead=*/10.0f));
    for (int i = 0; i < 5; ++i) f.slow();
    f.travel();
    for (int i = 0; i < 4; ++i) f.slow();

    REQUIRE(f.fired.size() == 1);
    REQUIRE(f.fired[0].kind == AnchorKind::Forced);
}

TEST_CASE("Thermal dwell schedule: excluded travels and retract-only moves are never breaks", "[ThermalDwell]")
{
    Feeder f(make_sched(Schedule::PreferAnyTravel, /*lookahead=*/10.0f));
    for (int i = 0; i < 5; ++i) f.slow();
    f.travel(1.0f, /*excluded=*/true);              // inside a custom-gcode block
    f.retract();                                    // non-travel non-extruding move
    for (int i = 0; i < 4; ++i) f.slow();

    REQUIRE(f.fired.size() == 1);
    REQUIRE(f.fired[0].kind == AnchorKind::Forced);
}

TEST_CASE("Thermal dwell schedule: a break before the episode started is not a candidate", "[ThermalDwell]")
{
    Feeder f(make_sched(Schedule::PreferAnyTravel, /*lookahead=*/100.0f));
    f.slow(); f.slow();
    f.fast();                                       // reset: episode over
    f.travel();                                     // acc == 0 here: cannot be pulled into
    for (int i = 0; i < 10; ++i) f.slow();          // new episode crosses at 10

    REQUIRE(f.fired.size() == 1);
    REQUIRE(f.fired[0].kind == AnchorKind::Forced);
}

TEST_CASE("Thermal dwell schedule: ids stay consecutive across scheduled and forced flushes", "[ThermalDwell]")
{
    Feeder f(make_sched(Schedule::PreferAnyTravel, /*lookahead=*/5.0f));
    for (int i = 0; i < 5; ++i) f.slow();
    f.travel();
    for (int i = 0; i < 4; ++i) f.slow();           // #1 scheduled on the travel, replay acc = 4
    for (int i = 0; i < 6; ++i) f.slow();           // #2 forced at acc 10 (no break in window)

    REQUIRE(f.fired.size() == 2);
    REQUIRE(f.fired[0].id == 1);
    REQUIRE(f.fired[1].id == 2);
    REQUIRE(f.fired[0].kind == AnchorKind::Travel);
    REQUIRE(f.fired[1].kind == AnchorKind::Forced);
    REQUIRE(f.fired[0].at.g1_line_id < f.fired[1].at.g1_line_id);
    REQUIRE(f.sched.flush_count() == 2);
}

TEST_CASE("Thermal dwell schedule: a replay re-crossing yields a second, forced flush", "[ThermalDwell]")
{
    // lookahead >= t_max is the only way the tail after the break can itself exceed t_max.
    Feeder f(make_sched(Schedule::PreferAnyTravel, /*lookahead=*/100.0f));
    f.slow();
    f.travel();                                     // break at t 2, acc 2
    for (int i = 0; i < 8; ++i) f.slow();           // crossing at acc 10 (t 10): scheduled on the travel
    // replay of the 8 slow seconds after the break: acc 8, no re-crossing
    REQUIRE(f.fired.size() == 1);
    REQUIRE_THAT(f.sched.accumulator().acc(), WithinAbs(8.0f, 1e-5f));

    f.slow(); f.slow();                             // 10 -> second crossing, forced (no break since)
    REQUIRE(f.fired.size() == 2);
    REQUIRE(f.fired[1].kind == AnchorKind::Forced);
}

TEST_CASE("Thermal dwell guard: scheduled anchors carry sched/early in the markers", "[ThermalDwell]")
{
    ThermalDwellGuard guard(make_config(), make_sched(Schedule::PreferAnyTravel, 5.0f), make_flush_params());
    unsigned int g1 = 0;
    auto feed = [&](bool extrude, bool travel, float dur, float vol) {
        // X advances per move: a break must actually displace in XY (a pure-Z hop is not one).
        guard.on_move(extrude, travel, dur, vol, false, ++g1, 3, 10.0f + float(g1), 20.0f, 1.0f, 0.0f, true, false, 1200.0f, 0);
    };
    for (int i = 0; i < 5; ++i) feed(true, false, 1.0f, 1.0f);
    feed(false, true, 1.0f, 0.0f);
    for (int i = 0; i < 4; ++i) feed(true, false, 1.0f, 1.0f);

    REQUIRE(guard.anchors().size() == 1);
    const auto &a = guard.anchors()[0];
    REQUIRE(a.kind == AnchorKind::Travel);
    REQUIRE(a.g1_line_id == 6);
    const auto lines = guard.format_flush_lines(a);
    REQUIRE_THAT(lines.front(), ContainsSubstring("sched=travel early=4.00s"));
    REQUIRE_THAT(guard.format_summary(), ContainsSubstring("scheduled=1"));
}
