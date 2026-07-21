#include <catch2/catch_all.hpp>

#include "libslic3r/GCode/ThermalDwellAccumulator.hpp"

using namespace Slic3r::ThermalDwell;
using Catch::Matchers::WithinAbs;

// Unit tests for the thermal dwell guard's accumulator (docs/thermal_dwell_guard/STAGE1.md §4).
// The accumulator is a pure state machine, so every case below is a synthetic move sequence --
// no GCodeProcessor, no config, no slicing.

namespace {

// Small, explicit defaults so each test only states what it actually cares about.
Config make_config()
{
    Config cfg;
    cfg.scope = Scope::WaveOverhangOnly;
    cfg.reset = Reset::OnFastSegment;
    cfg.q_slow = 2.0f;
    cfg.q_fast = 5.0f;
    cfg.t_max = 10.0f;
    cfg.v_melt = 15.0f;
    return cfg;
}

// q = volume/duration = 1.0 mm^3/s, comfortably at or below q_slow.
Move slow_move(bool in_wave = true, float duration = 1.0f)
{
    Move m;
    m.is_extrusion = true;
    m.duration = duration;
    m.volume = duration; // q = 1.0
    m.in_wave_region = in_wave;
    return m;
}

Move fast_move(float volume, bool in_wave = true)
{
    Move m;
    m.is_extrusion = true;
    m.duration = 0.5f;
    m.volume = volume; // q = 2*volume, >= q_fast for volume >= 2.5
    m.in_wave_region = in_wave;
    return m;
}

Move travel_move(float duration, bool in_wave = true)
{
    Move m;
    m.is_extrusion = false;
    m.duration = duration;
    m.volume = 0.0f;
    m.in_wave_region = in_wave;
    return m;
}

} // namespace

TEST_CASE("Thermal dwell: slow extrusion crosses the limit at t_max", "[ThermalDwell]")
{
    Accumulator acc(make_config());

    // Nine seconds of slow flow is under the limit.
    for (int i = 0; i < 9; ++i)
        REQUIRE_FALSE(acc.step(slow_move()).has_value());
    REQUIRE_THAT(acc.acc(), WithinAbs(9.0f, 1e-5f));

    // The tenth crosses it.
    const auto ev = acc.step(slow_move());
    REQUIRE(ev.has_value());
    REQUIRE(ev->id == 1);
    REQUIRE_THAT(ev->acc, WithinAbs(10.0f, 1e-5f));
}

TEST_CASE("Thermal dwell: a fast segment resets the accumulator (OnFastSegment)", "[ThermalDwell]")
{
    Accumulator acc(make_config());

    for (int i = 0; i < 5; ++i)
        acc.step(slow_move());
    REQUIRE_THAT(acc.acc(), WithinAbs(5.0f, 1e-5f));

    // q = 12 mm^3/s, well above q_fast.
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

    // Partial exchanges accumulate volume but must not touch acc.
    acc.step(fast_move(6.0f));
    REQUIRE_THAT(acc.fast_vol(), WithinAbs(6.0f, 1e-5f));
    REQUIRE_THAT(acc.acc(), WithinAbs(5.0f, 1e-5f));

    acc.step(fast_move(6.0f));
    REQUIRE_THAT(acc.fast_vol(), WithinAbs(12.0f, 1e-5f));
    REQUIRE_THAT(acc.acc(), WithinAbs(5.0f, 1e-5f));

    // 12 + 6 >= 15 -> the melt zone is considered exchanged.
    acc.step(fast_move(6.0f));
    REQUIRE_THAT(acc.acc(), WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(acc.fast_vol(), WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Thermal dwell: a travel extends an existing episode (row 7)", "[ThermalDwell]")
{
    Accumulator acc(make_config());

    for (int i = 0; i < 5; ++i)
        acc.step(slow_move());
    REQUIRE_THAT(acc.acc(), WithinAbs(5.0f, 1e-5f));

    // Filament is static in the hot zone during the travel, so residence keeps accruing.
    acc.step(travel_move(3.0f));
    REQUIRE_THAT(acc.acc(), WithinAbs(8.0f, 1e-5f));
}

TEST_CASE("Thermal dwell: a travel never starts an episode (row 8)", "[ThermalDwell]")
{
    Accumulator acc(make_config());

    // Long enough to cross t_max several times over, if it counted at all.
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
    REQUIRE_THAT(acc.acc(), WithinAbs(5.0f, 1e-5f));

    acc.step(travel_move(1.0f));
    REQUIRE_THAT(acc.acc(), WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Thermal dwell: scope gates slow-flow accumulation", "[ThermalDwell]")
{
    SECTION("WaveOverhangOnly ignores slow flow outside a wave region (row 2)") {
        Accumulator acc(make_config());
        acc.step(slow_move(/*in_wave=*/false));
        REQUIRE_THAT(acc.acc(), WithinAbs(0.0f, 1e-5f));
    }

    SECTION("WholePrint accumulates the same move (row 1)") {
        Config cfg = make_config();
        cfg.scope = Scope::WholePrint;
        Accumulator acc(cfg);
        acc.step(slow_move(/*in_wave=*/false));
        REQUIRE_THAT(acc.acc(), WithinAbs(1.0f, 1e-5f));
    }
}

TEST_CASE("Thermal dwell: hysteresis-band flow changes nothing (row 3)", "[ThermalDwell]")
{
    Config cfg = make_config();
    cfg.reset = Reset::OnVolume; // so fast_vol can hold a value to check against
    Accumulator acc(cfg);

    for (int i = 0; i < 5; ++i)
        acc.step(slow_move());
    acc.step(fast_move(3.0f)); // q = 6 >= q_fast; partial exchange leaves fast_vol = 3
    REQUIRE_THAT(acc.acc(), WithinAbs(5.0f, 1e-5f));
    REQUIRE_THAT(acc.fast_vol(), WithinAbs(3.0f, 1e-5f));

    // q = 3.5 mm^3/s: strictly between q_slow and q_fast.
    Move band;
    band.is_extrusion = true;
    band.duration = 1.0f;
    band.volume = 3.5f;
    band.in_wave_region = true;
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

    // The reported acc is the value at the crossing, but the state is cleared behind it, so
    // detection continues realistically rather than re-firing on every subsequent move.
    REQUIRE_THAT(acc.acc(), WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(acc.fast_vol(), WithinAbs(0.0f, 1e-5f));

    for (int i = 0; i < 9; ++i)
        REQUIRE_FALSE(acc.step(slow_move()).has_value());
    const auto second = acc.step(slow_move());
    REQUIRE(second.has_value());
    REQUIRE(second->id == 2);
    REQUIRE(acc.flush_count() == 2);
}

TEST_CASE("Thermal dwell: slow flow clears fast_vol regardless of scope", "[ThermalDwell]")
{
    // The subtlest line in the §4 table: rows 1 AND 2 both zero fast_vol, because a slow move
    // interrupts a partial fast exchange wherever it happens -- including out of scope, where
    // it is otherwise a no-op.
    Config cfg = make_config();
    cfg.reset = Reset::OnVolume;
    Accumulator acc(cfg);

    acc.step(fast_move(6.0f));
    REQUIRE_THAT(acc.fast_vol(), WithinAbs(6.0f, 1e-5f));

    acc.step(slow_move(/*in_wave=*/false)); // row 2: out of scope
    REQUIRE_THAT(acc.acc(), WithinAbs(0.0f, 1e-5f));      // acc untouched
    REQUIRE_THAT(acc.fast_vol(), WithinAbs(0.0f, 1e-5f)); // but the exchange is broken
}

TEST_CASE("Thermal dwell: total slow-flow seconds counts only slow extrusion", "[ThermalDwell]")
{
    Config cfg = make_config();
    cfg.t_max = 100.0f; // keep the episode open; this case is about the counter, not crossings
    Accumulator acc(cfg);

    acc.step(slow_move(true, 2.0f));
    acc.step(travel_move(5.0f)); // extends the episode, but is not slow-flow time
    acc.step(slow_move(true, 3.0f));

    REQUIRE_THAT(acc.total_slow_flow_seconds(), WithinAbs(5.0f, 1e-5f));
    REQUIRE_THAT(acc.acc(), WithinAbs(10.0f, 1e-5f));
}
