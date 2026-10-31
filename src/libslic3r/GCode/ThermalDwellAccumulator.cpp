#include "ThermalDwellAccumulator.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>

namespace Slic3r {
namespace ThermalDwell {

// ------------------------------------------------------------------------------------------
// Calibration table (STAGE4 §2/§3)

CalibrationTable parse_calibration_table(const std::string &text)
{
    // "q:t;q:t;..." — tolerant of whitespace and junk. std::map sorts by q and makes the last
    // duplicate win, both of which §3 asks for.
    std::map<float, float> entries;
    size_t pos = 0;
    while (pos <= text.size()) {
        size_t end = text.find_first_of(";,\n", pos);
        if (end == std::string::npos)
            end = text.size();
        std::string item = text.substr(pos, end - pos);
        pos = end + 1;
        const size_t colon = item.find(':');
        if (colon == std::string::npos)
            continue;
        char *q_end = nullptr;
        char *t_end = nullptr;
        const std::string qs = item.substr(0, colon);
        const std::string ts = item.substr(colon + 1);
        const float q = std::strtof(qs.c_str(), &q_end);
        const float t = std::strtof(ts.c_str(), &t_end);
        if (q_end == qs.c_str() || t_end == ts.c_str())
            continue; // not a number
        if (! (q > 0.0f) || ! (t > 0.0f))
            continue; // non-positive or NaN
        entries[q] = t;
    }
    CalibrationTable table;
    table.reserve(entries.size());
    for (const auto &kv : entries)
        table.emplace_back(kv.first, kv.second);
    return table;
}

std::string format_calibration_table(const CalibrationTable &table)
{
    if (table.empty())
        return "none";
    std::string out;
    char buf[64];
    for (size_t i = 0; i < table.size(); ++i) {
        snprintf(buf, sizeof(buf), "%s%g:%g", i ? ";" : "", table[i].first, table[i].second);
        out += buf;
    }
    return out;
}

float safe_seconds(const CalibrationTable &table, float q)
{
    // Linear interpolation in q, clamped to the end entries.
    if (q <= table.front().first)
        return table.front().second;
    if (q >= table.back().first)
        return table.back().second;
    const auto hi = std::upper_bound(table.begin(), table.end(), q,
        [](float value, const std::pair<float, float> &e) { return value < e.first; });
    const auto lo = hi - 1;
    const float span = hi->first - lo->first;
    const float w    = span > 0.0f ? (q - lo->first) / span : 0.0f;
    return lo->second + w * (hi->second - lo->second);
}

// ------------------------------------------------------------------------------------------
// Accumulator

float Accumulator::safe_time(float q) const
{
    return m_cfg.table.empty() ? m_cfg.t_max : safe_seconds(m_cfg.table, q);
}

float Accumulator::static_safe_time() const
{
    return m_cfg.table.empty() ? m_cfg.t_max : m_cfg.table.front().second;
}

void Accumulator::reset()
{
    zero_episode();
    m_flush_pending = false;
    m_flush_count = 0;
    m_total_slow_flow_s = 0.0f;
}

void Accumulator::zero_episode()
{
    m_acc      = 0.0f;
    m_frac     = 0.0f;
    m_fast_vol = 0.0f;
}

void Accumulator::set_episode(float acc, float frac, float fast_vol)
{
    m_acc           = acc;
    m_frac          = frac;
    m_fast_vol      = fast_vol;
    m_flush_pending = false;
}

bool Accumulator::limit_reached() const
{
    // STAGE4 §2: with a table the dwell fraction is the limit; without one the exact Stage-1
    // comparison is kept so the constant-model output stays byte-identical.
    if (m_cfg.table.empty())
        return m_cfg.t_max > 0.0f && m_acc >= m_cfg.t_max;
    return m_frac >= 1.0f;
}

// STAGE1 §4's table, one branch per row. Row numbers below refer to that table. As of Stage 2
// the guard is standalone, so rows 1 and 2 (in/out of scope) collapse: slow extrusion always
// accumulates.
std::optional<FlushEvent> Accumulator::step(const Move &move)
{
    // STAGE2 §6 deferral: a crossing that landed in an exclusion zone was held rather than
    // anchored. Place it on the first move that is not itself excluded, before this move's own
    // effects. The reset below means this move can no longer re-cross, so at most one event
    // still leaves per step (STAGE1 §1).
    if (m_flush_pending && ! move.excluded) {
        m_flush_pending = false;
        const FlushEvent ev{ ++m_flush_count, m_acc, m_cfg.table.empty() && m_cfg.t_max > 0.0f ? m_acc / m_cfg.t_max : m_frac };
        zero_episode();
        return ev;
    }

    if (move.is_extrusion) {
        // A zero-duration extrusion cannot soak the melt zone, so treat it as infinitely
        // fast rather than dividing by zero and mis-filing it as row 1.
        const float q = (move.duration > 0.0f) ? move.volume / move.duration
                                               : std::numeric_limits<float>::infinity();

        if (q <= m_cfg.q_slow) {
            // Row 1 (always in scope now). fast_vol is cleared: a slow move interrupts a
            // partial fast exchange wherever it happens.
            m_fast_vol = 0.0f;
            m_acc += move.duration;
            const float T = safe_time(q);
            if (T > 0.0f)
                m_frac += move.duration / T;
            m_total_slow_flow_s += move.duration;
            return check_threshold(move.excluded);
        }

        if (q < m_cfg.q_fast) {
            // Row 3, the hysteresis band. Mid-band flow does exchange some filament, but v1
            // keeps a binary model; the band stops acc flapping between "accumulating" and
            // "curing". Both acc and fast_vol are left strictly untouched.
            // TODO(Stage 5+): credit mid-band flow toward melt-zone exchange (OnVolume).
            return std::nullopt;
        }

        // q >= q_fast from here.
        if (m_cfg.reset == Reset::OnVolume) {
            // Row 5: reset only once enough fast filament has passed to exchange the melt
            // zone. Volume beyond v_melt is discarded rather than carried, per the table.
            // TODO(Stage 5+): decay acc proportionally to exchanged volume instead of
            // resetting it in one step.
            if (m_fast_vol + move.volume >= m_cfg.v_melt) {
                zero_episode();
            } else {
                m_fast_vol += move.volume;
            }
            return std::nullopt; // acc only ever falls here, so no threshold check.
        }

        // Row 4: OnFastSegment / OnTravel — any single fast segment resets.
        zero_episode();
        return std::nullopt;
    }

    // Non-extruding: travel, retract, unretract, pure-Z move, G4 dwell.
    // The filament soaks in the hot zone wherever the nozzle happens to be, so a travel still
    // extends an episode that started while extruding.
    if (m_cfg.reset == Reset::OnTravel) {
        zero_episode(); // Row 6.
        return std::nullopt;
    }

    if (m_acc > 0.0f) {
        // Row 7: during non-extruding time the filament is fully static in the hot zone, so
        // residence keeps accruing — against the slowest calibrated flow's safe time (STAGE4 §2).
        m_acc += move.duration;
        const float T = static_safe_time();
        if (T > 0.0f)
            m_frac += move.duration / T;
        return check_threshold(move.excluded);
    }

    // Row 8: travels never START an episode, they only extend one.
    return std::nullopt;
}

std::optional<FlushEvent> Accumulator::check_threshold(bool excluded)
{
    // Called only from rows 1 and 7 — the two rows that raise acc. Rows 4/5/6 move it only
    // downward (to zero), where a check is a provable no-op for any t_max > 0.
    //
    // The t_max <= 0 guard (inside limit_reached) closes a case the table leaves open: without
    // it the post-reset acc == 0 state would satisfy acc >= t_max and fire on every subsequent
    // move forever.
    if (! limit_reached())
        return std::nullopt;

    if (excluded) {
        // STAGE2 §6: the crossing landed inside an exclusion zone. Do NOT anchor here and do
        // NOT reset — arm a pending flush and keep accumulating. step() fires it on the first
        // later non-excluded move, so the flush lands late rather than corrupting the block.
        m_flush_pending = true;
        return std::nullopt;
    }

    const FlushEvent ev{ ++m_flush_count, m_acc, m_cfg.table.empty() && m_cfg.t_max > 0.0f ? m_acc / m_cfg.t_max : m_frac };
    // Simulate the flush the guard WOULD have performed here, so that detection downstream
    // stays realistic instead of re-firing on every following move.
    m_flush_pending = false;
    zero_episode();
    return ev;
}

} // namespace ThermalDwell
} // namespace Slic3r
