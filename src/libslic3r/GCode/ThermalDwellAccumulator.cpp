#include "ThermalDwellAccumulator.hpp"

#include <limits>

namespace Slic3r {
namespace ThermalDwell {

void Accumulator::reset()
{
    m_acc = 0.0f;
    m_fast_vol = 0.0f;
    m_flush_count = 0;
    m_total_slow_flow_s = 0.0f;
}

// STAGE1 §4's table, one branch per row. Row numbers below refer to that table.
std::optional<FlushEvent> Accumulator::step(const Move &move)
{
    if (move.is_extrusion) {
        // A zero-duration extrusion cannot soak the melt zone, so treat it as infinitely
        // fast rather than dividing by zero and mis-filing it as row 1.
        const float q = (move.duration > 0.0f) ? move.volume / move.duration
                                               : std::numeric_limits<float>::infinity();

        if (q <= m_cfg.q_slow) {
            // Rows 1 and 2. fast_vol is cleared regardless of scope: a slow move interrupts
            // a partial fast exchange wherever it happens.
            m_fast_vol = 0.0f;
            if (! in_scope(move))
                return std::nullopt; // Row 2: out of scope, acc unchanged.
            m_acc += move.duration;  // Row 1.
            m_total_slow_flow_s += move.duration;
            return check_threshold();
        }

        if (q < m_cfg.q_fast) {
            // Row 3, the hysteresis band. Mid-band flow does exchange some filament, but v1
            // keeps a binary model; the band stops acc flapping between "accumulating" and
            // "curing". Both acc and fast_vol are left strictly untouched.
            // TODO(Stage 4+): credit mid-band flow toward melt-zone exchange (OnVolume).
            return std::nullopt;
        }

        // q >= q_fast from here.
        if (m_cfg.reset == Reset::OnVolume) {
            // Row 5: reset only once enough fast filament has passed to exchange the melt
            // zone. Volume beyond v_melt is discarded rather than carried, per the table.
            // TODO(Stage 4+): decay acc proportionally to exchanged volume instead of
            // resetting it in one step.
            if (m_fast_vol + move.volume >= m_cfg.v_melt) {
                m_acc = 0.0f;
                m_fast_vol = 0.0f;
            } else {
                m_fast_vol += move.volume;
            }
            return std::nullopt; // acc only ever falls here, so no threshold check.
        }

        // Row 4: OnFastSegment / OnTravel — any single fast segment resets.
        m_acc = 0.0f;
        m_fast_vol = 0.0f;
        return std::nullopt;
    }

    // Non-extruding: travel, retract, unretract, pure-Z move, G4 dwell.
    // Rows 6-8 are deliberately NOT scope-gated. The filament soaks in the hot zone wherever
    // the nozzle happens to be, so a travel that leaves a wave region still extends an
    // episode that started inside it.
    if (m_cfg.reset == Reset::OnTravel) {
        m_acc = 0.0f; // Row 6.
        m_fast_vol = 0.0f;
        return std::nullopt;
    }

    if (m_acc > 0.0f) {
        // Row 7: during non-extruding time the filament is fully static in the hot zone, so
        // residence keeps accruing.
        m_acc += move.duration;
        return check_threshold();
    }

    // Row 8: travels never START an episode, they only extend one.
    return std::nullopt;
}

std::optional<FlushEvent> Accumulator::check_threshold()
{
    // Called only from rows 1 and 7 — the two rows that raise acc. Rows 4/5/6 move it only
    // downward (to zero), where a check is a provable no-op for any t_max > 0.
    //
    // The t_max <= 0 guard closes a case the table leaves open: without it the post-reset
    // acc == 0 state would satisfy acc >= t_max and fire on every subsequent move forever.
    if (m_cfg.t_max <= 0.0f || m_acc < m_cfg.t_max)
        return std::nullopt;

    const FlushEvent ev{ ++m_flush_count, m_acc };
    // Simulate the flush the guard WOULD have performed here, so that detection downstream
    // stays realistic instead of re-firing on every following move.
    m_acc = 0.0f;
    m_fast_vol = 0.0f;
    return ev;
}

} // namespace ThermalDwell
} // namespace Slic3r
