#include "ThermalDwellScheduler.hpp"

namespace Slic3r {
namespace ThermalDwell {

Scheduled Scheduler::make(const AnchorState &st, float acc, AnchorKind kind, float early)
{
    Scheduled s;
    s.at    = st;
    s.id    = ++m_count;
    s.acc   = acc;
    s.kind  = kind;
    s.early = early;
    m_have_anchor    = true;
    m_last_anchor_g1 = st.g1_line_id;
    return s;
}

AnchorKind Scheduler::classify_break(const Move &move, bool is_travel, const AnchorState &state)
{
    // Layer tracking (STAGE3 §3): the first eligible travel of a new layer is the layer-change
    // break. The flag survives excluded moves (the custom layer-change block) and is consumed
    // by the first travel candidate, or dropped once the new layer has started extruding —
    // after that a travel is just a travel.
    if (! m_have_layer || state.layer != m_layer) {
        m_layer_change_pending = m_have_layer; // the very first layer is not a "change"
        m_have_layer = true;
        m_layer      = state.layer;
    }
    if (move.is_extrusion && ! move.excluded)
        m_layer_change_pending = false;

    // GCodeProcessor files any non-extruding move with displacement as Travel, including the
    // pure-Z hop that follows a lifted travel. Only a move that actually changes XY is a break
    // (STAGE3 §3); the hop lands on the same XY as the travel before it and adds nothing.
    const bool moved_xy = state.x != m_prev_x || state.y != m_prev_y;
    m_prev_x = state.x;
    m_prev_y = state.y;

    if (! is_travel || ! moved_xy || move.is_extrusion || move.excluded)
        return AnchorKind::Forced;

    if (m_layer_change_pending) {
        m_layer_change_pending = false;
        return AnchorKind::LayerChange;
    }
    return AnchorKind::Travel;
}

int Scheduler::choose_break(float t_now) const
{
    if (m_sched.mode == Schedule::ForcedOnly || m_sched.lookahead <= 0.0f)
        return -1;

    int best_layer  = -1;
    int best_travel = -1;
    // Newest first, so the first hit of each kind is the latest one (STAGE3 §4 "latest").
    for (int i = int(m_buf.size()) - 1; i >= 0; --i) {
        const Rec &rec = m_buf[size_t(i)];
        if (t_now - rec.t_end > m_sched.lookahead)
            break; // older than the window; everything before it is older still
        if (rec.brk == AnchorKind::Forced)
            continue;
        // Anchors must never go backwards past the previous flush (STAGE3 §6).
        if (m_have_anchor && rec.st.g1_line_id < m_last_anchor_g1)
            continue;
        if (rec.brk == AnchorKind::LayerChange) {
            best_layer = i;
            break; // a layer change always wins; nothing older can beat it
        }
        if (m_sched.mode == Schedule::PreferAnyTravel && best_travel < 0)
            best_travel = i;
    }
    return best_layer >= 0 ? best_layer : best_travel;
}

void Scheduler::replay_after(size_t from_index, std::vector<Scheduled> &out)
{
    // STAGE3 §5. The moves strictly after the chosen break (the live accumulator was just zeroed
    // by the crossing, as if the flush had happened on the crossing move) are re-run through a
    // fresh state so the live accumulator looks as if the flush happened at the break instead.
    // flush_count / total_slow_flow_seconds of the live accumulator are left alone: the ids are
    // the scheduler's, and those seconds were counted when the moves were first fed.
    Accumulator sim(m_acc.config());
    std::deque<Rec> kept;
    for (size_t i = from_index + 1; i < m_buf.size(); ++i) {
        Rec rec = m_buf[i];
        if (const std::optional<FlushEvent> ev = sim.step(rec.move)) {
            // A re-crossing inside the window (only possible when lookahead >= t_max). Anchor
            // it forced on the move it fired on; the sim is already zeroed by the event.
            out.push_back(make(rec.st, ev->acc, AnchorKind::Forced, 0.0f));
            kept.clear();
            continue;
        }
        if (sim.acc() <= 0.0f) {
            kept.clear(); // left the episode
            continue;
        }
        rec.acc_after = sim.acc();
        kept.push_back(rec);
    }
    m_buf = std::move(kept);
    m_acc.set_episode(sim.acc(), sim.fast_vol());
}

std::vector<Scheduled> Scheduler::step(const Move &move, bool is_travel, const AnchorState &state)
{
    std::vector<Scheduled> out;

    const AnchorKind brk = classify_break(move, is_travel, state);
    m_t += move.duration;

    const std::optional<FlushEvent> ev = m_acc.step(move);

    if (! ev) {
        if (m_acc.acc() <= 0.0f) {
            // Not in an episode (reset row, or never started): nothing here can be pulled into.
            m_buf.clear();
            return out;
        }
        Rec rec;
        rec.move      = move;
        rec.st        = state;
        rec.brk       = brk;
        rec.t_end     = m_t;
        rec.acc_after = m_acc.acc();
        m_buf.push_back(rec);
        // Trim moves that have fallen out of the window (STAGE3 §6).
        while (! m_buf.empty() && m_t - m_buf.front().t_end > m_sched.lookahead)
            m_buf.pop_front();
        return out;
    }

    // A crossing fired on this move. Try to pull it back onto a break in the window.
    const int index = choose_break(m_t);
    if (index < 0) {
        // Forced (Stage 2 semantics): anchor here, new episode starts empty.
        out.push_back(make(state, ev->acc, AnchorKind::Forced, 0.0f));
        m_buf.clear();
        return out;
    }

    const Rec b = m_buf[size_t(index)];
    out.push_back(make(b.st, b.acc_after, b.brk, m_t - b.t_end));

    // The crossing move itself is part of the replayed tail: append it, then replay after B.
    Rec cur;
    cur.move  = move;
    cur.st    = state;
    cur.brk   = brk;
    cur.t_end = m_t;
    m_buf.push_back(cur);
    replay_after(size_t(index), out);
    return out;
}

} // namespace ThermalDwell
} // namespace Slic3r
