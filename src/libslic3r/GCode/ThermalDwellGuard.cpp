#include "ThermalDwellGuard.hpp"

#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"

#include <cstdio>
#include <cmath>

namespace Slic3r {

std::optional<ThermalDwell::Config> ThermalDwellGuard::resolve_config(const Print &print)
{
    // The accumulator models the hotend, which is global, but the thermal_dwell_* keys live in
    // PrintRegionConfig (beside the wave-overhang options). So take the settings from the first
    // region that enables the guard.
    //
    // TODO(Stage 2+): either run a per-region accumulator, or hoist these keys out of
    // PrintRegionConfig — the melt zone does not care which region the nozzle is printing.
    for (size_t i = 0; i < print.num_print_regions(); ++i) {
        const PrintRegionConfig &rc = print.get_print_region(i).config();
        if (! rc.thermal_dwell_guard_enable.value)
            continue;

        ThermalDwell::Config cfg;
        switch (rc.thermal_dwell_reset_mode.value) {
        case ThermalDwellReset::OnTravel: cfg.reset = ThermalDwell::Reset::OnTravel; break;
        case ThermalDwellReset::OnVolume: cfg.reset = ThermalDwell::Reset::OnVolume; break;
        default:                          cfg.reset = ThermalDwell::Reset::OnFastSegment; break;
        }
        cfg.q_slow = float(rc.thermal_dwell_flow_threshold.value);
        cfg.q_fast = float(rc.thermal_dwell_fast_flow_threshold.value);
        cfg.t_max  = float(rc.thermal_dwell_max_time.value);
        cfg.v_melt = float(rc.thermal_dwell_melt_zone_volume.value);
        return cfg;
    }
    return std::nullopt;
}

ThermalDwellFlushParams ThermalDwellGuard::resolve_flush_params(const Print &print)
{
    ThermalDwellFlushParams fp;

    // Group-3 flush keys come from the first guard-enabled region (same region resolve_config
    // reads). If none is enabled the defaults stand; callers gate on resolve_config() anyway.
    for (size_t i = 0; i < print.num_print_regions(); ++i) {
        const PrintRegionConfig &rc = print.get_print_region(i).config();
        if (! rc.thermal_dwell_guard_enable.value)
            continue;
        fp.flush_pos     = rc.thermal_dwell_flush_position.value;
        fp.z_lift        = float(rc.thermal_dwell_flush_z_lift.value);
        fp.purge_volume  = float(rc.thermal_dwell_purge_volume.value);
        fp.purge_flow    = float(rc.thermal_dwell_purge_flow.value);
        fp.wipe          = rc.thermal_dwell_wipe.value;
        break;
    }

    // Reused machine/filament settings. v1 is single-extruder/single-material, so the
    // per-extruder retraction and filament values are taken for tool 0.
    // TODO(Stage 2+): index by the anchor's active tool once multi-material is in scope.
    const PrintConfig &pc = print.config();
    auto at = [](const ConfigOptionFloats &opt, double dflt) -> float {
        return opt.empty() ? float(dflt) : float(opt.get_at(0));
    };
    fp.retract_length  = at(pc.retraction_length,  0.8);
    fp.retract_speed   = at(pc.retraction_speed,   30.0);
    fp.deretract_speed = at(pc.deretraction_speed, 30.0);
    if (fp.deretract_speed <= 0.0f) // 0 means "same as retraction speed" in this config
        fp.deretract_speed = fp.retract_speed;
    fp.travel_speed    = float(pc.travel_speed.value);
    const float dia    = at(pc.filament_diameter, 1.75);
    fp.filament_area   = float(M_PI) * (dia * 0.5f) * (dia * 0.5f);
    return fp;
}

std::string ThermalDwellGuard::format_config_comment(const ThermalDwell::Config &cfg,
                                                     const ThermalDwellFlushParams &flush)
{
    const char *reset = "OnFastSegment";
    switch (cfg.reset) {
    case ThermalDwell::Reset::OnTravel: reset = "OnTravel"; break;
    case ThermalDwell::Reset::OnVolume: reset = "OnVolume"; break;
    default: break;
    }

    char buf[512];
    snprintf(buf, sizeof(buf),
             ";THERMAL_DWELL_CONFIG flow_max=%g fast_min=%g max_time=%g reset=%s melt_zone=%g "
             "flush_pos=%g,%g z_lift=%g purge_vol=%g purge_flow=%g wipe=%d\n",
             cfg.q_slow, cfg.q_fast, cfg.t_max, reset, cfg.v_melt,
             flush.flush_pos.x(), flush.flush_pos.y(), flush.z_lift,
             flush.purge_volume, flush.purge_flow, int(flush.wipe));
    return std::string(buf);
}

void ThermalDwellGuard::on_move(bool is_extrusion, float duration, float volume, bool excluded,
                                unsigned int g1_line_id, unsigned int layer,
                                float x, float y, float z, float e, bool e_relative,
                                bool retracted, float feedrate_mmmin, unsigned int tool)
{
    ThermalDwell::Move move;
    move.is_extrusion = is_extrusion;
    move.duration     = duration;
    move.volume       = volume;
    move.excluded     = excluded;

    if (const std::optional<ThermalDwell::FlushEvent> ev = m_acc.step(move)) {
        FlushAnchor a;
        a.g1_line_id = g1_line_id;
        a.id         = ev->id;
        a.layer      = layer;
        a.z          = z;
        a.acc        = ev->acc;
        a.x          = x;
        a.y          = y;
        a.e          = e;
        a.e_relative = e_relative;
        a.retracted  = retracted;
        a.feedrate   = feedrate_mmmin;
        a.tool       = tool;
        m_anchors.push_back(a);
    }
}

std::vector<std::string> ThermalDwellGuard::format_flush_lines(const FlushAnchor &a) const
{
    // STAGE2 §5 — the exact flush order. Three invariants hold by construction:
    //  - pure insertion: only lines are added, never a byte of an original line changed;
    //  - state neutrality: after the block, XY/Z/logical-E/retract/modal-F match the anchor;
    //  - no machine-state side effects: motion + extrusion only, at the current temperature.
    // Every inserted move carries an explicit F, so none relies on ambient modal state, and
    // none changes state that outlives the block.
    const ThermalDwellFlushParams &p = m_flush;
    const float eps    = 1e-6f;
    const float A      = (p.filament_area > eps) ? p.filament_area : 1.0f;
    const float E_p    = p.purge_volume / A;             // filament mm to purge
    const float F_p    = p.purge_flow / A * 60.0f;       // mm/min
    const float r      = p.retract_length;               // mm
    const float F_ret  = p.retract_speed   * 60.0f;      // mm/min
    const float F_der  = p.deretract_speed * 60.0f;      // mm/min
    const float F_tr   = p.travel_speed    * 60.0f;      // mm/min
    const bool  do_r   = r > eps;

    std::vector<std::string> lines;
    char buf[96];

    // Running absolute E, only meaningful when !e_relative; the mandatory G92 at the end pins
    // it back to the anchor regardless of any residual.
    double e_run = a.e;
    auto emit_E = [&](double delta, float feed) {
        e_run += delta;
        snprintf(buf, sizeof(buf), "G1 E%.5f F%.0f\n", a.e_relative ? delta : e_run, feed);
        lines.emplace_back(buf);
    };
    auto emit_X = [&](float x, float feed) {
        snprintf(buf, sizeof(buf), "G1 X%.3f F%.0f\n", x, feed);
        lines.emplace_back(buf);
    };
    auto emit_Y = [&](float y, float feed) {
        snprintf(buf, sizeof(buf), "G1 Y%.3f F%.0f\n", y, feed);
        lines.emplace_back(buf);
    };

    // Kept from Stage 1 (§7): the flush point, then the block markers around the sequence.
    snprintf(buf, sizeof(buf), ";THERMAL_DWELL_FLUSH_POINT id=%u acc=%.2fs layer=%u z=%.3f\n",
             a.id, a.acc, a.layer, a.z);
    lines.emplace_back(buf);
    snprintf(buf, sizeof(buf), ";THERMAL_DWELL_FLUSH_START id=%u\n", a.id);
    lines.emplace_back(buf);

    // 1. Retract, only if not already retracted at the anchor.
    if (do_r && ! a.retracted) emit_E(-r, F_ret);
    // 2. Z-hop to clear the part.
    snprintf(buf, sizeof(buf), "G1 Z%.3f F%.0f\n", a.z + p.z_lift, F_tr);
    lines.emplace_back(buf);
    // 3. Travel to the flush position. X1C approach order: X first, then Y (into the chute,
    //    beyond the bed's Y edge).
    emit_X(float(p.flush_pos.x()), F_tr);
    emit_Y(float(p.flush_pos.y()), F_tr);
    // 4. Unretract to prime for the purge (we are retracted here whenever do_r).
    if (do_r) emit_E(r, F_der);
    // 5. Purge purge_volume at purge_flow, ejected into the chute.
    emit_E(E_p, F_p);
    // 6. Wipe on the machine wiper (built-in Bambu X1C pattern, motion only): sweep in X across
    //    the wiper at the flush Y, anchored on the flush X so it tracks a custom position.
    if (p.wipe) {
        const float x0 = float(p.flush_pos.x());
        const float y0 = float(p.flush_pos.y());
        const float x1 = x0 + 30.0f; // X70 <-> X100 in the extracted change_filament routine
        for (int i = 0; i < 3; ++i) {
            snprintf(buf, sizeof(buf), "G1 X%.3f Y%.3f F5000\n", x1, y0);
            lines.emplace_back(buf);
            snprintf(buf, sizeof(buf), "G1 X%.3f Y%.3f F10000\n", x0, y0);
            lines.emplace_back(buf);
        }
    }
    // 7. Retract before travelling back.
    if (do_r) emit_E(-r, F_ret);
    // 8. Travel back to the anchor XY (still lifted), X then Y.
    emit_X(a.x, F_tr);
    emit_Y(a.y, F_tr);
    // 9. Z restore.
    snprintf(buf, sizeof(buf), "G1 Z%.3f F%.0f\n", a.z, F_tr);
    lines.emplace_back(buf);
    // 10. Unretract only to match the anchor's retract state (skip if the anchor was retracted).
    if (do_r && ! a.retracted) emit_E(r, F_der);
    // 11. Absolute-E fixup — MANDATORY: without it every downstream absolute E target is wrong.
    //     A no-op for relative E (the fork's X1C default emits M83).
    if (! a.e_relative) {
        snprintf(buf, sizeof(buf), "G92 E%.5f\n", a.e);
        lines.emplace_back(buf);
    }
    // 12. Restore the modal feedrate the anchor left in force.
    snprintf(buf, sizeof(buf), "G1 F%.0f\n", a.feedrate);
    lines.emplace_back(buf);

    snprintf(buf, sizeof(buf), ";THERMAL_DWELL_FLUSH_END id=%u\n", a.id);
    lines.emplace_back(buf);
    return lines;
}

std::string ThermalDwellGuard::format_summary() const
{
    const ThermalDwellFlushParams &p = m_flush;
    const size_t count = m_anchors.size();
    const float total_purge = float(count) * p.purge_volume;

    // Rough per-flush wall-clock estimate (info only; M73 excludes flush time — §8). Sum of
    // round-trip travel, purge, and a nominal wipe allowance.
    const float purge_time = (p.purge_flow > 1e-6f) ? p.purge_volume / p.purge_flow : 0.0f;
    const float wipe_time  = p.wipe ? (3.0f * 2.0f * 30.0f) / std::max(1.0f, p.travel_speed) : 0.0f;
    float est_added = 0.0f;
    for (const FlushAnchor &a : m_anchors) {
        const float dx = float(p.flush_pos.x()) - a.x;
        const float dy = float(p.flush_pos.y()) - a.y;
        const float d  = std::sqrt(dx * dx + dy * dy);
        const float travel_time = (p.travel_speed > 1e-6f) ? 2.0f * d / p.travel_speed : 0.0f;
        est_added += travel_time + purge_time + wipe_time;
    }

    std::string out;
    char buf[192];
    snprintf(buf, sizeof(buf),
             ";THERMAL_DWELL_SUMMARY count=%zu total_purge_mm3=%.2f est_added_time_s=%.1f\n",
             count, total_purge, est_added);
    out += buf;
    for (const FlushAnchor &a : m_anchors) {
        snprintf(buf, sizeof(buf), ";THERMAL_DWELL_POINT id=%u layer=%u z=%.3f acc=%.2f\n",
                 a.id, a.layer, a.z, a.acc);
        out += buf;
    }
    return out;
}

} // namespace Slic3r
