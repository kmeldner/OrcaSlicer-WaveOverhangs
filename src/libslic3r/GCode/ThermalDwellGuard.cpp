#include "ThermalDwellGuard.hpp"

#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"

#include <cstdio>

namespace Slic3r {

std::optional<ThermalDwell::Config> ThermalDwellGuard::resolve_config(const Print &print)
{
    // The accumulator models the hotend, which is global, but Stage 0 placed the
    // thermal_dwell_* keys in PrintRegionConfig (beside the wave-overhang options). So take
    // the settings from the first region that enables the guard.
    //
    // TODO(Stage 2+): either run a per-region accumulator, or hoist these keys out of
    // PrintRegionConfig — the melt zone does not care which region the nozzle is printing.
    for (size_t i = 0; i < print.num_print_regions(); ++i) {
        const PrintRegionConfig &rc = print.get_print_region(i).config();
        if (! rc.thermal_dwell_guard_enable.value)
            continue;

        ThermalDwell::Config cfg;
        cfg.scope = (rc.thermal_dwell_scope.value == ThermalDwellScope::WholePrint)
                        ? ThermalDwell::Scope::WholePrint
                        : ThermalDwell::Scope::WaveOverhangOnly;
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

std::string ThermalDwellGuard::format_config_comment(const ThermalDwell::Config &cfg)
{
    const char *scope = (cfg.scope == ThermalDwell::Scope::WholePrint) ? "WholePrint"
                                                                      : "WaveOverhangOnly";
    const char *reset = "OnFastSegment";
    switch (cfg.reset) {
    case ThermalDwell::Reset::OnTravel: reset = "OnTravel"; break;
    case ThermalDwell::Reset::OnVolume: reset = "OnVolume"; break;
    default: break;
    }

    char buf[256];
    snprintf(buf, sizeof(buf),
             ";THERMAL_DWELL_CONFIG scope=%s flow_max=%g fast_min=%g max_time=%g reset=%s melt_zone=%g\n",
             scope, cfg.q_slow, cfg.q_fast, cfg.t_max, reset, cfg.v_melt);
    return std::string(buf);
}

void ThermalDwellGuard::on_move(bool is_extrusion, float duration, float volume,
                                bool in_wave_region, unsigned int g1_line_id,
                                unsigned int layer, float z)
{
    ThermalDwell::Move move;
    move.is_extrusion   = is_extrusion;
    move.duration       = duration;
    move.volume         = volume;
    move.in_wave_region = in_wave_region;

    if (const std::optional<ThermalDwell::FlushEvent> ev = m_acc.step(move))
        m_anchors.push_back({ g1_line_id, ev->id, layer, z, ev->acc });
}

} // namespace Slic3r
