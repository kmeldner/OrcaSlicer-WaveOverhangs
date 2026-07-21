# Thermal Dwell Guard — Feature Specification

**Status:** design / not yet implemented
**Target repo:** fork of `dennisklappe/OrcaSlicer-WaveOverhangs`
**Author:** Krzysztof Meldner
**Placement:** `docs/thermal_dwell_guard/SPEC.md`

This document is the authoritative spec for the *thermal dwell guard*. Implement against
this file; if reality forces a deviation, update this file in the same commit.

---

## 1. Problem

At very low volumetric flow (e.g. during wave-overhang printing), filament sits in the
hot zone of the hotend long enough to soften high up the melt path (heat creep). Eventually
it swells / jams, the extruder gear slips on the now-stuck filament, and the print fails.

The failure is a function of **residence time in the hot zone**, which rises as flow falls.

## 2. Approach — and the critical reframe

**This is a slice-time feature, NOT runtime monitoring.**

The target printer (Bambu X1C) runs closed firmware that cannot be modified, and no runtime
sensing is needed. The slicer already knows the exact duration of every move (it has an
acceleration-aware time estimator). Therefore the guard **accumulates "slow-flow seconds"
deterministically while walking the toolpath, and pre-injects flush sequences into the
G-code before it ever reaches the printer.** No sensors, no firmware changes.

## 3. Behaviour

While the guard is active (see scope), track cumulative time spent extruding at or below a
slow-flow threshold. When that accumulated time exceeds a safe limit:

1. Interrupt the profile extrusion.
2. Z-hop and travel the head to a configurable flush position (off-part).
3. Purge a configurable volume at a fast flow rate to eject the over-soaked filament.
4. Optionally wipe.
5. Return and resume printing.
6. Reset the slow-flow accumulator.

**Scheduling:** prefer to place each flush at the next layer change, or at an existing
travel move within an allowed look-ahead window. Only force a mid-print interruption if the
safe limit would otherwise be exceeded before any natural break.

## 4. Architecture

Implement as a **dedicated G-code post-processing pass** ("thermal dwell guard"), run after
toolpath generation — **not** by editing the wave generator. This keeps concerns separate:
the wave code makes slow toolpaths; the guard inserts flushes.

**Reuse existing OrcaSlicer machinery — do not reinvent.** Locate the current symbols in
this fork (names have likely drifted from upstream) and build on:

| Need | Reuse from |
|---|---|
| "How many seconds" (accel-aware time per move) | `GCodeProcessor` / time-estimate calculator |
| Travel head to a designated position / at layer change | timelapse feature (parks toolhead) |
| Purge a set volume + wipe | filament-change / wipe-tower / prime-tower logic |
| Splice sequences into the stream | layer-change and custom-G-code injection hooks |
| Scope to slow regions | this fork's `; WAVE_OVERHANG_START` / `; WAVE_OVERHANG_END` markers |

For the flush move, reuse existing `travel_speed`, retraction (`retract_length`,
`retract_speed`, `deretract_speed`) and `z_hop` config rather than adding duplicates.

## 5. Physical model notes (do not oversimplify)

- **Accumulator reset:** the real driver is residence time. A burst of *fast* flow flushes
  hot filament out and pulls cooler filament in, relieving heat creep. So the accumulator
  should reset/decay after enough fast extrusion, not tick up forever. The forced flush is
  itself an engineered version of this exchange.
- **Threshold vs flow:** safe dwell gets *shorter* as flow gets slower. v1 uses a single
  constant limit; the correct model is a `(flow -> safe-seconds)` calibration table with
  interpolation (Stage 4).
- **Per-filament:** the thermal options (`max_time`, both flow thresholds,
  `melt_zone_volume`) are logically per-filament (ASA vs PETG vs PLA soak differently). v1
  keeps them global in print config for simplicity — see the TODO in §6.

## 6. Configuration schema

All options share the `thermal_dwell_` prefix. All are inert when
`thermal_dwell_guard_enable` is `false`: the sliced **toolpath must be byte-identical to stock**.
(The one expected difference is OrcaSlicer's embedded `; CONFIG_BLOCK` — for Bambu printers it
dumps the full config, so it gains the new `; thermal_dwell_* = <default>` lines. That is inert
metadata, not behaviour; a diff of an off-guard slice vs stock must contain *only* those lines and
no toolpath changes.) Types are Slic3r `ConfigOption` kinds. `sidetext` = the Unit column.

### Group 1 — Enable & scope

| Key | Type | Default | Unit | Purpose |
|---|---|---|---|---|
| `thermal_dwell_guard_enable` | coBool | `false` | — | Master switch. Off = stock behaviour. |
| `thermal_dwell_scope` | coEnum `ThermalDwellScope` | `WaveOverhangOnly` | — | Where active. v1 scopes via `; WAVE_OVERHANG_START` / `; WAVE_OVERHANG_END` markers; `WholePrint` later. |

**Marker coupling (Stage 1).** Those markers are emitted per *ExtrusionPath* by `GCode::_extrude`,
and were originally gated on `wave_overhang_debug_gcode`. Since `WaveOverhangOnly` makes them a
*functional input* rather than debug output, `_extrude` now also emits them whenever the guard is
enabled with that scope. With the guard off the condition reduces to the original one, so
guard-off output stays byte-identical. Consequence: `wave_overhang_debug_gcode = false` no longer
guarantees a marker-free file when the guard is on.

### Group 2 — Trigger (thermal model)

| Key | Type | Default | Unit | Purpose |
|---|---|---|---|---|
| `thermal_dwell_flow_threshold` | coFloat | `2.0` | mm³/s | At/below this volumetric flow, accumulate slow-flow seconds. |
| `thermal_dwell_max_time` | coFloat | `120` | s | Safe accumulated slow-flow time before a flush is forced. **Calibration placeholder.** |
| `thermal_dwell_fast_flow_threshold` | coFloat | `5.0` | mm³/s | At/above this, extrusion counts as "fast" (drives reset). Must be ≥ `flow_threshold`. |
| `thermal_dwell_reset_mode` | coEnum `ThermalDwellReset` | `OnFastSegment` | — | How the accumulator resets. |
| `thermal_dwell_melt_zone_volume` | coFloat | `15` | mm³ | Effective molten volume in the hotend. Used by `OnVolume` reset and later calibration. |

### Group 3 — Flush action

| Key | Type | Default | Unit | Purpose |
|---|---|---|---|---|
| `thermal_dwell_flush_z_lift` | coFloat | `1.0` | mm | Z-hop before/along travel and during purge, so the nozzle clears the part. |
| `thermal_dwell_purge_volume` | coFloat | `25` | mm³ | Filament ejected to clear over-soaked material. ≈1.5–2× melt-zone volume. |
| `thermal_dwell_purge_flow` | coFloat | `12` | mm³/s | Purge flow — deliberately fast, to force the hot filament exchange. |
| `thermal_dwell_wipe` | coBool | `true` | — | Wipe the nozzle after purging before returning; the wipe sequence is reused from the printer (see below), not synthesised. |

**Flush position + wipe are NOT config options — they are derived from the selected printer.**
(Changed from an earlier `thermal_dwell_flush_position` coPoint: a static point cannot track custom
printer setups.) At flush time the guard resolves the park position and wipe sequence from the
printer, in order:

1. the printer's `change_filament_gcode` (for the X1C this is the AMS macro that parks at the
   poop-chute, ≈ `X70 / Y265`, and wipes),
2. else the printer's `machine_start_gcode`,
3. else a hard-coded fallback = **X1C default park `X70 / Y265`**
   (`THERMAL_DWELL_FALLBACK_FLUSH_POS` in `src/libslic3r/GCode/ThermalDwellGuard.hpp`).

**Do not invoke `change_filament_gcode` wholesale.** On Bambu it is a full AMS *cut + reload* macro
(`M620.x`); the guard must reuse only its **park position + wipe**, then perform its own
same-material purge of `purge_volume` at `purge_flow`. v1 targets the X1C; other printers derive
from their own equivalents later.

### Group 4 — Scheduling

| Key | Type | Default | Unit | Purpose |
|---|---|---|---|---|
| `thermal_dwell_schedule_mode` | coEnum `ThermalDwellSchedule` | `PreferAnyTravel` | — | Alignment strategy. Live from Stage 3; Stages 1–2 behave as `ForcedOnly` regardless. |
| `thermal_dwell_lookahead` | coFloat | `15` | s | Max seconds a flush may be pulled early to hit a layer change / existing travel. |

### Enums

```cpp
enum class ThermalDwellScope    { WaveOverhangOnly, WholePrint };
enum class ThermalDwellReset    { OnTravel,          // reset on any travel move
                                  OnFastSegment,     // reset after a fast-flow segment (v1 default)
                                  OnVolume };         // reset once fast-extruded volume >= melt_zone_volume (most physical)
enum class ThermalDwellSchedule { ForcedOnly,        // flush exactly at the limit
                                  PreferLayerChange, // snap to next layer change within lookahead
                                  PreferAnyTravel };  // snap to next layer change OR mid-layer travel
```

### Modes & constraints

- `mode`: **all options use `comAdvanced`.** (Deviation: the SPEC originally said `comExpert` for
  everything except the enable toggle, but this fork's `ConfigOptionMode` has only
  `comSimple / comAdvanced / comDevelop` — there is no `comExpert`. `comDevelop` is the
  developer-hidden tier, so it would hide the options from the normal power-user UI; `comAdvanced`
  matches the sibling wave-overhang options on the same page and keeps the whole group visible
  together.)
- Enforce `thermal_dwell_fast_flow_threshold >= thermal_dwell_flow_threshold` (validation).
- `// TODO: candidate for per-filament coFloats` on the four thermal options — write their
  accessors so a later move into filament settings is clean, not fought.

## 7. Wiring checklist (the "three touchpoints", expanded to **five**)

A new option that is declared but not wired is invisible, non-persistent, or **crashes the
settings tab**. For **every** key above, do all five:

1. **Declare** the `ConfigOptionDef` in the `PrintConfigDef` init in `PrintConfig.cpp`
   (label, tooltip, `sidetext`/unit, `min`/`max`, `mode`, default value).
2. **Member** — add the matching member to the config class in `PrintConfig.hpp`
   (`PRINT_CONFIG_CLASS` member list). These live in `PrintRegionConfig`, next to the
   wave-overhang options.
3. **Enums** — register enum values + labels so each `coEnum` serializes
   (`enum class` + `CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS` in the header;
   `s_keys_map_<Name>` + `CONFIG_OPTION_ENUM_DEFINE_STATIC_MAPS` in the .cpp).
4. **Preset registration** — add the key to `s_Preset_print_options` in `Preset.cpp`
   (returned by `Preset::print_options()`). **This step was missing from the original
   checklist and is not optional:** it is a hardcoded key list, and a key absent from it is
   not part of the Print preset, so the settings tab **crashes** when it tries to render the
   option. Every `wave_overhang_*` key is listed there too.
5. **UI** — add the options to the print-settings tab (`Tab.cpp` / print settings page)
   under a new **"Thermal dwell guard"** group, and gate the sub-options on the master toggle in
   `ConfigManipulation::toggle_print_fff_options` (`toggle_line(key, thermal_dwell_guard_enable)`),
   so only `thermal_dwell_guard_enable` shows until the guard is enabled — the same
   master-toggle + reveal-sub-options shape the wave-overhang sections use.

Idiom (match the fork's actual surrounding style):

```cpp
def = this->add("thermal_dwell_guard_enable", coBool);
def->label = L("Enable thermal dwell guard");
def->tooltip = L("Insert periodic filament flushes during very-slow extrusion "
                 "to prevent heat-creep jams.");
def->mode = comAdvanced;
def->set_default_value(new ConfigOptionBool(false));
```

## 8. Staged delivery plan

Build and verify each stage before starting the next.

- **Stage 0 — Config + scaffolding.** All options above, defaults off. Complete the
  five-step wiring for each (§7 — incl. the `Preset.cpp` registration). Also scaffolds: the inert flush-position fallback constant in
  `src/libslic3r/GCode/ThermalDwellGuard.hpp`, and a **placeholder "Thermal dwell guard" entry in
  OrcaSlicer's Calibration menu** (info dialog only; the real wizard lands with the calibration
  stage). *Verify:* options appear in the UI; the calibration placeholder appears; with the guard
  disabled, the sliced toolpath is byte-identical to stock — the only diff vs stock is the inert
  `; thermal_dwell_* = <default>` lines added to the embedded `; CONFIG_BLOCK`.
- **Stage 1 — Detect only.** Accumulate slow-flow time within `; WAVE_OVERHANG_START/END`
  regions and mark where a flush *would* occur. No injection. **See
  `docs/thermal_dwell_guard/STAGE1.md`, which governs this stage.** In particular the
  `;THERMAL_DWELL t=NN` sketch above is superseded by STAGE1 §5's three-form grammar
  (`;THERMAL_DWELL_CONFIG` echo, `;THERMAL_DWELL_FLUSH_POINT` lines, `;THERMAL_DWELL_SUMMARY`
  block). *Verify:* STAGE1 §7's acceptance checklist — comments land at sane spots and survive an
  independent recomputation by `scripts/check_thermal_dwell.py`.
- **Stage 2 — Forced flush injection.** At threshold: retract → z-hop → travel to flush pos
  → purge at purge flow → optional wipe → return → resume, then reset accumulator. *Verify:*
  sequences correct in preview and raw G-code.
- **Stage 3 — Smart scheduling.** Add look-ahead so flushes snap to the next layer change or
  existing travel within `thermal_dwell_lookahead`; force a mid-print break only when no
  natural break is available in time. *Verify:* flushes migrate to breaks when possible.
- **Stage 4 — Calibration model.** Replace the constant `max_time` with a
  `(flow -> safe-seconds)` table + interpolation, and turn the Stage-0 Calibration-menu
  placeholder into a real calibration wizard.

## 9. Verification without printing

The fork emits wave-overhang marker comments in the G-code. Confirmed in source
(`GCode::_extrude`): they are **`; WAVE_OVERHANG_START`** and **`; WAVE_OVERHANG_END`** — note the
space after the semicolon, which several older docs get wrong. A separate `;_WAVE_OVERHANG_FAN_*`
family (leading underscore, no space) is consumed and stripped by `CoolingBuffer` and is not
usable for scoping.

To check any stage: slice a small overhang test model, then diff toolpath/G-code before vs
after, and inspect the `;THERMAL_DWELL_*` / flush sequences. This closes the loop in seconds
rather than test prints. Only move to filament once G-code inspection passes.
`scripts/gen_overhang_test_model.py` generates a suitable cantilever test model, and
`scripts/check_thermal_dwell.py` verifies the Stage 1 markers independently.

## 10. Out of scope (v1)

- Runtime/firmware monitoring of any kind.
- Deriving the flush position + wipe from the printer's `change_filament_gcode` /
  `machine_start_gcode` — the mechanism is specified in §6, but v1 wires only the X1C fallback
  constant; robust parsing/derivation is Stage 2.
- Per-filament thermal options (flagged as a TODO, migrated in a later pass).
- Non-wave regions (`WholePrint` scope) — schema-ready, enabled later.
