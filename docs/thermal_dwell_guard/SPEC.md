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

The guard is a **standalone** module (Stage 2 onward): it watches every move's volumetric flow
across the whole print — there is no scope option and no dependency on wave-overhang markers.
Wave overhangs are simply one thing that produces slow flow; slow walls, infill, bridges and
overhang slowdowns count too. Track cumulative time spent extruding at or below a slow-flow
threshold. When that accumulated time exceeds a safe limit:

1. Interrupt the profile extrusion.
2. Z-hop and travel the head to the printer's designated flush position (the X1C poop-chute,
   off-bed) — never onto the bed.
3. Purge a configurable volume at a fast flow rate to eject the over-soaked filament.
4. Optionally wipe on the machine wiper at that position.
5. Return and resume printing, restoring machine state exactly.
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

For the flush move, reuse existing `travel_speed`, retraction (`retract_length`,
`retract_speed`, `deretract_speed`) and `z_hop` config rather than adding duplicates.

## 5. Physical model notes (do not oversimplify)

- **Accumulator reset:** the real driver is residence time. A burst of *fast* flow flushes
  hot filament out and pulls cooler filament in, relieving heat creep. So the accumulator
  should reset/decay after enough fast extrusion, not tick up forever. The forced flush is
  itself an engineered version of this exchange.
- **Threshold vs flow:** safe dwell gets *shorter* as flow gets slower. The constant limit
  `max_time` is the fallback; with `thermal_dwell_calibration_table` set, the guard accumulates
  the *dwell fraction* `Δt / T(q)` against an interpolated `(flow -> safe-seconds)` table and
  flushes at 1 (Stage 4, see `STAGE4.md`).
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

### Group 1 — Enable

| Key | Type | Default | Unit | Purpose |
|---|---|---|---|---|
| `thermal_dwell_guard_enable` | coBool | `false` | — | Master switch. Off = stock behaviour. |

**Standalone (Stage 2).** The `thermal_dwell_scope` option and the `ThermalDwellScope` enum were
**removed** in Stage 2 — the guard always monitors the whole print by flow rate, with no
dependency on wave-overhang markers. `GCode::_extrude` reverted its marker emission to being
gated solely on `wave_overhang_debug_gcode` (a pure debug aid again). The module is portable to
stock OrcaSlicer.

### Group 2 — Trigger (thermal model)

| Key | Type | Default | Unit | Purpose |
|---|---|---|---|---|
| `thermal_dwell_flow_threshold` | coFloat | `2.0` | mm³/s | At/below this volumetric flow, accumulate slow-flow seconds. |
| `thermal_dwell_max_time` | coFloat | `120` | s | Safe accumulated slow-flow time before a flush is forced. **Calibration placeholder.** |
| `thermal_dwell_fast_flow_threshold` | coFloat | `5.0` | mm³/s | At/above this, extrusion counts as "fast" (drives reset). Must be ≥ `flow_threshold`. |
| `thermal_dwell_reset_mode` | coEnum `ThermalDwellReset` | `OnFastSegment` | — | How the accumulator resets. |
| `thermal_dwell_melt_zone_volume` | coFloat | `15` | mm³ | Effective molten volume in the hotend. Used by `OnVolume` reset and later calibration. |
| `thermal_dwell_calibration_table` | coString | `""` | — | `flow:seconds` pairs (`;`-separated) from the calibration print; empty = constant `max_time` (Stage 4, `STAGE4.md`). |

### Group 3 — Flush action

| Key | Type | Default | Unit | Purpose |
|---|---|---|---|---|
| `thermal_dwell_flush_z_lift` | coFloat | `1.0` | mm | Z-hop before/along travel and during purge, so the nozzle clears the part. |
| `thermal_dwell_flush_position` | coPoint | `70, 265` | mm | Machine flush/wipe XY. Default = the Bambu X1C poop-chute. |
| `thermal_dwell_purge_volume` | coFloat | `25` | mm³ | Filament ejected to clear over-soaked material. ≈1.5–2× melt-zone volume. |
| `thermal_dwell_purge_flow` | coFloat | `12` | mm³/s | Purge flow — deliberately fast, to force the hot filament exchange. |
| `thermal_dwell_wipe` | coBool | `true` | — | Wipe on the machine wiper at the flush position; the built-in X1C wiper pattern (motion only) is replicated, not synthesised from filament-change G-code. |

**Flush position + wiper pattern (Stage 2 — reversed from the Stage-0/1 note).** Rather than
parse the user's machine G-code at slice time (arbitrary templates have no reliable anchor), the
X1C chute XY and wiper sweep were **extracted at development time** from the bundled profile
`resources/profiles/BBL/machine/Bambu Lab X1 Carbon 0.4 nozzle.json` (`change_filament_gcode`:
`G1 X70 …` / `G1 Y245 → Y265`) and baked as the default of `thermal_dwell_flush_position` and a
built-in wiper pattern. The fallback constant `THERMAL_DWELL_FALLBACK_FLUSH_POS` in
`src/libslic3r/GCode/ThermalDwellGuard.hpp` mirrors the same XY.

**Do not invoke `change_filament_gcode` wholesale.** On Bambu it is a full AMS *cut + reload* macro
(`M620.x`) with temperature ramps — minutes of destructive state change. The guard replicates only
the **motion subset** (travel to chute → purge → wiper drag), generating its own same-material
purge of `purge_volume` at `purge_flow` so E-accounting stays provably correct. v1 targets the X1C;
other printers get per-machine templates later.

### Group 4 — Scheduling

| Key | Type | Default | Unit | Purpose |
|---|---|---|---|---|
| `thermal_dwell_schedule_mode` | coEnum `ThermalDwellSchedule` | `PreferAnyTravel` | — | Where a flush lands: on the crossing move, or pulled early onto a layer change / travel within the look-ahead (live since Stage 3, see `STAGE3.md`). |
| `thermal_dwell_lookahead` | coFloat | `15` | s | Max seconds a flush may be pulled early to hit a layer change / existing travel. |

### Enums

```cpp
// (ThermalDwellScope was removed in Stage 2 — the guard is standalone / whole-print.)
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
- **Stage 1 — Detect only.** Accumulate slow-flow time (Stage 1 scoped this to
  `; WAVE_OVERHANG_START/END` regions; Stage 2 removed that scope) and mark where a flush *would*
  occur. No injection. **See `docs/thermal_dwell_guard/STAGE1.md`, which governs this stage.** In
  particular the `;THERMAL_DWELL t=NN` sketch above is superseded by STAGE1 §5's three-form grammar
  (`;THERMAL_DWELL_CONFIG` echo, `;THERMAL_DWELL_FLUSH_POINT` lines, `;THERMAL_DWELL_SUMMARY`
  block). *Verify:* STAGE1 §7's acceptance checklist — comments land at sane spots and survive an
  independent recomputation by `scripts/check_thermal_dwell.py`.
- **Stage 2 — Standalone module + chute flush injection.** Part A drops the scope option so the
  guard is standalone (whole-print, flow-only, no wave dependency). Part B, at threshold: retract →
  z-hop → travel to the machine flush position (X1C chute) → purge at purge flow → optional wiper
  drag → return → restore state → resume, then reset the accumulator. **See
  `docs/thermal_dwell_guard/STAGE2.md`, which governs this stage.** *Verify:* STAGE2 §10.
- **Stage 3 — Smart scheduling.** Flushes are pulled early onto the latest layer change or
  existing travel within `thermal_dwell_lookahead` (backward look-ahead at the crossing, then
  an accumulator replay from the chosen break); a mid-extrusion anchor is forced only when no
  natural break is close enough. **See `docs/thermal_dwell_guard/STAGE3.md`, which governs this
  stage.** *Verify:* STAGE3 §9.
- **Stage 4 — Calibration model.** Part A (done): `thermal_dwell_calibration_table` with linear
  interpolation and dwell-fraction accumulation; `max_time` stays as the fallback. Part B
  (pending): the guided calibration wizard — the Calibration-menu entry currently describes the
  manual procedure. **See `docs/thermal_dwell_guard/STAGE4.md`.**

## 9. Verification without printing

To check any stage: slice a small test model, then diff toolpath/G-code before vs after, and
inspect the `;THERMAL_DWELL_*` markers and flush sequences. This closes the loop in seconds rather
than test prints. Only move to filament once G-code inspection passes.
`scripts/gen_overhang_test_model.py` generates a suitable cantilever test model, and
`scripts/check_thermal_dwell.py` verifies the markers and (Stage 2) the inserted flush blocks
independently. (The fork still emits `; WAVE_OVERHANG_START/END` debug markers — space after the
semicolon — but the guard no longer reads them; they are a pure `wave_overhang_debug_gcode` aid.)

## 10. Out of scope (v1)

- Runtime/firmware monitoring of any kind.
- Per-machine flush templates for non-X1C printers — Stage 2 bakes the X1C chute XY + wiper as
  defaults; other printers get their own templates later.
- Per-filament thermal options (flagged as a TODO, migrated in a later pass).
