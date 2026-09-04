# Thermal Dwell Guard — Stage 1 Specification: Detection Pass

**Status:** implemented; **partially superseded by Stage 2** (see banner).
**Placement:** `docs/thermal_dwell_guard/STAGE1.md`
**Relationship to SPEC.md:** refines SPEC §8 "Stage 1". Where details differ (e.g. the
marker grammar below supersedes SPEC's `;THERMAL_DWELL t=NN` sketch), THIS file governs —
and SPEC.md §8 must be touched up to match in the same commit.

> **⚠ Superseded by Stage 2 (`STAGE2.md`).** Stage 2 made the guard **standalone**: the
> `thermal_dwell_scope` option and `ThermalDwellScope` enum are **gone**, `in_wave_region`
> tracking and all wave-marker parsing were **removed**, and slow extrusion **always**
> accumulates (STAGE1 §4 rows 1 and 2 merged). The `;THERMAL_DWELL_CONFIG` echo dropped `scope=`
> and gained the Group-3 flush values; the checker's wave-region containment check was removed
> and replaced by per-flush-block validation. Read the scope-specific parts below as historical.

---

## 1. Objective and non-goals

Stage 1 makes the guard *see* — it computes where flushes WOULD go and marks them with
G-code comments. It must not change a single motion, extrusion, temperature, or timing
byte of the output.

**Non-goals (hard):**
- No flush injection, no travel moves, no purging — comments only. (Stage 2)
- No scheduling/look-ahead — markers go exactly where the limit is crossed. (Stage 3)
- Do not read Group 3 or Group 4 config keys (`flush_position`, `flush_z_lift`,
  `purge_volume`, `purge_flow`, `wipe`, `schedule_mode`, `lookahead`). Stage 1 reads only:
  `guard_enable`, `scope`, `flow_threshold`, `max_time`, `fast_flow_threshold`,
  `reset_mode`, `melt_zone_volume`.
- No move splitting. If the limit is crossed mid-move, the marker goes AFTER that move
  (actual dwell may overshoot `max_time` by up to one move's duration — accepted for v1).

With `thermal_dwell_guard_enable = false`, none of the Stage 1 code executes and output
remains byte-identical to stock (Stage 0 guarantee carries forward).

## 2. Integration point — imitate the M73 pattern

OrcaSlicer already runs an acceleration-aware time simulation over the generated G-code
(`GCodeProcessor` + its time machinery) and then, on export, rewrites the file to insert
`M73` progress lines at the right places. That two-phase pattern (analyze during parse →
insert lines during the post-process rewrite) is exactly what Stage 1 needs. **Find the
M73 insertion site in this fork and ride the same mechanism.** Do not re-parse the file in
a separate pass and do not re-derive move times.

Structure:

- `ThermalDwellAccumulator` — a **pure** state machine class (no GCodeProcessor
  dependencies): consumes per-move records `(move kind, duration s, extruded volume mm³,
  in_wave_region flag)`, holds the accumulator, returns "flush point here" events.
  Pure so it is unit-testable with synthetic sequences.
- `ThermalDwellGuard` — the adapter: feeds the accumulator (using GCodeProcessor's existing
  move classification, durations, and volumetric data), records an insertion anchor for each
  flush point, and emits the marker / summary lines during the post-process rewrite,
  alongside where M73 lines are inserted.
- Files: `ThermalDwellGuard.{cpp,hpp}` and `ThermalDwellAccumulator.{cpp,hpp}` placed
  alongside `GCodeProcessor` per this repo's layout.

**Correction (found during implementation — this paragraph supersedes "feeds the accumulator
during parse" above).** A per-move duration does **not** exist at parse time. `TimeBlock::time()`
is derived from a trapezoid that is only final after a planner forward/reverse recalculation,
which `process_G1` triggers in batches once >256 blocks have accumulated (and which keeps the
newest 64 unresolved); the last stragglers resolve only in `finalize()`. The accumulator is
therefore fed from **`TimeMachine::calculate_time()`**, the one place where a block's resolved
duration is in scope — in `ETimeMode::Normal` only, since that function runs once per time mode.
Three consequences, all load-bearing:

- The per-move data the accumulator needs that is *not* on the resolved block (extruded volume,
  Z, the wave-region flag) is **stamped onto `TimeBlock` in `process_G1` / `process_VG1`**,
  because by the time the block resolves, the live parser state has advanced by up to 256 blocks.
  It is stamped rather than read back from `m_result.moves[block.move_id]` because `move_id` is
  remapped when actual-speed moves are inserted, and `process_VG1` never sets it at all.
- The feed must sit **above** `calculate_time`'s `if (mode == Normal)` block, whose type filter
  `continue`s the loop for anything that is not Extrude/Travel/Wipe — i.e. exactly the
  Retract/Unretract moves rows 6–8 depend on.
- It must use the block's **own** time, not the loop's `block_time`, which folds
  `simulate_st_synchronize()`'s `additional_time` into block 0 (see §4).

Anchors are recorded as **`TimeBlock::g1_line_id`**, not a line position: there is no stored line
table anywhere. The parse side stamps `g1_line_id`, and the rewrite independently re-derives the
identical counter (++ per G0/G1, += 1+internal per G2/G3, ++ per G28) and merge-joins the two —
the same mechanism the M73 emitter rides.

Use GCodeProcessor's own per-move data:
- **Duration:** the accel-aware move time it already computes.
- **Volume:** extruded volume per move (ΔE × filament cross-section), which it already
  tracks for the preview's volumetric-flow view.
- **Average flow q [mm³/s] = volume / duration** for extrusion moves.
- **Move kind:** reuse GCodeProcessor's move classification (extrude / travel / retract /
  unretract / etc.). Do not re-derive from raw G-code.

## 3. Region and layer tracking

- The fork emits wave-overhang region comments. **Confirmed in source** (`GCode::_extrude`):
  the exact strings are **`; WAVE_OVERHANG_START`** and **`; WAVE_OVERHANG_END`** — with a
  **space after the semicolon**, which this document and SPEC.md originally got wrong. An
  exact-prefix match on `;WAVE_OVERHANG_START` never fires. `process_gcode_line` strips only
  the `;`, so the tag constants carry the leading space. Match the strings in full:
  `; WAVE_OVERHANG_BUILD` and `; WAVE_OVERHANG_CONFIG` share the prefix.
- The markers are emitted **per ExtrusionPath**, so START/END pairs repeat many times across
  one visual overhang region. `in_wave_region` is therefore a *level*, not a latch — this is
  harmless for the flag, but no single pair brackets a whole region.
- `; WAVE_OVERHANG_CONFIG` is a **one-shot file header** (one line per region, inside
  `HEADER_BLOCK_START`), not a region-boundary banner. It cannot be used to detect transitions.
  The `;_WAVE_OVERHANG_FAN_*` family (leading underscore, no space) is a different convention
  and is consumed and stripped by `CoolingBuffer`.
- The markers were gated on `wave_overhang_debug_gcode`, a user-visible toggle — meaning
  `WaveOverhangOnly` would silently no-op if a user turned debug G-code off. Resolved by also
  emitting them when the guard is enabled with that scope; see SPEC.md §6 "Marker coupling".
- Track current layer index and Z (GCodeProcessor already follows layer changes) so each
  flush point can be labeled. Use the move's Z, not `MoveVertex::print_z`, which stays 0 for
  non-BBL printers (the emitter writes `;Z:` but the parser only reads `; Z_HEIGHT:`).

## 4. Accumulator state machine

State: `acc` (seconds, ≥ 0), `fast_vol` (mm³, used only by `OnVolume`).

Config: `q_slow = thermal_dwell_flow_threshold`, `q_fast = thermal_dwell_fast_flow_threshold`,
`T_max = thermal_dwell_max_time`, `V_melt = thermal_dwell_melt_zone_volume`.

"In scope" = (`scope == WholePrint`) OR (`scope == WaveOverhangOnly` AND `in_wave_region`).
**(Stage 2: scope removed — everything is in scope, so rows 1 and 2 below merge into one.)**

Per move (duration `d`, volume `v`, flow `q = v/d` for extrusion moves):

| # | Move | Condition | Effect on `acc` | Effect on `fast_vol` |
|---|------|-----------|-----------------|----------------------|
| 1 | Extrusion | `q ≤ q_slow`, in scope | `acc += d` | `= 0` |
| 2 | Extrusion | `q ≤ q_slow`, out of scope | unchanged | `= 0` |
| 3 | Extrusion | `q_slow < q < q_fast` (hysteresis band) | unchanged | unchanged |
| 4 | Extrusion | `q ≥ q_fast`, reset mode `OnFastSegment` or `OnTravel` | `= 0` | `= 0` |
| 5 | Extrusion | `q ≥ q_fast`, reset mode `OnVolume` | if `fast_vol + v ≥ V_melt`: `acc = 0` | if reset: `= 0`; else `+= v` |
| 6 | Non-extruding (travel, retract, unretract, z move, G4 dwell) | reset mode `OnTravel` | `= 0` | `= 0` |
| 7 | Non-extruding | other reset modes, `acc > 0` | `acc += d` | unchanged |
| 8 | Non-extruding | other reset modes, `acc == 0` | unchanged | unchanged |

**Rows 6–8 are NOT scope-gated** (the table's blank Condition cells only imply this). The
filament soaks in the hot zone wherever the nozzle happens to be, so a travel that leaves a wave
region still extends an episode that started inside it.

**Firmware-synchronize dwells.** `simulate_st_synchronize`'s `additional_time` (G4 / M400 / tool
change — including the `G4` that `wave_overhang_min_wave_time` emits after *every* wave path) is
folded into block 0's reported time by the time estimator, purely for total-time accounting.
Feeding that inflated figure as a move's duration would corrupt `q = v/d` and mis-file a fast
move as slow. It is instead fed as its own **synthetic non-extruding move** (rows 6–8) after the
batch of blocks it follows.

Rationale worth preserving in code comments:
- **Row 7 (travel extends an episode):** during non-extruding time the filament is fully
  static in the hot zone — residence continues to accrue. Travels never *start* an episode
  (row 8), only extend one.
- **Row 3 (hysteresis band):** mid-band flow does exchange some filament, but v1 keeps a
  binary model; the band prevents flapping between "accumulating" and "curing".
- **Row 1/2 (`fast_vol = 0` on any slow extrusion):** a slow move interrupts a partial
  fast exchange regardless of scope.

**Threshold check — after every row that changes `acc`:** if `acc ≥ T_max`, record a flush
point anchored AFTER the current move's line, labeled with (`id` 1..N global, current
layer, Z, `acc` value), then `acc = 0`, `fast_vol = 0` (simulating the flush the guard
would have performed, so detection continues realistically downstream).

Two clarifications from the implementation:
- Rows 4/5/6 change `acc` only *downward* (to zero), where the check is a provable no-op for any
  `T_max > 0`. It is therefore implemented on **rows 1 and 7 only** — the two that raise it.
- `T_max <= 0` must **never** fire. The table leaves this open, but without the guard the
  post-reset `acc == 0` state would satisfy `acc >= T_max` and fire on every subsequent move.
- A zero-duration extrusion is treated as infinitely fast (not a divide-by-zero into row 1).

Note: a flush point may therefore land right after a travel (row 7 crossing) — that is
fine and is in fact a preview of Stage 3's preferred placement.

## 5. Emitted comments — exact grammar

**The fork has no single comment convention** — the premise that there is one is false. It emits
`; WAVE_OVERHANG_START` (space), `;_WAVE_OVERHANG_FAN_START` (underscore, no space), `;TYPE:`
and `;LAYER_CHANGE` (no space), and `; CHANGE_LAYER` (space, BBL). The no-space `;THERMAL_DWELL_*`
form below is therefore a deliberate choice and is **final**; the checker script matches it
exactly. Canonical forms:

- **Config echo** — once, in the header/footer area. **(Stage 2 dropped `scope=` and appended
  the Group-3 flush values — see STAGE2 §7 for the current form.)** Stage-1 form:

  `;THERMAL_DWELL_CONFIG scope=<WaveOverhangOnly|WholePrint> flow_max=<f> fast_min=<f> max_time=<f> reset=<OnTravel|OnFastSegment|OnVolume> melt_zone=<f>`

- **Flush point** — at each recorded anchor:

  `;THERMAL_DWELL_FLUSH_POINT id=<n> acc=<f>s layer=<n> z=<f>`

- **Summary block** — appended at end of file:

  ```
  ;THERMAL_DWELL_SUMMARY count=<n>
  ;THERMAL_DWELL_POINT id=1 layer=<n> z=<f> acc=<f>
  ;THERMAL_DWELL_POINT id=2 ...
  ```

These strings are load-bearing: Stage 2 replaces flush-point comments with real flush
sequences, and the verification script parses them. Keep them stable.

Also log a one-line INFO per slice via the repo's logging (BOOST_LOG or equivalent):
flush-point count and total slow-flow seconds.

**Config transport.** The `thermal_dwell_*` keys live in `PrintRegionConfig` (Stage 0), while
GCodeProcessor is global and receives only `PrintConfig`. Rather than have the processor parse its
own echo back, a single shared `ThermalDwellGuard::resolve_config(const Print&)` is called by both
GCode.cpp (to emit the echo) and GCodeProcessor (to decide whether to build a guard at all) — so
the echo cannot describe anything other than what ran. `GCodeProcessor::m_print` is set before the
first byte is written, and `Print::num_print_regions()` / `get_print_region()` are public, so no
round-trip is needed.

The echo is emitted **outside** the `HEADER_BLOCK_START` section: that whole block — including
`; WAVE_OVERHANG_CONFIG` — is skipped when a BTT_TFT thumbnail is configured, and the guard must
not depend on a thumbnail setting.

Because the config is per-region but the hotend is not, the echo (and the accumulator) reflect the
**first guard-enabled region**. `TODO(Stage 2+)`: per-region accumulators, or hoist these keys out
of `PrintRegionConfig`.

## 6. Verification tooling (part of Stage 1, not optional)

**a) Independent checker script** — `check_thermal_dwell.py`, placed in the repo's
conventional scripts/tools directory. Python 3, stdlib only. Input: a G-code file path.
Behavior:
- Reads `;THERMAL_DWELL_CONFIG` to self-configure thresholds. It **also** needs
  `; filament_diameter = ` from the config block Orca appends to the file, to convert ΔE into
  mm³ — that value is not part of the echo.
- Independently re-computes the accumulation by walking G-code moves with a NAIVE time
  model (distance / feedrate, no acceleration). Because the slicer's model is
  accel-aware, allow tolerance: each marker's `acc` must be within ±25% of the naive
  recomputation at that point (tolerance configurable via CLI flag).
- Checks: (1) every `FLUSH_POINT` acc ≈ `max_time` within tolerance; (2) with
  `scope=WaveOverhangOnly`, every marker lies inside or immediately after a wave region;
  (3) inline marker count equals `SUMMARY count`; (4) ids are 1..N consecutive.
- Exit 0 with a PASS report, nonzero with specific failures.

Refinement to check (2): a *positional* "marker is near a wave region" test would be wrong,
because row 7 lets an episode span an unbounded travel or layer change before crossing the limit.
Since the checker re-runs the whole accumulator anyway, it tests the **causal** property instead —
each marker's episode must have accrued at least some time inside a wave region
(`episode_wave_seconds > 0`). Faithful to rows 1/7, and no fudge factor.

**b) Unit tests** on `ThermalDwellAccumulator` with synthetic move sequences, IF this
fork's test harness builds. The harness is **Catch2 v3.11.0**, vendored at `tests/catch2`
(`#include <catch2/catch_all.hpp>`, link `Catch2::Catch2WithMain`) — note `tests/CLAUDE.md`
claims v2, which is wrong. Tests are gated behind `BUILD_TESTS` (CMakeLists.txt), which is
**OFF** by default and had never been configured in this fork, so adding a test file is inert
until someone reconfigures with `-DBUILD_TESTS=ON`. Cases: slow-only
crossing at the right time; fast reset (`OnFastSegment`); `OnVolume` partial then full
exchange; travel extension of an episode (row 7); travel NOT starting an episode (row 8);
`OnTravel` reset; scope gating; hysteresis-band neutrality; post-flush-point reset. If
the harness does not build in this fork, report that and rely on the checker script —
do not stall Stage 1 on test infrastructure.

## 7. Acceptance checklist (all must pass before Stage 1 is done)

1. Slicer build succeeds (slicer build only, per CLAUDE.md — do not rebuild deps).
2. Guard OFF: sliced G-code of the test model is byte-identical to a stock slice (diff).
   "Stock" here means **the Stage-0 build** — the parent commit. SPEC.md §6 carves out the inert
   `; thermal_dwell_* = <default>` lines that Orca dumps into the embedded `; CONFIG_BLOCK`, but
   those landed in Stage 0, so they are already present in the baseline. Against a Stage-0
   baseline the diff must therefore be **exactly empty**, not "empty modulo config lines".
3. Guard ON on a wave-overhang test model with `max_time=30` (to force multiple events)
   and other options at defaults: at least 2 `FLUSH_POINT` markers appear, all within or
   immediately after wave regions; `CONFIG` echo and `SUMMARY` block present & consistent.
4. `check_thermal_dwell.py` passes on that file.
5. Unit tests pass (or a clear report that the fork's harness is unusable).
6. Completion report: files touched, final marker grammar, any deviations — with SPEC.md
   / STAGE1.md updated in the same commit per the working agreement.

## 8. Explicitly out of scope for Stage 1

Flush injection and any motion changes; scheduling/look-ahead; move splitting;
proportional/decay accumulator models; counting hysteresis-band flow toward `fast_vol`;
travel-initiated episodes; per-filament options; X1C purge-macro integration.

## 9. Future refinements to note in code TODOs (do not implement)

- Proportional decay of `acc` by exchanged volume instead of binary reset.
- Credit mid-band flow toward melt-zone exchange (`OnVolume`).
- Cold-start episodes after long pauses following fast printing.
