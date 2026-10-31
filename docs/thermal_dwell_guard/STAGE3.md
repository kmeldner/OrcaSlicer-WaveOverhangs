# Thermal Dwell Guard — Stage 3 Specification: Smart Scheduling

**Status:** implemented (this document governs Stage 3; written in the same commit as the code)
**Placement:** `docs/thermal_dwell_guard/STAGE3.md`
**Relationship to SPEC.md / STAGE2.md:** refines SPEC §8 "Stage 3". STAGE2 §5 (the flush
sequence), §6 (exclusion zones) and the three invariants carry forward unchanged; this stage only
decides *where* a flush is anchored.

---

## 1. Objective and non-goals

Stage 2 anchors every flush exactly on the move that crossed `thermal_dwell_max_time`
(ForcedOnly). That is correct but ugly: the interruption lands mid-perimeter, and the resume
blemish (STAGE2 §8) sits on a visible wall. Stage 3 makes the two Group-4 keys live:

- `thermal_dwell_schedule_mode` (`ForcedOnly` / `PreferLayerChange` / `PreferAnyTravel`)
- `thermal_dwell_lookahead` (seconds)

so that a flush is **pulled early onto a natural break** — a layer change, or any existing
travel move — when such a break lies within `lookahead` seconds *before* the crossing. A forced
mid-extrusion anchor is used only when no acceptable break exists in that window.

**Non-goals (hard):**
- No move splitting, no new travels, no reordering. The flush is still a pure insertion after an
  *existing* line (Invariant 1).
- No change to the accumulator rows (STAGE1 §4), the flush sequence (STAGE2 §5), or the flush
  parameters.
- No calibration model (Stage 4). `max_time` is still a constant.
- `ForcedOnly` must reproduce Stage 2 output byte-for-byte (apart from the appended marker
  fields of §7).

With `thermal_dwell_guard_enable = false`: byte-identical output, as before.

## 2. Why backward look-ahead

"Pull a flush early onto a break" sounds like it needs the future: at a break, the guard would
have to know whether the limit will be crossed within the next `lookahead` seconds. The guard
runs inside `TimeMachine::calculate_time()`, which resolves blocks in a short sliding window, so
genuine look-ahead is not available there — but it is not needed either. The anchors are only
consumed later, in the post-process rewrite, so the decision can be made **at the crossing,
looking back**:

> When the accumulator crosses `t_max` on move M at print-time `T_M`, consider every natural
> break B in the current episode with `T_M − T_B ≤ lookahead`. If one qualifies, anchor the flush
> on B instead of M.

Pulling the flush from M back to B by at most `lookahead` seconds is exactly the SPEC's
definition of the option, with no future knowledge required. The accumulator state is then
replayed from B to M (§5) so detection downstream stays realistic.

## 3. Natural breaks

A move is a **break candidate** iff all of:

1. it is a real **travel** (`TimeBlock::move_type == Travel`: XY displacement without
   extrusion) — retract/unretract-only and pure-Z moves are not candidates on their own;
2. it is **not excluded** (STAGE2 §6: not inside a toolchange / wipe-tower / custom-G-code block);
3. it belongs to the **current episode**: it happened after the last flush and after the last
   accumulator reset (a break before the episode started cannot be "pulled early into").

Its **kind** is:

- `layer_change` — the first break candidate of a new layer (its `layer_id` differs from the
  layer of the previous move the guard saw), i.e. the travel from the layer-change position to
  the first extrusion of the layer. The Z move itself is normally inside the custom layer-change
  block (excluded), so the first eligible travel after it is what gets anchored. The flush then
  returns the head to the start of the first extrusion of the new layer, at the new Z — a
  cosmetically invisible place.
- `travel` — any other candidate.

The guard needs no new markers to detect layer changes: `TimeBlock::layer_id` is already stamped.

## 4. Selection rule

At a crossing on move M (`T_M`), with `W = lookahead`:

| `schedule_mode` | Candidates considered | Chosen |
|---|---|---|
| `ForcedOnly` | none | M (Stage 2 behaviour) |
| `PreferLayerChange` | `layer_change` breaks with `T_M − T_B ≤ W` | the **latest** one; else M |
| `PreferAnyTravel` | `layer_change` and `travel` breaks with `T_M − T_B ≤ W` | the latest `layer_change` if any; else the latest `travel`; else M |

"Latest" minimises how early the flush is pulled, so the dwell actually relieved is as close to
`t_max` as the geometry allows. A layer change wins over a later travel because the resume point
there is invisible.

`T` is the running print time the guard keeps itself: the sum of every move's accel-aware
duration plus every fed dwell (G4 / M400 synchronisations), in the order `on_move` sees them.

**Cost of pulling early.** A flush pulled `early` seconds before the crossing relieves only
`t_max − early` seconds of dwell, so the next crossing comes sooner: with long slow paths that
only break every ~`lookahead` seconds (wave paths at 2 mm/s on the test model), `PreferAnyTravel`
roughly doubled the flush count at `max_time = 30 s, lookahead = 15 s`. At the production
defaults (`120 s` / `15 s`) the overhead is at most ~12 %. Keep `lookahead` well below
`max_time`; it is a cosmetic budget, not a safety one.

**Deferred crossings (STAGE2 §6).** A crossing that lands in an exclusion zone is held and fired
on the first non-excluded move. The scheduling rule applies to that deferred firing exactly as
to a direct one: the window is measured back from the move the crossing was *fired* on.

## 5. Accumulator replay

Anchoring on B means the flush physically happens at B, so the accumulator must look as if it
was reset at B, not at M. After the crossing event (which zeroed the accumulator at M):

1. take the moves strictly after B up to and including M, in order (they are in the guard's
   episode buffer, §6);
2. run them through a **fresh** accumulator state (acc = 0, fast_vol = 0) with the same config;
3. install the resulting `acc` / `fast_vol` as the live state.

The live accumulator's `flush_count` and `total_slow_flow_seconds` are *not* touched by the
replay (the slow seconds were counted once already when the moves were first fed).

If the replay itself crosses `t_max` (only possible when `lookahead ≥ t_max`, since slow time
between B and M is bounded by `T_M − T_B ≤ lookahead`), the replay event is anchored **forced**
on the move it fired on, and replay continues from a zeroed state. This keeps the invariant
"every crossing produces exactly one flush" without adding a second scheduling pass.

## 6. Episode buffer and anchor ordering

The guard keeps a buffer of the moves of the current episode, each with its anchor snapshot
(STAGE2 §4 fields + `g1_line_id`, `layer`, `z`, `T`, break kind). It is trimmed:

- entirely, when the accumulator leaves the episode (acc returns to 0 by a reset row);
- from the front, to moves with `T ≥ T_now − lookahead` (older moves can never be chosen).

Because every candidate B lies in the current episode and after the previous flush, and the
previous flush's anchor is at or before the start of the current episode, the anchors remain
**non-decreasing in `g1_line_id`** — the property the rewrite's merge-join relies on. The guard
checks it and falls back to the forced anchor M if it would ever be violated.

## 7. Marker grammar (extends STAGE2 §7; keep stable)

```
;THERMAL_DWELL_CONFIG ... wipe=<0|1> sched=<forced_only|prefer_layer_change|prefer_any_travel> lookahead=<f>
;THERMAL_DWELL_FLUSH_POINT id=<n> acc=<f>s layer=<n> z=<f> sched=<forced|layer_change|travel> early=<f>s
;THERMAL_DWELL_POINT id=<n> layer=<n> z=<f> acc=<f> sched=<...> early=<f>
```

- `acc` is the accumulator value **at the anchor** (at B for a scheduled flush, so it may be
  below `max_time` by up to `lookahead`). `early = T_M − T_B` (0 for `forced`).
- Both additions are appended at the end of the existing lines, so Stage-2 parsers that stop
  after `z=` keep working.
- The SUMMARY line gains `scheduled=<n>`: how many of `count` were pulled onto a break.

## 8. Verification

**a) Checker script** (`check_thermal_dwell.py`): the naive re-walk gains the same backward
scheduler (window selection + replay). New per-point checks:
- `sched=forced` → `acc ∈ [max_time, +tol]` as before;
- `sched=layer_change|travel` → `acc ≥ max_time − lookahead`, `0 ≤ early ≤ lookahead`, and the
  anchor line (the G0/G1 just before the FLUSH_POINT) is a non-extruding XY move;
- `scheduled` in SUMMARY equals the number of non-forced inline markers;
- the naive scheduler's kinds agree with the slicer's per point (tolerance only on `acc`).
All Stage-2 block checks (pure insertion, state neutrality, invariant-3 scan) still run.

**b) Unit tests** (`test_thermal_dwell.cpp`): the scheduler is a pure class — tests cover: no
break in window → forced; travel in window → anchored on the latest travel with replayed acc;
layer change beats later travel under `PreferAnyTravel`; `PreferLayerChange` ignores travels;
`ForcedOnly` ignores everything; excluded breaks are never chosen; `early` bounded by lookahead.

**c) A/B slice:** the same slow test model sliced with `ForcedOnly` and with `PreferAnyTravel`;
count must match (same crossings), `scheduled > 0`, and every scheduled anchor's `early ≤ W`.

## 9. Acceptance checklist

1. Slicer build succeeds.
2. Guard OFF: byte-identical to Stage 2 (diff empty).
3. `ForcedOnly`: identical to Stage 2 apart from the appended `sched=forced early=0.00s`
   fields and the CONFIG/SUMMARY additions.
4. `PreferAnyTravel` on the slow test model: `scheduled ≥ 1`, each scheduled FLUSH_POINT sits
   directly after a travel, `early ≤ lookahead`.
5. Extended checker passes on both slices (with `--baseline` guard-OFF).
6. Unit tests pass.
7. SPEC.md §6/§8 and CLAUDE.md updated in the same commit.

## 10. Out of scope for Stage 3

Calibration table and wizard (Stage 4); creating new travels or splitting moves to make a
break; forward look-ahead across the time-machine window; per-filament options; multi-material.
