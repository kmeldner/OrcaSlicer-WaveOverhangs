# Thermal Dwell Guard — Stage 4 Specification: Calibration Model

**Status:** Part A (model) implemented; Part B (wizard) designed, placeholder dialog describes the
manual procedure
**Placement:** `docs/thermal_dwell_guard/STAGE4.md`
**Relationship to SPEC.md:** refines SPEC §5 "Threshold vs flow" and §8 "Stage 4". Stages 1–3
carry forward unchanged; this stage only changes *when* the limit is reached.

---

## 1. Objective and non-goals

Stages 1–3 use one constant, `thermal_dwell_max_time`: the same safe dwell whether the filament
crawls at 0.3 mm³/s or ambles at 1.9 mm³/s. Physically the safe dwell **shrinks as flow falls**
(SPEC §5): slower flow brings less cold filament into the melt path per second, so the heat
front climbs further before anything is exchanged. Stage 4 lets the guard carry a per-filament
**calibration table** `(flow → safe seconds)` and accumulate *dwell fraction* against it.

**Non-goals (hard):**
- No change to scheduling (Stage 3), the flush sequence (Stage 2) or the reset rows (Stage 1).
- With an **empty table** the guard is byte-identical to Stage 3: `max_time` stays the limit.
- No runtime sensing. The table is produced by a calibration print and typed/stored once.

## 2. Model — dwell fraction (Part A)

Let `T(q)` be the safe dwell at volumetric flow `q`, taken from the table by **linear
interpolation in q**, clamped to the first entry below the table's lowest flow and to the last
entry above its highest. `T(q)` is only consulted for flow at or below `thermal_dwell_flow_threshold`
(row 1 of STAGE1 §4); faster flow is governed by the reset rows exactly as before.

The accumulator keeps, per episode:

- `acc` — slow-flow seconds, as before (reported in the markers, used by `--tolerance` checks);
- `frac` — **dwell fraction**: `frac += Δt / T(q)` for every slow move (row 1), and
  `frac += Δt / T_static` for non-extruding time inside an episode (row 7), where
  `T_static = T(lowest tabulated flow)` — static filament soaks at least as fast as the slowest
  flow we calibrated.

A flush is due when `frac ≥ 1` (Miner's rule: the damage fractions of segments at different
flows add). Resets (rows 4/5/6) and flushes zero both `acc` and `frac`. With an empty table
`T(q) ≡ max_time`, so `frac = acc / max_time` and the crossing condition is the Stage-3 one;
the implementation keeps the exact `acc ≥ t_max` comparison in that case so the output stays
byte-identical.

`thermal_dwell_max_time` remains meaningful as the **fallback** when the table is empty, and as
the documented "calibration placeholder" (SPEC §6).

## 3. Configuration

| Key | Type | Default | Unit | Purpose |
|---|---|---|---|---|
| `thermal_dwell_calibration_table` | coString | `""` | — | `flow:seconds` pairs separated by `;`, e.g. `0.5:45;1:70;2:120`. Empty = use `max_time`. |

Parsing is tolerant: whitespace ignored, pairs sorted by flow, entries with non-positive flow or
seconds dropped, duplicates collapsed (last wins). The parsed table is echoed back in the CONFIG
marker so the checker can reproduce it. Mode `comAdvanced`, same page/group ("Trigger") and
master-toggle gating as the other thermal options. It is wired through all five touchpoints of
SPEC §7 (declare, member, —, `Preset.cpp`, `Tab.cpp`/`ConfigManipulation.cpp`).

`// TODO: candidate for per-filament` applies to this key above all others — ASA, PETG and PLA
soak differently. v1 keeps it in print config beside its siblings.

## 4. Marker grammar (extends STAGE3 §7; keep stable)

```
;THERMAL_DWELL_CONFIG ... lookahead=<f> table=<q:t;q:t;...|none>
;THERMAL_DWELL_FLUSH_POINT id=<n> acc=<f>s layer=<n> z=<f> sched=<kind> early=<f>s frac=<f>
;THERMAL_DWELL_POINT id=<n> layer=<n> z=<f> acc=<f> sched=<kind> early=<f> frac=<f>
```

`frac` is the dwell fraction at the anchor (≥ 1 for a forced flush; for a scheduled one it is
the fraction at the break). With an empty table `frac = acc / max_time`.

## 5. Checker

`check_thermal_dwell.py` parses `table=` and `frac=`; its naive accumulator computes `frac`
with the same interpolation. Check 1a becomes: forced → `frac ∈ [1, 1 + tol]`; scheduled →
`frac ≥ 1 − lookahead / T_static`, `early ≤ lookahead`. The `acc`-vs-`max_time` test is kept
only when `table=none`. Everything else is unchanged.

## 6. Calibration print (Part B — the wizard)

**What is measured.** For one filament and one flow `q`, how long continuous extrusion at `q`
can run before heat creep shows (under-extrusion, grinding, or a jam). That time is `T(q)`.

**Test object.** A single-wall hollow cylinder (vase-like, one perimeter per layer, no infill,
no top/bottom), diameter 25 mm, 100 mm tall, with a small notch every 10 layers as a height
ruler. Printed with the guard **off** and `wave_overhangs` off, at a constant flow set through
the process preset: `q = outer_wall_speed × line_width × layer_height`. With 0.42 mm lines and
0.2 mm layers, `q = 0.5 / 1.0 / 2.0 mm³/s` means wall speeds of `6 / 12 / 24 mm/s`. Cooling
fan and temperatures as the filament's normal profile.

**Procedure.** Print one cylinder per flow (three flows spanning the slow band is enough: the
lowest flow the user's wave overhangs produce, `flow_threshold`, and one in between). Watch,
or let it fail unattended and read the ruler: the notch count where extrusion thinned or
stopped, times seconds-per-layer (the slicer prints that on the plate name), is `T(q)`. Take
80 % of the observed time as the safe value. A cylinder that completes gives a **lower bound**
(`T ≥ print time`); use it as is and note that the flow is safe for at least that long.

**Wizard.** The Calibration-menu entry opens a dialog that (1) asks for the three flows and the
layer height / line width, (2) generates the three cylinders onto the plate with per-object
wall speed set for each flow, the guard and wave overhangs disabled, and the plate named with
seconds-per-layer, (3) after printing, lets the user type the observed times and writes
`thermal_dwell_calibration_table` into the current process preset. Follows the shape of the
existing flow-rate / pressure-advance calibration dialogs (`calib_dlg.cpp`, `CalibUtils`).

**Status.** Part B is **not yet implemented**: the menu entry shows this procedure as text so the
calibration can be done by hand today (the model, table key, checker and tests are live). The
wizard's generator + results dialog is the remaining Stage-4 work.

## 7. Acceptance checklist

1. Slicer build succeeds.
2. Guard OFF: byte-identical to Stage 3.
3. Guard ON, empty table: byte-identical to Stage 3 apart from the appended `table=none` and
   `frac=` marker fields.
4. Guard ON with a table on the slow test model: flush count changes in the expected direction
   (a table stricter than `max_time` at the model's flows gives more flushes; a looser one
   fewer), `frac ≥ 1` on every forced point, checker passes with `--baseline`.
5. Unit tests: interpolation (inside, clamped both ends, single entry), empty table equals the
   constant model, mixed-flow episode sums fractions, static time uses `T_static`, parser
   tolerance.
6. SPEC.md §5/§6/§8/§10 and CLAUDE.md updated in the same commit.

## 7a. Known limitation surfaced by the table (all stages): single-move overshoot

STAGE1 §1 forbids move splitting, so a flush is anchored after the move that crossed the limit
and the real dwell overshoots by up to that move's duration. On the cantilever test model the
wave paths at 2 mm/s are single ~15 s extrusion moves: with the constant `max_time = 30 s` two
such moves land exactly on the limit, but with a table giving `T = 20 s` the crossing happens
mid-move and the flush lands at ~30 s (`frac ≈ 1.5`). The checker accepts an overshoot that
stays within the crossing move (its naive model confirms the limit was unreached before it).

For slow, long paths this margin can be a large fraction of the safe time. The natural fix is
**pre-emptive anchoring**: when the *next* move alone would carry the episode past the limit,
anchor the flush at the end of the previous move (any move end is a valid forced anchor) so the
limit is never exceeded, at the cost of flushing early by up to one move. It fits the Stage-3
scheduler (it is a one-move backward pull with a different trigger) and is the recommended
next refinement; it is not in Stage 4.

## 8. Out of scope for Stage 4

Per-filament placement of the thermal keys; proportional decay of `acc`/`frac` by exchanged
volume (STAGE1 §9); multi-material; the wizard's generator and results dialog (Part B, see §6).
