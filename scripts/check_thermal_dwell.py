#!/usr/bin/env python3
"""Independently verify the thermal dwell guard's markers in a sliced G-code file.

This is a deliberate second implementation of the accumulator (see
docs/thermal_dwell_guard/STAGE1.md §4 and §6a). It re-walks the G-code with a NAIVE time
model -- distance / feedrate, no acceleration -- and checks the slicer's markers against it.
Because the slicer's estimate IS acceleration-aware, the two never agree exactly; the
comparison is tolerance-based (default +/-25%, see --tolerance).

Checks performed:
  1. every FLUSH_POINT's acc is close to max_time, and close to the naive recomputation
  2. with scope=WaveOverhangOnly, every marker's episode accrued time inside a wave region
  3. the inline FLUSH_POINT count equals the SUMMARY count
  4. ids are 1..N consecutive and the inline markers agree with the SUMMARY entries

Usage:
    python3 check_thermal_dwell.py <file.gcode> [--tolerance 0.25] [--verbose]
"""

import argparse
import math
import re
import sys

DEFAULT_TOLERANCE = 0.25

# Marker grammar (STAGE1 §5). These strings are load-bearing: Stage 2 replaces the flush-point
# comments with real flush sequences, so keep them in sync with ThermalDwellGuard.
RE_CONFIG = re.compile(
    r"^;THERMAL_DWELL_CONFIG scope=(\S+) flow_max=(\S+) fast_min=(\S+) "
    r"max_time=(\S+) reset=(\S+) melt_zone=(\S+)"
)
RE_FLUSH_POINT = re.compile(
    r"^;THERMAL_DWELL_FLUSH_POINT id=(\d+) acc=([\d.]+)s layer=(\d+) z=([\d.-]+)"
)
RE_SUMMARY = re.compile(r"^;THERMAL_DWELL_SUMMARY count=(\d+)")
RE_POINT = re.compile(
    r"^;THERMAL_DWELL_POINT id=(\d+) layer=(\d+) z=([\d.-]+) acc=([\d.]+)"
)

# Wave-overhang region markers as GCode::_extrude() actually emits them. The space after the
# semicolon is real -- matching ";WAVE_OVERHANG_START" finds nothing.
WAVE_START = "; WAVE_OVERHANG_START"
WAVE_END = "; WAVE_OVERHANG_END"

# Filament diameter is needed to turn dE into mm^3 and is NOT part of the CONFIG echo; it comes
# from the config block Orca appends to the end of the file.
RE_FIL_DIAM = re.compile(r"^; filament_diameter\s*=\s*([\d.]+)")

RE_WORD = re.compile(r"([A-Za-z])\s*(-?[\d.]+)")


def print_error(msg):
    print(f"\033[91m[FAIL]\033[0m {msg}")


def print_ok(msg):
    print(f"\033[92m[PASS]\033[0m {msg}")


def print_info(msg):
    print(f"\033[94m[INFO]\033[0m {msg}")


def parse_words(line):
    """Split a G-code line's parameters into {letter: value}, ignoring any comment."""
    code = line.split(";", 1)[0]
    return {m.group(1).upper(): float(m.group(2)) for m in RE_WORD.finditer(code)}


class NaiveAccumulator:
    """The §4 state machine again, fed by a no-acceleration time model.

    Mirrors ThermalDwellAccumulator deliberately rather than sharing code: a bug copied into
    both implementations would be invisible, which would defeat the point of this script.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.acc = 0.0
        self.fast_vol = 0.0
        self.count = 0
        # Seconds of THIS episode accrued inside a wave region. Reset with acc. Used for
        # check 2: a positional "marker is near a wave region" test would be wrong, because
        # row 7 lets an episode span an unbounded travel or layer change.
        self.episode_wave_s = 0.0

    def step(self, is_extrusion, duration, volume, in_wave):
        if is_extrusion:
            q = volume / duration if duration > 0.0 else math.inf
            if q <= self.cfg["q_slow"]:
                self.fast_vol = 0.0
                if self.cfg["scope"] == "WholePrint" or in_wave:
                    self.acc += duration
                    if in_wave:
                        self.episode_wave_s += duration
                    return self._check()
                return None
            if q < self.cfg["q_fast"]:
                return None
            if self.cfg["reset"] == "OnVolume":
                if self.fast_vol + volume >= self.cfg["v_melt"]:
                    self._reset_episode()
                else:
                    self.fast_vol += volume
                return None
            self._reset_episode()
            return None

        if self.cfg["reset"] == "OnTravel":
            self._reset_episode()
            return None
        if self.acc > 0.0:
            self.acc += duration
            return self._check()
        return None

    def _reset_episode(self):
        self.acc = 0.0
        self.fast_vol = 0.0
        self.episode_wave_s = 0.0

    def _check(self):
        if self.cfg["t_max"] <= 0.0 or self.acc < self.cfg["t_max"]:
            return None
        self.count += 1
        event = {"id": self.count, "acc": self.acc, "wave_s": self.episode_wave_s}
        self._reset_episode()
        return event


def parse_config(lines):
    for line in lines:
        m = RE_CONFIG.match(line)
        if m:
            return {
                "scope": m.group(1),
                "q_slow": float(m.group(2)),
                "q_fast": float(m.group(3)),
                "t_max": float(m.group(4)),
                "reset": m.group(5),
                "v_melt": float(m.group(6)),
            }
    return None


def filament_diameter(lines):
    for line in lines:
        m = RE_FIL_DIAM.match(line)
        if m:
            return float(m.group(1))
    return None


def recompute(lines, cfg, diameter):
    """Walk the G-code with a naive time model, returning the events it would produce."""
    acc = NaiveAccumulator(cfg)
    area = math.pi * (0.5 * diameter) ** 2
    pos = {"X": 0.0, "Y": 0.0, "Z": 0.0, "E": 0.0}
    feedrate = 0.0  # mm/min
    absolute_xyz = True
    absolute_e = True
    in_wave = False
    events = []

    for raw in lines:
        line = raw.strip()
        if line.startswith(WAVE_START):
            in_wave = True
            continue
        if line.startswith(WAVE_END):
            in_wave = False
            continue
        if not line or line.startswith(";"):
            continue

        cmd = line.split()[0].upper()

        if cmd == "G90":
            absolute_xyz = True
            continue
        if cmd == "G91":
            absolute_xyz = False
            continue
        if cmd == "M82":
            absolute_e = True
            continue
        if cmd == "M83":
            absolute_e = False
            continue
        if cmd == "G92":
            for axis, value in parse_words(line).items():
                if axis in pos:
                    pos[axis] = value
            continue
        if cmd == "G4":
            # Mirrors the guard's additional_time handling: a dwell is non-extruding time.
            words = parse_words(line)
            dwell = words.get("S", 0.0) + words.get("P", 0.0) / 1000.0
            if dwell > 0.0:
                ev = acc.step(False, dwell, 0.0, in_wave)
                if ev:
                    events.append(ev)
            continue
        if cmd not in ("G0", "G1"):
            continue

        words = parse_words(line)
        if "F" in words:
            feedrate = words["F"]

        delta = {}
        for axis in ("X", "Y", "Z", "E"):
            if axis not in words:
                delta[axis] = 0.0
                continue
            absolute = absolute_xyz if axis != "E" else absolute_e
            delta[axis] = words[axis] - pos[axis] if absolute else words[axis]

        for axis in ("X", "Y", "Z", "E"):
            pos[axis] += delta[axis]

        dist_xyz = math.sqrt(delta["X"] ** 2 + delta["Y"] ** 2 + delta["Z"] ** 2)
        if dist_xyz == 0.0 and delta["E"] == 0.0:
            continue

        # Classification mirrors GCodeProcessor's move_type lambda: extrusion requires
        # positive E together with XY motion; everything else is non-extruding.
        is_extrusion = delta["E"] > 0.0 and (delta["X"] != 0.0 or delta["Y"] != 0.0)
        length = dist_xyz if dist_xyz > 0.0 else abs(delta["E"])
        if feedrate <= 0.0:
            continue
        duration = length / (feedrate / 60.0)
        volume = area * delta["E"] if is_extrusion else 0.0

        ev = acc.step(is_extrusion, duration, volume, in_wave)
        if ev:
            events.append(ev)

    return events


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("gcode", help="path to a sliced G-code file")
    parser.add_argument(
        "--tolerance",
        type=float,
        default=DEFAULT_TOLERANCE,
        help="relative tolerance for acc comparisons (default %(default)s)",
    )
    parser.add_argument("--verbose", action="store_true", help="print every marker")
    args = parser.parse_args()

    with open(args.gcode, encoding="utf-8", errors="replace") as f:
        lines = f.read().splitlines()

    cfg = parse_config(lines)
    if cfg is None:
        print_error("no ;THERMAL_DWELL_CONFIG line found -- was the guard enabled?")
        return 2
    print_info(
        f"config: scope={cfg['scope']} reset={cfg['reset']} q_slow={cfg['q_slow']} "
        f"q_fast={cfg['q_fast']} t_max={cfg['t_max']} v_melt={cfg['v_melt']}"
    )

    diameter = filament_diameter(lines)
    if diameter is None:
        print_error("no '; filament_diameter = ' in the config block; cannot convert dE to mm^3")
        return 2
    print_info(f"filament diameter: {diameter} mm")

    inline = [
        {
            "id": int(m.group(1)),
            "acc": float(m.group(2)),
            "layer": int(m.group(3)),
            "z": float(m.group(4)),
        }
        for m in (RE_FLUSH_POINT.match(l) for l in lines)
        if m
    ]
    summary_points = [
        {
            "id": int(m.group(1)),
            "layer": int(m.group(2)),
            "z": float(m.group(3)),
            "acc": float(m.group(4)),
        }
        for m in (RE_POINT.match(l) for l in lines)
        if m
    ]
    summary_count = None
    for line in lines:
        m = RE_SUMMARY.match(line)
        if m:
            summary_count = int(m.group(1))
            break

    if args.verbose:
        for point in inline:
            print_info(
                f"  FLUSH_POINT id={point['id']} acc={point['acc']}s "
                f"layer={point['layer']} z={point['z']}"
            )

    naive_events = recompute(lines, cfg, diameter)

    failures = []

    # --- Check 1: acc lands at the limit, and the naive model roughly agrees -------------
    for point in inline:
        if point["acc"] < cfg["t_max"]:
            failures.append(
                f"id={point['id']}: acc {point['acc']}s < max_time {cfg['t_max']}s "
                "(the accumulator fires at >= max_time)"
            )
        elif point["acc"] > cfg["t_max"] * (1.0 + args.tolerance):
            failures.append(
                f"id={point['id']}: acc {point['acc']}s overshoots max_time {cfg['t_max']}s "
                f"by more than {args.tolerance:.0%} (max one move's overshoot is expected)"
            )
    if not failures:
        print_ok(f"check 1a: all {len(inline)} marker(s) fire in [max_time, +{args.tolerance:.0%}]")

    if len(naive_events) != len(inline):
        print_info(
            f"check 1b: naive model found {len(naive_events)} event(s) vs {len(inline)} marker(s); "
            "counts may differ legitimately (accel-aware vs naive timing), comparing pairwise "
            "where both exist"
        )
    mismatches = []
    for point, ev in zip(inline, naive_events):
        if ev["acc"] <= 0.0:
            continue
        rel = abs(point["acc"] - ev["acc"]) / ev["acc"]
        if rel > args.tolerance:
            mismatches.append(
                f"id={point['id']}: slicer acc {point['acc']:.2f}s vs naive {ev['acc']:.2f}s "
                f"({rel:.0%} apart, tolerance {args.tolerance:.0%})"
            )
    if mismatches:
        failures.extend(mismatches)
    elif naive_events:
        print_ok(f"check 1b: slicer acc within {args.tolerance:.0%} of the naive recomputation")

    # --- Check 2: markers are caused by wave-region time ---------------------------------
    if cfg["scope"] == "WaveOverhangOnly":
        bad = [ev["id"] for ev in naive_events if ev["wave_s"] <= 0.0]
        if bad:
            failures.append(
                f"scope=WaveOverhangOnly but episode(s) {bad} accrued no time inside a wave region"
            )
        else:
            print_ok("check 2: every episode accrued time inside a wave region")
    else:
        print_info("check 2: skipped (scope != WaveOverhangOnly)")

    # --- Check 3: inline count == SUMMARY count ------------------------------------------
    if summary_count is None:
        failures.append("no ;THERMAL_DWELL_SUMMARY line found")
    elif summary_count != len(inline):
        failures.append(
            f"SUMMARY count={summary_count} but {len(inline)} inline FLUSH_POINT marker(s)"
        )
    else:
        print_ok(f"check 3: inline marker count matches SUMMARY count ({summary_count})")

    # --- Check 4: ids consecutive, inline agrees with SUMMARY ----------------------------
    expected_ids = list(range(1, len(inline) + 1))
    if [p["id"] for p in inline] != expected_ids:
        failures.append(f"inline ids are not 1..N consecutive: {[p['id'] for p in inline]}")
    elif [p["id"] for p in summary_points] != expected_ids:
        failures.append(f"SUMMARY ids are not 1..N consecutive: {[p['id'] for p in summary_points]}")
    else:
        disagreements = [
            f"id={a['id']}: inline (layer={a['layer']} z={a['z']} acc={a['acc']}) != "
            f"SUMMARY (layer={b['layer']} z={b['z']} acc={b['acc']})"
            for a, b in zip(inline, summary_points)
            if a["layer"] != b["layer"] or abs(a["z"] - b["z"]) > 1e-3 or abs(a["acc"] - b["acc"]) > 1e-2
        ]
        if disagreements:
            failures.extend(disagreements)
        else:
            print_ok("check 4: ids are 1..N and inline markers agree with the SUMMARY block")

    print()
    if failures:
        for failure in failures:
            print_error(failure)
        print(f"\n\033[91mFAILED\033[0m: {len(failures)} problem(s) found")
        return 1

    print(f"\033[92mPASS\033[0m: {len(inline)} flush point(s) verified")
    return 0


if __name__ == "__main__":
    sys.exit(main())
