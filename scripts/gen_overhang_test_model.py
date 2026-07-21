#!/usr/bin/env python3
"""Generate a deterministic cantilever overhang test model for the thermal dwell guard.

The model is an L-shaped prism: a vertical stem with a horizontal arm cantilevered off
it. The arm's underside is a fully unsupported 90-degree overhang, which is exactly the
geometry wave overhangs replace bridges on -- so slicing this with wave_overhangs=true
produces a long, low-flow wave region, which is what the thermal dwell guard is built to
detect.

    z
    ^
    |  +-------------------+  z = stem_h + arm_h
    |  |                   |
    |  +--------+----------+  z = stem_h   <-- arm underside: the overhang
    |  |        |
    |  |        |          the span from x=stem_w to x=stem_w+arm_len is unsupported
    |  |        |
    |  +--------+             z = 0
    |  0     stem_w      stem_w+arm_len
    +---------------------------------> x

Sized so one overhang layer sustains well over 60 s of sub-threshold flow, which forces
several flush points at thermal_dwell_max_time=30 (STAGE1 §7.3 needs at least two).

Usage:
    python3 gen_overhang_test_model.py --output overhang_test.stl
"""

import argparse
import struct
import sys

# Cantilever geometry, mm. The arm underside sits at z = STEM_H and spans ARM_LEN in x.
STEM_W = 10.0
ARM_LEN = 20.0
STEM_H = 10.0
ARM_H = 6.0
DEPTH = 30.0


def print_error(msg):
    print(f"\033[91m[ERROR]\033[0m {msg}")  # Red


def print_ok(msg):
    print(f"\033[92m[OK]\033[0m {msg}")  # Green


def l_outline():
    """The L cross-section in the XZ plane, counter-clockwise.

    The edge from (STEM_W, STEM_H) to (STEM_W + ARM_LEN, STEM_H) is the overhang.

    Carries two vertices the bare L shape does not need -- (STEM_W, top) on the top edge
    and (0, STEM_H) on the left edge -- so that every outline edge coincides exactly with
    one cap_rects() boundary edge. Without them the side walls span a full edge while the
    caps split it, leaving T-junctions and an unclosed mesh.
    """
    top = STEM_H + ARM_H
    return [
        (0.0, 0.0),
        (STEM_W, 0.0),
        (STEM_W, STEM_H),
        (STEM_W + ARM_LEN, STEM_H),
        (STEM_W + ARM_LEN, top),
        (STEM_W, top),
        (0.0, top),
        (0.0, STEM_H),
    ]


def cap_rects():
    """Tile the L cross-section with rectangles, counter-clockwise in XZ.

    Split into three rather than two so no rectangle corner lands mid-edge of another:
    a T-junction there would make the mesh non-manifold and could skew the slice.
    """
    top = STEM_H + ARM_H
    return [
        [(0.0, 0.0), (STEM_W, 0.0), (STEM_W, STEM_H), (0.0, STEM_H)],
        [(0.0, STEM_H), (STEM_W, STEM_H), (STEM_W, top), (0.0, top)],
        [(STEM_W, STEM_H), (STEM_W + ARM_LEN, STEM_H), (STEM_W + ARM_LEN, top), (STEM_W, top)],
    ]


def build_triangles():
    """Return the closed prism as a list of (v0, v1, v2), each vertex an (x, y, z) tuple."""
    tris = []
    outline = l_outline()

    # Side walls: one quad per outline edge, swept along y.
    # The outline is counter-clockwise in XZ, whose right-hand normal is -Y, so sweeping it
    # along +Y with the naive winding would face every triangle inward. Wind them reversed.
    for i in range(len(outline)):
        x0, z0 = outline[i]
        x1, z1 = outline[(i + 1) % len(outline)]
        a = (x0, 0.0, z0)
        b = (x1, 0.0, z1)
        c = (x1, DEPTH, z1)
        d = (x0, DEPTH, z0)
        tris.append((a, c, b))
        tris.append((a, d, c))

    # End caps, wound to face away from the solid: -Y at y=0, +Y at y=DEPTH.
    for rect in cap_rects():
        near = [(x, 0.0, z) for (x, z) in rect]
        far = [(x, DEPTH, z) for (x, z) in rect]
        tris.append((near[0], near[1], near[2]))
        tris.append((near[0], near[2], near[3]))
        tris.append((far[0], far[2], far[1]))
        tris.append((far[0], far[3], far[2]))

    return tris


def normal(tri):
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = tri
    ux, uy, uz = bx - ax, by - ay, bz - az
    vx, vy, vz = cx - ax, cy - ay, cz - az
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    length = (nx * nx + ny * ny + nz * nz) ** 0.5
    if length == 0.0:
        return (0.0, 0.0, 0.0)
    return (nx / length, ny / length, nz / length)


def signed_volume(tris):
    """Volume via the divergence theorem. Positive means outward-facing winding."""
    total = 0.0
    for (ax, ay, az), (bx, by, bz), (cx, cy, cz) in tris:
        total += (
            ax * (by * cz - bz * cy)
            - ay * (bx * cz - bz * cx)
            + az * (bx * cy - by * cx)
        ) / 6.0
    return total


def validate(tris):
    """Assert the mesh is closed and outward-wound before we hand it to a slicer.

    A silently broken model would make a failed acceptance run look like a guard bug.
    """
    edges = {}
    for tri in tris:
        for i in range(3):
            a, b = tri[i], tri[(i + 1) % 3]
            edges[(a, b)] = edges.get((a, b), 0) + 1

    unmatched = [e for e in edges if edges.get((e[1], e[0]), 0) != edges[e]]
    if unmatched:
        print_error(f"mesh is not closed: {len(unmatched)} unmatched half-edge(s)")
        return False

    dupes = [e for e, n in edges.items() if n != 1]
    if dupes:
        print_error(f"mesh is non-manifold: {len(dupes)} half-edge(s) used more than once")
        return False

    vol = signed_volume(tris)
    if vol <= 0.0:
        print_error(f"winding is inverted (signed volume {vol:.3f})")
        return False

    expected = (STEM_W * (STEM_H + ARM_H) + ARM_LEN * ARM_H) * DEPTH
    if abs(vol - expected) > 1e-6:
        print_error(f"volume {vol:.3f} != expected {expected:.3f}")
        return False

    print_ok(f"mesh closed, manifold, outward-wound; volume {vol:.1f} mm^3")
    return True


def write_binary_stl(path, tris):
    with open(path, "wb") as f:
        f.write(b"thermal dwell guard cantilever overhang test model".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tris)))
        for tri in tris:
            f.write(struct.pack("<3f", *normal(tri)))
            for vertex in tri:
                f.write(struct.pack("<3f", *vertex))
            f.write(struct.pack("<H", 0))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--output", default="overhang_test.stl", help="output STL path")
    args = parser.parse_args()

    tris = build_triangles()
    if not validate(tris):
        return 1

    write_binary_stl(args.output, tris)
    print_ok(f"wrote {args.output}: {len(tris)} triangles")
    print(
        f"     overhang: {ARM_LEN:.0f} mm span x {DEPTH:.0f} mm deep, "
        f"underside at z={STEM_H:.0f} mm"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
