"""Make the hand-estimated circuits self-consistent, reproducibly.

A circuit sketched from a published layout gets two things wrong that are
worth fixing before it is simulated:

* the signed corner angles do not sum to one full turn, so the lap is not
  geometrically a lap;
* the total length does not match the official figure, and lap time scales
  almost linearly with length, so this error goes straight into the answer.

This script fixes both: angle overrides are applied by name, then every
straight is scaled by one common factor until the lap length matches the
published distance. Corner radii are never touched, because they set the
cornering speeds that the whole simulation turns on.

    python tools/fit_tracks.py          # report only
    python tools/fit_tracks.py --write  # apply

What it cannot fix is that the corners are circular arcs in the first
place. See Scope in the README.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

import track as track_module  # noqa: E402

OFFICIAL = {"monza": 5793.0, "monaco": 3337.0, "silverstone": 5891.0}

# Corner angles adjusted so each lap closes to exactly one turn. Chosen to
# spread the correction over the corners least well defined by the layout.
ANGLE_OVERRIDES = {
    "monaco": {"Mirabeau": 70},
    "silverstone": {"Stowe": 95, "Club": 105},
}


def signed_turn(path: Path) -> float:
    t = track_module.from_json(path, ds=1.0)
    return float(np.rad2deg(t.heading[-1]))


def fit(name: str, write: bool) -> None:
    path = ROOT / "data" / "tracks" / f"{name}.json"
    data = json.loads(path.read_text(encoding="utf-8"))

    for segment in data["segments"]:
        override = ANGLE_OVERRIDES.get(name, {}).get(segment.get("name", ""))
        if override is not None:
            segment["angle"] = override

    corner_length = sum(
        seg["radius"] * np.deg2rad(seg["angle"])
        for seg in data["segments"] if seg["type"] == "corner")
    straight_length = sum(
        seg["length"] for seg in data["segments"] if seg["type"] == "straight")

    target = OFFICIAL[name]
    scale = (target - corner_length) / straight_length
    if scale <= 0:
        raise SystemExit(f"{name}: corners alone exceed the official length")

    for segment in data["segments"]:
        if segment["type"] == "straight":
            segment["length"] = round(segment["length"] * scale, 1)

    data["official_length_m"] = target
    data["straight_scale_applied"] = round(scale, 4)

    if write:
        path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")

    resolved = track_module.build(
        data["name"],
        [track_module._segment_from_dict(s) for s in data["segments"]], 1.0)
    print(f"{name:12s} straights x{scale:.4f}  "
          f"length {resolved.length:7.1f} m (official {target:.0f})  "
          f"net turn {np.rad2deg(resolved.heading[-1]):7.1f} deg  "
          f"closure {resolved.closure_error():6.0f} m  "
          f"corner fraction {resolved.corner_fraction:5.1%}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    for name in OFFICIAL:
        fit(name, args.write)
    if not args.write:
        print("\n(report only — pass --write to apply)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
