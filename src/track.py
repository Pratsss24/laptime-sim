"""The circuit, as curvature against distance.

A lap time simulation does not need a map. It needs to know how tightly the
car is being asked to turn at every point, so a track here is a list of
straights and constant-radius arcs, resolved onto an evenly spaced grid of
curvature κ(s) = 1/R.

That is a real simplification and it is the dominant source of error in the
absolute lap times this repo produces — real corners have entry and exit
spirals and varying radius, and a constant-radius arc is slower through the
middle and faster at the ends than the real thing. It is stated here rather
than buried, and the Scope section of the README says what it costs.

The centre line is also integrated into x–y so the track can be drawn and
so a closure error can be reported: a circuit described by hand rarely
joins up exactly, and how badly it misses is a useful check on the data.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

DIRECTIONS = {"left": 1.0, "right": -1.0}


@dataclass(frozen=True)
class Segment:
    """One straight or one constant-radius arc."""

    kind: str                    # "straight" | "corner"
    length: float                # m, arc length for a corner
    radius: float = np.inf       # m
    turn: float = 0.0            # signed radians, + is left
    name: str = ""

    @property
    def curvature(self) -> float:
        return 0.0 if not np.isfinite(self.radius) else 1.0 / self.radius


def _segment_from_dict(raw: dict) -> Segment:
    kind = raw["type"]
    name = raw.get("name", "")
    if kind == "straight":
        return Segment("straight", float(raw["length"]), name=name)
    if kind == "corner":
        radius = float(raw["radius"])
        angle = np.deg2rad(float(raw["angle"]))
        sign = DIRECTIONS[raw.get("direction", "left")]
        return Segment("corner", radius * angle, radius, sign * angle, name)
    raise ValueError(f"unknown segment type {kind!r}")


@dataclass
class Track:
    """A closed circuit sampled at an even spacing.

    `curvature` is unsigned — the solver only cares how hard the car turns,
    not which way. `heading` keeps the sign so the centre line can be drawn.
    """

    name: str
    s: np.ndarray                # m, distance along the lap
    curvature: np.ndarray        # 1/m, unsigned
    heading: np.ndarray          # rad, signed
    ds: float
    segments: tuple[Segment, ...]
    corner_names: tuple[tuple[float, str], ...] = ()

    @property
    def length(self) -> float:
        return float(self.s[-1] + self.ds)

    @property
    def n(self) -> int:
        return int(self.s.size)

    def centreline(self) -> tuple[np.ndarray, np.ndarray]:
        """Integrate heading into an x–y path, m."""
        x = np.concatenate(([0.0], np.cumsum(np.cos(self.heading) * self.ds)))
        y = np.concatenate(([0.0], np.cumsum(np.sin(self.heading) * self.ds)))
        return x, y

    def closure_error(self) -> float:
        """How far the described lap misses its own start line, m."""
        x, y = self.centreline()
        return float(np.hypot(x[-1] - x[0], y[-1] - y[0]))

    @property
    def corner_fraction(self) -> float:
        """Share of the lap spent turning. A track-character number."""
        return float(np.count_nonzero(self.curvature > 0) / self.n)

    def resampled(self, ds: float) -> "Track":
        """The same circuit at a different step — used for convergence."""
        return build(self.name, self.segments, ds)


def build(name: str, segments, ds: float = 1.0) -> Track:
    """Resolve a segment list onto an evenly spaced grid.

    Each segment is given a whole number of steps, so the resolved length
    differs from the nominal length by less than `ds` per segment. For a
    5 km circuit at ds = 1 m that is a few metres in total, which matters
    far less than the constant-radius assumption above it.
    """
    segments = tuple(segments)
    if not segments:
        raise ValueError("a track needs at least one segment")

    curvature_parts, heading_parts, names = [], [], []
    heading = 0.0
    travelled = 0.0

    for segment in segments:
        steps = max(1, int(round(segment.length / ds)))
        step_len = segment.length / steps
        if segment.kind == "corner":
            curvature_parts.append(np.full(steps, segment.curvature))
            turn_per_step = segment.turn / steps
            heading_parts.append(heading + turn_per_step * np.arange(steps))
            heading += segment.turn
            if segment.name:
                names.append((travelled, segment.name))
        else:
            curvature_parts.append(np.zeros(steps))
            heading_parts.append(np.full(steps, heading))
        travelled += steps * step_len

    curvature = np.concatenate(curvature_parts)
    headings = np.concatenate(heading_parts)
    s = np.arange(curvature.size) * (travelled / curvature.size)

    return Track(name=name, s=s, curvature=curvature, heading=headings,
                 ds=travelled / curvature.size, segments=segments,
                 corner_names=tuple(names))


def from_json(path: str | Path, ds: float = 1.0) -> Track:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    segments = [_segment_from_dict(raw) for raw in data["segments"]]
    return build(data["name"], segments, ds)


# -- shapes used by the tests -------------------------------------------

def skidpad(radius: float, ds: float = 0.5) -> Track:
    """A single constant-radius circle. The closed-form test case."""
    return build(f"skidpad r={radius:g}",
                 [Segment("corner", 2 * np.pi * radius, radius, 2 * np.pi)],
                 ds)


def straight(length: float, ds: float = 1.0) -> Track:
    """A straight treated as a closed loop — for acceleration tests."""
    return build(f"straight {length:g}", [Segment("straight", length)], ds)
