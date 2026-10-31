"""What the simulation is actually for.

An absolute lap time from an uncalibrated model is worth little — it
inherits every error in the track file and every guess in the car file.
What survives those errors is the *derivative*: how lap time moves when one
thing about the car changes, with everything else held fixed. Those are the
numbers a race engineer uses, and they are quoted in the same units here:
seconds per 10 kg, seconds per 10 kW, seconds per 0.1 of grip.

The headline study is the downforce trade. Adding wing buys cornering speed
and costs straight-line speed, so there is an optimum, and it is different
at every circuit. That is the reason teams bring different aero packages to
Monza and Monaco, and it falls straight out of this model.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import solver
from track import Track
from vehicle import Vehicle


@dataclass(frozen=True)
class SweepPoint:
    value: float
    lap_time: float
    top_speed: float
    min_speed: float


@dataclass
class DownforceSweep:
    """Lap time against downforce level, at fixed aerodynamic efficiency."""

    track_name: str
    aero_efficiency: float
    points: tuple[SweepPoint, ...]

    @property
    def optimum(self) -> SweepPoint:
        return min(self.points, key=lambda p: p.lap_time)

    @property
    def baseline(self) -> SweepPoint:
        """The point nearest the car file's own downforce level."""
        return self.points[len(self.points) // 2]

    def gain_over(self, other: SweepPoint) -> float:
        return other.lap_time - self.optimum.lap_time

    def flatness(self, within: float = 0.1) -> tuple[float, float]:
        """Range of CLA whose lap time is within `within` seconds of best.

        How sharp the optimum is matters as much as where it sits: a broad
        optimum means the setup is forgiving, a narrow one means wing level
        is worth arguing about.
        """
        best = self.optimum.lap_time
        inside = [p.value for p in self.points if p.lap_time <= best + within]
        return (min(inside), max(inside)) if inside else (np.nan, np.nan)


def downforce_sweep(vehicle: Vehicle, track: Track,
                    levels: np.ndarray | None = None) -> DownforceSweep:
    """Vary downforce, holding L/D constant so drag follows it.

    Holding L/D fixed is the honest way to pose the question. Adding wing
    with no drag penalty would make more downforce free, and the optimum
    would be meaningless.
    """
    if levels is None:
        levels = np.linspace(1.0, 7.0, 25)
    ratio = vehicle.aero_efficiency

    points = []
    for cla in levels:
        variant = vehicle.with_changes(cla=float(cla), cda=float(cla) / ratio)
        result = solver.solve(variant, track)
        points.append(SweepPoint(float(cla), result.lap_time,
                                 result.top_speed, result.min_speed))
    return DownforceSweep(track.name, ratio, tuple(points))


@dataclass(frozen=True)
class Sensitivity:
    """Lap time derivative with respect to one vehicle parameter."""

    parameter: str
    unit: str
    step: float
    seconds_per_step: float
    baseline_lap: float

    @property
    def percent_per_step(self) -> float:
        return 100.0 * self.seconds_per_step / self.baseline_lap


def _derivative(vehicle: Vehicle, track: Track, field: str,
                half_step: float) -> float:
    """d(lap time)/d(parameter), by central difference."""
    base = getattr(vehicle, field)
    high = solver.solve(vehicle.with_changes(**{field: base + half_step}), track)
    low = solver.solve(vehicle.with_changes(**{field: base - half_step}), track)
    return (high.lap_time - low.lap_time) / (2.0 * half_step)


def sensitivities(vehicle: Vehicle, track: Track) -> list[Sensitivity]:
    """The engineer's shopping list, in seconds per unit of change.

    Central differences, because a one-sided difference on a quantity this
    non-linear leaks curvature into the slope.
    """
    baseline = solver.solve(vehicle, track).lap_time

    specs = [
        ("mass", "10 kg", 10.0, 5.0),
        ("power", "10 kW", 10_000.0, 5_000.0),
        ("mu", "0.1 grip", 0.1, 0.05),
        ("cda", "0.1 m2 drag", 0.1, 0.05),
        ("cla", "0.5 m2 downforce", 0.5, 0.25),
    ]

    out = []
    for field, unit, step, half in specs:
        slope = _derivative(vehicle, track, field, half)
        out.append(Sensitivity(field, unit, step, slope * step, baseline))
    return out


def cross_table(vehicle: Vehicle, tracks: dict[str, Track],
                optima: dict[str, float]) -> dict[str, dict[str, float]]:
    """Every circuit's optimum wing level, run at every circuit.

    The diagonal is each track with its own setup. Everything off it is the
    cost of turning up with the wrong aerodynamic package, which is the
    question the trade-off is really asking.
    """
    ratio = vehicle.aero_efficiency
    table: dict[str, dict[str, float]] = {}
    for setup_name, cla in optima.items():
        variant = vehicle.with_changes(cla=cla, cda=cla / ratio)
        table[setup_name] = {name: solver.solve(variant, track).lap_time
                             for name, track in tracks.items()}
    return table


@dataclass
class GripCalibration:
    """Can one global grip level reconcile the model with reality?

    If the residual error were in the *car* model, a single scalar would
    absorb it. If it is in the *track* geometry, it cannot — the circuits
    will pull in opposite directions and the best achievable error stays
    stubbornly above zero. This is the test that tells the two apart.
    """

    mus: np.ndarray
    rms_error: np.ndarray
    per_track: dict[str, np.ndarray]
    best_mu: float
    best_rms: float


def grip_calibration(vehicle: Vehicle, tracks: dict[str, Track],
                     references: dict[str, float],
                     mus: np.ndarray | None = None) -> GripCalibration:
    if mus is None:
        mus = np.linspace(1.3, 2.3, 21)

    per_track = {name: np.zeros(mus.size) for name in tracks}
    rms = np.zeros(mus.size)

    for i, mu in enumerate(mus):
        variant = vehicle.with_changes(mu=float(mu))
        errors = []
        for name, track in tracks.items():
            lap = solver.solve(variant, track).lap_time
            error = lap - references[name]
            per_track[name][i] = error
            errors.append(error)
        rms[i] = float(np.sqrt(np.mean(np.square(errors))))

    best = int(np.argmin(rms))
    return GripCalibration(mus, rms, per_track,
                           float(mus[best]), float(rms[best]))
