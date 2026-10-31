"""Quasi-steady-state lap time solver.

The method is the standard one in motorsport, and it rests on one
assumption: at every point of the lap the car is on the limit of something.
Either the tyres are saturated laterally (cornering), or longitudinally
(braking, or accelerating out of a slow corner), or the engine has run out
of power, or the gearing has run out of revs. The driver is never in the
middle of the envelope.

That makes the solution a three-step construction rather than a search:

1. **Limit speed.** At every point, the fastest the car could go through
   that curvature in a steady state. On a straight this is the top speed.
2. **Braking pass**, walked backwards. No point may be faster than the
   speed from which the car could still slow to the next limit.
3. **Acceleration pass**, walked forwards. No point may be faster than the
   speed the car could have reached from the one before.

Take the lowest of the three everywhere and the speed trace is determined —
no optimisation, no driver model. On a closed circuit the passes wrap
around and are repeated until the lap stops changing, because the exit of
the last corner sets the speed onto the start line.

What this method does **not** model: transient weight transfer, tyre
relaxation, the driver's choice of line, differential behaviour, and any
trade of lap time now for lap time later. It is a lower bound on the car,
not a prediction of the driver.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from track import Track
from vehicle import Vehicle

# Which constraint is binding at each point of the lap.
CORNERING, BRAKING, ACCELERATING, STRAIGHTLINE = 0, 1, 2, 3
REGIME_NAMES = {CORNERING: "cornering", BRAKING: "braking",
                ACCELERATING: "accelerating", STRAIGHTLINE: "flat out"}


@dataclass
class LapResult:
    """One solved lap."""

    track_name: str
    vehicle_name: str
    s: np.ndarray                # m
    speed: np.ndarray            # m/s
    limit_speed: np.ndarray      # m/s, the cornering limit alone
    time: np.ndarray             # s, cumulative at each point
    long_accel: np.ndarray       # m/s², + accelerating
    lat_accel: np.ndarray        # m/s²
    regime: np.ndarray           # int codes, see above
    power_limited: np.ndarray    # bool, engine is the binding constraint
    ds: float
    iterations: int

    @property
    def lap_time(self) -> float:
        return float(self.time[-1])

    @property
    def top_speed(self) -> float:
        return float(self.speed.max())

    @property
    def min_speed(self) -> float:
        return float(self.speed.min())

    @property
    def mean_speed(self) -> float:
        return float((self.s[-1] + self.ds) / self.lap_time)

    @property
    def peak_lateral_g(self) -> float:
        return float(self.lat_accel.max() / 9.80665)

    def time_fraction(self, regime: int) -> float:
        """Share of lap *time* — not distance — spent in one regime.

        Distance flatters the straights; time is what the stopwatch sees.
        """
        dt = np.diff(self.time, prepend=0.0)
        return float(dt[self.regime == regime].sum() / self.lap_time)

    def regime_breakdown(self) -> dict[str, float]:
        return {name: self.time_fraction(code)
                for code, name in REGIME_NAMES.items()}

    @property
    def full_throttle_fraction(self) -> float:
        """Share of lap time with the engine, not the tyre, as the limit.

        This is the number teams quote for a circuit's character, and it is
        not the same as "on a straight": Curva Grande and the Parabolica
        are taken at full throttle while cornering hard. Nor is it the same
        as "accelerating" — a car held at a steady speed through a long
        corner is on partial throttle, balancing drag.
        """
        dt = np.diff(self.time, prepend=0.0)
        return float(dt[self.power_limited].sum() / self.lap_time)


def limit_speeds(vehicle: Vehicle, track: Track) -> np.ndarray:
    """Steady-state cornering speed at every point.

    Distinct curvatures are solved once and broadcast — a track built from
    constant-radius arcs has a handful of distinct values among thousands
    of points, and the root find is the expensive part.
    """
    unique, inverse = np.unique(track.curvature, return_inverse=True)
    solved = np.array([vehicle.corner_speed(float(k)) for k in unique])
    return solved[inverse]


def solve(vehicle: Vehicle, track: Track, *, closed: bool = True,
          entry_speed: float | None = None, max_iterations: int = 20,
          tolerance: float = 1e-6) -> LapResult:
    """Solve a lap. `entry_speed` applies only to an open section."""
    n = track.n
    ds = track.ds
    limit = limit_speeds(vehicle, track)
    speed = limit.copy()

    if not closed:
        speed[0] = min(speed[0], float(entry_speed if entry_speed is not None
                                       else limit[0]))

    iterations = 0
    for iterations in range(1, max_iterations + 1):
        previous = speed.copy()

        # Braking, walked backwards: can the car still make the next limit?
        last = n - 1 if closed else n - 2
        for i in range(last, -1, -1):
            j = (i + 1) % n
            decel = vehicle.braking(speed[j], track.curvature[j])
            reachable = np.sqrt(speed[j] ** 2 + 2.0 * decel * ds)
            if reachable < speed[i]:
                speed[i] = reachable

        # Acceleration, walked forwards: could the car have got here?
        stop = n if closed else n - 1
        for i in range(stop):
            j = (i + 1) % n
            accel = vehicle.acceleration(speed[i], track.curvature[i])
            reachable = np.sqrt(max(speed[i] ** 2 + 2.0 * accel * ds, 0.0))
            if reachable < speed[j]:
                speed[j] = reachable

        if np.max(np.abs(speed - previous)) < tolerance:
            break
        if not closed:
            break   # an open section needs exactly one pass of each

    return _assemble(vehicle, track, speed, limit, iterations)


def _assemble(vehicle: Vehicle, track: Track, speed: np.ndarray,
              limit: np.ndarray, iterations: int) -> LapResult:
    """Derive time, accelerations and the binding constraint."""
    ds = track.ds
    n = track.n
    nxt = np.roll(speed, -1)

    # dt over a step of constant acceleration is 2·ds/(v1+v2), which is
    # exact for the trapezoid and well behaved as v approaches zero.
    dt = 2.0 * ds / (speed + nxt)
    time = np.cumsum(dt)

    long_accel = (nxt ** 2 - speed ** 2) / (2.0 * ds)
    lat_accel = np.square(speed) * track.curvature

    # Classify by which constraint is actually binding, measured against the
    # friction circle — not by comparing speed to the steady-state limit.
    # A long corner taken at constant speed sits a hair under that limit
    # because the solver is still shedding the last of the entry speed, and
    # a tolerance-based test mislabels the whole corner as a straight.
    lateral_usage = np.divide(lat_accel, vehicle.grip_acceleration(speed),
                              out=np.zeros(n), where=speed > 1e-6)

    regime = np.full(n, STRAIGHTLINE, dtype=int)
    regime[long_accel > 0.1] = ACCELERATING
    regime[long_accel < -0.1] = BRAKING
    regime[lateral_usage > 0.98] = CORNERING

    # Full throttle means the engine is the limit, not the tyres: the force
    # the powertrain can deliver is less than the grip left over after
    # cornering, and the car is not being slowed.
    drive = np.array([vehicle.drive_force(float(v)) for v in speed])
    budget = np.array([vehicle._longitudinal_budget(float(v), float(k))
                       for v, k in zip(speed, track.curvature)])
    power_limited = (drive <= budget + 1e-9) & (long_accel > -0.1)

    return LapResult(track_name=track.name, vehicle_name=vehicle.name,
                     s=track.s, speed=speed, limit_speed=limit, time=time,
                     long_accel=long_accel, lat_accel=lat_accel,
                     regime=regime, power_limited=power_limited,
                     ds=ds, iterations=iterations)


def converged_lap_time(vehicle: Vehicle, track: Track,
                       steps: tuple[float, ...] = (8.0, 4.0, 2.0, 1.0, 0.5),
                       ) -> list[tuple[float, float]]:
    """Lap time against step size — the discretisation convergence check.

    A spatial solver that has not been refined until the answer stops
    moving is reporting the mesh, not the car.
    """
    out = []
    for ds in steps:
        refined = track.resampled(ds)
        out.append((ds, solve(vehicle, refined).lap_time))
    return out
