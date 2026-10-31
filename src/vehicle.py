"""The car: mass, aerodynamics, tyres, engine.

Everything here is a point mass. No weight transfer, no suspension, no
individual wheels — the whole car is one lump with a friction limit and a
downforce curve. That is the standard first model in lap time simulation
because the two things that dominate a lap, grip and drag, are both
properties of the car as a whole.

Sign and unit conventions, held everywhere in this repo:

* SI throughout. Metres, seconds, kilograms, newtons, watts. Speeds are
  converted to km/h only for display.
* ``cla`` and ``cda`` are *areas* already multiplied by their coefficient
  (C_L·A and C_D·A, m²), because that product is what a wind tunnel reports
  and what the equations need. Downforce is positive downwards.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
from scipy.optimize import brentq

G = 9.80665          # m/s², standard gravity
RHO_AIR = 1.225      # kg/m³, ISA sea level at 15 °C


@dataclass(frozen=True)
class Vehicle:
    """A point-mass race car.

    `mu` is the tyre friction coefficient at the reference load. Real tyres
    lose grip as they are loaded harder, which is what `load_sensitivity`
    captures: mu_effective = mu * (Fz / Fz_ref) ** -load_sensitivity.

    That exponent is the reason a downforce-limited car does not corner as
    fast as the naive calculation says: the downforce it generates is also
    making its own tyres less efficient. Set it to zero and the cornering
    equation collapses to a closed form, which is how the solver is tested.
    """

    name: str
    mass: float                 # kg, car + driver + fuel
    power: float                # W, at the wheels after `efficiency`
    efficiency: float = 0.95    # drivetrain
    cda: float = 1.00           # m², drag area C_D·A
    cla: float = 3.50           # m², downforce area C_L·A
    mu: float = 1.70            # tyre friction at the reference load
    load_sensitivity: float = 0.15
    reference_load: float = 7_000.0   # N, where `mu` is quoted
    rolling_resistance: float = 0.015
    brake_power_limit: float = np.inf  # W, inf = tyre-limited braking
    v_max: float = 120.0        # m/s, hard ceiling (gearing)

    # -- aerodynamics ----------------------------------------------------

    def downforce(self, v: float | np.ndarray) -> float | np.ndarray:
        """Aerodynamic download, N. Quadratic in speed."""
        return 0.5 * RHO_AIR * self.cla * np.square(v)

    def drag(self, v: float | np.ndarray) -> float | np.ndarray:
        """Aerodynamic drag, N."""
        return 0.5 * RHO_AIR * self.cda * np.square(v)

    def normal_load(self, v: float | np.ndarray) -> float | np.ndarray:
        """Total vertical load on the tyres, N: weight plus downforce."""
        return self.mass * G + self.downforce(v)

    @property
    def aero_efficiency(self) -> float:
        """L/D. How much downforce each unit of drag buys."""
        return self.cla / self.cda

    # -- tyres -----------------------------------------------------------

    def friction(self, load: float | np.ndarray) -> float | np.ndarray:
        """Friction coefficient at a given vertical load.

        Falls with load when `load_sensitivity` > 0. This is not a detail:
        at 300 km/h a Formula car carries several times its own weight in
        downforce, so the tyre is operating far from where `mu` was quoted.
        """
        ratio = np.asarray(load, dtype=float) / self.reference_load
        return self.mu * np.power(np.maximum(ratio, 1e-9),
                                  -self.load_sensitivity)

    def grip_force(self, v: float | np.ndarray) -> float | np.ndarray:
        """Total horizontal force the tyres can carry at speed `v`, N."""
        load = self.normal_load(v)
        return self.friction(load) * load

    def grip_acceleration(self, v: float | np.ndarray) -> float | np.ndarray:
        """The radius of the friction ellipse at speed `v`, m/s²."""
        return self.grip_force(v) / self.mass

    # -- cornering -------------------------------------------------------

    def corner_speed(self, curvature: float) -> float:
        """Steady-state speed through a corner of the given curvature, m/s.

        Solves  m·v²·κ = mu(Fz)·Fz  with  Fz = m·g + ½ρ·CLA·v².

        Above a certain speed a winged car generates grip faster than it
        generates the demand for it, so for gentle curvature there is no
        lateral limit at all — the corner is flat, and the speed is set by
        power and gearing instead. That case returns `v_max`.
        """
        if curvature <= 0.0:
            return self.v_max

        def excess(v: float) -> float:
            # Positive while the tyres can still hold the corner.
            return self.grip_force(v) - self.mass * v * v * curvature

        if excess(self.v_max) >= 0.0:
            return self.v_max          # aero-limited: flat in top gear
        return brentq(excess, 1e-3, self.v_max, xtol=1e-9, rtol=1e-12)

    # -- longitudinal ----------------------------------------------------

    def drive_force(self, v: float) -> float:
        """Tractive force available at the contact patch, N.

        Power-limited, and at a standstill the quotient P/v diverges, so it
        is capped by what the tyres can transmit.
        """
        v = max(v, 1e-6)
        return min(self.power * self.efficiency / v, self.grip_force(v))

    def resistance(self, v: float) -> float:
        """Everything opposing motion, N."""
        return self.drag(v) + self.rolling_resistance * self.mass * G

    def _longitudinal_budget(self, v: float, curvature: float) -> float:
        """Tyre force left for accelerating or braking, N.

        The friction ellipse: a tyre already working laterally has less to
        give longitudinally. At the limit of cornering this is zero, which
        is why a driver cannot brake and turn at full commitment at once.
        """
        total = self.grip_force(v)
        lateral = self.mass * v * v * curvature
        if lateral >= total:
            return 0.0
        return float(np.sqrt(total * total - lateral * lateral))

    def acceleration(self, v: float, curvature: float = 0.0) -> float:
        """Longitudinal acceleration available at `v`, m/s²."""
        traction = min(self.drive_force(v), self._longitudinal_budget(v, curvature))
        return (traction - self.resistance(v)) / self.mass

    def braking(self, v: float, curvature: float = 0.0) -> float:
        """Braking deceleration available at `v`, m/s², positive.

        Drag and rolling resistance help, which is why a high-drag car stops
        in a shorter distance from high speed than its tyres alone allow.
        """
        tyre = self._longitudinal_budget(v, curvature)
        if np.isfinite(self.brake_power_limit):
            tyre = min(tyre, self.brake_power_limit / max(v, 1e-6))
        return (tyre + self.resistance(v)) / self.mass

    # -- variants --------------------------------------------------------

    def with_changes(self, **kwargs) -> "Vehicle":
        """A copy with fields replaced — for sweeps and sensitivities."""
        return replace(self, **kwargs)

    @classmethod
    def from_json(cls, path: str | Path) -> "Vehicle":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        data.pop("_comment", None)
        if "power_kw" in data:
            data["power"] = data.pop("power_kw") * 1000.0
        return cls(**data)
