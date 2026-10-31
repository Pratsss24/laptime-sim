"""Checks against closed-form answers, not against stored output.

Every assertion here is either a value derived by hand from the physics, a
monotonicity that must hold for the model to mean anything, or a numerical
property the solver has to have. Nothing is compared to a previous run of
this code, because a golden file only proves that nothing changed.

    python tests/test_laptime.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

import solver  # noqa: E402
import sweeps  # noqa: E402
import track as track_module  # noqa: E402
import validate  # noqa: E402
import vehicle as vehicle_module  # noqa: E402

G = vehicle_module.G
RHO = vehicle_module.RHO_AIR


def plain_car(**overrides) -> vehicle_module.Vehicle:
    """A car with no aerodynamics and a load-independent tyre.

    Stripping those two out is what makes the textbook formulas exact, so
    this is the vehicle most of the anchors below are written against.
    """
    base = dict(name="test", mass=800.0, power=750_000.0, efficiency=1.0,
                cda=0.0, cla=0.0, mu=1.5, load_sensitivity=0.0,
                rolling_resistance=0.0, v_max=120.0)
    base.update(overrides)
    return vehicle_module.Vehicle(**base)


# -- the car -------------------------------------------------------------

def test_skidpad_speed_is_the_textbook_value():
    """v = sqrt(mu g R) — the first equation in any vehicle dynamics course."""
    car = plain_car()
    for radius in (20.0, 50.0, 150.0):
        expected = math.sqrt(car.mu * G * radius)
        assert abs(car.corner_speed(1.0 / radius) - expected) < 1e-6, radius


def test_downforce_corner_speed_matches_its_closed_form():
    """With a load-independent tyre the aero case is still solvable by hand.

        m v² κ = mu (m g + ½ρ·CLA·v²)   ->   v² = mu·m·g / (mκ − ½·mu·ρ·CLA)
    """
    car = plain_car(cla=3.0)
    for radius in (40.0, 80.0, 200.0):
        kappa = 1.0 / radius
        denominator = car.mass * kappa - 0.5 * car.mu * RHO * car.cla
        expected = math.sqrt(car.mu * car.mass * G / denominator)
        assert abs(car.corner_speed(kappa) - expected) < 1e-6, radius


def test_a_gentle_enough_corner_is_flat_for_a_winged_car():
    """Below a critical curvature the car makes grip faster than demand.

    The closed form's denominator goes to zero there: the corner stops
    having a lateral speed limit at all, and the answer is the top speed.
    """
    car = plain_car(cla=3.0)
    critical = 0.5 * car.mu * RHO * car.cla / car.mass
    assert car.corner_speed(critical * 0.9) == car.v_max
    assert car.corner_speed(critical * 1.8) < car.v_max


def test_downforce_only_ever_helps_cornering():
    car = plain_car()
    speeds = [car.with_changes(cla=cla).corner_speed(1 / 100.0)
              for cla in (0.0, 1.0, 2.0, 3.0, 4.0)]
    assert all(b > a for a, b in zip(speeds, speeds[1:])), speeds


def test_tyre_load_sensitivity_reduces_grip_at_high_load():
    """Doubling the load must not double the grip, or downforce is free."""
    car = plain_car(load_sensitivity=0.25, reference_load=7000.0)
    assert abs(car.friction(7000.0) - car.mu) < 1e-12
    doubled = car.friction(14_000.0) * 14_000.0
    single = car.friction(7_000.0) * 7_000.0
    assert 1.0 < doubled / single < 2.0, doubled / single


def test_the_friction_ellipse_leaves_nothing_at_the_lateral_limit():
    """A car already using all its grip to turn cannot also brake.

    "Nothing" has to be expressed as a fraction, not an absolute. The
    budget is sqrt(total² − lateral²), and at the limit those two are
    equal, so the square root has infinite slope there: the root find
    lands within 6e-9 N of the limit and that still leaves ~1e-2 N of
    apparent longitudinal force, six orders of magnitude of amplification.
    Physically it is nothing — a ten-millionth of the friction circle —
    but it is not zero, and a test written as `< 1e-6 m/s²` would be
    testing brentq's tolerance rather than the physics.
    """
    car = plain_car()
    radius = 60.0
    kappa = 1.0 / radius
    v = car.corner_speed(kappa)
    circle = car.mu * G

    assert car.braking(v, kappa) / circle < 1e-5
    assert car.acceleration(v, kappa) / circle < 1e-5
    # And with the steering straight, the whole circle is available.
    assert abs(car.braking(v, 0.0) - circle) < 1e-6


def test_braking_distance_is_v_squared_over_two_mu_g():
    car = plain_car()
    v = 80.0
    expected = v * v / (2.0 * car.mu * G)
    distance, speed = 0.0, v
    step = 0.001
    while speed > 1e-6:
        speed -= car.braking(speed, 0.0) * step
        distance += max(speed, 0.0) * step
    assert abs(distance - expected) / expected < 2e-3, (distance, expected)


def test_top_speed_is_where_power_equals_drag():
    """P·eta = ½rho·CdA·v³ + Crr·m·g·v, solved by the solver, not by hand."""
    car = plain_car(cda=1.0, rolling_resistance=0.01, v_max=150.0)
    straight = track_module.straight(12_000.0, ds=2.0)
    result = solver.solve(car, straight)

    def balance(v):
        return (car.power * car.efficiency
                - 0.5 * RHO * car.cda * v ** 3
                - car.rolling_resistance * car.mass * G * v)

    from scipy.optimize import brentq
    expected = brentq(balance, 1.0, 149.0)
    assert abs(result.top_speed - expected) < 0.05, (result.top_speed, expected)


def test_drag_shortens_the_braking_distance():
    """Air resistance helps the brakes, and the model must show it."""
    clean = plain_car()
    draggy = plain_car(cda=1.5)
    assert draggy.braking(90.0, 0.0) > clean.braking(90.0, 0.0)


# -- the track -----------------------------------------------------------

def test_skidpad_geometry_is_a_circle():
    radius = 50.0
    circle = track_module.skidpad(radius, ds=0.25)
    assert abs(circle.length - 2 * math.pi * radius) < 0.3
    assert abs(np.rad2deg(circle.heading[-1]) - 360.0) < 1.0
    assert circle.closure_error() < 0.5
    assert circle.corner_fraction == 1.0


def test_a_straight_has_no_curvature_and_does_not_turn():
    line = track_module.straight(1000.0, ds=1.0)
    assert np.all(line.curvature == 0.0)
    assert line.corner_fraction == 0.0
    assert abs(line.length - 1000.0) < 1e-6


def test_the_real_tracks_close_to_one_full_turn():
    """A lap that does not sum to 360 degrees is not a lap."""
    for name in ("monza", "silverstone", "monaco"):
        circuit = track_module.from_json(
            ROOT / "data" / "tracks" / f"{name}.json", ds=2.0)
        turn = abs(np.rad2deg(circuit.heading[-1]))
        assert abs(turn - 360.0) < 1.0, (name, turn)


def test_the_real_tracks_match_their_official_length():
    official = {"monza": 5793.0, "silverstone": 5891.0, "monaco": 3337.0}
    for name, length in official.items():
        circuit = track_module.from_json(
            ROOT / "data" / "tracks" / f"{name}.json", ds=1.0)
        assert abs(circuit.length - length) < 1.0, (name, circuit.length)


# -- the solver ----------------------------------------------------------

def test_a_skidpad_lap_is_circumference_over_steady_speed():
    """The whole solver, against one division."""
    car = plain_car()
    radius = 60.0
    circle = track_module.skidpad(radius, ds=0.25)
    expected = 2 * math.pi * radius / math.sqrt(car.mu * G * radius)
    result = solver.solve(car, circle)
    assert abs(result.lap_time - expected) / expected < 1e-3, (
        result.lap_time, expected)
    # Steady state: the car is neither gaining nor losing speed anywhere.
    assert np.max(np.abs(result.long_accel)) < 1e-6


def test_a_lap_is_never_faster_than_the_cornering_limit():
    car = vehicle_module.Vehicle.from_json(ROOT / "data" / "car.json")
    circuit = track_module.from_json(ROOT / "data" / "tracks" / "monza.json",
                                     ds=2.0)
    result = solver.solve(car, circuit)
    assert np.all(result.speed <= result.limit_speed + 1e-6)


def test_the_solution_converges_as_the_step_shrinks():
    """Refining the step must move the answer steadily towards its limit.

    Stated as distance from the finest solution rather than as the gap
    between successive refinements: the gaps are not monotone here,
    because a coarse grid places each braking point to within half a step
    and that error is not a smooth function of the step. The approach to
    the limit is monotone, and that is the property that matters.
    """
    car = vehicle_module.Vehicle.from_json(ROOT / "data" / "car.json")
    circuit = track_module.from_json(ROOT / "data" / "tracks" / "monza.json",
                                     ds=8.0)
    laps = dict(solver.converged_lap_time(
        car, circuit, steps=(8.0, 4.0, 2.0, 1.0, 0.5)))

    # The grid never changes the track itself, only how finely it is read.
    lengths = {circuit.resampled(ds).length for ds in laps}
    assert max(lengths) - min(lengths) < 1e-6, lengths

    finest = laps.pop(0.5)
    errors = [abs(laps[ds] - finest) for ds in (8.0, 4.0, 2.0, 1.0)]
    assert all(b < a for a, b in zip(errors, errors[1:])), errors
    assert errors[-1] < 0.1, errors


def test_a_closed_lap_needs_more_than_one_pass():
    """The exit of the last corner sets the speed onto the start line.

    A solver that stops after a single forward and backward sweep gets a
    closed circuit wrong, so this asserts the wrap-around actually runs.
    """
    car = vehicle_module.Vehicle.from_json(ROOT / "data" / "car.json")
    circuit = track_module.from_json(ROOT / "data" / "tracks" / "monaco.json",
                                     ds=2.0)
    assert solver.solve(car, circuit).iterations >= 2


def test_a_pure_straight_loop_ends_up_flat_out_everywhere():
    car = plain_car(cda=1.0)
    line = track_module.straight(15_000.0, ds=5.0)
    result = solver.solve(car, line)
    assert result.time_fraction(solver.STRAIGHTLINE) > 0.98
    assert result.time_fraction(solver.CORNERING) == 0.0


def test_lap_time_responds_the_right_way_to_every_parameter():
    """Signs first. A model that gets these backwards is worthless."""
    car = vehicle_module.Vehicle.from_json(ROOT / "data" / "car.json")
    circuit = track_module.from_json(ROOT / "data" / "tracks" / "monza.json",
                                     ds=4.0)
    base = solver.solve(car, circuit).lap_time

    heavier = solver.solve(car.with_changes(mass=car.mass + 50), circuit)
    stronger = solver.solve(car.with_changes(power=car.power * 1.1), circuit)
    grippier = solver.solve(car.with_changes(mu=car.mu + 0.1), circuit)
    draggier = solver.solve(car.with_changes(cda=car.cda + 0.2), circuit)

    assert heavier.lap_time > base
    assert stronger.lap_time < base
    assert grippier.lap_time < base
    assert draggier.lap_time > base


# -- the studies ---------------------------------------------------------

def test_every_circuit_is_fastest_with_its_own_wing_level():
    """The diagonal of the cross table has to be the minimum of its column."""
    car = vehicle_module.Vehicle.from_json(ROOT / "data" / "car.json")
    tracks = {name: track_module.from_json(
        ROOT / "data" / "tracks" / f"{name}.json", ds=6.0)
        for name in ("monza", "monaco")}
    optima = {name: sweeps.downforce_sweep(
        car, circuit, np.linspace(2.0, 9.0, 8)).optimum.value
        for name, circuit in tracks.items()}
    table = sweeps.cross_table(car, tracks, optima)
    for circuit in tracks:
        own = table[circuit][circuit]
        assert all(own <= table[setup][circuit] + 1e-9 for setup in table), circuit


def test_monaco_wants_more_wing_than_monza():
    """The result the whole study exists to produce.

    If this ever fails, either the physics or the track files are wrong —
    no plausible model prefers more wing at Monza than at Monaco.
    """
    car = vehicle_module.Vehicle.from_json(ROOT / "data" / "car.json")
    levels = np.linspace(2.0, 10.0, 9)
    best = {}
    for name in ("monza", "monaco"):
        circuit = track_module.from_json(
            ROOT / "data" / "tracks" / f"{name}.json", ds=6.0)
        best[name] = sweeps.downforce_sweep(car, circuit, levels).optimum.value
    assert best["monaco"] > best["monza"], best


def test_sensitivities_carry_the_expected_signs():
    car = vehicle_module.Vehicle.from_json(ROOT / "data" / "car.json")
    circuit = track_module.from_json(ROOT / "data" / "tracks" / "monza.json",
                                     ds=6.0)
    rows = {s.parameter: s.seconds_per_step for s in sweeps.sensitivities(car, circuit)}
    assert rows["mass"] > 0
    assert rows["power"] < 0
    assert rows["mu"] < 0
    assert rows["cda"] > 0


def test_no_single_grip_level_fits_every_circuit():
    """The point of the calibration study, as an assertion.

    If one scalar could absorb the error, the residual would collapse to
    nearly zero and the model's problem would be the car. It does not, so
    the problem is the track geometry.
    """
    car = vehicle_module.Vehicle.from_json(ROOT / "data" / "car.json")
    tracks = {name: track_module.from_json(
        ROOT / "data" / "tracks" / f"{name}.json", ds=6.0)
        for name in ("monza", "silverstone", "monaco")}
    references = validate.load_references()
    result = sweeps.grip_calibration(
        car, tracks, {k: v.lap_time for k, v in references.items()},
        mus=np.linspace(1.3, 2.3, 11))
    assert result.best_rms > 2.0, result.best_rms
    index = int(np.argmin(result.rms_error))
    signs = {np.sign(errors[index]) for errors in result.per_track.values()}
    assert signs == {-1.0, 1.0}, "residuals should straddle zero"


# -- reference data ------------------------------------------------------

def test_lap_times_parse_from_the_api_format():
    assert abs(validate._parse_clock("1:23.504") - 83.504) < 1e-9
    assert abs(validate._parse_clock("59.123") - 59.123) < 1e-9


def test_the_cached_reference_laps_are_plausible():
    """Guards against a schema change silently poisoning the comparison."""
    references = validate.load_references()
    assert set(references) == {"monza", "silverstone", "monaco"}
    for name, lap in references.items():
        assert 60.0 < lap.lap_time < 120.0, (name, lap.lap_time)
        assert lap.season >= 2024
        assert lap.driver


def main() -> int:
    tests = [(name, obj) for name, obj in sorted(globals().items())
             if name.startswith("test_") and callable(obj)]
    passed = 0
    for name, test in tests:
        try:
            test()
        except AssertionError as error:
            print(f"FAIL  {name}  {error}")
        except Exception as error:                    # noqa: BLE001
            print(f"ERROR {name}  {type(error).__name__}: {error}")
        else:
            passed += 1
            print(f"pass  {name}")
    print(f"\n{passed}/{len(tests)} passed")
    return 0 if passed == len(tests) else 1


if __name__ == "__main__":
    raise SystemExit(main())
