"""Build the simulation report as a printable PDF.

`paper.py` owns the page furniture; the argument below is this project's
own, in the order the argument runs.
"""

from __future__ import annotations

from pathlib import Path

from paper import Paper

CAVEAT = (
    "Read the absolute lap times with care. The circuit geometry is "
    "estimated from published layouts, not surveyed, and every corner is a "
    "constant-radius arc. The sensitivities and the downforce trade below "
    "are far more robust than any single lap time, because the geometry "
    "errors largely cancel when the same track is run twice with different "
    "cars.")


def _clock(seconds: float) -> str:
    minutes, rest = divmod(seconds, 60)
    return f"{int(minutes)}:{rest:06.3f}"


def _laps(results) -> tuple[list[str], list[list[str]]]:
    rows = [[name, _clock(one.lap_time), f"{one.top_speed * 3.6:.1f}",
             f"{one.min_speed * 3.6:.1f}", f"{one.mean_speed * 3.6:.1f}",
             f"{one.peak_lateral_g:.2f}",
             f"{one.full_throttle_fraction:.1%}"]
            for name, one in results.items()]
    return (["circuit", "lap", "top km/h", "min km/h", "mean km/h",
             "peak lat g", "full throttle"], rows)


def _validation(comparisons) -> tuple[list[str], list[list[str]]]:
    rows = [[one.circuit, one.clock, one.reference.clock,
             one.reference.driver, f"{one.error:+.2f} s",
             f"{one.error_percent:+.1f}%"]
            for one in comparisons]
    return (["circuit", "simulated", "fastest race lap", "driver", "error",
             "error"], rows)


def _sweeps(sweeps_by_track) -> tuple[list[str], list[list[str]]]:
    rows = []
    for name, sweep in sweeps_by_track.items():
        low, high = sweep.flatness(0.1)
        rows.append([name, f"{sweep.optimum.value:.2f}",
                     _clock(sweep.optimum.lap_time),
                     f"{sweep.optimum.top_speed * 3.6:.1f}",
                     f"{low:.2f} – {high:.2f}"])
    return (["circuit", "optimum CLA", "lap", "top km/h", "within 0.1 s"],
            rows)


def _cross(cross, names) -> tuple[list[str], list[list[str]]]:
    """Each setup run at every circuit, against that circuit's own best."""
    rows = []
    for setup, row in cross.items():
        cells = [f"{setup} setup"]
        for circuit in names:
            own = min(cross[one][circuit] for one in cross)
            lost = row[circuit] - own
            cells.append(_clock(row[circuit])
                         + (f"  +{lost:.2f}" if lost > 0.01 else "  best"))
        rows.append(cells)
    return ([""] + list(names), rows)


def _sensitivity(rows) -> tuple[list[str], list[list[str]]]:
    out = [[one.parameter, one.unit, f"{one.seconds_per_step:+.3f} s",
            f"{one.percent_per_step:+.2f}%"] for one in rows]
    return (["parameter", "per", "lap time", "of a lap"], out)


def build(results, comparisons, sweeps_by_track, cross, sensitivity_rows,
          calibration, figures, out_path: Path, car) -> Path:
    best = min(comparisons, key=lambda one: abs(one.error_percent))
    worst = max(comparisons, key=lambda one: abs(one.error_percent))
    names = list(results)
    out = Path(out_path)
    out = out.with_name("report.pdf") if out.suffix != ".pdf" else out
    out.parent.mkdir(parents=True, exist_ok=True)

    with Paper(out, "Quasi-steady-state lap time simulation",
               f"{car.name} — {car.mass:.0f} kg, "
               f"{car.power / 1000:.0f} kW, CLA {car.cla:.2f} m², "
               f"CDA {car.cda:.2f} m², L/D "
               f"{car.aero_efficiency:.2f}, μ {car.mu:.2f} with load "
               f"exponent {car.load_sensitivity:.2f}") as paper:

        paper.cover(
            headline=f"{len(results)} circuits, agreeing with reality to "
                     f"within {abs(worst.error_percent):.1f}%",
            small=f"Best agreement {best.error_percent:+.1f}% at "
                  f"{best.circuit}, worst {worst.error_percent:+.1f}% at "
                  f"{worst.circuit}. The irreducible RMS after calibration "
                  f"is {calibration.best_rms:.2f} s.",
            tiles=[("circuits simulated", str(len(results))),
                   ("best agreement", f"{best.error_percent:+.1f}%"),
                   ("worst agreement", f"{worst.error_percent:+.1f}%"),
                   ("irreducible RMS", f"{calibration.best_rms:.2f} s")],
            blurb=CAVEAT, figure_path=figures["speed"])

        paper.section("Simulated laps", figure_path=figures["gg"],
                      table=_laps(results))

        paper.section(
            "Against reality",
            note="The reference is the fastest race lap actually set at each "
                 "circuit, which is a lower bound on what the car could do "
                 "there, not a target.",
            figure_path=[figures["validation"], figures["calibration"]],
            table=_validation(comparisons))

        paper.section(
            "The downforce trade-off",
            body="The optimum is flat: the \"within 0.1 s\" column is the "
                 "range of CLA that costs less than a tenth, and it is wide "
                 "enough at most circuits that the setup choice is being "
                 "made on something other than lap time.",
            figure_path=figures["downforce"], table=_sweeps(sweeps_by_track))

        paper.section(
            "Taking the wrong wing to the wrong circuit",
            note="Each row is one setup run at every circuit. The number "
                 "after each lap is what that setup loses against the best "
                 "setup for that circuit.",
            table=_cross(cross, names))

        paper.section(
            "Sensitivities",
            note="What one step of each parameter is worth at the reference "
                 "circuit, holding everything else fixed.",
            table=_sensitivity(sensitivity_rows))

        paper.section(
            "Numerics",
            body="The solver is run at several step sizes so the "
                 "discretisation error can be seen rather than assumed. The "
                 "lap time quoted above is from the finest grid.",
            figure_path=figures["convergence"],
            after="Seeded and deterministic: re-running reproduces every "
                  "number above.")

    return out
