"""Simulate three circuits, check against real lap times, run the studies.

    python run.py                  # everything (~2 min)
    python run.py --quick          # coarser, for a smoke test
    python run.py --refresh        # re-fetch the real lap times
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "src"))

import charts  # noqa: E402
import report  # noqa: E402
import solver  # noqa: E402
import sweeps  # noqa: E402
import track as track_module  # noqa: E402
import validate  # noqa: E402
import vehicle as vehicle_module  # noqa: E402

CIRCUITS = ("monza", "silverstone", "monaco")
REFERENCE_CIRCUIT = "monza"


def clock(seconds: float) -> str:
    minutes, rest = divmod(seconds, 60)
    return f"{int(minutes)}:{rest:06.3f}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ds", type=float, default=1.0,
                        help="spatial step, m")
    parser.add_argument("--quick", action="store_true")
    parser.add_argument("--refresh", action="store_true",
                        help="re-fetch reference laps from the API")
    parser.add_argument("--out", default=str(ROOT / "out"))
    args = parser.parse_args()

    levels = np.linspace(2.0, 10.0, 33)
    if args.quick:
        args.ds, levels = 4.0, np.linspace(2.0, 10.0, 9)

    out_dir = Path(args.out)
    car = vehicle_module.Vehicle.from_json(ROOT / "data" / "car.json")
    print(f"car   {car.name}: {car.mass:.0f} kg, {car.power / 1000:.0f} kW, "
          f"CLA {car.cla:.2f}, CdA {car.cda:.2f}, L/D "
          f"{car.aero_efficiency:.2f}, mu {car.mu:.2f} "
          f"(load exponent {car.load_sensitivity:.2f})")

    tracks = {name: track_module.from_json(
        ROOT / "data" / "tracks" / f"{name}.json", ds=args.ds)
        for name in CIRCUITS}

    print("\ntracks")
    for name, track in tracks.items():
        print(f"  {name:12s} {track.length:7.1f} m, {len(track.corner_names):2d} "
              f"corners, {track.corner_fraction:5.1%} of the lap turning, "
              f"closure error {track.closure_error():5.0f} m")

    # -- the laps ---------------------------------------------------------
    results = {name: solver.solve(car, track) for name, track in tracks.items()}

    print("\nsimulated laps")
    for name, result in results.items():
        print(f"  {name:12s} {clock(result.lap_time)}  "
              f"top {result.top_speed * 3.6:5.1f} km/h  "
              f"min {result.min_speed * 3.6:5.1f}  "
              f"peak {result.peak_lateral_g:4.2f} g lat, "
              f"{abs(result.long_accel.min()) / 9.80665:4.2f} g braking  "
              f"full throttle {result.full_throttle_fraction:5.1%}  "
              f"({result.iterations} passes)")
        breakdown = result.regime_breakdown()
        print("               " + "  ".join(
            f"{k} {v:.1%}" for k, v in breakdown.items()))

    # -- against reality --------------------------------------------------
    references = validate.load_references(refresh=args.refresh)
    comparisons = validate.compare(
        {name: r.lap_time for name, r in results.items()}, references)

    print("\nagainst the fastest race lap actually set")
    for comparison in comparisons:
        print(f"  {comparison.circuit:12s} sim {comparison.clock}  "
              f"real {comparison.reference.clock} "
              f"({comparison.reference.driver}, {comparison.reference.season} "
              f"{comparison.reference.race})  "
              f"{comparison.error:+6.2f} s  {comparison.error_percent:+5.1f}%")

    # Is the residual in the car model or in the track files? A single grip
    # level can absorb the first and cannot absorb the second.
    calibration = sweeps.grip_calibration(
        car, tracks, {k: v.lap_time for k, v in references.items()})
    print(f"\nbest single grip level across all three: mu "
          f"{calibration.best_mu:.2f}, RMS error {calibration.best_rms:.2f} s")
    index = int(np.argmin(calibration.rms_error))
    for name, errors in calibration.per_track.items():
        print(f"  {name:12s} residual at that mu {errors[index]:+6.2f} s")

    # -- the downforce trade ---------------------------------------------
    print("\ndownforce trade-off (drag follows downforce at fixed L/D)")
    sweeps_by_track = {}
    for name, track in tracks.items():
        sweep = sweeps.downforce_sweep(car, track, levels)
        sweeps_by_track[name] = sweep
        low, high = sweep.flatness(0.1)
        worst = max(p.lap_time for p in sweep.points)
        print(f"  {name:12s} optimum CLA {sweep.optimum.value:5.2f}  "
              f"lap {clock(sweep.optimum.lap_time)}  "
              f"top {sweep.optimum.top_speed * 3.6:5.1f} km/h  "
              f"within 0.1 s: {low:.2f}-{high:.2f}  "
              f"worst in range costs {worst - sweep.optimum.lap_time:+.2f} s")

    optima = {name: sweep.optimum.value for name, sweep in sweeps_by_track.items()}
    cross = sweeps.cross_table(car, tracks, optima)
    print("\nthe wrong wing at the wrong circuit (rows = setup, cols = circuit)")
    for setup, row in cross.items():
        cells = []
        for circuit in tracks:
            own = min(cross[s][circuit] for s in cross)
            lost = row[circuit] - own
            cells.append(f"{circuit} {clock(row[circuit])} "
                         f"({'best' if lost < 0.01 else f'+{lost:.2f}'})")
        print(f"  {setup:12s} setup: " + "   ".join(cells))

    # -- sensitivities ----------------------------------------------------
    sensitivity_rows = sweeps.sensitivities(car, tracks[REFERENCE_CIRCUIT])
    print(f"\nsensitivities at {REFERENCE_CIRCUIT}")
    for row in sensitivity_rows:
        print(f"  {row.parameter:6s} per {row.unit:18s} "
              f"{row.seconds_per_step:+7.3f} s  ({row.percent_per_step:+.2f}%)")

    # -- numerics ---------------------------------------------------------
    convergence = solver.converged_lap_time(car, tracks[REFERENCE_CIRCUIT])
    print("\ndiscretisation convergence")
    finest = convergence[-1][1]
    for step, lap in convergence:
        print(f"  ds {step:4.1f} m  lap {lap:8.4f} s  "
              f"({lap - finest:+.4f} vs finest)")

    # -- output -----------------------------------------------------------
    figures = {
        "speed": charts.speed_trace(results[REFERENCE_CIRCUIT],
                                    tracks[REFERENCE_CIRCUIT], out_dir),
        "gg": charts.gg_diagram(results, out_dir),
        "downforce": charts.downforce_sweeps(sweeps_by_track, out_dir),
        "validation": charts.validation(comparisons, out_dir),
        "calibration": charts.calibration(calibration, out_dir),
        "convergence": charts.convergence(convergence, out_dir),
    }
    report_path = report.build(results, comparisons, sweeps_by_track, cross,
                               sensitivity_rows, calibration, figures,
                               out_dir / "report.pdf", car)

    print(f"\nreport  {report_path}")
    for path in figures.values():
        print(f"figure  {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
