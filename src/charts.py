"""Figures. Committed to the repo so the README renders on GitHub."""

from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import solver  # noqa: E402

REGIME_COLOURS = {
    solver.CORNERING: "#d62728",
    solver.BRAKING: "#1f77b4",
    solver.ACCELERATING: "#2ca02c",
    solver.STRAIGHTLINE: "#999999",
}


def _save(fig, out_dir: Path, name: str) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / name
    fig.savefig(path, dpi=130, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return path


def speed_trace(result: solver.LapResult, track, out_dir: Path) -> Path:
    """Speed, the cornering limit, and what is limiting the car where."""
    fig, (top, bottom) = plt.subplots(
        2, 1, figsize=(13, 7), sharex=True,
        gridspec_kw={"height_ratios": [3, 1], "hspace": 0.08})

    s = result.s / 1000.0
    top.plot(s, result.limit_speed * 3.6, color="#bbbbbb", lw=1.0,
             label="cornering limit (steady state)")
    top.plot(s, result.speed * 3.6, color="#111111", lw=1.6, label="speed")

    for code, colour in REGIME_COLOURS.items():
        mask = result.regime == code
        if mask.any():
            top.fill_between(s, 0, result.speed * 3.6, where=mask,
                             color=colour, alpha=0.18, linewidth=0,
                             label=solver.REGIME_NAMES[code])

    top.set_ylabel("speed  [km/h]")
    top.set_ylim(0, result.top_speed * 3.6 * 1.08)
    top.set_title(f"{result.track_name} — {result.vehicle_name} — "
                  f"lap {_clock(result.lap_time)}")
    top.legend(loc="lower right", fontsize=8, ncols=3, framealpha=0.9)
    top.grid(alpha=0.25)

    for distance, name in getattr(track, "corner_names", ()):
        top.axvline(distance / 1000.0, color="#999999", lw=0.5, alpha=0.5)
        top.text(distance / 1000.0, result.top_speed * 3.6 * 1.02, name,
                 rotation=90, fontsize=5.5, ha="right", va="top",
                 color="#666666")

    bottom.plot(s, result.lat_accel / 9.80665, color="#d62728", lw=1.1,
                label="lateral")
    bottom.plot(s, result.long_accel / 9.80665, color="#1f77b4", lw=1.1,
                label="longitudinal")
    bottom.axhline(0, color="#333333", lw=0.6)
    bottom.set_xlabel("distance round the lap  [km]")
    bottom.set_ylabel("acceleration  [g]")
    bottom.legend(loc="lower right", fontsize=8, ncols=2)
    bottom.grid(alpha=0.25)

    return _save(fig, out_dir, "speed_trace.png")


def gg_diagram(results: dict[str, solver.LapResult], out_dir: Path) -> Path:
    """The g-g diagram: every point of every lap in the acceleration plane.

    The envelope is the friction ellipse the car was given. Points sit on
    it wherever the tyres are saturated and inside it wherever something
    else — power, gearing, the end of the straight — is the limit.
    """
    fig, axes = plt.subplots(1, len(results), figsize=(4.6 * len(results), 4.6),
                             sharex=True, sharey=True)
    axes = np.atleast_1d(axes)

    for ax, (name, result) in zip(axes, results.items()):
        ax.scatter(result.lat_accel / 9.80665, result.long_accel / 9.80665,
                   s=3, alpha=0.35, color="#1f77b4", linewidths=0)
        ax.axhline(0, color="#333333", lw=0.6)
        ax.axvline(0, color="#333333", lw=0.6)
        ax.set_title(f"{name}\npeak {result.peak_lateral_g:.2f} g lateral",
                     fontsize=10)
        ax.set_xlabel("lateral  [g]")
        ax.grid(alpha=0.25)
    axes[0].set_ylabel("longitudinal  [g]   + accelerating")
    fig.suptitle("g-g diagram — where the car actually operates", y=1.02)

    return _save(fig, out_dir, "gg_diagram.png")


def downforce_sweeps(sweeps_by_track: dict, out_dir: Path) -> Path:
    """Lap time against wing level, each curve normalised to its own best."""
    fig, (left, right) = plt.subplots(1, 2, figsize=(13, 5))

    for name, sweep in sweeps_by_track.items():
        values = np.array([p.value for p in sweep.points])
        times = np.array([p.lap_time for p in sweep.points])
        line, = left.plot(values, times - times.min(), lw=1.8, label=name)
        best = sweep.optimum
        left.plot([best.value], [0.0], "o", color=line.get_color(), ms=7)
        right.plot(values, [p.top_speed * 3.6 for p in sweep.points],
                   lw=1.8, color=line.get_color(), label=name)

    left.set_xlabel("downforce  C_L·A  [m²]   (drag follows at fixed L/D)")
    left.set_ylabel("lap time lost vs that circuit's own best  [s]")
    left.set_title("The downforce trade-off")
    left.set_ylim(-0.2, 9.0)
    left.legend()
    left.grid(alpha=0.25)

    right.set_xlabel("downforce  C_L·A  [m²]")
    right.set_ylabel("top speed  [km/h]")
    right.set_title("What the wing costs in a straight line")
    right.legend()
    right.grid(alpha=0.25)

    return _save(fig, out_dir, "downforce.png")


def validation(comparisons, out_dir: Path) -> Path:
    """Simulated against real, as a signed error per circuit."""
    fig, ax = plt.subplots(figsize=(8, 4.2))

    names = [c.circuit for c in comparisons]
    errors = [c.error_percent for c in comparisons]
    colours = ["#2ca02c" if abs(e) < 5 else "#d62728" for e in errors]

    bars = ax.bar(names, errors, color=colours, alpha=0.85)
    for bar, comparison in zip(bars, comparisons):
        offset = 0.35 if comparison.error_percent >= 0 else -0.9
        ax.text(bar.get_x() + bar.get_width() / 2,
                comparison.error_percent + offset,
                f"{comparison.error:+.2f} s\nsim {comparison.clock}\n"
                f"real {comparison.reference.clock}",
                ha="center", fontsize=8)

    ax.axhline(0, color="#333333", lw=1.0)
    ax.axhspan(-5, 5, color="#2ca02c", alpha=0.07)
    ax.set_ylabel("simulated − real  [%]")
    ax.set_title("Against the fastest race lap actually set  "
                 "(+ means the model is slow)")
    ax.set_ylim(min(errors) - 4, max(errors) + 5)
    ax.grid(alpha=0.25, axis="y")

    return _save(fig, out_dir, "validation.png")


def calibration(result, out_dir: Path) -> Path:
    """Shows that no single grip level reconciles all three circuits."""
    fig, ax = plt.subplots(figsize=(8.5, 4.6))

    for name, errors in result.per_track.items():
        ax.plot(result.mus, errors, lw=1.7, label=name)
    ax.plot(result.mus, result.rms_error, lw=2.2, color="#111111",
            ls="--", label="RMS error")

    ax.axhline(0, color="#333333", lw=1.0)
    ax.axvline(result.best_mu, color="#888888", lw=1.0, ls=":")
    ax.annotate(f"best compromise\nμ = {result.best_mu:.2f}, "
                f"RMS {result.best_rms:.2f} s",
                xy=(result.best_mu, result.best_rms),
                xytext=(result.best_mu + 0.08, result.best_rms + 6),
                fontsize=9, arrowprops={"arrowstyle": "->", "color": "#888888"})

    ax.set_xlabel("tyre friction coefficient μ at the reference load")
    ax.set_ylabel("simulated − real  [s]")
    ax.set_title("No single grip level fits all three — the residual is "
                 "geometric, not vehicular")
    ax.legend(fontsize=9)
    ax.grid(alpha=0.25)

    return _save(fig, out_dir, "calibration.png")


def convergence(points, out_dir: Path) -> Path:
    """Lap time against spatial step."""
    fig, ax = plt.subplots(figsize=(7, 4.2))
    steps = [p[0] for p in points]
    times = [p[1] for p in points]
    finest = times[-1]

    ax.plot(steps, [t - finest for t in times], "o-", lw=1.8, color="#1f77b4")
    ax.axhline(0, color="#333333", lw=0.8)
    ax.set_xscale("log")
    ax.set_xlabel("spatial step  ds  [m]   (log scale)")
    ax.set_ylabel(f"lap time − value at ds = {steps[-1]:g} m  [s]")
    ax.set_title("Discretisation convergence")
    ax.grid(alpha=0.25, which="both")

    return _save(fig, out_dir, "convergence.png")


def _clock(seconds: float) -> str:
    minutes, rest = divmod(seconds, 60)
    return f"{int(minutes)}:{rest:06.3f}"
