"""One self-contained HTML file: charts inlined, no assets, no server."""

from __future__ import annotations

import base64
from pathlib import Path

CSS = """
body{font:15px/1.6 -apple-system,Segoe UI,Roboto,sans-serif;max-width:1120px;
margin:40px auto;padding:0 20px;color:#16181d}
h1{margin-bottom:4px}h2{margin-top:38px;border-bottom:1px solid #e3e6ea;
padding-bottom:6px}
.sub{color:#5b6472;margin-top:0}
.tiles{display:flex;gap:14px;flex-wrap:wrap;margin:22px 0}
.tile{flex:1;min-width:150px;background:#f6f8fa;border:1px solid #e3e6ea;
border-radius:9px;padding:14px 16px}
.tile .v{font-size:25px;font-weight:650;letter-spacing:-.5px}
.tile .k{font-size:11px;text-transform:uppercase;letter-spacing:.07em;
color:#5b6472;margin-top:3px}
table{border-collapse:collapse;width:100%;margin:14px 0;font-size:14px}
th,td{border:1px solid #e3e6ea;padding:7px 10px;text-align:right}
th:first-child,td:first-child{text-align:left}
th{background:#f6f8fa;font-weight:600}
td.good{color:#17803d}td.bad{color:#b4232c}
img{width:100%;margin:14px 0;border:1px solid #e3e6ea;border-radius:7px}
.note{background:#fff8e6;border-left:3px solid #d9a441;padding:11px 15px;
margin:18px 0;font-size:14px}
code{background:#f0f2f5;padding:1px 5px;border-radius:4px;font-size:13px}
"""


def _img(path: Path) -> str:
    data = base64.b64encode(Path(path).read_bytes()).decode("ascii")
    return f'<img src="data:image/png;base64,{data}" alt="{Path(path).stem}">'


def _clock(seconds: float) -> str:
    minutes, rest = divmod(seconds, 60)
    return f"{int(minutes)}:{rest:06.3f}"


def build(results, comparisons, sweeps_by_track, cross, sensitivity_rows,
          calibration, figures, out_path: Path, car) -> Path:
    best = min(comparisons, key=lambda c: abs(c.error_percent))

    tiles = [
        ("circuits simulated", str(len(results))),
        ("best agreement", f"{best.error_percent:+.1f}%"),
        ("worst agreement",
         f"{max(comparisons, key=lambda c: abs(c.error_percent)).error_percent:+.1f}%"),
        ("irreducible RMS", f"{calibration.best_rms:.2f} s"),
    ]
    tiles_html = "".join(
        f'<div class="tile"><div class="v">{v}</div><div class="k">{k}</div></div>'
        for k, v in tiles)

    lap_rows = "".join(
        f"<tr><td>{name}</td><td>{_clock(r.lap_time)}</td>"
        f"<td>{r.top_speed * 3.6:.1f}</td><td>{r.min_speed * 3.6:.1f}</td>"
        f"<td>{r.mean_speed * 3.6:.1f}</td><td>{r.peak_lateral_g:.2f}</td>"
        f"<td>{r.full_throttle_fraction:.1%}</td></tr>"
        for name, r in results.items())

    val_rows = "".join(
        f"<tr><td>{c.circuit}</td><td>{c.clock}</td>"
        f"<td>{c.reference.clock}</td><td>{c.reference.driver}</td>"
        f'<td class="{"good" if abs(c.error_percent) < 5 else "bad"}">'
        f"{c.error:+.2f} s</td>"
        f'<td class="{"good" if abs(c.error_percent) < 5 else "bad"}">'
        f"{c.error_percent:+.1f}%</td></tr>"
        for c in comparisons)

    names = list(results)
    cross_head = "".join(f"<th>{n}</th>" for n in names)
    cross_rows = ""
    for setup, row in cross.items():
        cells = ""
        for circuit in names:
            own = min(cross[s][circuit] for s in cross)
            lost = row[circuit] - own
            klass = "good" if lost < 0.01 else ("bad" if lost > 1.0 else "")
            cells += (f'<td class="{klass}">{_clock(row[circuit])}'
                      + (f"<br><small>+{lost:.2f}</small>" if lost > 0.01
                         else "<br><small>best</small>") + "</td>")
        cross_rows += f"<tr><td>{setup} setup</td>{cells}</tr>"

    sens_rows = "".join(
        f"<tr><td>{s.parameter}</td><td>{s.unit}</td>"
        f"<td>{s.seconds_per_step:+.3f} s</td>"
        f"<td>{s.percent_per_step:+.2f}%</td></tr>"
        for s in sensitivity_rows)

    sweep_rows = "".join(
        f"<tr><td>{name}</td><td>{sw.optimum.value:.2f}</td>"
        f"<td>{_clock(sw.optimum.lap_time)}</td>"
        f"<td>{sw.optimum.top_speed * 3.6:.1f}</td>"
        f"<td>{sw.flatness(0.1)[0]:.2f} – {sw.flatness(0.1)[1]:.2f}</td></tr>"
        for name, sw in sweeps_by_track.items())

    html = f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Lap time simulation</title><style>{CSS}</style></head><body>
<h1>Quasi-steady-state lap time simulation</h1>
<p class="sub">{car.name} — {car.mass:.0f} kg, {car.power / 1000:.0f} kW,
C<sub>L</sub>A {car.cla:.2f} m², C<sub>D</sub>A {car.cda:.2f} m²,
L/D {car.aero_efficiency:.2f}, μ {car.mu:.2f} with load exponent
{car.load_sensitivity:.2f}</p>
<div class="tiles">{tiles_html}</div>

<div class="note"><b>Read the absolute lap times with care.</b> The circuit
geometry is estimated from published layouts, not surveyed, and every corner
is a constant-radius arc. The sensitivities and the downforce trade below are
far more robust than any single lap time, because the geometry errors largely
cancel when the same track is run twice with different cars.</div>

<h2>Simulated laps</h2>
<table><tr><th>circuit</th><th>lap</th><th>top km/h</th><th>min km/h</th>
<th>mean km/h</th><th>peak lat g</th><th>full throttle</th></tr>
{lap_rows}</table>
{_img(figures["speed"])}
{_img(figures["gg"])}

<h2>Against reality</h2>
<table><tr><th>circuit</th><th>simulated</th><th>fastest race lap</th>
<th>driver</th><th>error</th><th>error</th></tr>{val_rows}</table>
{_img(figures["validation"])}
{_img(figures["calibration"])}

<h2>The downforce trade-off</h2>
<table><tr><th>circuit</th><th>optimum C<sub>L</sub>A</th><th>lap</th>
<th>top km/h</th><th>within 0.1 s</th></tr>{sweep_rows}</table>
{_img(figures["downforce"])}

<h2>Taking the wrong wing to the wrong circuit</h2>
<table><tr><th></th>{cross_head}</tr>{cross_rows}</table>

<h2>Sensitivities</h2>
<table><tr><th>parameter</th><th>per</th><th>lap time</th><th>of a lap</th>
</tr>{sens_rows}</table>

<h2>Numerics</h2>
{_img(figures["convergence"])}
</body></html>"""

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(html, encoding="utf-8")
    return out_path
