"""Compare the simulation against real Formula 1 lap times.

A lap time simulation that is never checked against a stopwatch is an
animation. This module pulls the fastest race lap actually set at each
circuit from the public Jolpica API — the open continuation of Ergast —
and reports the error.

Two honest caveats travel with every number here:

* The fastest *race* lap is set on a heavy, worn car in traffic, so it is
  slower than the car's true potential. Qualifying is closer to the
  simulation's assumption of a clean lap on the limit, but is not exposed
  as a lap time by this API, so the race lap is what is compared and the
  simulation should be expected to sit a little *under* it.
* The circuit geometry here is estimated, not surveyed, so part of any
  error belongs to the track file rather than to the vehicle model.

Results are cached on disk, so the repo can be re-run without hammering a
free community API, and so the numbers in the README stay reproducible.
"""

from __future__ import annotations

import json
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path

API = "https://api.jolpi.ca/ergast/f1/"
CACHE = Path(__file__).resolve().parent.parent / "data" / "reference_laps.json"

# Circuit identifiers as the API spells them.
CIRCUIT_IDS = {"monza": "monza", "monaco": "monaco",
               "silverstone": "silverstone"}


@dataclass(frozen=True)
class ReferenceLap:
    """The fastest race lap actually recorded at a circuit."""

    circuit: str
    season: int
    race: str
    driver: str
    lap_time: float      # s
    lap_number: int

    @property
    def clock(self) -> str:
        minutes, seconds = divmod(self.lap_time, 60)
        return f"{int(minutes)}:{seconds:06.3f}"


def _parse_clock(text: str) -> float:
    minutes, _, seconds = text.partition(":")
    return int(minutes) * 60 + float(seconds) if _ else float(text)


def _fetch(path: str, timeout: float = 30.0) -> dict:
    with urllib.request.urlopen(API + path, timeout=timeout) as response:
        return json.load(response)["MRData"]


def fetch_reference(circuit: str, season: int) -> ReferenceLap | None:
    """Fastest race lap at one circuit in one season, or None."""
    data = _fetch(f"{season}/circuits/{CIRCUIT_IDS[circuit]}/results.json?limit=100")
    races = data["RaceTable"]["Races"]
    if not races:
        return None
    race = races[0]

    best: ReferenceLap | None = None
    for result in race["Results"]:
        fastest = result.get("FastestLap")
        if not fastest or "Time" not in fastest:
            continue
        seconds = _parse_clock(fastest["Time"]["time"])
        if best is None or seconds < best.lap_time:
            best = ReferenceLap(
                circuit=circuit, season=season, race=race["raceName"],
                driver=result["Driver"]["familyName"], lap_time=seconds,
                lap_number=int(fastest.get("lap", 0)))
    return best


def load_references(seasons=(2026, 2025), *, refresh: bool = False,
                    ) -> dict[str, ReferenceLap]:
    """Best available real lap per circuit, newest season first.

    Falls back to the previous season where the current one has not yet
    visited a circuit, and falls back to the cache when offline.
    """
    if CACHE.exists() and not refresh:
        raw = json.loads(CACHE.read_text(encoding="utf-8"))
        return {k: ReferenceLap(**v) for k, v in raw.items()}

    found: dict[str, ReferenceLap] = {}
    for circuit in CIRCUIT_IDS:
        for season in seasons:
            try:
                lap = fetch_reference(circuit, season)
            except (urllib.error.URLError, TimeoutError, KeyError):
                lap = None
            if lap is not None:
                found[circuit] = lap
                break

    CACHE.parent.mkdir(parents=True, exist_ok=True)
    CACHE.write_text(json.dumps({k: v.__dict__ for k, v in found.items()},
                                indent=2) + "\n", encoding="utf-8")
    return found


@dataclass(frozen=True)
class Comparison:
    circuit: str
    simulated: float
    reference: ReferenceLap

    @property
    def error(self) -> float:
        """Simulated minus real, s. Negative means the model is optimistic."""
        return self.simulated - self.reference.lap_time

    @property
    def error_percent(self) -> float:
        return 100.0 * self.error / self.reference.lap_time

    @property
    def clock(self) -> str:
        minutes, seconds = divmod(self.simulated, 60)
        return f"{int(minutes)}:{seconds:06.3f}"


def compare(simulated: dict[str, float],
            references: dict[str, ReferenceLap]) -> list[Comparison]:
    return [Comparison(name, simulated[name], references[name])
            for name in simulated if name in references]
