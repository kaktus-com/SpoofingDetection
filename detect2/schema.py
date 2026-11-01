"""Measured carrier inputs, plus a separate explicit oracle-displacement loader.

No loader here reads labels, generated trends, phase components or true source
directions. Oracle displacement is the only deliberately permitted truth input.
"""

from dataclasses import dataclass
import csv
from pathlib import Path

import numpy as np


@dataclass
class CarrierSeries:
    """Measured carrier/geometry stream without any displacement attached."""
    time_s: np.ndarray
    phase_cycles: np.ndarray
    nominal_los: np.ndarray
    wavelength_m: np.ndarray
    sigma_cycles: np.ndarray
    signal_ids: tuple
    carrier_valid: np.ndarray
    lock_time_s: np.ndarray
    arc_id: np.ndarray

    def subset(self, indices):
        return CarrierSeries(self.time_s[indices], self.phase_cycles[indices], self.nominal_los,
                              self.wavelength_m, self.sigma_cycles[indices], self.signal_ids,
                              self.carrier_valid[indices], self.lock_time_s[indices], self.arc_id[indices])

    def with_displacement(self, displacement):
        return WindowInput(self.time_s, self.phase_cycles, displacement, self.nominal_los,
                            self.wavelength_m, self.sigma_cycles, self.signal_ids,
                            self.carrier_valid, self.lock_time_s, self.arc_id)


@dataclass
class WindowInput:
    time_s: np.ndarray
    phase_cycles: np.ndarray
    displacement_m: np.ndarray
    nominal_los: np.ndarray
    wavelength_m: np.ndarray
    sigma_cycles: np.ndarray
    signal_ids: tuple
    carrier_valid: np.ndarray | None = None
    lock_time_s: np.ndarray | None = None
    arc_id: np.ndarray | None = None

    def __post_init__(self):
        self.time_s = np.asarray(self.time_s, dtype=float)
        self.phase_cycles = np.asarray(self.phase_cycles, dtype=float)
        self.displacement_m = np.asarray(self.displacement_m, dtype=float)
        self.nominal_los = np.asarray(self.nominal_los, dtype=float)
        self.wavelength_m = np.asarray(self.wavelength_m, dtype=float)
        self.signal_ids = tuple(self.signal_ids)
        n, s = len(self.time_s), len(self.signal_ids)
        if n == 0 or s == 0 or self.time_s.shape != (n,) or self.phase_cycles.shape != (n, s):
            raise ValueError("Expected nonempty time (N), phase (N,S) and S signal identities")
        if len(set(self.signal_ids)) != s:
            raise ValueError("Signal identities must be unique")
        if (not np.isfinite(self.time_s).all() or np.any(np.diff(self.time_s) <= 0)
                or self.displacement_m.shape != (n, 3) or not np.isfinite(self.displacement_m).all()):
            raise ValueError("Times must increase strictly and displacement must be finite (N,3)")
        if (self.nominal_los.shape != (s, 3) or not np.isfinite(self.nominal_los).all()
                or not np.allclose(np.linalg.norm(self.nominal_los, axis=1), 1., rtol=0, atol=1e-9)
                or self.wavelength_m.shape != (s,) or not np.isfinite(self.wavelength_m).all()
                or np.any(self.wavelength_m <= 0)):
            raise ValueError("Geometry requires S unit LOS vectors and positive wavelengths")
        try:
            self.sigma_cycles = np.broadcast_to(np.asarray(self.sigma_cycles, dtype=float), (n, s)).copy()
        except ValueError as exc:
            raise ValueError("Sigma must broadcast to phase shape (N,S)") from exc
        defaults = {"carrier_valid": np.ones((n, s), dtype=bool),
                    "lock_time_s": np.broadcast_to((self.time_s-self.time_s[0])[:, None], (n, s)),
                    "arc_id": np.zeros((n, s), dtype=int)}
        for name, default in defaults.items():
            value = getattr(self, name)
            value = default if value is None else np.asarray(value)
            if value.shape != (n, s):
                raise ValueError(f"{name} must have phase shape (N,S)")
            setattr(self, name, value.copy())

    def subset(self, indices):
        return WindowInput(self.time_s[indices], self.phase_cycles[indices], self.displacement_m[indices],
                           self.nominal_los, self.wavelength_m, self.sigma_cycles[indices], self.signal_ids,
                           self.carrier_valid[indices], self.lock_time_s[indices], self.arc_id[indices])


def _rows(path, required):
    with Path(path).open(newline="") as stream:
        reader = csv.DictReader(stream)
        missing = set(required)-set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"{path}: missing fields {sorted(missing)}")
        rows = list(reader)
    if not rows:
        raise ValueError(f"{path}: no measurements")
    return rows


def load_geometry(path):
    required = ["signal_id", "wavelength_m", "nominal_los_enu_east", "nominal_los_enu_north", "nominal_los_enu_up"]
    rows = _rows(path, required)
    ids = tuple(row["signal_id"] for row in rows)
    if len(set(ids)) != len(ids):
        raise ValueError("Duplicate geometry signal identity")
    return ids, np.array([[float(row[key]) for key in required[2:]] for row in rows]), np.array([float(row["wavelength_m"]) for row in rows])


def load_oracle_displacement(path, measurement_times):
    """Select exact GNSS epochs; do not interpolate or read other truth columns."""
    keys = ["time_s", "antenna_displacement_enu_east_m", "antenna_displacement_enu_north_m", "antenna_displacement_enu_up_m"]
    rows = _rows(path, keys)
    values = np.array([[float(row[key]) for key in keys] for row in rows])
    times = values[:, 0]
    if not np.isfinite(values).all() or np.any(np.diff(times) <= 0):
        raise ValueError("Oracle displacement must have finite, strictly increasing epochs")
    wanted = np.asarray(measurement_times)
    index = np.searchsorted(times, wanted)
    safe = np.minimum(index, len(times)-1)
    if np.any(index >= len(times)) or not np.allclose(times[safe], wanted, atol=1e-9, rtol=0):
        raise ValueError("Oracle displacement does not match carrier measurement epochs")
    return values[index, 1:]


def load_carrier(carrier_path, geometry_path):
    ids, los, wavelengths = load_geometry(geometry_path)
    lookup = {name: i for i, name in enumerate(ids)}
    required = ["time_s", "signal_id", "carrier_phase_cycles", "phase_sigma_cycles", "carrier_valid", "lock_time_s", "arc_id"]
    rows = _rows(carrier_path, required)
    entries, previous = {}, {}
    for row in rows:
        time, name = float(row["time_s"]), row["signal_id"]
        if not np.isfinite(time) or name not in lookup:
            raise ValueError("Carrier row has an invalid time or unknown signal identity")
        if name in previous and time <= previous[name]:
            raise ValueError("Carrier epochs must increase strictly for each signal; duplicates are invalid")
        previous[name] = time
        valid = int(row["carrier_valid"])
        if valid not in (0, 1):
            raise ValueError("carrier_valid must be 0 or 1")
        entries[time, name] = (float(row["carrier_phase_cycles"]), float(row["phase_sigma_cycles"]),
                               valid, float(row["lock_time_s"]), int(row["arc_id"]))
    times = np.array(sorted({time for time, _ in entries}))
    n, s = len(times), len(ids)
    phase, sigma, locks = [np.full((n, s), np.nan) for _ in range(3)]
    valid, arcs = np.zeros((n, s), dtype=bool), np.zeros((n, s), dtype=int)
    for i, time in enumerate(times):
        for j, name in enumerate(ids):
            entry = entries.get((time, name))
            if entry is not None:
                phase[i, j], sigma[i, j], valid[i, j], locks[i, j], arcs[i, j] = entry
    # Validate carrier/geometry structure using the established window schema.
    # Zero is not returned as displacement: CarrierSeries has NO displacement.
    WindowInput(times, phase, np.zeros((n, 3)), los, wavelengths, sigma, ids, valid, locks, arcs)
    return CarrierSeries(times, phase, los, wavelengths, sigma, ids, valid, locks, arcs)


def load_inputs(carrier_path, geometry_path, oracle_path):
    measured = load_carrier(carrier_path, geometry_path)
    displacement = load_oracle_displacement(oracle_path, measured.time_s)
    return measured.with_displacement(displacement)
