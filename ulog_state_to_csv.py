#!/usr/bin/env python3
"""Export synchronized PX4 attitude and global position from a ULog file."""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

try:
    from pyulog import ULog
except ImportError as exc:  # pragma: no cover - only reached without the dependency
    raise SystemExit("pyulog is required: install it with 'python3 -m pip install pyulog'") from exc


@dataclass(frozen=True)
class ExportStats:
    attitude_samples: int
    position_samples: int
    rows_written: int
    invalid_attitude_samples: int
    unavailable_position_samples: int


def get_dataset(ulog: ULog, name: str, multi_id: int):
    """Return one named uORB dataset instance with a useful error on failure."""
    for dataset in ulog.data_list:
        if dataset.name == name and dataset.multi_id == multi_id:
            return dataset

    instances = sorted(dataset.multi_id for dataset in ulog.data_list if dataset.name == name)
    if instances:
        raise ValueError(
            f"Topic '{name}' has instances {instances}, but instance {multi_id} was requested."
        )
    raise ValueError(f"Required topic '{name}' is not present in the ULog file.")


def sample_timestamps(data: dict[str, np.ndarray]) -> np.ndarray:
    """Prefer the sensor/estimator sample time over the publication time."""
    field = "timestamp_sample" if "timestamp_sample" in data else "timestamp"
    return np.asarray(data[field], dtype=np.int64)


def quaternions_to_euler_deg(q: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Convert PX4 Hamilton quaternions [w, x, y, z] to NED Euler angles."""
    norms = np.linalg.norm(q, axis=1)
    valid = np.all(np.isfinite(q), axis=1) & np.isfinite(norms) & (norms > 1e-12)

    normalized = np.full_like(q, np.nan, dtype=np.float64)
    normalized[valid] = q[valid] / norms[valid, None]
    w, x, y, z = normalized.T

    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def _optional_bool(data: dict[str, np.ndarray], name: str, length: int) -> np.ndarray:
    if name not in data:
        return np.ones(length, dtype=bool)
    return np.asarray(data[name], dtype=bool)


def _optional_counter(data: dict[str, np.ndarray], name: str, length: int) -> np.ndarray:
    if name not in data:
        return np.zeros(length, dtype=np.int64)
    return np.asarray(data[name], dtype=np.int64)


def interpolate_global_position(
    attitude_us: np.ndarray,
    position_data: dict[str, np.ndarray],
    max_gap_us: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Interpolate latitude, longitude and AMSL altitude at attitude timestamps.

    Interpolation is accepted only between two valid position samples from the
    same estimator-reset segment, and never outside the recorded position range.
    """
    position_us = sample_timestamps(position_data)
    lat = np.asarray(position_data["lat"], dtype=np.float64)
    lon = np.asarray(position_data["lon"], dtype=np.float64)
    alt = np.asarray(position_data["alt"], dtype=np.float64)

    order = np.argsort(position_us, kind="stable")
    position_us = position_us[order]
    lat, lon, alt = lat[order], lon[order], alt[order]

    # Keep the last publication if a log contains duplicate sample timestamps.
    keep = np.r_[position_us[1:] != position_us[:-1], True]
    position_us = position_us[keep]
    lat, lon, alt = lat[keep], lon[keep], alt[keep]
    original_indices = order[keep]

    count = len(position_us)
    if count == 0:
        raise ValueError("Topic 'vehicle_global_position' contains no samples.")

    sample_valid = (
        np.isfinite(lat)
        & np.isfinite(lon)
        & np.isfinite(alt)
        & _optional_bool(position_data, "lat_lon_valid", len(order))[original_indices]
        & _optional_bool(position_data, "alt_valid", len(order))[original_indices]
    )
    lat_lon_reset = _optional_counter(
        position_data, "lat_lon_reset_counter", len(order)
    )[original_indices]
    alt_reset = _optional_counter(position_data, "alt_reset_counter", len(order))[
        original_indices
    ]

    right = np.searchsorted(position_us, attitude_us, side="left")
    exact = (right < count) & (position_us[np.minimum(right, count - 1)] == attitude_us)
    left = np.where(exact, right, right - 1)

    in_range = (left >= 0) & (right < count)
    safe_left = np.clip(left, 0, count - 1)
    safe_right = np.clip(right, 0, count - 1)
    gap_us = position_us[safe_right] - position_us[safe_left]

    usable = (
        in_range
        & sample_valid[safe_left]
        & sample_valid[safe_right]
        & (lat_lon_reset[safe_left] == lat_lon_reset[safe_right])
        & (alt_reset[safe_left] == alt_reset[safe_right])
        & (gap_us <= max_gap_us)
    )

    fraction = np.zeros(len(attitude_us), dtype=np.float64)
    between = usable & (gap_us > 0)
    fraction[between] = (
        (attitude_us[between] - position_us[safe_left[between]]) / gap_us[between]
    )

    latitude = lat[safe_left] + fraction * (lat[safe_right] - lat[safe_left])

    # Follow the shortest longitude path if a track crosses the antimeridian.
    lon_delta = (lon[safe_right] - lon[safe_left] + 180.0) % 360.0 - 180.0
    longitude = (lon[safe_left] + fraction * lon_delta + 180.0) % 360.0 - 180.0
    altitude = alt[safe_left] + fraction * (alt[safe_right] - alt[safe_left])

    latitude[~usable] = np.nan
    longitude[~usable] = np.nan
    altitude[~usable] = np.nan
    return latitude, longitude, altitude, usable


def export_ulog_state(
    input_path: Path,
    output_path: Path,
    max_position_gap_s: float = 1.0,
    attitude_instance: int = 0,
    position_instance: int = 0,
) -> ExportStats:
    """Convert a PX4 ULog to one synchronized state CSV."""
    if max_position_gap_s <= 0:
        raise ValueError("Maximum position gap must be greater than zero.")

    ulog = ULog(str(input_path), ["vehicle_attitude", "vehicle_global_position"])
    attitude_data = get_dataset(ulog, "vehicle_attitude", attitude_instance).data
    position_data = get_dataset(ulog, "vehicle_global_position", position_instance).data

    attitude_us = sample_timestamps(attitude_data)
    q = np.column_stack(
        [np.asarray(attitude_data[f"q[{index}]"], dtype=np.float64) for index in range(4)]
    )
    roll, pitch, yaw = quaternions_to_euler_deg(q)
    attitude_valid = np.isfinite(roll) & np.isfinite(pitch) & np.isfinite(yaw)

    latitude, longitude, altitude, position_available = interpolate_global_position(
        attitude_us,
        position_data,
        max_gap_us=int(round(max_position_gap_s * 1_000_000.0)),
    )
    write_row = attitude_valid & position_available
    written_indices = np.flatnonzero(write_row)
    if len(written_indices) == 0:
        raise ValueError("No attitude samples have a safely interpolated global position.")
    first_timestamp_us = attitude_us[written_indices[0]]

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "timestamp_us",
                "time_s",
                "relative_time_s",
                "roll_deg",
                "pitch_deg",
                "yaw_deg",
                "latitude_deg",
                "longitude_deg",
                "altitude_msl_m",
            ]
        )
        for index in written_indices:
            writer.writerow(
                [
                    int(attitude_us[index]),
                    f"{attitude_us[index] * 1e-6:.6f}",
                    f"{(attitude_us[index] - first_timestamp_us) * 1e-6:.6f}",
                    f"{roll[index]:.9f}",
                    f"{pitch[index]:.9f}",
                    f"{yaw[index]:.9f}",
                    f"{latitude[index]:.10f}",
                    f"{longitude[index]:.10f}",
                    f"{altitude[index]:.4f}",
                ]
            )

    return ExportStats(
        attitude_samples=len(attitude_us),
        position_samples=len(sample_timestamps(position_data)),
        rows_written=int(np.count_nonzero(write_row)),
        invalid_attitude_samples=int(np.count_nonzero(~attitude_valid)),
        unavailable_position_samples=int(np.count_nonzero(attitude_valid & ~position_available)),
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Export PX4 vehicle-body roll/pitch/yaw and interpolated fused global "
            "position at every usable attitude sample."
        )
    )
    parser.add_argument("input_file", type=Path, help="Input PX4 .ulg file")
    parser.add_argument(
        "output_file",
        nargs="?",
        type=Path,
        help="Output CSV (default: <input_stem>_state.csv beside the input)",
    )
    parser.add_argument(
        "--max-position-gap",
        type=float,
        default=1.0,
        metavar="SECONDS",
        help="Do not interpolate across a larger position gap (default: 1.0)",
    )
    parser.add_argument("--attitude-instance", type=int, default=0)
    parser.add_argument("--position-instance", type=int, default=0)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    input_path = args.input_file
    output_path = args.output_file or input_path.with_name(f"{input_path.stem}_state.csv")

    if input_path.suffix.lower() != ".ulg":
        print(f"error: expected a .ulg input file, got '{input_path}'", file=sys.stderr)
        return 2
    if not input_path.is_file():
        print(f"error: input file does not exist: '{input_path}'", file=sys.stderr)
        return 2

    try:
        stats = export_ulog_state(
            input_path,
            output_path,
            max_position_gap_s=args.max_position_gap,
            attitude_instance=args.attitude_instance,
            position_instance=args.position_instance,
        )
    except (KeyError, OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    print(f"Wrote {stats.rows_written} rows to {output_path}")
    print(
        f"Read {stats.attitude_samples} attitude samples and "
        f"{stats.position_samples} global-position samples."
    )
    if stats.invalid_attitude_samples:
        print(f"Skipped {stats.invalid_attitude_samples} invalid attitude samples.")
    if stats.unavailable_position_samples:
        print(
            f"Skipped {stats.unavailable_position_samples} attitude samples without "
            "safe position interpolation."
        )
    print("Angles: degrees, yaw range [-180, 180]. Altitude: AMSL metres. Time: since boot.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
