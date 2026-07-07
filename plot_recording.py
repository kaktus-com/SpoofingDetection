#!/usr/bin/env python3

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from recording_database import DEFAULT_RECORDING_DIR, load_stream


MAX_LEGEND_ITEMS = 20
DEFAULT_TOP_CARRIERS = 6
COMPARE_CARRIERS = 3
MIN_CORRELATION_SAMPLES = 1000
ATTITUDE_ACTIVITY_PERCENTILE = 98
ATTITUDE_ACTIVITY_MIN_SPEED = 5.0
ATTITUDE_ACTIVITY_MARGIN_SECONDS = 20.0
Y_LIMIT_PERCENTILES = (1, 99)
ATTITUDE_FIELDS = ("roll", "pitch", "yaw")


def latest_session(record_dir):
    sessions = sorted(path for path in Path(record_dir).iterdir() if path.is_dir())
    if not sessions:
        raise FileNotFoundError(f"No recording sessions found in {record_dir}")
    return sessions[-1]


def relative_time(values):
    if len(values) == 0:
        return values
    return values - values[0]


def finite_xy(time, values):
    mask = np.isfinite(time) & np.isfinite(values)
    return time[mask], values[mask]


def window_mask(time, time_window):
    mask = np.isfinite(time)
    if time_window is None:
        return mask

    start, stop = time_window
    return mask & (time >= start) & (time <= stop)


def attitude_values(data, field):
    values = data[field]
    if field == "yaw":
        return np.rad2deg(np.unwrap(np.deg2rad(values)))
    return values


def set_robust_ylim(axis, values):
    values = np.asarray(values)
    values = values[np.isfinite(values)]
    if len(values) == 0:
        return

    low, high = np.percentile(values, Y_LIMIT_PERCENTILES)
    if low == high:
        low = values.min()
        high = values.max()

    if low == high:
        margin = abs(low) * 0.05 or 1.0
    else:
        margin = (high - low) * 0.08

    axis.set_ylim(low - margin, high + margin)


def correlation(x_values, y_values):
    mask = np.isfinite(x_values) & np.isfinite(y_values)
    if mask.sum() < MIN_CORRELATION_SAMPLES:
        return np.nan

    x_values = x_values[mask]
    y_values = y_values[mask]
    if np.std(x_values) == 0 or np.std(y_values) == 0:
        return np.nan

    return np.corrcoef(x_values, y_values)[0, 1]


def attitude_activity_window(data):
    time = relative_time(data["carrier_sync_time"])
    _, unique_indices = np.unique(data["mavlink_time_boot_ms"], return_index=True)
    unique_indices = np.sort(unique_indices)

    attitude_time = time[unique_indices]
    attitude = np.column_stack(
        [
            data["roll"][unique_indices],
            data["pitch"][unique_indices],
            np.rad2deg(np.unwrap(np.deg2rad(data["yaw"][unique_indices]))),
        ]
    )
    valid = np.isfinite(attitude_time) & np.all(np.isfinite(attitude), axis=1)
    attitude_time = attitude_time[valid]
    attitude = attitude[valid]
    if len(attitude_time) < 3:
        return None

    delta_time = np.diff(attitude_time)
    delta_attitude = np.diff(attitude, axis=0)
    usable = delta_time > 0
    speed = np.zeros(len(attitude_time))
    speed[1:][usable] = np.linalg.norm(
        delta_attitude[usable] / delta_time[usable, None],
        axis=1,
    )

    threshold = max(
        np.nanpercentile(speed, ATTITUDE_ACTIVITY_PERCENTILE),
        ATTITUDE_ACTIVITY_MIN_SPEED,
    )
    active = speed > threshold
    if not np.any(active):
        return None

    full_start = attitude_time[0]
    full_stop = attitude_time[-1]
    start = max(full_start, attitude_time[active][0] - ATTITUDE_ACTIVITY_MARGIN_SECONDS)
    stop = min(full_stop, attitude_time[active][-1] + ATTITUDE_ACTIVITY_MARGIN_SECONDS)
    if stop <= start:
        return None

    return start, stop


def ranked_carrier_signals(data, top_count, time_window=None):
    sats = sorted(set(data["sat"]))
    ranked = []

    for sat in sats:
        sat_data = data[data["sat"] == sat]
        sat_time = sat_data["carrier_sync_time"] - data["carrier_sync_time"][0]
        sat_data = sat_data[window_mask(sat_time, time_window)]
        axis_correlations = {
            field: correlation(sat_data["detrended"], attitude_values(sat_data, field))
            for field in ATTITUDE_FIELDS
        }
        finite_correlations = {
            field: value
            for field, value in axis_correlations.items()
            if np.isfinite(value)
        }
        if finite_correlations:
            best_axis, best_correlation = max(
                finite_correlations.items(),
                key=lambda item: abs(item[1]),
            )
        else:
            best_axis, best_correlation = None, np.nan

        ranked.append(
            {
                "sat": sat,
                "rows": len(sat_data),
                "best_axis": best_axis,
                "best_correlation": best_correlation,
                "axis_correlations": axis_correlations,
            }
        )

    ranked.sort(
        key=lambda item: (
            np.nan_to_num(abs(item["best_correlation"]), nan=-1.0),
            item["rows"],
        ),
        reverse=True,
    )
    return ranked[:top_count]


def zscore(values):
    values = np.asarray(values)
    mean = np.nanmean(values)
    std = np.nanstd(values)
    if not np.isfinite(std) or std == 0:
        return values - mean
    return (values - mean) / std


def plot_synced_measurements(data, session_dir, output_path, top_carriers, full_time):
    time = relative_time(data["carrier_sync_time"])
    focus_window = None if full_time else attitude_activity_window(data)
    ranked = ranked_carrier_signals(data, top_carriers, focus_window)

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(14, 10), constrained_layout=True)
    fig.suptitle(f"Synchronized GNSS + IMU recording: {session_dir}")

    selected_carrier = []
    for item in ranked:
        sat = item["sat"]
        sat_data = data[data["sat"] == sat]
        sat_time = sat_data["carrier_sync_time"] - data["carrier_sync_time"][0]
        visible = window_mask(sat_time, focus_window)
        sat_time, detrended = finite_xy(sat_time, sat_data["detrended"])
        selected_carrier.append(sat_data["detrended"][visible])
        axes[0].plot(
            sat_time,
            detrended,
            label=f"{sat} vs {item['best_axis']} r={item['best_correlation']:.2f}",
            linewidth=1.0,
            alpha=0.9,
            rasterized=True,
        )

    axes[0].set_ylabel("Detrended carrier (cycles)")
    axes[0].grid(True, alpha=0.3)
    if selected_carrier:
        set_robust_ylim(axes[0], np.concatenate(selected_carrier))
    if len(ranked) <= MAX_LEGEND_ITEMS:
        axes[0].legend(loc="upper right", fontsize="small", ncols=2)

    for name in ATTITUDE_FIELDS:
        plot_time, values = finite_xy(time, attitude_values(data, name))
        axes[1].plot(plot_time, values, label=name, linewidth=0.9)
    axes[1].set_ylabel("Attitude (deg)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right")
    set_robust_ylim(
        axes[1],
        np.concatenate([attitude_values(data, name) for name in ATTITUDE_FIELDS]),
    )

    for item in ranked[:COMPARE_CARRIERS]:
        if item["best_axis"] is None:
            continue

        sat_data = data[data["sat"] == item["sat"]]
        sat_time = sat_data["carrier_sync_time"] - data["carrier_sync_time"][0]
        mask = (
            np.isfinite(sat_time)
            & np.isfinite(sat_data["detrended"])
            & np.isfinite(attitude_values(sat_data, item["best_axis"]))
        )
        best_attitude = attitude_values(sat_data, item["best_axis"])
        axes[2].plot(
            sat_time[mask],
            zscore(sat_data["detrended"][mask]),
            label=f"{item['sat']} carrier",
            linewidth=1.0,
            alpha=0.9,
        )
        axes[2].plot(
            sat_time[mask],
            zscore(best_attitude[mask]),
            label=f"{item['best_axis']} for {item['sat']}",
            linewidth=1.0,
            alpha=0.75,
            linestyle="--",
        )

    axes[2].set_ylabel("Normalized comparison")
    axes[2].set_xlabel("Recording time (s)")
    axes[2].grid(True, alpha=0.3)
    if ranked:
        axes[2].legend(loc="upper right", fontsize="small", ncols=2)
        axes[2].set_ylim(-3.5, 3.5)

    finite_time = time[np.isfinite(time)]
    if focus_window is not None:
        axes[2].set_xlim(*focus_window)
    elif len(finite_time) > 0:
        axes[2].set_xlim(finite_time.min(), finite_time.max())
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def print_summary(data, session_dir):
    duration = data["carrier_sync_time"][-1] - data["carrier_sync_time"][0]
    sats = sorted(set(data["sat"]))

    print(f"Session: {session_dir}")
    print(f"Rows: {len(data)}")
    print(f"Duration: {duration:.2f} s")
    print(f"Satellites/signals: {len(sats)}")
    print(f"Time offset min/max: {data['time_offset'].min():.4f} / {data['time_offset'].max():.4f} s")
    print(f"Time offset mean abs: {abs(data['time_offset']).mean():.4f} s")


def print_selected_carriers(data, top_carriers, full_time):
    focus_window = None if full_time else attitude_activity_window(data)
    if focus_window is not None:
        print(f"Focused time window: {focus_window[0]:.1f} to {focus_window[1]:.1f} s")
    ranked = ranked_carrier_signals(data, top_carriers, focus_window)
    if not ranked:
        return

    print("Top carrier/attitude correlations:")
    for item in ranked:
        if item["best_axis"] is None:
            print(f"  {item['sat']}: not enough finite samples")
        else:
            print(
                f"  {item['sat']}: {item['best_axis']} r={item['best_correlation']:.3f} "
                f"({item['rows']} rows)"
            )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot synchronized GNSS carrier phase and IMU attitude from CSV recordings."
    )
    parser.add_argument(
        "session",
        nargs="?",
        help="recording session folder; defaults to newest folder in recordings/",
    )
    parser.add_argument(
        "--record-dir",
        default=DEFAULT_RECORDING_DIR,
        help="base recordings directory used when session is omitted",
    )
    parser.add_argument(
        "--output",
        help="output image path; defaults to <session>/synced_measurements.png",
    )
    parser.add_argument(
        "--top-carriers",
        type=int,
        default=DEFAULT_TOP_CARRIERS,
        help="number of carrier signals to plot, ranked by correlation with attitude",
    )
    parser.add_argument(
        "--full-time",
        action="store_true",
        help="plot the full recording instead of zooming to the active attitude window",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    session_dir = Path(args.session) if args.session else latest_session(args.record_dir)
    data = load_stream(session_dir, "synced_measurements")

    if data is None or len(data) == 0:
        raise SystemExit(f"No synced_measurements data found in {session_dir}")

    output_path = Path(args.output) if args.output else session_dir / "synced_measurements.png"

    print_summary(data, session_dir)
    print_selected_carriers(data, args.top_carriers, args.full_time)
    plot_synced_measurements(
        data,
        session_dir,
        output_path,
        args.top_carriers,
        args.full_time,
    )
    print(f"Saved plot: {output_path}")


if __name__ == "__main__":
    main()
