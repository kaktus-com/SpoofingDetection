#!/usr/bin/env python3

import argparse
import csv
from collections import defaultdict, deque
from pathlib import Path
from statistics import fmean


DEFAULT_WINDOW = 20
DEFAULT_BIAS_WINDOW = 100
DEFAULT_WARMUP = "blank"


def parse_float(value):
    if value is None or value == "":
        return None
    return float(value)


def format_float(value):
    if value is None:
        return ""
    return repr(float(value))


class CarrierDetrender:
    def __init__(
        self,
        window=DEFAULT_WINDOW,
        bias_window=DEFAULT_BIAS_WINDOW,
        window_offset=0,
        output_offset=0.0,
    ):
        if window < 1:
            raise ValueError("window must be at least 1")
        if bias_window < 1:
            raise ValueError("bias_window must be at least 1")
        if window_offset < 0:
            raise ValueError("window_offset must be at least 0")

        self.window = window
        self.bias_window = bias_window
        self.window_offset = window_offset
        self.output_offset = output_offset
        self.carrier_windows = defaultdict(
            lambda: deque(maxlen=window + window_offset)
        )
        self.bias_windows = defaultdict(lambda: deque(maxlen=bias_window))

    def update(self, sat, carrier):
        carrier_window = self.carrier_windows[sat]
        carrier_window.append(carrier)

        if len(carrier_window) < self.window + self.window_offset:
            return None

        if self.window_offset:
            window_values = list(carrier_window)[:-self.window_offset]
        else:
            window_values = carrier_window

        window_mean = fmean(window_values)
        detrended_with_bias = carrier - window_mean

        bias_window = self.bias_windows[sat]
        bias_window.append(detrended_with_bias)
        bias = fmean(bias_window)

        return detrended_with_bias - bias - self.output_offset


def warmup_value(mode, original):
    if mode == "blank":
        return ""
    if mode == "zero":
        return "0.0"
    return original


def satellite_from_carrier_column(column):
    return column.removeprefix("carrier_").replace("_", ":")


def detrended_column_for_carrier(column):
    return "detrended_" + column.removeprefix("carrier_")


def detect_layout(fieldnames):
    fields = set(fieldnames)
    if {"sat", "carrier", "detrended"}.issubset(fields):
        return "long"

    carrier_columns = [
        field
        for field in fieldnames
        if field.startswith("carrier_")
        and detrended_column_for_carrier(field) in fields
    ]
    if carrier_columns:
        return "wide"

    raise ValueError(
        "could not find either long columns "
        "(sat, carrier, detrended) or wide carrier_/detrended_ columns"
    )


def redetrend_long_rows(rows, detrender, warmup):
    for row in rows:
        carrier = parse_float(row.get("carrier"))
        if carrier is None:
            row["detrended"] = warmup_value(warmup, row.get("detrended", ""))
            yield row
            continue

        detrended = detrender.update(row["sat"], carrier)
        if detrended is None:
            row["detrended"] = warmup_value(warmup, row.get("detrended", ""))
        else:
            row["detrended"] = format_float(detrended)
        yield row


def redetrend_wide_rows(rows, fieldnames, detrender, warmup):
    carrier_columns = [
        field
        for field in fieldnames
        if field.startswith("carrier_")
        and detrended_column_for_carrier(field) in fieldnames
    ]

    for row in rows:
        for carrier_column in carrier_columns:
            carrier = parse_float(row.get(carrier_column))
            detrended_column = detrended_column_for_carrier(carrier_column)
            if carrier is None:
                row[detrended_column] = warmup_value(
                    warmup,
                    row.get(detrended_column, ""),
                )
                continue

            sat = satellite_from_carrier_column(carrier_column)
            detrended = detrender.update(sat, carrier)
            if detrended is None:
                row[detrended_column] = warmup_value(
                    warmup,
                    row.get(detrended_column, ""),
                )
            else:
                row[detrended_column] = format_float(detrended)

        yield row


def default_output_path(input_path, window, bias_window, window_offset, output_offset):
    suffix = f"redetrended_w{window}_b{bias_window}"
    if window_offset:
        suffix += f"_wo{window_offset}"
    if output_offset:
        suffix += f"_oo{output_offset:g}"
    return input_path.with_name(f"{input_path.stem}_{suffix}{input_path.suffix}")


def redetrend_file(
    input_path,
    output_path,
    window,
    bias_window,
    window_offset,
    output_offset,
    warmup,
):
    detrender = CarrierDetrender(
        window=window,
        bias_window=bias_window,
        window_offset=window_offset,
        output_offset=output_offset,
    )

    with input_path.open(newline="", encoding="utf-8") as input_file:
        reader = csv.DictReader(input_file)
        if reader.fieldnames is None:
            raise ValueError(f"{input_path} has no CSV header")

        fieldnames = reader.fieldnames
        layout = detect_layout(fieldnames)
        rows = reader

        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", newline="", encoding="utf-8") as output_file:
            writer = csv.DictWriter(output_file, fieldnames=fieldnames)
            writer.writeheader()

            if layout == "long":
                output_rows = redetrend_long_rows(rows, detrender, warmup)
            else:
                output_rows = redetrend_wide_rows(rows, fieldnames, detrender, warmup)

            count = 0
            for row in output_rows:
                writer.writerow(row)
                count += 1

    return layout, count


def find_default_inputs(record_dir):
    return sorted(
        path
        for path in record_dir.glob("*/synced_measurements*.csv")
        if "_redetrended_" not in path.stem
    )


def parse_args():
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description=(
            "Recompute synced_measurements CSV detrended carrier columns from "
            "saved raw carrier phase values."
        )
    )
    parser.add_argument(
        "inputs",
        nargs="*",
        type=Path,
        help=(
            "CSV file(s) to process. Defaults to all "
            "recordings/*/synced_measurements*.csv beside this script."
        ),
    )
    parser.add_argument(
        "--record-dir",
        type=Path,
        default=script_dir / "recordings",
        help="recording directory used when no input files are given",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="output path; only valid when processing one input file",
    )
    parser.add_argument(
        "--window",
        type=int,
        default=DEFAULT_WINDOW,
        help="carrier moving-mean bucket size; original live value is 20",
    )
    parser.add_argument(
        "--bias-window",
        type=int,
        default=DEFAULT_BIAS_WINDOW,
        help="detrended bias moving-mean bucket size; original live value is 100",
    )
    parser.add_argument(
        "--window-offset",
        type=int,
        default=0,
        help=(
            "samples to shift the carrier mean bucket into the past; "
            "0 matches the live code and includes the current sample"
        ),
    )
    parser.add_argument(
        "--output-offset",
        type=float,
        default=0.0,
        help="constant offset, in cycles, subtracted from every recomputed value",
    )
    parser.add_argument(
        "--warmup",
        choices=("blank", "zero", "original"),
        default=DEFAULT_WARMUP,
        help=(
            "value to write before enough samples exist for the selected bucket; "
            "the live code skips these samples"
        ),
    )
    return parser.parse_args()


def main():
    args = parse_args()
    inputs = args.inputs or find_default_inputs(args.record_dir)

    if not inputs:
        raise SystemExit(f"No synced_measurements CSV files found in {args.record_dir}")
    if args.output is not None and len(inputs) != 1:
        raise SystemExit("--output can only be used with exactly one input file")

    for input_path in inputs:
        input_path = input_path.resolve()
        output_path = args.output
        if output_path is None:
            output_path = default_output_path(
                input_path,
                args.window,
                args.bias_window,
                args.window_offset,
                args.output_offset,
            )

        layout, count = redetrend_file(
            input_path=input_path,
            output_path=output_path,
            window=args.window,
            bias_window=args.bias_window,
            window_offset=args.window_offset,
            output_offset=args.output_offset,
            warmup=args.warmup,
        )
        print(f"{input_path} -> {output_path} ({layout}, {count} rows)")


if __name__ == "__main__":
    main()
