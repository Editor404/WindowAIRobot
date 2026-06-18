from __future__ import annotations

import argparse
import csv
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from window_cleaner.arduino_imu import ArduinoSensorSample, parse_sensor_line


@dataclass(frozen=True)
class PressureSummary:
    count: int
    pressure_min: int
    pressure_max: int
    pressure_mean: float
    pressure_stdev: float
    blower_min: int | None
    blower_max: int | None
    adhesion_secure_ratio: float | None


def summarize_samples(samples: list[ArduinoSensorSample]) -> PressureSummary:
    if not samples:
        raise ValueError("No SENSOR samples were collected")

    pressure_values = [sample.pressure_raw for sample in samples]
    blower_values = [sample.blower_pwm for sample in samples if sample.blower_pwm is not None]
    adhesion_values = [sample.adhesion_secure for sample in samples if sample.adhesion_secure is not None]

    return PressureSummary(
        count=len(samples),
        pressure_min=min(pressure_values),
        pressure_max=max(pressure_values),
        pressure_mean=statistics.fmean(pressure_values),
        pressure_stdev=statistics.pstdev(pressure_values) if len(pressure_values) > 1 else 0.0,
        blower_min=min(blower_values) if blower_values else None,
        blower_max=max(blower_values) if blower_values else None,
        adhesion_secure_ratio=(sum(1 for value in adhesion_values if value) / len(adhesion_values))
        if adhesion_values
        else None,
    )


def collect_samples(port: str, baud: int, duration_s: float) -> list[ArduinoSensorSample]:
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("pyserial is required: install python3-serial or pyserial") from exc

    deadline = time.monotonic() + duration_s
    samples: list[ArduinoSensorSample] = []
    with serial.Serial(port, baud, timeout=0.2) as arduino:
        while time.monotonic() < deadline:
            raw_line = arduino.readline().decode("ascii", errors="replace").strip()
            if not raw_line:
                continue
            try:
                sample = parse_sensor_line(raw_line)
            except ValueError:
                continue
            if sample is not None:
                samples.append(sample)
    return samples


def write_csv(path: Path, samples: list[ArduinoSensorSample]) -> None:
    with path.open("w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(
            [
                "timestamp_ms",
                "pressure_raw",
                "gyro_valid",
                "gyro_x_dps",
                "gyro_y_dps",
                "gyro_z_dps",
                "blower_pwm",
                "adhesion_secure",
            ]
        )
        for sample in samples:
            writer.writerow(
                [
                    sample.timestamp_ms,
                    sample.pressure_raw,
                    int(sample.gyro_valid),
                    sample.gyro_x_dps,
                    sample.gyro_y_dps,
                    sample.gyro_z_dps,
                    "" if sample.blower_pwm is None else sample.blower_pwm,
                    "" if sample.adhesion_secure is None else int(sample.adhesion_secure),
                ]
            )


def print_summary(summary: PressureSummary) -> None:
    print(f"samples: {summary.count}")
    print(
        "pressure_raw: "
        f"min={summary.pressure_min} "
        f"max={summary.pressure_max} "
        f"mean={summary.pressure_mean:.1f} "
        f"stdev={summary.pressure_stdev:.1f}"
    )
    if summary.blower_min is not None and summary.blower_max is not None:
        print(f"blower_pwm: min={summary.blower_min} max={summary.blower_max}")
    if summary.adhesion_secure_ratio is not None:
        print(f"adhesion_secure: {summary.adhesion_secure_ratio * 100:.1f}% of samples")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Collect Arduino pressure_raw samples for window adhesion diagnostics."
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="Arduino serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Arduino serial baud rate")
    parser.add_argument("--duration", type=float, default=10.0, help="Collection duration in seconds")
    parser.add_argument("--csv", type=Path, help="Optional CSV output path")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    samples = collect_samples(args.port, args.baud, args.duration)
    summary = summarize_samples(samples)
    print_summary(summary)
    if args.csv:
        write_csv(args.csv, samples)
        print(f"wrote: {args.csv}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"adhesion diagnostic failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
