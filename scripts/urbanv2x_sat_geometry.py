#!/usr/bin/env python3
"""Compute audited UrbanV2X satellite azimuth/elevation geometry.

Inputs:
  * audited per-signal vehicle/reference C/N0 CSV;
  * UrbanV2X GT trajectory (ECEF + UTC at about 100 Hz); and
  * a daily mixed RINEX 3 broadcast navigation file.

Outputs:
  * one geometry row per unique (UTC epoch, satellite);
  * the original per-signal C/N0 rows augmented with that geometry; and
  * a JSON audit of trajectory matching, ephemeris age and orbit validity.

No elevation mask is applied to the output.  Below-mask observations remain
available for auditing and downstream sensitivity analysis.
"""

import argparse
import bisect
import csv
import json
import math
import os
import statistics
import sys
from collections import Counter, defaultdict
from dataclasses import asdict


SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIRECTORY not in sys.path:
    sys.path.insert(0, SCRIPT_DIRECTORY)

from gnss_broadcast import (  # noqa: E402
    ecef_to_geodetic,
    elevation_azimuth,
    norm3,
    read_rinex3_navigation,
    satellite_clock,
    select_ephemeris,
    transmit_geometry,
)


GEOMETRY_FIELDS = [
    "geometry_status",
    "gt_nearest_dt_s",
    "gt_interpolation_span_s",
    "rx_x_ecef_m",
    "rx_y_ecef_m",
    "rx_z_ecef_m",
    "rx_lat_deg",
    "rx_lon_deg",
    "rx_height_m",
    "sat_x_ecef_m",
    "sat_y_ecef_m",
    "sat_z_ecef_m",
    "sat_radius_m",
    "geometric_range_m",
    "signal_travel_time_s",
    "azimuth_deg",
    "elevation_deg",
    "ephemeris_toe_gpst",
    "ephemeris_toc_gpst",
    "ephemeris_age_s",
    "ephemeris_age_limit_s",
    "ephemeris_health_raw",
    "ephemeris_health_zero",
    "satellite_clock_s",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Compute audited multi-GNSS geometry for UrbanV2X"
    )
    parser.add_argument("--cn0", required=True, help="Audited reference C/N0 CSV")
    parser.add_argument("--gt", required=True, help="UrbanV2X gt_small_loop.txt")
    parser.add_argument("--nav", required=True, help="Mixed RINEX 3 navigation file")
    parser.add_argument(
        "--out-geometry", required=True, help="Unique epoch-satellite geometry CSV"
    )
    parser.add_argument(
        "--out-joined", required=True, help="Per-signal C/N0 plus geometry CSV"
    )
    parser.add_argument("--audit", required=True, help="Geometry audit JSON")
    parser.add_argument(
        "--leap-seconds",
        type=float,
        default=18.0,
        help="GPS-UTC leap seconds at the dataset epoch (default: 18)",
    )
    parser.add_argument(
        "--gt-tolerance",
        type=float,
        default=0.05,
        help="Maximum nearest GT time residual in seconds (default: 0.05)",
    )
    parser.add_argument(
        "--audit-elevation-mask",
        type=float,
        default=5.0,
        help="Mask used only for audit counts; rows are not removed (default: 5)",
    )
    return parser.parse_args()


def percentile(values, probability):
    values = sorted(float(value) for value in values)
    if not values:
        return None
    if len(values) == 1:
        return values[0]
    position = (len(values) - 1) * probability
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return values[lower]
    weight = position - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def describe(values):
    values = [float(value) for value in values if value is not None]
    if not values:
        return {
            "n": 0,
            "min": None,
            "p05": None,
            "median": None,
            "p95": None,
            "max": None,
        }
    return {
        "n": len(values),
        "min": min(values),
        "p05": percentile(values, 0.05),
        "median": statistics.median(values),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


class GroundTruthTrajectory:
    def __init__(self, samples):
        if not samples:
            raise ValueError("GT trajectory contains no numeric samples")
        samples.sort(key=lambda item: item[0])
        self.samples = samples
        self.times = [sample[0] for sample in samples]

    @classmethod
    def read(cls, path):
        samples = []
        with open(path, "r", encoding="ascii", errors="replace", newline=None) as handle:
            for line_number, line in enumerate(handle, 1):
                parts = line.split()
                if len(parts) < 9:
                    continue
                try:
                    epoch = float(parts[0])
                    ecef = (float(parts[6]), float(parts[7]), float(parts[8]))
                except ValueError:
                    continue
                if not all(math.isfinite(value) for value in (epoch,) + ecef):
                    continue
                samples.append((epoch, ecef, line_number))
        return cls(samples)

    def interpolate(self, epoch, tolerance):
        index = bisect.bisect_left(self.times, epoch)
        candidate_indices = []
        if index < len(self.samples):
            candidate_indices.append(index)
        if index > 0:
            candidate_indices.append(index - 1)
        if not candidate_indices:
            return None
        nearest_index = min(
            candidate_indices, key=lambda item: abs(self.times[item] - epoch)
        )
        nearest_dt = self.times[nearest_index] - epoch
        if abs(nearest_dt) > tolerance:
            return None

        if index == 0 or index == len(self.samples):
            _, ecef, _ = self.samples[nearest_index]
            return ecef, nearest_dt, 0.0

        before = self.samples[index - 1]
        after = self.samples[index]
        span = after[0] - before[0]
        if span <= 0.0:
            return before[1], nearest_dt, 0.0
        weight = (epoch - before[0]) / span
        ecef = tuple(
            before[1][axis] + weight * (after[1][axis] - before[1][axis])
            for axis in range(3)
        )
        return ecef, nearest_dt, span


def read_cn0_rows(path):
    with open(path, "r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"epoch_utc", "sat_id", "sys"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError("C/N0 CSV missing columns: %s" % sorted(missing))
        rows = list(reader)
        fieldnames = list(reader.fieldnames)
    return rows, fieldnames


def unique_satellite_epochs(rows):
    keys = {}
    inconsistent = []
    for index, row in enumerate(rows, 2):
        try:
            epoch = float(row["epoch_utc"])
        except ValueError as exc:
            raise ValueError("Invalid epoch_utc on CSV line %d" % index) from exc
        sat_id = row["sat_id"].strip()
        system = row["sys"].strip()
        if not sat_id or sat_id[0] != system:
            inconsistent.append(index)
            continue
        key = (round(epoch, 6), sat_id)
        if key not in keys:
            keys[key] = {
                "epoch_utc": epoch,
                "sat_id": sat_id,
                "sys": system,
                "prn": int(sat_id[1:]),
                "signal_count": 0,
            }
        keys[key]["signal_count"] += 1
    if inconsistent:
        raise ValueError(
            "C/N0 CSV has inconsistent sat_id/sys on lines %s"
            % inconsistent[:10]
        )
    return list(keys.values())


def geometry_for_record(
    record,
    trajectory,
    ephemerides,
    leap_seconds,
    gt_tolerance,
):
    output = dict(record)
    output.update({field: None for field in GEOMETRY_FIELDS})

    gt = trajectory.interpolate(record["epoch_utc"], gt_tolerance)
    if gt is None:
        output["geometry_status"] = "no_gt"
        return output
    receiver_ecef, nearest_dt, interpolation_span = gt
    output["gt_nearest_dt_s"] = nearest_dt
    output["gt_interpolation_span_s"] = interpolation_span
    output["rx_x_ecef_m"], output["rx_y_ecef_m"], output["rx_z_ecef_m"] = receiver_ecef
    latitude, longitude, height = ecef_to_geodetic(receiver_ecef)
    output["rx_lat_deg"] = latitude
    output["rx_lon_deg"] = longitude
    output["rx_height_m"] = height

    receive_gpst = record["epoch_utc"] + leap_seconds
    selected, age, age_limit = select_ephemeris(
        ephemerides.get(record["sat_id"], []), receive_gpst
    )
    output["ephemeris_age_s"] = age
    output["ephemeris_age_limit_s"] = age_limit
    if selected is None:
        output["geometry_status"] = (
            "no_ephemeris" if age is None else "stale_ephemeris"
        )
        return output

    output["ephemeris_toe_gpst"] = selected.toe_gpst
    output["ephemeris_toc_gpst"] = selected.toc_gpst
    output["ephemeris_health_raw"] = selected.health
    output["ephemeris_health_zero"] = selected.health == 0

    try:
        satellite_ecef, geometric_range, travel_time, transmit_gpst = (
            transmit_geometry(selected, receive_gpst, receiver_ecef)
        )
        satellite_radius = norm3(satellite_ecef)
        if not (1.5e7 <= satellite_radius <= 5.0e7):
            output["geometry_status"] = "invalid_satellite_radius"
            output["sat_radius_m"] = satellite_radius
            return output
        elevation, azimuth = elevation_azimuth(receiver_ecef, satellite_ecef)
        values = satellite_ecef + (
            satellite_radius,
            geometric_range,
            travel_time,
            elevation,
            azimuth,
        )
        if not all(math.isfinite(value) for value in values):
            output["geometry_status"] = "nonfinite_geometry"
            return output
        output["sat_x_ecef_m"], output["sat_y_ecef_m"], output["sat_z_ecef_m"] = satellite_ecef
        output["sat_radius_m"] = satellite_radius
        output["geometric_range_m"] = geometric_range
        output["signal_travel_time_s"] = travel_time
        output["elevation_deg"] = elevation
        output["azimuth_deg"] = azimuth
        output["satellite_clock_s"] = satellite_clock(selected, transmit_gpst)
        output["geometry_status"] = "ok"
    except (ArithmeticError, RuntimeError, ValueError) as exc:
        output["geometry_status"] = "orbit_error:%s" % type(exc).__name__
    return output


def _format_value(value):
    if value is None:
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float):
        return "%.9f" % value
    return value


def write_geometry_csv(path, rows):
    fields = [
        "epoch_utc",
        "sat_id",
        "sys",
        "prn",
        "signal_count",
    ] + GEOMETRY_FIELDS
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: _format_value(row.get(field)) for field in fields})


def write_joined_csv(path, cn0_rows, cn0_fields, geometry_by_key):
    appended = [field for field in GEOMETRY_FIELDS if field not in cn0_fields]
    fields = cn0_fields + appended
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for original in cn0_rows:
            key = (round(float(original["epoch_utc"]), 6), original["sat_id"])
            geometry = geometry_by_key[key]
            output = dict(original)
            for field in appended:
                output[field] = _format_value(geometry.get(field))
            writer.writerow(output)


def build_audit(
    cn0_rows,
    geometry_rows,
    trajectory,
    nav_audit,
    elevation_mask,
    settings,
):
    status_counts = Counter(row["geometry_status"] for row in geometry_rows)
    successful = [row for row in geometry_rows if row["geometry_status"] == "ok"]
    systems = sorted({row["sys"] for row in geometry_rows})
    per_system = []
    for system in systems:
        all_system = [row for row in geometry_rows if row["sys"] == system]
        good = [row for row in all_system if row["geometry_status"] == "ok"]
        per_system.append(
            {
                "sys": system,
                "input_unique_satellite_epochs": len(all_system),
                "geometry_ok": len(good),
                "success_ratio": len(good) / len(all_system) if all_system else None,
                "unique_satellites": len({row["sat_id"] for row in all_system}),
                "ephemeris_age_s": describe(
                    [abs(row["ephemeris_age_s"]) for row in good]
                ),
                "elevation_deg": describe([row["elevation_deg"] for row in good]),
                "below_horizon": sum(row["elevation_deg"] < 0.0 for row in good),
                "below_audit_mask": sum(
                    row["elevation_deg"] < elevation_mask for row in good
                ),
                "selected_nonzero_health": sum(
                    not row["ephemeris_health_zero"] for row in good
                ),
            }
        )

    return {
        "schema_version": 1,
        "settings": settings,
        "notes": {
            "time_axis": "epoch_utc is Unix UTC; orbit propagation uses epoch_utc + GPS-UTC",
            "earth_rotation": "Satellite transmit position rotated into receive-time ECEF frame",
            "health_warning": "Galileo health is a signal-specific bit field; nonzero is reported, not automatically rejected",
            "elevation_mask": "Audit count only; no geometry rows are removed",
            "receiver_origin_warning": "GT ECEF is used as receiver origin; GNSS antenna lever arm is not applied",
        },
        "input": {
            "cn0_signal_rows": len(cn0_rows),
            "unique_satellite_epochs": len(geometry_rows),
            "gt_samples": len(trajectory.samples),
            "gt_start_utc": trajectory.times[0],
            "gt_end_utc": trajectory.times[-1],
        },
        "navigation": asdict(nav_audit),
        "geometry": {
            "status_counts": dict(sorted(status_counts.items())),
            "success_ratio": (
                len(successful) / len(geometry_rows) if geometry_rows else None
            ),
            "unique_satellites_ok": len({row["sat_id"] for row in successful}),
            "gt_nearest_dt_s": describe(
                [abs(row["gt_nearest_dt_s"]) for row in successful]
            ),
            "gt_interpolation_span_s": describe(
                [row["gt_interpolation_span_s"] for row in successful]
            ),
            "geometric_range_m": describe(
                [row["geometric_range_m"] for row in successful]
            ),
            "elevation_deg": describe([row["elevation_deg"] for row in successful]),
            "per_system": per_system,
        },
    }


def print_audit(audit):
    print("\n=== Geometry audit ===")
    print("C/N0 signal rows:          %d" % audit["input"]["cn0_signal_rows"])
    print("unique satellite epochs:   %d" % audit["input"]["unique_satellite_epochs"])
    print("GT samples:                %d" % audit["input"]["gt_samples"])
    print("navigation raw records:    %d" % audit["navigation"]["raw_records"])
    print("navigation unique records: %d" % audit["navigation"]["unique_records"])
    print("navigation duplicates:     %d" % audit["navigation"]["duplicate_records"])
    print("geometry status:           %s" % audit["geometry"]["status_counts"])
    print("geometry success:          %.2f%%" % (
        100.0 * audit["geometry"]["success_ratio"]
    ))
    print("unique satellites OK:      %d" % audit["geometry"]["unique_satellites_ok"])

    print("\nPer constellation")
    print("sys,input,ok,success%,sats,age_p95_s,elev_min,elev_median,below0,health_nonzero")
    for row in audit["geometry"]["per_system"]:
        print(
            "%s,%d,%d,%.2f,%d,%.1f,%.2f,%.2f,%d,%d"
            % (
                row["sys"],
                row["input_unique_satellite_epochs"],
                row["geometry_ok"],
                100.0 * row["success_ratio"],
                row["unique_satellites"],
                row["ephemeris_age_s"]["p95"],
                row["elevation_deg"]["min"],
                row["elevation_deg"]["median"],
                row["below_horizon"],
                row["selected_nonzero_health"],
            )
        )


def main():
    args = parse_args()
    if args.gt_tolerance <= 0.0:
        raise ValueError("--gt-tolerance must be positive")

    print("Reading audited C/N0 rows...")
    cn0_rows, cn0_fields = read_cn0_rows(args.cn0)
    satellite_epochs = unique_satellite_epochs(cn0_rows)
    print("  signal rows: %d" % len(cn0_rows))
    print("  unique epoch-satellite rows: %d" % len(satellite_epochs))

    print("Reading GT trajectory...")
    trajectory = GroundTruthTrajectory.read(args.gt)
    print(
        "  samples: %d, UTC %.3f to %.3f"
        % (len(trajectory.samples), trajectory.times[0], trajectory.times[-1])
    )

    print("Reading and deduplicating RINEX navigation...")
    ephemerides, nav_audit = read_rinex3_navigation(
        args.nav, leap_seconds=args.leap_seconds
    )
    print(
        "  raw=%d unique=%d duplicates=%d satellites=%d"
        % (
            nav_audit.raw_records,
            nav_audit.unique_records,
            nav_audit.duplicate_records,
            len(ephemerides),
        )
    )

    print("Computing transmit-time satellite geometry...")
    geometry_rows = []
    for index, record in enumerate(satellite_epochs, 1):
        geometry_rows.append(
            geometry_for_record(
                record,
                trajectory,
                ephemerides,
                args.leap_seconds,
                args.gt_tolerance,
            )
        )
        if index % 1000 == 0:
            print("  processed %d/%d" % (index, len(satellite_epochs)))

    geometry_by_key = {
        (round(row["epoch_utc"], 6), row["sat_id"]): row
        for row in geometry_rows
    }
    write_geometry_csv(args.out_geometry, geometry_rows)
    write_joined_csv(args.out_joined, cn0_rows, cn0_fields, geometry_by_key)

    settings = {
        "leap_seconds": args.leap_seconds,
        "gt_tolerance_s": args.gt_tolerance,
        "audit_elevation_mask_deg": args.audit_elevation_mask,
        "cn0_path": os.path.abspath(args.cn0),
        "gt_path": os.path.abspath(args.gt),
        "nav_path": os.path.abspath(args.nav),
    }
    audit = build_audit(
        cn0_rows,
        geometry_rows,
        trajectory,
        nav_audit,
        args.audit_elevation_mask,
        settings,
    )
    os.makedirs(os.path.dirname(os.path.abspath(args.audit)), exist_ok=True)
    with open(args.audit, "w", encoding="utf-8") as handle:
        json.dump(audit, handle, ensure_ascii=False, indent=2, allow_nan=False)

    print_audit(audit)
    print("\nGeometry CSV: %s" % args.out_geometry)
    print("Joined CSV:   %s" % args.out_joined)
    print("Audit JSON:   %s" % args.audit)


if __name__ == "__main__":
    main()
