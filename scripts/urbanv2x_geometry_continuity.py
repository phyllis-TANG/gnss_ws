#!/usr/bin/env python3
"""Audit temporal continuity of UrbanV2X satellite geometry.

The primary discontinuity metric is the angular rate between consecutive 3-D
line-of-sight unit vectors.  This avoids false alarms when azimuth wraps from
359 to 0 degrees and is better behaved than azimuth alone near zenith.
"""

import argparse
import csv
import json
import math
import os
import statistics
from collections import Counter, defaultdict


def parse_args():
    parser = argparse.ArgumentParser(
        description="Audit satellite azimuth/elevation temporal continuity"
    )
    parser.add_argument("--geometry", required=True, help="Unique geometry CSV")
    parser.add_argument("--out-summary", required=True, help="Per-satellite CSV")
    parser.add_argument("--out-flags", required=True, help="Flagged-pair CSV")
    parser.add_argument("--audit", required=True, help="Audit JSON")
    parser.add_argument(
        "--max-pair-gap",
        type=float,
        default=2.0,
        help="Largest epoch gap evaluated as a consecutive pair (default: 2 s)",
    )
    parser.add_argument(
        "--max-los-rate",
        type=float,
        default=0.10,
        help="Maximum 3-D LOS angular rate in deg/s (default: 0.10)",
    )
    parser.add_argument(
        "--max-sat-speed",
        type=float,
        default=6000.0,
        help="Maximum apparent satellite ECEF speed in m/s (default: 6000)",
    )
    parser.add_argument(
        "--max-range-rate",
        type=float,
        default=2000.0,
        help="Maximum absolute geometric range rate in m/s (default: 2000)",
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
    values = [float(value) for value in values]
    if not values:
        return {"n": 0, "median": None, "p95": None, "max": None}
    return {
        "n": len(values),
        "median": statistics.median(values),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def circular_difference_degrees(left, right):
    return (left - right + 180.0) % 360.0 - 180.0


def los_unit(azimuth_deg, elevation_deg):
    azimuth = math.radians(azimuth_deg)
    elevation = math.radians(elevation_deg)
    cosine = math.cos(elevation)
    # ENU coordinates: azimuth clockwise from North.
    return (
        cosine * math.sin(azimuth),
        cosine * math.cos(azimuth),
        math.sin(elevation),
    )


def angular_separation_degrees(first, second):
    dot = sum(first[index] * second[index] for index in range(3))
    dot = min(1.0, max(-1.0, dot))
    return math.degrees(math.acos(dot))


def distance3(first, second):
    return math.sqrt(
        sum((first[index] - second[index]) ** 2 for index in range(3))
    )


def read_geometry(path):
    required = {
        "epoch_utc",
        "sat_id",
        "sys",
        "geometry_status",
        "azimuth_deg",
        "elevation_deg",
        "sat_x_ecef_m",
        "sat_y_ecef_m",
        "sat_z_ecef_m",
        "rx_x_ecef_m",
        "rx_y_ecef_m",
        "rx_z_ecef_m",
        "geometric_range_m",
        "ephemeris_toe_gpst",
    }
    rows = []
    ignored_status = Counter()
    duplicate_keys = Counter()
    seen = set()
    with open(path, "r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError("Geometry CSV missing columns: %s" % sorted(missing))
        for line_number, row in enumerate(reader, 2):
            if row["geometry_status"] != "ok":
                ignored_status[row["geometry_status"]] += 1
                continue
            parsed = {
                "line": line_number,
                "epoch_utc": float(row["epoch_utc"]),
                "sat_id": row["sat_id"],
                "sys": row["sys"],
                "azimuth_deg": float(row["azimuth_deg"]),
                "elevation_deg": float(row["elevation_deg"]),
                "sat_ecef": tuple(float(row[name]) for name in (
                    "sat_x_ecef_m", "sat_y_ecef_m", "sat_z_ecef_m"
                )),
                "rx_ecef": tuple(float(row[name]) for name in (
                    "rx_x_ecef_m", "rx_y_ecef_m", "rx_z_ecef_m"
                )),
                "geometric_range_m": float(row["geometric_range_m"]),
                "ephemeris_toe_gpst": float(row["ephemeris_toe_gpst"]),
            }
            values = (
                parsed["epoch_utc"],
                parsed["azimuth_deg"],
                parsed["elevation_deg"],
                parsed["geometric_range_m"],
            ) + parsed["sat_ecef"] + parsed["rx_ecef"]
            if not all(math.isfinite(value) for value in values):
                raise ValueError("Non-finite geometry on CSV line %d" % line_number)
            key = (round(parsed["epoch_utc"], 6), parsed["sat_id"])
            if key in seen:
                duplicate_keys[parsed["sat_id"]] += 1
                continue
            seen.add(key)
            rows.append(parsed)
    return rows, dict(ignored_status), sum(duplicate_keys.values())


def evaluate_pair(previous, current, thresholds):
    delta_time = current["epoch_utc"] - previous["epoch_utc"]
    result = {
        "sat_id": current["sat_id"],
        "sys": current["sys"],
        "previous_epoch_utc": previous["epoch_utc"],
        "epoch_utc": current["epoch_utc"],
        "dt_s": delta_time,
        "ephemeris_switched": (
            abs(current["ephemeris_toe_gpst"] - previous["ephemeris_toe_gpst"])
            > 1e-6
        ),
        "time_gap": False,
        "flags": [],
    }
    if delta_time <= 0.0:
        result["flags"].append("nonpositive_dt")
        return result
    if delta_time > thresholds["max_pair_gap"]:
        # A visibility/measurement gap is audit information, not evidence that
        # the geometry calculation itself is discontinuous.
        result["time_gap"] = True
        return result

    previous_los = los_unit(
        previous["azimuth_deg"], previous["elevation_deg"]
    )
    current_los = los_unit(current["azimuth_deg"], current["elevation_deg"])
    separation = angular_separation_degrees(previous_los, current_los)
    result.update(
        {
            "los_step_deg": separation,
            "los_rate_deg_s": separation / delta_time,
            "azimuth_step_deg": circular_difference_degrees(
                current["azimuth_deg"], previous["azimuth_deg"]
            ),
            "elevation_step_deg": (
                current["elevation_deg"] - previous["elevation_deg"]
            ),
            "satellite_speed_m_s": distance3(
                current["sat_ecef"], previous["sat_ecef"]
            ) / delta_time,
            "receiver_speed_m_s": distance3(
                current["rx_ecef"], previous["rx_ecef"]
            ) / delta_time,
            "range_rate_m_s": (
                current["geometric_range_m"]
                - previous["geometric_range_m"]
            ) / delta_time,
        }
    )
    if result["los_rate_deg_s"] > thresholds["max_los_rate"]:
        result["flags"].append("los_rate")
    if result["satellite_speed_m_s"] > thresholds["max_sat_speed"]:
        result["flags"].append("satellite_speed")
    if abs(result["range_rate_m_s"]) > thresholds["max_range_rate"]:
        result["flags"].append("range_rate")
    return result


def audit_continuity(rows, thresholds):
    by_satellite = defaultdict(list)
    for row in rows:
        by_satellite[row["sat_id"]].append(row)

    all_pairs = []
    flagged_pairs = []
    summaries = []
    for sat_id in sorted(by_satellite):
        satellite_rows = sorted(
            by_satellite[sat_id], key=lambda item: item["epoch_utc"]
        )
        pairs = [
            evaluate_pair(satellite_rows[index - 1], satellite_rows[index], thresholds)
            for index in range(1, len(satellite_rows))
        ]
        all_pairs.extend(pairs)
        flagged_pairs.extend(pair for pair in pairs if pair["flags"])
        evaluated = [pair for pair in pairs if "los_rate_deg_s" in pair]
        summaries.append(
            {
                "sat_id": sat_id,
                "sys": satellite_rows[0]["sys"],
                "epochs": len(satellite_rows),
                "evaluated_pairs": len(evaluated),
                "time_gaps": sum(pair["time_gap"] for pair in pairs),
                "ephemeris_switches": sum(
                    pair["ephemeris_switched"] for pair in evaluated
                ),
                "flagged_pairs": sum(bool(pair["flags"]) for pair in pairs),
                "los_rate": describe(
                    [pair["los_rate_deg_s"] for pair in evaluated]
                ),
                "satellite_speed": describe(
                    [pair["satellite_speed_m_s"] for pair in evaluated]
                ),
                "receiver_speed": describe(
                    [pair["receiver_speed_m_s"] for pair in evaluated]
                ),
                "absolute_range_rate": describe(
                    [abs(pair["range_rate_m_s"]) for pair in evaluated]
                ),
                "switch_los_rate": describe(
                    [
                        pair["los_rate_deg_s"]
                        for pair in evaluated
                        if pair["ephemeris_switched"]
                    ]
                ),
            }
        )
    return summaries, all_pairs, flagged_pairs


def write_summary(path, summaries):
    fields = [
        "sat_id", "sys", "epochs", "evaluated_pairs", "time_gaps",
        "ephemeris_switches", "flagged_pairs", "los_rate_median_deg_s",
        "los_rate_p95_deg_s", "los_rate_max_deg_s", "sat_speed_p95_m_s",
        "sat_speed_max_m_s", "rx_speed_max_m_s", "abs_range_rate_p95_m_s",
        "abs_range_rate_max_m_s", "switch_los_rate_max_deg_s",
    ]
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in summaries:
            writer.writerow({
                "sat_id": row["sat_id"],
                "sys": row["sys"],
                "epochs": row["epochs"],
                "evaluated_pairs": row["evaluated_pairs"],
                "time_gaps": row["time_gaps"],
                "ephemeris_switches": row["ephemeris_switches"],
                "flagged_pairs": row["flagged_pairs"],
                "los_rate_median_deg_s": row["los_rate"]["median"],
                "los_rate_p95_deg_s": row["los_rate"]["p95"],
                "los_rate_max_deg_s": row["los_rate"]["max"],
                "sat_speed_p95_m_s": row["satellite_speed"]["p95"],
                "sat_speed_max_m_s": row["satellite_speed"]["max"],
                "rx_speed_max_m_s": row["receiver_speed"]["max"],
                "abs_range_rate_p95_m_s": row["absolute_range_rate"]["p95"],
                "abs_range_rate_max_m_s": row["absolute_range_rate"]["max"],
                "switch_los_rate_max_deg_s": row["switch_los_rate"]["max"],
            })


def write_flags(path, pairs):
    fields = [
        "sat_id", "sys", "previous_epoch_utc", "epoch_utc", "dt_s",
        "flags", "ephemeris_switched", "los_step_deg", "los_rate_deg_s",
        "azimuth_step_deg", "elevation_step_deg", "satellite_speed_m_s",
        "receiver_speed_m_s", "range_rate_m_s",
    ]
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for pair in pairs:
            output = {field: pair.get(field, "") for field in fields}
            output["flags"] = ";".join(pair["flags"])
            output["ephemeris_switched"] = int(pair["ephemeris_switched"])
            writer.writerow(output)


def build_json_audit(rows, summaries, pairs, flagged, thresholds, ignored, duplicates):
    evaluated = [pair for pair in pairs if "los_rate_deg_s" in pair]
    switches = [pair for pair in evaluated if pair["ephemeris_switched"]]
    flag_counts = Counter(
        flag for pair in flagged for flag in pair["flags"]
    )
    return {
        "schema_version": 1,
        "decision": "pass" if not flagged else "review",
        "thresholds": thresholds,
        "input": {
            "geometry_ok_rows": len(rows),
            "satellites": len(summaries),
            "ignored_geometry_status": ignored,
            "duplicate_epoch_satellite_rows": duplicates,
        },
        "pairs": {
            "all_consecutive_pairs": len(pairs),
            "evaluated_pairs": len(evaluated),
            "time_gap_pairs": sum(pair["time_gap"] for pair in pairs),
            "ephemeris_switch_pairs": len(switches),
            "flagged_pairs": len(flagged),
            "flag_counts": dict(sorted(flag_counts.items())),
            "los_rate_deg_s": describe(
                [pair["los_rate_deg_s"] for pair in evaluated]
            ),
            "satellite_speed_m_s": describe(
                [pair["satellite_speed_m_s"] for pair in evaluated]
            ),
            "receiver_speed_m_s": describe(
                [pair["receiver_speed_m_s"] for pair in evaluated]
            ),
            "absolute_range_rate_m_s": describe(
                [abs(pair["range_rate_m_s"]) for pair in evaluated]
            ),
            "switch_los_rate_deg_s": describe(
                [pair["los_rate_deg_s"] for pair in switches]
            ),
        },
        "per_satellite": summaries,
    }


def main():
    args = parse_args()
    thresholds = {
        "max_pair_gap": args.max_pair_gap,
        "max_los_rate": args.max_los_rate,
        "max_sat_speed": args.max_sat_speed,
        "max_range_rate": args.max_range_rate,
    }
    if any(value <= 0.0 for value in thresholds.values()):
        raise ValueError("All thresholds must be positive")

    print("Reading geometry...")
    rows, ignored, duplicates = read_geometry(args.geometry)
    summaries, pairs, flagged = audit_continuity(rows, thresholds)
    write_summary(args.out_summary, summaries)
    write_flags(args.out_flags, flagged)
    audit = build_json_audit(
        rows, summaries, pairs, flagged, thresholds, ignored, duplicates
    )
    os.makedirs(os.path.dirname(os.path.abspath(args.audit)), exist_ok=True)
    with open(args.audit, "w", encoding="utf-8") as handle:
        json.dump(audit, handle, ensure_ascii=False, indent=2, allow_nan=False)

    pair_audit = audit["pairs"]
    print("\n=== Geometry continuity audit ===")
    print("geometry rows:           %d" % audit["input"]["geometry_ok_rows"])
    print("satellites:              %d" % audit["input"]["satellites"])
    print("evaluated pairs:         %d" % pair_audit["evaluated_pairs"])
    print("time-gap pairs:          %d" % pair_audit["time_gap_pairs"])
    print("ephemeris switches:      %d" % pair_audit["ephemeris_switch_pairs"])
    print("flagged pairs:           %d" % pair_audit["flagged_pairs"])
    print("flag counts:             %s" % pair_audit["flag_counts"])
    print(
        "LOS rate median/p95/max: %.6f / %.6f / %.6f deg/s"
        % (
            pair_audit["los_rate_deg_s"]["median"],
            pair_audit["los_rate_deg_s"]["p95"],
            pair_audit["los_rate_deg_s"]["max"],
        )
    )
    print(
        "sat speed p95/max:       %.3f / %.3f m/s"
        % (
            pair_audit["satellite_speed_m_s"]["p95"],
            pair_audit["satellite_speed_m_s"]["max"],
        )
    )
    print(
        "abs range rate p95/max:  %.3f / %.3f m/s"
        % (
            pair_audit["absolute_range_rate_m_s"]["p95"],
            pair_audit["absolute_range_rate_m_s"]["max"],
        )
    )
    if pair_audit["ephemeris_switch_pairs"]:
        print(
            "switch LOS rate max:      %.6f deg/s"
            % pair_audit["switch_los_rate_deg_s"]["max"]
        )
    print("decision:                %s" % audit["decision"].upper())

    worst = sorted(
        (pair for pair in pairs if "los_rate_deg_s" in pair),
        key=lambda pair: pair["los_rate_deg_s"],
        reverse=True,
    )[:10]
    print("\nTop LOS angular rates")
    print("sat,epoch,dt,rate_deg_s,ephemeris_switched,flags")
    for pair in worst:
        print(
            "%s,%.3f,%.3f,%.6f,%d,%s"
            % (
                pair["sat_id"],
                pair["epoch_utc"],
                pair["dt_s"],
                pair["los_rate_deg_s"],
                int(pair["ephemeris_switched"]),
                ";".join(pair["flags"]),
            )
        )

    print("\nSummary CSV: %s" % args.out_summary)
    print("Flags CSV:   %s" % args.out_flags)
    print("Audit JSON:  %s" % args.audit)


if __name__ == "__main__":
    main()
