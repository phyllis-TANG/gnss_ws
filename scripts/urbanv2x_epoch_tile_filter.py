#!/usr/bin/env python3
"""Filter UrbanV2X GNSS observations by audited local-map eligibility.

This script joins the 310 GNSS epochs / 10,850 signal rows to the audited
five-second FAST-LIO tiles.  Selection depends only on independent data
quality: tile decision, distance from a tile boundary, satellite geometry,
and availability of at least one roadside C/N0 reference.  It deliberately
does not filter on C/N0 magnitude, reflectivity, constellation, or NLOS class.
"""

import argparse
import csv
import json
import math
import os
import statistics
import sys
from collections import Counter, defaultdict


SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIRECTORY not in sys.path:
    sys.path.insert(0, SCRIPT_DIRECTORY)

from urbanv2x_fastlio_gt_eval import read_odometry  # noqa: E402


APPENDED_SIGNAL_FIELDS = [
    "odom_elapsed_s",
    "tile_window_index",
    "tile_start_elapsed_s",
    "tile_end_elapsed_s",
    "tile_left_boundary_elapsed_s",
    "tile_right_boundary_elapsed_s",
    "tile_boundary_distance_s",
    "tile_anchor_decision",
    "tile_source_decision",
    "tile_geometry_decision",
    "tile_is_tail",
    "tile_first_pcd",
    "tile_last_pcd",
    "west_reference_available",
    "east_reference_available",
    "reference_availability",
    "analysis_group",
    "eligibility_reasons",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Join UrbanV2X GNSS signals to audited FAST-LIO tiles"
    )
    parser.add_argument("--geometry", required=True)
    parser.add_argument("--anchor-audit", required=True)
    parser.add_argument("--odometry-bag", required=True)
    parser.add_argument("--odometry-topic", default="/Odometry")
    parser.add_argument("--boundary-margin", type=float, default=0.5)
    parser.add_argument("--out-signals", required=True)
    parser.add_argument("--out-epochs", required=True)
    parser.add_argument("--audit", required=True)
    return parser.parse_args()


def finite_value(value):
    if value in (None, ""):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def load_anchor_windows(path):
    with open(path, "r", encoding="utf-8") as handle:
        audit = json.load(handle)
    windows = audit.get("windows")
    if not isinstance(windows, list) or not windows:
        raise ValueError("Anchor audit contains no windows")
    windows = sorted(windows, key=lambda row: int(row["window_index"]))
    expected = list(range(len(windows)))
    observed = [int(row["window_index"]) for row in windows]
    if observed != expected:
        raise ValueError("Anchor window indices are not contiguous from zero")
    required = {
        "start_elapsed_s", "end_elapsed_s", "anchor_decision",
        "source_decision", "geometry_decision", "is_tail",
        "first_pcd", "last_pcd",
    }
    for row in windows:
        missing = required - set(row)
        if missing:
            raise ValueError("Anchor window missing fields: %s" % sorted(missing))
    return audit, windows


def add_continuous_boundaries(windows):
    """Use midpoints between sampled tile endpoints to avoid 10 Hz gaps."""
    result = []
    for index, source in enumerate(windows):
        row = dict(source)
        start = float(row["start_elapsed_s"])
        end = float(row["end_elapsed_s"])
        if end < start:
            raise ValueError("Tile end precedes start")
        if index == 0:
            left = start
        else:
            previous_end = float(windows[index - 1]["end_elapsed_s"])
            left = 0.5 * (previous_end + start)
        if index == len(windows) - 1:
            right = end
        else:
            next_start = float(windows[index + 1]["start_elapsed_s"])
            right = 0.5 * (end + next_start)
        if right < left:
            raise ValueError("Derived tile boundaries overlap in reverse order")
        row["left_boundary_elapsed_s"] = left
        row["right_boundary_elapsed_s"] = right
        result.append(row)
    for left, right in zip(result[:-1], result[1:]):
        if abs(left["right_boundary_elapsed_s"]
               - right["left_boundary_elapsed_s"]) > 1e-9:
            raise ValueError("Tile boundaries are not continuous")
    return result


def find_tile(windows, elapsed):
    for index, row in enumerate(windows):
        left = float(row["left_boundary_elapsed_s"])
        right = float(row["right_boundary_elapsed_s"])
        inside = left <= elapsed < right
        if index == len(windows) - 1:
            inside = left <= elapsed <= right + 1e-6
        if inside:
            return row
    return None


def reference_availability(row):
    west = finite_value(row.get("west_cn0_dbhz")) is not None
    east = finite_value(row.get("east_cn0_dbhz")) is not None
    if west and east:
        label = "BOTH"
    elif west:
        label = "WEST"
    elif east:
        label = "EAST"
    else:
        label = "NONE"
    return west, east, label


def classify_signal(row, tile, boundary_margin):
    """Return group and independent quality reasons for one signal row."""
    reasons = []
    if tile is None:
        reasons.append("outside_tile_coverage")
        return "EXCLUDED", reasons
    left = float(tile["left_boundary_elapsed_s"])
    right = float(tile["right_boundary_elapsed_s"])
    elapsed = float(row["odom_elapsed_s"])
    boundary_distance = min(elapsed - left, right - elapsed)
    if bool(tile["is_tail"]):
        reasons.append("tail_tile")
    if tile["anchor_decision"] == "FAIL":
        reasons.append("tile_fail")
    elif tile["anchor_decision"] == "REVIEW":
        reasons.append("tile_review")
    elif tile["anchor_decision"] != "PASS":
        reasons.append("unknown_tile_decision")
    if boundary_distance < boundary_margin:
        reasons.append("near_tile_boundary")
    if row.get("geometry_status") != "ok":
        reasons.append("geometry_not_ok")
    west, east, _ = reference_availability(row)
    if not (west or east):
        reasons.append("no_reference_cn0")

    blocking = {
        "tail_tile", "tile_fail", "unknown_tile_decision",
        "near_tile_boundary", "geometry_not_ok", "no_reference_cn0",
    }
    if any(reason in blocking for reason in reasons):
        return "EXCLUDED", reasons
    if tile["anchor_decision"] == "REVIEW":
        return "REVIEW_SENSITIVITY", reasons
    return "PRIMARY", reasons


def append_tile_fields(row, tile, elapsed, boundary_margin):
    output = dict(row)
    output["odom_elapsed_s"] = elapsed
    west, east, availability = reference_availability(row)
    output["west_reference_available"] = int(west)
    output["east_reference_available"] = int(east)
    output["reference_availability"] = availability
    if tile is None:
        for field in (
            "tile_window_index", "tile_start_elapsed_s", "tile_end_elapsed_s",
            "tile_left_boundary_elapsed_s", "tile_right_boundary_elapsed_s",
            "tile_boundary_distance_s", "tile_anchor_decision",
            "tile_source_decision", "tile_geometry_decision", "tile_is_tail",
            "tile_first_pcd", "tile_last_pcd",
        ):
            output[field] = ""
    else:
        left = float(tile["left_boundary_elapsed_s"])
        right = float(tile["right_boundary_elapsed_s"])
        output.update({
            "tile_window_index": int(tile["window_index"]),
            "tile_start_elapsed_s": float(tile["start_elapsed_s"]),
            "tile_end_elapsed_s": float(tile["end_elapsed_s"]),
            "tile_left_boundary_elapsed_s": left,
            "tile_right_boundary_elapsed_s": right,
            "tile_boundary_distance_s": min(elapsed - left, right - elapsed),
            "tile_anchor_decision": tile["anchor_decision"],
            "tile_source_decision": tile["source_decision"],
            "tile_geometry_decision": tile["geometry_decision"],
            "tile_is_tail": int(bool(tile["is_tail"])),
            "tile_first_pcd": tile["first_pcd"],
            "tile_last_pcd": tile["last_pcd"],
        })
    group, reasons = classify_signal(output, tile, boundary_margin)
    output["analysis_group"] = group
    output["eligibility_reasons"] = ";".join(reasons) if reasons else "eligible"
    return output


def aggregate_epochs(signal_rows):
    grouped = defaultdict(list)
    for row in signal_rows:
        grouped[round(float(row["epoch_utc"]), 6)].append(row)
    output = []
    priority = {"PRIMARY": 2, "REVIEW_SENSITIVITY": 1, "EXCLUDED": 0}
    for epoch in sorted(grouped):
        rows = grouped[epoch]
        best_group = max(
            (row["analysis_group"] for row in rows), key=priority.get
        )
        reasons = Counter()
        for row in rows:
            for reason in row["eligibility_reasons"].split(";"):
                reasons[reason] += 1
        first = rows[0]
        output.append({
            "epoch_utc": epoch,
            "odom_elapsed_s": first["odom_elapsed_s"],
            "tile_window_index": first["tile_window_index"],
            "tile_anchor_decision": first["tile_anchor_decision"],
            "tile_boundary_distance_s": first["tile_boundary_distance_s"],
            "epoch_analysis_group": best_group,
            "signals_total": len(rows),
            "signals_primary": sum(
                row["analysis_group"] == "PRIMARY" for row in rows
            ),
            "signals_review_sensitivity": sum(
                row["analysis_group"] == "REVIEW_SENSITIVITY" for row in rows
            ),
            "signals_excluded": sum(
                row["analysis_group"] == "EXCLUDED" for row in rows
            ),
            "west_signals": sum(
                int(row["west_reference_available"]) for row in rows
            ),
            "east_signals": sum(
                int(row["east_reference_available"]) for row in rows
            ),
            "both_reference_signals": sum(
                row["reference_availability"] == "BOTH" for row in rows
            ),
            "constellations": "".join(sorted(set(row["sys"] for row in rows))),
            "reason_counts_json": json.dumps(dict(reasons), sort_keys=True),
        })
    return output


def write_csv(path, rows, fields=None):
    if not rows:
        raise ValueError("Cannot write an empty CSV")
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    if fields is None:
        fields = list(rows[0].keys())
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def nested_retention(rows, keys):
    counts = Counter()
    for row in rows:
        key = tuple(row[field] for field in keys) + (row["analysis_group"],)
        counts[key] += 1
    output = []
    for key in sorted(counts):
        record = {field: key[index] for index, field in enumerate(keys)}
        record["analysis_group"] = key[-1]
        record["records"] = counts[key]
        output.append(record)
    return output


def main():
    args = parse_args()
    if args.boundary_margin < 0.0:
        raise ValueError("Boundary margin cannot be negative")
    print("Reading anchor audit and FAST-LIO odometry time axis...")
    anchor_audit, source_windows = load_anchor_windows(args.anchor_audit)
    windows = add_continuous_boundaries(source_windows)
    odometry, odometry_audit = read_odometry(
        args.odometry_bag, args.odometry_topic
    )
    odom_start = float(odometry[0]["time"])
    odom_end = float(odometry[-1]["time"])
    odom_duration = odom_end - odom_start
    window_end = float(windows[-1]["end_elapsed_s"])
    if abs(window_end - odom_duration) > 0.02:
        raise ValueError(
            "Anchor/odometry duration mismatch: %.6f vs %.6f"
            % (window_end, odom_duration)
        )
    print("  odometry UTC %.6f to %.6f; tiles=%d" % (
        odom_start, odom_end, len(windows)
    ))

    print("Reading GNSS C/N0 and satellite geometry rows...")
    with open(args.geometry, "r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        input_fields = list(reader.fieldnames or [])
        required = {
            "epoch_utc", "sat_id", "sys", "signal", "geometry_status",
            "west_cn0_dbhz", "east_cn0_dbhz",
        }
        missing = required - set(input_fields)
        if missing:
            raise ValueError("Geometry CSV missing fields: %s" % sorted(missing))
        raw_rows = list(reader)
    signal_rows = []
    for row in raw_rows:
        epoch = finite_value(row.get("epoch_utc"))
        if epoch is None:
            raise ValueError("Non-finite epoch_utc")
        elapsed = epoch - odom_start
        tile = find_tile(windows, elapsed)
        signal_rows.append(
            append_tile_fields(row, tile, elapsed, args.boundary_margin)
        )
    epoch_rows = aggregate_epochs(signal_rows)
    write_csv(
        args.out_signals, signal_rows,
        fields=input_fields + APPENDED_SIGNAL_FIELDS,
    )
    write_csv(args.out_epochs, epoch_rows)

    signal_groups = Counter(row["analysis_group"] for row in signal_rows)
    epoch_groups = Counter(row["epoch_analysis_group"] for row in epoch_rows)
    reasons = Counter()
    for row in signal_rows:
        for reason in row["eligibility_reasons"].split(";"):
            reasons[reason] += 1
    primary = [row for row in signal_rows if row["analysis_group"] == "PRIMARY"]
    primary_systems = sorted(set(row["sys"] for row in primary))
    project_ready = (
        epoch_groups["PRIMARY"] >= 100
        and signal_groups["PRIMARY"] >= 1000
        and len(primary_systems) >= 3
    )
    decision = "READY_FOR_LOCAL_SURFACE_AUDIT" if project_ready else "REVIEW_SAMPLE_SIZE"
    audit = {
        "schema_version": 1,
        "decision": decision,
        "thresholds_are_project_screens_not_field_standards": {
            "minimum_primary_epochs": 100,
            "minimum_primary_signal_rows": 1000,
            "minimum_primary_constellations": 3,
        },
        "inputs": {
            "geometry_csv": os.path.abspath(args.geometry),
            "anchor_audit": os.path.abspath(args.anchor_audit),
            "odometry_bag": os.path.abspath(args.odometry_bag),
            "odometry_topic": args.odometry_topic,
        },
        "parameters": {
            "boundary_margin_s": args.boundary_margin,
            "selection_is_independent_of_cn0_magnitude_reflectivity_and_nlos": True,
        },
        "time_axis": {
            "odometry_start_utc": odom_start,
            "odometry_end_utc": odom_end,
            "odometry_duration_s": odom_duration,
            "anchor_window_end_elapsed_s": window_end,
            "odometry_audit": odometry_audit,
        },
        "anchor_input_decision": anchor_audit.get("decision"),
        "anchor_window_decision_counts": anchor_audit.get(
            "window_decision_counts"
        ),
        "signal_rows": {
            "total": len(signal_rows),
            "groups": dict(signal_groups),
            "reason_counts": dict(reasons),
        },
        "epochs": {
            "total": len(epoch_rows),
            "groups": dict(epoch_groups),
        },
        "primary_constellations": primary_systems,
        "retention_by_system": nested_retention(signal_rows, ["sys"]),
        "retention_by_system_signal": nested_retention(
            signal_rows, ["sys", "signal"]
        ),
        "retention_by_reference": nested_retention(
            signal_rows, ["reference_availability"]
        ),
        "limitations": [
            "Roadside-reference availability is required but receiver/channel bias is not yet physically calibrated",
            "REVIEW tiles are retained only as a sensitivity-analysis group",
            "Signals near tile boundaries are excluded rather than reassigned after inspecting C/N0",
            "This filter establishes sample eligibility and does not perform ray casting",
        ],
        "outputs": {
            "signals_csv": os.path.abspath(args.out_signals),
            "epochs_csv": os.path.abspath(args.out_epochs),
        },
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.audit)), exist_ok=True)
    with open(args.audit, "w", encoding="utf-8") as handle:
        json.dump(audit, handle, indent=2, sort_keys=True)
        handle.write("\n")

    print("\n=== UrbanV2X epoch/tile eligibility ===")
    print("odometry start UTC:       %.6f" % odom_start)
    print("GNSS epochs total:        %d" % len(epoch_rows))
    print("epochs PRIMARY/REVIEW/EXCLUDED: %d / %d / %d" % (
        epoch_groups["PRIMARY"], epoch_groups["REVIEW_SENSITIVITY"],
        epoch_groups["EXCLUDED"],
    ))
    print("signal rows total:        %d" % len(signal_rows))
    print("signals PRIMARY/REVIEW/EXCLUDED: %d / %d / %d" % (
        signal_groups["PRIMARY"], signal_groups["REVIEW_SENSITIVITY"],
        signal_groups["EXCLUDED"],
    ))
    print("primary constellations:   %s" % ",".join(primary_systems))
    print("exclusion/review reasons:")
    for reason, count in reasons.most_common():
        print("  %-24s %d" % (reason, count))
    print("\nretention by constellation:")
    print("sys,PRIMARY,REVIEW_SENSITIVITY,EXCLUDED")
    for system in sorted(set(row["sys"] for row in signal_rows)):
        values = Counter(
            row["analysis_group"] for row in signal_rows if row["sys"] == system
        )
        print("%s,%d,%d,%d" % (
            system, values["PRIMARY"], values["REVIEW_SENSITIVITY"],
            values["EXCLUDED"],
        ))
    print("decision:                 %s" % decision)
    print("Signals CSV: %s" % args.out_signals)
    print("Epochs CSV:  %s" % args.out_epochs)
    print("Audit JSON:  %s" % args.audit)
    print("Caution: eligibility is not a physical C/N0 attenuation calibration.")


if __name__ == "__main__":
    main()
