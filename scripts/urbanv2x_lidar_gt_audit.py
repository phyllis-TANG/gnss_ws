#!/usr/bin/env python3
"""Audit the UrbanV2X Velodyne, ground-truth, and calibration inputs.

This script deliberately stops before point-cloud mapping.  It verifies the
time bases and the LiDAR-to-IMU calibration structure, and records both
possible interpretations of the signed calibration lag.  The final world
frame convention still has to be validated with an accumulated static map.
"""

import argparse
import bisect
import csv
import json
import math
import os
import re
import statistics
import struct
import sys
from collections import Counter


SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIRECTORY not in sys.path:
    sys.path.insert(0, SCRIPT_DIRECTORY)

from gnss_broadcast import geodetic_to_ecef  # noqa: E402


POINT_FIELD_FORMATS = {
    1: "b",   # INT8
    2: "B",   # UINT8
    3: "h",   # INT16
    4: "H",   # UINT16
    5: "i",   # INT32
    6: "I",   # UINT32
    7: "f",   # FLOAT32
    8: "d",   # FLOAT64
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Audit UrbanV2X LiDAR/GT time and calibration inputs"
    )
    parser.add_argument("--bag", required=True, help="UrbanV2X ROS1 bag")
    parser.add_argument("--gt", required=True, help="UrbanV2X GT text file")
    parser.add_argument(
        "--calibration", required=True,
        help="Velodyne Initialization_result calibration text",
    )
    parser.add_argument("--topic", default="/velodyne_points")
    parser.add_argument("--out-frames", required=True, help="Per-frame audit CSV")
    parser.add_argument("--audit", required=True, help="Audit JSON")
    parser.add_argument(
        "--point-time-frame-step", type=int, default=100,
        help="Inspect point timestamps every N LiDAR frames (default: 100)",
    )
    parser.add_argument(
        "--point-time-samples", type=int, default=512,
        help="Maximum evenly spaced points inspected per sampled frame",
    )
    parser.add_argument(
        "--gt-tolerance", type=float, default=0.01,
        help="Maximum nearest-GT residual for timing PASS (default: 0.01 s)",
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
        return {"n": 0, "min": None, "p05": None, "median": None,
                "p95": None, "max": None}
    return {
        "n": len(values),
        "min": min(values),
        "p05": percentile(values, 0.05),
        "median": statistics.median(values),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def norm3(vector):
    return math.sqrt(sum(float(value) ** 2 for value in vector))


def subtract3(left, right):
    return tuple(float(left[index]) - float(right[index]) for index in range(3))


def signed_angle_difference_degrees(left, right):
    return (float(left) - float(right) + 180.0) % 360.0 - 180.0


def determinant3(matrix):
    return (
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )


def rotation_metrics(matrix4):
    rotation = [row[:3] for row in matrix4[:3]]
    gram = [
        [sum(rotation[k][i] * rotation[k][j] for k in range(3))
         for j in range(3)]
        for i in range(3)
    ]
    orthogonality_max_error = max(
        abs(gram[i][j] - (1.0 if i == j else 0.0))
        for i in range(3) for j in range(3)
    )
    return {
        "determinant": determinant3(rotation),
        "orthogonality_max_error": orthogonality_max_error,
        "translation_norm_m": norm3([matrix4[i][3] for i in range(3)]),
        "bottom_row_max_error": max(
            abs(matrix4[3][i] - (1.0 if i == 3 else 0.0)) for i in range(4)
        ),
    }


def _numbers(text):
    return [
        float(value)
        for value in re.findall(r"[-+]?(?:\d+\.\d*|\d*\.\d+|\d+)(?:[Ee][-+]?\d+)?", text)
    ]


def parse_calibration(path):
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        text = handle.read()
    source = "refinement" if "Refinement result:" in text else "initialization"
    section = text.split("Refinement result:")[-1] if source == "refinement" else text
    lag_match = re.search(
        r"Time\s+Lag\s+IMU\s+to\s+LiDAR\s*\(second\)\s*=\s*([-+0-9.eE]+)",
        section,
    )
    if not lag_match:
        raise ValueError("Calibration time lag was not found")
    lines = section.splitlines()
    matrix = None
    for index, line in enumerate(lines):
        if "Homogeneous Transformation Matrix from LiDAR to IMU" not in line:
            continue
        candidate = []
        for matrix_line in lines[index + 1:index + 7]:
            values = _numbers(matrix_line)
            if len(values) == 4:
                candidate.append(values)
                if len(candidate) == 4:
                    break
        if len(candidate) == 4:
            matrix = candidate
            break
    if matrix is None:
        raise ValueError("LiDAR-to-IMU homogeneous matrix was not found")
    return {
        "source_section": source,
        "direction": "lidar_to_imu",
        "time_lag_imu_to_lidar_s": float(lag_match.group(1)),
        "matrix": matrix,
        "metrics": rotation_metrics(matrix),
    }


def read_ground_truth(path):
    samples = []
    with open(path, "r", encoding="ascii", errors="replace", newline=None) as handle:
        for line_number, line in enumerate(handle, 1):
            parts = line.split()
            if len(parts) < 25:
                continue
            try:
                values = [float(value) for value in parts[:24]]
                quality = int(float(parts[24]))
            except ValueError:
                continue
            if not all(math.isfinite(value) for value in values):
                continue
            samples.append({
                "line": line_number,
                "time": values[0],
                "latitude": values[3],
                "longitude": values[4],
                "height": values[5],
                "ecef": tuple(values[6:9]),
                "velocity_ecef": tuple(values[9:12]),
                "velocity_enu": tuple(values[12:15]),
                "roll": values[21],
                "pitch": values[22],
                "heading": values[23],
                "quality": quality,
            })
    if not samples:
        raise ValueError("No numeric GT samples found")
    samples.sort(key=lambda item: item["time"])
    return samples


def nearest_time(times, epoch):
    index = bisect.bisect_left(times, epoch)
    candidates = []
    if index < len(times):
        candidates.append(index)
    if index > 0:
        candidates.append(index - 1)
    if not candidates:
        return None, None
    nearest = min(candidates, key=lambda item: abs(times[item] - epoch))
    return nearest, times[nearest] - epoch


def audit_ground_truth(samples):
    times = [sample["time"] for sample in samples]
    intervals = [times[i] - times[i - 1] for i in range(1, len(times))]
    lla_ecef_errors = []
    horizontal_speeds = []
    course_heading_errors = []
    for sample in samples:
        calculated = geodetic_to_ecef(
            sample["latitude"], sample["longitude"], sample["height"]
        )
        lla_ecef_errors.append(norm3(subtract3(calculated, sample["ecef"])))
        east, north = sample["velocity_enu"][:2]
        speed = math.hypot(east, north)
        horizontal_speeds.append(speed)
        if speed >= 2.0:
            course = math.degrees(math.atan2(east, north)) % 360.0
            course_heading_errors.append(
                abs(signed_angle_difference_degrees(sample["heading"], course))
            )

    velocity_errors = []
    # A one-second central difference avoids magnifying the millimetre-rounded
    # ECEF positions at the native 100 Hz rate.
    median_interval = statistics.median(
        interval for interval in intervals if interval > 0.0
    )
    half_window = max(1, int(round(0.5 / median_interval)))
    for index in range(half_window, len(samples) - half_window):
        before = samples[index - half_window]
        after = samples[index + half_window]
        delta_time = after["time"] - before["time"]
        if delta_time <= 0.0:
            continue
        derivative = tuple(
            (after["ecef"][axis] - before["ecef"][axis]) / delta_time
            for axis in range(3)
        )
        velocity_errors.append(
            norm3(subtract3(derivative, samples[index]["velocity_ecef"]))
        )
    return {
        "samples": len(samples),
        "start_utc": times[0],
        "end_utc": times[-1],
        "interval_s": describe(intervals),
        "nonpositive_intervals": sum(value <= 0.0 for value in intervals),
        "quality_counts": dict(sorted(Counter(
            str(sample["quality"]) for sample in samples
        ).items())),
        "lla_to_ecef_residual_m": describe(lla_ecef_errors),
        "ecef_velocity_consistency_m_s": describe(velocity_errors),
        "horizontal_speed_m_s": describe(horizontal_speeds),
        "course_minus_heading_abs_deg_at_speed_ge_2": describe(
            course_heading_errors
        ),
        "roll_deg": describe(sample["roll"] for sample in samples),
        "pitch_deg": describe(sample["pitch"] for sample in samples),
        "heading_deg": describe(sample["heading"] for sample in samples),
    }


def sample_point_field(message, field_name, maximum_samples):
    field = next((item for item in message.fields if item.name == field_name), None)
    if field is None or field.datatype not in POINT_FIELD_FORMATS or field.count < 1:
        return []
    total = int(message.width) * int(message.height)
    if total <= 0:
        return []
    sample_count = min(total, max(1, int(maximum_samples)))
    if sample_count == 1:
        indices = [0]
    else:
        indices = [round(i * (total - 1) / (sample_count - 1))
                   for i in range(sample_count)]
    endian = ">" if message.is_bigendian else "<"
    fmt = endian + POINT_FIELD_FORMATS[field.datatype]
    values = []
    for linear_index in indices:
        row = linear_index // int(message.width)
        column = linear_index % int(message.width)
        offset = row * int(message.row_step) + column * int(message.point_step) + int(field.offset)
        try:
            value = struct.unpack_from(fmt, message.data, offset)[0]
        except (struct.error, TypeError):
            continue
        if math.isfinite(float(value)):
            values.append(float(value))
    return values


def read_lidar_frames(path, topic, gt_times, calibration_lag, frame_step, point_samples):
    try:
        import rosbag
    except ImportError as exc:
        raise RuntimeError("rosbag is required; run inside the ROS1 container") from exc

    frames = []
    frame_ids = Counter()
    field_layouts = Counter()
    with rosbag.Bag(path, "r") as bag:
        for frame_index, (_, message, bag_stamp) in enumerate(
            bag.read_messages(topics=[topic])
        ):
            bag_time = bag_stamp.to_sec()
            header_time = message.header.stamp.to_sec()
            _, raw_dt = nearest_time(gt_times, header_time)
            # The calibration report names a signed "IMU to LiDAR" lag but
            # does not define the algebraic convention.  Preserve both
            # candidates until map sharpness provides an empirical check.
            pose_time_minus_lag = header_time - calibration_lag
            pose_time_plus_lag = header_time + calibration_lag
            _, minus_dt = nearest_time(gt_times, pose_time_minus_lag)
            _, plus_dt = nearest_time(gt_times, pose_time_plus_lag)
            point_times = []
            if frame_index % max(1, frame_step) == 0:
                point_times = sample_point_field(message, "time", point_samples)
            point_min = min(point_times) if point_times else None
            point_max = max(point_times) if point_times else None
            frame_ids[str(message.header.frame_id)] += 1
            layout = ",".join(
                "%s:%d:%d:%d" % (field.name, field.offset, field.datatype, field.count)
                for field in message.fields
            )
            field_layouts[layout] += 1
            frames.append({
                "frame_index": frame_index,
                "bag_time": bag_time,
                "header_time": header_time,
                "bag_minus_header_s": bag_time - header_time,
                "nearest_gt_minus_header_s": raw_dt,
                "pose_time_header_minus_lag": pose_time_minus_lag,
                "nearest_gt_minus_pose_minus_lag_s": minus_dt,
                "pose_time_header_plus_lag": pose_time_plus_lag,
                "nearest_gt_minus_pose_plus_lag_s": plus_dt,
                "width": int(message.width),
                "height": int(message.height),
                "frame_id": str(message.header.frame_id),
                "point_time_sample_n": len(point_times),
                "point_time_min_s": point_min,
                "point_time_max_s": point_max,
                "point_time_span_s": (
                    point_max - point_min if point_times else None
                ),
            })
    if not frames:
        raise ValueError("No LiDAR frames found on topic %s" % topic)
    return frames, frame_ids, field_layouts


def audit_lidar(frames, gt_times, gt_tolerance):
    header_times = [frame["header_time"] for frame in frames]
    periods = [header_times[i] - header_times[i - 1]
               for i in range(1, len(header_times))]
    raw_dt = [abs(frame["nearest_gt_minus_header_s"]) for frame in frames]
    minus_dt = [abs(frame["nearest_gt_minus_pose_minus_lag_s"]) for frame in frames]
    plus_dt = [abs(frame["nearest_gt_minus_pose_plus_lag_s"]) for frame in frames]
    sampled = [frame for frame in frames if frame["point_time_sample_n"]]
    scan_starts = [
        frame["header_time"] + frame["point_time_min_s"] for frame in sampled
    ]
    scan_ends = [
        frame["header_time"] + frame["point_time_max_s"] for frame in sampled
    ]
    inside_gt = sum(
        gt_times[0] <= frame["header_time"] <= gt_times[-1] for frame in frames
    )
    scan_inside_gt = sum(
        gt_times[0] <= start and end <= gt_times[-1]
        for start, end in zip(scan_starts, scan_ends)
    )
    return {
        "frames": len(frames),
        "header_start_utc": header_times[0],
        "header_end_utc": header_times[-1],
        "header_period_s": describe(periods),
        "nonpositive_header_intervals": sum(value <= 0.0 for value in periods),
        "bag_minus_header_s": describe(
            frame["bag_minus_header_s"] for frame in frames
        ),
        "nearest_gt_abs_dt_raw_header_s": describe(raw_dt),
        "nearest_gt_abs_dt_header_minus_lag_s": describe(minus_dt),
        "nearest_gt_abs_dt_header_plus_lag_s": describe(plus_dt),
        "raw_header_within_gt_tolerance": sum(value <= gt_tolerance for value in raw_dt),
        "raw_header_within_gt_tolerance_ratio": sum(
            value <= gt_tolerance for value in raw_dt
        ) / len(frames),
        "header_inside_gt_coverage": inside_gt,
        "header_inside_gt_coverage_ratio": inside_gt / len(frames),
        "point_time_sampled_frames": len(sampled),
        "point_time_min_s": describe(frame["point_time_min_s"] for frame in sampled),
        "point_time_max_s": describe(frame["point_time_max_s"] for frame in sampled),
        "point_time_span_s": describe(frame["point_time_span_s"] for frame in sampled),
        "sampled_scans_inside_gt_coverage": scan_inside_gt,
        "sampled_scans_inside_gt_coverage_ratio": (
            scan_inside_gt / len(sampled) if sampled else None
        ),
        "point_count": describe(frame["width"] * frame["height"] for frame in frames),
    }


def write_frames(path, frames):
    fields = list(frames[0].keys())
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(frames)


def evaluate_decision(gt, calibration, lidar):
    blockers = []
    metrics = calibration["metrics"]
    if abs(metrics["determinant"] - 1.0) > 0.005:
        blockers.append("calibration_rotation_determinant")
    if metrics["orthogonality_max_error"] > 0.005:
        blockers.append("calibration_rotation_orthogonality")
    if metrics["bottom_row_max_error"] > 1e-6:
        blockers.append("calibration_homogeneous_bottom_row")
    if gt["nonpositive_intervals"]:
        blockers.append("gt_nonmonotonic_time")
    if gt["lla_to_ecef_residual_m"]["p95"] > 1.0:
        blockers.append("gt_lla_ecef_inconsistent")
    if lidar["nonpositive_header_intervals"]:
        blockers.append("lidar_nonmonotonic_time")
    if lidar["header_inside_gt_coverage_ratio"] < 1.0:
        blockers.append("lidar_outside_gt_coverage")
    if lidar["raw_header_within_gt_tolerance_ratio"] < 0.999:
        blockers.append("lidar_gt_nearest_time_residual")
    period = lidar["header_period_s"]["median"]
    if period is None or not 0.08 <= period <= 0.12:
        blockers.append("unexpected_lidar_rate")
    span = lidar["point_time_span_s"]["median"]
    if span is None or not 0.07 <= span <= 0.13:
        blockers.append("unexpected_or_missing_point_time_span")
    return ("PASS" if not blockers else "REVIEW"), blockers


def fmt(value, digits=6):
    return "n/a" if value is None else ("%%.%df" % digits) % value


def main():
    args = parse_args()
    if args.point_time_frame_step <= 0 or args.point_time_samples <= 0:
        raise ValueError("Point-time sampling arguments must be positive")

    print("Reading calibration...")
    calibration = parse_calibration(args.calibration)
    print("Reading ground truth...")
    gt_samples = read_ground_truth(args.gt)
    gt_times = [sample["time"] for sample in gt_samples]
    gt_audit = audit_ground_truth(gt_samples)
    print("Reading LiDAR frame metadata...")
    frames, frame_ids, field_layouts = read_lidar_frames(
        args.bag,
        args.topic,
        gt_times,
        calibration["time_lag_imu_to_lidar_s"],
        args.point_time_frame_step,
        args.point_time_samples,
    )
    lidar_audit = audit_lidar(frames, gt_times, args.gt_tolerance)
    decision, blockers = evaluate_decision(gt_audit, calibration, lidar_audit)
    write_frames(args.out_frames, frames)

    audit = {
        "schema_version": 1,
        "decision": decision,
        "decision_scope": "time_and_calibration_input_audit",
        "blockers": blockers,
        "important_limit": (
            "PASS does not select the signed lag convention or prove the GT IMU/body "
            "world-frame rotation; validate both with static-map consistency next."
        ),
        "inputs": {
            "bag": args.bag,
            "gt": args.gt,
            "calibration": args.calibration,
            "topic": args.topic,
        },
        "ground_truth": gt_audit,
        "calibration": calibration,
        "lidar": lidar_audit,
        "frame_ids": dict(sorted(frame_ids.items())),
        "point_field_layouts": dict(sorted(field_layouts.items())),
    }
    with open(args.audit, "w", encoding="utf-8") as handle:
        json.dump(audit, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write("\n")

    gt_dt = gt_audit["interval_s"]
    ecef = gt_audit["lla_to_ecef_residual_m"]
    velocity = gt_audit["ecef_velocity_consistency_m_s"]
    course = gt_audit["course_minus_heading_abs_deg_at_speed_ge_2"]
    matrix_metrics = calibration["metrics"]
    header_dt = lidar_audit["header_period_s"]
    raw = lidar_audit["nearest_gt_abs_dt_raw_header_s"]
    lag_minus = lidar_audit["nearest_gt_abs_dt_header_minus_lag_s"]
    lag_plus = lidar_audit["nearest_gt_abs_dt_header_plus_lag_s"]
    delay = lidar_audit["bag_minus_header_s"]
    span = lidar_audit["point_time_span_s"]

    print("\n=== UrbanV2X LiDAR-GT input audit ===")
    print("GT samples:                  %d" % gt_audit["samples"])
    print("GT interval median/max:      %s / %s s" % (
        fmt(gt_dt["median"]), fmt(gt_dt["max"])))
    print("GT nonpositive intervals:    %d" % gt_audit["nonpositive_intervals"])
    print("LLA-ECEF residual p95/max:   %s / %s m" % (
        fmt(ecef["p95"], 4), fmt(ecef["max"], 4)))
    print("ECEF velocity error p95:     %s m/s" % fmt(velocity["p95"], 4))
    print("course-heading abs med/p95:  %s / %s deg (speed >= 2 m/s)" % (
        fmt(course["median"], 3), fmt(course["p95"], 3)))
    print("GT quality counts:           %s" % gt_audit["quality_counts"])
    print("\nCalibration section:         %s" % calibration["source_section"])
    print("transform direction:         LiDAR -> IMU")
    print("IMU-to-LiDAR signed lag:     %+0.6f s" % calibration["time_lag_imu_to_lidar_s"])
    print("rotation determinant:        %.9f" % matrix_metrics["determinant"])
    print("rotation orthogonality err:  %.3e" % matrix_metrics["orthogonality_max_error"])
    print("translation norm:            %.4f m" % matrix_metrics["translation_norm_m"])
    print("\nLiDAR frames:                %d" % lidar_audit["frames"])
    print("frame IDs:                   %s" % dict(frame_ids))
    print("header interval med/p95/max: %s / %s / %s s" % (
        fmt(header_dt["median"]), fmt(header_dt["p95"]), fmt(header_dt["max"])))
    print("bag-header delay med/p95:    %s / %s s" % (
        fmt(delay["median"]), fmt(delay["p95"])))
    print("nearest GT |dt| raw p95/max: %s / %s s" % (
        fmt(raw["p95"]), fmt(raw["max"])))
    print("nearest GT |dt| h-lag p95:   %s s" % fmt(lag_minus["p95"]))
    print("nearest GT |dt| h+lag p95:   %s s" % fmt(lag_plus["p95"]))
    print("raw within tolerance:        %d/%d (%.2f%%)" % (
        lidar_audit["raw_header_within_gt_tolerance"], lidar_audit["frames"],
        100.0 * lidar_audit["raw_header_within_gt_tolerance_ratio"]))
    print("sampled point-time frames:   %d" % lidar_audit["point_time_sampled_frames"])
    print("point-time span med/p95:     %s / %s s" % (
        fmt(span["median"]), fmt(span["p95"])))
    print("decision:                    %s" % decision)
    print("blockers:                    %s" % (blockers or "NONE"))
    print("\nFrame CSV: %s" % args.out_frames)
    print("Audit JSON: %s" % args.audit)
    print("Note: lag sign and GT body/world rotation remain for map-consistency testing.")


if __name__ == "__main__":
    main()
