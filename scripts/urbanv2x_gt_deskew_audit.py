#!/usr/bin/env python3
"""Audit UrbanV2X point-level GT motion compensation across route segments.

The existing ``urbanv2x_map_consistency.py`` already implements the core
LiDAR-point transformation using ``header.stamp + point.time``.  This script
reuses that implementation over several automatically selected motion regimes
and adds a frame-rigid control in which every point uses only the frame header
time.  It compares three signed-lag hypotheses against that control without
writing a production map.

The output is a relative motion-compensation audit.  It does not resolve the
undocumented Xsens-to-GT lever arm and does not authorize GNSS ray casting.
"""

import argparse
import csv
import json
import math
import os
import statistics
import sys

import numpy as np


SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIRECTORY not in sys.path:
    sys.path.insert(0, SCRIPT_DIRECTORY)

from urbanv2x_lidar_gt_audit import parse_calibration, read_ground_truth  # noqa: E402
from urbanv2x_map_consistency import (  # noqa: E402
    PoseInterpolator,
    cross_frame_overlap,
    ground_thickness,
    imu_flu_to_enu,
    planar_cell_residual,
    read_frame_bounds,
    read_segment_frames,
    voxel_downsample,
)


METRICS = (
    "map_voxels_per_1000_points",
    "ground_z_iqr_median_m",
    "planar_residual_median_m",
    "overlap_nearest_median_m",
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Multi-segment UrbanV2X point-level GT deskew audit"
    )
    parser.add_argument("--bag", required=True)
    parser.add_argument("--gt", required=True)
    parser.add_argument("--calibration", required=True)
    parser.add_argument("--frames-audit", required=True)
    parser.add_argument("--topic", default="/velodyne_points")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--segments", type=int, default=6)
    parser.add_argument("--point-stride", type=int, default=20)
    parser.add_argument("--min-range", type=float, default=2.0)
    parser.add_argument("--max-range", type=float, default=50.0)
    parser.add_argument("--map-voxel", type=float, default=0.15)
    parser.add_argument(
        "--preference-margin", type=float, default=0.01,
        help="Minimum aggregate-score improvement for a preference",
    )
    parser.add_argument("--out-details", required=True)
    parser.add_argument("--out-summary", required=True)
    parser.add_argument("--audit", required=True)
    parser.add_argument("--out-plot")
    return parser.parse_args()


def percentile(values, probability):
    values = [float(value) for value in values if value is not None]
    if not values:
        return None
    return float(np.percentile(values, 100.0 * probability))


def safe_median(values):
    values = [float(value) for value in values if value is not None]
    return statistics.median(values) if values else None


def _motion_arrays(samples):
    times = np.asarray([sample["time"] for sample in samples], dtype=np.float64)
    velocity = np.asarray(
        [sample["velocity_enu"][:2] for sample in samples], dtype=np.float64
    )
    speed = np.linalg.norm(velocity, axis=1)
    heading = np.unwrap(np.radians(
        [sample["heading"] for sample in samples]
    ))
    yaw_rate = np.zeros_like(heading)
    dt = np.diff(times)
    valid = dt > 0.0
    yaw_rate[1:][valid] = np.abs(np.degrees(np.diff(heading)[valid] / dt[valid]))
    quality = np.asarray([sample["quality"] for sample in samples], dtype=np.int32)
    return times, speed, yaw_rate, quality


def candidate_motion_windows(samples, lidar_start, lidar_end, duration):
    """Build deterministic non-overlapping windows and their motion metadata."""
    if duration <= 2.0 or lidar_end - lidar_start < duration:
        raise ValueError("Invalid duration or insufficient LiDAR coverage")
    times, speed, yaw_rate, quality = _motion_arrays(samples)
    windows = []
    start = math.ceil(lidar_start * 10.0) / 10.0
    while start + duration <= lidar_end + 1e-9:
        mask = (times >= start) & (times <= start + duration)
        if np.count_nonzero(mask) >= max(10, int(duration * 50)):
            q_values, q_counts = np.unique(quality[mask], return_counts=True)
            quality_mode = int(q_values[int(np.argmax(q_counts))])
            windows.append({
                "start_utc": float(start),
                "end_utc": float(start + duration),
                "median_speed_m_s": float(np.median(speed[mask])),
                "yaw_rate_p75_deg_s": float(np.percentile(yaw_rate[mask], 75)),
                "moving_ratio": float(np.mean(speed[mask] >= 2.0)),
                "gt_quality_mode": quality_mode,
                "gt_quality_mode_ratio": float(np.max(q_counts) / np.sum(q_counts)),
            })
        start += duration
    if not windows:
        raise ValueError("No eligible motion windows inside LiDAR coverage")
    return windows


def select_representative_windows(windows, count):
    """Select route coverage plus turn, fast-straight, and low-motion regimes."""
    if count <= 0:
        raise ValueError("Segment count must be positive")
    if len(windows) <= count:
        selected = [dict(row) for row in windows]
        for row in selected:
            row["selection_role"] = "all_available"
        return selected

    selected = {}

    def add(index, role):
        if index not in selected:
            selected[index] = dict(windows[index])
            selected[index]["selection_role"] = role

    turn_index = max(
        range(len(windows)), key=lambda index: windows[index]["yaw_rate_p75_deg_s"]
    )
    add(turn_index, "high_turn")

    yaw_median = statistics.median(
        row["yaw_rate_p75_deg_s"] for row in windows
    )
    straight = [
        index for index, row in enumerate(windows)
        if row["yaw_rate_p75_deg_s"] <= yaw_median
    ]
    fast_index = max(straight, key=lambda index: windows[index]["median_speed_m_s"])
    add(fast_index, "fast_straight")

    low_index = min(
        range(len(windows)), key=lambda index: windows[index]["median_speed_m_s"]
    )
    add(low_index, "low_motion")

    route_indices = np.linspace(0, len(windows) - 1, count, dtype=int)
    for index in route_indices:
        if len(selected) >= count:
            break
        add(int(index), "route_coverage")
    if len(selected) < count:
        for index in range(len(windows)):
            if len(selected) >= count:
                break
            add(index, "route_fill")
    return [selected[index] for index in sorted(selected)]


def transform_frame_candidate(frame, pose, lidar_to_imu, pose_time_offset,
                              pointwise):
    """Transform a sampled frame with point epochs or one frame-header epoch."""
    rotation = lidar_to_imu[:3, :3]
    translation = lidar_to_imu[:3, 3]
    imu_points = frame["xyz"] @ rotation.T + translation
    if pointwise:
        epochs = (
            frame["header_time"] + frame["relative_time"] + pose_time_offset
        )
    else:
        epochs = np.full(
            len(imu_points), frame["header_time"] + pose_time_offset,
            dtype=np.float64,
        )
    position, roll, pitch, heading = pose.interpolate(epochs)
    world = imu_flu_to_enu(imu_points, roll, pitch, heading) + position
    return {
        "xyz": world.astype(np.float32),
        "intensity": frame["intensity"],
        "local_z": frame["xyz"][:, 2].astype(np.float32),
    }


def evaluate_candidate(segment_index, window, name, pointwise, offset, frames,
                       pose, matrix, map_voxel):
    transformed = [
        transform_frame_candidate(frame, pose, matrix, offset, pointwise)
        for frame in frames
    ]
    xyz = np.concatenate([frame["xyz"] for frame in transformed])
    intensity = np.concatenate([frame["intensity"] for frame in transformed])
    local_z = np.concatenate([frame["local_z"] for frame in transformed])
    voxel_xyz, _, _, counts = voxel_downsample(xyz, intensity, map_voxel)
    ground, ground_cells = ground_thickness(xyz, local_z)
    plane, plane_cells = planar_cell_residual(voxel_xyz)
    overlap = cross_frame_overlap(transformed)
    return {
        "segment_index": int(segment_index),
        "selection_role": window["selection_role"],
        "start_utc": window["start_utc"],
        "start_elapsed_s": window["start_elapsed_s"],
        "duration_s": window["end_utc"] - window["start_utc"],
        "median_speed_m_s": window["median_speed_m_s"],
        "yaw_rate_p75_deg_s": window["yaw_rate_p75_deg_s"],
        "gt_quality_mode": window["gt_quality_mode"],
        "gt_quality_mode_ratio": window["gt_quality_mode_ratio"],
        "candidate": name,
        "pointwise": bool(pointwise),
        "pose_time_offset_s": float(offset),
        "frames": len(frames),
        "input_points": int(len(xyz)),
        "map_voxels": int(len(voxel_xyz)),
        "map_voxels_per_1000_points": float(1000.0 * len(voxel_xyz) / len(xyz)),
        "mean_points_per_map_voxel": float(np.mean(counts)),
        "ground_cells": int(ground_cells),
        "ground_z_iqr_median_m": ground["median"],
        "ground_z_iqr_p95_m": ground["p95"],
        "planar_cells": int(plane_cells),
        "planar_residual_median_m": plane["median"],
        "planar_residual_p95_m": plane["p95"],
        "overlap_nearest_median_m": overlap["median"],
        "overlap_nearest_p95_m": overlap["p95"],
    }


def aggregate_candidate_scores(rows, metrics=METRICS):
    """Normalize lower-is-better metrics within each segment and aggregate."""
    candidates = sorted(set(row["candidate"] for row in rows))
    segments = sorted(set(int(row["segment_index"]) for row in rows))
    accum = {name: [] for name in candidates}
    wins = {name: 0 for name in candidates}
    usable = []
    for segment in segments:
        selected = [row for row in rows if int(row["segment_index"]) == segment]
        by_name = {row["candidate"]: row for row in selected}
        if set(by_name) != set(candidates):
            raise ValueError("Every segment must contain every candidate")
        for metric in metrics:
            values = {name: by_name[name].get(metric) for name in candidates}
            if any(value is None or not math.isfinite(float(value)) or value <= 0.0
                   for value in values.values()):
                continue
            best = min(float(value) for value in values.values())
            winner = min(candidates, key=lambda name: float(values[name]))
            wins[winner] += 1
            usable.append((segment, metric))
            for name in candidates:
                accum[name].append(float(values[name]) / best)
    if not usable:
        raise ValueError("No common finite metrics for candidate ranking")
    summary = []
    for name in candidates:
        candidate_rows = [row for row in rows if row["candidate"] == name]
        summary.append({
            "candidate": name,
            "segments": len(candidate_rows),
            "normalized_score_mean": float(np.mean(accum[name])),
            "normalized_score_median": float(np.median(accum[name])),
            "metric_wins": int(wins[name]),
            **{
                metric + "_across_segments_median": safe_median(
                    row.get(metric) for row in candidate_rows
                )
                for metric in metrics
            },
        })
    summary.sort(key=lambda row: row["normalized_score_mean"])
    return summary, usable


def decide(summary, preference_margin):
    by_name = {row["candidate"]: row for row in summary}
    point_rows = sorted(
        [row for row in summary if row["candidate"].startswith("point_")],
        key=lambda row: row["normalized_score_mean"],
    )
    if len(point_rows) != 3 or "frame_raw" not in by_name:
        raise ValueError("Expected three pointwise candidates and frame_raw")
    best_point = point_rows[0]
    second_point = point_rows[1]
    frame = by_name["frame_raw"]
    deskew_improvement = (
        frame["normalized_score_mean"] - best_point["normalized_score_mean"]
    ) / frame["normalized_score_mean"]
    lag_margin = (
        second_point["normalized_score_mean"] - best_point["normalized_score_mean"]
    ) / best_point["normalized_score_mean"]
    deskew_decision = (
        "POINTWISE_SUPPORTED" if deskew_improvement >= preference_margin
        else "POINTWISE_INCONCLUSIVE"
    )
    if lag_margin >= preference_margin:
        lag_decision = "PREFERRED"
        preferred_lag = best_point["candidate"]
    else:
        lag_decision = "INDISTINGUISHABLE"
        preferred_lag = None
    return {
        "deskew_decision": deskew_decision,
        "deskew_score_improvement_fraction": float(deskew_improvement),
        "best_point_candidate": best_point["candidate"],
        "lag_decision": lag_decision,
        "preferred_lag_candidate": preferred_lag,
        "lag_score_margin_fraction": float(lag_margin),
    }


def write_csv(path, rows):
    if not rows:
        raise ValueError("Cannot write empty CSV")
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fields = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_plot(path, rows):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False, "matplotlib_not_installed"
    candidates = sorted(set(row["candidate"] for row in rows))
    figure, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    for axis, metric in zip(axes.ravel(), METRICS):
        for candidate in candidates:
            selected = sorted(
                [row for row in rows if row["candidate"] == candidate],
                key=lambda row: row["start_elapsed_s"],
            )
            axis.plot(
                [row["start_elapsed_s"] for row in selected],
                [row[metric] for row in selected], marker="o", label=candidate,
            )
        axis.set_title(metric)
        axis.grid(True, alpha=0.3)
    axes[1, 0].set_xlabel("Elapsed time [s]")
    axes[1, 1].set_xlabel("Elapsed time [s]")
    axes[0, 0].legend(fontsize=8)
    figure.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    figure.savefig(path, dpi=160)
    plt.close(figure)
    return True, None


def fmt(value, digits=4):
    return "NA" if value is None else ("%%.%df" % digits) % value


def main():
    args = parse_args()
    if args.segments < 3 or args.point_stride <= 0 or args.map_voxel <= 0.0:
        raise ValueError("Need >=3 segments and positive stride/voxel")
    print("Reading GT, calibration, and LiDAR coverage...")
    samples = read_ground_truth(args.gt)
    calibration = parse_calibration(args.calibration)
    lidar_start, lidar_end = read_frame_bounds(args.frames_audit)
    candidates = candidate_motion_windows(
        samples, lidar_start, lidar_end, args.duration
    )
    windows = select_representative_windows(candidates, args.segments)
    for row in windows:
        row["start_elapsed_s"] = row["start_utc"] - lidar_start
    print("Selected %d representative %.1f-second segments:" % (
        len(windows), args.duration
    ))
    for index, row in enumerate(windows):
        print("  %d %.1fs role=%s speed=%.2f yaw_p75=%.2f quality=%d" % (
            index, row["start_elapsed_s"], row["selection_role"],
            row["median_speed_m_s"], row["yaw_rate_p75_deg_s"],
            row["gt_quality_mode"],
        ))

    pose = PoseInterpolator(samples)
    matrix = np.asarray(calibration["matrix"], dtype=np.float64)
    lag = float(calibration["time_lag_imu_to_lidar_s"])
    hypotheses = [
        ("point_minus_lag", True, -lag),
        ("point_raw", True, 0.0),
        ("point_plus_lag", True, lag),
        ("frame_raw", False, 0.0),
    ]
    rows = []
    for segment_index, window in enumerate(windows):
        print("\nExtracting segment %d at %.1fs..." % (
            segment_index, window["start_elapsed_s"]
        ))
        frames = read_segment_frames(
            args.bag, args.topic, window["start_utc"], window["end_utc"],
            args.point_stride, args.min_range, args.max_range,
        )
        print("  frames=%d sampled_points=%d" % (
            len(frames), sum(len(frame["xyz"]) for frame in frames)
        ))
        for name, pointwise, offset in hypotheses:
            result = evaluate_candidate(
                segment_index, window, name, pointwise, offset, frames,
                pose, matrix, args.map_voxel,
            )
            rows.append(result)
            print("  %-16s plane=%s ground=%s overlap=%s vox/1000=%s" % (
                name, fmt(result["planar_residual_median_m"]),
                fmt(result["ground_z_iqr_median_m"]),
                fmt(result["overlap_nearest_median_m"]),
                fmt(result["map_voxels_per_1000_points"], 2),
            ))

    summary, usable_metrics = aggregate_candidate_scores(rows)
    decision = decide(summary, args.preference_margin)
    write_csv(args.out_details, rows)
    write_csv(args.out_summary, summary)
    plot_written = False
    plot_error = None
    if args.out_plot:
        plot_written, plot_error = write_plot(args.out_plot, rows)
    audit = {
        "schema_version": 1,
        **decision,
        "decision_scope": "relative_GT_point_deskew_audit_not_raycast_authorization",
        "inputs": {
            "bag": os.path.abspath(args.bag),
            "gt": os.path.abspath(args.gt),
            "calibration": os.path.abspath(args.calibration),
            "frames_audit": os.path.abspath(args.frames_audit),
        },
        "parameters": {
            "duration_s": args.duration,
            "segments_requested": args.segments,
            "segments_evaluated": len(windows),
            "point_stride": args.point_stride,
            "min_range_m": args.min_range,
            "max_range_m": args.max_range,
            "map_voxel_m": args.map_voxel,
            "preference_margin_fraction": args.preference_margin,
            "calibration_lag_imu_to_lidar_s": lag,
        },
        "usable_segment_metrics": [
            {"segment_index": segment, "metric": metric}
            for segment, metric in usable_metrics
        ],
        "selected_windows": windows,
        "summary": summary,
        "details": rows,
        "limitations": [
            "The GT origin is treated as the Xsens IMU origin because their lever arm is undocumented",
            "The frame-rigid control isolates point-time compensation but shares the same GT and extrinsic assumptions",
            "Dynamic objects and changing view geometry affect all map sharpness metrics",
            "A preferred lag is a dataset-processing result, not a calibrated material property",
            "Passing this relative audit does not by itself authorize GNSS ray casting",
        ],
        "outputs": {
            "details_csv": os.path.abspath(args.out_details),
            "summary_csv": os.path.abspath(args.out_summary),
            "plot": os.path.abspath(args.out_plot) if args.out_plot else None,
            "plot_written": plot_written,
            "plot_error": plot_error,
        },
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.audit)), exist_ok=True)
    with open(args.audit, "w", encoding="utf-8") as handle:
        json.dump(audit, handle, indent=2, sort_keys=True)
        handle.write("\n")

    print("\n=== Multi-segment GT point-deskew audit ===")
    print("candidate,score_mean,score_median,wins,plane,ground,overlap")
    for row in summary:
        print("%s,%.6f,%.6f,%d,%s,%s,%s" % (
            row["candidate"], row["normalized_score_mean"],
            row["normalized_score_median"], row["metric_wins"],
            fmt(row["planar_residual_median_m_across_segments_median"]),
            fmt(row["ground_z_iqr_median_m_across_segments_median"]),
            fmt(row["overlap_nearest_median_m_across_segments_median"]),
        ))
    print("pointwise decision:       %s" % decision["deskew_decision"])
    print("pointwise improvement:    %+.2f%%" % (
        100.0 * decision["deskew_score_improvement_fraction"]
    ))
    print("best point candidate:     %s" % decision["best_point_candidate"])
    print("lag decision:             %s" % decision["lag_decision"])
    print("preferred lag candidate:  %s" % (
        decision["preferred_lag_candidate"] or "NONE"
    ))
    print("lag score margin:         %.2f%%" % (
        100.0 * decision["lag_score_margin_fraction"]
    ))
    print("Details CSV: %s" % args.out_details)
    print("Summary CSV: %s" % args.out_summary)
    print("Audit JSON: %s" % args.audit)
    if args.out_plot:
        print("Plot: %s (%s)" % (
            args.out_plot, "written" if plot_written else plot_error
        ))
    print("Caution: this is a relative deskew audit, not a final map-quality pass.")


if __name__ == "__main__":
    main()
