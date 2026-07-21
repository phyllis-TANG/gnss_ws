#!/usr/bin/env python3
"""Audit local FAST-LIO accuracy in overlapping UrbanV2X time windows.

Each window receives its own rigid SE(3) position alignment to GT, without a
scale factor. This separates locally useful LiDAR geometry from long-range
LIO drift. Ten-, twenty-, and thirty-second windows can be compared in one
run to choose a defensible local-submap duration.
"""

import argparse
import csv
import json
import math
import os
import sys

import numpy as np


SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIRECTORY not in sys.path:
    sys.path.insert(0, SCRIPT_DIRECTORY)

from urbanv2x_fastlio_gt_eval import (  # noqa: E402
    compute_rpe,
    describe,
    error_trend,
    estimate_body_axis_rotation,
    navigation_rotation,
    read_odometry,
    rigid_alignment,
    rotation_angle_degrees,
    trajectory_length,
)
from urbanv2x_lidar_gt_audit import read_ground_truth  # noqa: E402
from urbanv2x_map_consistency import PoseInterpolator  # noqa: E402


def parse_args():
    parser = argparse.ArgumentParser(
        description="Audit FAST-LIO in overlapping GT-aligned local windows"
    )
    parser.add_argument("--bag", required=True)
    parser.add_argument("--gt", required=True)
    parser.add_argument("--topic", default="/Odometry")
    parser.add_argument(
        "--window-seconds", type=float, nargs="+", default=[10.0, 20.0, 30.0]
    )
    parser.add_argument(
        "--step-seconds", type=float, default=10.0,
        help="Sliding-window start interval (default: 10 s)",
    )
    parser.add_argument(
        "--rpe-seconds", type=float, default=1.0,
        help="Local RPE horizon (default: 1 s)",
    )
    parser.add_argument("--out-windows", required=True)
    parser.add_argument("--out-summary", required=True)
    parser.add_argument("--audit", required=True)
    parser.add_argument("--out-plot")
    parser.add_argument("--ate-p95-pass", type=float, default=0.50)
    parser.add_argument("--rpe-translation-p95-pass", type=float, default=0.20)
    parser.add_argument("--rpe-rotation-p95-pass", type=float, default=1.00)
    parser.add_argument("--ate-p95-review", type=float, default=1.00)
    parser.add_argument("--rpe-translation-p95-review", type=float, default=0.50)
    parser.add_argument("--rpe-rotation-p95-review", type=float, default=2.00)
    parser.add_argument(
        "--minimum-poses", type=int, default=30,
        help="Minimum odometry poses per evaluated window",
    )
    return parser.parse_args()


def evaluate_window(times, odom_positions, odom_rotations,
                    gt_positions, gt_rotations, start_index, end_index,
                    requested_start, requested_end, rpe_horizon,
                    thresholds, minimum_poses):
    selection = slice(start_index, end_index)
    local_times = times[selection]
    local_odom_positions = odom_positions[selection]
    local_odom_rotations = odom_rotations[selection]
    local_gt_positions = gt_positions[selection]
    local_gt_rotations = gt_rotations[selection]
    count = len(local_times)
    base = {
        "requested_start_utc": float(requested_start),
        "requested_end_utc": float(requested_end),
        "window_s": float(requested_end - requested_start),
        "poses": int(count),
        "actual_start_utc": float(local_times[0]) if count else None,
        "actual_end_utc": float(local_times[-1]) if count else None,
        "actual_duration_s": (
            float(local_times[-1] - local_times[0]) if count >= 2 else None
        ),
    }
    if count < minimum_poses:
        return {
            **base,
            "decision": "INSUFFICIENT",
            "reason": "too_few_poses",
        }

    alignment_rotation, alignment_translation, singular_values = rigid_alignment(
        local_odom_positions, local_gt_positions
    )
    aligned_positions = (
        (alignment_rotation @ local_odom_positions.T).T + alignment_translation
    )
    aligned_rotations = np.asarray([
        alignment_rotation @ rotation for rotation in local_odom_rotations
    ])
    body_axis_rotation = estimate_body_axis_rotation(
        aligned_rotations, local_gt_rotations
    )
    corrected_rotations = np.asarray([
        rotation @ body_axis_rotation for rotation in aligned_rotations
    ])
    position_errors = np.linalg.norm(
        aligned_positions - local_gt_positions, axis=1
    )
    rotation_errors = np.asarray([
        rotation_angle_degrees(
            local_gt_rotations[index].T @ corrected_rotations[index]
        )
        for index in range(count)
    ])
    rpe = compute_rpe(
        local_times, aligned_positions, corrected_rotations,
        local_gt_positions, local_gt_rotations,
        rpe_horizon, tolerance=0.15,
    )
    ate = describe(position_errors, include_rmse=True)
    orientation = describe(rotation_errors, include_rmse=True)
    trend = error_trend(local_times - local_times[0], position_errors)
    odom_length = trajectory_length(local_odom_positions)
    gt_length = trajectory_length(local_gt_positions)
    path_ratio = odom_length / gt_length if gt_length > 0.0 else None
    rpe_translation_p95 = rpe["translation_error_m"]["p95"]
    rpe_rotation_p95 = rpe["rotation_error_deg"]["p95"]

    pass_checks = {
        "ate": ate["p95"] <= thresholds["ate_pass"],
        "translation": (
            rpe_translation_p95 is not None
            and rpe_translation_p95 <= thresholds["translation_pass"]
        ),
        "rotation": (
            rpe_rotation_p95 is not None
            and rpe_rotation_p95 <= thresholds["rotation_pass"]
        ),
    }
    review_checks = {
        "ate": ate["p95"] <= thresholds["ate_review"],
        "translation": (
            rpe_translation_p95 is not None
            and rpe_translation_p95 <= thresholds["translation_review"]
        ),
        "rotation": (
            rpe_rotation_p95 is not None
            and rpe_rotation_p95 <= thresholds["rotation_review"]
        ),
    }
    if all(pass_checks.values()):
        decision = "PASS"
        reason = "all_pass_thresholds"
    elif all(review_checks.values()):
        decision = "REVIEW"
        reason = "inside_review_thresholds"
    else:
        decision = "FAIL"
        reason = ";".join(
            name for name, passed in review_checks.items() if not passed
        )
    return {
        **base,
        "decision": decision,
        "reason": reason,
        "odom_path_m": odom_length,
        "gt_path_m": gt_length,
        "odom_over_gt_path_ratio": path_ratio,
        "ate_median_m": ate["median"],
        "ate_rmse_m": ate["rmse"],
        "ate_p95_m": ate["p95"],
        "ate_max_m": ate["max"],
        "ate_trend_m_s": trend["linear_slope_m_s"],
        "orientation_median_deg": orientation["median"],
        "orientation_p95_deg": orientation["p95"],
        "rpe_pairs": rpe["pairs"],
        "rpe_translation_p95_m": rpe_translation_p95,
        "rpe_rotation_p95_deg": rpe_rotation_p95,
        "alignment_singular_1": float(singular_values[0]),
        "alignment_singular_2": float(singular_values[1]),
        "alignment_singular_3": float(singular_values[2]),
        "body_axis_offset_deg": rotation_angle_degrees(body_axis_rotation),
    }


def build_windows(times, odom_positions, odom_rotations,
                  gt_positions, gt_rotations, durations, step,
                  rpe_horizon, thresholds, minimum_poses):
    rows = []
    first = float(times[0])
    last = float(times[-1])
    for duration in sorted(set(float(value) for value in durations)):
        start = first
        window_index = 0
        while start + duration <= last + 1e-6:
            end = start + duration
            start_index = int(np.searchsorted(times, start, side="left"))
            end_index = int(np.searchsorted(times, end, side="right"))
            row = evaluate_window(
                times, odom_positions, odom_rotations,
                gt_positions, gt_rotations,
                start_index, end_index, start, end,
                rpe_horizon, thresholds, minimum_poses,
            )
            row["window_index"] = window_index
            row["center_elapsed_s"] = start + duration / 2.0 - first
            rows.append(row)
            window_index += 1
            start += step
    return rows


def metric_distribution(rows, key):
    return describe([
        row.get(key) for row in rows if row.get(key) is not None
    ])


def summarize_windows(rows, durations):
    summaries = []
    for duration in sorted(set(float(value) for value in durations)):
        selected = [row for row in rows if abs(row["window_s"] - duration) < 1e-6]
        evaluated = [row for row in selected if row["decision"] != "INSUFFICIENT"]
        counts = {
            decision: sum(row["decision"] == decision for row in selected)
            for decision in ("PASS", "REVIEW", "FAIL", "INSUFFICIENT")
        }
        denominator = len(evaluated)
        summaries.append({
            "window_s": duration,
            "windows": len(selected),
            **{decision.lower(): count for decision, count in counts.items()},
            "pass_ratio": counts["PASS"] / denominator if denominator else None,
            "fail_ratio": counts["FAIL"] / denominator if denominator else None,
            "ate_p95_across_windows_median_m": metric_distribution(
                evaluated, "ate_p95_m"
            )["median"],
            "ate_p95_across_windows_p95_m": metric_distribution(
                evaluated, "ate_p95_m"
            )["p95"],
            "rpe_translation_p95_across_windows_median_m": metric_distribution(
                evaluated, "rpe_translation_p95_m"
            )["median"],
            "rpe_translation_p95_across_windows_p95_m": metric_distribution(
                evaluated, "rpe_translation_p95_m"
            )["p95"],
            "rpe_rotation_p95_across_windows_median_deg": metric_distribution(
                evaluated, "rpe_rotation_p95_deg"
            )["median"],
            "rpe_rotation_p95_across_windows_p95_deg": metric_distribution(
                evaluated, "rpe_rotation_p95_deg"
            )["p95"],
        })
    candidates = [
        row for row in summaries
        if row["pass_ratio"] is not None
        and row["pass_ratio"] >= 0.80
        and row["fail_ratio"] <= 0.10
    ]
    recommended = max(
        (row["window_s"] for row in candidates), default=None
    )
    return summaries, recommended


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


def write_plot(path, rows, thresholds):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False, "matplotlib_not_installed"
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    figure, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    for duration in sorted(set(row["window_s"] for row in rows)):
        selected = [
            row for row in rows
            if row["window_s"] == duration and row.get("ate_p95_m") is not None
        ]
        x = [row["center_elapsed_s"] for row in selected]
        label = "%gs window" % duration
        axes[0].plot(x, [row["ate_p95_m"] for row in selected], marker=".", label=label)
        axes[1].plot(
            x, [row["rpe_translation_p95_m"] for row in selected], marker=".", label=label
        )
        axes[2].plot(
            x, [row["rpe_rotation_p95_deg"] for row in selected], marker=".", label=label
        )
    axes[0].axhline(thresholds["ate_pass"], color="k", linestyle="--", alpha=0.5)
    axes[1].axhline(
        thresholds["translation_pass"], color="k", linestyle="--", alpha=0.5
    )
    axes[2].axhline(
        thresholds["rotation_pass"], color="k", linestyle="--", alpha=0.5
    )
    axes[0].set_ylabel("Local ATE p95 [m]")
    axes[1].set_ylabel("1 s translation RPE p95 [m]")
    axes[2].set_ylabel("1 s rotation RPE p95 [deg]")
    axes[2].set_xlabel("Window centre elapsed time [s]")
    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.legend()
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)
    return True, None


def fmt(value, digits=3):
    return "NA" if value is None else ("%%.%df" % digits) % value


def main():
    args = parse_args()
    durations = sorted(set(float(value) for value in args.window_seconds))
    if any(value <= args.rpe_seconds for value in durations):
        raise ValueError("Each window must be longer than the RPE horizon")
    if args.step_seconds <= 0.0 or args.rpe_seconds <= 0.0:
        raise ValueError("Step and RPE horizon must be positive")
    thresholds = {
        "ate_pass": args.ate_p95_pass,
        "translation_pass": args.rpe_translation_p95_pass,
        "rotation_pass": args.rpe_rotation_p95_pass,
        "ate_review": args.ate_p95_review,
        "translation_review": args.rpe_translation_p95_review,
        "rotation_review": args.rpe_rotation_p95_review,
    }

    print("Reading FAST-LIO odometry...")
    odometry, odometry_audit = read_odometry(args.bag, args.topic)
    print("  poses: %d" % len(odometry))
    print("Reading GT...")
    gt_samples = read_ground_truth(args.gt)
    gt_start = gt_samples[0]["time"]
    gt_end = gt_samples[-1]["time"]
    odometry = [
        row for row in odometry if gt_start <= row["time"] <= gt_end
    ]
    if len(odometry) < args.minimum_poses:
        raise ValueError("Too few odometry poses inside GT coverage")
    times = np.asarray([row["time"] for row in odometry])
    odom_positions = np.asarray([row["position"] for row in odometry])
    odom_rotations = np.asarray([row["rotation"] for row in odometry])
    pose = PoseInterpolator(gt_samples)
    gt_positions, roll, pitch, heading = pose.interpolate(times)
    gt_rotations = np.asarray([
        navigation_rotation(roll[index], pitch[index], heading[index])
        for index in range(len(times))
    ])

    print("Evaluating %s s windows every %.1f s..." % (
        "/".join("%g" % value for value in durations), args.step_seconds
    ))
    rows = build_windows(
        times, odom_positions, odom_rotations, gt_positions, gt_rotations,
        durations, args.step_seconds, args.rpe_seconds,
        thresholds, args.minimum_poses,
    )
    summaries, recommended = summarize_windows(rows, durations)
    write_csv(args.out_windows, rows)
    write_csv(args.out_summary, summaries)
    plot_written = False
    plot_error = None
    if args.out_plot:
        plot_written, plot_error = write_plot(args.out_plot, rows, thresholds)

    audit = {
        "schema_version": 1,
        "inputs": {
            "bag": os.path.abspath(args.bag),
            "gt": os.path.abspath(args.gt),
            "topic": args.topic,
        },
        "odometry": odometry_audit,
        "coverage": {
            "poses_inside_gt": len(odometry),
            "start_utc": float(times[0]),
            "end_utc": float(times[-1]),
            "duration_s": float(times[-1] - times[0]),
        },
        "parameters": {
            "window_seconds": durations,
            "step_seconds": args.step_seconds,
            "rpe_seconds": args.rpe_seconds,
            "minimum_poses": args.minimum_poses,
        },
        "thresholds_are_project_screens_not_field_standards": thresholds,
        "summaries": summaries,
        "recommended_largest_window_s": recommended,
        "recommendation_rule": "pass_ratio >= 0.80 and fail_ratio <= 0.10",
        "windows": rows,
        "limitations": [
            "Each window is independently rigid-aligned and cannot prove global accuracy",
            "Overlapping windows are statistically dependent",
            "The GT-to-Xsens lever arm remains undocumented",
            "A passing trajectory window still needs local point-cloud geometry auditing",
        ],
        "outputs": {
            "windows_csv": os.path.abspath(args.out_windows),
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

    print("\n=== FAST-LIO local-window audit ===")
    print("poses/duration: %d / %.3f s" % (
        len(odometry), times[-1] - times[0]
    ))
    print("window_s,n,pass,review,fail,pass_ratio,ATEp95_med,ATEp95_p95,RPEt_p95,RPEr_p95")
    for row in summaries:
        print("%g,%d,%d,%d,%d,%s,%s,%s,%s,%s" % (
            row["window_s"], row["windows"], row["pass"], row["review"],
            row["fail"], fmt(row["pass_ratio"]),
            fmt(row["ate_p95_across_windows_median_m"]),
            fmt(row["ate_p95_across_windows_p95_m"]),
            fmt(row["rpe_translation_p95_across_windows_p95_m"]),
            fmt(row["rpe_rotation_p95_across_windows_p95_deg"]),
        ))
    print("recommended largest window: %s s" % fmt(recommended, 1))
    failed = [row for row in rows if row["decision"] == "FAIL"]
    if failed:
        print("failed windows (start elapsed, duration, reason):")
        for row in failed:
            print("  %.1f s, %g s, %s" % (
                row["requested_start_utc"] - times[0], row["window_s"], row["reason"]
            ))
    print("Windows CSV: %s" % args.out_windows)
    print("Summary CSV: %s" % args.out_summary)
    print("Audit JSON: %s" % args.audit)
    if args.out_plot:
        print("Plot: %s (%s)" % (
            args.out_plot, "written" if plot_written else plot_error
        ))
    print("Caution: local alignment supports submap selection, not global-map claims.")


if __name__ == "__main__":
    main()
