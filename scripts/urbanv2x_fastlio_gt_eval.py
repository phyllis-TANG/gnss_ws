#!/usr/bin/env python3
"""Evaluate an UrbanV2X FAST-LIO trajectory against the supplied GT.

The FAST-LIO map frame is arbitrary, so positions are aligned to the local
GT ENU frame with a rigid transform (rotation + translation, never scale).
Absolute trajectory error (ATE) is reported after that alignment. Relative
pose error (RPE) is also reported at requested time horizons so a visually
plausible map cannot hide short-term motion error or drift.

UrbanV2X does not document a rigid lever arm from the GT reference point to
the Xsens IMU origin used by the LiDAR calibration. A constant offset is
absorbed by trajectory alignment, but a rotating lever arm can remain in the
residuals; the limitation is recorded in the audit output.
"""

import argparse
import bisect
import csv
import json
import math
import os
import statistics
import sys
from collections import Counter

import numpy as np


SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIRECTORY not in sys.path:
    sys.path.insert(0, SCRIPT_DIRECTORY)

from urbanv2x_lidar_gt_audit import read_ground_truth  # noqa: E402
from urbanv2x_map_consistency import PoseInterpolator  # noqa: E402


def parse_args():
    parser = argparse.ArgumentParser(
        description="Evaluate UrbanV2X FAST-LIO odometry against GT"
    )
    parser.add_argument("--bag", required=True, help="Bag containing FAST-LIO odometry")
    parser.add_argument("--gt", required=True, help="UrbanV2X gt_small_loop.txt")
    parser.add_argument("--topic", default="/Odometry")
    parser.add_argument("--out-csv", required=True, help="Per-epoch aligned error CSV")
    parser.add_argument("--audit", required=True, help="Summary audit JSON")
    parser.add_argument("--out-plot", help="Optional trajectory/error PNG")
    parser.add_argument(
        "--rpe-seconds", type=float, nargs="+", default=[1.0, 5.0],
        help="RPE horizons in seconds (default: 1 5)",
    )
    parser.add_argument(
        "--pair-tolerance", type=float, default=0.15,
        help="Maximum RPE target-time mismatch (default: 0.15 s)",
    )
    parser.add_argument(
        "--ate-p95-screen", type=float, default=0.5,
        help="Project screening threshold for ATE p95 in metres",
    )
    parser.add_argument(
        "--rpe1-translation-p95-screen", type=float, default=0.2,
        help="Project screening threshold for 1 s translation RPE p95 in metres",
    )
    parser.add_argument(
        "--rpe1-rotation-p95-screen", type=float, default=1.0,
        help="Project screening threshold for 1 s rotation RPE p95 in degrees",
    )
    return parser.parse_args()


def percentile(values, probability):
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return None
    return float(np.percentile(values, 100.0 * probability))


def describe(values, include_rmse=False):
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]
    if values.size == 0:
        result = {
            "n": 0, "min": None, "p05": None, "median": None,
            "mean": None, "p95": None, "max": None,
        }
    else:
        result = {
            "n": int(values.size),
            "min": float(np.min(values)),
            "p05": percentile(values, 0.05),
            "median": float(np.median(values)),
            "mean": float(np.mean(values)),
            "p95": percentile(values, 0.95),
            "max": float(np.max(values)),
        }
    if include_rmse:
        result["rmse"] = (
            float(np.sqrt(np.mean(values ** 2))) if values.size else None
        )
    return result


def quaternion_to_rotation(x, y, z, w):
    quaternion = np.asarray([x, y, z, w], dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid zero/non-finite odometry quaternion")
    x, y, z, w = quaternion / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ])


def navigation_rotation(roll, pitch, heading):
    """Return ENU-from-IMU-FLU using the audited UrbanV2X convention."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    yaw = math.pi / 2.0 - heading
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, -sp], [0.0, 1.0, 0.0], [sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def read_odometry(path, topic):
    try:
        import rosbag
    except ImportError as exc:
        raise RuntimeError("rosbag is required; run inside the ROS1 container") from exc

    rows = []
    frame_ids = Counter()
    child_frame_ids = Counter()
    quaternion_norms = []
    bag_minus_header = []
    with rosbag.Bag(path, "r") as bag:
        for _, message, bag_time in bag.read_messages(topics=[topic]):
            epoch = float(message.header.stamp.to_sec())
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            values = [
                epoch, position.x, position.y, position.z,
                orientation.x, orientation.y, orientation.z, orientation.w,
            ]
            if not all(math.isfinite(float(value)) for value in values):
                continue
            rotation = quaternion_to_rotation(
                orientation.x, orientation.y, orientation.z, orientation.w
            )
            rows.append({
                "time": epoch,
                "position": np.array(
                    [position.x, position.y, position.z], dtype=np.float64
                ),
                "rotation": rotation,
            })
            quaternion_norms.append(math.sqrt(
                orientation.x ** 2 + orientation.y ** 2
                + orientation.z ** 2 + orientation.w ** 2
            ))
            bag_minus_header.append(float(bag_time.to_sec()) - epoch)
            frame_ids[str(message.header.frame_id)] += 1
            child_frame_ids[str(message.child_frame_id)] += 1
    if not rows:
        raise ValueError("No finite odometry messages found on %s" % topic)
    rows.sort(key=lambda item: item["time"])
    unique = []
    duplicate_timestamps = 0
    for row in rows:
        if unique and abs(row["time"] - unique[-1]["time"]) < 1e-9:
            duplicate_timestamps += 1
            unique[-1] = row
        else:
            unique.append(row)
    return unique, {
        "input_messages": len(rows),
        "unique_messages": len(unique),
        "duplicate_timestamps": duplicate_timestamps,
        "frame_ids": dict(frame_ids),
        "child_frame_ids": dict(child_frame_ids),
        "quaternion_norm": describe(quaternion_norms),
        "bag_minus_header_s": describe(bag_minus_header),
    }


def rigid_alignment(source, target):
    """Return target ~= R @ source + t, without estimating scale."""
    source = np.asarray(source, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("Alignment inputs must be equal N x 3 arrays")
    if source.shape[0] < 3:
        raise ValueError("At least three positions are required for alignment")
    source_mean = np.mean(source, axis=0)
    target_mean = np.mean(target, axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean
    covariance = source_centered.T @ target_centered
    u, singular_values, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = target_mean - rotation @ source_mean
    return rotation, translation, singular_values


def rotation_angle_degrees(rotation):
    cosine = (float(np.trace(rotation)) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def nearest_residual(sorted_times, epoch):
    index = bisect.bisect_left(sorted_times, epoch)
    candidates = []
    if index < len(sorted_times):
        candidates.append(sorted_times[index] - epoch)
    if index > 0:
        candidates.append(sorted_times[index - 1] - epoch)
    return min(candidates, key=abs) if candidates else None


def trajectory_length(positions):
    positions = np.asarray(positions, dtype=np.float64)
    if positions.shape[0] < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1)))


def compute_rpe(times, estimated_positions, estimated_rotations,
                gt_positions, gt_rotations, horizon, tolerance):
    times = np.asarray(times, dtype=np.float64)
    translation_errors = []
    rotation_errors = []
    actual_horizons = []
    used_pairs = set()
    for start in range(len(times)):
        target = times[start] + horizon
        insertion = int(np.searchsorted(times, target))
        candidates = [index for index in (insertion - 1, insertion)
                      if start < index < len(times)]
        if not candidates:
            continue
        end = min(candidates, key=lambda index: abs(times[index] - target))
        if abs(times[end] - target) > tolerance or (start, end) in used_pairs:
            continue
        used_pairs.add((start, end))

        estimated_delta = (
            estimated_rotations[start].T
            @ (estimated_positions[end] - estimated_positions[start])
        )
        gt_delta = (
            gt_rotations[start].T @ (gt_positions[end] - gt_positions[start])
        )
        translation_errors.append(float(np.linalg.norm(estimated_delta - gt_delta)))

        estimated_relative_rotation = (
            estimated_rotations[start].T @ estimated_rotations[end]
        )
        gt_relative_rotation = gt_rotations[start].T @ gt_rotations[end]
        rotation_errors.append(rotation_angle_degrees(
            gt_relative_rotation.T @ estimated_relative_rotation
        ))
        actual_horizons.append(times[end] - times[start])
    return {
        "requested_horizon_s": float(horizon),
        "pairs": len(translation_errors),
        "actual_horizon_s": describe(actual_horizons),
        "translation_error_m": describe(translation_errors, include_rmse=True),
        "rotation_error_deg": describe(rotation_errors, include_rmse=True),
    }


def error_trend(elapsed, errors):
    elapsed = np.asarray(elapsed, dtype=np.float64)
    errors = np.asarray(errors, dtype=np.float64)
    if elapsed.size < 4 or elapsed[-1] <= elapsed[0]:
        return {"linear_slope_m_s": None, "first_quarter_median_m": None,
                "last_quarter_median_m": None, "last_minus_first_median_m": None}
    slope = float(np.polyfit(elapsed, errors, 1)[0])
    quarter = max(1, len(errors) // 4)
    first = float(np.median(errors[:quarter]))
    last = float(np.median(errors[-quarter:]))
    return {
        "linear_slope_m_s": slope,
        "first_quarter_median_m": first,
        "last_quarter_median_m": last,
        "last_minus_first_median_m": last - first,
    }


def write_csv(path, times, odom_positions, aligned_positions, gt_positions,
              position_errors, rotation_errors, nearest_gt_dt, gt_quality):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "epoch", "elapsed_s", "odom_x", "odom_y", "odom_z",
            "aligned_e", "aligned_n", "aligned_u",
            "gt_e", "gt_n", "gt_u", "error_e", "error_n", "error_u",
            "position_error_m", "rotation_error_deg", "nearest_gt_dt_s",
            "gt_quality",
        ])
        for index, epoch in enumerate(times):
            vector_error = aligned_positions[index] - gt_positions[index]
            writer.writerow([
                "%.9f" % epoch,
                "%.6f" % (epoch - times[0]),
                *("%.6f" % value for value in odom_positions[index]),
                *("%.6f" % value for value in aligned_positions[index]),
                *("%.6f" % value for value in gt_positions[index]),
                *("%.6f" % value for value in vector_error),
                "%.6f" % position_errors[index],
                "%.6f" % rotation_errors[index],
                "%.9f" % nearest_gt_dt[index],
                int(gt_quality[index]),
            ])


def write_plot(path, elapsed, aligned_positions, gt_positions,
               position_errors, rotation_errors):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False, "matplotlib_not_installed"

    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    figure, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    axes[0].plot(gt_positions[:, 0], gt_positions[:, 1], "k-", label="GT")
    axes[0].plot(
        aligned_positions[:, 0], aligned_positions[:, 1], "C0--", label="FAST-LIO"
    )
    axes[0].axis("equal")
    axes[0].set_xlabel("East [m]")
    axes[0].set_ylabel("North [m]")
    axes[0].set_title("Rigid-aligned trajectory")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(elapsed, position_errors, color="C1")
    axes[1].set_xlabel("Elapsed time [s]")
    axes[1].set_ylabel("Position error [m]")
    axes[1].set_title("ATE over time")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(elapsed, rotation_errors, color="C2")
    axes[2].set_xlabel("Elapsed time [s]")
    axes[2].set_ylabel("Rotation error [deg]")
    axes[2].set_title("Absolute orientation error")
    axes[2].grid(True, alpha=0.3)
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)
    return True, None


def fmt(value, digits=4):
    return "NA" if value is None else ("%%.%df" % digits) % value


def main():
    args = parse_args()
    print("Reading FAST-LIO odometry...")
    odometry, odometry_audit = read_odometry(args.bag, args.topic)
    print("  unique messages: %d" % len(odometry))
    print("Reading UrbanV2X GT...")
    gt_samples = read_ground_truth(args.gt)
    print("  GT samples: %d" % len(gt_samples))

    gt_times_all = np.asarray([sample["time"] for sample in gt_samples])
    in_coverage = [
        row for row in odometry
        if gt_times_all[0] <= row["time"] <= gt_times_all[-1]
    ]
    if len(in_coverage) < 3:
        raise ValueError("Fewer than three odometry poses fall inside GT coverage")

    times = np.asarray([row["time"] for row in in_coverage])
    odom_positions = np.asarray([row["position"] for row in in_coverage])
    odom_rotations = np.asarray([row["rotation"] for row in in_coverage])

    pose = PoseInterpolator(gt_samples)
    gt_positions, gt_roll, gt_pitch, gt_heading = pose.interpolate(times)
    gt_rotations = np.asarray([
        navigation_rotation(gt_roll[index], gt_pitch[index], gt_heading[index])
        for index in range(len(times))
    ])

    alignment_rotation, alignment_translation, singular_values = rigid_alignment(
        odom_positions, gt_positions
    )
    aligned_positions = (
        (alignment_rotation @ odom_positions.T).T + alignment_translation
    )
    aligned_rotations = np.asarray([
        alignment_rotation @ rotation for rotation in odom_rotations
    ])
    position_errors = np.linalg.norm(aligned_positions - gt_positions, axis=1)
    rotation_errors = np.asarray([
        rotation_angle_degrees(gt_rotations[index].T @ aligned_rotations[index])
        for index in range(len(times))
    ])

    nearest_gt_dt = np.asarray([
        nearest_residual(gt_times_all, epoch) for epoch in times
    ])
    nearest_indices = np.searchsorted(gt_times_all, times)
    nearest_indices = np.clip(nearest_indices, 0, len(gt_times_all) - 1)
    for index in range(len(times)):
        if nearest_indices[index] > 0:
            before = nearest_indices[index] - 1
            after = nearest_indices[index]
            if abs(gt_times_all[before] - times[index]) <= abs(
                gt_times_all[after] - times[index]
            ):
                nearest_indices[index] = before
    gt_quality = np.asarray([
        gt_samples[int(index)]["quality"] for index in nearest_indices
    ])

    rpe = {}
    for horizon in sorted(set(float(value) for value in args.rpe_seconds)):
        if horizon <= 0.0:
            raise ValueError("RPE horizons must be positive")
        key = ("%.3f" % horizon).rstrip("0").rstrip(".") + "s"
        rpe[key] = compute_rpe(
            times, aligned_positions, aligned_rotations,
            gt_positions, gt_rotations, horizon, args.pair_tolerance,
        )

    ate = describe(position_errors, include_rmse=True)
    absolute_rotation = describe(rotation_errors, include_rmse=True)
    elapsed = times - times[0]
    trend = error_trend(elapsed, position_errors)
    odom_length = trajectory_length(odom_positions)
    gt_length = trajectory_length(gt_positions)
    path_length_ratio = odom_length / gt_length if gt_length > 0.0 else None

    rpe_1s = min(
        rpe.values(),
        key=lambda item: abs(item["requested_horizon_s"] - 1.0),
    ) if rpe else None
    screening_checks = {
        "odometry_inside_gt_ratio_ge_0_99": len(in_coverage) / len(odometry) >= 0.99,
        "nearest_gt_abs_dt_p95_le_0_01_s": percentile(abs(nearest_gt_dt), 0.95) <= 0.01,
        "ate_p95_le_project_threshold": ate["p95"] <= args.ate_p95_screen,
        "rpe_near_1s_available": (
            rpe_1s is not None
            and abs(rpe_1s["requested_horizon_s"] - 1.0) <= 0.25
            and rpe_1s["pairs"] > 0
        ),
    }
    if screening_checks["rpe_near_1s_available"]:
        screening_checks["rpe_1s_translation_p95_le_project_threshold"] = (
            rpe_1s["translation_error_m"]["p95"]
            <= args.rpe1_translation_p95_screen
        )
        screening_checks["rpe_1s_rotation_p95_le_project_threshold"] = (
            rpe_1s["rotation_error_deg"]["p95"]
            <= args.rpe1_rotation_p95_screen
        )
    screening_decision = (
        "SCREEN_PASS" if all(screening_checks.values()) else "REVIEW"
    )

    plot_written = False
    plot_error = None
    if args.out_plot:
        plot_written, plot_error = write_plot(
            args.out_plot, elapsed, aligned_positions, gt_positions,
            position_errors, rotation_errors,
        )

    audit = {
        "inputs": {
            "bag": os.path.abspath(args.bag),
            "gt": os.path.abspath(args.gt),
            "topic": args.topic,
        },
        "odometry": odometry_audit,
        "matching": {
            "odometry_total": len(odometry),
            "inside_gt_coverage": len(in_coverage),
            "inside_gt_ratio": len(in_coverage) / len(odometry),
            "start_utc": float(times[0]),
            "end_utc": float(times[-1]),
            "duration_s": float(times[-1] - times[0]),
            "interval_s": describe(np.diff(times)),
            "nearest_gt_abs_dt_s": describe(abs(nearest_gt_dt)),
            "gt_quality_counts": dict(sorted(Counter(
                str(int(value)) for value in gt_quality
            ).items())),
        },
        "alignment": {
            "method": "rigid_SE3_no_scale",
            "rotation": alignment_rotation.tolist(),
            "translation_m": alignment_translation.tolist(),
            "determinant": float(np.linalg.det(alignment_rotation)),
            "singular_values": singular_values.tolist(),
            "odom_path_length_m": odom_length,
            "gt_path_length_m": gt_length,
            "odom_over_gt_path_length_ratio_diagnostic_only": path_length_ratio,
        },
        "ate_position_m": ate,
        "absolute_rotation_error_deg": absolute_rotation,
        "position_error_trend": trend,
        "rpe": rpe,
        "screening": {
            "decision": screening_decision,
            "checks": screening_checks,
            "thresholds_are_project_screening_not_field_standards": {
                "ate_p95_m": args.ate_p95_screen,
                "rpe_1s_translation_p95_m": args.rpe1_translation_p95_screen,
                "rpe_1s_rotation_p95_deg": args.rpe1_rotation_p95_screen,
            },
        },
        "limitations": [
            "GT-to-Xsens lever arm is undocumented; rotating lever-arm residual may remain",
            "ATE is reported after one global rigid alignment and must be read with RPE",
            "This short-segment screen does not establish full-sequence loop consistency",
        ],
        "outputs": {
            "per_epoch_csv": os.path.abspath(args.out_csv),
            "plot": os.path.abspath(args.out_plot) if args.out_plot else None,
            "plot_written": plot_written,
            "plot_error": plot_error,
        },
    }

    os.makedirs(os.path.dirname(os.path.abspath(args.audit)), exist_ok=True)
    with open(args.audit, "w", encoding="utf-8") as handle:
        json.dump(audit, handle, indent=2, sort_keys=True)
        handle.write("\n")
    write_csv(
        args.out_csv, times, odom_positions, aligned_positions, gt_positions,
        position_errors, rotation_errors, nearest_gt_dt, gt_quality,
    )

    print("\n=== FAST-LIO vs GT trajectory audit ===")
    print("odometry total/inside GT:  %d / %d (%.2f%%)" % (
        len(odometry), len(in_coverage), 100.0 * len(in_coverage) / len(odometry)
    ))
    print("duration:                  %.3f s" % (times[-1] - times[0]))
    print("nearest GT |dt| p95/max:   %s / %s s" % (
        fmt(percentile(abs(nearest_gt_dt), 0.95), 6),
        fmt(float(np.max(abs(nearest_gt_dt))), 6),
    ))
    print("path length odom/GT:       %.3f / %.3f m (ratio %s)" % (
        odom_length, gt_length, fmt(path_length_ratio, 5)
    ))
    print("ATE median/RMSE/p95/max:   %s / %s / %s / %s m" % (
        fmt(ate["median"]), fmt(ate["rmse"]),
        fmt(ate["p95"]), fmt(ate["max"]),
    ))
    print("abs rotation med/p95/max:  %s / %s / %s deg" % (
        fmt(absolute_rotation["median"]), fmt(absolute_rotation["p95"]),
        fmt(absolute_rotation["max"]),
    ))
    print("ATE trend slope:           %s m/s" % fmt(trend["linear_slope_m_s"], 6))
    for key, metrics in rpe.items():
        print("RPE %s pairs=%d: trans p95=%s m, rot p95=%s deg" % (
            key, metrics["pairs"],
            fmt(metrics["translation_error_m"]["p95"]),
            fmt(metrics["rotation_error_deg"]["p95"]),
        ))
    print("screening decision:        %s" % screening_decision)
    for name, passed in screening_checks.items():
        print("  %s: %s" % ("PASS" if passed else "FAIL", name))
    print("CSV:   %s" % args.out_csv)
    print("Audit: %s" % args.audit)
    if args.out_plot:
        print("Plot:  %s (%s)" % (
            args.out_plot, "written" if plot_written else plot_error
        ))
    print("Caution: project thresholds are screening targets, not field standards.")


if __name__ == "__main__":
    main()
