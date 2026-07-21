#!/usr/bin/env python3
"""Resolve the UrbanV2X Velodyne point-time convention on moving segments.

Four hypotheses are compared without applying the small calibration lag:

* ``frame_header``: every point uses the frame header time;
* ``header_plus_time``: header + per-point relative time;
* ``header_minus_time``: header - per-point relative time; and
* ``header_plus_time_minus_span``: header + relative time - scan span.

Only high-turn and fast-straight windows are evaluated.  The primary metrics
come from mutual nearest-neighbour correspondences between adjacent frames.
For every mutual match, a local plane is fitted in the reference frame and
non-planar neighbourhoods are rejected.  This suppresses vegetation and
low-overlap points more effectively than whole-cloud nearest-neighbour scores.
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

from urbanv2x_gt_deskew_audit import candidate_motion_windows  # noqa: E402
from urbanv2x_lidar_gt_audit import parse_calibration, read_ground_truth  # noqa: E402
from urbanv2x_map_consistency import (  # noqa: E402
    PoseInterpolator,
    imu_flu_to_enu,
    read_frame_bounds,
    read_segment_frames,
    voxel_downsample,
)


CONVENTIONS = (
    "frame_header",
    "header_plus_time",
    "header_minus_time",
    "header_plus_time_minus_span",
)

RANKING_METRICS = {
    "static_plane_residual_median_m": "lower",
    "mutual_distance_median_m": "lower",
    "mutual_ratio_median": "higher",
    "planar_matches": "higher",
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Audit UrbanV2X Velodyne point-time conventions"
    )
    parser.add_argument("--bag", required=True)
    parser.add_argument("--gt", required=True)
    parser.add_argument("--calibration", required=True)
    parser.add_argument("--frames-audit", required=True)
    parser.add_argument("--topic", default="/velodyne_points")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--turn-segments", type=int, default=2)
    parser.add_argument("--straight-segments", type=int, default=2)
    parser.add_argument("--point-stride", type=int, default=20)
    parser.add_argument("--min-range", type=float, default=5.0)
    parser.add_argument("--max-range", type=float, default=40.0)
    parser.add_argument("--pair-step", type=int, default=5)
    parser.add_argument("--overlap-voxel", type=float, default=0.20)
    parser.add_argument("--mutual-radius", type=float, default=0.60)
    parser.add_argument("--plane-neighbours", type=int, default=8)
    parser.add_argument("--plane-radius", type=float, default=0.60)
    parser.add_argument("--planarity-ratio", type=float, default=0.12)
    parser.add_argument("--max-planar-samples-per-pair", type=int, default=1200)
    parser.add_argument("--preference-margin", type=float, default=0.01)
    parser.add_argument("--out-details", required=True)
    parser.add_argument("--out-summary", required=True)
    parser.add_argument("--audit", required=True)
    parser.add_argument("--out-plot")
    return parser.parse_args()


def percentile(values, probability):
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return None
    return float(np.percentile(values, 100.0 * probability))


def describe(values):
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return {
            "n": 0, "min": None, "median": None, "p95": None, "max": None
        }
    return {
        "n": int(values.size),
        "min": float(np.min(values)),
        "median": float(np.median(values)),
        "p95": percentile(values, 0.95),
        "max": float(np.max(values)),
    }


def select_target_motion_windows(windows, turn_count, straight_count):
    """Select distinct high-turn and fast-straight windows."""
    if turn_count <= 0 or straight_count <= 0:
        raise ValueError("Turn and straight segment counts must be positive")
    eligible = [
        (index, row) for index, row in enumerate(windows)
        if row["moving_ratio"] >= 0.75
    ]
    if len(eligible) < turn_count + straight_count:
        raise ValueError("Too few moving windows for requested selection")
    selected = {}
    turns = sorted(
        eligible, key=lambda item: item[1]["yaw_rate_p75_deg_s"], reverse=True
    )
    for index, row in turns:
        if len([value for value in selected.values()
                if value["selection_role"] == "high_turn"]) >= turn_count:
            break
        selected[index] = {**row, "selection_role": "high_turn"}

    yaw_values = [row["yaw_rate_p75_deg_s"] for _, row in eligible]
    yaw_limit = float(np.percentile(yaw_values, 40))
    straight_pool = [
        (index, row) for index, row in eligible
        if row["yaw_rate_p75_deg_s"] <= yaw_limit and index not in selected
    ]
    straight_pool.sort(
        key=lambda item: item[1]["median_speed_m_s"], reverse=True
    )
    for index, row in straight_pool[:straight_count]:
        selected[index] = {**row, "selection_role": "fast_straight"}
    straight_found = sum(
        row["selection_role"] == "fast_straight" for row in selected.values()
    )
    if straight_found < straight_count:
        fallback = sorted(
            [item for item in eligible if item[0] not in selected],
            key=lambda item: (
                item[1]["median_speed_m_s"],
                -item[1]["yaw_rate_p75_deg_s"],
            ),
            reverse=True,
        )
        for index, row in fallback:
            if straight_found >= straight_count:
                break
            selected[index] = {**row, "selection_role": "fast_straight_fallback"}
            straight_found += 1
    return [selected[index] for index in sorted(selected)]


def point_epochs(frame, convention):
    header = float(frame["header_time"])
    relative = np.asarray(frame["relative_time"], dtype=np.float64)
    if convention == "frame_header":
        return np.full(len(relative), header, dtype=np.float64)
    if convention == "header_plus_time":
        return header + relative
    if convention == "header_minus_time":
        return header - relative
    if convention == "header_plus_time_minus_span":
        span = float(np.max(relative) - np.min(relative)) if len(relative) else 0.0
        return header + relative - span
    raise ValueError("Unknown point-time convention: %s" % convention)


def transform_frame(frame, pose, lidar_to_imu, convention):
    rotation = lidar_to_imu[:3, :3]
    translation = lidar_to_imu[:3, 3]
    imu_points = frame["xyz"] @ rotation.T + translation
    epochs = point_epochs(frame, convention)
    position, roll, pitch, heading = pose.interpolate(epochs)
    world = imu_flu_to_enu(imu_points, roll, pitch, heading) + position
    return {
        "xyz": world.astype(np.float32),
        "intensity": frame["intensity"],
    }


def deterministic_indices(total, maximum):
    if total <= maximum:
        return np.arange(total, dtype=np.int64)
    return np.linspace(0, total - 1, maximum, dtype=np.int64)


def mutual_planar_pair(left_xyz, right_xyz, voxel, mutual_radius,
                       plane_neighbours, plane_radius, planarity_ratio,
                       maximum_samples):
    """Return mutual point distances and point-to-local-plane residuals."""
    try:
        from scipy.spatial import cKDTree
    except ImportError as exc:
        raise RuntimeError("scipy.spatial.cKDTree is required") from exc
    dummy_left = np.zeros(len(left_xyz), dtype=np.float32)
    dummy_right = np.zeros(len(right_xyz), dtype=np.float32)
    left, _, _, _ = voxel_downsample(left_xyz, dummy_left, voxel)
    right, _, _, _ = voxel_downsample(right_xyz, dummy_right, voxel)
    if len(left) < plane_neighbours or len(right) < plane_neighbours:
        return {
            "left_voxels": len(left), "right_voxels": len(right),
            "mutual_matches": 0, "mutual_ratio": 0.0,
            "mutual_distances": np.empty(0),
            "plane_residuals": np.empty(0),
        }
    left_tree = cKDTree(left)
    right_tree = cKDTree(right)
    distance, left_index = left_tree.query(
        right, k=1, distance_upper_bound=mutual_radius
    )
    _, right_index = right_tree.query(
        left, k=1, distance_upper_bound=mutual_radius
    )
    valid = np.isfinite(distance) & (left_index < len(left))
    mutual = np.zeros(len(right), dtype=bool)
    valid_ids = np.flatnonzero(valid)
    if len(valid_ids):
        mutual[valid_ids] = (
            right_index[left_index[valid_ids]] == valid_ids
        )
    mutual_ids = np.flatnonzero(mutual)
    sampled_ids = mutual_ids[
        deterministic_indices(len(mutual_ids), maximum_samples)
    ]
    plane_residuals = []
    for right_id in sampled_ids:
        neighbour_distance, neighbour_index = left_tree.query(
            right[right_id], k=plane_neighbours,
            distance_upper_bound=plane_radius,
        )
        if np.any(~np.isfinite(neighbour_distance)):
            continue
        neighbourhood = left[neighbour_index].astype(np.float64)
        centre = np.mean(neighbourhood, axis=0)
        covariance = np.cov(neighbourhood - centre, rowvar=False, bias=True)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        eigenvalues = np.maximum(eigenvalues, 0.0)
        if eigenvalues[1] <= 1e-7:
            continue
        if eigenvalues[0] / eigenvalues[1] > planarity_ratio:
            continue
        normal = eigenvectors[:, 0]
        plane_residuals.append(abs(float((right[right_id] - centre) @ normal)))
    return {
        "left_voxels": int(len(left)),
        "right_voxels": int(len(right)),
        "mutual_matches": int(len(mutual_ids)),
        "mutual_ratio": float(len(mutual_ids) / min(len(left), len(right))),
        "mutual_distances": distance[mutual],
        "plane_residuals": np.asarray(plane_residuals, dtype=np.float64),
    }


def evaluate_static_pairs(transformed, pair_step, **kwargs):
    mutual_distances = []
    plane_residuals = []
    ratios = []
    mutual_matches = 0
    evaluated_pairs = 0
    for right_index in range(pair_step, len(transformed), pair_step):
        left_index = right_index - 1
        result = mutual_planar_pair(
            transformed[left_index]["xyz"], transformed[right_index]["xyz"],
            **kwargs,
        )
        if result["mutual_matches"] == 0:
            continue
        evaluated_pairs += 1
        mutual_matches += result["mutual_matches"]
        ratios.append(result["mutual_ratio"])
        mutual_distances.append(result["mutual_distances"])
        if len(result["plane_residuals"]):
            plane_residuals.append(result["plane_residuals"])
    distances = (
        np.concatenate(mutual_distances) if mutual_distances else np.empty(0)
    )
    residuals = (
        np.concatenate(plane_residuals) if plane_residuals else np.empty(0)
    )
    return {
        "evaluated_pairs": evaluated_pairs,
        "mutual_matches": int(mutual_matches),
        "mutual_ratio_median": (
            float(np.median(ratios)) if ratios else None
        ),
        "mutual_distance_median_m": percentile(distances, 0.50),
        "mutual_distance_p95_m": percentile(distances, 0.95),
        "planar_matches": int(len(residuals)),
        "static_plane_residual_median_m": percentile(residuals, 0.50),
        "static_plane_residual_p95_m": percentile(residuals, 0.95),
    }


def evaluate_convention(segment_index, window, convention, frames, pose, matrix,
                        parameters):
    transformed = [
        transform_frame(frame, pose, matrix, convention) for frame in frames
    ]
    metrics = evaluate_static_pairs(
        transformed,
        pair_step=parameters["pair_step"],
        voxel=parameters["overlap_voxel"],
        mutual_radius=parameters["mutual_radius"],
        plane_neighbours=parameters["plane_neighbours"],
        plane_radius=parameters["plane_radius"],
        planarity_ratio=parameters["planarity_ratio"],
        maximum_samples=parameters["max_planar_samples_per_pair"],
    )
    return {
        "segment_index": int(segment_index),
        "selection_role": window["selection_role"],
        "start_utc": window["start_utc"],
        "start_elapsed_s": window["start_elapsed_s"],
        "median_speed_m_s": window["median_speed_m_s"],
        "yaw_rate_p75_deg_s": window["yaw_rate_p75_deg_s"],
        "gt_quality_mode": window["gt_quality_mode"],
        "convention": convention,
        "frames": len(frames),
        **metrics,
    }


def aggregate_scores(rows, metric_directions=RANKING_METRICS):
    conventions = sorted(set(row["convention"] for row in rows))
    segments = sorted(set(int(row["segment_index"]) for row in rows))
    scores = {name: [] for name in conventions}
    wins = {name: 0 for name in conventions}
    usable = []
    for segment in segments:
        selected = [row for row in rows if int(row["segment_index"]) == segment]
        by_name = {row["convention"]: row for row in selected}
        if set(by_name) != set(conventions):
            raise ValueError("Every segment must contain every convention")
        for metric, direction in metric_directions.items():
            values = {name: by_name[name].get(metric) for name in conventions}
            if any(value is None or not math.isfinite(float(value)) or value <= 0.0
                   for value in values.values()):
                continue
            if direction == "lower":
                best = min(float(value) for value in values.values())
                winner = min(conventions, key=lambda name: float(values[name]))
                normalized = {
                    name: float(values[name]) / best for name in conventions
                }
            elif direction == "higher":
                best = max(float(value) for value in values.values())
                winner = max(conventions, key=lambda name: float(values[name]))
                normalized = {
                    name: best / float(values[name]) for name in conventions
                }
            else:
                raise ValueError("Unknown metric direction: %s" % direction)
            wins[winner] += 1
            usable.append((segment, metric))
            for name in conventions:
                scores[name].append(normalized[name])
    if not usable:
        raise ValueError("No usable common metrics")
    summary = []
    for name in conventions:
        selected = [row for row in rows if row["convention"] == name]
        summary.append({
            "convention": name,
            "segments": len(selected),
            "normalized_score_mean": float(np.mean(scores[name])),
            "normalized_score_median": float(np.median(scores[name])),
            "metric_wins": int(wins[name]),
            **{
                metric + "_across_segments_median": statistics.median(
                    float(row[metric]) for row in selected
                )
                for metric in metric_directions
            },
        })
    summary.sort(key=lambda row: row["normalized_score_mean"])
    return summary, usable


def decide(summary, preference_margin):
    if len(summary) < 2:
        raise ValueError("At least two conventions are required")
    best, second = summary[:2]
    margin = (
        second["normalized_score_mean"] - best["normalized_score_mean"]
    ) / best["normalized_score_mean"]
    if margin >= preference_margin:
        decision = "PREFERRED"
        preferred = best["convention"]
    else:
        decision = "INDISTINGUISHABLE"
        preferred = None
    return decision, preferred, float(margin)


def write_csv(path, rows):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_plot(path, rows):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False, "matplotlib_not_installed"
    figure, axes = plt.subplots(1, 2, figsize=(13, 5), sharex=True)
    for convention in CONVENTIONS:
        selected = sorted(
            [row for row in rows if row["convention"] == convention],
            key=lambda row: row["start_elapsed_s"],
        )
        axes[0].plot(
            [row["start_elapsed_s"] for row in selected],
            [row["static_plane_residual_median_m"] for row in selected],
            marker="o", label=convention,
        )
        axes[1].plot(
            [row["start_elapsed_s"] for row in selected],
            [row["mutual_distance_median_m"] for row in selected],
            marker="o", label=convention,
        )
    axes[0].set_ylabel("Static point-to-plane median [m]")
    axes[1].set_ylabel("Mutual point distance median [m]")
    for axis in axes:
        axis.set_xlabel("Elapsed time [s]")
        axis.grid(True, alpha=0.3)
    axes[0].legend(fontsize=8)
    figure.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    figure.savefig(path, dpi=160)
    plt.close(figure)
    return True, None


def fmt(value, digits=4):
    return "NA" if value is None else ("%%.%df" % digits) % value


def main():
    args = parse_args()
    if min(
        args.point_stride, args.pair_step, args.plane_neighbours,
        args.max_planar_samples_per_pair,
    ) <= 0:
        raise ValueError("Stride, pair step, neighbours, and samples must be positive")
    print("Reading GT, calibration, and LiDAR coverage...")
    samples = read_ground_truth(args.gt)
    calibration = parse_calibration(args.calibration)
    lidar_start, lidar_end = read_frame_bounds(args.frames_audit)
    all_windows = candidate_motion_windows(
        samples, lidar_start, lidar_end, args.duration
    )
    windows = select_target_motion_windows(
        all_windows, args.turn_segments, args.straight_segments
    )
    for row in windows:
        row["start_elapsed_s"] = row["start_utc"] - lidar_start
    print("Selected %d targeted moving segments:" % len(windows))
    for index, row in enumerate(windows):
        print("  %d %.1fs role=%s speed=%.2f yaw_p75=%.2f quality=%d" % (
            index, row["start_elapsed_s"], row["selection_role"],
            row["median_speed_m_s"], row["yaw_rate_p75_deg_s"],
            row["gt_quality_mode"],
        ))

    pose = PoseInterpolator(samples)
    matrix = np.asarray(calibration["matrix"], dtype=np.float64)
    parameters = {
        "pair_step": args.pair_step,
        "overlap_voxel": args.overlap_voxel,
        "mutual_radius": args.mutual_radius,
        "plane_neighbours": args.plane_neighbours,
        "plane_radius": args.plane_radius,
        "planarity_ratio": args.planarity_ratio,
        "max_planar_samples_per_pair": args.max_planar_samples_per_pair,
    }
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
        for convention in CONVENTIONS:
            result = evaluate_convention(
                segment_index, window, convention, frames, pose, matrix,
                parameters,
            )
            rows.append(result)
            print("  %-27s plane=%s p95=%s mutual=%s ratio=%s planar_n=%d" % (
                convention,
                fmt(result["static_plane_residual_median_m"]),
                fmt(result["static_plane_residual_p95_m"]),
                fmt(result["mutual_distance_median_m"]),
                fmt(result["mutual_ratio_median"]),
                result["planar_matches"],
            ))

    summary, usable = aggregate_scores(rows)
    decision, preferred, margin = decide(summary, args.preference_margin)
    write_csv(args.out_details, rows)
    write_csv(args.out_summary, summary)
    plot_written = False
    plot_error = None
    if args.out_plot:
        plot_written, plot_error = write_plot(args.out_plot, rows)
    audit = {
        "schema_version": 1,
        "decision": decision,
        "preferred_convention": preferred,
        "score_margin_fraction": margin,
        "decision_scope": "point_time_convention_only_not_map_authorization",
        "inputs": {
            "bag": os.path.abspath(args.bag),
            "gt": os.path.abspath(args.gt),
            "calibration": os.path.abspath(args.calibration),
            "frames_audit": os.path.abspath(args.frames_audit),
        },
        "parameters": {
            "duration_s": args.duration,
            "turn_segments": args.turn_segments,
            "straight_segments": args.straight_segments,
            "point_stride": args.point_stride,
            "min_range_m": args.min_range,
            "max_range_m": args.max_range,
            **parameters,
            "preference_margin_fraction": args.preference_margin,
            "calibration_lag_deliberately_not_applied_s": calibration[
                "time_lag_imu_to_lidar_s"
            ],
        },
        "selected_windows": windows,
        "usable_metrics": [
            {"segment_index": segment, "metric": metric}
            for segment, metric in usable
        ],
        "summary": summary,
        "details": rows,
        "limitations": [
            "The Xsens-to-GT lever arm is undocumented and is treated as zero",
            "Geometric planarity suppresses but cannot identify every moving object",
            "Only high-turn and fast-straight windows are used by design",
            "The calibration lag is intentionally excluded because the prior audit found it indistinguishable",
            "A preferred convention still requires a separate absolute local-map quality gate",
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

    print("\n=== Point-time convention audit ===")
    print("convention,score_mean,score_median,wins,plane,mutual,ratio,planar_n")
    for row in summary:
        print("%s,%.6f,%.6f,%d,%s,%s,%s,%.0f" % (
            row["convention"], row["normalized_score_mean"],
            row["normalized_score_median"], row["metric_wins"],
            fmt(row["static_plane_residual_median_m_across_segments_median"]),
            fmt(row["mutual_distance_median_m_across_segments_median"]),
            fmt(row["mutual_ratio_median_across_segments_median"]),
            row["planar_matches_across_segments_median"],
        ))
    print("decision:                 %s" % decision)
    print("preferred convention:     %s" % (preferred or "NONE"))
    print("score margin:             %.2f%%" % (100.0 * margin))
    print("Details CSV: %s" % args.out_details)
    print("Summary CSV: %s" % args.out_summary)
    print("Audit JSON: %s" % args.audit)
    if args.out_plot:
        print("Plot: %s (%s)" % (
            args.out_plot, "written" if plot_written else plot_error
        ))
    print("Caution: this resolves time convention only, not final map usability.")


if __name__ == "__main__":
    main()
