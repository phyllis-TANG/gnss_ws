#!/usr/bin/env python3
"""Simulate GT-anchored five-second FAST-LIO tiles and audit their seams.

No corrected production map is written. Each saved per-scan PCD is mapped to
the odometry message with the same sequence index, sampled, and transformed
by the rigid no-scale FAST-LIO-to-GT alignment of its local time window. The
script then measures local planar thickness and adjacent corrected-tile
nearest-neighbour residuals. This is the final gate before writing a large
GT-anchored intensity tile set.
"""

import argparse
import csv
import glob
import json
import math
import os
import re
import sys

import numpy as np


SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIRECTORY not in sys.path:
    sys.path.insert(0, SCRIPT_DIRECTORY)

from urbanv2x_fastlio_gt_eval import (  # noqa: E402
    navigation_rotation,
    read_odometry,
    rigid_alignment,
)
from urbanv2x_fastlio_map_audit import (  # noqa: E402
    adjacent_overlap,
    deterministic_indices,
    parse_pcd_header,
    percentile,
    voxel_downsample_xyz,
)
from urbanv2x_lidar_gt_audit import read_ground_truth  # noqa: E402
from urbanv2x_map_consistency import PoseInterpolator, planar_cell_residual  # noqa: E402


def parse_args():
    parser = argparse.ArgumentParser(
        description="Audit sampled GT-anchored UrbanV2X FAST-LIO tiles"
    )
    parser.add_argument("--bag", required=True)
    parser.add_argument("--gt", required=True)
    parser.add_argument("--pcd-dir", required=True)
    parser.add_argument("--windows-csv", required=True)
    parser.add_argument("--topic", default="/Odometry")
    parser.add_argument("--window-seconds", type=float, default=5.0)
    parser.add_argument("--max-sample-per-window", type=int, default=100000)
    parser.add_argument("--plane-voxel", type=float, default=0.10)
    parser.add_argument("--plane-cell", type=float, default=1.0)
    parser.add_argument("--plane-min-points", type=int, default=12)
    parser.add_argument("--overlap-voxel", type=float, default=0.20)
    parser.add_argument("--overlap-radius", type=float, default=0.50)
    parser.add_argument("--plane-median-pass", type=float, default=0.10)
    parser.add_argument("--plane-median-review", type=float, default=0.20)
    parser.add_argument("--seam-median-pass", type=float, default=0.15)
    parser.add_argument("--seam-median-review", type=float, default=0.30)
    parser.add_argument("--out-windows", required=True)
    parser.add_argument("--out-seams", required=True)
    parser.add_argument("--audit", required=True)
    parser.add_argument("--out-plot")
    return parser.parse_args()


def scan_number(path):
    match = re.fullmatch(r"scans_(\d+)\.pcd", os.path.basename(path))
    if not match:
        raise ValueError("Unexpected per-scan PCD name: %s" % path)
    return int(match.group(1))


def ordered_scan_files(directory):
    paths = glob.glob(os.path.join(directory, "scans_[0-9]*.pcd"))
    paths.sort(key=scan_number)
    numbers = [scan_number(path) for path in paths]
    if not paths:
        raise ValueError("No scans_N.pcd files found in %s" % directory)
    expected = list(range(1, len(paths) + 1))
    if numbers != expected:
        missing = sorted(set(expected) - set(numbers))
        raise ValueError("PCD sequence is not contiguous from 1: missing %s" % missing[:20])
    return paths


def _float_or_none(value):
    if value in (None, "", "None", "NA"):
        return None
    return float(value)


def read_source_windows(path, duration):
    rows = []
    with open(path, "r", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            if abs(float(row["window_s"]) - duration) > 1e-6:
                continue
            rows.append({
                "requested_start_utc": float(row["requested_start_utc"]),
                "requested_end_utc": float(row["requested_end_utc"]),
                "source_decision": row["decision"],
                "source_reason": row["reason"],
                "source_ate_p95_m": _float_or_none(row.get("ate_p95_m")),
                "source_rpe_translation_p95_m": _float_or_none(
                    row.get("rpe_translation_p95_m")
                ),
                "source_rpe_rotation_p95_deg": _float_or_none(
                    row.get("rpe_rotation_p95_deg")
                ),
            })
    rows.sort(key=lambda row: row["requested_start_utc"])
    if not rows:
        raise ValueError("No %.3f-second rows found in %s" % (duration, path))
    return rows


def build_window_assignments(times, source_rows):
    """Return non-overlapping source windows plus an explicit final tail."""
    times = np.asarray(times, dtype=np.float64)
    assignments = []
    for row in source_rows:
        start_index = int(np.searchsorted(
            times, row["requested_start_utc"], side="left"
        ))
        end_index = int(np.searchsorted(
            times, row["requested_end_utc"], side="left"
        ))
        assignments.append({
            **row,
            "start_index": start_index,
            "end_index": end_index,
            "is_tail": False,
        })
    final_end = source_rows[-1]["requested_end_utc"]
    tail_start = int(np.searchsorted(times, final_end, side="left"))
    if tail_start < len(times):
        assignments.append({
            "requested_start_utc": float(final_end),
            "requested_end_utc": float(times[-1] + 1e-6),
            "source_decision": "TAIL_REVIEW",
            "source_reason": "short_final_tail_not_in_fixed_window_audit",
            "source_ate_p95_m": None,
            "source_rpe_translation_p95_m": None,
            "source_rpe_rotation_p95_deg": None,
            "start_index": tail_start,
            "end_index": len(times),
            "is_tail": True,
        })

    coverage = np.zeros(len(times), dtype=np.int16)
    for row in assignments:
        coverage[row["start_index"]:row["end_index"]] += 1
    if np.any(coverage != 1):
        missing = int(np.count_nonzero(coverage == 0))
        duplicated = int(np.count_nonzero(coverage > 1))
        raise ValueError(
            "Window assignment is not one-to-one: missing=%d duplicated=%d"
            % (missing, duplicated)
        )
    return assignments


def load_xyz_sample(path, maximum):
    metadata = parse_pcd_header(path)
    if not metadata["payload_exact"]:
        raise ValueError("PCD payload mismatch: %s" % path)
    missing = {"x", "y", "z"} - set(metadata["fields"])
    if missing:
        raise ValueError("PCD missing XYZ fields %s: %s" % (sorted(missing), path))
    array = np.memmap(
        path, mode="r", dtype=metadata["dtype"],
        offset=metadata["data_offset"], shape=(metadata["points"],),
    )
    indices = deterministic_indices(metadata["points"], maximum)
    sample = array[indices]
    finite = (
        np.isfinite(sample["x"]) & np.isfinite(sample["y"])
        & np.isfinite(sample["z"])
    )
    sample = sample[finite]
    xyz = np.column_stack([sample["x"], sample["y"], sample["z"]]).astype(
        np.float32, copy=True
    )
    points = metadata["points"]
    del array
    return xyz, points


def choose_per_scan_sample(maximum_per_window, scan_count):
    if maximum_per_window <= 0 or scan_count <= 0:
        raise ValueError("Sample maximum and scan count must be positive")
    return max(1, int(math.ceil(maximum_per_window / scan_count)))


def transform_points(points, rotation, translation):
    return ((rotation @ np.asarray(points).T).T + translation).astype(np.float32)


def classify_anchor_decision(source_decision, geometry_decision):
    """Preserve known source failures while allowing review-only tail tiles."""
    if source_decision == "FAIL" or geometry_decision == "FAIL":
        return "FAIL"
    if source_decision == "PASS" and geometry_decision == "PASS":
        return "PASS"
    return "REVIEW"


def evaluate_anchor_windows(assignments, times, odom_positions, gt_positions,
                            pcd_files, max_sample, plane_voxel, plane_cell,
                            plane_min_points, plane_pass, plane_review):
    results = []
    samples = []
    for index, assignment in enumerate(assignments):
        start = assignment["start_index"]
        end = assignment["end_index"]
        if end - start < 3:
            raise ValueError("Anchor window has fewer than three poses")
        rotation, translation, singular_values = rigid_alignment(
            odom_positions[start:end], gt_positions[start:end]
        )
        per_scan_sample = choose_per_scan_sample(max_sample, end - start)
        point_samples = []
        input_points = 0
        for path in pcd_files[start:end]:
            xyz, points = load_xyz_sample(path, per_scan_sample)
            input_points += points
            point_samples.append(xyz)
        xyz = np.concatenate(point_samples, axis=0)
        if len(xyz) > max_sample:
            xyz = xyz[deterministic_indices(len(xyz), max_sample)]
        anchored = transform_points(xyz, rotation, translation)
        planar_xyz = voxel_downsample_xyz(anchored, plane_voxel)
        plane, plane_cells = planar_cell_residual(
            planar_xyz, coarse_cell=plane_cell,
            minimum_points=plane_min_points,
        )
        median = plane["median"]
        if median is None or plane_cells < 20 or median > plane_review:
            geometry_decision = "FAIL"
        elif median <= plane_pass:
            geometry_decision = "PASS"
        else:
            geometry_decision = "REVIEW"
        source = assignment["source_decision"]
        anchor_decision = classify_anchor_decision(source, geometry_decision)
        results.append({
            "window_index": index,
            "start_elapsed_s": float(times[start] - times[0]),
            "end_elapsed_s": float(times[end - 1] - times[0]),
            "duration_s": float(times[end - 1] - times[start]),
            "is_tail": assignment["is_tail"],
            "source_decision": source,
            "source_reason": assignment["source_reason"],
            "source_ate_p95_m": assignment["source_ate_p95_m"],
            "source_rpe_translation_p95_m": assignment[
                "source_rpe_translation_p95_m"
            ],
            "source_rpe_rotation_p95_deg": assignment[
                "source_rpe_rotation_p95_deg"
            ],
            "poses": end - start,
            "first_pcd": os.path.basename(pcd_files[start]),
            "last_pcd": os.path.basename(pcd_files[end - 1]),
            "input_points": input_points,
            "sample_points": int(len(anchored)),
            "planar_voxels": int(len(planar_xyz)),
            "planar_cells": plane_cells,
            "plane_residual_median_m": plane["median"],
            "plane_residual_p95_m": plane["p95"],
            "geometry_decision": geometry_decision,
            "anchor_decision": anchor_decision,
            "alignment_det": float(np.linalg.det(rotation)),
            "alignment_singular_1": float(singular_values[0]),
            "alignment_singular_2": float(singular_values[1]),
            "alignment_singular_3": float(singular_values[2]),
            "alignment_rotation": rotation.tolist(),
            "alignment_translation_m": translation.tolist(),
        })
        samples.append(anchored)
        print("  tile %02d %.1f-%.1fs source=%s plane=%s med=%s p95=%s" % (
            index, results[-1]["start_elapsed_s"], results[-1]["end_elapsed_s"],
            source, geometry_decision,
            _fmt(plane["median"]), _fmt(plane["p95"]),
        ))
    return results, samples


def evaluate_seams(window_results, samples, overlap_voxel, overlap_radius,
                   seam_pass, seam_review):
    seams = []
    for index in range(len(samples) - 1):
        overlap = adjacent_overlap(
            samples[index], samples[index + 1], overlap_voxel, overlap_radius
        )
        median = overlap["mutual_distance_median_m"]
        matches = overlap["mutual_matches"]
        if median is None or matches < 50 or median > seam_review:
            decision = "FAIL"
        elif matches >= 100 and median <= seam_pass:
            decision = "PASS"
        else:
            decision = "REVIEW"
        seams.append({
            "left_window": index,
            "right_window": index + 1,
            "boundary_elapsed_s": window_results[index + 1]["start_elapsed_s"],
            "left_source_decision": window_results[index]["source_decision"],
            "right_source_decision": window_results[index + 1]["source_decision"],
            **overlap,
            "seam_decision": decision,
        })
        print("  seam %02d->%02d boundary=%.1fs %s med=%s p95=%s mutual=%d" % (
            index, index + 1, seams[-1]["boundary_elapsed_s"], decision,
            _fmt(median), _fmt(overlap["mutual_distance_p95_m"]), matches,
        ))
    return seams


def write_csv(path, rows, excluded=()):
    if not rows:
        raise ValueError("Cannot write empty CSV")
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fields = []
    for row in rows:
        for key, value in row.items():
            if key in excluded or isinstance(value, (list, dict)):
                continue
            if key not in fields:
                fields.append(key)
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def write_plot(path, windows, seams, plane_pass, seam_pass):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False, "matplotlib_not_installed"
    figure, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    axes[0].plot(
        [row["start_elapsed_s"] + row["duration_s"] / 2.0 for row in windows],
        [row["plane_residual_median_m"] for row in windows], marker="o",
    )
    axes[0].axhline(plane_pass, color="k", linestyle="--", alpha=0.5)
    axes[0].set_ylabel("Planar residual median [m]")
    axes[1].plot(
        [row["boundary_elapsed_s"] for row in seams],
        [row["mutual_distance_median_m"] for row in seams], marker="o",
        color="C1",
    )
    axes[1].axhline(seam_pass, color="k", linestyle="--", alpha=0.5)
    axes[1].set_ylabel("Anchored seam median [m]")
    axes[1].set_xlabel("Elapsed time [s]")
    for axis in axes:
        axis.grid(True, alpha=0.3)
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)
    return True, None


def _fmt(value, digits=4):
    return "NA" if value is None else ("%%.%df" % digits) % value


def decision_counts(rows, key):
    return {
        decision: sum(row[key] == decision for row in rows)
        for decision in ("PASS", "REVIEW", "FAIL")
    }


def main():
    args = parse_args()
    if args.max_sample_per_window <= 0:
        raise ValueError("Sample limit must be positive")
    print("Reading odometry and validating one-to-one PCD manifest...")
    odometry, odometry_audit = read_odometry(args.bag, args.topic)
    pcd_files = ordered_scan_files(args.pcd_dir)
    if len(pcd_files) != len(odometry):
        raise ValueError(
            "PCD/odometry count mismatch: %d vs %d"
            % (len(pcd_files), len(odometry))
        )
    times = np.asarray([row["time"] for row in odometry])
    odom_positions = np.asarray([row["position"] for row in odometry])
    print("  manifest PASS: %d PCDs == %d poses" % (
        len(pcd_files), len(odometry)
    ))

    print("Reading GT and source window decisions...")
    gt_samples = read_ground_truth(args.gt)
    pose = PoseInterpolator(gt_samples)
    gt_positions, roll, pitch, heading = pose.interpolate(times)
    # Constructing these rotations also verifies the audited GT convention,
    # although local PCD anchoring itself uses position-derived rigid fits.
    _ = np.asarray([
        navigation_rotation(roll[index], pitch[index], heading[index])
        for index in range(len(times))
    ])
    source_rows = read_source_windows(args.windows_csv, args.window_seconds)
    assignments = build_window_assignments(times, source_rows)
    print("  windows: %d fixed + %d tail" % (
        sum(not row["is_tail"] for row in assignments),
        sum(row["is_tail"] for row in assignments),
    ))

    print("Sampling and simulating GT-anchored tiles...")
    windows, samples = evaluate_anchor_windows(
        assignments, times, odom_positions, gt_positions, pcd_files,
        args.max_sample_per_window, args.plane_voxel, args.plane_cell,
        args.plane_min_points, args.plane_median_pass,
        args.plane_median_review,
    )
    print("Evaluating adjacent anchored-tile seams...")
    seams = evaluate_seams(
        windows, samples, args.overlap_voxel, args.overlap_radius,
        args.seam_median_pass, args.seam_median_review,
    )
    window_counts = decision_counts(windows, "anchor_decision")
    seam_counts = decision_counts(seams, "seam_decision")
    if window_counts["FAIL"] == 0 and seam_counts["FAIL"] == 0:
        overall = (
            "SCREEN_PASS" if window_counts["REVIEW"] == 0
            and seam_counts["REVIEW"] == 0 else "PASS_WITH_REVIEW"
        )
    else:
        overall = "REVIEW_REQUIRED"

    write_csv(args.out_windows, windows, excluded={
        "alignment_rotation", "alignment_translation_m"
    })
    write_csv(args.out_seams, seams)
    plot_written = False
    plot_error = None
    if args.out_plot:
        plot_written, plot_error = write_plot(
            args.out_plot, windows, seams,
            args.plane_median_pass, args.seam_median_pass,
        )
    audit = {
        "schema_version": 1,
        "decision": overall,
        "inputs": {
            "bag": os.path.abspath(args.bag),
            "gt": os.path.abspath(args.gt),
            "pcd_dir": os.path.abspath(args.pcd_dir),
            "windows_csv": os.path.abspath(args.windows_csv),
        },
        "manifest": {
            "pcd_files": len(pcd_files),
            "odometry_poses": len(odometry),
            "one_to_one": len(pcd_files) == len(odometry),
            "first_pcd": os.path.basename(pcd_files[0]),
            "last_pcd": os.path.basename(pcd_files[-1]),
        },
        "odometry": odometry_audit,
        "parameters": {
            "window_seconds": args.window_seconds,
            "max_sample_per_window": args.max_sample_per_window,
            "plane_voxel_m": args.plane_voxel,
            "plane_cell_m": args.plane_cell,
            "overlap_voxel_m": args.overlap_voxel,
            "overlap_radius_m": args.overlap_radius,
        },
        "thresholds_are_project_screens_not_field_standards": {
            "plane_median_pass_m": args.plane_median_pass,
            "plane_median_review_m": args.plane_median_review,
            "seam_median_pass_m": args.seam_median_pass,
            "seam_median_review_m": args.seam_median_review,
        },
        "window_decision_counts": window_counts,
        "seam_decision_counts": seam_counts,
        "plane_residual_median_across_tiles_m": {
            "median": percentile(
                [row["plane_residual_median_m"] for row in windows], 0.5
            ),
            "p95": percentile(
                [row["plane_residual_median_m"] for row in windows], 0.95
            ),
        },
        "seam_median_across_boundaries_m": {
            "median": percentile(
                [row["mutual_distance_median_m"] for row in seams], 0.5
            ),
            "p95": percentile(
                [row["mutual_distance_median_m"] for row in seams], 0.95
            ),
        },
        "windows": windows,
        "seams": seams,
        "limitations": [
            "PCD-to-odometry association is sequence-based after exact count/contiguity checks",
            "Only deterministic point samples are transformed during this audit",
            "Dynamic objects and vegetation affect plane and seam metrics",
            "Rigid anchoring preserves within-tile plane thickness; adjacent seam metrics test cross-tile placement",
            "GT and Xsens reference origins are treated as coincident because their lever arm is undocumented",
            "Passing this audit authorizes tile generation, not calibrated-reflectivity claims",
        ],
        "outputs": {
            "windows_csv": os.path.abspath(args.out_windows),
            "seams_csv": os.path.abspath(args.out_seams),
            "plot": os.path.abspath(args.out_plot) if args.out_plot else None,
            "plot_written": plot_written,
            "plot_error": plot_error,
        },
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.audit)), exist_ok=True)
    with open(args.audit, "w", encoding="utf-8") as handle:
        json.dump(audit, handle, indent=2, sort_keys=True)
        handle.write("\n")

    print("\n=== GT-anchored tile simulation audit ===")
    print("manifest:                 %d PCDs / %d poses PASS" % (
        len(pcd_files), len(odometry)
    ))
    print("tiles PASS/REVIEW/FAIL:    %d / %d / %d" % (
        window_counts["PASS"], window_counts["REVIEW"], window_counts["FAIL"]
    ))
    print("seams PASS/REVIEW/FAIL:    %d / %d / %d" % (
        seam_counts["PASS"], seam_counts["REVIEW"], seam_counts["FAIL"]
    ))
    print("tile plane median/p95:    %s / %s m" % (
        _fmt(audit["plane_residual_median_across_tiles_m"]["median"]),
        _fmt(audit["plane_residual_median_across_tiles_m"]["p95"]),
    ))
    print("seam median median/p95:   %s / %s m" % (
        _fmt(audit["seam_median_across_boundaries_m"]["median"]),
        _fmt(audit["seam_median_across_boundaries_m"]["p95"]),
    ))
    print("decision:                 %s" % overall)
    problematic_tiles = [row for row in windows if row["anchor_decision"] != "PASS"]
    if problematic_tiles:
        print("non-PASS tiles (start, source, geometry, anchor):")
        for row in problematic_tiles:
            print("  %.1fs %s %s %s plane=%s" % (
                row["start_elapsed_s"], row["source_decision"],
                row["geometry_decision"], row["anchor_decision"],
                _fmt(row["plane_residual_median_m"]),
            ))
    problematic_seams = [row for row in seams if row["seam_decision"] != "PASS"]
    if problematic_seams:
        print("non-PASS seams (boundary, decision, median, p95):")
        for row in problematic_seams:
            print("  %.1fs %s %s %s" % (
                row["boundary_elapsed_s"], row["seam_decision"],
                _fmt(row["mutual_distance_median_m"]),
                _fmt(row["mutual_distance_p95_m"]),
            ))
    print("Windows CSV: %s" % args.out_windows)
    print("Seams CSV: %s" % args.out_seams)
    print("Audit JSON: %s" % args.audit)
    if args.out_plot:
        print("Plot: %s (%s)" % (
            args.out_plot, "written" if plot_written else plot_error
        ))
    print("Caution: this is a sampled pre-write audit, not the final fused map.")


if __name__ == "__main__":
    main()
