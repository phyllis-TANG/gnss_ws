#!/usr/bin/env python3
"""Build a pre-C/N0 inventory of repeatable local facade reflectivity patches.

Each audited five-second FAST-LIO tile is rigidly anchored to the GT local ENU
frame and divided into fixed 3-D cells.  PCA is used to identify approximately
vertical planar surface patches.  The tile's scan files are interleaved into
odd/even A/B subsets so both subsets cover approximately the same trajectory;
geometry and intensity repeatability can then be evaluated without confusing
vehicle motion with surface instability.  No GNSS signal-strength outcome is
read or used.
"""

import argparse
import csv
import json
import math
import os
import sys
from collections import Counter, defaultdict

import numpy as np


SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIRECTORY not in sys.path:
    sys.path.insert(0, SCRIPT_DIRECTORY)

from urbanv2x_fastlio_map_audit import percentile  # noqa: E402
from urbanv2x_local_surface_audit import (  # noqa: E402
    pcd_range,
    read_anchor_windows,
    read_pcd_points,
    voxelise_surface,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Inventory repeatable UrbanV2X local facade reflectivity patches"
    )
    parser.add_argument("--rays", required=True,
                        help="Representative ray CSV used only to select tile IDs")
    parser.add_argument("--anchor-audit", required=True)
    parser.add_argument("--pcd-dir", required=True)
    parser.add_argument("--out-patches", required=True)
    parser.add_argument("--out-tiles", required=True)
    parser.add_argument("--audit", required=True)
    parser.add_argument("--out-plot")
    parser.add_argument("--surface-voxel", type=float, default=0.20)
    parser.add_argument("--cell-size", type=float, default=2.0)
    parser.add_argument("--cell-min-points", type=int, default=20)
    parser.add_argument("--half-min-points", type=int, default=8)
    parser.add_argument("--vertical-normal-z-max", type=float, default=0.35)
    parser.add_argument("--planarity-min", type=float, default=0.30)
    parser.add_argument("--plane-residual-median-max", type=float, default=0.10)
    parser.add_argument("--plane-residual-p95-max", type=float, default=0.20)
    parser.add_argument("--horizontal-span-min", type=float, default=0.80)
    parser.add_argument("--vertical-span-min", type=float, default=0.80)
    parser.add_argument("--diffuse-points-min", type=int, default=12)
    parser.add_argument("--diffuse-intensity-max", type=float, default=100.0)
    parser.add_argument("--normal-repeat-max", type=float, default=15.0)
    parser.add_argument("--plane-offset-repeat-max", type=float, default=0.15)
    parser.add_argument("--intensity-repeat-absolute", type=float, default=5.0)
    parser.add_argument("--intensity-repeat-relative", type=float, default=0.30)
    return parser.parse_args()


def describe(values):
    values = np.asarray([value for value in values if value is not None], dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return {"n": 0, "median": None, "p05": None, "p95": None, "max": None}
    return {
        "n": int(len(values)),
        "median": float(np.median(values)),
        "p05": float(np.percentile(values, 5)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def read_tile_ids(path):
    with open(path, "r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows or "tile_window_index" not in rows[0]:
        raise ValueError("Ray CSV is empty or missing tile_window_index")
    return sorted(set(int(row["tile_window_index"]) for row in rows))


def transform_points(points, window):
    rotation = np.asarray(window["alignment_rotation"], dtype=np.float64)
    translation = np.asarray(window["alignment_translation_m"], dtype=np.float64)
    return ((rotation @ np.asarray(points).T).T + translation).astype(np.float32)


def load_scan_group(paths, window, voxel):
    xyz_parts = []
    intensity_parts = []
    input_points = 0
    for path in paths:
        xyz, intensity = read_pcd_points(path)
        input_points += len(xyz)
        xyz_parts.append(xyz)
        intensity_parts.append(intensity)
    if not xyz_parts:
        return (np.empty((0, 3), dtype=np.float32),
                np.empty(0, dtype=np.float32), 0)
    xyz = transform_points(np.concatenate(xyz_parts, axis=0), window)
    intensity = np.concatenate(intensity_parts, axis=0)
    xyz, intensity = voxelise_surface(xyz, intensity, voxel)
    return xyz, intensity, input_points


def load_tile_splits(window, pcd_dir, voxel):
    paths = pcd_range(window["first_pcd"], window["last_pcd"], pcd_dir)
    if len(paths) < 4:
        raise ValueError("Tile has fewer than four PCD files")
    paths_a = paths[::2]
    paths_b = paths[1::2]
    subset_a_xyz, subset_a_intensity, subset_a_input = load_scan_group(
        paths_a, window, voxel
    )
    subset_b_xyz, subset_b_intensity, subset_b_input = load_scan_group(
        paths_b, window, voxel
    )
    all_xyz, all_intensity = voxelise_surface(
        np.concatenate([subset_a_xyz, subset_b_xyz], axis=0),
        np.concatenate([subset_a_intensity, subset_b_intensity], axis=0),
        voxel,
    )
    return {
        "paths": paths,
        "paths_a": paths_a,
        "paths_b": paths_b,
        "input_points": subset_a_input + subset_b_input,
        "all_xyz": all_xyz,
        "all_intensity": all_intensity,
        "subset_a_xyz": subset_a_xyz,
        "subset_a_intensity": subset_a_intensity,
        "subset_b_xyz": subset_b_xyz,
        "subset_b_intensity": subset_b_intensity,
    }


def cell_keys(xyz, size):
    return np.floor(np.asarray(xyz, dtype=np.float64) / size).astype(np.int64)


def grouped_indices(xyz, size):
    keys = cell_keys(xyz, size)
    groups = defaultdict(list)
    for index, key in enumerate(keys):
        groups[tuple(int(value) for value in key)].append(index)
    return {key: np.asarray(indices, dtype=np.int64)
            for key, indices in groups.items()}


def orient_vertical_normal(normal):
    normal = np.asarray(normal, dtype=np.float64)
    horizontal = np.hypot(normal[0], normal[1])
    if horizontal > 0.0:
        # Deterministic sign: horizontal projection points into the positive
        # half-plane, while downstream ray association remains sign-invariant.
        if normal[0] < 0.0 or (abs(normal[0]) < 1e-12 and normal[1] < 0.0):
            normal = -normal
    elif normal[2] < 0.0:
        normal = -normal
    return normal


def fit_patch_geometry(points):
    points = np.asarray(points, dtype=np.float64)
    if len(points) < 3:
        return None
    centroid = np.mean(points, axis=0)
    centred = points - centroid
    covariance = centred.T @ centred / max(len(points) - 1, 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    if not np.all(np.isfinite(eigenvalues)) or eigenvalues[2] <= 0.0:
        return None
    normal = orient_vertical_normal(eigenvectors[:, 0])
    residual = np.abs(centred @ normal)
    horizontal_tangent = np.array([-normal[1], normal[0], 0.0])
    tangent_norm = np.linalg.norm(horizontal_tangent)
    if tangent_norm <= 1e-12:
        horizontal_span = 0.0
    else:
        horizontal_tangent /= tangent_norm
        projection = centred @ horizontal_tangent
        horizontal_span = float(
            np.percentile(projection, 95) - np.percentile(projection, 5)
        )
    vertical_span = float(
        np.percentile(points[:, 2], 95) - np.percentile(points[:, 2], 5)
    )
    return {
        "centroid": centroid,
        "normal": normal,
        "eigenvalues": eigenvalues,
        "planarity": float((eigenvalues[1] - eigenvalues[0]) / eigenvalues[2]),
        "linearity": float((eigenvalues[2] - eigenvalues[1]) / eigenvalues[2]),
        "plane_residual_median_m": float(np.median(residual)),
        "plane_residual_p95_m": float(np.percentile(residual, 95)),
        "horizontal_span_m": horizontal_span,
        "vertical_span_m": vertical_span,
    }


def intensity_statistics(values, diffuse_max):
    values = np.asarray(values, dtype=np.float64)
    finite_positive = np.isfinite(values) & (values > 0.0)
    diffuse = values[finite_positive & (values <= diffuse_max)]
    retro_count = int(np.count_nonzero(finite_positive & (values > diffuse_max)))
    positive_count = int(np.count_nonzero(finite_positive))
    output = {
        "positive_points": positive_count,
        "diffuse_points": int(len(diffuse)),
        "retro_points": retro_count,
        "retro_fraction": retro_count / positive_count if positive_count else None,
        "intensity_median": None,
        "intensity_iqr": None,
        "intensity_mad": None,
    }
    if len(diffuse):
        median = float(np.median(diffuse))
        output.update({
            "intensity_median": median,
            "intensity_iqr": float(
                np.percentile(diffuse, 75) - np.percentile(diffuse, 25)
            ),
            "intensity_mad": float(np.median(np.abs(diffuse - median))),
        })
    return output


def unsigned_normal_angle(left, right):
    cosine = float(np.clip(abs(np.dot(left, right)), 0.0, 1.0))
    return math.degrees(math.acos(cosine))


def evaluate_repeatability(subset_a_geometry, subset_b_geometry,
                           subset_a_intensity, subset_b_intensity, args):
    output = {
        "normal_repeat_angle_deg": None,
        "plane_offset_repeat_m": None,
        "intensity_repeat_abs": None,
        "intensity_repeat_relative": None,
        "geometry_repeatable": 0,
        "reflectivity_repeatable": 0,
    }
    if subset_a_geometry is None or subset_b_geometry is None:
        return output
    normal_angle = unsigned_normal_angle(
        subset_a_geometry["normal"], subset_b_geometry["normal"]
    )
    # Symmetric centroid-to-plane disagreement is sign invariant.
    a_to_b = abs(np.dot(
        subset_a_geometry["centroid"] - subset_b_geometry["centroid"],
        subset_b_geometry["normal"],
    ))
    b_to_a = abs(np.dot(
        subset_b_geometry["centroid"] - subset_a_geometry["centroid"],
        subset_a_geometry["normal"],
    ))
    plane_offset = float(max(a_to_b, b_to_a))
    geometry_repeatable = (
        normal_angle <= args.normal_repeat_max
        and plane_offset <= args.plane_offset_repeat_max
    )
    subset_a_value = subset_a_intensity.get("intensity_median")
    subset_b_value = subset_b_intensity.get("intensity_median")
    absolute = None
    relative = None
    reflectivity_repeatable = False
    if subset_a_value is not None and subset_b_value is not None:
        absolute = abs(subset_a_value - subset_b_value)
        denominator = max(0.5 * (subset_a_value + subset_b_value), 1e-6)
        relative = absolute / denominator
        reflectivity_repeatable = (
            absolute <= args.intensity_repeat_absolute
            or relative <= args.intensity_repeat_relative
        )
    output.update({
        "normal_repeat_angle_deg": normal_angle,
        "plane_offset_repeat_m": plane_offset,
        "intensity_repeat_abs": absolute,
        "intensity_repeat_relative": relative,
        "geometry_repeatable": int(geometry_repeatable),
        "reflectivity_repeatable": int(reflectivity_repeatable),
    })
    return output


def facade_quality_reasons(geometry, intensity, args):
    reasons = []
    if abs(geometry["normal"][2]) > args.vertical_normal_z_max:
        reasons.append("not_vertical")
    if geometry["planarity"] < args.planarity_min:
        reasons.append("low_planarity")
    if geometry["plane_residual_median_m"] > args.plane_residual_median_max:
        reasons.append("thick_surface_median")
    if geometry["plane_residual_p95_m"] > args.plane_residual_p95_max:
        reasons.append("thick_surface_p95")
    if geometry["horizontal_span_m"] < args.horizontal_span_min:
        reasons.append("narrow_horizontal_span")
    if geometry["vertical_span_m"] < args.vertical_span_min:
        reasons.append("short_vertical_span")
    if intensity["diffuse_points"] < args.diffuse_points_min:
        reasons.append("few_diffuse_points")
    return reasons


def serialise_patch(tile_index, key, indices, geometry, intensity, reasons,
                    subset_a_indices, subset_a_geometry, subset_a_intensity,
                    subset_b_indices, subset_b_geometry, subset_b_intensity,
                    repeatability):
    centroid = geometry["centroid"]
    normal = geometry["normal"]
    eigenvalues = geometry["eigenvalues"]
    output = {
        "tile_window_index": tile_index,
        "cell_x": key[0], "cell_y": key[1], "cell_z": key[2],
        "patch_id": "%d:%d:%d:%d" % (tile_index, key[0], key[1], key[2]),
        "points": len(indices),
        "centroid_e_m": float(centroid[0]),
        "centroid_n_m": float(centroid[1]),
        "centroid_u_m": float(centroid[2]),
        "normal_e": float(normal[0]),
        "normal_n": float(normal[1]),
        "normal_u": float(normal[2]),
        "abs_normal_u": float(abs(normal[2])),
        "eigenvalue_0": float(eigenvalues[0]),
        "eigenvalue_1": float(eigenvalues[1]),
        "eigenvalue_2": float(eigenvalues[2]),
        "planarity": geometry["planarity"],
        "linearity": geometry["linearity"],
        "plane_residual_median_m": geometry["plane_residual_median_m"],
        "plane_residual_p95_m": geometry["plane_residual_p95_m"],
        "horizontal_span_m": geometry["horizontal_span_m"],
        "vertical_span_m": geometry["vertical_span_m"],
        "positive_points": intensity["positive_points"],
        "diffuse_points": intensity["diffuse_points"],
        "retro_points": intensity["retro_points"],
        "retro_fraction": intensity["retro_fraction"],
        "intensity_median": intensity["intensity_median"],
        "intensity_iqr": intensity["intensity_iqr"],
        "intensity_mad": intensity["intensity_mad"],
        "facade_candidate": int(not reasons),
        "facade_quality_reasons": ";".join(reasons) or "eligible",
        "subset_a_points": len(subset_a_indices),
        "subset_a_intensity_median": subset_a_intensity["intensity_median"],
        "subset_b_points": len(subset_b_indices),
        "subset_b_intensity_median": subset_b_intensity["intensity_median"],
    }
    output.update(repeatability)
    output["stable_facade_patch"] = int(
        not reasons
        and repeatability["geometry_repeatable"] == 1
        and repeatability["reflectivity_repeatable"] == 1
    )
    return output


def inventory_tile(tile_index, tile, args):
    all_groups = grouped_indices(tile["all_xyz"], args.cell_size)
    subset_a_groups = grouped_indices(tile["subset_a_xyz"], args.cell_size)
    subset_b_groups = grouped_indices(tile["subset_b_xyz"], args.cell_size)
    rows = []
    rejected = Counter()
    cells_min_points = 0
    for key in sorted(all_groups):
        indices = all_groups[key]
        if len(indices) < args.cell_min_points:
            continue
        cells_min_points += 1
        geometry = fit_patch_geometry(tile["all_xyz"][indices])
        if geometry is None:
            rejected["invalid_geometry"] += 1
            continue
        intensity = intensity_statistics(
            tile["all_intensity"][indices], args.diffuse_intensity_max
        )
        reasons = facade_quality_reasons(geometry, intensity, args)
        rejected.update(reasons)

        subset_a_indices = subset_a_groups.get(key, np.empty(0, dtype=np.int64))
        subset_b_indices = subset_b_groups.get(key, np.empty(0, dtype=np.int64))
        subset_a_geometry = (
            fit_patch_geometry(tile["subset_a_xyz"][subset_a_indices])
            if len(subset_a_indices) >= args.half_min_points else None
        )
        subset_b_geometry = (
            fit_patch_geometry(tile["subset_b_xyz"][subset_b_indices])
            if len(subset_b_indices) >= args.half_min_points else None
        )
        subset_a_intensity = intensity_statistics(
            tile["subset_a_intensity"][subset_a_indices],
            args.diffuse_intensity_max
        )
        subset_b_intensity = intensity_statistics(
            tile["subset_b_intensity"][subset_b_indices],
            args.diffuse_intensity_max
        )
        repeatability = evaluate_repeatability(
            subset_a_geometry, subset_b_geometry,
            subset_a_intensity, subset_b_intensity, args
        )
        rows.append(serialise_patch(
            tile_index, key, indices, geometry, intensity, reasons,
            subset_a_indices, subset_a_geometry, subset_a_intensity,
            subset_b_indices, subset_b_geometry, subset_b_intensity,
            repeatability,
        ))
    return rows, cells_min_points, rejected


def write_csv(path, rows):
    if not rows:
        raise ValueError("Cannot write empty CSV")
    fields = []
    for row in rows:
        for field in row:
            if field not in fields:
                fields.append(field)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def write_plot(path, tile_rows, patch_rows):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False, "matplotlib_not_installed"
    repeated = [row for row in patch_rows
                if row["subset_a_intensity_median"] is not None
                and row["subset_b_intensity_median"] is not None
                and row["facade_candidate"] == 1]
    figure, axes = plt.subplots(1, 2, figsize=(12, 4.8))
    x = np.arange(len(tile_rows))
    axes[0].bar(x - 0.2, [row["facade_candidates"] for row in tile_rows],
                width=0.4, label="facade candidates")
    axes[0].bar(x + 0.2, [row["stable_facade_patches"] for row in tile_rows],
                width=0.4, label="stable patches")
    axes[0].set_xticks(x)
    axes[0].set_xticklabels([row["tile_window_index"] for row in tile_rows])
    axes[0].set_xlabel("5 s tile index")
    axes[0].set_ylabel("patch count")
    axes[0].legend()
    if repeated:
        subset_a = [row["subset_a_intensity_median"] for row in repeated]
        subset_b = [row["subset_b_intensity_median"] for row in repeated]
        maximum = max(subset_a + subset_b)
        axes[1].scatter(subset_a, subset_b, s=12, alpha=0.55)
        axes[1].plot([0, maximum], [0, maximum], "k--", linewidth=1)
    axes[1].set_xlabel("interleaved subset A intensity median")
    axes[1].set_ylabel("interleaved subset B intensity median")
    for axis in axes:
        axis.grid(True, alpha=0.25)
    figure.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    figure.savefig(path, dpi=160)
    plt.close(figure)
    return True, None


def main():
    args = parse_args()
    positive = [
        args.surface_voxel, args.cell_size, args.plane_residual_median_max,
        args.plane_residual_p95_max, args.horizontal_span_min,
        args.vertical_span_min, args.normal_repeat_max,
        args.plane_offset_repeat_max, args.intensity_repeat_absolute,
        args.intensity_repeat_relative,
    ]
    if any(value <= 0.0 for value in positive):
        raise ValueError("Distance/quality parameters must be positive")
    if not 0.0 < args.vertical_normal_z_max < 1.0:
        raise ValueError("vertical-normal-z screen must be in (0,1)")
    if args.cell_min_points < 3 or args.half_min_points < 3:
        raise ValueError("Point-count screens must be at least three")

    tile_ids = read_tile_ids(args.rays)
    windows, anchor_audit = read_anchor_windows(args.anchor_audit)
    print("=== UrbanV2X facade reflectivity inventory ===")
    print("selected tiles: %d (%s)" % (
        len(tile_ids), ",".join(str(value) for value in tile_ids)
    ))
    patch_rows = []
    tile_rows = []
    all_rejections = Counter()
    for sequence, tile_index in enumerate(tile_ids, 1):
        window = windows.get(tile_index)
        if window is None:
            raise ValueError("Tile %d missing from anchor audit" % tile_index)
        if window.get("anchor_decision") != "PASS":
            raise ValueError("Selected tile %d is not anchor PASS" % tile_index)
        print("Processing tile %d (%d/%d)..." % (
            tile_index, sequence, len(tile_ids)
        ))
        tile = load_tile_splits(window, args.pcd_dir, args.surface_voxel)
        rows, cells_min_points, rejections = inventory_tile(
            tile_index, tile, args
        )
        patch_rows.extend(rows)
        all_rejections.update(rejections)
        candidates = [row for row in rows if row["facade_candidate"] == 1]
        geometry_repeatable = [
            row for row in candidates if row["geometry_repeatable"] == 1
        ]
        reflectivity_repeatable = [
            row for row in candidates if row["reflectivity_repeatable"] == 1
        ]
        stable = [row for row in candidates if row["stable_facade_patch"] == 1]
        tile_rows.append({
            "tile_window_index": tile_index,
            "start_elapsed_s": window["start_elapsed_s"],
            "end_elapsed_s": window["end_elapsed_s"],
            "pcd_files": len(tile["paths"]),
            "subset_a_pcd_files": len(tile["paths_a"]),
            "subset_b_pcd_files": len(tile["paths_b"]),
            "input_points": tile["input_points"],
            "surface_voxels": len(tile["all_xyz"]),
            "cells_with_min_points": cells_min_points,
            "inventory_rows": len(rows),
            "facade_candidates": len(candidates),
            "geometry_repeatable": len(geometry_repeatable),
            "reflectivity_repeatable": len(reflectivity_repeatable),
            "stable_facade_patches": len(stable),
            "stable_ratio_of_candidates": (
                len(stable) / len(candidates) if candidates else None
            ),
        })
        print("  voxels=%d cells=%d facade=%d geom-repeat=%d I-repeat=%d stable=%d" % (
            len(tile["all_xyz"]), cells_min_points, len(candidates),
            len(geometry_repeatable), len(reflectivity_repeatable), len(stable),
        ))
        del tile

    write_csv(args.out_patches, patch_rows)
    write_csv(args.out_tiles, tile_rows)
    candidates = [row for row in patch_rows if row["facade_candidate"] == 1]
    stable = [row for row in patch_rows if row["stable_facade_patch"] == 1]
    tiles_with_five = sum(row["stable_facade_patches"] >= 5 for row in tile_rows)
    enough = len(stable) >= 100 and tiles_with_five >= max(1, math.ceil(0.67 * len(tile_rows)))
    decision = (
        "FACADE_REFLECTIVITY_PATCHES_AVAILABLE"
        if enough else "REVIEW_FACADE_PATCH_YIELD"
    )
    plot_written = False
    plot_error = None
    if args.out_plot:
        plot_written, plot_error = write_plot(
            args.out_plot, tile_rows, patch_rows
        )
    audit = {
        "schema_version": 1,
        "decision": decision,
        "inputs": {
            "rays": os.path.abspath(args.rays),
            "anchor_audit": os.path.abspath(args.anchor_audit),
            "pcd_dir": os.path.abspath(args.pcd_dir),
        },
        "parameters": {
            "surface_voxel_m": args.surface_voxel,
            "cell_size_m": args.cell_size,
            "cell_min_points": args.cell_min_points,
            "half_min_points": args.half_min_points,
            "vertical_normal_z_max": args.vertical_normal_z_max,
            "planarity_min": args.planarity_min,
            "plane_residual_median_max_m": args.plane_residual_median_max,
            "plane_residual_p95_max_m": args.plane_residual_p95_max,
            "horizontal_span_min_m": args.horizontal_span_min,
            "vertical_span_min_m": args.vertical_span_min,
            "diffuse_points_min": args.diffuse_points_min,
            "diffuse_intensity_max": args.diffuse_intensity_max,
            "normal_repeat_max_deg": args.normal_repeat_max,
            "plane_offset_repeat_max_m": args.plane_offset_repeat_max,
            "intensity_repeat_absolute": args.intensity_repeat_absolute,
            "intensity_repeat_relative": args.intensity_repeat_relative,
        },
        "thresholds_are_pre_cn0_project_screens_not_field_standards": {
            "stable_patches_min": 100,
            "tiles_with_at_least_five_stable_min": math.ceil(0.67 * len(tile_rows)),
        },
        "results": {
            "tiles": len(tile_rows),
            "inventory_rows": len(patch_rows),
            "facade_candidates": len(candidates),
            "geometry_repeatable_candidates": sum(
                row["geometry_repeatable"] == 1 for row in candidates
            ),
            "reflectivity_repeatable_candidates": sum(
                row["reflectivity_repeatable"] == 1 for row in candidates
            ),
            "stable_facade_patches": len(stable),
            "stable_ratio_of_candidates": (
                len(stable) / len(candidates) if candidates else None
            ),
            "tiles_with_at_least_five_stable": tiles_with_five,
            "stable_by_tile": {
                str(row["tile_window_index"]): row["stable_facade_patches"]
                for row in tile_rows
            },
            "stable_intensity_median": describe([
                row["intensity_median"] for row in stable
            ]),
            "stable_intensity_iqr": describe([
                row["intensity_iqr"] for row in stable
            ]),
            "stable_normal_repeat_angle_deg": describe([
                row["normal_repeat_angle_deg"] for row in stable
            ]),
            "stable_plane_offset_repeat_m": describe([
                row["plane_offset_repeat_m"] for row in stable
            ]),
            "stable_intensity_repeat_abs": describe([
                row["intensity_repeat_abs"] for row in stable
            ]),
            "stable_intensity_repeat_relative": describe([
                row["intensity_repeat_relative"] for row in stable
            ]),
        },
        "candidate_rejection_reasons": dict(all_rejections.most_common()),
        "upstream_anchor_decision": anchor_audit.get("decision"),
        "interpretation_guards": [
            "No C/N0 field is read or used",
            "Intensity is a voxel-density-normalised LiDAR proxy, not calibrated material reflectance",
            "A/B repeatability uses interleaved scans with similar trajectory coverage and is not long-term material stability",
            "A fixed grid can split one physical facade across multiple inventory rows",
            "Vegetation and dynamic objects can occasionally satisfy local plane screens",
            "Stable facade patches are candidates for exposure modelling, not LOS/NLOS labels",
            "Cross-tile facade deduplication is intentionally deferred",
        ],
        "recommended_next_step": (
            "cluster_stable_cells_into_facade_planes_and_build_continuous_exposure"
            if enough else "retain_as_feasibility_boundary_and_prioritise_controlled_collection"
        ),
        "outputs": {
            "patches_csv": os.path.abspath(args.out_patches),
            "tiles_csv": os.path.abspath(args.out_tiles),
            "plot": os.path.abspath(args.out_plot) if args.out_plot else None,
            "plot_written": plot_written,
            "plot_error": plot_error,
        },
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.audit)), exist_ok=True)
    with open(args.audit, "w", encoding="utf-8") as handle:
        json.dump(audit, handle, indent=2, sort_keys=True)
        handle.write("\n")

    print("\n=== Facade reflectivity inventory summary ===")
    print("tiles:                         %d" % len(tile_rows))
    print("inventory rows:                %d" % len(patch_rows))
    print("facade candidates:             %d" % len(candidates))
    print("geometry-repeatable candidates:%d" % audit["results"][
        "geometry_repeatable_candidates"
    ])
    print("intensity-repeatable candidates:%d" % audit["results"][
        "reflectivity_repeatable_candidates"
    ])
    print("stable facade patches:         %d (%.1f%% of candidates)" % (
        len(stable), 100.0 * len(stable) / len(candidates) if candidates else 0.0
    ))
    print("tiles with >=5 stable:         %d / %d" % (
        tiles_with_five, len(tile_rows)
    ))
    print("stable intensity med p05/med/p95: %s / %s / %s" % (
        audit["results"]["stable_intensity_median"]["p05"],
        audit["results"]["stable_intensity_median"]["median"],
        audit["results"]["stable_intensity_median"]["p95"],
    ))
    print("stable normal repeat med/p95:  %s / %s deg" % (
        audit["results"]["stable_normal_repeat_angle_deg"]["median"],
        audit["results"]["stable_normal_repeat_angle_deg"]["p95"],
    ))
    print("stable intensity repeat med/p95: %s / %s" % (
        audit["results"]["stable_intensity_repeat_abs"]["median"],
        audit["results"]["stable_intensity_repeat_abs"]["p95"],
    ))
    print("top rejection reasons:")
    for reason, count in all_rejections.most_common(10):
        print("  %-28s %d" % (reason, count))
    print("decision:                      %s" % decision)
    print("recommended next step:         %s" % audit["recommended_next_step"])
    print("Patches CSV: %s" % args.out_patches)
    print("Tiles CSV:   %s" % args.out_tiles)
    print("Audit:       %s" % args.audit)
    if args.out_plot:
        print("Plot:        %s (%s)" % (
            args.out_plot, "written" if plot_written else plot_error
        ))
    print("Caution: this is a pre-C/N0 facade candidate inventory.")


if __name__ == "__main__":
    main()
