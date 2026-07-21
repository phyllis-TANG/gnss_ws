#!/usr/bin/env python3
"""Audit local LiDAR surface hits for eligible UrbanV2X satellite rays.

This stage deliberately stops before fitting a reflectivity--C/N0 model.  It
uses only PRIMARY signal rows, deduplicates frequency channels to one
epoch--satellite ray, anchors each selected FAST-LIO tile to the GT local ENU
frame, and asks whether that finite local tile contains a supported surface
near the satellite ray.  A ray without a hit is ``LOCAL_MAP_UNOBSERVED`` and
must not be interpreted as line of sight.

For supported hits, a local PCA plane and density-normalised (voxelised)
intensity patch are reported.  The output is an audit of coordinate
consistency, geometric coverage, surface-normal availability, and patch sample
size; it is not a physical attenuation calibration.
"""

import argparse
import csv
import json
import math
import os
import re
import sys
from collections import defaultdict

import numpy as np


SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIRECTORY not in sys.path:
    sys.path.insert(0, SCRIPT_DIRECTORY)

from urbanv2x_fastlio_map_audit import parse_pcd_header, percentile  # noqa: E402
from urbanv2x_lidar_gt_audit import read_ground_truth  # noqa: E402
from urbanv2x_map_consistency import ecef_to_local_enu  # noqa: E402


def parse_args():
    parser = argparse.ArgumentParser(
        description="Audit GT-anchored UrbanV2X local reflection surfaces"
    )
    parser.add_argument("--eligibility", required=True)
    parser.add_argument("--anchor-audit", required=True)
    parser.add_argument("--gt", required=True)
    parser.add_argument("--pcd-dir", required=True)
    parser.add_argument("--out-rays", required=True)
    parser.add_argument("--out-tiles", required=True)
    parser.add_argument("--audit", required=True)
    parser.add_argument("--out-plot")
    parser.add_argument(
        "--max-tiles", type=int, default=12,
        help="Evenly spaced PRIMARY PASS tiles to audit; 0 means all",
    )
    parser.add_argument("--surface-voxel", type=float, default=0.20)
    parser.add_argument("--ray-start", type=float, default=3.0)
    parser.add_argument("--ray-max-range", type=float, default=60.0)
    parser.add_argument("--ray-step", type=float, default=0.25)
    parser.add_argument("--ray-radius", type=float, default=0.40)
    parser.add_argument("--ray-min-support", type=int, default=3)
    parser.add_argument("--patch-radius", type=float, default=0.80)
    parser.add_argument("--patch-slab", type=float, default=0.20)
    parser.add_argument("--patch-min-points", type=int, default=8)
    parser.add_argument("--diffuse-intensity-max", type=float, default=100.0)
    return parser.parse_args()


def azel_to_enu(azimuth_deg, elevation_deg):
    azimuth = math.radians(float(azimuth_deg))
    elevation = math.radians(float(elevation_deg))
    cosine = math.cos(elevation)
    return np.array([
        math.sin(azimuth) * cosine,
        math.cos(azimuth) * cosine,
        math.sin(elevation),
    ], dtype=np.float64)


def scan_number(name):
    match = re.fullmatch(r"scans_(\d+)\.pcd", os.path.basename(name))
    if not match:
        raise ValueError("Unexpected per-scan PCD name: %s" % name)
    return int(match.group(1))


def pcd_range(first_name, last_name, pcd_dir):
    first = scan_number(first_name)
    last = scan_number(last_name)
    if last < first:
        raise ValueError("Reversed PCD range: %s to %s" % (first_name, last_name))
    paths = [
        os.path.join(pcd_dir, "scans_%d.pcd" % index)
        for index in range(first, last + 1)
    ]
    missing = [path for path in paths if not os.path.isfile(path)]
    if missing:
        raise ValueError("Missing tile PCD files, first: %s" % missing[0])
    return paths


def select_representative_tiles(tile_ids, maximum):
    values = sorted(set(int(value) for value in tile_ids))
    if maximum == 0 or len(values) <= maximum:
        return values
    if maximum < 1:
        raise ValueError("--max-tiles must be nonnegative")
    positions = np.linspace(0, len(values) - 1, maximum)
    selected = []
    for position in positions:
        value = values[int(round(float(position)))]
        if value not in selected:
            selected.append(value)
    if len(selected) < maximum:
        for value in values:
            if value not in selected:
                selected.append(value)
            if len(selected) == maximum:
                break
    return sorted(selected)


def read_primary_rays(path):
    grouped = defaultdict(list)
    with open(path, "r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        fields = set(reader.fieldnames or [])
        required = {
            "epoch_utc", "sat_id", "sys", "analysis_group",
            "tile_window_index", "azimuth_deg", "elevation_deg",
            "rx_x_ecef_m", "rx_y_ecef_m", "rx_z_ecef_m",
        }
        missing = sorted(required - fields)
        if missing:
            raise ValueError("Eligibility CSV is missing fields: %s" % missing)
        for row in reader:
            if row["analysis_group"] != "PRIMARY":
                continue
            key = (round(float(row["epoch_utc"]), 6), row["sat_id"])
            grouped[key].append(row)
    rays = []
    for (epoch, sat_id), rows in sorted(grouped.items()):
        first = rows[0]
        invariant = (
            int(first["tile_window_index"]), first["sys"],
            float(first["azimuth_deg"]), float(first["elevation_deg"]),
            float(first["rx_x_ecef_m"]), float(first["rx_y_ecef_m"]),
            float(first["rx_z_ecef_m"]),
        )
        for row in rows[1:]:
            candidate = (
                int(row["tile_window_index"]), row["sys"],
                float(row["azimuth_deg"]), float(row["elevation_deg"]),
                float(row["rx_x_ecef_m"]), float(row["rx_y_ecef_m"]),
                float(row["rx_z_ecef_m"]),
            )
            if candidate[:2] != invariant[:2] or not np.allclose(
                candidate[2:], invariant[2:], rtol=0.0, atol=1e-5
            ):
                raise ValueError("Geometry differs across signals for %s %s" % key)
        rays.append({
            "epoch_utc": epoch,
            "sat_id": sat_id,
            "sys": invariant[1],
            "tile_window_index": invariant[0],
            "azimuth_deg": invariant[2],
            "elevation_deg": invariant[3],
            "rx_ecef": np.asarray(invariant[4:7], dtype=np.float64),
            "signal_records": len(rows),
            "signals": ";".join(sorted(set(row.get("signal", "") for row in rows))),
        })
    if not rays:
        raise ValueError("No PRIMARY epoch-satellite rays found")
    return rays


def read_anchor_windows(path):
    with open(path, "r", encoding="utf-8") as handle:
        audit = json.load(handle)
    windows = {}
    for index, row in enumerate(audit.get("windows", [])):
        window_index = int(row.get("window_index", index))
        windows[window_index] = row
    if not windows:
        raise ValueError("Anchor audit contains no windows")
    return windows, audit


def read_pcd_points(path):
    metadata = parse_pcd_header(path)
    required = {"x", "y", "z", "intensity"}
    missing = sorted(required - set(metadata["fields"]))
    if missing:
        raise ValueError("PCD is missing fields %s: %s" % (missing, path))
    if not metadata["payload_exact"]:
        raise ValueError("PCD payload mismatch: %s" % path)
    array = np.memmap(
        path, mode="r", dtype=metadata["dtype"],
        offset=metadata["data_offset"], shape=(metadata["points"],),
    )
    valid = (
        np.isfinite(array["x"]) & np.isfinite(array["y"])
        & np.isfinite(array["z"]) & np.isfinite(array["intensity"])
    )
    xyz = np.column_stack([
        array["x"][valid], array["y"][valid], array["z"][valid]
    ]).astype(np.float32, copy=False)
    intensity = np.asarray(array["intensity"][valid], dtype=np.float32)
    return xyz, intensity


def voxelise_surface(xyz, intensity, voxel):
    """Return one density-normalised centroid and mean intensity per voxel."""
    if len(xyz) == 0:
        return xyz.astype(np.float32), intensity.astype(np.float32)
    keys = np.floor(np.asarray(xyz, dtype=np.float64) / voxel).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    counts = np.bincount(inverse).astype(np.float64)
    centroids = np.column_stack([
        np.bincount(inverse, weights=xyz[:, axis]) / counts
        for axis in range(3)
    ])
    mean_intensity = np.bincount(inverse, weights=intensity) / counts
    return centroids.astype(np.float32), mean_intensity.astype(np.float32)


def load_anchored_tile(window, pcd_dir, surface_voxel):
    xyz_parts = []
    intensity_parts = []
    input_points = 0
    paths = pcd_range(window["first_pcd"], window["last_pcd"], pcd_dir)
    for path in paths:
        xyz, intensity = read_pcd_points(path)
        input_points += len(xyz)
        xyz_parts.append(xyz)
        intensity_parts.append(intensity)
    xyz = np.concatenate(xyz_parts, axis=0)
    intensity = np.concatenate(intensity_parts, axis=0)
    rotation = np.asarray(window["alignment_rotation"], dtype=np.float64)
    translation = np.asarray(window["alignment_translation_m"], dtype=np.float64)
    xyz = ((rotation @ xyz.T).T + translation).astype(np.float32)
    xyz, intensity = voxelise_surface(xyz, intensity, surface_voxel)
    return xyz, intensity, input_points, len(paths)


def trace_local_ray(tree, xyz, origin, direction, start, maximum, step,
                    radius, min_support):
    distances = np.arange(start, maximum + 0.5 * step, step)
    samples = origin[None, :] + distances[:, None] * direction[None, :]
    nearest_distance, nearest_index = tree.query(
        samples, k=1, distance_upper_bound=radius
    )
    for sample_index in np.flatnonzero(np.isfinite(nearest_distance)):
        neighbours = tree.query_ball_point(samples[sample_index], r=radius)
        if len(neighbours) < min_support:
            continue
        neighbours = np.asarray(neighbours, dtype=np.int64)
        offsets = xyz[neighbours].astype(np.float64) - origin
        axial = offsets @ direction
        perpendicular = np.linalg.norm(
            offsets - axial[:, None] * direction[None, :], axis=1
        )
        valid = (
            (axial >= start) & (axial <= maximum)
            & (perpendicular <= radius)
        )
        if np.count_nonzero(valid) < min_support:
            continue
        valid_indices = neighbours[valid]
        valid_axial = axial[valid]
        valid_perpendicular = perpendicular[valid]
        order = np.lexsort((valid_perpendicular, valid_axial))
        best = int(valid_indices[order[0]])
        return {
            "index": best,
            "axial_distance_m": float(valid_axial[order[0]]),
            "perpendicular_distance_m": float(valid_perpendicular[order[0]]),
            "support": int(np.count_nonzero(valid)),
        }
    return None


def fit_surface_patch(xyz, intensity, indices, hit, ray_direction,
                      slab, diffuse_max, minimum):
    points = np.asarray(xyz[indices], dtype=np.float64)
    values = np.asarray(intensity[indices], dtype=np.float64)
    if len(points) < minimum:
        return None
    centred = points - np.mean(points, axis=0)
    covariance = centred.T @ centred / max(len(points) - 1, 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    if not np.all(np.isfinite(eigenvalues)) or eigenvalues[2] <= 0.0:
        return None
    normal = eigenvectors[:, 0]
    if np.dot(normal, -ray_direction) < 0.0:
        normal = -normal
    plane_distance = np.abs((points - hit) @ normal)
    planar = plane_distance <= slab
    diffuse = planar & np.isfinite(values) & (values > 0.0) & (values <= diffuse_max)
    diffuse_values = values[diffuse]
    retro = planar & np.isfinite(values) & (values > diffuse_max)
    incidence = math.degrees(math.acos(np.clip(abs(np.dot(
        ray_direction, normal
    )), 0.0, 1.0)))
    output = {
        "normal_x": float(normal[0]),
        "normal_y": float(normal[1]),
        "normal_z": float(normal[2]),
        "eigenvalue_0": float(eigenvalues[0]),
        "eigenvalue_1": float(eigenvalues[1]),
        "eigenvalue_2": float(eigenvalues[2]),
        "planarity": float((eigenvalues[1] - eigenvalues[0]) / eigenvalues[2]),
        "plane_residual_median_m": float(np.median(plane_distance)),
        "incidence_deg": incidence,
        "patch_points": int(len(points)),
        "patch_planar_points": int(np.count_nonzero(planar)),
        "patch_diffuse_points": int(len(diffuse_values)),
        "patch_retro_points": int(np.count_nonzero(retro)),
        "patch_retro_fraction": (
            float(np.count_nonzero(retro) / np.count_nonzero(planar))
            if np.count_nonzero(planar) else None
        ),
        "patch_intensity_median": None,
        "patch_intensity_iqr": None,
        "patch_intensity_mad": None,
    }
    if len(diffuse_values):
        median = float(np.median(diffuse_values))
        output.update({
            "patch_intensity_median": median,
            "patch_intensity_iqr": float(
                np.percentile(diffuse_values, 75) - np.percentile(diffuse_values, 25)
            ),
            "patch_intensity_mad": float(np.median(np.abs(diffuse_values - median))),
        })
    return output


def describe(values):
    clean = np.asarray([value for value in values if value is not None], dtype=float)
    clean = clean[np.isfinite(clean)]
    if not len(clean):
        return {"n": 0, "median": None, "p05": None, "p95": None}
    return {
        "n": int(len(clean)),
        "median": float(np.median(clean)),
        "p05": percentile(clean, 0.05),
        "p95": percentile(clean, 0.95),
    }


def write_csv(path, rows):
    if not rows:
        raise ValueError("Cannot write empty CSV")
    fields = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def write_plot(path, tile_rows):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False, "matplotlib_not_installed"
    x = [row["tile_window_index"] for row in tile_rows]
    hit = [row["hit_ratio"] for row in tile_rows]
    patch = [row["patch_valid_ratio"] for row in tile_rows]
    figure, axis = plt.subplots(figsize=(11, 4.5))
    axis.plot(x, hit, marker="o", label="supported surface hit")
    axis.plot(x, patch, marker="s", label="valid diffuse patch")
    axis.set_xlabel("5 s tile index")
    axis.set_ylabel("ratio among unique PRIMARY rays")
    axis.set_ylim(0.0, 1.02)
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    figure.savefig(path, dpi=160)
    plt.close(figure)
    return True, None


def main():
    args = parse_args()
    positive = {
        "surface_voxel": args.surface_voxel,
        "ray_start": args.ray_start,
        "ray_max_range": args.ray_max_range,
        "ray_step": args.ray_step,
        "ray_radius": args.ray_radius,
        "patch_radius": args.patch_radius,
        "patch_slab": args.patch_slab,
    }
    if any(value <= 0.0 for value in positive.values()):
        raise ValueError("Distance parameters must be positive: %s" % positive)
    if args.ray_start >= args.ray_max_range:
        raise ValueError("Ray start must be smaller than maximum range")
    if args.ray_min_support < 1 or args.patch_min_points < 3:
        raise ValueError("Support thresholds are too small")
    try:
        from scipy.spatial import cKDTree
    except ImportError as exc:
        raise RuntimeError("scipy.spatial.cKDTree is required") from exc

    print("Reading PRIMARY epoch-satellite rays...")
    all_rays = read_primary_rays(args.eligibility)
    windows, anchor_audit = read_anchor_windows(args.anchor_audit)
    eligible_tiles = sorted(set(row["tile_window_index"] for row in all_rays))
    selected_tiles = select_representative_tiles(eligible_tiles, args.max_tiles)
    selected_set = set(selected_tiles)
    rays = [row for row in all_rays if row["tile_window_index"] in selected_set]
    print("  PRIMARY signal rows represented: %d" % sum(
        row["signal_records"] for row in rays
    ))
    print("  unique rays: %d; selected tiles: %d/%d" % (
        len(rays), len(selected_tiles), len(eligible_tiles)
    ))

    gt = read_ground_truth(args.gt)
    reference = gt[0]
    rays_by_tile = defaultdict(list)
    for ray in rays:
        ray["rx_enu"] = ecef_to_local_enu(
            ray["rx_ecef"][None, :], reference["ecef"],
            reference["latitude"], reference["longitude"],
        )[0]
        rays_by_tile[ray["tile_window_index"]].append(ray)

    ray_rows = []
    tile_rows = []
    for sequence, tile_index in enumerate(selected_tiles, 1):
        window = windows.get(tile_index)
        if window is None:
            raise ValueError("Tile %d missing from anchor audit" % tile_index)
        if window.get("anchor_decision") != "PASS":
            raise ValueError("PRIMARY tile %d is not PASS" % tile_index)
        print("Processing tile %d (%d/%d)..." % (
            tile_index, sequence, len(selected_tiles)
        ))
        xyz, intensity, input_points, pcd_files = load_anchored_tile(
            window, args.pcd_dir, args.surface_voxel
        )
        tree = cKDTree(xyz)
        tile_ray_rows = []
        receiver_nearest = []
        for ray in rays_by_tile[tile_index]:
            origin = ray["rx_enu"]
            nearest = float(tree.query(origin, k=1)[0])
            receiver_nearest.append(nearest)
            direction = azel_to_enu(ray["azimuth_deg"], ray["elevation_deg"])
            hit = trace_local_ray(
                tree, xyz, origin, direction, args.ray_start,
                args.ray_max_range, args.ray_step, args.ray_radius,
                args.ray_min_support,
            )
            output = {
                "epoch_utc": "%.6f" % ray["epoch_utc"],
                "sat_id": ray["sat_id"],
                "sys": ray["sys"],
                "signal_records": ray["signal_records"],
                "signals": ray["signals"],
                "tile_window_index": tile_index,
                "azimuth_deg": ray["azimuth_deg"],
                "elevation_deg": ray["elevation_deg"],
                "receiver_e_m": float(origin[0]),
                "receiver_n_m": float(origin[1]),
                "receiver_u_m": float(origin[2]),
                "receiver_nearest_surface_m": nearest,
                "surface_status": "LOCAL_MAP_UNOBSERVED",
                "surface_hit": 0,
                "hit_distance_m": None,
                "hit_perpendicular_m": None,
                "hit_support": 0,
                "hit_e_m": None,
                "hit_n_m": None,
                "hit_u_m": None,
                "normal_available": 0,
                "patch_valid": 0,
            }
            if hit is not None:
                hit_xyz = xyz[hit["index"]].astype(np.float64)
                output.update({
                    "surface_status": "SUPPORTED_HIT",
                    "surface_hit": 1,
                    "hit_distance_m": hit["axial_distance_m"],
                    "hit_perpendicular_m": hit["perpendicular_distance_m"],
                    "hit_support": hit["support"],
                    "hit_e_m": float(hit_xyz[0]),
                    "hit_n_m": float(hit_xyz[1]),
                    "hit_u_m": float(hit_xyz[2]),
                })
                neighbours = tree.query_ball_point(hit_xyz, r=args.patch_radius)
                patch = fit_surface_patch(
                    xyz, intensity, np.asarray(neighbours, dtype=np.int64),
                    hit_xyz, direction, args.patch_slab,
                    args.diffuse_intensity_max, args.patch_min_points,
                )
                if patch is not None:
                    output.update(patch)
                    output["normal_available"] = 1
                    output["patch_valid"] = int(
                        patch["patch_diffuse_points"] >= args.patch_min_points
                    )
            ray_rows.append(output)
            tile_ray_rows.append(output)

        hits = sum(row["surface_hit"] for row in tile_ray_rows)
        valid_patches = sum(row["patch_valid"] for row in tile_ray_rows)
        tile_rows.append({
            "tile_window_index": tile_index,
            "start_elapsed_s": window["start_elapsed_s"],
            "end_elapsed_s": window["end_elapsed_s"],
            "rays": len(tile_ray_rows),
            "signal_records": sum(row["signal_records"] for row in tile_ray_rows),
            "pcd_files": pcd_files,
            "input_points": input_points,
            "surface_voxels": len(xyz),
            "receiver_nearest_median_m": float(np.median(receiver_nearest)),
            "receiver_nearest_p95_m": percentile(receiver_nearest, 0.95),
            "surface_hits": hits,
            "hit_ratio": hits / len(tile_ray_rows),
            "normal_available": sum(
                row["normal_available"] for row in tile_ray_rows
            ),
            "valid_patches": valid_patches,
            "patch_valid_ratio": valid_patches / len(tile_ray_rows),
        })
        print("  rays=%d voxels=%d rx-nearest-med=%.2fm hit=%d patch=%d" % (
            len(tile_ray_rows), len(xyz), tile_rows[-1]["receiver_nearest_median_m"],
            hits, valid_patches,
        ))
        del tree, xyz, intensity

    write_csv(args.out_rays, ray_rows)
    write_csv(args.out_tiles, tile_rows)
    hits = [row for row in ray_rows if row["surface_hit"]]
    patches = [row for row in ray_rows if row["patch_valid"]]
    system_rows = []
    for system in sorted(set(row["sys"] for row in ray_rows)):
        subset = [row for row in ray_rows if row["sys"] == system]
        system_rows.append({
            "sys": system,
            "rays": len(subset),
            "hits": sum(row["surface_hit"] for row in subset),
            "patches": sum(row["patch_valid"] for row in subset),
        })
    coordinate_sanity = describe([
        row["receiver_nearest_surface_m"] for row in ray_rows
    ])
    decision = (
        "SURFACE_HITS_AVAILABLE"
        if len(hits) >= 100 and len(patches) >= 50
        else "REVIEW_LOCAL_COVERAGE"
    )
    plot_written = False
    plot_error = None
    if args.out_plot:
        plot_written, plot_error = write_plot(args.out_plot, tile_rows)
    audit = {
        "schema_version": 1,
        "decision": decision,
        "scope": "representative_primary_pass_tiles" if args.max_tiles else "all_primary_pass_tiles",
        "inputs": {
            "eligibility": os.path.abspath(args.eligibility),
            "anchor_audit": os.path.abspath(args.anchor_audit),
            "gt": os.path.abspath(args.gt),
            "pcd_dir": os.path.abspath(args.pcd_dir),
        },
        "parameters": {
            "max_tiles": args.max_tiles,
            "surface_voxel_m": args.surface_voxel,
            "ray_start_m": args.ray_start,
            "ray_max_range_m": args.ray_max_range,
            "ray_step_m": args.ray_step,
            "ray_radius_m": args.ray_radius,
            "ray_min_support": args.ray_min_support,
            "patch_radius_m": args.patch_radius,
            "patch_slab_m": args.patch_slab,
            "patch_min_points": args.patch_min_points,
            "diffuse_intensity_max": args.diffuse_intensity_max,
        },
        "selection": {
            "eligible_tiles": eligible_tiles,
            "selected_tiles": selected_tiles,
            "all_primary_unique_rays": len(all_rays),
            "selected_unique_rays": len(ray_rows),
            "selected_signal_records": sum(row["signal_records"] for row in ray_rows),
        },
        "results": {
            "surface_hits": len(hits),
            "surface_hit_ratio": len(hits) / len(ray_rows),
            "normal_available": sum(row["normal_available"] for row in ray_rows),
            "valid_diffuse_patches": len(patches),
            "patch_valid_ratio": len(patches) / len(ray_rows),
            "receiver_nearest_surface_m": coordinate_sanity,
            "hit_distance_m": describe([row["hit_distance_m"] for row in hits]),
            "patch_diffuse_points": describe([
                row.get("patch_diffuse_points") for row in patches
            ]),
            "patch_intensity_median": describe([
                row.get("patch_intensity_median") for row in patches
            ]),
        },
        "by_constellation": system_rows,
        "tiles": tile_rows,
        "interpretation_guards": [
            "A no-hit ray means LOCAL_MAP_UNOBSERVED, not LOS",
            "Satellite frequency channels are deduplicated for geometric ray casting",
            "Voxel mean intensity is a density-normalised LiDAR proxy, not calibrated material reflectance",
            "Intensity above the configured diffuse threshold is excluded from patch summaries and counted as retroreflective",
            "This audit does not fit or test a reflectivity--C/N0 attenuation relation",
            "FAST-LIO tiles are independently rigid-anchored to GT; cross-tile continuity is not assumed",
            "GT and Xsens reference origins are treated as coincident because their lever arm is undocumented",
        ],
        "upstream_anchor_decision": anchor_audit.get("decision"),
        "outputs": {
            "rays_csv": os.path.abspath(args.out_rays),
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

    print("\n=== UrbanV2X local reflection-surface audit ===")
    print("selected tiles:            %d / %d" % (
        len(selected_tiles), len(eligible_tiles)
    ))
    print("unique rays / signals:     %d / %d" % (
        len(ray_rows), sum(row["signal_records"] for row in ray_rows)
    ))
    print("receiver-nearest med/p95:  %.3f / %.3f m" % (
        coordinate_sanity["median"], coordinate_sanity["p95"]
    ))
    print("surface hits:              %d (%.1f%%)" % (
        len(hits), 100.0 * len(hits) / len(ray_rows)
    ))
    print("normal available:          %d" % sum(
        row["normal_available"] for row in ray_rows
    ))
    print("valid diffuse patches:     %d (%.1f%%)" % (
        len(patches), 100.0 * len(patches) / len(ray_rows)
    ))
    print("by constellation (rays/hits/patches):")
    for row in system_rows:
        print("  %s %d/%d/%d" % (
            row["sys"], row["rays"], row["hits"], row["patches"]
        ))
    print("decision:                  %s" % decision)
    print("Rays CSV:  %s" % args.out_rays)
    print("Tiles CSV: %s" % args.out_tiles)
    print("Audit:     %s" % args.audit)
    if args.out_plot:
        print("Plot:      %s (%s)" % (
            args.out_plot, "written" if plot_written else plot_error
        ))
    print("Caution: no hit means local-map unobserved; it is not a LOS label.")


if __name__ == "__main__":
    main()
