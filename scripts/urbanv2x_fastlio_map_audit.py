#!/usr/bin/env python3
"""Audit chunked FAST-LIO PCD geometry before full-sequence mapping.

The audit deliberately uses only claims supported by the saved chunk maps:

* binary payload/header integrity and finite XYZ/intensity coverage;
* local planar-cell residuals after deterministic sampling/downsampling; and
* adjacent-chunk overlap using bidirectional and mutual nearest neighbours.

Range-banded errors are not reported because the aggregated PCD files no
longer retain each point's originating scan pose. Those require replaying the
raw bag with the odometry trajectory and belong to a later audit.
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

from urbanv2x_map_consistency import describe, planar_cell_residual  # noqa: E402


PCD_SCALAR_DTYPES = {
    ("F", 4): "<f4",
    ("F", 8): "<f8",
    ("I", 1): "<i1",
    ("I", 2): "<i2",
    ("I", 4): "<i4",
    ("I", 8): "<i8",
    ("U", 1): "<u1",
    ("U", 2): "<u2",
    ("U", 4): "<u4",
    ("U", 8): "<u8",
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Audit chunked UrbanV2X FAST-LIO PCD map geometry"
    )
    parser.add_argument(
        "--pcd", nargs="+", required=True,
        help="Ordered FAST-LIO PCD chunks (shell globs may be expanded)",
    )
    parser.add_argument("--out-chunks", required=True)
    parser.add_argument("--out-pairs", required=True)
    parser.add_argument("--audit", required=True)
    parser.add_argument(
        "--max-sample-per-chunk", type=int, default=250000,
        help="Maximum deterministic point sample per chunk (default: 250000)",
    )
    parser.add_argument(
        "--overlap-voxel", type=float, default=0.20,
        help="Voxel size before adjacent-chunk matching (default: 0.20 m)",
    )
    parser.add_argument(
        "--overlap-radius", type=float, default=0.50,
        help="Maximum distance counted as overlap (default: 0.50 m)",
    )
    parser.add_argument(
        "--plane-voxel", type=float, default=0.10,
        help="Voxel size before planar-cell analysis (default: 0.10 m)",
    )
    parser.add_argument(
        "--plane-cell", type=float, default=1.0,
        help="Local planar cell size (default: 1.0 m)",
    )
    parser.add_argument(
        "--plane-min-points", type=int, default=12,
        help="Minimum voxel centroids per planar cell (default: 12)",
    )
    parser.add_argument(
        "--plane-median-screen", type=float, default=0.10,
        help="Project screen for global median planar residual (default: 0.10 m)",
    )
    parser.add_argument(
        "--overlap-median-screen", type=float, default=0.15,
        help="Project screen for each mutual-overlap median (default: 0.15 m)",
    )
    return parser.parse_args()


def _parse_numbers(values, cast):
    try:
        return [cast(value) for value in values]
    except ValueError as exc:
        raise ValueError("Invalid numeric PCD header value") from exc


def parse_pcd_header(path):
    header = {}
    data_offset = None
    with open(path, "rb") as handle:
        while True:
            raw = handle.readline()
            if not raw:
                break
            try:
                line = raw.decode("ascii").strip()
            except UnicodeDecodeError as exc:
                raise ValueError("PCD header is not ASCII: %s" % path) from exc
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            key = parts[0].upper()
            header[key] = parts[1:]
            if key == "DATA":
                data_offset = handle.tell()
                break
    if data_offset is None:
        raise ValueError("PCD DATA declaration is missing: %s" % path)
    required = {"FIELDS", "SIZE", "TYPE", "WIDTH", "HEIGHT", "POINTS", "DATA"}
    missing = sorted(required - set(header))
    if missing:
        raise ValueError("PCD header missing %s: %s" % (missing, path))
    data_type = header["DATA"][0].lower()
    if data_type != "binary":
        raise ValueError("Only uncompressed binary PCD is supported, got %s" % data_type)

    fields = header["FIELDS"]
    sizes = _parse_numbers(header["SIZE"], int)
    types = [value.upper() for value in header["TYPE"]]
    counts = _parse_numbers(header.get("COUNT", ["1"] * len(fields)), int)
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError("PCD FIELDS/SIZE/TYPE/COUNT lengths differ")
    if any(count <= 0 for count in counts):
        raise ValueError("PCD COUNT values must be positive")
    points = int(header["POINTS"][0])
    width = int(header["WIDTH"][0])
    height = int(header["HEIGHT"][0])
    if points < 0 or width * height != points:
        raise ValueError("PCD WIDTH*HEIGHT does not equal POINTS")

    names = []
    formats = []
    offsets = []
    offset = 0
    for field, size, scalar_type, count in zip(fields, sizes, types, counts):
        key = (scalar_type, size)
        if key not in PCD_SCALAR_DTYPES:
            raise ValueError("Unsupported PCD scalar type/size %s" % (key,))
        names.append(field)
        scalar_dtype = np.dtype(PCD_SCALAR_DTYPES[key])
        formats.append(scalar_dtype if count == 1 else (scalar_dtype, (count,)))
        offsets.append(offset)
        offset += size * count
    dtype = np.dtype({
        "names": names,
        "formats": formats,
        "offsets": offsets,
        "itemsize": offset,
    })
    actual_bytes = os.path.getsize(path)
    expected_bytes = data_offset + points * dtype.itemsize
    return {
        "path": os.path.abspath(path),
        "fields": fields,
        "sizes": sizes,
        "types": types,
        "counts": counts,
        "points": points,
        "width": width,
        "height": height,
        "data": data_type,
        "data_offset": data_offset,
        "point_step": dtype.itemsize,
        "dtype": dtype,
        "actual_bytes": actual_bytes,
        "expected_bytes": expected_bytes,
        "payload_exact": actual_bytes == expected_bytes,
    }


def deterministic_indices(total, maximum):
    if maximum <= 0:
        raise ValueError("Maximum sample size must be positive")
    if total <= maximum:
        return np.arange(total, dtype=np.int64)
    step = int(math.ceil(total / maximum))
    return np.arange(0, total, step, dtype=np.int64)[:maximum]


def percentile(values, probability):
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return None
    return float(np.percentile(values, probability * 100.0))


def inspect_chunk(path, max_sample):
    metadata = parse_pcd_header(path)
    required = {"x", "y", "z", "intensity"}
    missing = sorted(required - set(metadata["fields"]))
    if missing:
        raise ValueError("PCD is missing required fields %s: %s" % (missing, path))
    if not metadata["payload_exact"]:
        raise ValueError(
            "PCD payload size mismatch (%d actual, %d expected): %s"
            % (metadata["actual_bytes"], metadata["expected_bytes"], path)
        )

    array = np.memmap(
        path, mode="r", dtype=metadata["dtype"],
        offset=metadata["data_offset"], shape=(metadata["points"],),
    )
    finite_xyz_all = (
        np.isfinite(array["x"]) & np.isfinite(array["y"])
        & np.isfinite(array["z"])
    )
    finite_intensity_all = np.isfinite(array["intensity"])
    indices = deterministic_indices(metadata["points"], max_sample)
    sample = array[indices]
    finite_sample = (
        np.isfinite(sample["x"]) & np.isfinite(sample["y"])
        & np.isfinite(sample["z"]) & np.isfinite(sample["intensity"])
    )
    sample = sample[finite_sample]
    xyz = np.column_stack([sample["x"], sample["y"], sample["z"]]).astype(
        np.float32, copy=False
    )
    intensity = np.asarray(sample["intensity"], dtype=np.float32)
    result = {
        "chunk": os.path.basename(path),
        "path": os.path.abspath(path),
        "file_bytes": metadata["actual_bytes"],
        "points": metadata["points"],
        "point_step": metadata["point_step"],
        "payload_exact": metadata["payload_exact"],
        "fields": " ".join(metadata["fields"]),
        "finite_xyz_ratio": float(np.mean(finite_xyz_all)) if len(array) else None,
        "finite_intensity_ratio": (
            float(np.mean(finite_intensity_all)) if len(array) else None
        ),
        "sample_points": int(len(xyz)),
        "x_min": float(np.min(xyz[:, 0])) if len(xyz) else None,
        "x_max": float(np.max(xyz[:, 0])) if len(xyz) else None,
        "y_min": float(np.min(xyz[:, 1])) if len(xyz) else None,
        "y_max": float(np.max(xyz[:, 1])) if len(xyz) else None,
        "z_min": float(np.min(xyz[:, 2])) if len(xyz) else None,
        "z_max": float(np.max(xyz[:, 2])) if len(xyz) else None,
        "intensity_p05": percentile(intensity, 0.05),
        "intensity_median": percentile(intensity, 0.50),
        "intensity_p95": percentile(intensity, 0.95),
    }
    del array
    return result, xyz, intensity


def voxel_downsample_xyz(xyz, voxel):
    xyz = np.asarray(xyz, dtype=np.float32)
    if len(xyz) == 0:
        return xyz
    if voxel <= 0.0:
        raise ValueError("Voxel size must be positive")
    keys = np.floor(xyz / voxel).astype(np.int32)
    _, inverse, counts = np.unique(
        keys, axis=0, return_inverse=True, return_counts=True
    )
    sums = np.column_stack([
        np.bincount(inverse, weights=xyz[:, axis]) for axis in range(3)
    ])
    return (sums / counts[:, None]).astype(np.float32)


def query_nearest(reference, query):
    """Return nearest-reference distance/index for every query point."""
    try:
        from scipy.spatial import cKDTree
    except ImportError:
        # Small deterministic fallback keeps the core geometry testable in a
        # minimal Python runtime. Real map audits intentionally require the
        # cKDTree path to avoid quadratic work on hundreds of thousands of
        # voxels.
        if len(reference) * len(query) > 5_000_000:
            raise RuntimeError(
                "scipy.spatial.cKDTree is required for production-size overlap"
            )
        distances = []
        indices = []
        for start in range(0, len(query), 256):
            block = query[start:start + 256]
            squared = np.sum(
                (block[:, None, :] - reference[None, :, :]) ** 2, axis=2
            )
            nearest = np.argmin(squared, axis=1)
            distances.append(np.sqrt(squared[np.arange(len(block)), nearest]))
            indices.append(nearest)
        return np.concatenate(distances), np.concatenate(indices)
    tree = cKDTree(reference)
    return tree.query(query, k=1)


def adjacent_overlap(left_xyz, right_xyz, voxel, radius):
    left = voxel_downsample_xyz(left_xyz, voxel)
    right = voxel_downsample_xyz(right_xyz, voxel)
    if len(left) < 20 or len(right) < 20:
        raise ValueError("Too few points for adjacent overlap")
    left_to_right, left_indices = query_nearest(right, left)
    right_to_left, right_indices = query_nearest(left, right)
    left_within = left_to_right <= radius
    right_within = right_to_left <= radius
    left_ids = np.arange(len(left), dtype=np.int64)
    mutual = (
        left_within
        & (right_to_left[left_indices] <= radius)
        & (right_indices[left_indices] == left_ids)
    )
    mutual_distances = left_to_right[mutual]
    return {
        "left_voxels": int(len(left)),
        "right_voxels": int(len(right)),
        "left_within_radius": int(np.count_nonzero(left_within)),
        "right_within_radius": int(np.count_nonzero(right_within)),
        "left_within_radius_ratio": float(np.mean(left_within)),
        "right_within_radius_ratio": float(np.mean(right_within)),
        "mutual_matches": int(len(mutual_distances)),
        "mutual_over_min_voxels_ratio": float(
            len(mutual_distances) / min(len(left), len(right))
        ),
        "mutual_distance_median_m": percentile(mutual_distances, 0.50),
        "mutual_distance_p95_m": percentile(mutual_distances, 0.95),
        "mutual_distance_max_m": (
            float(np.max(mutual_distances)) if len(mutual_distances) else None
        ),
    }


def write_csv(path, rows):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    if not rows:
        raise ValueError("Cannot write an empty CSV")
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def fmt(value, digits=4):
    return "NA" if value is None else ("%%.%df" % digits) % value


def main():
    args = parse_args()
    if len(args.pcd) < 2:
        raise ValueError("At least two ordered PCD chunks are required")
    if args.max_sample_per_chunk <= 0:
        raise ValueError("--max-sample-per-chunk must be positive")
    if min(args.overlap_voxel, args.overlap_radius,
           args.plane_voxel, args.plane_cell) <= 0.0:
        raise ValueError("Voxel, radius, and cell sizes must be positive")

    chunks = []
    xyz_samples = []
    intensity_samples = []
    print("Reading and validating PCD chunks...")
    for index, path in enumerate(args.pcd, 1):
        result, xyz, intensity = inspect_chunk(path, args.max_sample_per_chunk)
        chunks.append(result)
        xyz_samples.append(xyz)
        intensity_samples.append(intensity)
        print("  %d/%d %s: points=%d sample=%d payload=%s" % (
            index, len(args.pcd), result["chunk"], result["points"],
            result["sample_points"], "PASS" if result["payload_exact"] else "FAIL",
        ))

    print("Computing adjacent-chunk overlap...")
    pairs = []
    for index in range(len(chunks) - 1):
        metrics = adjacent_overlap(
            xyz_samples[index], xyz_samples[index + 1],
            args.overlap_voxel, args.overlap_radius,
        )
        row = {
            "left": chunks[index]["chunk"],
            "right": chunks[index + 1]["chunk"],
            **metrics,
        }
        pairs.append(row)
        print("  %s -> %s: mutual=%d median=%s p95=%s m" % (
            row["left"], row["right"], row["mutual_matches"],
            fmt(row["mutual_distance_median_m"]),
            fmt(row["mutual_distance_p95_m"]),
        ))

    print("Computing global planar-cell residuals...")
    global_xyz = np.concatenate(xyz_samples, axis=0)
    planar_xyz = voxel_downsample_xyz(global_xyz, args.plane_voxel)
    plane, plane_cells = planar_cell_residual(
        planar_xyz, coarse_cell=args.plane_cell,
        minimum_points=args.plane_min_points,
    )

    integrity_checks = {
        "all_payloads_exact": all(row["payload_exact"] for row in chunks),
        "all_xyz_finite_ratio_ge_0_999": all(
            row["finite_xyz_ratio"] is not None
            and row["finite_xyz_ratio"] >= 0.999 for row in chunks
        ),
        "all_intensity_finite_ratio_ge_0_999": all(
            row["finite_intensity_ratio"] is not None
            and row["finite_intensity_ratio"] >= 0.999 for row in chunks
        ),
    }
    geometry_checks = {
        "planar_cells_ge_100": plane_cells >= 100,
        "planar_residual_median_le_project_threshold": (
            plane["median"] is not None
            and plane["median"] <= args.plane_median_screen
        ),
        "all_adjacent_pairs_have_mutual_matches": all(
            row["mutual_matches"] >= 100 for row in pairs
        ),
        "all_adjacent_mutual_medians_le_project_threshold": all(
            row["mutual_distance_median_m"] is not None
            and row["mutual_distance_median_m"] <= args.overlap_median_screen
            for row in pairs
        ),
    }
    if all(integrity_checks.values()) and all(geometry_checks.values()):
        decision = "SCREEN_PASS"
    elif all(integrity_checks.values()):
        decision = "GEOMETRY_REVIEW"
    else:
        decision = "INTEGRITY_FAIL"

    write_csv(args.out_chunks, chunks)
    write_csv(args.out_pairs, pairs)
    audit = {
        "schema_version": 1,
        "decision": decision,
        "inputs": [os.path.abspath(path) for path in args.pcd],
        "parameters": {
            "max_sample_per_chunk": args.max_sample_per_chunk,
            "overlap_voxel_m": args.overlap_voxel,
            "overlap_radius_m": args.overlap_radius,
            "plane_voxel_m": args.plane_voxel,
            "plane_cell_m": args.plane_cell,
            "plane_min_points": args.plane_min_points,
        },
        "totals": {
            "chunks": len(chunks),
            "points": sum(row["points"] for row in chunks),
            "file_bytes": sum(row["file_bytes"] for row in chunks),
            "sample_points": sum(row["sample_points"] for row in chunks),
            "planar_voxels": int(len(planar_xyz)),
            "planar_cells": plane_cells,
        },
        "planar_residual_m": plane,
        "integrity_checks": integrity_checks,
        "geometry_checks": geometry_checks,
        "screening_thresholds_are_project_targets_not_field_standards": {
            "plane_residual_median_m": args.plane_median_screen,
            "adjacent_mutual_distance_median_m": args.overlap_median_screen,
        },
        "chunks": chunks,
        "adjacent_pairs": pairs,
        "limitations": [
            "Dynamic objects and vegetation can inflate residuals",
            "Mutual nearest-neighbour residuals measure repeatability, not surface truth",
            "Deterministic point sampling is used to bound memory and runtime",
            "Range-banded error needs original scan poses and is not inferred from aggregate PCD",
            "This short segment cannot establish full-sequence loop-closure quality",
        ],
        "outputs": {
            "chunks_csv": os.path.abspath(args.out_chunks),
            "pairs_csv": os.path.abspath(args.out_pairs),
            "audit_json": os.path.abspath(args.audit),
        },
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.audit)), exist_ok=True)
    with open(args.audit, "w", encoding="utf-8") as handle:
        json.dump(audit, handle, indent=2, sort_keys=True)
        handle.write("\n")

    print("\n=== FAST-LIO chunked-map audit ===")
    print("chunks/points/sample:       %d / %d / %d" % (
        len(chunks), audit["totals"]["points"], audit["totals"]["sample_points"]
    ))
    print("payload integrity:          %s" % (
        "PASS" if integrity_checks["all_payloads_exact"] else "FAIL"
    ))
    print("planar cells:               %d" % plane_cells)
    print("planar residual med/p95:    %s / %s m" % (
        fmt(plane["median"]), fmt(plane["p95"])
    ))
    print("adjacent pair summary:")
    for row in pairs:
        print("  %s -> %s: mutual=%d ratio=%s median=%s p95=%s m" % (
            row["left"], row["right"], row["mutual_matches"],
            fmt(row["mutual_over_min_voxels_ratio"], 3),
            fmt(row["mutual_distance_median_m"]),
            fmt(row["mutual_distance_p95_m"]),
        ))
    print("decision:                   %s" % decision)
    for group, checks in (("integrity", integrity_checks), ("geometry", geometry_checks)):
        for name, passed in checks.items():
            print("  %s [%s]: %s" % ("PASS" if passed else "FAIL", group, name))
    print("Chunks CSV: %s" % args.out_chunks)
    print("Pairs CSV:  %s" % args.out_pairs)
    print("Audit JSON: %s" % args.audit)
    print("Caution: thresholds are project screens, not field standards.")


if __name__ == "__main__":
    main()
