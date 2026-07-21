#!/usr/bin/env python3
"""Cluster stable facade patches and group repeated physical facades.

Stage 1 connects spatially adjacent, similarly oriented, mutually coplanar
stable patch cells within each GT-anchored tile.  Stage 2 links compatible
plane observations from different tiles into physical-facade groups.  Every
observation and patch membership is retained; cross-tile grouping does not
delete or prematurely average repeated measurements.  Neither patch intensity
nor C/N0 is used to decide geometric membership.
"""

import argparse
import csv
import json
import math
import os
from collections import Counter, defaultdict

import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(
        description="Cluster UrbanV2X stable facade patches and cross-tile duplicates"
    )
    parser.add_argument("--patches", required=True)
    parser.add_argument("--out-planes", required=True)
    parser.add_argument("--out-membership", required=True)
    parser.add_argument("--out-physical", required=True)
    parser.add_argument("--audit", required=True)
    parser.add_argument("--out-plot")
    parser.add_argument("--cell-size", type=float, default=2.0)
    parser.add_argument("--within-normal-angle", type=float, default=10.0)
    parser.add_argument("--within-plane-offset", type=float, default=0.20)
    parser.add_argument("--within-cell-gap", type=int, default=1)
    parser.add_argument("--cluster-normal-review", type=float, default=15.0)
    parser.add_argument("--cluster-plane-review", type=float, default=0.30)
    parser.add_argument("--cross-centroid-max", type=float, default=30.0)
    parser.add_argument("--cross-normal-angle", type=float, default=10.0)
    parser.add_argument("--cross-plane-offset", type=float, default=0.30)
    parser.add_argument("--cross-horizontal-gap", type=float, default=2.5)
    parser.add_argument("--cross-vertical-gap", type=float, default=2.5)
    parser.add_argument("--physical-normal-review", type=float, default=15.0)
    parser.add_argument("--physical-plane-review", type=float, default=0.50)
    parser.add_argument("--intensity-repeat-absolute", type=float, default=5.0)
    parser.add_argument("--intensity-repeat-relative", type=float, default=0.30)
    return parser.parse_args()


class UnionFind:
    def __init__(self, size):
        self.parent = list(range(size))
        self.rank = [0] * size

    def find(self, value):
        while self.parent[value] != value:
            self.parent[value] = self.parent[self.parent[value]]
            value = self.parent[value]
        return value

    def union(self, left, right):
        left = self.find(left)
        right = self.find(right)
        if left == right:
            return
        if self.rank[left] < self.rank[right]:
            left, right = right, left
        self.parent[right] = left
        if self.rank[left] == self.rank[right]:
            self.rank[left] += 1

    def components(self):
        output = defaultdict(list)
        for index in range(len(self.parent)):
            output[self.find(index)].append(index)
        return list(output.values())


def number(row, field):
    value = row.get(field, "")
    if value in ("", "None", None):
        return None
    output = float(value)
    return output if math.isfinite(output) else None


def integer(row, field):
    return int(float(row[field]))


def is_true(row, field):
    value = row.get(field)
    return value is True or value == 1 or value in ("1", "1.0", "true", "True")


def vector(row, fields):
    values = [number(row, field) for field in fields]
    if any(value is None for value in values):
        raise ValueError("Missing vector field among %s" % (fields,))
    return np.asarray(values, dtype=np.float64)


def centroid(row):
    return vector(row, ("centroid_e_m", "centroid_n_m", "centroid_u_m"))


def normal(row):
    output = vector(row, ("normal_e", "normal_n", "normal_u"))
    norm = np.linalg.norm(output)
    if norm <= 0.0:
        raise ValueError("Zero normal in patch/plane row")
    return output / norm


def unsigned_normal_angle(left, right):
    cosine = float(np.clip(abs(np.dot(left, right)), 0.0, 1.0))
    return math.degrees(math.acos(cosine))


def mutual_plane_offset(left_centroid, left_normal, right_centroid, right_normal):
    delta = right_centroid - left_centroid
    return float(max(abs(np.dot(left_normal, delta)),
                     abs(np.dot(right_normal, delta))))


def interval_gap(left_min, left_max, right_min, right_max):
    if left_max < right_min:
        return float(right_min - left_max)
    if right_max < left_min:
        return float(left_min - right_max)
    return 0.0


def orient_normals(normals, weights):
    reference = normals[0]
    aligned = np.asarray([
        value if np.dot(value, reference) >= 0.0 else -value
        for value in normals
    ])
    mean = np.average(aligned, axis=0, weights=weights)
    norm = np.linalg.norm(mean)
    return mean / norm if norm > 0.0 else reference


def weighted_median(values, weights):
    values = np.asarray(values, dtype=np.float64)
    weights = np.asarray(weights, dtype=np.float64)
    valid = np.isfinite(values) & np.isfinite(weights) & (weights > 0.0)
    values = values[valid]
    weights = weights[valid]
    if not len(values):
        return None
    order = np.argsort(values, kind="mergesort")
    values = values[order]
    weights = weights[order]
    threshold = 0.5 * np.sum(weights)
    return float(values[np.searchsorted(np.cumsum(weights), threshold, side="left")])


def weighted_quantile(values, weights, probability):
    values = np.asarray(values, dtype=np.float64)
    weights = np.asarray(weights, dtype=np.float64)
    valid = np.isfinite(values) & np.isfinite(weights) & (weights > 0.0)
    values = values[valid]
    weights = weights[valid]
    if not len(values):
        return None
    order = np.argsort(values, kind="mergesort")
    values = values[order]
    weights = weights[order]
    threshold = probability * np.sum(weights)
    index = np.searchsorted(np.cumsum(weights), threshold, side="left")
    return float(values[min(index, len(values) - 1)])


def weighted_intensity_summary(rows, field="intensity_median"):
    values = []
    weights = []
    for row in rows:
        value = number(row, field)
        if value is None:
            continue
        values.append(value)
        weight = number(row, "diffuse_points")
        if weight is None:
            weight = number(row, "total_diffuse_points")
        weights.append(weight if weight is not None and weight > 0.0 else 1.0)
    if not values:
        return {"median": None, "p25": None, "p75": None, "iqr": None,
                "mad": None, "min": None, "max": None}
    median = weighted_median(values, weights)
    p25 = weighted_quantile(values, weights, 0.25)
    p75 = weighted_quantile(values, weights, 0.75)
    mad = weighted_median(np.abs(np.asarray(values) - median), weights)
    return {
        "median": median, "p25": p25, "p75": p75,
        "iqr": p75 - p25, "mad": mad,
        "min": float(np.min(values)), "max": float(np.max(values)),
    }


def read_stable_patches(path):
    with open(path, "r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError("Empty patch inventory")
    required = {
        "patch_id", "tile_window_index", "cell_x", "cell_y", "cell_z",
        "centroid_e_m", "centroid_n_m", "centroid_u_m",
        "normal_e", "normal_n", "normal_u", "points", "diffuse_points",
        "horizontal_span_m", "vertical_span_m", "intensity_median",
        "stable_facade_patch",
    }
    missing = sorted(required - set(rows[0]))
    if missing:
        raise ValueError("Patch inventory is missing fields: %s" % missing)
    stable = [row for row in rows if is_true(row, "stable_facade_patch")]
    if not stable:
        raise ValueError("Patch inventory contains no stable facade patches")
    stable.sort(key=lambda row: (
        integer(row, "tile_window_index"), integer(row, "cell_x"),
        integer(row, "cell_y"), integer(row, "cell_z"), row["patch_id"],
    ))
    return stable, len(rows)


def patch_cells_adjacent(left, right, maximum_gap):
    return max(abs(integer(left, field) - integer(right, field))
               for field in ("cell_x", "cell_y", "cell_z")) <= maximum_gap


def can_join_patches(left, right, args):
    if not patch_cells_adjacent(left, right, args.within_cell_gap):
        return False
    left_normal = normal(left)
    right_normal = normal(right)
    if unsigned_normal_angle(left_normal, right_normal) > args.within_normal_angle:
        return False
    return mutual_plane_offset(
        centroid(left), left_normal, centroid(right), right_normal
    ) <= args.within_plane_offset


def component_geometry(rows):
    centres = np.asarray([centroid(row) for row in rows])
    normals = np.asarray([normal(row) for row in rows])
    weights = np.asarray([
        max(number(row, "points") or number(row, "total_points") or 1.0, 1.0)
        for row in rows
    ])
    merged_centroid = np.average(centres, axis=0, weights=weights)
    merged_normal = orient_normals(normals, weights)
    normal_angles = [unsigned_normal_angle(value, merged_normal) for value in normals]
    plane_offsets = np.abs((centres - merged_centroid) @ merged_normal)
    tangent = np.array([-merged_normal[1], merged_normal[0], 0.0])
    tangent_norm = np.linalg.norm(tangent)
    tangent = tangent / tangent_norm if tangent_norm > 0.0 else np.array([1.0, 0.0, 0.0])
    tangent_centres = centres @ tangent
    half_width = np.asarray([
        0.5 * (number(row, "horizontal_span_m") or 0.0) for row in rows
    ])
    horizontal_min = float(np.min(tangent_centres - half_width))
    horizontal_max = float(np.max(tangent_centres + half_width))
    vertical_centres = centres[:, 2]
    half_height = np.asarray([
        0.5 * (number(row, "vertical_span_m") or 0.0) for row in rows
    ])
    vertical_min = float(np.min(vertical_centres - half_height))
    vertical_max = float(np.max(vertical_centres + half_height))
    return {
        "centroid": merged_centroid,
        "normal": merged_normal,
        "normal_angle_median_deg": float(np.median(normal_angles)),
        "normal_angle_max_deg": float(np.max(normal_angles)),
        "centroid_plane_offset_median_m": float(np.median(plane_offsets)),
        "centroid_plane_offset_max_m": float(np.max(plane_offsets)),
        "horizontal_min_m": horizontal_min,
        "horizontal_max_m": horizontal_max,
        "horizontal_span_m": horizontal_max - horizontal_min,
        "vertical_min_m": vertical_min,
        "vertical_max_m": vertical_max,
        "vertical_span_m": vertical_max - vertical_min,
        "tangent": tangent,
        "weights": weights,
    }


def serialise_plane(tile_index, plane_index, rows, args):
    geometry = component_geometry(rows)
    intensity = weighted_intensity_summary(rows)
    subset_a = weighted_intensity_summary(rows, "subset_a_intensity_median")
    subset_b = weighted_intensity_summary(rows, "subset_b_intensity_median")
    repeat_abs = None
    repeat_relative = None
    if subset_a["median"] is not None and subset_b["median"] is not None:
        repeat_abs = abs(subset_a["median"] - subset_b["median"])
        repeat_relative = repeat_abs / max(
            0.5 * (subset_a["median"] + subset_b["median"]), 1e-6
        )
    consistent = (
        geometry["normal_angle_max_deg"] <= args.cluster_normal_review
        and geometry["centroid_plane_offset_max_m"] <= args.cluster_plane_review
    )
    plane_id = "T%02d_P%04d" % (tile_index, plane_index)
    centre = geometry["centroid"]
    plane_normal = geometry["normal"]
    return {
        "plane_observation_id": plane_id,
        "tile_window_index": tile_index,
        "member_patches": len(rows),
        "total_points": int(sum(integer(row, "points") for row in rows)),
        "total_diffuse_points": int(sum(integer(row, "diffuse_points") for row in rows)),
        "centroid_e_m": float(centre[0]),
        "centroid_n_m": float(centre[1]),
        "centroid_u_m": float(centre[2]),
        "normal_e": float(plane_normal[0]),
        "normal_n": float(plane_normal[1]),
        "normal_u": float(plane_normal[2]),
        "normal_angle_median_deg": geometry["normal_angle_median_deg"],
        "normal_angle_max_deg": geometry["normal_angle_max_deg"],
        "centroid_plane_offset_median_m": geometry["centroid_plane_offset_median_m"],
        "centroid_plane_offset_max_m": geometry["centroid_plane_offset_max_m"],
        "horizontal_min_m": geometry["horizontal_min_m"],
        "horizontal_max_m": geometry["horizontal_max_m"],
        "horizontal_span_m": geometry["horizontal_span_m"],
        "vertical_min_m": geometry["vertical_min_m"],
        "vertical_max_m": geometry["vertical_max_m"],
        "vertical_span_m": geometry["vertical_span_m"],
        "intensity_median": intensity["median"],
        "intensity_iqr_between_patches": intensity["iqr"],
        "intensity_mad_between_patches": intensity["mad"],
        "intensity_min": intensity["min"],
        "intensity_max": intensity["max"],
        "subset_a_intensity_median": subset_a["median"],
        "subset_b_intensity_median": subset_b["median"],
        "intensity_repeat_abs": repeat_abs,
        "intensity_repeat_relative": repeat_relative,
        "cluster_geometry_consistent": int(consistent),
        "cluster_geometry_status": "PASS" if consistent else "CHAIN_REVIEW",
    }


def cluster_within_tiles(patches, args):
    by_tile = defaultdict(list)
    for row in patches:
        by_tile[integer(row, "tile_window_index")].append(row)
    planes = []
    membership = []
    edge_counts = {}
    for tile_index in sorted(by_tile):
        rows = by_tile[tile_index]
        union = UnionFind(len(rows))
        edges = 0
        for left in range(len(rows)):
            for right in range(left + 1, len(rows)):
                if can_join_patches(rows[left], rows[right], args):
                    union.union(left, right)
                    edges += 1
        components = union.components()
        components.sort(key=lambda values: min(
            (integer(rows[index], "cell_x"), integer(rows[index], "cell_y"),
             integer(rows[index], "cell_z"), rows[index]["patch_id"])
            for index in values
        ))
        edge_counts[tile_index] = edges
        for plane_index, indices in enumerate(components):
            members = [rows[index] for index in indices]
            plane = serialise_plane(tile_index, plane_index, members, args)
            planes.append(plane)
            for row in members:
                membership.append({
                    "patch_id": row["patch_id"],
                    "tile_window_index": tile_index,
                    "plane_observation_id": plane["plane_observation_id"],
                    "physical_facade_id": None,
                })
    return planes, membership, edge_counts


def plane_intervals_in_common_tangent(left, right, common_normal):
    tangent = np.array([-common_normal[1], common_normal[0], 0.0])
    norm = np.linalg.norm(tangent)
    tangent = tangent / norm if norm > 0.0 else np.array([1.0, 0.0, 0.0])
    left_centre = float(np.dot(centroid(left), tangent))
    right_centre = float(np.dot(centroid(right), tangent))
    left_half = 0.5 * (number(left, "horizontal_span_m") or 0.0)
    right_half = 0.5 * (number(right, "horizontal_span_m") or 0.0)
    return (left_centre - left_half, left_centre + left_half,
            right_centre - right_half, right_centre + right_half)


def can_match_planes(left, right, args):
    if integer(left, "tile_window_index") == integer(right, "tile_window_index"):
        return False
    left_centre = centroid(left)
    right_centre = centroid(right)
    if np.linalg.norm(left_centre - right_centre) > args.cross_centroid_max:
        return False
    left_normal = normal(left)
    right_normal = normal(right)
    if unsigned_normal_angle(left_normal, right_normal) > args.cross_normal_angle:
        return False
    if mutual_plane_offset(
        left_centre, left_normal, right_centre, right_normal
    ) > args.cross_plane_offset:
        return False
    common_normal = orient_normals(
        np.asarray([left_normal, right_normal]), np.ones(2)
    )
    intervals = plane_intervals_in_common_tangent(left, right, common_normal)
    horizontal_gap = interval_gap(*intervals)
    if horizontal_gap > args.cross_horizontal_gap:
        return False
    vertical_gap = interval_gap(
        number(left, "vertical_min_m"), number(left, "vertical_max_m"),
        number(right, "vertical_min_m"), number(right, "vertical_max_m"),
    )
    return vertical_gap <= args.cross_vertical_gap


def serialise_physical_facade(facade_index, rows, args):
    geometry = component_geometry(rows)
    intensity = weighted_intensity_summary(rows)
    values = [number(row, "intensity_median") for row in rows]
    values = [value for value in values if value is not None]
    repeat_abs = max(values) - min(values) if len(values) >= 2 else None
    repeat_relative = (
        repeat_abs / max(intensity["median"], 1e-6)
        if repeat_abs is not None and intensity["median"] is not None else None
    )
    repeatable = (
        len(values) >= 2
        and (repeat_abs <= args.intensity_repeat_absolute
             or repeat_relative <= args.intensity_repeat_relative)
    )
    consistent = (
        all(is_true(row, "cluster_geometry_consistent") for row in rows)
        and
        geometry["normal_angle_max_deg"] <= args.physical_normal_review
        and geometry["centroid_plane_offset_max_m"] <= args.physical_plane_review
    )
    centre = geometry["centroid"]
    facade_normal = geometry["normal"]
    tile_ids = sorted(set(integer(row, "tile_window_index") for row in rows))
    return {
        "physical_facade_id": "F%05d" % facade_index,
        "plane_observations": len(rows),
        "unique_tiles": len(tile_ids),
        "tile_window_indices": ";".join(str(value) for value in tile_ids),
        "repeated_across_tiles": int(len(tile_ids) >= 2),
        "total_member_patches": int(sum(integer(row, "member_patches") for row in rows)),
        "total_points": int(sum(integer(row, "total_points") for row in rows)),
        "centroid_e_m": float(centre[0]),
        "centroid_n_m": float(centre[1]),
        "centroid_u_m": float(centre[2]),
        "normal_e": float(facade_normal[0]),
        "normal_n": float(facade_normal[1]),
        "normal_u": float(facade_normal[2]),
        "normal_angle_max_deg": geometry["normal_angle_max_deg"],
        "centroid_plane_offset_max_m": geometry["centroid_plane_offset_max_m"],
        "horizontal_span_m": geometry["horizontal_span_m"],
        "vertical_span_m": geometry["vertical_span_m"],
        "intensity_median": intensity["median"],
        "intensity_iqr_between_observations": intensity["iqr"],
        "intensity_mad_between_observations": intensity["mad"],
        "intensity_min": intensity["min"],
        "intensity_max": intensity["max"],
        "cross_tile_intensity_range": repeat_abs,
        "cross_tile_intensity_relative_range": repeat_relative,
        "cross_tile_reflectivity_repeatable": int(repeatable),
        "physical_geometry_consistent": int(consistent),
        "physical_geometry_status": "PASS" if consistent else "CHAIN_REVIEW",
    }


def group_cross_tile_facades(planes, membership, args):
    eligible_indices = [index for index, row in enumerate(planes)
                        if is_true(row, "cluster_geometry_consistent")]
    union = UnionFind(len(planes))
    edges = 0
    for position, left in enumerate(eligible_indices):
        for right in eligible_indices[position + 1:]:
            if can_match_planes(planes[left], planes[right], args):
                union.union(left, right)
                edges += 1
    components = union.components()
    components.sort(key=lambda indices: min(
        (integer(planes[index], "tile_window_index"),
         planes[index]["plane_observation_id"]) for index in indices
    ))
    physical = []
    plane_to_facade = {}
    for facade_index, indices in enumerate(components):
        rows = [planes[index] for index in indices]
        facade = serialise_physical_facade(facade_index, rows, args)
        physical.append(facade)
        for row in rows:
            plane_to_facade[row["plane_observation_id"]] = facade[
                "physical_facade_id"
            ]
    for row in planes:
        row["physical_facade_id"] = plane_to_facade[row["plane_observation_id"]]
    for row in membership:
        row["physical_facade_id"] = plane_to_facade[row["plane_observation_id"]]
    return physical, edges


def describe(values):
    values = np.asarray([value for value in values if value is not None], dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return {"n": 0, "median": None, "p05": None, "p95": None, "max": None}
    return {
        "n": int(len(values)), "median": float(np.median(values)),
        "p05": float(np.percentile(values, 5)),
        "p95": float(np.percentile(values, 95)), "max": float(np.max(values)),
    }


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


def write_plot(path, planes, physical):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False, "matplotlib_not_installed"
    tile_counts = Counter(integer(row, "tile_window_index") for row in planes)
    repeated = [row for row in physical if is_true(row, "repeated_across_tiles")]
    figure, axes = plt.subplots(1, 2, figsize=(12, 4.8))
    axes[0].bar([str(key) for key in sorted(tile_counts)],
                [tile_counts[key] for key in sorted(tile_counts)])
    axes[0].set_xlabel("5 s tile index")
    axes[0].set_ylabel("plane observations")
    if repeated:
        axes[1].scatter(
            [integer(row, "unique_tiles") for row in repeated],
            [number(row, "cross_tile_intensity_range") for row in repeated],
            s=14, alpha=0.55,
        )
    axes[1].set_xlabel("unique tile observations per physical facade")
    axes[1].set_ylabel("cross-tile intensity range")
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
        args.cell_size, args.within_normal_angle, args.within_plane_offset,
        args.cluster_normal_review, args.cluster_plane_review,
        args.cross_centroid_max, args.cross_normal_angle,
        args.cross_plane_offset, args.cross_horizontal_gap,
        args.cross_vertical_gap, args.physical_normal_review,
        args.physical_plane_review, args.intensity_repeat_absolute,
        args.intensity_repeat_relative,
    ]
    if any(value <= 0.0 for value in positive) or args.within_cell_gap < 1:
        raise ValueError("All clustering thresholds must be positive")

    patches, inventory_rows = read_stable_patches(args.patches)
    print("Reading stable facade patches...")
    print("  inventory rows=%d stable patches=%d tiles=%d" % (
        inventory_rows, len(patches), len(set(
            integer(row, "tile_window_index") for row in patches
        )),
    ))
    print("Clustering patches within tiles...")
    planes, membership, within_edges = cluster_within_tiles(patches, args)
    print("  plane observations=%d multi-patch=%d chain-review=%d" % (
        len(planes), sum(integer(row, "member_patches") >= 2 for row in planes),
        sum(not is_true(row, "cluster_geometry_consistent") for row in planes),
    ))
    print("Matching compatible planes across tiles...")
    physical, cross_edges = group_cross_tile_facades(
        planes, membership, args
    )
    repeated = [row for row in physical if is_true(row, "repeated_across_tiles")]
    print("  physical facades=%d repeated=%d cross-edges=%d" % (
        len(physical), len(repeated), cross_edges
    ))

    write_csv(args.out_planes, planes)
    write_csv(args.out_membership, membership)
    write_csv(args.out_physical, physical)
    plot_written = False
    plot_error = None
    if args.out_plot:
        plot_written, plot_error = write_plot(args.out_plot, planes, physical)

    consistent_planes = [row for row in planes
                         if is_true(row, "cluster_geometry_consistent")]
    consistent_physical = [row for row in physical
                           if is_true(row, "physical_geometry_consistent")]
    repeated_consistent = [row for row in consistent_physical
                           if is_true(row, "repeated_across_tiles")]
    repeated_reflectivity = [row for row in repeated_consistent
                             if is_true(row, "cross_tile_reflectivity_repeatable")]
    enough = len(consistent_physical) >= 50 and len(repeated_consistent) >= 10
    decision = (
        "PHYSICAL_FACADE_GROUPS_AVAILABLE"
        if enough else "REVIEW_PHYSICAL_FACADE_YIELD"
    )
    audit = {
        "schema_version": 1,
        "decision": decision,
        "inputs": {"patches": os.path.abspath(args.patches)},
        "parameters": {
            "cell_size_m": args.cell_size,
            "within_normal_angle_deg": args.within_normal_angle,
            "within_plane_offset_m": args.within_plane_offset,
            "within_cell_gap": args.within_cell_gap,
            "cluster_normal_review_deg": args.cluster_normal_review,
            "cluster_plane_review_m": args.cluster_plane_review,
            "cross_centroid_max_m": args.cross_centroid_max,
            "cross_normal_angle_deg": args.cross_normal_angle,
            "cross_plane_offset_m": args.cross_plane_offset,
            "cross_horizontal_gap_m": args.cross_horizontal_gap,
            "cross_vertical_gap_m": args.cross_vertical_gap,
            "physical_normal_review_deg": args.physical_normal_review,
            "physical_plane_review_m": args.physical_plane_review,
            "intensity_repeat_absolute": args.intensity_repeat_absolute,
            "intensity_repeat_relative": args.intensity_repeat_relative,
        },
        "thresholds_are_pre_cn0_project_screens_not_field_standards": {
            "consistent_physical_facades_min": 50,
            "repeated_consistent_physical_facades_min": 10,
        },
        "results": {
            "inventory_rows": inventory_rows,
            "stable_patches": len(patches),
            "tiles": len(set(integer(row, "tile_window_index") for row in patches)),
            "within_tile_edges": int(sum(within_edges.values())),
            "within_tile_edges_by_tile": {str(key): value
                                           for key, value in sorted(within_edges.items())},
            "plane_observations": len(planes),
            "multi_patch_plane_observations": sum(
                integer(row, "member_patches") >= 2 for row in planes
            ),
            "cluster_geometry_consistent": len(consistent_planes),
            "cluster_chain_review": len(planes) - len(consistent_planes),
            "cross_tile_edges": cross_edges,
            "physical_facades": len(physical),
            "physical_geometry_consistent": len(consistent_physical),
            "repeated_across_tiles": len(repeated),
            "repeated_consistent": len(repeated_consistent),
            "repeated_reflectivity_repeatable": len(repeated_reflectivity),
            "plane_member_patches": describe([
                integer(row, "member_patches") for row in planes
            ]),
            "plane_horizontal_span_m": describe([
                number(row, "horizontal_span_m") for row in consistent_planes
            ]),
            "plane_vertical_span_m": describe([
                number(row, "vertical_span_m") for row in consistent_planes
            ]),
            "physical_observations": describe([
                integer(row, "plane_observations") for row in physical
            ]),
            "repeated_intensity_range": describe([
                number(row, "cross_tile_intensity_range")
                for row in repeated_consistent
            ]),
            "repeated_intensity_relative_range": describe([
                number(row, "cross_tile_intensity_relative_range")
                for row in repeated_consistent
            ]),
        },
        "interpretation_guards": [
            "No C/N0 field is read or used",
            "Intensity is not used for geometric clustering or cross-tile membership",
            "Connected components can chain locally compatible patches; chain-review flags audit this drift",
            "Cross-tile grouping retains every plane observation and patch membership",
            "One long physical building face may remain split into multiple local facade groups",
            "Different coplanar materials on one wall remain one geometric facade but retain patch-level intensities",
            "Cross-tile seam uncertainty motivates a looser plane-offset screen than within-tile clustering",
            "Physical facade groups are candidates for continuous exposure modelling, not material ground truth",
        ],
        "recommended_next_step": (
            "build_satellite_facade_exposure_without_cn0_then_freeze_features"
            if enough else "review_cluster_threshold_sensitivity_before_exposure"
        ),
        "outputs": {
            "planes_csv": os.path.abspath(args.out_planes),
            "membership_csv": os.path.abspath(args.out_membership),
            "physical_csv": os.path.abspath(args.out_physical),
            "plot": os.path.abspath(args.out_plot) if args.out_plot else None,
            "plot_written": plot_written,
            "plot_error": plot_error,
        },
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.audit)), exist_ok=True)
    with open(args.audit, "w", encoding="utf-8") as handle:
        json.dump(audit, handle, indent=2, sort_keys=True)
        handle.write("\n")

    print("\n=== UrbanV2X facade clustering and deduplication ===")
    print("stable patches:                    %d" % len(patches))
    print("plane observations:                %d" % len(planes))
    print("multi-patch planes:                %d" % audit["results"][
        "multi_patch_plane_observations"
    ])
    print("cluster consistent / review:       %d / %d" % (
        len(consistent_planes), len(planes) - len(consistent_planes)
    ))
    print("physical facade groups:            %d" % len(physical))
    print("physical geometry consistent:      %d" % len(consistent_physical))
    print("repeated across tiles:             %d" % len(repeated))
    print("repeated and geometry-consistent:  %d" % len(repeated_consistent))
    print("repeated reflectivity-repeatable:  %d" % len(repeated_reflectivity))
    print("plane member patches med/p95:      %s / %s" % (
        audit["results"]["plane_member_patches"]["median"],
        audit["results"]["plane_member_patches"]["p95"],
    ))
    print("plane width med/p95:               %s / %s m" % (
        audit["results"]["plane_horizontal_span_m"]["median"],
        audit["results"]["plane_horizontal_span_m"]["p95"],
    ))
    print("plane height med/p95:              %s / %s m" % (
        audit["results"]["plane_vertical_span_m"]["median"],
        audit["results"]["plane_vertical_span_m"]["p95"],
    ))
    if repeated_consistent:
        print("cross-tile intensity range med/p95:%s / %s" % (
            audit["results"]["repeated_intensity_range"]["median"],
            audit["results"]["repeated_intensity_range"]["p95"],
        ))
    print("decision:                          %s" % decision)
    print("recommended next step:             %s" % audit["recommended_next_step"])
    print("Plane observations: %s" % args.out_planes)
    print("Patch membership:   %s" % args.out_membership)
    print("Physical facades:   %s" % args.out_physical)
    print("Audit:              %s" % args.audit)
    if args.out_plot:
        print("Plot:               %s (%s)" % (
            args.out_plot, "written" if plot_written else plot_error
        ))
    print("Caution: deduplicated facades remain geometric groups, not material labels.")


if __name__ == "__main__":
    main()
