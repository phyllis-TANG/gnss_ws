#!/usr/bin/env python3
"""Audit whether local LiDAR points can directly support satellite rays.

For every audited epoch--satellite ray, this script compares satellite
elevation with the empirical elevation envelope of GT-anchored local LiDAR
surface voxels in the same azimuth sector.  It does not use C/N0 and it does
not infer LOS/NLOS.  The purpose is to decide whether direct point-tube
raycasting is structurally observable or whether locally fitted facade planes
must be used instead.
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
    load_anchored_tile,
    read_anchor_windows,
)


ELEVATION_BINS = [(-90.0, 15.0), (15.0, 30.0), (30.0, 45.0),
                  (45.0, 60.0), (60.0, 90.000001)]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Audit UrbanV2X vertical LiDAR observability by azimuth sector"
    )
    parser.add_argument("--rays", required=True)
    parser.add_argument("--anchor-audit", required=True)
    parser.add_argument("--pcd-dir", required=True)
    parser.add_argument("--out-rays", required=True)
    parser.add_argument("--out-tiles", required=True)
    parser.add_argument("--out-elevation-bins", required=True)
    parser.add_argument("--audit", required=True)
    parser.add_argument("--out-plot")
    parser.add_argument("--surface-voxel", type=float, default=0.20)
    parser.add_argument("--min-range", type=float, default=3.0)
    parser.add_argument("--max-range", type=float, default=60.0)
    parser.add_argument("--sector-half-width", type=float, default=5.0)
    parser.add_argument("--sector-min-points", type=int, default=20)
    return parser.parse_args()


def number(row, field):
    value = row.get(field, "")
    if value in ("", "None", None):
        return None
    output = float(value)
    return output if math.isfinite(output) else None


def is_true(row, field):
    return row.get(field) in ("1", "1.0", "true", "True")


def circular_difference_degrees(left, right):
    return (np.asarray(left, dtype=np.float64) - float(right) + 180.0) % 360.0 - 180.0


def point_angles(points, origin, min_range, max_range):
    offsets = np.asarray(points, dtype=np.float64) - np.asarray(origin, dtype=np.float64)
    horizontal = np.hypot(offsets[:, 0], offsets[:, 1])
    ranges = np.linalg.norm(offsets, axis=1)
    valid = (
        np.isfinite(ranges) & (ranges >= min_range) & (ranges <= max_range)
        & (horizontal > 1e-6)
    )
    offsets = offsets[valid]
    horizontal = horizontal[valid]
    azimuth = np.degrees(np.arctan2(offsets[:, 0], offsets[:, 1])) % 360.0
    elevation = np.degrees(np.arctan2(offsets[:, 2], horizontal))
    return azimuth, elevation


def sector_statistics(point_azimuth, point_elevation, satellite_azimuth,
                      half_width, minimum_points):
    difference = np.abs(circular_difference_degrees(
        point_azimuth, satellite_azimuth
    ))
    values = np.asarray(point_elevation)[difference <= half_width]
    values = values[np.isfinite(values)]
    output = {
        "sector_points": int(len(values)),
        "sector_sufficient": int(len(values) >= minimum_points),
        "sector_elevation_p50_deg": None,
        "sector_elevation_p95_deg": None,
        "sector_elevation_p99_deg": None,
        "sector_elevation_max_deg": None,
    }
    if len(values):
        output.update({
            "sector_elevation_p50_deg": float(np.percentile(values, 50)),
            "sector_elevation_p95_deg": float(np.percentile(values, 95)),
            "sector_elevation_p99_deg": float(np.percentile(values, 99)),
            "sector_elevation_max_deg": float(np.max(values)),
        })
    return output


def elevation_bin(value):
    value = float(value)
    for lower, upper in ELEVATION_BINS:
        if lower <= value < upper:
            return "%g-%g" % (max(0.0, lower), upper if upper <= 90.0 else 90.0)
    return "outside"


def read_rays(path):
    with open(path, "r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    required = {
        "epoch_utc", "sat_id", "sys", "tile_window_index",
        "azimuth_deg", "elevation_deg", "receiver_e_m", "receiver_n_m",
        "receiver_u_m", "surface_hit", "patch_valid",
    }
    fields = set(rows[0]) if rows else set()
    missing = sorted(required - fields)
    if missing:
        raise ValueError("Ray CSV is missing fields: %s" % missing)
    return rows


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


def summarise_elevation_bins(rows):
    grouped = defaultdict(list)
    for row in rows:
        grouped[elevation_bin(row["elevation_deg"])].append(row)
    output = []
    ordered_labels = ["%g-%g" % (max(0.0, lower), upper if upper <= 90 else 90)
                      for lower, upper in ELEVATION_BINS]
    for label in ordered_labels:
        subset = grouped.get(label, [])
        if not subset:
            continue
        sufficient = [row for row in subset if row["sector_sufficient"] == 1]
        output.append({
            "elevation_bin_deg": label,
            "rays": len(subset),
            "sector_sufficient": len(sufficient),
            "surface_hits": sum(int(row["surface_hit"]) for row in subset),
            "hit_ratio": sum(int(row["surface_hit"]) for row in subset) / len(subset),
            "valid_patches": sum(int(row["patch_valid"]) for row in subset),
            "above_sector_p99": sum(
                int(row["sat_above_sector_p99"]) for row in sufficient
            ),
            "above_sector_p99_ratio": (
                sum(int(row["sat_above_sector_p99"]) for row in sufficient)
                / len(sufficient) if sufficient else None
            ),
            "satellite_elevation_median_deg": float(np.median([
                float(row["elevation_deg"]) for row in subset
            ])),
            "sector_p99_median_deg": (
                float(np.median([
                    row["sector_elevation_p99_deg"] for row in sufficient
                ])) if sufficient else None
            ),
            "elevation_gap_p99_median_deg": (
                float(np.median([
                    row["sat_minus_sector_p99_deg"] for row in sufficient
                ])) if sufficient else None
            ),
        })
    return output


def write_plot(path, ray_rows, elevation_rows):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False, "matplotlib_not_installed"
    sufficient = [row for row in ray_rows if row["sector_sufficient"] == 1]
    figure, axes = plt.subplots(1, 2, figsize=(12, 4.8))
    axes[0].bar(
        [row["elevation_bin_deg"] for row in elevation_rows],
        [row["hit_ratio"] for row in elevation_rows],
    )
    axes[0].set_xlabel("Satellite elevation bin [deg]")
    axes[0].set_ylabel("Direct point-tube hit ratio")
    axes[0].set_ylim(0.0, 1.0)
    colours = ["C1" if int(row["surface_hit"]) else "C0" for row in sufficient]
    axes[1].scatter(
        [float(row["elevation_deg"]) for row in sufficient],
        [row["sector_elevation_p99_deg"] for row in sufficient],
        c=colours, s=10, alpha=0.55,
    )
    axes[1].plot([0, 90], [0, 90], "k--", linewidth=1)
    axes[1].set_xlim(0, 90)
    axes[1].set_ylim(-30, 90)
    axes[1].set_xlabel("Satellite elevation [deg]")
    axes[1].set_ylabel("Local-sector LiDAR elevation p99 [deg]")
    for axis in axes:
        axis.grid(True, alpha=0.25)
    figure.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    figure.savefig(path, dpi=160)
    plt.close(figure)
    return True, None


def main():
    args = parse_args()
    if not (0.0 < args.sector_half_width <= 45.0):
        raise ValueError("Sector half width must be in (0,45] degrees")
    if args.min_range < 0.0 or args.max_range <= args.min_range:
        raise ValueError("Invalid point range limits")
    if args.surface_voxel <= 0.0 or args.sector_min_points < 1:
        raise ValueError("Invalid voxel/support parameters")

    print("Reading direct-ray audit and anchor metadata...")
    rays = read_rays(args.rays)
    windows, anchor_audit = read_anchor_windows(args.anchor_audit)
    rays_by_tile = defaultdict(list)
    for row in rays:
        rays_by_tile[int(row["tile_window_index"])].append(row)
    print("  rays=%d tiles=%d" % (len(rays), len(rays_by_tile)))

    output_rows = []
    tile_rows = []
    for sequence, tile_index in enumerate(sorted(rays_by_tile), 1):
        window = windows.get(tile_index)
        if window is None:
            raise ValueError("Tile %d missing from anchor audit" % tile_index)
        print("Processing tile %d (%d/%d)..." % (
            tile_index, sequence, len(rays_by_tile)
        ))
        xyz, intensity, input_points, pcd_files = load_anchored_tile(
            window, args.pcd_dir, args.surface_voxel
        )
        tile_rays = rays_by_tile[tile_index]
        angle_cache = {}
        for row in tile_rays:
            origin = tuple(round(float(row[field]), 4) for field in (
                "receiver_e_m", "receiver_n_m", "receiver_u_m"
            ))
            if origin not in angle_cache:
                angle_cache[origin] = point_angles(
                    xyz, origin, args.min_range, args.max_range
                )
            point_azimuth, point_elevation = angle_cache[origin]
            sector = sector_statistics(
                point_azimuth, point_elevation, float(row["azimuth_deg"]),
                args.sector_half_width, args.sector_min_points,
            )
            output = dict(row)
            output.update(sector)
            satellite_elevation = float(row["elevation_deg"])
            p99 = sector["sector_elevation_p99_deg"]
            maximum = sector["sector_elevation_max_deg"]
            output.update({
                "elevation_bin_deg": elevation_bin(satellite_elevation),
                "sat_minus_sector_p99_deg": (
                    satellite_elevation - p99 if p99 is not None else None
                ),
                "sat_minus_sector_max_deg": (
                    satellite_elevation - maximum if maximum is not None else None
                ),
                "sat_above_sector_p99": int(
                    p99 is not None and satellite_elevation > p99
                ),
                "sat_above_sector_max": int(
                    maximum is not None and satellite_elevation > maximum
                ),
            })
            output_rows.append(output)

        origins = np.asarray(list(angle_cache), dtype=np.float64)
        representative_origin = np.median(origins, axis=0)
        _, representative_elevation = point_angles(
            xyz, representative_origin, args.min_range, args.max_range
        )
        tile_outputs = output_rows[-len(tile_rays):]
        tile_rows.append({
            "tile_window_index": tile_index,
            "rays": len(tile_rays),
            "unique_receiver_epochs": len(angle_cache),
            "pcd_files": pcd_files,
            "input_points": input_points,
            "surface_voxels": len(xyz),
            "range_filtered_surface_voxels": len(representative_elevation),
            "point_elevation_p50_deg": percentile(representative_elevation, 0.50),
            "point_elevation_p95_deg": percentile(representative_elevation, 0.95),
            "point_elevation_p99_deg": percentile(representative_elevation, 0.99),
            "point_elevation_max_deg": (
                float(np.max(representative_elevation))
                if len(representative_elevation) else None
            ),
            "surface_hits": sum(is_true(row, "surface_hit") for row in tile_rays),
            "sector_sufficient": sum(
                row["sector_sufficient"] == 1 for row in tile_outputs
            ),
            "sat_above_sector_p99": sum(
                row["sat_above_sector_p99"] == 1 for row in tile_outputs
            ),
        })
        print("  point elev p95/p99=%.1f/%.1f deg; hits=%d/%d" % (
            tile_rows[-1]["point_elevation_p95_deg"],
            tile_rows[-1]["point_elevation_p99_deg"],
            tile_rows[-1]["surface_hits"], len(tile_rays),
        ))
        del xyz, intensity, angle_cache

    elevation_rows = summarise_elevation_bins(output_rows)
    write_csv(args.out_rays, output_rows)
    write_csv(args.out_tiles, tile_rows)
    write_csv(args.out_elevation_bins, elevation_rows)

    sufficient = [row for row in output_rows if row["sector_sufficient"] == 1]
    hits = [row for row in output_rows if is_true(row, "surface_hit")]
    above_p99 = [row for row in sufficient if row["sat_above_sector_p99"] == 1]
    above_max = [row for row in sufficient if row["sat_above_sector_max"] == 1]
    hit_ratio = len(hits) / len(output_rows)
    above_ratio = len(above_p99) / len(sufficient) if sufficient else None
    limited = (
        len(sufficient) >= 100 and hit_ratio < 0.15
        and above_ratio is not None and above_ratio > 0.50
    )
    decision = (
        "DIRECT_POINT_RAYCAST_OBSERVABILITY_LIMITED"
        if limited else "REVIEW_DIRECT_POINT_OBSERVABILITY"
    )
    plot_written = False
    plot_error = None
    if args.out_plot:
        plot_written, plot_error = write_plot(
            args.out_plot, output_rows, elevation_rows
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
            "min_range_m": args.min_range,
            "max_range_m": args.max_range,
            "sector_half_width_deg": args.sector_half_width,
            "sector_min_points": args.sector_min_points,
        },
        "results": {
            "rays": len(output_rows),
            "tiles": len(tile_rows),
            "surface_hits": len(hits),
            "surface_hit_ratio": hit_ratio,
            "sector_sufficient": len(sufficient),
            "sector_sufficient_ratio": len(sufficient) / len(output_rows),
            "sat_above_sector_p99": len(above_p99),
            "sat_above_sector_p99_ratio": above_ratio,
            "sat_above_sector_max": len(above_max),
            "sat_above_sector_max_ratio": (
                len(above_max) / len(sufficient) if sufficient else None
            ),
            "satellite_elevation_deg": describe([
                number(row, "elevation_deg") for row in output_rows
            ]),
            "sector_elevation_p99_deg": describe([
                row["sector_elevation_p99_deg"] for row in sufficient
            ]),
            "sat_minus_sector_p99_deg": describe([
                row["sat_minus_sector_p99_deg"] for row in sufficient
            ]),
            "tile_point_elevation_p99_deg": describe([
                row["point_elevation_p99_deg"] for row in tile_rows
            ]),
        },
        "by_elevation_bin": elevation_rows,
        "by_constellation": [{
            "sys": system,
            "rays": sum(row["sys"] == system for row in output_rows),
            "hits": sum(
                row["sys"] == system and is_true(row, "surface_hit")
                for row in output_rows
            ),
            "above_sector_p99": sum(
                row["sys"] == system and row["sat_above_sector_p99"] == 1
                for row in sufficient
            ),
        } for system in sorted(set(row["sys"] for row in output_rows))],
        "upstream_anchor_decision": anchor_audit.get("decision"),
        "interpretation_guards": [
            "No C/N0 field is read or used",
            "Sector elevation envelopes measure mapped LiDAR support, not manufacturer field of view",
            "A satellite above the local sector envelope is not automatically NLOS or LOS",
            "Local maps combine several viewpoints within each five-second tile",
            "Dynamic objects and vegetation remain in the empirical surface envelope",
            "The audit tests direct point-tube observability, not facade-plane extrapolation",
        ],
        "recommended_next_representation": (
            "local_facade_planes_with_continuous_spatial_confidence"
            if limited else "retain_direct_raycast_for_sensitivity_only"
        ),
        "outputs": {
            "rays_csv": os.path.abspath(args.out_rays),
            "tiles_csv": os.path.abspath(args.out_tiles),
            "elevation_bins_csv": os.path.abspath(args.out_elevation_bins),
            "plot": os.path.abspath(args.out_plot) if args.out_plot else None,
            "plot_written": plot_written,
            "plot_error": plot_error,
        },
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.audit)), exist_ok=True)
    with open(args.audit, "w", encoding="utf-8") as handle:
        json.dump(audit, handle, indent=2, sort_keys=True)
        handle.write("\n")

    print("\n=== UrbanV2X vertical surface observability ===")
    print("rays / tiles:                    %d / %d" % (
        len(output_rows), len(tile_rows)
    ))
    print("sector sufficient:               %d (%.1f%%)" % (
        len(sufficient), 100.0 * len(sufficient) / len(output_rows)
    ))
    print("direct surface hits:             %d (%.1f%%)" % (
        len(hits), 100.0 * hit_ratio
    ))
    print("satellite elevation med/p95:     %.2f / %.2f deg" % (
        audit["results"]["satellite_elevation_deg"]["median"],
        audit["results"]["satellite_elevation_deg"]["p95"],
    ))
    print("sector LiDAR elevation p99 med:  %.2f deg" % audit[
        "results"
    ]["sector_elevation_p99_deg"]["median"])
    print("sat minus sector p99 med/p95:    %.2f / %.2f deg" % (
        audit["results"]["sat_minus_sector_p99_deg"]["median"],
        audit["results"]["sat_minus_sector_p99_deg"]["p95"],
    ))
    print("sat above sector p99:            %d (%.1f%%)" % (
        len(above_p99), 100.0 * above_ratio
    ))
    print("sat above sector maximum:        %d (%.1f%%)" % (
        len(above_max), 100.0 * len(above_max) / len(sufficient)
    ))
    print("by satellite elevation:")
    print("bin,rays,hits,hit_ratio,above_p99_ratio,gap_median")
    for row in elevation_rows:
        print("%s,%d,%d,%.3f,%s,%s" % (
            row["elevation_bin_deg"], row["rays"], row["surface_hits"],
            row["hit_ratio"], row["above_sector_p99_ratio"],
            row["elevation_gap_p99_median_deg"],
        ))
    print("decision:                         %s" % decision)
    print("recommended representation:      %s" % audit[
        "recommended_next_representation"
    ])
    print("Rays CSV: %s" % args.out_rays)
    print("Tiles CSV: %s" % args.out_tiles)
    print("Bins CSV: %s" % args.out_elevation_bins)
    print("Audit:    %s" % args.audit)
    if args.out_plot:
        print("Plot:     %s (%s)" % (
            args.out_plot, "written" if plot_written else plot_error
        ))
    print("Caution: empirical observability is not a LOS/NLOS label.")


if __name__ == "__main__":
    main()
