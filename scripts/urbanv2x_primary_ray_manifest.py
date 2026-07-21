#!/usr/bin/env python3
"""Create an all-tile PRIMARY epoch--satellite ray manifest.

Frequency-channel signal rows are deduplicated to one geometric ray per
epoch--satellite while their signal labels and multiplicity are retained.  The
receiver ECEF coordinates are converted to the same GT-local ENU frame used by
the independently anchored FAST-LIO tiles.  No LiDAR surface matching and no
C/N0 outcome analysis is performed.
"""

import argparse
import csv
import json
import os
from collections import Counter

import numpy as np


from urbanv2x_lidar_gt_audit import read_ground_truth
from urbanv2x_local_surface_audit import read_primary_rays
from urbanv2x_map_consistency import ecef_to_local_enu


def parse_args():
    parser = argparse.ArgumentParser(
        description="Create all-tile UrbanV2X PRIMARY satellite ray manifest"
    )
    parser.add_argument("--eligibility", required=True)
    parser.add_argument("--gt", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--audit", required=True)
    return parser.parse_args()


def build_manifest(primary_rays, gt_samples):
    if not gt_samples:
        raise ValueError("Ground-truth trajectory is empty")
    reference = gt_samples[0]
    output = []
    for ray in primary_rays:
        receiver_enu = ecef_to_local_enu(
            ray["rx_ecef"][None, :], reference["ecef"],
            reference["latitude"], reference["longitude"],
        )[0]
        output.append({
            "epoch_utc": "%.6f" % ray["epoch_utc"],
            "sat_id": ray["sat_id"],
            "sys": ray["sys"],
            "signal_records": ray["signal_records"],
            "signals": ray["signals"],
            "tile_window_index": ray["tile_window_index"],
            "azimuth_deg": ray["azimuth_deg"],
            "elevation_deg": ray["elevation_deg"],
            "receiver_e_m": float(receiver_enu[0]),
            "receiver_n_m": float(receiver_enu[1]),
            "receiver_u_m": float(receiver_enu[2]),
            "receiver_x_ecef_m": float(ray["rx_ecef"][0]),
            "receiver_y_ecef_m": float(ray["rx_ecef"][1]),
            "receiver_z_ecef_m": float(ray["rx_ecef"][2]),
            "analysis_group": "PRIMARY",
            "direct_surface_audit_available": 0,
        })
    return output


def write_csv(path, rows):
    if not rows:
        raise ValueError("Cannot write empty ray manifest")
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def describe(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return {"n": 0, "min": None, "median": None, "p95": None, "max": None}
    return {
        "n": int(len(values)), "min": float(np.min(values)),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)), "max": float(np.max(values)),
    }


def main():
    args = parse_args()
    print("Reading all PRIMARY signal rows and deduplicating geometric rays...")
    primary_rays = read_primary_rays(args.eligibility)
    print("Reading GT reference frame...")
    gt_samples = read_ground_truth(args.gt)
    rows = build_manifest(primary_rays, gt_samples)
    write_csv(args.out, rows)

    tile_counts = Counter(row["tile_window_index"] for row in rows)
    system_counts = Counter(row["sys"] for row in rows)
    signal_records = sum(row["signal_records"] for row in rows)
    epochs = set(row["epoch_utc"] for row in rows)
    satellites = set(row["sat_id"] for row in rows)
    decision = (
        "ALL_PRIMARY_RAYS_READY"
        if len(tile_counts) >= 40 and len(rows) >= 1000 and len(system_counts) >= 3
        else "REVIEW_PRIMARY_RAY_COVERAGE"
    )
    audit = {
        "schema_version": 1,
        "decision": decision,
        "inputs": {
            "eligibility": os.path.abspath(args.eligibility),
            "gt": os.path.abspath(args.gt),
        },
        "results": {
            "signal_records_represented": signal_records,
            "unique_epoch_satellite_rays": len(rows),
            "unique_epochs": len(epochs),
            "unique_satellites": len(satellites),
            "tiles": len(tile_counts),
            "tile_indices": sorted(tile_counts),
            "rays_per_tile": describe(list(tile_counts.values())),
            "rays_by_constellation": dict(sorted(system_counts.items())),
            "signal_records_per_ray": describe([
                row["signal_records"] for row in rows
            ]),
        },
        "reference_frame": {
            "gt_first_epoch_utc": gt_samples[0]["time"],
            "latitude_deg": gt_samples[0]["latitude"],
            "longitude_deg": gt_samples[0]["longitude"],
            "ecef_m": list(gt_samples[0]["ecef"]),
            "frame": "local_ENU",
        },
        "interpretation_guards": [
            "Frequency channels are deduplicated only for geometry; signal multiplicity and labels are retained",
            "Only rows preclassified PRIMARY by the frozen tile-eligibility audit are included",
            "No LiDAR surface hit, facade exposure, or C/N0 outcome is computed",
            "Receiver ENU uses the same first-GT reference as the anchored tile pipeline",
        ],
        "output": os.path.abspath(args.out),
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.audit)), exist_ok=True)
    with open(args.audit, "w", encoding="utf-8") as handle:
        json.dump(audit, handle, indent=2, sort_keys=True)
        handle.write("\n")

    print("\n=== UrbanV2X all-PRIMARY ray manifest ===")
    print("signal records represented:  %d" % signal_records)
    print("unique epoch-satellite rays: %d" % len(rows))
    print("epochs / satellites / tiles: %d / %d / %d" % (
        len(epochs), len(satellites), len(tile_counts)
    ))
    print("rays by constellation:       %s" % dict(sorted(system_counts.items())))
    print("rays per tile med/p95:       %s / %s" % (
        audit["results"]["rays_per_tile"]["median"],
        audit["results"]["rays_per_tile"]["p95"],
    ))
    print("decision:                    %s" % decision)
    print("Manifest: %s" % args.out)
    print("Audit:    %s" % args.audit)
    print("Caution: this is geometry-only and contains no C/N0 analysis.")


if __name__ == "__main__":
    main()
