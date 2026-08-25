#!/usr/bin/env python3
"""Build an outcome-independent consensus audit across three ray radii.

The strict, primary, and loose ray casts are interpreted as spatial confidence
bands.  Their hit sets must be nested.  A high-confidence consensus patch must
be valid at all three radii, retain the same first surface within transparent
distance/normal screens, and pass geometry-only patch quality screens.  C/N0
is never read or used by this script.
"""

import argparse
import csv
import json
import math
import os
from collections import Counter

import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(
        description="Audit strict/primary/loose UrbanV2X surface consensus"
    )
    parser.add_argument("--strict", required=True)
    parser.add_argument("--primary", required=True)
    parser.add_argument("--loose", required=True)
    parser.add_argument("--out-csv", required=True)
    parser.add_argument("--audit", required=True)
    parser.add_argument("--distance-spread-max", type=float, default=0.50)
    parser.add_argument("--normal-angle-max", type=float, default=20.0)
    parser.add_argument("--planarity-min", type=float, default=0.30)
    parser.add_argument("--plane-residual-max", type=float, default=0.15)
    parser.add_argument("--diffuse-points-min", type=int, default=12)
    parser.add_argument("--incidence-max", type=float, default=80.0)
    parser.add_argument("--retro-fraction-max", type=float, default=0.20)
    return parser.parse_args()


def key_for(row):
    return round(float(row["epoch_utc"]), 6), row["sat_id"]


def read_rows(path):
    with open(path, "r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    indexed = {}
    for row in rows:
        key = key_for(row)
        if key in indexed:
            raise ValueError("Duplicate epoch-satellite key in %s: %s" % (path, key))
        indexed[key] = row
    if not indexed:
        raise ValueError("Empty ray CSV: %s" % path)
    return indexed


def number(row, field):
    value = row.get(field, "")
    if value in ("", "None", None):
        return None
    output = float(value)
    return output if math.isfinite(output) else None


def is_true(row, field):
    return row.get(field) in ("1", "1.0", "True", "true")


def normal(row):
    values = [number(row, field) for field in ("normal_x", "normal_y", "normal_z")]
    if any(value is None for value in values):
        return None
    vector = np.asarray(values, dtype=np.float64)
    norm = np.linalg.norm(vector)
    return vector / norm if norm > 0.0 else None


def unsigned_normal_angle(left, right):
    left = np.asarray(left, dtype=np.float64)
    right = np.asarray(right, dtype=np.float64)
    cosine = float(np.clip(abs(np.dot(left, right)), 0.0, 1.0))
    return math.degrees(math.acos(cosine))


def max_pairwise_normal_angle(rows):
    normals = [normal(row) for row in rows]
    if any(value is None for value in normals):
        return None
    return max(
        unsigned_normal_angle(normals[left], normals[right])
        for left in range(len(normals))
        for right in range(left + 1, len(normals))
    )


def confidence_band(pattern):
    return {
        "111": "CORE_STRICT",
        "011": "MID_TOLERANCE",
        "001": "LOOSE_ONLY",
        "000": "LOCAL_MAP_UNOBSERVED",
    }.get(pattern, "NON_MONOTONIC")


def quality_reasons(row, args):
    checks = [
        ("few_diffuse_points", "patch_diffuse_points", args.diffuse_points_min, "min"),
        ("low_planarity", "planarity", args.planarity_min, "min"),
        ("thick_surface", "plane_residual_median_m", args.plane_residual_max, "max"),
        ("grazing_incidence", "incidence_deg", args.incidence_max, "max"),
        ("retro_dominated", "patch_retro_fraction", args.retro_fraction_max, "max"),
    ]
    reasons = []
    for reason, field, threshold, direction in checks:
        value = number(row, field)
        if value is None:
            reasons.append("missing_%s" % field)
        elif direction == "min" and value < threshold:
            reasons.append(reason)
        elif direction == "max" and value > threshold:
            reasons.append(reason)
    return reasons


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


def spearman(left, right):
    if len(left) < 3:
        return {"n": len(left), "r": None, "p": None}
    try:
        from scipy.stats import spearmanr
    except ImportError:
        return {"n": len(left), "r": None, "p": None, "error": "scipy_not_installed"}
    result = spearmanr(left, right)
    return {"n": len(left), "r": float(result.statistic), "p": float(result.pvalue)}


def write_csv(path, rows):
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


def main():
    args = parse_args()
    if args.distance_spread_max <= 0.0 or args.normal_angle_max <= 0.0:
        raise ValueError("Consensus screens must be positive")
    datasets = {
        "strict": read_rows(args.strict),
        "primary": read_rows(args.primary),
        "loose": read_rows(args.loose),
    }
    key_sets = {name: set(rows) for name, rows in datasets.items()}
    if not (key_sets["strict"] == key_sets["primary"] == key_sets["loose"]):
        raise ValueError("Strict/primary/loose CSV key sets differ")

    output_rows = []
    patterns = Counter()
    exclusions = Counter()
    intensity_pairs = {"strict_primary": [[], []], "primary_loose": [[], []]}
    for key in sorted(key_sets["strict"]):
        strict = datasets["strict"][key]
        primary = datasets["primary"][key]
        loose = datasets["loose"][key]
        radius_rows = [strict, primary, loose]
        pattern = "".join("1" if is_true(row, "surface_hit") else "0"
                          for row in radius_rows)
        band = confidence_band(pattern)
        patterns[pattern] += 1
        all_patch_valid = all(is_true(row, "patch_valid") for row in radius_rows)

        distances = [number(row, "hit_distance_m") for row in radius_rows]
        intensities = [number(row, "patch_intensity_median") for row in radius_rows]
        distance_spread = None
        intensity_spread = None
        normal_angle = None
        if all_patch_valid:
            distance_spread = max(distances) - min(distances)
            intensity_spread = max(intensities) - min(intensities)
            normal_angle = max_pairwise_normal_angle(radius_rows)
            intensity_pairs["strict_primary"][0].append(intensities[0])
            intensity_pairs["strict_primary"][1].append(intensities[1])
            intensity_pairs["primary_loose"][0].append(intensities[1])
            intensity_pairs["primary_loose"][1].append(intensities[2])

        same_surface_reasons = []
        if pattern != "111":
            same_surface_reasons.append("not_strict_hit")
        if not all_patch_valid:
            same_surface_reasons.append("not_all_radii_patch_valid")
        if distance_spread is not None and distance_spread > args.distance_spread_max:
            same_surface_reasons.append("first_surface_distance_switch")
        if normal_angle is not None and normal_angle > args.normal_angle_max:
            same_surface_reasons.append("surface_normal_switch")
        same_surface = not same_surface_reasons

        patch_reasons = quality_reasons(primary, args) if is_true(
            primary, "patch_valid"
        ) else ["primary_patch_invalid"]
        consensus_eligible = same_surface and not patch_reasons
        if not consensus_eligible:
            exclusions.update(same_surface_reasons + patch_reasons)

        output = dict(primary)
        output.update({
            "hit_pattern_strict_primary_loose": pattern,
            "spatial_confidence_band": band,
            "all_radii_patch_valid": int(all_patch_valid),
            "hit_distance_strict_m": distances[0],
            "hit_distance_primary_m": distances[1],
            "hit_distance_loose_m": distances[2],
            "hit_distance_spread_m": distance_spread,
            "intensity_strict": intensities[0],
            "intensity_primary": intensities[1],
            "intensity_loose": intensities[2],
            "intensity_spread": intensity_spread,
            "normal_max_pairwise_angle_deg": normal_angle,
            "same_surface_consensus": int(same_surface),
            "same_surface_reasons": ";".join(same_surface_reasons) or "eligible",
            "primary_geometry_quality": int(not patch_reasons),
            "primary_quality_reasons": ";".join(patch_reasons) or "eligible",
            "consensus_eligible": int(consensus_eligible),
        })
        output_rows.append(output)

    write_csv(args.out_csv, output_rows)
    eligible = [row for row in output_rows if row["consensus_eligible"] == 1]
    all_valid = [row for row in output_rows if row["all_radii_patch_valid"] == 1]
    nonmonotonic = sum(confidence_band(pattern) == "NON_MONOTONIC"
                       for pattern in patterns.elements())
    audit = {
        "schema_version": 1,
        "decision": "CONSENSUS_AUDITED" if nonmonotonic == 0 else "REVIEW_NONMONOTONIC_HITS",
        "inputs": {name: os.path.abspath(path) for name, path in {
            "strict": args.strict, "primary": args.primary, "loose": args.loose,
        }.items()},
        "thresholds_are_pre_cn0_project_screens": {
            "distance_spread_max_m": args.distance_spread_max,
            "normal_angle_max_deg": args.normal_angle_max,
            "planarity_min": args.planarity_min,
            "plane_residual_max_m": args.plane_residual_max,
            "diffuse_points_min": args.diffuse_points_min,
            "incidence_max_deg": args.incidence_max,
            "retro_fraction_max": args.retro_fraction_max,
        },
        "rows": len(output_rows),
        "hit_patterns": dict(sorted(patterns.items())),
        "confidence_bands": dict(Counter(
            row["spatial_confidence_band"] for row in output_rows
        )),
        "all_radii_patch_valid": len(all_valid),
        "same_surface_consensus": sum(
            row["same_surface_consensus"] == 1 for row in output_rows
        ),
        "primary_geometry_quality": sum(
            row["primary_geometry_quality"] == 1 for row in output_rows
        ),
        "consensus_eligible": len(eligible),
        "consensus_eligible_by_constellation": dict(Counter(
            row["sys"] for row in eligible
        )),
        "exclusion_reasons": dict(exclusions.most_common()),
        "all_valid_patch_stability": {
            "hit_distance_spread_m": describe([
                number(row, "hit_distance_spread_m") for row in all_valid
            ]),
            "normal_max_pairwise_angle_deg": describe([
                number(row, "normal_max_pairwise_angle_deg") for row in all_valid
            ]),
            "intensity_spread": describe([
                number(row, "intensity_spread") for row in all_valid
            ]),
            "intensity_spearman_strict_primary": spearman(
                *intensity_pairs["strict_primary"]
            ),
            "intensity_spearman_primary_loose": spearman(
                *intensity_pairs["primary_loose"]
            ),
        },
        "interpretation_guards": [
            "No C/N0 field is read or used by this audit",
            "CORE_STRICT requires a hit at all three radii but is not by itself a material label",
            "Intensity stability is reported but is not used to select consensus samples",
            "LOCAL_MAP_UNOBSERVED is not a LOS classification",
            "Thresholds are transparent project screens, not field standards",
        ],
        "output_csv": os.path.abspath(args.out_csv),
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.audit)), exist_ok=True)
    with open(args.audit, "w", encoding="utf-8") as handle:
        json.dump(audit, handle, indent=2, sort_keys=True)
        handle.write("\n")

    print("=== UrbanV2X three-radius surface consensus ===")
    print("rows:                         %d" % len(output_rows))
    print("hit patterns:")
    for pattern, count in sorted(patterns.items()):
        print("  %s %-22s %d" % (pattern, confidence_band(pattern), count))
    print("all-radii valid patches:      %d" % len(all_valid))
    print("same-surface consensus:       %d" % audit["same_surface_consensus"])
    print("primary geometry quality:     %d" % audit["primary_geometry_quality"])
    print("final consensus eligible:     %d" % len(eligible))
    print("eligible by constellation:    %s" % audit[
        "consensus_eligible_by_constellation"
    ])
    print("exclusion reasons:")
    for reason, count in exclusions.most_common():
        print("  %-32s %d" % (reason, count))
    stability = audit["all_valid_patch_stability"]
    print("distance spread med/p95:      %s / %s m" % (
        stability["hit_distance_spread_m"]["median"],
        stability["hit_distance_spread_m"]["p95"],
    ))
    print("normal spread med/p95:        %s / %s deg" % (
        stability["normal_max_pairwise_angle_deg"]["median"],
        stability["normal_max_pairwise_angle_deg"]["p95"],
    ))
    print("intensity spread med/p95:     %s / %s" % (
        stability["intensity_spread"]["median"],
        stability["intensity_spread"]["p95"],
    ))
    print("decision:                     %s" % audit["decision"])
    print("CSV:   %s" % args.out_csv)
    print("Audit: %s" % args.audit)
    print("Caution: consensus eligibility is pre-C/N0 geometry screening only.")


if __name__ == "__main__":
    main()
