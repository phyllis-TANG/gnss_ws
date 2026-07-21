#!/usr/bin/env python3
"""Build pre-C/N0 continuous satellite--facade exposure features.

Satellite rays are intersected with compatible local facade *planes* from the
same GT-anchored tile.  Finite observed facade extents are never silently
treated as infinite: horizontal and vertical extrapolation gaps are retained
and penalised with three predeclared sensitivity scales.  Raw candidate
associations and one-row-per-ray aggregates are both written before any C/N0
outcome is joined.
"""

import argparse
import csv
import json
import math
import os
from collections import Counter, defaultdict

import numpy as np


SCENARIOS = {
    "strict": (1.0, 5.0),
    "primary": (2.5, 10.0),
    "loose": (5.0, 20.0),
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Build UrbanV2X satellite--facade exposure features without C/N0"
    )
    parser.add_argument("--rays", required=True)
    parser.add_argument("--planes", required=True)
    parser.add_argument("--physical", required=True)
    parser.add_argument("--out-candidates", required=True)
    parser.add_argument("--out-rays", required=True)
    parser.add_argument("--audit", required=True)
    parser.add_argument("--out-plot")
    parser.add_argument("--min-intersection", type=float, default=3.0)
    parser.add_argument("--max-intersection", type=float, default=100.0)
    parser.add_argument("--parallel-cos-min", type=float, default=0.05)
    parser.add_argument("--max-horizontal-gap", type=float, default=15.0)
    parser.add_argument("--max-vertical-gap", type=float, default=60.0)
    parser.add_argument("--active-weight-min", type=float, default=0.05)
    return parser.parse_args()


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
        raise ValueError("Missing vector value among %s" % (fields,))
    return np.asarray(values, dtype=np.float64)


def azel_to_enu(azimuth_deg, elevation_deg):
    azimuth = math.radians(float(azimuth_deg))
    elevation = math.radians(float(elevation_deg))
    cosine = math.cos(elevation)
    return np.asarray([
        math.sin(azimuth) * cosine,
        math.cos(azimuth) * cosine,
        math.sin(elevation),
    ], dtype=np.float64)


def interval_gap(value, lower, upper):
    if value < lower:
        return float(lower - value)
    if value > upper:
        return float(value - upper)
    return 0.0


def facade_tier(plane, physical):
    if not is_true(plane, "cluster_geometry_consistent"):
        return "REVIEW_CLUSTER"
    if physical is None or not is_true(physical, "physical_geometry_consistent"):
        return "REVIEW_PHYSICAL"
    if (is_true(physical, "repeated_across_tiles")
            and is_true(physical, "cross_tile_reflectivity_repeatable")):
        return "A_REPEATED"
    if integer(plane, "member_patches") >= 2:
        return "A_MULTI_PATCH"
    return "B_SINGLE_PATCH"


def intersect_ray_plane(origin, direction, plane, min_distance, max_distance,
                        parallel_cos_min):
    centre = vector(plane, ("centroid_e_m", "centroid_n_m", "centroid_u_m"))
    normal = vector(plane, ("normal_e", "normal_n", "normal_u"))
    normal /= np.linalg.norm(normal)
    denominator = float(np.dot(normal, direction))
    crossing_cosine = abs(denominator)
    if crossing_cosine < parallel_cos_min:
        return None
    distance = float(np.dot(normal, centre - origin) / denominator)
    if distance < min_distance or distance > max_distance:
        return None
    point = origin + distance * direction
    tangent = np.asarray([-normal[1], normal[0], 0.0])
    tangent_norm = np.linalg.norm(tangent)
    if tangent_norm <= 1e-12:
        return None
    tangent /= tangent_norm
    horizontal_coordinate = float(np.dot(point, tangent))
    horizontal_gap = interval_gap(
        horizontal_coordinate, number(plane, "horizontal_min_m"),
        number(plane, "horizontal_max_m"),
    )
    vertical_gap = interval_gap(
        float(point[2]), number(plane, "vertical_min_m"),
        number(plane, "vertical_max_m"),
    )
    return {
        "intersection": point,
        "intersection_distance_m": distance,
        "crossing_cosine": crossing_cosine,
        "horizontal_coordinate_m": horizontal_coordinate,
        "horizontal_gap_m": horizontal_gap,
        "vertical_gap_m": vertical_gap,
        "extent_gap_m": math.hypot(horizontal_gap, vertical_gap),
        "within_observed_horizontal": int(horizontal_gap == 0.0),
        "within_observed_vertical": int(vertical_gap == 0.0),
        "within_observed_rectangle": int(horizontal_gap == 0.0 and vertical_gap == 0.0),
    }


def exposure_weight(crossing_cosine, horizontal_gap, vertical_gap,
                    horizontal_scale, vertical_scale):
    return float(crossing_cosine * math.exp(
        -horizontal_gap / horizontal_scale - vertical_gap / vertical_scale
    ))


def read_csv(path):
    with open(path, "r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError("Empty CSV: %s" % path)
    return rows


def read_inputs(ray_path, plane_path, physical_path):
    rays = read_csv(ray_path)
    planes = read_csv(plane_path)
    physical_rows = read_csv(physical_path)
    ray_required = {
        "epoch_utc", "sat_id", "sys", "tile_window_index", "azimuth_deg",
        "elevation_deg", "receiver_e_m", "receiver_n_m", "receiver_u_m",
    }
    plane_required = {
        "plane_observation_id", "physical_facade_id", "tile_window_index",
        "centroid_e_m", "centroid_n_m", "centroid_u_m",
        "normal_e", "normal_n", "normal_u", "horizontal_min_m",
        "horizontal_max_m", "vertical_min_m", "vertical_max_m",
        "intensity_median", "member_patches", "cluster_geometry_consistent",
    }
    physical_required = {
        "physical_facade_id", "physical_geometry_consistent",
        "repeated_across_tiles", "cross_tile_reflectivity_repeatable",
    }
    for label, rows, required in (
        ("rays", rays, ray_required), ("planes", planes, plane_required),
        ("physical", physical_rows, physical_required),
    ):
        missing = sorted(required - set(rows[0]))
        if missing:
            raise ValueError("%s CSV is missing fields: %s" % (label, missing))
    physical = {row["physical_facade_id"]: row for row in physical_rows}
    if len(physical) != len(physical_rows):
        raise ValueError("Duplicate physical_facade_id")
    return rays, planes, physical


def build_candidate(ray, plane, physical, args):
    origin = vector(ray, ("receiver_e_m", "receiver_n_m", "receiver_u_m"))
    direction = azel_to_enu(ray["azimuth_deg"], ray["elevation_deg"])
    intersection = intersect_ray_plane(
        origin, direction, plane, args.min_intersection,
        args.max_intersection, args.parallel_cos_min,
    )
    if intersection is None:
        return None
    if (intersection["horizontal_gap_m"] > args.max_horizontal_gap
            or intersection["vertical_gap_m"] > args.max_vertical_gap):
        return None
    tier = facade_tier(plane, physical)
    if tier.startswith("REVIEW"):
        return None
    point = intersection.pop("intersection")
    output = {
        "epoch_utc": ray["epoch_utc"],
        "sat_id": ray["sat_id"],
        "sys": ray["sys"],
        "tile_window_index": integer(ray, "tile_window_index"),
        "azimuth_deg": number(ray, "azimuth_deg"),
        "elevation_deg": number(ray, "elevation_deg"),
        "plane_observation_id": plane["plane_observation_id"],
        "physical_facade_id": plane["physical_facade_id"],
        "facade_tier": tier,
        "member_patches": integer(plane, "member_patches"),
        "plane_horizontal_span_m": number(plane, "horizontal_span_m"),
        "plane_vertical_span_m": number(plane, "vertical_span_m"),
        "plane_intensity_median": number(plane, "intensity_median"),
        "plane_intensity_iqr_between_patches": number(
            plane, "intensity_iqr_between_patches"
        ),
        "intersection_e_m": float(point[0]),
        "intersection_n_m": float(point[1]),
        "intersection_u_m": float(point[2]),
    }
    output.update(intersection)
    for scenario, (horizontal_scale, vertical_scale) in SCENARIOS.items():
        output["weight_%s" % scenario] = exposure_weight(
            output["crossing_cosine"], output["horizontal_gap_m"],
            output["vertical_gap_m"], horizontal_scale, vertical_scale,
        )
    return output


def aggregate_subset(candidates, scenario, tier_a_only=False):
    if tier_a_only:
        candidates = [row for row in candidates
                      if row["facade_tier"] in ("A_REPEATED", "A_MULTI_PATCH")]
    prefix = "%s_%s" % (scenario, "a" if tier_a_only else "all")
    if not candidates:
        return {
            "%s_candidates" % prefix: 0,
            "%s_unique_facades" % prefix: 0,
            "%s_sum_weight" % prefix: 0.0,
            "%s_max_weight" % prefix: 0.0,
            "%s_reflectivity_weighted" % prefix: None,
            "%s_reflectivity_exposure_sum" % prefix: 0.0,
            "%s_best_plane_id" % prefix: None,
            "%s_best_physical_facade_id" % prefix: None,
            "%s_best_intensity" % prefix: None,
            "%s_best_vertical_gap_m" % prefix: None,
            "%s_best_horizontal_gap_m" % prefix: None,
            "%s_best_distance_m" % prefix: None,
        }
    weight_field = "weight_%s" % scenario
    weights = np.asarray([row[weight_field] for row in candidates], dtype=float)
    intensities = np.asarray([
        row["plane_intensity_median"] if row["plane_intensity_median"] is not None
        else np.nan for row in candidates
    ])
    valid_intensity = np.isfinite(intensities)
    intensity_weight = float(np.sum(weights[valid_intensity]))
    weighted_reflectivity = (
        float(np.sum(weights[valid_intensity] * intensities[valid_intensity])
              / intensity_weight) if intensity_weight > 0.0 else None
    )
    exposure_sum = float(np.sum(
        weights[valid_intensity] * intensities[valid_intensity]
    ))
    best_index = int(np.argmax(weights))
    best = candidates[best_index]
    return {
        "%s_candidates" % prefix: len(candidates),
        "%s_unique_facades" % prefix: len(set(
            row["physical_facade_id"] for row in candidates
        )),
        "%s_sum_weight" % prefix: float(np.sum(weights)),
        "%s_max_weight" % prefix: float(weights[best_index]),
        "%s_reflectivity_weighted" % prefix: weighted_reflectivity,
        "%s_reflectivity_exposure_sum" % prefix: exposure_sum,
        "%s_best_plane_id" % prefix: best["plane_observation_id"],
        "%s_best_physical_facade_id" % prefix: best["physical_facade_id"],
        "%s_best_intensity" % prefix: best["plane_intensity_median"],
        "%s_best_vertical_gap_m" % prefix: best["vertical_gap_m"],
        "%s_best_horizontal_gap_m" % prefix: best["horizontal_gap_m"],
        "%s_best_distance_m" % prefix: best["intersection_distance_m"],
    }


def aggregate_ray(ray, candidates, args):
    direct_available = (
        "surface_hit" in ray and ray.get("surface_hit") not in ("", None, "None")
    )
    output = {
        "epoch_utc": ray["epoch_utc"],
        "sat_id": ray["sat_id"],
        "sys": ray["sys"],
        "signal_records": ray.get("signal_records"),
        "signals": ray.get("signals"),
        "tile_window_index": integer(ray, "tile_window_index"),
        "azimuth_deg": number(ray, "azimuth_deg"),
        "elevation_deg": number(ray, "elevation_deg"),
        "receiver_e_m": number(ray, "receiver_e_m"),
        "receiver_n_m": number(ray, "receiver_n_m"),
        "receiver_u_m": number(ray, "receiver_u_m"),
        "direct_surface_audit_available": int(direct_available),
        "direct_surface_hit": int(is_true(ray, "surface_hit")) if direct_available else None,
        "direct_patch_valid": int(is_true(ray, "patch_valid")) if direct_available else None,
        "plane_intersection_candidates": len(candidates),
        "unique_physical_facades": len(set(
            row["physical_facade_id"] for row in candidates
        )),
        "exact_observed_rectangle_candidates": sum(
            row["within_observed_rectangle"] == 1 for row in candidates
        ),
        "tier_a_repeated_candidates": sum(
            row["facade_tier"] == "A_REPEATED" for row in candidates
        ),
        "tier_a_multi_patch_candidates": sum(
            row["facade_tier"] == "A_MULTI_PATCH" for row in candidates
        ),
        "tier_b_single_patch_candidates": sum(
            row["facade_tier"] == "B_SINGLE_PATCH" for row in candidates
        ),
        "minimum_horizontal_gap_m": min(
            (row["horizontal_gap_m"] for row in candidates), default=None
        ),
        "minimum_vertical_gap_m": min(
            (row["vertical_gap_m"] for row in candidates), default=None
        ),
        "minimum_extent_gap_m": min(
            (row["extent_gap_m"] for row in candidates), default=None
        ),
        "nearest_intersection_distance_m": min(
            (row["intersection_distance_m"] for row in candidates), default=None
        ),
    }
    for scenario in SCENARIOS:
        output.update(aggregate_subset(candidates, scenario, False))
        output.update(aggregate_subset(candidates, scenario, True))
        output["%s_all_active" % scenario] = int(
            output["%s_all_max_weight" % scenario] >= args.active_weight_min
        )
        output["%s_a_active" % scenario] = int(
            output["%s_a_max_weight" % scenario] >= args.active_weight_min
        )
    return output


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


def spearman(left, right):
    pairs = [(a, b) for a, b in zip(left, right)
             if a is not None and b is not None
             and math.isfinite(a) and math.isfinite(b)]
    if len(pairs) < 3:
        return {"n": len(pairs), "r": None, "p": None}
    try:
        from scipy.stats import spearmanr
    except ImportError:
        return {"n": len(pairs), "r": None, "p": None,
                "error": "scipy_not_installed"}
    result = spearmanr([value[0] for value in pairs],
                       [value[1] for value in pairs])
    statistic = float(result.statistic)
    pvalue = float(result.pvalue)
    return {
        "n": len(pairs),
        "r": statistic if math.isfinite(statistic) else None,
        "p": pvalue if math.isfinite(pvalue) else None,
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


def elevation_bin(value):
    value = float(value)
    for lower, upper in ((0, 15), (15, 30), (30, 45), (45, 60), (60, 91)):
        if lower <= value < upper:
            return "%d-%d" % (lower, min(upper, 90))
    return "outside"


def summarise_bins(ray_rows):
    grouped = defaultdict(list)
    for row in ray_rows:
        grouped[elevation_bin(row["elevation_deg"])].append(row)
    output = []
    for label in ("0-15", "15-30", "30-45", "45-60", "60-90"):
        rows = grouped.get(label, [])
        if not rows:
            continue
        output.append({
            "elevation_bin_deg": label,
            "rays": len(rows),
            "with_candidates": sum(row["plane_intersection_candidates"] > 0
                                   for row in rows),
            "primary_all_active": sum(row["primary_all_active"] for row in rows),
            "primary_a_active": sum(row["primary_a_active"] for row in rows),
            "primary_all_active_ratio": sum(
                row["primary_all_active"] for row in rows
            ) / len(rows),
            "primary_a_active_ratio": sum(
                row["primary_a_active"] for row in rows
            ) / len(rows),
            "primary_max_weight_median": float(np.median([
                row["primary_all_max_weight"] for row in rows
            ])),
            "vertical_gap_median_m": (
                float(np.median([
                    row["minimum_vertical_gap_m"] for row in rows
                    if row["minimum_vertical_gap_m"] is not None
                ])) if any(row["minimum_vertical_gap_m"] is not None
                            for row in rows) else None
            ),
        })
    return output


def write_plot(path, ray_rows):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False, "matplotlib_not_installed"
    figure, axes = plt.subplots(1, 2, figsize=(12, 4.8))
    elevation = [row["elevation_deg"] for row in ray_rows]
    axes[0].scatter(elevation, [row["primary_all_max_weight"] for row in ray_rows],
                    s=10, alpha=0.5)
    axes[0].set_xlabel("Satellite elevation [deg]")
    axes[0].set_ylabel("Primary maximum facade weight")
    active = [row for row in ray_rows if row["primary_all_active"] == 1]
    if active:
        axes[1].scatter(
            [row["primary_all_reflectivity_weighted"] for row in active],
            [row["primary_all_sum_weight"] for row in active],
            s=10, alpha=0.5,
        )
    axes[1].set_xlabel("Primary weighted facade intensity")
    axes[1].set_ylabel("Primary total exposure weight")
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
        args.min_intersection, args.max_intersection, args.parallel_cos_min,
        args.max_horizontal_gap, args.max_vertical_gap, args.active_weight_min,
    ]
    if any(value <= 0.0 for value in positive):
        raise ValueError("All geometric thresholds must be positive")
    if args.max_intersection <= args.min_intersection:
        raise ValueError("Maximum intersection must exceed minimum")
    if args.parallel_cos_min >= 1.0 or args.active_weight_min >= 1.0:
        raise ValueError("Cosine/weight screens must be below one")

    rays, planes, physical = read_inputs(
        args.rays, args.planes, args.physical
    )
    planes_by_tile = defaultdict(list)
    tier_counts = Counter()
    for plane in planes:
        physical_row = physical.get(plane["physical_facade_id"])
        tier = facade_tier(plane, physical_row)
        tier_counts[tier] += 1
        if not tier.startswith("REVIEW"):
            planes_by_tile[integer(plane, "tile_window_index")].append(plane)
    print("=== UrbanV2X satellite--facade exposure ===")
    print("rays=%d plane observations=%d usable=%d" % (
        len(rays), len(planes), sum(len(values) for values in planes_by_tile.values())
    ))
    print("facade tiers: %s" % dict(sorted(tier_counts.items())))

    candidate_rows = []
    ray_rows = []
    for index, ray in enumerate(rays, 1):
        tile_index = integer(ray, "tile_window_index")
        candidates = []
        for plane in planes_by_tile.get(tile_index, []):
            candidate = build_candidate(
                ray, plane, physical.get(plane["physical_facade_id"]), args
            )
            if candidate is not None:
                candidates.append(candidate)
        candidates.sort(key=lambda row: (
            -row["weight_primary"], row["intersection_distance_m"],
            row["plane_observation_id"],
        ))
        candidate_rows.extend(candidates)
        ray_rows.append(aggregate_ray(ray, candidates, args))
        if index % 200 == 0:
            print("  processed %d/%d rays; candidates=%d" % (
                index, len(rays), len(candidate_rows)
            ))

    # Preserve an explicit empty-association row only in the per-ray table;
    # the candidate table contains genuine plane intersections only.
    if candidate_rows:
        write_csv(args.out_candidates, candidate_rows)
    else:
        raise ValueError("No satellite--facade plane candidates were produced")
    write_csv(args.out_rays, ray_rows)
    elevation_rows = summarise_bins(ray_rows)
    active_primary = [row for row in ray_rows if row["primary_all_active"] == 1]
    active_primary_a = [row for row in ray_rows if row["primary_a_active"] == 1]
    active_systems = sorted(set(row["sys"] for row in active_primary))
    enough = len(active_primary) >= 200 and len(active_systems) >= 3
    decision = (
        "PRE_CN0_EXPOSURE_FEATURES_AVAILABLE"
        if enough else "REVIEW_EXPOSURE_FEATURE_COVERAGE"
    )
    plot_written = False
    plot_error = None
    if args.out_plot:
        plot_written, plot_error = write_plot(args.out_plot, ray_rows)
    audit = {
        "schema_version": 1,
        "decision": decision,
        "feature_definition_frozen_before_cn0_join": True,
        "inputs": {
            "rays": os.path.abspath(args.rays),
            "planes": os.path.abspath(args.planes),
            "physical": os.path.abspath(args.physical),
        },
        "parameters": {
            "min_intersection_m": args.min_intersection,
            "max_intersection_m": args.max_intersection,
            "parallel_cos_min": args.parallel_cos_min,
            "max_horizontal_gap_m": args.max_horizontal_gap,
            "max_vertical_gap_m": args.max_vertical_gap,
            "active_weight_min": args.active_weight_min,
            "scenarios": {
                name: {"horizontal_scale_m": values[0],
                       "vertical_scale_m": values[1]}
                for name, values in SCENARIOS.items()
            },
            "weight_formula": "abs(n_dot_d)*exp(-horizontal_gap/H-vertical_gap/V)",
            "tier_a_definition": "A_REPEATED or A_MULTI_PATCH",
        },
        "thresholds_are_pre_cn0_project_screens_not_field_standards": {
            "active_rays_min": 200,
            "active_constellations_min": 3,
        },
        "facade_tiers": dict(sorted(tier_counts.items())),
        "results": {
            "rays": len(ray_rows),
            "candidate_associations": len(candidate_rows),
            "rays_with_candidates": sum(
                row["plane_intersection_candidates"] > 0 for row in ray_rows
            ),
            "rays_with_exact_observed_rectangle": sum(
                row["exact_observed_rectangle_candidates"] > 0 for row in ray_rows
            ),
            "primary_active_rays": len(active_primary),
            "primary_active_ratio": len(active_primary) / len(ray_rows),
            "primary_tier_a_active_rays": len(active_primary_a),
            "primary_tier_a_active_ratio": len(active_primary_a) / len(ray_rows),
            "primary_active_constellations": active_systems,
            "minimum_horizontal_gap_m": describe([
                row["minimum_horizontal_gap_m"] for row in ray_rows
            ]),
            "minimum_vertical_gap_m": describe([
                row["minimum_vertical_gap_m"] for row in ray_rows
            ]),
            "candidate_vertical_gap_m": describe([
                row["vertical_gap_m"] for row in candidate_rows
            ]),
            "candidate_horizontal_gap_m": describe([
                row["horizontal_gap_m"] for row in candidate_rows
            ]),
            "candidate_intersection_distance_m": describe([
                row["intersection_distance_m"] for row in candidate_rows
            ]),
            "scenario_active_rays": {
                scenario: {
                    "all": sum(row["%s_all_active" % scenario] for row in ray_rows),
                    "tier_a": sum(row["%s_a_active" % scenario] for row in ray_rows),
                } for scenario in SCENARIOS
            },
            "scenario_max_weight": {
                scenario: describe([
                    row["%s_all_max_weight" % scenario] for row in ray_rows
                ]) for scenario in SCENARIOS
            },
            "scenario_weight_spearman": {
                "strict_primary": spearman(
                    [row["strict_all_sum_weight"] for row in ray_rows],
                    [row["primary_all_sum_weight"] for row in ray_rows],
                ),
                "primary_loose": spearman(
                    [row["primary_all_sum_weight"] for row in ray_rows],
                    [row["loose_all_sum_weight"] for row in ray_rows],
                ),
            },
        },
        "by_elevation_bin": elevation_rows,
        "by_constellation": [{
            "sys": system,
            "rays": sum(row["sys"] == system for row in ray_rows),
            "primary_active": sum(
                row["sys"] == system and row["primary_all_active"] == 1
                for row in ray_rows
            ),
            "primary_tier_a_active": sum(
                row["sys"] == system and row["primary_a_active"] == 1
                for row in ray_rows
            ),
        } for system in sorted(set(row["sys"] for row in ray_rows))],
        "interpretation_guards": [
            "No C/N0 field is read or used",
            "Plane intensity is a LiDAR proxy and is not used to create geometric membership",
            "An infinite-plane intersection is retained only with explicit finite-extent extrapolation gaps",
            "Exposure weights are continuous confidence features, not LOS/NLOS labels",
            "Strict/primary/loose scales are fixed sensitivity scenarios and must not be selected by outcome significance",
            "Tier-A and all-facade aggregates are both retained to expose singleton-patch sensitivity",
            "Distance, crossing cosine, extent gaps, plane identity, and raw candidate rows remain available for alternative physical models",
        ],
        "recommended_next_step": (
            "freeze_commit_then_join_reference_cn0_and_run_clustered_exploratory_models"
            if enough else "review_pre_cn0_geometry_coverage_without_using_cn0"
        ),
        "outputs": {
            "candidates_csv": os.path.abspath(args.out_candidates),
            "rays_csv": os.path.abspath(args.out_rays),
            "plot": os.path.abspath(args.out_plot) if args.out_plot else None,
            "plot_written": plot_written,
            "plot_error": plot_error,
        },
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.audit)), exist_ok=True)
    with open(args.audit, "w", encoding="utf-8") as handle:
        json.dump(audit, handle, indent=2, sort_keys=True)
        handle.write("\n")

    print("\n=== Pre-C/N0 facade exposure summary ===")
    print("rays:                              %d" % len(ray_rows))
    print("candidate associations:            %d" % len(candidate_rows))
    print("rays with candidates:              %d" % audit["results"][
        "rays_with_candidates"
    ])
    print("rays with exact observed extent:   %d" % audit["results"][
        "rays_with_exact_observed_rectangle"
    ])
    print("primary active rays:               %d (%.1f%%)" % (
        len(active_primary), 100.0 * len(active_primary) / len(ray_rows)
    ))
    print("primary Tier-A active rays:        %d (%.1f%%)" % (
        len(active_primary_a), 100.0 * len(active_primary_a) / len(ray_rows)
    ))
    print("active constellations:             %s" % ",".join(active_systems))
    print("scenario active rays all/Tier-A:")
    for scenario in SCENARIOS:
        values = audit["results"]["scenario_active_rays"][scenario]
        print("  %-8s %d / %d" % (scenario, values["all"], values["tier_a"]))
    print("minimum vertical gap med/p95:      %s / %s m" % (
        audit["results"]["minimum_vertical_gap_m"]["median"],
        audit["results"]["minimum_vertical_gap_m"]["p95"],
    ))
    print("minimum horizontal gap med/p95:    %s / %s m" % (
        audit["results"]["minimum_horizontal_gap_m"]["median"],
        audit["results"]["minimum_horizontal_gap_m"]["p95"],
    ))
    print("by satellite elevation:")
    print("bin,rays,primary_active,A_active,vertical_gap_median")
    for row in elevation_rows:
        print("%s,%d,%d,%d,%s" % (
            row["elevation_bin_deg"], row["rays"], row["primary_all_active"],
            row["primary_a_active"], row["vertical_gap_median_m"],
        ))
    print("decision:                          %s" % decision)
    print("recommended next step:             %s" % audit["recommended_next_step"])
    print("Candidates CSV: %s" % args.out_candidates)
    print("Ray features:   %s" % args.out_rays)
    print("Audit:          %s" % args.audit)
    if args.out_plot:
        print("Plot:           %s (%s)" % (
            args.out_plot, "written" if plot_written else plot_error
        ))
    print("Caution: exposure is a frozen geometric feature, not an attenuation result.")


if __name__ == "__main__":
    main()
