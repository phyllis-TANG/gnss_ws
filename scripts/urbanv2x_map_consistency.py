#!/usr/bin/env python3
"""Compare short-segment UrbanV2X LiDAR motion-compensation hypotheses.

The experiment builds three otherwise identical local intensity maps using
point pose times ``header-lag``, ``header``, and ``header+lag``.  It compares
voxel concentration, local plane thickness, ground-cell thickness, and
cross-frame nearest-neighbour residuals.  The supplied LiDAR->IMU refinement
is used.  GT roll/pitch/heading are interpreted as navigation angles for an
FLU body frame in ENU.

Important limitation: UrbanV2X does not provide an explicit rigid transform
between the Xsens IMU used by the LiDAR calibration and the GNSS/INS GT
reference point.  This test temporarily treats those origins as coincident;
the limitation is recorded in the audit and prevents an automatic physical
claim when candidate differences are marginal.
"""

import argparse
import csv
import json
import math
import os
import statistics
import struct
import sys

import numpy as np


SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIRECTORY not in sys.path:
    sys.path.insert(0, SCRIPT_DIRECTORY)

from urbanv2x_lidar_gt_audit import parse_calibration, read_ground_truth  # noqa: E402


def parse_args():
    parser = argparse.ArgumentParser(
        description="Compare UrbanV2X short-segment map consistency"
    )
    parser.add_argument("--bag", required=True)
    parser.add_argument("--gt", required=True)
    parser.add_argument("--calibration", required=True)
    parser.add_argument("--frames-audit", required=True)
    parser.add_argument("--topic", default="/velodyne_points")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument(
        "--start-utc", type=float,
        help="Manual segment start; default automatically selects dynamic window",
    )
    parser.add_argument("--point-stride", type=int, default=10)
    parser.add_argument("--min-range", type=float, default=2.0)
    parser.add_argument("--max-range", type=float, default=50.0)
    parser.add_argument("--map-voxel", type=float, default=0.15)
    parser.add_argument("--out-prefix", required=True)
    parser.add_argument("--out-csv", required=True)
    parser.add_argument("--audit", required=True)
    return parser.parse_args()


def percentile(values, probability):
    values = np.asarray(values, dtype=np.float64)
    if values.size == 0:
        return None
    return float(np.percentile(values, 100.0 * probability))


def describe(values):
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return {"n": 0, "min": None, "p05": None, "median": None,
                "p95": None, "max": None}
    return {
        "n": int(values.size),
        "min": float(np.min(values)),
        "p05": percentile(values, 0.05),
        "median": float(np.median(values)),
        "p95": percentile(values, 0.95),
        "max": float(np.max(values)),
    }


def angle_difference_degrees(left, right):
    return (np.asarray(left) - np.asarray(right) + 180.0) % 360.0 - 180.0


def read_frame_bounds(path):
    times = []
    with open(path, "r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        if "header_time" not in (reader.fieldnames or []):
            raise ValueError("Frame audit CSV is missing header_time")
        for row in reader:
            times.append(float(row["header_time"]))
    if not times:
        raise ValueError("Frame audit CSV contains no frames")
    return min(times), max(times)


def choose_dynamic_window(samples, lidar_start, lidar_end, duration):
    if lidar_end - lidar_start < duration:
        raise ValueError("LiDAR coverage is shorter than requested duration")
    times = np.array([sample["time"] for sample in samples], dtype=np.float64)
    east = np.array([sample["velocity_enu"][0] for sample in samples])
    north = np.array([sample["velocity_enu"][1] for sample in samples])
    speed = np.hypot(east, north)
    heading = np.unwrap(np.radians(
        [sample["heading"] for sample in samples]
    ))
    yaw_rate = np.zeros_like(heading)
    delta_time = np.diff(times)
    valid = delta_time > 0.0
    yaw_rate[1:][valid] = np.abs(
        np.degrees(np.diff(heading)[valid] / delta_time[valid])
    )

    candidates = []
    start = math.ceil(lidar_start)
    final_start = lidar_end - duration
    while start <= final_start:
        mask = (times >= start) & (times <= start + duration)
        if np.count_nonzero(mask) >= max(10, int(duration * 50)):
            median_speed = float(np.median(speed[mask]))
            yaw_p75 = float(np.percentile(yaw_rate[mask], 75))
            moving_ratio = float(np.mean(speed[mask] >= 2.0))
            # Turns and acceleration expose timestamp errors more clearly than
            # constant-velocity straight motion.
            score = moving_ratio * (median_speed + 2.0 * min(yaw_p75, 15.0))
            candidates.append((score, start, median_speed, yaw_p75, moving_ratio))
        start += 2.0
    if not candidates:
        raise ValueError("No eligible GT window inside LiDAR coverage")
    return max(candidates, key=lambda item: item[0])


def ecef_to_local_enu(ecef, reference_ecef, latitude_deg, longitude_deg):
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    rotation = np.array([
        [-math.sin(longitude), math.cos(longitude), 0.0],
        [-math.sin(latitude) * math.cos(longitude),
         -math.sin(latitude) * math.sin(longitude), math.cos(latitude)],
        [math.cos(latitude) * math.cos(longitude),
         math.cos(latitude) * math.sin(longitude), math.sin(latitude)],
    ])
    return (np.asarray(ecef) - np.asarray(reference_ecef)) @ rotation.T


class PoseInterpolator:
    def __init__(self, samples, reference_index=0):
        self.times = np.array([sample["time"] for sample in samples])
        ecef = np.array([sample["ecef"] for sample in samples])
        reference = samples[reference_index]
        self.positions = ecef_to_local_enu(
            ecef, reference["ecef"], reference["latitude"], reference["longitude"]
        )
        self.roll = np.radians([sample["roll"] for sample in samples])
        self.pitch = np.radians([sample["pitch"] for sample in samples])
        self.heading = np.unwrap(np.radians(
            [sample["heading"] for sample in samples]
        ))

    def interpolate(self, epochs):
        epochs = np.asarray(epochs, dtype=np.float64)
        if np.min(epochs) < self.times[0] or np.max(epochs) > self.times[-1]:
            raise ValueError("Requested point pose is outside GT coverage")
        position = np.column_stack([
            np.interp(epochs, self.times, self.positions[:, axis])
            for axis in range(3)
        ])
        roll = np.interp(epochs, self.times, self.roll)
        pitch = np.interp(epochs, self.times, self.pitch)
        heading = np.interp(epochs, self.times, self.heading)
        return position, roll, pitch, heading


def imu_flu_to_enu(vectors, roll, pitch, heading):
    """Rotate FLU IMU vectors using navigation roll/pitch/heading.

    Equivalent to P_ENU_NED Rz(heading) Ry(pitch) Rx(roll) P_FRD_FLU,
    or Rz(90deg-heading) Ry(-pitch) Rx(roll).
    """
    vectors = np.asarray(vectors, dtype=np.float64)
    x = vectors[:, 0]
    y = vectors[:, 1]
    z = vectors[:, 2]
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    yaw = math.pi / 2.0 - heading
    cy, sy = np.cos(yaw), np.sin(yaw)

    x1 = x
    y1 = cr * y - sr * z
    z1 = sr * y + cr * z
    x2 = cp * x1 - sp * z1
    y2 = y1
    z2 = sp * x1 + cp * z1
    return np.column_stack([
        cy * x2 - sy * y2,
        sy * x2 + cy * y2,
        z2,
    ])


def point_dtype(message):
    fields = {field.name: field for field in message.fields}
    required = {"x", "y", "z", "intensity", "time"}
    missing = required - set(fields)
    if missing:
        raise ValueError("PointCloud2 missing fields: %s" % sorted(missing))
    endian = ">" if message.is_bigendian else "<"
    formats = []
    offsets = []
    names = []
    datatype_formats = {7: "f4", 8: "f8"}
    for name in ("x", "y", "z", "intensity", "time"):
        field = fields[name]
        if field.datatype not in datatype_formats or field.count != 1:
            raise ValueError("Unsupported %s field datatype/count" % name)
        names.append(name)
        offsets.append(int(field.offset))
        formats.append(endian + datatype_formats[field.datatype])
    return np.dtype({
        "names": names,
        "formats": formats,
        "offsets": offsets,
        "itemsize": int(message.point_step),
    })


def extract_frame(message, header_time, stride, min_range, max_range):
    if int(message.row_step) != int(message.width) * int(message.point_step):
        raise ValueError("Organized PointCloud2 row padding is not supported")
    count = int(message.width) * int(message.height)
    points = np.frombuffer(message.data, dtype=point_dtype(message), count=count)
    points = points[::stride]
    xyz = np.column_stack([points["x"], points["y"], points["z"]]).astype(
        np.float64, copy=False
    )
    intensity = np.asarray(points["intensity"], dtype=np.float32)
    relative_time = np.asarray(points["time"], dtype=np.float64)
    ranges = np.linalg.norm(xyz, axis=1)
    valid = (
        np.all(np.isfinite(xyz), axis=1)
        & np.isfinite(intensity)
        & np.isfinite(relative_time)
        & (ranges >= min_range)
        & (ranges <= max_range)
        & (relative_time >= -0.01)
        & (relative_time <= 0.2)
    )
    return {
        "header_time": float(header_time),
        "xyz": xyz[valid],
        "intensity": intensity[valid],
        "relative_time": relative_time[valid],
    }


def read_segment_frames(path, topic, start, end, stride, min_range, max_range):
    try:
        import rosbag
        import rospy
    except ImportError as exc:
        raise RuntimeError("rosbag/rospy are required inside the ROS1 container") from exc
    frames = []
    with rosbag.Bag(path, "r") as bag:
        iterator = bag.read_messages(
            topics=[topic],
            start_time=rospy.Time.from_sec(start - 0.2),
            end_time=rospy.Time.from_sec(end + 0.2),
        )
        for _, message, _ in iterator:
            header_time = message.header.stamp.to_sec()
            if header_time < start or header_time > end:
                continue
            frames.append(extract_frame(
                message, header_time, stride, min_range, max_range
            ))
    if len(frames) < 10:
        raise ValueError("Too few LiDAR frames extracted: %d" % len(frames))
    return frames


def transform_frame(frame, pose, lidar_to_imu, pose_time_offset):
    rotation = lidar_to_imu[:3, :3]
    translation = lidar_to_imu[:3, 3]
    imu_points = frame["xyz"] @ rotation.T + translation
    epochs = frame["header_time"] + frame["relative_time"] + pose_time_offset
    position, roll, pitch, heading = pose.interpolate(epochs)
    world = imu_flu_to_enu(imu_points, roll, pitch, heading) + position
    return {
        "xyz": world.astype(np.float32),
        "intensity": frame["intensity"],
        "local_z": frame["xyz"][:, 2].astype(np.float32),
    }


def voxel_downsample(xyz, intensity, voxel):
    coordinates = np.floor(np.asarray(xyz) / voxel).astype(np.int32)
    unique, inverse, counts = np.unique(
        coordinates, axis=0, return_inverse=True, return_counts=True
    )
    sums = np.column_stack([
        np.bincount(inverse, weights=xyz[:, axis]) for axis in range(3)
    ])
    intensity_sum = np.bincount(inverse, weights=intensity)
    return (
        (sums / counts[:, None]).astype(np.float32),
        (intensity_sum / counts).astype(np.float32),
        unique,
        counts,
    )


def grouped_slices(keys):
    order = np.lexsort(tuple(keys[:, axis] for axis in reversed(range(keys.shape[1]))))
    sorted_keys = keys[order]
    changes = np.any(np.diff(sorted_keys, axis=0) != 0, axis=1)
    boundaries = np.flatnonzero(changes) + 1
    starts = np.r_[0, boundaries]
    ends = np.r_[boundaries, len(order)]
    return order, starts, ends


def ground_thickness(xyz, local_z, cell_size=0.5, minimum_points=20):
    mask = (local_z >= -3.0) & (local_z <= -0.5)
    points = xyz[mask]
    if len(points) == 0:
        return describe([]), 0
    keys = np.floor(points[:, :2] / cell_size).astype(np.int32)
    order, starts, ends = grouped_slices(keys)
    thickness = []
    for start, end in zip(starts, ends):
        if end - start < minimum_points:
            continue
        z = points[order[start:end], 2]
        thickness.append(float(np.percentile(z, 75) - np.percentile(z, 25)))
    return describe(thickness), len(thickness)


def planar_cell_residual(voxel_xyz, coarse_cell=1.0, minimum_points=12):
    if len(voxel_xyz) == 0:
        return describe([]), 0
    keys = np.floor(voxel_xyz / coarse_cell).astype(np.int32)
    order, starts, ends = grouped_slices(keys)
    residuals = []
    for start, end in zip(starts, ends):
        if end - start < minimum_points:
            continue
        points = voxel_xyz[order[start:end]].astype(np.float64)
        covariance = np.cov(points, rowvar=False, bias=True)
        eigenvalues = np.maximum(np.linalg.eigvalsh(covariance), 0.0)
        if eigenvalues[1] <= 1e-8 or eigenvalues[0] / eigenvalues[1] > 0.20:
            continue
        residuals.append(math.sqrt(float(eigenvalues[0])))
    return describe(residuals), len(residuals)


def cross_frame_overlap(transformed_frames, voxel=0.25, separation=5):
    try:
        from scipy.spatial import cKDTree
    except ImportError as exc:
        raise RuntimeError("scipy.spatial.cKDTree is required for overlap metrics") from exc
    distances = []
    for current_index in range(separation, len(transformed_frames), separation):
        previous = transformed_frames[current_index - separation]
        current = transformed_frames[current_index]
        previous_xyz, _, _, _ = voxel_downsample(
            previous["xyz"], previous["intensity"], voxel
        )
        current_xyz, _, _, _ = voxel_downsample(
            current["xyz"], current["intensity"], voxel
        )
        if len(previous_xyz) < 20 or len(current_xyz) < 20:
            continue
        tree = cKDTree(previous_xyz)
        nearest, _ = tree.query(current_xyz, k=1, distance_upper_bound=2.0)
        nearest = nearest[np.isfinite(nearest)]
        if nearest.size:
            distances.append(nearest)
    if not distances:
        return describe([])
    return describe(np.concatenate(distances))


def write_binary_pcd(path, xyz, intensity):
    array = np.column_stack([xyz, intensity]).astype(np.float32)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        "WIDTH %d\nHEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS %d\nDATA binary\n"
    ) % (len(array), len(array))
    with open(path, "wb") as handle:
        handle.write(header.encode("ascii"))
        handle.write(array.tobytes())


def evaluate_candidate(name, offset, frames, pose, matrix, map_voxel, out_prefix):
    transformed = []
    for index, frame in enumerate(frames):
        transformed.append(transform_frame(frame, pose, matrix, offset))
        if (index + 1) % 50 == 0:
            print("  %s transformed %d/%d frames" % (name, index + 1, len(frames)))
    xyz = np.concatenate([frame["xyz"] for frame in transformed])
    intensity = np.concatenate([frame["intensity"] for frame in transformed])
    local_z = np.concatenate([frame["local_z"] for frame in transformed])
    voxel_xyz, voxel_intensity, _, voxel_counts = voxel_downsample(
        xyz, intensity, map_voxel
    )
    ground, ground_cells = ground_thickness(xyz, local_z)
    plane, plane_cells = planar_cell_residual(voxel_xyz)
    overlap = cross_frame_overlap(transformed)
    pcd_path = "%s_%s.pcd" % (out_prefix, name)
    write_binary_pcd(pcd_path, voxel_xyz, voxel_intensity)
    return {
        "candidate": name,
        "pose_time_offset_s": float(offset),
        "input_points": int(len(xyz)),
        "map_voxels": int(len(voxel_xyz)),
        "map_voxels_per_1000_points": float(1000.0 * len(voxel_xyz) / len(xyz)),
        "mean_points_per_map_voxel": float(np.mean(voxel_counts)),
        "ground_cells": ground_cells,
        "ground_z_iqr_median_m": ground["median"],
        "ground_z_iqr_p95_m": ground["p95"],
        "planar_cells": plane_cells,
        "planar_residual_median_m": plane["median"],
        "planar_residual_p95_m": plane["p95"],
        "overlap_nearest_median_m": overlap["median"],
        "overlap_nearest_p95_m": overlap["p95"],
        "pcd": pcd_path,
    }


def rank_candidates(results):
    metric_names = [
        "map_voxels_per_1000_points",
        "ground_z_iqr_median_m",
        "planar_residual_median_m",
        "overlap_nearest_median_m",
    ]
    usable = [
        metric for metric in metric_names
        if all(result.get(metric) not in (None, 0.0) for result in results)
    ]
    for result in results:
        result["normalized_score"] = 0.0
        result["metric_wins"] = 0
    for metric in usable:
        best = min(result[metric] for result in results)
        winner = min(results, key=lambda result: result[metric])
        winner["metric_wins"] += 1
        for result in results:
            result["normalized_score"] += result[metric] / best
    ranked = sorted(results, key=lambda result: result["normalized_score"])
    if len(ranked) < 2 or not usable:
        return None, "insufficient_metrics", None, usable
    margin = (
        ranked[1]["normalized_score"] - ranked[0]["normalized_score"]
    ) / ranked[0]["normalized_score"]
    if margin >= 0.01 and ranked[0]["metric_wins"] >= 2:
        decision = "preferred"
        preferred = ranked[0]["candidate"]
    else:
        decision = "indistinguishable"
        preferred = None
    return preferred, decision, float(margin), usable


def write_results_csv(path, results):
    fields = list(results[0].keys())
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(results)


def fmt(value, digits=5):
    return "n/a" if value is None else ("%%.%df" % digits) % value


def main():
    args = parse_args()
    if args.duration <= 2.0 or args.point_stride <= 0 or args.map_voxel <= 0.0:
        raise ValueError("Invalid duration, point stride, or map voxel")

    print("Reading GT and calibration...")
    samples = read_ground_truth(args.gt)
    calibration = parse_calibration(args.calibration)
    lidar_start, lidar_end = read_frame_bounds(args.frames_audit)
    if args.start_utc is None:
        selection = choose_dynamic_window(
            samples, lidar_start, lidar_end, args.duration
        )
        _, start, median_speed, yaw_p75, moving_ratio = selection
        selection_mode = "automatic_dynamic_window"
    else:
        start = args.start_utc
        median_speed = yaw_p75 = moving_ratio = None
        selection_mode = "manual"
    end = start + args.duration
    if start < lidar_start or end > lidar_end:
        raise ValueError("Selected segment lies outside LiDAR header coverage")
    print("Selected UTC %.3f to %.3f (%s)" % (start, end, selection_mode))
    if median_speed is not None:
        print("  median speed=%.2f m/s, yaw-rate p75=%.2f deg/s, moving=%.1f%%" % (
            median_speed, yaw_p75, 100.0 * moving_ratio
        ))

    frames = read_segment_frames(
        args.bag, args.topic, start, end, args.point_stride,
        args.min_range, args.max_range,
    )
    print("Extracted %d frames, %d sampled points" % (
        len(frames), sum(len(frame["xyz"]) for frame in frames)
    ))
    pose = PoseInterpolator(samples)
    matrix = np.asarray(calibration["matrix"], dtype=np.float64)
    lag = calibration["time_lag_imu_to_lidar_s"]
    candidates = [
        ("minus_lag", -lag),
        ("raw", 0.0),
        ("plus_lag", lag),
    ]
    results = []
    for name, offset in candidates:
        print("Evaluating %s (pose offset %+0.6f s)..." % (name, offset))
        results.append(evaluate_candidate(
            name, offset, frames, pose, matrix, args.map_voxel, args.out_prefix
        ))
    preferred, decision, margin, metrics = rank_candidates(results)
    write_results_csv(args.out_csv, results)

    audit = {
        "schema_version": 1,
        "decision": decision,
        "preferred_candidate": preferred,
        "score_margin_fraction": margin,
        "ranking_metrics": metrics,
        "segment": {
            "selection_mode": selection_mode,
            "start_utc": start,
            "end_utc": end,
            "duration_s": args.duration,
            "frames": len(frames),
            "sampled_points": sum(len(frame["xyz"]) for frame in frames),
            "median_speed_m_s": median_speed,
            "yaw_rate_p75_deg_s": yaw_p75,
            "moving_ratio": moving_ratio,
        },
        "calibration_lag_imu_to_lidar_s": lag,
        "pose_convention": (
            "ENU_from_IMU_FLU = Rz(90deg-heading) Ry(-pitch) Rx(roll); "
            "refined LiDAR-to-IMU matrix applied first"
        ),
        "assumptions_and_limits": [
            "GT pose origin is temporarily treated as the Xsens IMU origin.",
            "The missing Xsens-to-GNSS/INS lever arm can affect sharp-turn map metrics.",
            "Dynamic objects and changing visibility affect all map-sharpness metrics.",
            "A preferred candidate is diagnostic evidence, not a calibrated-material result.",
        ],
        "results": results,
    }
    with open(args.audit, "w", encoding="utf-8") as handle:
        json.dump(audit, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write("\n")

    print("\n=== Short-segment map consistency ===")
    print("candidate,offset_s,vox/1000,ground_IQR,plane_resid,overlap_med,wins,score")
    for result in results:
        print("%s,%+.6f,%s,%s,%s,%s,%d,%s" % (
            result["candidate"], result["pose_time_offset_s"],
            fmt(result["map_voxels_per_1000_points"], 3),
            fmt(result["ground_z_iqr_median_m"]),
            fmt(result["planar_residual_median_m"]),
            fmt(result["overlap_nearest_median_m"]),
            result["metric_wins"], fmt(result["normalized_score"], 5),
        ))
    print("decision:             %s" % decision.upper())
    print("preferred candidate:  %s" % (preferred or "NONE"))
    print("score margin:         %s" % (
        "n/a" if margin is None else "%.2f%%" % (100.0 * margin)
    ))
    print("Results CSV: %s" % args.out_csv)
    print("Audit JSON: %s" % args.audit)
    for result in results:
        print("Map PCD: %s" % result["pcd"])
    print("Caution: an unknown Xsens-to-GT lever arm remains a documented limitation.")


if __name__ == "__main__":
    main()
