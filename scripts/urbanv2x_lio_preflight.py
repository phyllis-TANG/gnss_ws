#!/usr/bin/env python3
"""Audit whether the current ROS1 container is ready for UrbanV2X LIO.

The LiDAR metadata are reused from ``urbanv2x_lidar_gt_audit.py`` so only the
much smaller IMU connection is deserialized from the bag.  The script also
discovers installed ROS packages and their launch/config files; it does not
install, build, or run a mapping package.
"""

import argparse
import bisect
import csv
import importlib.util
import json
import math
import os
import statistics
import subprocess
from collections import Counter


LIO_PACKAGE_TOKENS = (
    "fast_lio", "fastlio", "point_lio", "pointlio", "lio_sam", "liosam",
    "lio_mapping", "lidar_inertial",
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Audit UrbanV2X LIO inputs and installed ROS packages"
    )
    parser.add_argument("--bag", required=True)
    parser.add_argument("--frames-audit", required=True)
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--workspace", default="/root/gnss_ws")
    parser.add_argument("--audit", required=True)
    return parser.parse_args()


def percentile(values, probability):
    values = sorted(float(value) for value in values)
    if not values:
        return None
    if len(values) == 1:
        return values[0]
    position = (len(values) - 1) * probability
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return values[lower]
    weight = position - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def describe(values):
    values = [float(value) for value in values if value is not None and math.isfinite(float(value))]
    if not values:
        return {"n": 0, "min": None, "p05": None, "median": None,
                "p95": None, "max": None}
    return {
        "n": len(values),
        "min": min(values),
        "p05": percentile(values, 0.05),
        "median": statistics.median(values),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def norm(values):
    return math.sqrt(sum(float(value) ** 2 for value in values))


def nearest_residual(sorted_times, epoch):
    index = bisect.bisect_left(sorted_times, epoch)
    candidates = []
    if index < len(sorted_times):
        candidates.append(sorted_times[index])
    if index > 0:
        candidates.append(sorted_times[index - 1])
    if not candidates:
        return None
    nearest = min(candidates, key=lambda value: abs(value - epoch))
    return nearest - epoch


def read_lidar_frame_audit(path):
    frames = []
    with open(path, "r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {
            "header_time", "bag_time", "frame_id", "width", "height",
            "point_time_min_s", "point_time_max_s",
        }
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError("Frame audit CSV missing columns: %s" % sorted(missing))
        for row in reader:
            frames.append({
                "header_time": float(row["header_time"]),
                "bag_time": float(row["bag_time"]),
                "frame_id": row["frame_id"],
                "points": int(row["width"]) * int(row["height"]),
                "point_time_min_s": (
                    float(row["point_time_min_s"]) if row["point_time_min_s"] else None
                ),
                "point_time_max_s": (
                    float(row["point_time_max_s"]) if row["point_time_max_s"] else None
                ),
            })
    if not frames:
        raise ValueError("Frame audit CSV contains no rows")
    frames.sort(key=lambda item: item["header_time"])
    return frames


def audit_lidar_frames(frames):
    header_times = [frame["header_time"] for frame in frames]
    intervals = [header_times[index] - header_times[index - 1]
                 for index in range(1, len(header_times))]
    sampled = [frame for frame in frames if frame["point_time_max_s"] is not None]
    return {
        "frames": len(frames),
        "start_header_utc": header_times[0],
        "end_header_utc": header_times[-1],
        "duration_s": header_times[-1] - header_times[0],
        "rate_hz": (
            1.0 / statistics.median(intervals) if intervals else None
        ),
        "interval_s": describe(intervals),
        "nonpositive_intervals": sum(value <= 0.0 for value in intervals),
        "frame_ids": dict(sorted(Counter(frame["frame_id"] for frame in frames).items())),
        "points_per_frame": describe(frame["points"] for frame in frames),
        "sampled_scan_duration_s": describe(
            frame["point_time_max_s"] - frame["point_time_min_s"]
            for frame in sampled
        ),
    }


def read_and_audit_imu(path, topic):
    try:
        import rosbag
    except ImportError as exc:
        raise RuntimeError("rosbag is required inside the ROS1 container") from exc

    header_times = []
    bag_delays = []
    frame_ids = Counter()
    quaternion_norms = []
    gyro_norms = []
    acceleration_norms = []
    nonfinite_messages = 0
    orientation_covariance_unknown = 0
    angular_covariance_unknown = 0
    acceleration_covariance_unknown = 0
    with rosbag.Bag(path, "r") as bag:
        for _, message, bag_stamp in bag.read_messages(topics=[topic]):
            header_time = message.header.stamp.to_sec()
            quaternion = (
                message.orientation.x, message.orientation.y,
                message.orientation.z, message.orientation.w,
            )
            gyro = (
                message.angular_velocity.x, message.angular_velocity.y,
                message.angular_velocity.z,
            )
            acceleration = (
                message.linear_acceleration.x, message.linear_acceleration.y,
                message.linear_acceleration.z,
            )
            values = (header_time,) + quaternion + gyro + acceleration
            if not all(math.isfinite(float(value)) for value in values):
                nonfinite_messages += 1
                continue
            header_times.append(header_time)
            bag_delays.append(bag_stamp.to_sec() - header_time)
            frame_ids[str(message.header.frame_id)] += 1
            quaternion_norms.append(norm(quaternion))
            gyro_norms.append(norm(gyro))
            acceleration_norms.append(norm(acceleration))
            orientation_covariance_unknown += int(
                len(message.orientation_covariance) > 0
                and message.orientation_covariance[0] < 0.0
            )
            angular_covariance_unknown += int(
                len(message.angular_velocity_covariance) > 0
                and message.angular_velocity_covariance[0] < 0.0
            )
            acceleration_covariance_unknown += int(
                len(message.linear_acceleration_covariance) > 0
                and message.linear_acceleration_covariance[0] < 0.0
            )
    if not header_times:
        raise ValueError("No finite IMU messages found on %s" % topic)
    intervals = [header_times[index] - header_times[index - 1]
                 for index in range(1, len(header_times))]
    duration = header_times[-1] - header_times[0]
    return {
        "messages": len(header_times),
        "nonfinite_messages": nonfinite_messages,
        "start_header_utc": header_times[0],
        "end_header_utc": header_times[-1],
        "duration_s": duration,
        "rate_hz": ((len(header_times) - 1) / duration if duration > 0.0 else None),
        "interval_s": describe(intervals),
        "nonpositive_intervals": sum(value <= 0.0 for value in intervals),
        "frame_ids": dict(sorted(frame_ids.items())),
        "bag_minus_header_s": describe(bag_delays),
        "quaternion_norm": describe(quaternion_norms),
        "gyro_norm_rad_s": describe(gyro_norms),
        "acceleration_norm_m_s2": describe(acceleration_norms),
        "orientation_covariance_unknown": orientation_covariance_unknown,
        "angular_velocity_covariance_unknown": angular_covariance_unknown,
        "linear_acceleration_covariance_unknown": acceleration_covariance_unknown,
        "header_times": header_times,
    }


def cross_sensor_timing(frames, imu_times):
    header_residuals = []
    end_residuals = []
    end_count = 0
    for frame in frames:
        residual = nearest_residual(imu_times, frame["header_time"])
        if residual is not None:
            header_residuals.append(abs(residual))
        if frame["point_time_max_s"] is not None:
            end_epoch = frame["header_time"] + frame["point_time_max_s"]
            residual = nearest_residual(imu_times, end_epoch)
            if residual is not None:
                end_residuals.append(abs(residual))
                end_count += 1
    return {
        "lidar_header_to_nearest_imu_abs_s": describe(header_residuals),
        "sampled_scan_end_to_nearest_imu_abs_s": describe(end_residuals),
        "sampled_scan_ends": end_count,
        "imu_covers_all_lidar_headers": (
            imu_times[0] <= frames[0]["header_time"]
            and imu_times[-1] >= frames[-1]["header_time"]
        ),
    }


def normalize_package_name(name):
    return name.lower().replace("-", "_")


def is_lio_candidate(name, path):
    text = normalize_package_name(name + " " + path)
    return any(token in text for token in LIO_PACKAGE_TOKENS)


def discover_ros_packages(workspace):
    packages = []
    error = None
    try:
        result = subprocess.run(
            ["rospack", "list"], check=False, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
        if result.returncode != 0:
            error = result.stderr.strip() or "rospack list failed"
        else:
            for line in result.stdout.splitlines():
                parts = line.split(None, 1)
                if len(parts) == 2 and is_lio_candidate(parts[0], parts[1]):
                    packages.append({"name": parts[0], "path": parts[1], "source": "rospack"})
    except OSError as exc:
        error = str(exc)

    source_root = os.path.join(workspace, "src")
    if os.path.isdir(source_root):
        for root, directories, files in os.walk(source_root):
            directories[:] = [item for item in directories if not item.startswith(".")]
            if "package.xml" not in files:
                continue
            basename = os.path.basename(root)
            if is_lio_candidate(basename, root):
                packages.append({"name": basename, "path": root, "source": "filesystem"})
            directories[:] = []

    unique = {}
    for package in packages:
        key = os.path.realpath(package["path"])
        if key not in unique:
            unique[key] = package
        elif package["source"] == "rospack":
            unique[key] = package
    discovered = sorted(unique.values(), key=lambda item: (item["name"], item["path"]))
    for package in discovered:
        launch_files = []
        config_files = []
        for root, _, files in os.walk(package["path"]):
            for filename in files:
                full_path = os.path.join(root, filename)
                lower = filename.lower()
                if lower.endswith(".launch"):
                    launch_files.append(full_path)
                elif lower.endswith((".yaml", ".yml")):
                    config_files.append(full_path)
        package["launch_files"] = sorted(launch_files)[:100]
        package["config_files"] = sorted(config_files)[:100]
        package["built_library_directory"] = os.path.isdir(
            os.path.join(workspace, "devel", "lib", package["name"])
        )
    return discovered, error


def dependency_audit():
    modules = {}
    for name in ("numpy", "scipy", "rosbag", "rospy"):
        spec = importlib.util.find_spec(name)
        modules[name] = spec is not None
    commands = {}
    for command in (["rosversion", "-d"], ["pcl_version"]):
        try:
            result = subprocess.run(
                command, check=False, text=True,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            )
            commands[" ".join(command)] = {
                "available": result.returncode == 0,
                "output": (result.stdout or result.stderr).strip(),
            }
        except OSError as exc:
            commands[" ".join(command)] = {"available": False, "output": str(exc)}
    return {"python_modules": modules, "commands": commands}


def decide(lidar, imu, timing, packages, dependencies):
    blockers = []
    warnings = []
    if lidar["nonpositive_intervals"]:
        blockers.append("lidar_nonmonotonic_time")
    if imu["nonpositive_intervals"]:
        blockers.append("imu_nonmonotonic_time")
    if imu["nonfinite_messages"]:
        blockers.append("imu_nonfinite_values")
    if not timing["imu_covers_all_lidar_headers"]:
        blockers.append("imu_does_not_cover_lidar")
    if not 300.0 <= imu["rate_hz"] <= 500.0:
        blockers.append("unexpected_imu_rate")
    acceleration = imu["acceleration_norm_m_s2"]["median"]
    if acceleration is None or not 7.0 <= acceleration <= 12.0:
        blockers.append("unexpected_acceleration_norm")
    quaternion = imu["quaternion_norm"]["median"]
    if quaternion is not None and not (quaternion < 0.05 or 0.95 <= quaternion <= 1.05):
        warnings.append("unexpected_orientation_quaternion_norm")
    if timing["lidar_header_to_nearest_imu_abs_s"]["p95"] > 0.005:
        blockers.append("lidar_imu_nearest_time_residual")
    if not dependencies["python_modules"].get("scipy"):
        warnings.append("scipy_missing")
    if blockers:
        return "REVIEW_INPUTS", blockers, warnings
    if not packages:
        return "INPUTS_READY_NO_LIO_PACKAGE", blockers, warnings
    if not any(package["launch_files"] for package in packages):
        return "LIO_SOURCE_FOUND_NO_LAUNCH", blockers, warnings
    return "READY_FOR_CONFIG_AUDIT", blockers, warnings


def fmt(value, digits=6):
    return "n/a" if value is None else ("%%.%df" % digits) % value


def main():
    args = parse_args()
    print("Reading LiDAR frame audit...")
    frames = read_lidar_frame_audit(args.frames_audit)
    lidar = audit_lidar_frames(frames)
    print("Reading IMU messages...")
    imu = read_and_audit_imu(args.bag, args.imu_topic)
    imu_times = imu.pop("header_times")
    timing = cross_sensor_timing(frames, imu_times)
    print("Discovering ROS LIO packages...")
    packages, rospack_error = discover_ros_packages(args.workspace)
    dependencies = dependency_audit()
    decision, blockers, warnings = decide(
        lidar, imu, timing, packages, dependencies
    )
    audit = {
        "schema_version": 1,
        "decision": decision,
        "blockers": blockers,
        "warnings": warnings,
        "inputs": {
            "bag": args.bag,
            "frames_audit": args.frames_audit,
            "imu_topic": args.imu_topic,
            "workspace": args.workspace,
        },
        "lidar": lidar,
        "imu": imu,
        "cross_sensor_timing": timing,
        "lio_packages": packages,
        "rospack_error": rospack_error,
        "dependencies": dependencies,
    }
    with open(args.audit, "w", encoding="utf-8") as handle:
        json.dump(audit, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write("\n")

    print("\n=== UrbanV2X LIO preflight ===")
    print("LiDAR frames/rate:          %d / %s Hz" % (
        lidar["frames"], fmt(lidar["rate_hz"], 3)))
    print("LiDAR frame IDs:            %s" % lidar["frame_ids"])
    print("IMU messages/rate:          %d / %s Hz" % (
        imu["messages"], fmt(imu["rate_hz"], 3)))
    print("IMU frame IDs:              %s" % imu["frame_ids"])
    print("IMU interval p95/max:       %s / %s s" % (
        fmt(imu["interval_s"]["p95"]), fmt(imu["interval_s"]["max"])))
    print("IMU bag-header med/p95:     %s / %s s" % (
        fmt(imu["bag_minus_header_s"]["median"]),
        fmt(imu["bag_minus_header_s"]["p95"])))
    print("acc norm med/p05/p95:       %s / %s / %s m/s^2" % (
        fmt(imu["acceleration_norm_m_s2"]["median"], 4),
        fmt(imu["acceleration_norm_m_s2"]["p05"], 4),
        fmt(imu["acceleration_norm_m_s2"]["p95"], 4)))
    print("gyro norm med/p95/max:      %s / %s / %s rad/s" % (
        fmt(imu["gyro_norm_rad_s"]["median"], 4),
        fmt(imu["gyro_norm_rad_s"]["p95"], 4),
        fmt(imu["gyro_norm_rad_s"]["max"], 4)))
    print("quaternion norm med/p95:    %s / %s" % (
        fmt(imu["quaternion_norm"]["median"], 6),
        fmt(imu["quaternion_norm"]["p95"], 6)))
    print("LiDAR->nearest IMU p95/max: %s / %s s" % (
        fmt(timing["lidar_header_to_nearest_imu_abs_s"]["p95"]),
        fmt(timing["lidar_header_to_nearest_imu_abs_s"]["max"])))
    print("IMU covers LiDAR:           %s" % timing["imu_covers_all_lidar_headers"])
    print("Python modules:             %s" % dependencies["python_modules"])
    print("\nLIO packages found:         %d" % len(packages))
    for package in packages:
        print("  %s [%s] built=%s" % (
            package["name"], package["path"], package["built_library_directory"]
        ))
        for path in package["launch_files"][:10]:
            print("    launch: %s" % path)
        for path in package["config_files"][:10]:
            print("    config: %s" % path)
    print("\ndecision: %s" % decision)
    print("blockers: %s" % (blockers or "NONE"))
    print("warnings: %s" % (warnings or "NONE"))
    print("Audit JSON: %s" % args.audit)


if __name__ == "__main__":
    main()
