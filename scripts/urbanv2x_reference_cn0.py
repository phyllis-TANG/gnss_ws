#!/usr/bin/env python3
"""Build audited vehicle-to-roadside C/N0 differences for UrbanV2X.

The UrbanV2X small-loop bag reports Galileo E5b observations with a
1227.60 MHz value in ``GnssObsMsg.freqs``.  That value is the GPS L2 carrier
frequency, not Galileo E5b.  The accompanying RTKLIB observation code is
still correct (code 28, L7Q), so signal identity must be derived from the
satellite system and observation code rather than frequency alone.

This script does not modify the source bag.  It:

* decodes vehicle signals using (constellation, RTKLIB code);
* parses signal-strength observations from the WEST and optional EAST RINEX;
* performs nearest-time matching per satellite and signal;
* writes raw reference-minus-vehicle C/N0 differences;
* writes channel-median-centred differences as diagnostic quantities; and
* records the Galileo frequency correction and complete audit statistics.

The centred differences are not absolute physical attenuation labels.  A
physical receiver/channel bias requires trusted LOS observations (and should
normally be estimated after satellite elevation and obstruction geometry are
available).
"""

import argparse
import bisect
import csv
import datetime as dt
import json
import math
import os
import statistics
from collections import Counter, defaultdict


GPS_EPOCH_UNIX = 315964800.0

# RTKLIB/gnss_comm observation codes found in the UrbanV2X vehicle bag.
# The roadside RINEX records combined tracking attributes (X) for several
# signals, while the u-blox message exposes a component tracking code.
SIGNAL_BY_SYSTEM_CODE = {
    ("G", 1): "S1C",
    ("G", 17): "S2X",
    ("R", 1): "S1C",
    ("R", 14): "S2C",
    ("E", 1): "S1X",
    ("E", 28): "S7X",  # L7Q (Galileo E5b Q) matched to RINEX S7X
    ("C", 47): "S2I",
    ("C", 27): "S7I",
}

PHYSICAL_FREQUENCY_MHZ = {
    ("G", "S1C"): 1575.420,
    ("G", "S2X"): 1227.600,
    ("E", "S1X"): 1575.420,
    ("E", "S7X"): 1207.140,
    ("C", "S2I"): 1561.098,
    ("C", "S7I"): 1207.140,
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Audit and repair UrbanV2X vehicle/roadside C/N0 matching"
    )
    parser.add_argument("--bag", required=True, help="UrbanV2X ROS1 bag")
    parser.add_argument("--west", required=True, help="WEST roadside RINEX obs")
    parser.add_argument("--east", help="Optional EAST_HIGH roadside RINEX obs")
    parser.add_argument(
        "--topic", default="/ublox_driver/range_meas", help="Vehicle GNSS topic"
    )
    parser.add_argument("--out", required=True, help="Output matched CSV")
    parser.add_argument("--audit", required=True, help="Output audit JSON")
    parser.add_argument(
        "--tolerance",
        type=float,
        default=0.20,
        help="Nearest reference epoch tolerance in seconds (default: 0.20)",
    )
    parser.add_argument(
        "--leap-seconds",
        type=float,
        default=18.0,
        help="GPS-UTC leap seconds for the dataset epoch (default: 18)",
    )
    return parser.parse_args()


def sat_system_prn(sat):
    """Decode the gnss_comm continuous satellite number."""
    sat = int(sat)
    if 1 <= sat <= 32:
        return "G", sat
    if 33 <= sat <= 59:
        return "R", sat - 32
    if 60 <= sat <= 97:
        return "E", sat - 59
    if sat >= 98:
        return "C", sat - 97
    return None, None


def satellite_id(sat):
    system, prn = sat_system_prn(sat)
    if system is None:
        return None
    return "%s%02d" % (system, prn)


def finite_number(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def percentile(values, probability):
    values = sorted(float(value) for value in values if value is not None)
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
    values = [float(value) for value in values if value is not None]
    if not values:
        return {"n": 0, "median": None, "p05": None, "p95": None}
    return {
        "n": len(values),
        "median": statistics.median(values),
        "p05": percentile(values, 0.05),
        "p95": percentile(values, 0.95),
    }


def gps_week_tow_to_unix_utc(week, tow, leap_seconds):
    return GPS_EPOCH_UNIX + int(week) * 604800.0 + float(tow) - leap_seconds


def gps_calendar_to_unix_utc(parts, leap_seconds):
    year, month, day, hour, minute = (int(parts[index]) for index in range(5))
    second = float(parts[5])
    whole_second = int(math.floor(second))
    microsecond = int(round((second - whole_second) * 1e6))
    if microsecond == 1000000:
        whole_second += 1
        microsecond = 0
    timestamp = dt.datetime(
        year,
        month,
        day,
        hour,
        minute,
        whole_second,
        microsecond,
        tzinfo=dt.timezone.utc,
    ).timestamp()
    return timestamp - leap_seconds


def parse_rinex_observation_types(handle):
    observation_types = {}
    time_system = None
    current_system = None

    for raw_line in handle:
        line = raw_line.rstrip("\r\n")
        padded = line.ljust(80)
        label = padded[60:80].strip()

        if label == "SYS / # / OBS TYPES":
            system = padded[0].strip() or current_system
            if not system:
                raise ValueError("RINEX observation-type continuation has no system")
            current_system = system
            if system not in observation_types:
                try:
                    expected = int(padded[3:6])
                except ValueError:
                    expected = None
                observation_types[system] = {"expected": expected, "types": []}
            observation_types[system]["types"].extend(padded[7:60].split())

        elif label == "TIME OF FIRST OBS":
            tokens = padded[:60].split()
            if tokens:
                candidate = tokens[-1]
                if candidate.isalpha():
                    time_system = candidate.upper()

        elif label == "END OF HEADER":
            break
    else:
        raise ValueError("RINEX END OF HEADER not found")

    result = {}
    for system, entry in observation_types.items():
        types = entry["types"]
        expected = entry["expected"]
        if expected is not None:
            if len(types) < expected:
                raise ValueError(
                    "RINEX %s declares %d observation types but %d were parsed"
                    % (system, expected, len(types))
                )
            types = types[:expected]
        result[system] = types
    return result, time_system


def parse_rinex_cn0(path, leap_seconds):
    """Return signal-strength records and parsing metadata from RINEX 3."""
    records = []
    duplicate_counter = Counter()
    seen = set()

    with open(path, "r", encoding="ascii", errors="replace", newline=None) as handle:
        observation_types, time_system = parse_rinex_observation_types(handle)
        if time_system not in (None, "GPS"):
            raise ValueError(
                "%s uses unsupported RINEX time system %s" % (path, time_system)
            )

        while True:
            raw_line = handle.readline()
            if not raw_line:
                break
            line = raw_line.rstrip("\r\n")
            if not line.startswith(">"):
                continue

            parts = line[1:].split()
            if len(parts) < 8:
                continue
            flag = int(parts[6])
            count = int(parts[7])
            epoch_utc = gps_calendar_to_unix_utc(parts, leap_seconds)

            if flag not in (0, 1):
                for _ in range(count):
                    handle.readline()
                continue

            for _ in range(count):
                observation_line = handle.readline()
                if not observation_line:
                    raise ValueError("Unexpected end of RINEX observation block")
                observation_line = observation_line.rstrip("\r\n")
                sat_id = observation_line[:3].strip()
                if len(sat_id) < 2:
                    continue
                system = sat_id[0]
                types = observation_types.get(system, [])
                payload = observation_line[3:]
                required = 16 * len(types)

                while len(payload) < required:
                    continuation = handle.readline()
                    if not continuation:
                        raise ValueError("Unexpected end of RINEX continuation")
                    continuation = continuation.rstrip("\r\n")
                    payload += continuation[3:] if len(continuation) >= 3 else ""

                for index, observation_type in enumerate(types):
                    if not observation_type.startswith("S"):
                        continue
                    field = payload[index * 16 : (index + 1) * 16]
                    value = finite_number(field[:14].strip())
                    if value is None:
                        continue
                    key = (round(epoch_utc, 6), sat_id, observation_type)
                    if key in seen:
                        duplicate_counter[(system, observation_type)] += 1
                        continue
                    seen.add(key)
                    records.append(
                        {
                            "epoch_utc": epoch_utc,
                            "sat_id": sat_id,
                            "sys": system,
                            "signal": observation_type,
                            "cn0": value,
                        }
                    )

    return records, {
        "path": os.path.abspath(path),
        "time_system": time_system or "unspecified_assumed_GPS",
        "records": len(records),
        "duplicates": sum(duplicate_counter.values()),
        "observation_types": observation_types,
    }


def read_vehicle_bag(path, topic, leap_seconds):
    try:
        import rosbag
    except ImportError as exc:
        raise RuntimeError(
            "rosbag is required; source /opt/ros/noetic/setup.bash and the "
            "workspace devel/setup.bash before running"
        ) from exc

    rows = []
    message_epochs = 0
    satellite_observations = 0
    input_signal_records = 0
    array_length_errors = 0
    duplicate_counter = Counter()
    unmapped_counter = Counter()
    corrected_frequency_counter = Counter()
    seen = set()

    with rosbag.Bag(path, "r") as bag:
        for _, message, _ in bag.read_messages(topics=[topic]):
            message_epochs += 1
            for observation in message.meas:
                satellite_observations += 1
                lengths = {
                    len(observation.freqs),
                    len(observation.CN0),
                    len(observation.code),
                    len(observation.status),
                }
                if len(lengths) != 1:
                    array_length_errors += 1
                    continue

                system, prn = sat_system_prn(observation.sat)
                sat_id = satellite_id(observation.sat)
                epoch_utc = gps_week_tow_to_unix_utc(
                    observation.time.week, observation.time.tow, leap_seconds
                )

                for frequency, cn0, code, status in zip(
                    observation.freqs,
                    observation.CN0,
                    observation.code,
                    observation.status,
                ):
                    input_signal_records += 1
                    reported_mhz = float(frequency) / 1e6
                    code = int(code)
                    status = int(status)
                    signal = SIGNAL_BY_SYSTEM_CODE.get((system, code))
                    if signal is None:
                        unmapped_counter[
                            (system or "?", code, round(reported_mhz, 3))
                        ] += 1
                        continue

                    key = (round(epoch_utc, 6), sat_id, signal)
                    if key in seen:
                        duplicate_counter[(system, signal)] += 1
                        continue
                    seen.add(key)

                    physical_mhz = PHYSICAL_FREQUENCY_MHZ.get((system, signal))
                    # GLONASS is FDMA, so the physical carrier depends on the
                    # satellite slot and the reported per-satellite frequency
                    # is the value that should be preserved.
                    if system == "R" and physical_mhz is None:
                        physical_mhz = reported_mhz
                    frequency_corrected = (
                        physical_mhz is not None
                        and abs(reported_mhz - physical_mhz) > 0.001
                    )
                    if frequency_corrected:
                        corrected_frequency_counter[
                            (system, signal, round(reported_mhz, 3), physical_mhz)
                        ] += 1

                    rows.append(
                        {
                            "epoch_utc": epoch_utc,
                            "gps_week": int(observation.time.week),
                            "gps_tow": float(observation.time.tow),
                            "sat_raw": int(observation.sat),
                            "sat_id": sat_id,
                            "sys": system,
                            "prn": prn,
                            "signal": signal,
                            "code": code,
                            "status": status,
                            "reported_frequency_mhz": reported_mhz,
                            "physical_frequency_mhz": physical_mhz,
                            "frequency_metadata_corrected": frequency_corrected,
                            "vehicle_cn0_dbhz": float(cn0),
                        }
                    )

    metadata = {
        "path": os.path.abspath(path),
        "topic": topic,
        "message_epochs": message_epochs,
        "satellite_epoch_observations": satellite_observations,
        "input_signal_records": input_signal_records,
        "signal_records": len(rows),
        "mapping_ratio": (
            len(rows) / input_signal_records if input_signal_records else None
        ),
        "array_length_errors": array_length_errors,
        "duplicates": sum(duplicate_counter.values()),
        "unmapped_records": sum(unmapped_counter.values()),
        "unmapped_combinations": [
            {
                "sys": key[0],
                "code": key[1],
                "reported_frequency_mhz": key[2],
                "n": count,
            }
            for key, count in sorted(unmapped_counter.items())
        ],
        "frequency_metadata_corrections": [
            {
                "sys": key[0],
                "signal": key[1],
                "reported_frequency_mhz": key[2],
                "physical_frequency_mhz": key[3],
                "n": count,
            }
            for key, count in sorted(corrected_frequency_counter.items())
        ],
    }
    return rows, metadata


class ReferenceIndex:
    def __init__(self, records):
        grouped = defaultdict(list)
        for record in records:
            grouped[(record["sat_id"], record["signal"])].append(
                (record["epoch_utc"], record["cn0"])
            )
        self.series = {}
        for key, samples in grouped.items():
            samples.sort()
            self.series[key] = (
                [sample[0] for sample in samples],
                [sample[1] for sample in samples],
            )

    def nearest(self, epoch_utc, sat_id, signal, tolerance):
        series = self.series.get((sat_id, signal))
        if series is None:
            return None, None
        times, values = series
        insertion = bisect.bisect_left(times, epoch_utc)
        candidates = []
        if insertion < len(times):
            candidates.append(insertion)
        if insertion > 0:
            candidates.append(insertion - 1)
        if not candidates:
            return None, None
        best = min(candidates, key=lambda index: abs(times[index] - epoch_utc))
        delta = times[best] - epoch_utc
        if abs(delta) > tolerance:
            return None, None
        return values[best], delta


def add_reference_matches(rows, label, reference_index, tolerance):
    cn0_column = "%s_cn0_dbhz" % label
    dt_column = "%s_dt_s" % label
    drop_column = "%s_drop_raw_db" % label
    for row in rows:
        if reference_index is None:
            row[cn0_column] = None
            row[dt_column] = None
            row[drop_column] = None
            continue
        cn0, delta = reference_index.nearest(
            row["epoch_utc"], row["sat_id"], row["signal"], tolerance
        )
        row[cn0_column] = cn0
        row[dt_column] = delta
        row[drop_column] = (
            None if cn0 is None else cn0 - row["vehicle_cn0_dbhz"]
        )


def add_channel_centered_differences(rows, label):
    raw_column = "%s_drop_raw_db" % label
    offset_column = "%s_channel_median_db" % label
    centered_column = "%s_drop_channel_centered_db" % label
    values = defaultdict(list)
    for row in rows:
        if row[raw_column] is not None:
            values[(row["sys"], row["signal"])].append(row[raw_column])
    medians = {key: statistics.median(group) for key, group in values.items()}
    for row in rows:
        offset = medians.get((row["sys"], row["signal"]))
        row[offset_column] = offset
        row[centered_column] = (
            None if row[raw_column] is None else row[raw_column] - offset
        )
    return medians


def channel_audit(rows, label):
    result = []
    cn0_column = "%s_cn0_dbhz" % label
    dt_column = "%s_dt_s" % label
    drop_column = "%s_drop_raw_db" % label
    groups = defaultdict(list)
    for row in rows:
        groups[(row["sys"], row["signal"])].append(row)

    for key in sorted(groups):
        group = groups[key]
        matched = [row for row in group if row[cn0_column] is not None]
        result.append(
            {
                "sys": key[0],
                "signal": key[1],
                "vehicle_n": len(group),
                "matched_n": len(matched),
                "match_ratio": len(matched) / len(group) if group else None,
                "vehicle_cn0": describe(
                    [row["vehicle_cn0_dbhz"] for row in group]
                ),
                "reference_cn0": describe([row[cn0_column] for row in matched]),
                "raw_drop": describe([row[drop_column] for row in matched]),
                "time_delta_s": describe([row[dt_column] for row in matched]),
            }
        )
    return result


def dual_reference_audit(rows):
    groups = defaultdict(list)
    all_differences = []
    for row in rows:
        west = row.get("west_cn0_dbhz")
        east = row.get("east_cn0_dbhz")
        if west is None or east is None:
            row["west_minus_east_cn0_db"] = None
            continue
        difference = west - east
        row["west_minus_east_cn0_db"] = difference
        all_differences.append(difference)
        groups[(row["sys"], row["signal"])].append(difference)
    return {
        "overall": describe(all_differences),
        "by_channel": [
            {"sys": key[0], "signal": key[1], **describe(values)}
            for key, values in sorted(groups.items())
        ],
    }


def write_csv(path, rows):
    fields = [
        "epoch_utc",
        "gps_week",
        "gps_tow",
        "sat_raw",
        "sat_id",
        "sys",
        "prn",
        "signal",
        "code",
        "status",
        "reported_frequency_mhz",
        "physical_frequency_mhz",
        "frequency_metadata_corrected",
        "vehicle_cn0_dbhz",
        "west_cn0_dbhz",
        "west_dt_s",
        "west_drop_raw_db",
        "west_channel_median_db",
        "west_drop_channel_centered_db",
        "east_cn0_dbhz",
        "east_dt_s",
        "east_drop_raw_db",
        "east_channel_median_db",
        "east_drop_channel_centered_db",
        "west_minus_east_cn0_db",
    ]
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            output = {}
            for field in fields:
                value = row.get(field)
                if isinstance(value, float):
                    output[field] = "%.6f" % value
                elif isinstance(value, bool):
                    output[field] = int(value)
                elif value is None:
                    output[field] = ""
                else:
                    output[field] = value
            writer.writerow(output)


def print_channel_table(title, entries):
    print("\n" + title)
    print(
        "sys signal vehicle matched match% median_vehicle median_ref "
        "median_raw_drop"
    )
    for entry in entries:
        print(
            "%3s %6s %7d %7d %6.1f %14.3f %10.3f %+15.3f"
            % (
                entry["sys"],
                entry["signal"],
                entry["vehicle_n"],
                entry["matched_n"],
                100.0 * entry["match_ratio"],
                entry["vehicle_cn0"]["median"],
                entry["reference_cn0"]["median"]
                if entry["reference_cn0"]["median"] is not None
                else float("nan"),
                entry["raw_drop"]["median"]
                if entry["raw_drop"]["median"] is not None
                else float("nan"),
            )
        )


def main():
    args = parse_args()
    if args.tolerance <= 0:
        raise ValueError("--tolerance must be positive")

    print("Reading vehicle GNSS bag...")
    rows, vehicle_metadata = read_vehicle_bag(
        args.bag, args.topic, args.leap_seconds
    )

    print("Reading WEST RINEX...")
    west_records, west_metadata = parse_rinex_cn0(args.west, args.leap_seconds)
    west_index = ReferenceIndex(west_records)

    east_records = []
    east_metadata = None
    east_index = None
    if args.east:
        print("Reading EAST_HIGH RINEX...")
        east_records, east_metadata = parse_rinex_cn0(
            args.east, args.leap_seconds
        )
        east_index = ReferenceIndex(east_records)

    print("Matching vehicle records to references...")
    add_reference_matches(rows, "west", west_index, args.tolerance)
    add_reference_matches(rows, "east", east_index, args.tolerance)
    west_channel_medians = add_channel_centered_differences(rows, "west")
    east_channel_medians = add_channel_centered_differences(rows, "east")
    dual_audit = dual_reference_audit(rows)

    west_audit = channel_audit(rows, "west")
    east_audit = channel_audit(rows, "east")
    audit = {
        "schema_version": 1,
        "notes": {
            "raw_drop_definition": "reference_cn0 - vehicle_cn0",
            "centered_drop_definition": (
                "raw_drop - median(raw_drop within constellation and signal)"
            ),
            "centered_drop_warning": (
                "Diagnostic only; not an absolute attenuation label or a "
                "trusted LOS receiver-bias calibration"
            ),
            "galileo_repair": (
                "E/code28 is L7Q (E5b) and maps to RINEX S7X even when the "
                "bag reports 1227.60 MHz"
            ),
        },
        "settings": {
            "time_tolerance_s": args.tolerance,
            "leap_seconds": args.leap_seconds,
        },
        "vehicle": vehicle_metadata,
        "west_rinex": west_metadata,
        "east_rinex": east_metadata,
        "west_by_channel": west_audit,
        "east_by_channel": east_audit if east_index is not None else [],
        "west_channel_medians_db": [
            {"sys": key[0], "signal": key[1], "median": value}
            for key, value in sorted(west_channel_medians.items())
        ],
        "east_channel_medians_db": [
            {"sys": key[0], "signal": key[1], "median": value}
            for key, value in sorted(east_channel_medians.items())
        ],
        "dual_reference": dual_audit,
    }

    write_csv(args.out, rows)
    audit_directory = os.path.dirname(os.path.abspath(args.audit))
    if audit_directory:
        os.makedirs(audit_directory, exist_ok=True)
    with open(args.audit, "w", encoding="utf-8") as handle:
        json.dump(audit, handle, ensure_ascii=False, indent=2, allow_nan=False)

    corrected_count = sum(
        entry["n"]
        for entry in vehicle_metadata["frequency_metadata_corrections"]
    )
    print("\nVehicle signal audit")
    print("  message epochs:           %d" % vehicle_metadata["message_epochs"])
    print(
        "  input signal records:     %d"
        % vehicle_metadata["input_signal_records"]
    )
    print("  signal records:           %d" % vehicle_metadata["signal_records"])
    print("  mapping ratio:            %.1f%%" % (
        100.0 * vehicle_metadata["mapping_ratio"]
    ))
    print("  unmapped records:         %d" % vehicle_metadata["unmapped_records"])
    print("  array length errors:      %d" % vehicle_metadata["array_length_errors"])
    print("  duplicate records:        %d" % vehicle_metadata["duplicates"])
    print("  corrected freq metadata:  %d" % corrected_count)

    print_channel_table("WEST minus VEHICLE", west_audit)
    if east_index is not None:
        print_channel_table("EAST_HIGH minus VEHICLE", east_audit)
        print("\nDual-reference overlap")
        print("  records: %d" % dual_audit["overall"]["n"])
        if dual_audit["overall"]["n"]:
            print(
                "  WEST minus EAST median: %+.3f dB"
                % dual_audit["overall"]["median"]
            )
            print(
                "  WEST minus EAST p05/p95: [%+.3f, %+.3f] dB"
                % (
                    dual_audit["overall"]["p05"],
                    dual_audit["overall"]["p95"],
                )
            )
        else:
            print("  no records matched to both references")

    print("\nOutput CSV:  %s" % args.out)
    print("Audit JSON:  %s" % args.audit)
    print(
        "Note: channel-centred drops are diagnostic only; estimate a physical "
        "bias from trusted LOS samples later."
    )


if __name__ == "__main__":
    main()
