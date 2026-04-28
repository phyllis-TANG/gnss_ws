"""Offline tool: extract NAV-PVT records from a .ubx capture into CSV.

Usable both as a console script (installed by setup.py as ``ubx_to_csv``) and
as a standalone Python module (``python3 -m ubx_parser.ubx_to_csv``).
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

from ubx_parser.ubx_protocol import (
    CLS_NAV,
    ID_NAV_PVT,
    decode_nav_pvt,
    iter_frames,
)

CSV_COLUMNS = [
    'iTOW_ms', 'utc_year', 'utc_month', 'utc_day',
    'utc_hour', 'utc_min', 'utc_sec', 'nano_ns',
    'fix_type', 'num_sv',
    'lat_deg', 'lon_deg', 'h_msl_m', 'height_m',
    'h_acc_m', 'v_acc_m',
    'vel_n_mps', 'vel_e_mps', 'vel_d_mps', 'g_speed_mps',
    'head_mot_deg', 'p_dop',
]


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('input', type=Path, help='Path to a .ubx capture file')
    parser.add_argument(
        '-o', '--output', type=Path, default=None,
        help='CSV output path (default: stdout)')
    return parser.parse_args(argv)


def export(input_path: Path, output_path: Path | None) -> int:
    data = input_path.read_bytes()
    out_stream = open(output_path, 'w', newline='') if output_path else sys.stdout
    try:
        writer = csv.writer(out_stream)
        writer.writerow(CSV_COLUMNS)

        count = 0
        for frame in iter_frames(data):
            if frame.key != (CLS_NAV, ID_NAV_PVT):
                continue
            pvt = decode_nav_pvt(frame.payload)
            if pvt is None:
                continue
            writer.writerow([
                pvt.i_tow_ms, pvt.year, pvt.month, pvt.day,
                pvt.hour, pvt.minute, pvt.second, pvt.nano_ns,
                pvt.fix_type, pvt.num_sv,
                f'{pvt.lat_deg:.7f}', f'{pvt.lon_deg:.7f}',
                f'{pvt.h_msl_m:.3f}', f'{pvt.height_m:.3f}',
                f'{pvt.h_acc_m:.3f}', f'{pvt.v_acc_m:.3f}',
                f'{pvt.vel_n_mps:.3f}', f'{pvt.vel_e_mps:.3f}',
                f'{pvt.vel_d_mps:.3f}', f'{pvt.g_speed_mps:.3f}',
                f'{pvt.head_mot_deg:.5f}', f'{pvt.p_dop:.2f}',
            ])
            count += 1
        return count
    finally:
        if output_path:
            out_stream.close()


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if not args.input.is_file():
        print(f"error: input file not found: {args.input}", file=sys.stderr)
        return 1
    count = export(args.input, args.output)
    print(f"wrote {count} NAV-PVT rows", file=sys.stderr)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
