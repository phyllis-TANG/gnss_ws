# gnss_ws

ROS 2 Humble workspace for working with u-blox GNSS receivers.

## Layout

```
gnss_ws/
├── 2026-4-15_174245_serial-COM5(1).ubx   # sample u-blox capture (~5 MB)
├── ubx.repos                              # external dependencies (vcstool)
└── src/
    └── ubx_parser/                        # in-tree package
        ├── ubx_parser/
        │   ├── ubx_protocol.py            # pure-Python UBX framing + NAV-PVT decoder
        │   ├── replay_node.py             # ROS 2 node: replays .ubx as NavSatFix/TwistStamped
        │   └── ubx_to_csv.py              # offline tool: NAV-PVT → CSV
        ├── launch/replay.launch.py
        ├── config/replay.yaml
        └── test/test_ubx_protocol.py
```

## Setup

```bash
# Source ROS 2 Humble first
source /opt/ros/humble/setup.bash

# Pull the external u-blox driver into src/ (optional, for live hardware)
sudo apt install python3-vcstool
vcs import src < ubx.repos

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash
```

## Replay a .ubx capture

The `ubx_parser` package ships a node that decodes NAV-PVT records from a
captured `.ubx` file and republishes them as live ROS topics, honouring the
original inter-message timing (derived from the iTOW field).

```bash
# Default 1× real-time playback
ros2 launch ubx_parser replay.launch.py \
    ubx_file:=$PWD/2026-4-15_174245_serial-COM5\(1\).ubx

# In a second terminal:
ros2 topic echo /fix
ros2 topic hz /fix
```

Topics published:

| Topic  | Type                          | Notes                                                       |
|--------|-------------------------------|-------------------------------------------------------------|
| `/fix` | `sensor_msgs/NavSatFix`       | Lat/Lon/Alt + diagonal covariance from `hAcc`/`vAcc`.       |
| `/vel` | `geometry_msgs/TwistStamped`  | NED → ENU translated linear velocity.                       |

Parameters (see `config/replay.yaml`):

- `ubx_file` *(string, required)* — absolute path to the capture file.
- `frame_id` *(string, default `gps`)* — frame attached to outgoing messages.
- `rate` *(double, default `1.0`)* — playback speed; `0.0` plays as fast as possible.
- `loop` *(bool, default `false`)* — restart from the beginning when EOF is reached.
- `fix_topic` / `vel_topic` — output topic names.

## Offline NAV-PVT → CSV

```bash
ros2 run ubx_parser ubx_to_csv \
    "2026-4-15_174245_serial-COM5(1).ubx" \
    -o pvt.csv
# or, without sourcing the workspace:
python3 -m ubx_parser.ubx_to_csv input.ubx -o pvt.csv
```

Columns: `iTOW_ms, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_sec,
nano_ns, fix_type, num_sv, lat_deg, lon_deg, h_msl_m, height_m, h_acc_m,
v_acc_m, vel_n_mps, vel_e_mps, vel_d_mps, g_speed_mps, head_mot_deg, p_dop`.

## Tests

```bash
# Pure-Python (no ROS needed):
cd src/ubx_parser && PYTHONPATH=. python3 -m pytest test/ -v

# Or via colcon after building:
colcon test --packages-select ubx_parser
colcon test-result --verbose
```

## Notes

- Sample capture stats: 19,636 UBX frames, 325 NAV-PVT records at 1 Hz, 3D fix
  with ~12 SVs, location near Shenzhen, China.
- The parser intentionally tolerates desynchronised streams (interleaved NMEA,
  truncated tails) by skipping bytes that don't form a valid frame.
- Only NAV-PVT is decoded today. Adding additional message types means
  appending a `struct.Struct` definition + dataclass in `ubx_protocol.py`.
