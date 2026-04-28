"""ROS 2 node that replays a .ubx capture as live ROS messages.

The node loads a u-blox binary log, decodes NAV-PVT frames, and republishes
them on ``/fix`` (sensor_msgs/NavSatFix) and ``/vel`` (geometry_msgs/TwistStamped),
honouring the original inter-message timing derived from the iTOW field.
"""

from __future__ import annotations

import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

from ubx_parser.ubx_protocol import (
    CLS_NAV,
    ID_NAV_PVT,
    decode_nav_pvt,
    iter_frames,
)


def _fix_status(fix_type: int) -> int:
    """Translate a UBX fix type to a NavSatStatus.status value."""
    if fix_type in (3, 4):  # 3D fix or GNSS+DR combined fix
        return NavSatStatus.STATUS_FIX
    if fix_type == 2:  # 2D fix
        return NavSatStatus.STATUS_FIX
    return NavSatStatus.STATUS_NO_FIX


class UbxReplayNode(Node):

    def __init__(self) -> None:
        super().__init__('ubx_replay_node')

        self.declare_parameter('ubx_file', '')
        self.declare_parameter('frame_id', 'gps')
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('loop', False)
        self.declare_parameter('fix_topic', 'fix')
        self.declare_parameter('vel_topic', 'vel')

        self._frame_id = self.get_parameter('frame_id').value
        self._rate = float(self.get_parameter('rate').value)
        self._loop = bool(self.get_parameter('loop').value)
        ubx_path = str(self.get_parameter('ubx_file').value)

        if not ubx_path:
            raise RuntimeError(
                "ubx_file parameter is required (path to a u-blox .ubx capture)")
        self._ubx_path = Path(ubx_path).expanduser()
        if not self._ubx_path.is_file():
            raise FileNotFoundError(f"UBX file not found: {self._ubx_path}")

        self._fix_pub = self.create_publisher(
            NavSatFix, str(self.get_parameter('fix_topic').value), 10)
        self._vel_pub = self.create_publisher(
            TwistStamped, str(self.get_parameter('vel_topic').value), 10)

        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def destroy_node(self) -> bool:
        self._stop.set()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        return super().destroy_node()

    def _run(self) -> None:
        self.get_logger().info(
            f"Replaying {self._ubx_path} at {self._rate}x (loop={self._loop})")
        data = self._ubx_path.read_bytes()
        self.get_logger().info(f"Loaded {len(data)} bytes from capture")

        while not self._stop.is_set():
            self._play_once(data)
            if not self._loop:
                break

        self.get_logger().info("Replay finished")

    def _play_once(self, data: bytes) -> None:
        prev_itow_ms: int | None = None
        wall_anchor: float | None = None
        first_itow_ms: int | None = None
        published = 0

        for frame in iter_frames(data):
            if self._stop.is_set():
                return
            if frame.key != (CLS_NAV, ID_NAV_PVT):
                continue
            pvt = decode_nav_pvt(frame.payload)
            if pvt is None:
                continue

            if first_itow_ms is None:
                first_itow_ms = pvt.i_tow_ms
                wall_anchor = time.monotonic()
            elif self._rate > 0.0:
                target_offset = (pvt.i_tow_ms - first_itow_ms) / 1000.0 / self._rate
                sleep_for = (wall_anchor + target_offset) - time.monotonic()
                if sleep_for > 0:
                    if self._stop.wait(timeout=sleep_for):
                        return

            self._publish(pvt)
            prev_itow_ms = pvt.i_tow_ms
            published += 1

        self.get_logger().info(
            f"Played {published} NAV-PVT messages "
            f"(last iTOW={prev_itow_ms} ms)")

    def _publish(self, pvt) -> None:
        stamp = self.get_clock().now().to_msg()

        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = self._frame_id
        fix.status.status = _fix_status(pvt.fix_type)
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = pvt.lat_deg
        fix.longitude = pvt.lon_deg
        fix.altitude = pvt.h_msl_m
        # Diagonal covariance derived from horizontal/vertical accuracy (m).
        h_var = pvt.h_acc_m * pvt.h_acc_m
        v_var = pvt.v_acc_m * pvt.v_acc_m
        fix.position_covariance = [
            h_var, 0.0, 0.0,
            0.0, h_var, 0.0,
            0.0, 0.0, v_var,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._fix_pub.publish(fix)

        vel = TwistStamped()
        vel.header.stamp = stamp
        vel.header.frame_id = self._frame_id
        # NED -> ENU: x=East, y=North, z=Up
        vel.twist.linear.x = pvt.vel_e_mps
        vel.twist.linear.y = pvt.vel_n_mps
        vel.twist.linear.z = -pvt.vel_d_mps
        self._vel_pub.publish(vel)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UbxReplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
