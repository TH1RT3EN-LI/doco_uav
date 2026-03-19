#!/usr/bin/env python3

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import Callable, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


@dataclass
class PositionSample:
    x: float
    y: float
    z: float
    stamp_s: float
    frame_label: str


def parse_args():
    parser = argparse.ArgumentParser(
        description="Subscribe to an odometry topic and print position in realtime."
    )
    parser.add_argument(
        "--source",
        choices=["openvins", "px4"],
        default="openvins",
        help="Message source preset. openvins -> nav_msgs/Odometry, px4 -> px4_msgs/VehicleOdometry.",
    )
    parser.add_argument(
        "--topic",
        default=None,
        help="Override topic name. Defaults: /ov_msckf/odomimu for openvins, /fmu/out/vehicle_odometry for px4.",
    )
    parser.add_argument(
        "--refresh-hz",
        type=float,
        default=10.0,
        help="Terminal refresh rate.",
    )
    parser.add_argument(
        "--no-relative",
        action="store_true",
        help="Do not show delta from first received sample.",
    )
    return parser.parse_args()


def make_openvins_spec():
    from nav_msgs.msg import Odometry

    def extract(msg):
        position = msg.pose.pose.position
        stamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        frame_id = msg.header.frame_id or "<empty>"
        child_frame_id = msg.child_frame_id or "<empty>"
        return PositionSample(
            x=float(position.x),
            y=float(position.y),
            z=float(position.z),
            stamp_s=stamp_s,
            frame_label=f"{frame_id}->{child_frame_id}",
        )

    return Odometry, "/ov_msckf/odomimu", extract


def make_px4_spec():
    try:
        from px4_msgs.msg import VehicleOdometry
    except ImportError as exc:
        raise RuntimeError(
            "px4_msgs is not available in this terminal. Source the workspace that contains px4_msgs first."
        ) from exc

    def extract(msg):
        stamp_us = int(msg.timestamp_sample) if int(msg.timestamp_sample) > 0 else int(msg.timestamp)
        pose_frame = int(msg.pose_frame)
        frame_label = f"px4_pose_frame={pose_frame}"
        return PositionSample(
            x=float(msg.x),
            y=float(msg.y),
            z=float(msg.z),
            stamp_s=float(stamp_us) * 1e-6,
            frame_label=frame_label,
        )

    return VehicleOdometry, "/fmu/out/vehicle_odometry", extract


def build_spec(source: str):
    if source == "openvins":
        return make_openvins_spec()
    if source == "px4":
        return make_px4_spec()
    raise RuntimeError(f"unsupported source: {source}")


class PositionMonitor(Node):
    def __init__(
        self,
        topic_name: str,
        message_type,
        extractor: Callable,
        refresh_hz: float,
        show_relative: bool,
    ):
        super().__init__("monitor_topic_position")
        self.topic_name = topic_name
        self.extractor = extractor
        self.show_relative = show_relative
        self.first_sample: Optional[PositionSample] = None
        self.latest_sample: Optional[PositionSample] = None
        self.sample_count = 0
        self.last_render_width = 0
        self.started_at = time.monotonic()

        self.subscription = self.create_subscription(
            message_type,
            topic_name,
            self.handle_message,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(max(1.0 / max(refresh_hz, 0.5), 0.05), self.render)

    def handle_message(self, msg):
        sample = self.extractor(msg)
        if not self._is_finite(sample):
            return

        self.latest_sample = sample
        if self.first_sample is None:
            self.first_sample = sample
        self.sample_count += 1

    def render(self):
        if self.latest_sample is None:
            line = (
                f"waiting topic={self.topic_name} elapsed={time.monotonic() - self.started_at:.1f}s "
                f"samples={self.sample_count}"
            )
            self._write_line(line)
            return

        sample = self.latest_sample
        line = (
            f"topic={self.topic_name} samples={self.sample_count} frame={sample.frame_label} "
            f"pos=({sample.x:+.3f}, {sample.y:+.3f}, {sample.z:+.3f})"
        )

        if self.show_relative and self.first_sample is not None:
            dx = sample.x - self.first_sample.x
            dy = sample.y - self.first_sample.y
            dz = sample.z - self.first_sample.z
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            line += (
                f" delta=({dx:+.3f}, {dy:+.3f}, {dz:+.3f})"
                f" |delta|={distance:.3f}m"
            )

        self._write_line(line)

    def _write_line(self, line: str):
        padding = max(self.last_render_width - len(line), 0)
        sys.stdout.write("\r" + line + (" " * padding))
        sys.stdout.flush()
        self.last_render_width = len(line)

    @staticmethod
    def _is_finite(sample: PositionSample):
        return all(math.isfinite(value) for value in (sample.x, sample.y, sample.z))


def main():
    args = parse_args()
    message_type, default_topic, extractor = build_spec(args.source)
    topic_name = args.topic or default_topic

    rclpy.init()
    node = PositionMonitor(
        topic_name=topic_name,
        message_type=message_type,
        extractor=extractor,
        refresh_hz=args.refresh_hz,
        show_relative=not args.no_relative,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write("\n")
        sys.stdout.flush()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
