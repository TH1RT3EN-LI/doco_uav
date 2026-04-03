#!/usr/bin/env python3

import argparse
import math
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from math import ceil
from typing import Callable, Dict, List, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


@dataclass
class PositionSample:
    x: float
    y: float
    z: float
    stamp_s: float
    frame_label: str


@dataclass
class SourceSpec:
    key: str
    source_label: str
    topic_name: str
    message_type: type
    extractor: Callable


@dataclass
class SourceState:
    spec: SourceSpec
    first_sample: Optional[PositionSample] = None
    latest_sample: Optional[PositionSample] = None
    sample_count: int = 0
    started_at_wall_s: float = field(default_factory=time.monotonic)
    last_receive_wall_s: Optional[float] = None
    lock: threading.Lock = field(default_factory=threading.Lock)

    def update(self, sample: PositionSample):
        with self.lock:
            self.latest_sample = sample
            if self.first_sample is None:
                self.first_sample = sample
            self.sample_count += 1
            self.last_receive_wall_s = time.monotonic()

    def snapshot(self):
        with self.lock:
            return {
                "first_sample": self.first_sample,
                "latest_sample": self.latest_sample,
                "sample_count": self.sample_count,
                "started_at_wall_s": self.started_at_wall_s,
                "last_receive_wall_s": self.last_receive_wall_s,
                "source_label": self.spec.source_label,
                "topic_name": self.spec.topic_name,
            }

    def reset_origin(self):
        with self.lock:
            if self.latest_sample is not None:
                self.first_sample = self.latest_sample


@dataclass
class SourceWidgets:
    status_var: tk.StringVar
    topic_var: tk.StringVar
    frame_var: tk.StringVar
    sample_count_var: tk.StringVar
    freshness_var: tk.StringVar
    stamp_var: tk.StringVar
    current_x_var: tk.StringVar
    current_y_var: tk.StringVar
    current_z_var: tk.StringVar
    delta_x_var: tk.StringVar
    delta_y_var: tk.StringVar
    delta_z_var: tk.StringVar
    distance_var: tk.StringVar


class MultiSourceMonitorNode(Node):
    def __init__(self, source_states: List[SourceState]):
        super().__init__("monitor_topic_position_gui")
        self.source_states = {state.spec.key: state for state in source_states}
        self._subscriptions = []

        for state in source_states:
            subscription = self.create_subscription(
                state.spec.message_type,
                state.spec.topic_name,
                self._make_callback(state),
                qos_profile_sensor_data,
            )
            self._subscriptions.append(subscription)

    def _make_callback(self, state: SourceState):
        def callback(msg):
            sample = state.spec.extractor(msg)
            if self._is_finite(sample):
                state.update(sample)

        return callback

    @staticmethod
    def _is_finite(sample: PositionSample) -> bool:
        return all(math.isfinite(value) for value in (sample.x, sample.y, sample.z))


class PositionMonitorGui:
    def __init__(self, root: tk.Tk, source_states: List[SourceState], refresh_hz: float, show_relative: bool):
        self.root = root
        self.source_states = source_states
        self.show_relative = show_relative
        self.refresh_period_ms = int(max(1000.0 / max(refresh_hz, 0.5), 100.0))
        self.widgets: Dict[str, SourceWidgets] = {}

        self._build_window()
        self._schedule_refresh()

    def _build_window(self):
        self.root.title("OpenVINS / PX4 位置实时监视")
        self.root.geometry("1520x920")
        self.root.configure(bg="#eef3f8")

        header = tk.Frame(self.root, bg="#0f172a", padx=16, pady=12)
        header.pack(fill="x")

        tk.Label(
            header,
            text="OpenVINS / PX4 位置实时监视",
            font=("Sans", 22, "bold"),
            fg="#f8fafc",
            bg="#0f172a",
        ).pack(anchor="w")
        tk.Label(
            header,
            text="默认对比 OpenVINS 原始输出、发给 PX4 的视觉里程计，以及 PX4 融合输出；单位统一为米（m）。",
            font=("Sans", 11),
            fg="#cbd5e1",
            bg="#0f172a",
        ).pack(anchor="w", pady=(4, 0))

        controls = tk.Frame(self.root, bg="#eef3f8", padx=12, pady=10)
        controls.pack(fill="x")

        tk.Button(
            controls,
            text="全部重置起点",
            font=("Sans", 11, "bold"),
            padx=14,
            pady=8,
            command=self._reset_all_origins,
        ).pack(side="left")

        tk.Button(
            controls,
            text="退出",
            font=("Sans", 11, "bold"),
            padx=14,
            pady=8,
            command=self.root.quit,
        ).pack(side="left", padx=(10, 0))

        grid = tk.Frame(self.root, bg="#eef3f8", padx=12, pady=4)
        grid.pack(fill="both", expand=True)

        column_count = 2
        row_count = max(1, ceil(len(self.source_states) / column_count))
        for row in range(row_count):
            grid.grid_rowconfigure(row, weight=1)
        for column in range(column_count):
            grid.grid_columnconfigure(column, weight=1)

        for index, state in enumerate(self.source_states):
            frame = self._create_source_frame(grid, state)
            row = index // column_count
            column = index % column_count
            frame.grid(row=row, column=column, sticky="nsew", padx=8, pady=8)

    def _create_source_frame(self, parent: tk.Widget, state: SourceState):
        frame = tk.LabelFrame(
            parent,
            text=state.spec.source_label,
            font=("Sans", 14, "bold"),
            bg="#ffffff",
            fg="#111827",
            bd=2,
            padx=14,
            pady=12,
        )

        left = tk.Frame(frame, bg="#ffffff")
        left.pack(side="left", fill="both", expand=True, padx=(0, 16))
        right = tk.Frame(frame, bg="#f8fafc", bd=1, relief="solid")
        right.pack(side="right", fill="y", ipadx=8, ipady=8)

        status_var = tk.StringVar(value="等待数据")
        topic_var = tk.StringVar(value=state.spec.topic_name)
        frame_var = tk.StringVar(value="--")
        sample_count_var = tk.StringVar(value="0")
        freshness_var = tk.StringVar(value="等待数据")
        stamp_var = tk.StringVar(value="--")
        current_x_var = tk.StringVar(value="-- m")
        current_y_var = tk.StringVar(value="-- m")
        current_z_var = tk.StringVar(value="-- m")
        delta_x_var = tk.StringVar(value="-- m")
        delta_y_var = tk.StringVar(value="-- m")
        delta_z_var = tk.StringVar(value="-- m")
        distance_var = tk.StringVar(value="-- m")

        self.widgets[state.spec.key] = SourceWidgets(
            status_var=status_var,
            topic_var=topic_var,
            frame_var=frame_var,
            sample_count_var=sample_count_var,
            freshness_var=freshness_var,
            stamp_var=stamp_var,
            current_x_var=current_x_var,
            current_y_var=current_y_var,
            current_z_var=current_z_var,
            delta_x_var=delta_x_var,
            delta_y_var=delta_y_var,
            delta_z_var=delta_z_var,
            distance_var=distance_var,
        )

        self._add_info_row(left, 0, "状态", status_var)
        self._add_info_row(left, 1, "话题", topic_var)
        self._add_info_row(left, 2, "坐标系", frame_var)
        self._add_info_row(left, 3, "已收样本", sample_count_var)
        self._add_info_row(left, 4, "最新消息距今", freshness_var)
        self._add_info_row(left, 5, "消息时间戳", stamp_var)

        current_box = tk.LabelFrame(
            right,
            text="当前位置",
            font=("Sans", 12, "bold"),
            bg="#f8fafc",
            fg="#111827",
            padx=12,
            pady=10,
        )
        current_box.pack(fill="x", padx=10, pady=(8, 6))
        self._add_big_value_row(current_box, 0, "X", current_x_var)
        self._add_big_value_row(current_box, 1, "Y", current_y_var)
        self._add_big_value_row(current_box, 2, "Z", current_z_var)

        relative_box = tk.LabelFrame(
            right,
            text="相对起点",
            font=("Sans", 12, "bold"),
            bg="#f8fafc",
            fg="#111827",
            padx=12,
            pady=10,
        )
        relative_box.pack(fill="x", padx=10, pady=(6, 8))
        self._add_big_value_row(relative_box, 0, "ΔX", delta_x_var)
        self._add_big_value_row(relative_box, 1, "ΔY", delta_y_var)
        self._add_big_value_row(relative_box, 2, "ΔZ", delta_z_var)
        self._add_big_value_row(relative_box, 3, "总位移", distance_var)

        tk.Button(
            right,
            text="将当前点设为起点",
            font=("Sans", 11, "bold"),
            padx=12,
            pady=8,
            command=state.reset_origin,
        ).pack(fill="x", padx=10, pady=(0, 8))

        return frame

    def _add_info_row(self, parent: tk.Widget, row: int, label: str, variable: tk.StringVar):
        tk.Label(
            parent,
            text=label,
            anchor="w",
            font=("Sans", 11, "bold"),
            bg="#ffffff",
            fg="#374151",
            width=10,
        ).grid(row=row, column=0, sticky="nw", pady=5)
        tk.Label(
            parent,
            textvariable=variable,
            justify="left",
            anchor="w",
            wraplength=420,
            font=("Sans", 11),
            bg="#ffffff",
            fg="#111827",
        ).grid(row=row, column=1, sticky="nw", pady=5, padx=(8, 0))

    def _add_big_value_row(self, parent: tk.Widget, row: int, label: str, variable: tk.StringVar):
        tk.Label(
            parent,
            text=label,
            anchor="w",
            font=("Sans", 12, "bold"),
            bg="#f8fafc",
            fg="#374151",
            width=6,
        ).grid(row=row, column=0, sticky="w", pady=4)
        tk.Label(
            parent,
            textvariable=variable,
            anchor="e",
            font=("Consolas", 18, "bold"),
            bg="#f8fafc",
            fg="#0f172a",
            width=14,
        ).grid(row=row, column=1, sticky="e", pady=4, padx=(10, 0))

    def _reset_all_origins(self):
        for state in self.source_states:
            state.reset_origin()

    def _schedule_refresh(self):
        self._refresh_widgets()
        self.root.after(self.refresh_period_ms, self._schedule_refresh)

    def _refresh_widgets(self):
        now = time.monotonic()
        for state in self.source_states:
            snapshot = state.snapshot()
            widgets = self.widgets[state.spec.key]
            latest_sample = snapshot["latest_sample"]
            first_sample = snapshot["first_sample"]
            sample_count = snapshot["sample_count"]
            last_receive_wall_s = snapshot["last_receive_wall_s"]
            started_at_wall_s = snapshot["started_at_wall_s"]

            widgets.topic_var.set(snapshot["topic_name"])
            widgets.sample_count_var.set(str(sample_count))

            if latest_sample is None:
                waited_s = now - started_at_wall_s
                widgets.status_var.set("等待数据")
                widgets.frame_var.set("--")
                widgets.freshness_var.set(f"已等待 {waited_s:.1f} s")
                widgets.stamp_var.set("--")
                self._set_position_vars(widgets, None, None)
                continue

            freshness_s = max(now - last_receive_wall_s, 0.0)
            widgets.status_var.set("正常接收" if freshness_s < 1.0 else "数据变慢")
            widgets.frame_var.set(latest_sample.frame_label)
            widgets.freshness_var.set(f"{freshness_s:.2f} s")
            widgets.stamp_var.set(f"{latest_sample.stamp_s:.3f} s")
            self._set_position_vars(widgets, latest_sample, first_sample)

    def _set_position_vars(
        self,
        widgets: SourceWidgets,
        latest_sample: Optional[PositionSample],
        first_sample: Optional[PositionSample],
    ):
        if latest_sample is None:
            widgets.current_x_var.set("-- m")
            widgets.current_y_var.set("-- m")
            widgets.current_z_var.set("-- m")
            widgets.delta_x_var.set("-- m")
            widgets.delta_y_var.set("-- m")
            widgets.delta_z_var.set("-- m")
            widgets.distance_var.set("-- m")
            return

        widgets.current_x_var.set(self._format_distance(latest_sample.x))
        widgets.current_y_var.set(self._format_distance(latest_sample.y))
        widgets.current_z_var.set(self._format_distance(latest_sample.z))

        if not self.show_relative:
            widgets.delta_x_var.set("已关闭")
            widgets.delta_y_var.set("已关闭")
            widgets.delta_z_var.set("已关闭")
            widgets.distance_var.set("已关闭")
            return

        dx = latest_sample.x - first_sample.x
        dy = latest_sample.y - first_sample.y
        dz = latest_sample.z - first_sample.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        widgets.delta_x_var.set(self._format_distance(dx))
        widgets.delta_y_var.set(self._format_distance(dy))
        widgets.delta_z_var.set(self._format_distance(dz))
        widgets.distance_var.set(self._format_distance(distance))

    @staticmethod
    def _format_distance(value: float) -> str:
        return f"{value:+.3f} m"


def describe_px4_pose_frame(pose_frame: int) -> str:
    mapping = {
        0: "PX4 未知坐标系",
        1: "PX4 NED 坐标系",
        2: "PX4 FRD 坐标系",
    }
    return mapping.get(pose_frame, f"PX4 未知 pose_frame={pose_frame}")


def convert_px4_position_to_ros_xyz(x: float, y: float, z: float, pose_frame: int):
    if pose_frame == 1:
        return y, x, -z, "ROS ENU（由 PX4 NED 转换）"
    if pose_frame == 2:
        return x, -y, -z, "ROS FLU（由 PX4 FRD 转换）"
    return x, y, z, f"ROS 默认方向显示（原始: {describe_px4_pose_frame(pose_frame)})"


def describe_ros_odometry_frame(frame_id: str, child_frame_id: str) -> str:
    if frame_id and child_frame_id:
        return f"ROS Odom（{frame_id} -> {child_frame_id}）"
    if frame_id:
        return f"ROS frame_id={frame_id}"
    if child_frame_id:
        return f"ROS child_frame_id={child_frame_id}"
    return "ROS Odom（未设置 frame_id / child_frame_id）"


def make_openvins_spec(key: str, source_label: str, topic_name: str) -> SourceSpec:
    try:
        from nav_msgs.msg import Odometry
    except ImportError as exc:
        raise RuntimeError("当前终端没有 nav_msgs。请先 source 含有 nav_msgs 的工作区。") from exc

    def extract(msg):
        stamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        return PositionSample(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            z=float(msg.pose.pose.position.z),
            stamp_s=stamp_s,
            frame_label=describe_ros_odometry_frame(msg.header.frame_id, msg.child_frame_id),
        )

    return SourceSpec(
        key=key,
        source_label=source_label,
        topic_name=topic_name,
        message_type=Odometry,
        extractor=extract,
    )


def make_px4_spec(key: str, source_label: str, topic_name: str) -> SourceSpec:
    try:
        from px4_msgs.msg import VehicleOdometry
    except ImportError as exc:
        raise RuntimeError("当前终端没有 px4_msgs。请先 source 含有 px4_msgs 的工作区。") from exc

    def extract(msg):
        stamp_us = int(msg.timestamp_sample) if int(msg.timestamp_sample) > 0 else int(msg.timestamp)
        pose_frame = int(msg.pose_frame)
        ros_x, ros_y, ros_z, ros_frame_label = convert_px4_position_to_ros_xyz(
            float(msg.position[0]),
            float(msg.position[1]),
            float(msg.position[2]),
            pose_frame,
        )
        return PositionSample(
            x=ros_x,
            y=ros_y,
            z=ros_z,
            stamp_s=float(stamp_us) * 1e-6,
            frame_label=ros_frame_label,
        )

    return SourceSpec(
        key=key,
        source_label=source_label,
        topic_name=topic_name,
        message_type=VehicleOdometry,
        extractor=extract,
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description="GUI 方式实时监视 OpenVINS 原始输出、PX4 视觉输入与 PX4 融合输出的位置。"
    )
    parser.add_argument(
        "--sources",
        default="openvins,px4_in,px4_out",
        help="要显示的数据源，默认全部显示：openvins,px4_in,px4_out",
    )
    parser.add_argument(
        "--source",
        default=None,
        help="兼容旧用法。若设置，则只显示单一路数据源。可选：openvins / px4_in / px4_out / px4",
    )
    parser.add_argument("--topic", default=None, help="只在单一路模式下使用：覆盖该数据源的话题名。")
    parser.add_argument(
        "--openvins-topic",
        default="/ov_msckf/odomimu",
        help="OpenVINS 原始里程计话题。",
    )
    parser.add_argument(
        "--px4-input-topic",
        default="/fmu/in/vehicle_visual_odometry",
        help="发给 PX4 的视觉里程计输入话题。",
    )
    parser.add_argument(
        "--px4-output-topic",
        default="/fmu/out/vehicle_odometry",
        help="PX4 融合后的输出话题。",
    )
    parser.add_argument(
        "--refresh-hz",
        type=float,
        default=5.0,
        help="界面刷新频率，默认 5 Hz。",
    )
    parser.add_argument(
        "--no-relative",
        action="store_true",
        help="不显示相对起点位移。",
    )
    return parser.parse_args()


def build_source_specs(args) -> List[SourceSpec]:
    source_keys = [item.strip() for item in args.sources.split(",") if item.strip()]
    if args.source:
        alias = "px4_out" if args.source == "px4" else args.source
        source_keys = [alias]

    builders = {
        "openvins": lambda topic: make_openvins_spec("openvins", "OpenVINS 原始位置", topic),
        "px4_in": lambda topic: make_px4_spec("px4_in", "发给 PX4 的视觉位置", topic),
        "px4_out": lambda topic: make_px4_spec("px4_out", "PX4 输出位置", topic),
    }
    default_topics = {
        "openvins": args.openvins_topic,
        "px4_in": args.px4_input_topic,
        "px4_out": args.px4_output_topic,
    }

    specs = []
    for key in source_keys:
        if key not in builders:
            raise RuntimeError(f"不支持的数据源: {key}")
        topic_name = args.topic if args.topic and len(source_keys) == 1 else default_topics[key]
        specs.append(builders[key](topic_name))

    if not specs:
        raise RuntimeError("没有可显示的数据源。")
    return specs


def main():
    args = parse_args()
    source_specs = build_source_specs(args)
    source_states = [SourceState(spec=spec) for spec in source_specs]

    rclpy.init()
    node = MultiSourceMonitorNode(source_states)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    root = tk.Tk()
    app = PositionMonitorGui(
        root=root,
        source_states=source_states,
        refresh_hz=args.refresh_hz,
        show_relative=not args.no_relative,
    )

    def close_app():
        root.quit()

    root.protocol("WM_DELETE_WINDOW", close_app)

    try:
        root.mainloop()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
