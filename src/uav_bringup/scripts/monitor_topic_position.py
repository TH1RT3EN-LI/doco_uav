#!/usr/bin/env python3

import argparse
import math
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


FAST_TOPIC_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

STATUS_STREAM_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

STATUS_LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


LEVEL_STYLES = {
    "wait": {"bg": "#e2e8f0", "fg": "#334155"},
    "ok": {"bg": "#dcfce7", "fg": "#166534"},
    "warn": {"bg": "#fef3c7", "fg": "#92400e"},
    "error": {"bg": "#fee2e2", "fg": "#b91c1c"},
}


@dataclass(frozen=True)
class PositionSample:
    x: float
    y: float
    z: float
    stamp_s: Optional[float]
    frame_label: str
    orientation_xyzw: Optional[Tuple[float, float, float, float]] = None
    rpy_rad: Optional[Tuple[float, float, float]] = None
    orientation_frame_label: str = "--"
    velocity_xyz: Optional[Tuple[float, float, float]] = None
    velocity_frame_label: str = "--"


@dataclass(frozen=True)
class PositionSourceSpec:
    key: str
    source_label: str
    topic_name: str
    message_type: type
    extractor: Callable[[Any], PositionSample]
    qos_profile: QoSProfile
    slow_after_s: float = 0.5
    stale_after_s: float = 2.0


@dataclass(frozen=True)
class PositionSourceView:
    spec: PositionSourceSpec
    first_sample: Optional[PositionSample]
    latest_sample: Optional[PositionSample]
    receive_count: int
    sample_count: int
    started_at_wall_s: float
    last_receive_wall_s: Optional[float]
    estimated_rate_hz: Optional[float]
    last_error: Optional[str]


@dataclass(frozen=True)
class StatusChannelSpec:
    key: str
    label: str
    topic_name: str
    message_type: type
    extractor: Callable[[Any], Any]
    qos_profile: QoSProfile
    slow_after_s: float
    stale_after_s: float
    expect_continuous_updates: bool = True


@dataclass(frozen=True)
class StatusChannelSnapshot:
    spec: StatusChannelSpec
    payload: Any
    receive_count: int
    sample_count: int
    started_at_wall_s: float
    last_receive_wall_s: Optional[float]
    last_change_wall_s: Optional[float]
    estimated_rate_hz: Optional[float]
    last_error: Optional[str]


@dataclass(frozen=True)
class StatusRender:
    level: str
    summary: str
    detail_rows: List[Tuple[str, str]]


@dataclass(frozen=True)
class StatusSourceSpec:
    key: str
    source_label: str
    channels: List[StatusChannelSpec]
    renderer: Callable[[Dict[str, StatusChannelSnapshot], float], StatusRender]


@dataclass(frozen=True)
class StatusSourceView:
    spec: StatusSourceSpec
    channels: Dict[str, StatusChannelSnapshot]
    render: StatusRender


@dataclass
class PositionWidgets:
    status_var: tk.StringVar
    topic_var: tk.StringVar
    frame_var: tk.StringVar
    receive_count_var: tk.StringVar
    sample_count_var: tk.StringVar
    rate_var: tk.StringVar
    freshness_var: tk.StringVar
    stamp_var: tk.StringVar
    current_x_var: tk.StringVar
    current_y_var: tk.StringVar
    current_z_var: tk.StringVar
    delta_x_var: tk.StringVar
    delta_y_var: tk.StringVar
    delta_z_var: tk.StringVar
    distance_var: tk.StringVar
    orientation_frame_var: tk.StringVar
    roll_var: tk.StringVar
    pitch_var: tk.StringVar
    yaw_var: tk.StringVar
    quaternion_var: tk.StringVar
    velocity_frame_var: tk.StringVar
    velocity_x_var: tk.StringVar
    velocity_y_var: tk.StringVar
    velocity_z_var: tk.StringVar
    badge_label: tk.Label


@dataclass
class StatusWidgets:
    summary_var: tk.StringVar
    topic_var: tk.StringVar
    freshness_var: tk.StringVar
    receive_count_var: tk.StringVar
    sample_count_var: tk.StringVar
    details_var: tk.StringVar
    badge_label: tk.Label


class PositionSourceState:
    def __init__(self, spec: PositionSourceSpec):
        self.spec = spec
        self.first_sample: Optional[PositionSample] = None
        self.latest_sample: Optional[PositionSample] = None
        self.receive_count = 0
        self.sample_count = 0
        self.started_at_wall_s = time.monotonic()
        self.last_receive_wall_s: Optional[float] = None
        self.estimated_rate_hz: Optional[float] = None
        self.last_error: Optional[str] = None
        self.lock = threading.Lock()

    def _record_receive_locked(self, now: float):
        if self.last_receive_wall_s is not None:
            interval_s = now - self.last_receive_wall_s
            if interval_s > 1e-6:
                instant_rate_hz = 1.0 / interval_s
                if self.estimated_rate_hz is None:
                    self.estimated_rate_hz = instant_rate_hz
                else:
                    self.estimated_rate_hz = (
                        0.25 * instant_rate_hz + 0.75 * self.estimated_rate_hz
                    )
        self.receive_count += 1
        self.last_receive_wall_s = now

    def record_receive(self):
        now = time.monotonic()
        with self.lock:
            self._record_receive_locked(now)

    def accept_sample(self, sample: PositionSample):
        with self.lock:
            self.latest_sample = sample
            if self.first_sample is None:
                self.first_sample = sample
            self.sample_count += 1
            self.last_error = None

    def record_error(self, error_message: str):
        with self.lock:
            self.last_error = error_message

    def snapshot(self) -> PositionSourceView:
        with self.lock:
            return PositionSourceView(
                spec=self.spec,
                first_sample=self.first_sample,
                latest_sample=self.latest_sample,
                receive_count=self.receive_count,
                sample_count=self.sample_count,
                started_at_wall_s=self.started_at_wall_s,
                last_receive_wall_s=self.last_receive_wall_s,
                estimated_rate_hz=self.estimated_rate_hz,
                last_error=self.last_error,
            )

    def reset_origin(self):
        with self.lock:
            if self.latest_sample is not None:
                self.first_sample = self.latest_sample


class StatusChannelState:
    def __init__(self, spec: StatusChannelSpec):
        self.spec = spec
        self.payload: Any = None
        self.receive_count = 0
        self.sample_count = 0
        self.started_at_wall_s = time.monotonic()
        self.last_receive_wall_s: Optional[float] = None
        self.last_change_wall_s: Optional[float] = None
        self.estimated_rate_hz: Optional[float] = None
        self.last_error: Optional[str] = None
        self.lock = threading.Lock()

    def _record_receive_locked(self, now: float):
        if self.last_receive_wall_s is not None:
            interval_s = now - self.last_receive_wall_s
            if interval_s > 1e-6:
                instant_rate_hz = 1.0 / interval_s
                if self.estimated_rate_hz is None:
                    self.estimated_rate_hz = instant_rate_hz
                else:
                    self.estimated_rate_hz = (
                        0.25 * instant_rate_hz + 0.75 * self.estimated_rate_hz
                    )
        self.receive_count += 1
        self.last_receive_wall_s = now

    def record_receive(self):
        now = time.monotonic()
        with self.lock:
            self._record_receive_locked(now)

    def accept_payload(self, payload: Any):
        now = time.monotonic()
        with self.lock:
            if self.sample_count == 0 or payload != self.payload:
                self.last_change_wall_s = now
            self.payload = payload
            self.sample_count += 1
            self.last_error = None

    def record_error(self, error_message: str):
        with self.lock:
            self.last_error = error_message

    def snapshot(self) -> StatusChannelSnapshot:
        with self.lock:
            return StatusChannelSnapshot(
                spec=self.spec,
                payload=self.payload,
                receive_count=self.receive_count,
                sample_count=self.sample_count,
                started_at_wall_s=self.started_at_wall_s,
                last_receive_wall_s=self.last_receive_wall_s,
                last_change_wall_s=self.last_change_wall_s,
                estimated_rate_hz=self.estimated_rate_hz,
                last_error=self.last_error,
            )


class StatusSourceState:
    def __init__(self, spec: StatusSourceSpec):
        self.spec = spec
        self.channel_states = {
            channel_spec.key: StatusChannelState(channel_spec)
            for channel_spec in spec.channels
        }

    def snapshot(self, now_wall_s: float) -> StatusSourceView:
        channel_snapshots = {
            key: state.snapshot() for key, state in self.channel_states.items()
        }
        return StatusSourceView(
            spec=self.spec,
            channels=channel_snapshots,
            render=self.spec.renderer(channel_snapshots, now_wall_s),
        )


class TopicMonitorNode(Node):
    def __init__(
        self,
        position_states: List[PositionSourceState],
        status_states: List[StatusSourceState],
    ):
        super().__init__("monitor_topic_position_gui")
        self._position_states = {
            state.spec.key: state for state in position_states
        }
        self._status_states = {
            state.spec.key: state for state in status_states
        }
        self._subscriptions_by_source: Dict[str, List[Any]] = {}
        self._enabled_sources = set()
        self._subscription_lock = threading.Lock()

        for source_key in list(self._position_states.keys()) + list(self._status_states.keys()):
            self.set_source_enabled(source_key, True)

    def set_source_enabled(self, source_key: str, enabled: bool):
        with self._subscription_lock:
            already_enabled = source_key in self._enabled_sources
            if enabled == already_enabled:
                return

            if enabled:
                subscriptions: List[Any] = []
                position_state = self._position_states.get(source_key)
                if position_state is not None:
                    subscriptions.append(
                        self.create_subscription(
                            position_state.spec.message_type,
                            position_state.spec.topic_name,
                            self._make_position_callback(position_state),
                            position_state.spec.qos_profile,
                        )
                    )

                status_state = self._status_states.get(source_key)
                if status_state is not None:
                    for channel_key, channel_state in status_state.channel_states.items():
                        subscriptions.append(
                            self.create_subscription(
                                channel_state.spec.message_type,
                                channel_state.spec.topic_name,
                                self._make_status_callback(
                                    status_state.spec.key,
                                    channel_key,
                                    channel_state,
                                ),
                                channel_state.spec.qos_profile,
                            )
                        )

                if not subscriptions:
                    raise KeyError(f"unknown source key: {source_key}")
                self._subscriptions_by_source[source_key] = subscriptions
                self._enabled_sources.add(source_key)
                return

            subscriptions = self._subscriptions_by_source.pop(source_key, [])
            for subscription in subscriptions:
                self.destroy_subscription(subscription)
            self._enabled_sources.discard(source_key)

    def is_source_enabled(self, source_key: str) -> bool:
        with self._subscription_lock:
            return source_key in self._enabled_sources

    def _make_position_callback(self, state: PositionSourceState):
        def callback(msg: Any):
            state.record_receive()
            try:
                sample = state.spec.extractor(msg)
                if self._is_finite_position(sample):
                    state.accept_sample(sample)
                else:
                    state.record_error("收到非有限位置值，已忽略。")
            except Exception as exc:
                state.record_error(f"{type(exc).__name__}: {exc}")

        return callback

    def _make_status_callback(
        self,
        source_key: str,
        channel_key: str,
        channel_state: StatusChannelState,
    ):
        def callback(msg: Any):
            channel_state.record_receive()
            try:
                payload = channel_state.spec.extractor(msg)
                channel_state.accept_payload(payload)
            except Exception as exc:
                channel_state.record_error(
                    f"{source_key}/{channel_key} {type(exc).__name__}: {exc}"
                )

        return callback

    @staticmethod
    def _is_finite_position(sample: PositionSample) -> bool:
        return all(math.isfinite(value) for value in (sample.x, sample.y, sample.z))


class PositionMonitorGui:
    def __init__(
        self,
        root: tk.Tk,
        monitor_node: TopicMonitorNode,
        position_states: List[PositionSourceState],
        status_states: List[StatusSourceState],
        refresh_hz: float,
        show_relative: bool,
    ):
        self.root = root
        self.monitor_node = monitor_node
        self.position_states = position_states
        self.status_states = status_states
        self.show_relative = show_relative
        self.refresh_period_ms = int(max(1000.0 / max(refresh_hz, 0.5), 100.0))

        self.position_widgets: Dict[str, PositionWidgets] = {}
        self.status_widgets: Dict[str, StatusWidgets] = {}
        self.position_frames: List[tk.LabelFrame] = []
        self.status_frames: List[tk.LabelFrame] = []
        self.position_frame_by_key: Dict[str, tk.LabelFrame] = {}
        self.status_frame_by_key: Dict[str, tk.LabelFrame] = {}
        self.position_grid: Optional[tk.Frame] = None
        self.status_grid: Optional[tk.Frame] = None
        self.position_empty_label: Optional[tk.Label] = None
        self.status_empty_label: Optional[tk.Label] = None
        self.body_canvas: Optional[tk.Canvas] = None
        self.body_window_id: Optional[int] = None
        self._layout_after_id: Optional[str] = None
        self.source_enabled_vars: Dict[str, tk.BooleanVar] = {}

        self._build_window()
        self._schedule_refresh()

    def _build_window(self):
        self.root.title("OV / PX4 低开销状态监视器")
        self.root.geometry("1560x960")
        self.root.minsize(960, 640)
        self.root.configure(bg="#eef3f8")

        header = tk.Frame(self.root, bg="#0f172a", padx=16, pady=12)
        header.pack(fill="x")

        tk.Label(
            header,
            text="OV / PX4 低开销状态监视器",
            font=("Sans", 22, "bold"),
            fg="#f8fafc",
            bg="#0f172a",
        ).pack(anchor="w")
        tk.Label(
            header,
            text=(
                "同时监视位置、姿态、速度链路，以及 OV guard、PX4 local validity 和飞控模式；"
                "所有订阅仅缓存最新快照，GUI 定时刷新，不累计历史曲线。"
            ),
            font=("Sans", 11),
            fg="#cbd5e1",
            bg="#0f172a",
        ).pack(anchor="w", pady=(4, 0))

        controls = tk.Frame(self.root, bg="#eef3f8", padx=12, pady=10)
        controls.pack(fill="x")

        reset_button = tk.Button(
            controls,
            text="全部重置起点",
            font=("Sans", 11, "bold"),
            padx=14,
            pady=8,
            command=self._reset_all_origins,
            state="normal" if self.position_states else "disabled",
        )
        reset_button.pack(side="left")

        tk.Button(
            controls,
            text="退出",
            font=("Sans", 11, "bold"),
            padx=14,
            pady=8,
            command=self.root.quit,
        ).pack(side="left", padx=(10, 0))

        source_controls = tk.LabelFrame(
            controls,
            text="订阅源",
            font=("Sans", 11, "bold"),
            bg="#eef3f8",
            fg="#0f172a",
            padx=10,
            pady=6,
        )
        source_controls.pack(side="left", fill="x", expand=True, padx=(18, 0))
        source_controls.grid_columnconfigure(0, weight=1)
        source_order = [state.spec.key for state in self.position_states] + [
            state.spec.key for state in self.status_states
        ]
        source_labels = {
            **{state.spec.key: state.spec.source_label for state in self.position_states},
            **{state.spec.key: state.spec.source_label for state in self.status_states},
        }
        for index, source_key in enumerate(source_order):
            enabled_var = tk.BooleanVar(value=self.monitor_node.is_source_enabled(source_key))
            self.source_enabled_vars[source_key] = enabled_var
            tk.Checkbutton(
                source_controls,
                text=source_labels[source_key],
                variable=enabled_var,
                onvalue=True,
                offvalue=False,
                command=lambda key=source_key: self._toggle_source_subscription(key),
                anchor="w",
                bg="#eef3f8",
                fg="#0f172a",
                selectcolor="#ffffff",
                activebackground="#eef3f8",
            ).grid(
                row=index // 4,
                column=index % 4,
                sticky="w",
                padx=(0, 18),
                pady=2,
            )

        body_shell = tk.Frame(self.root, bg="#eef3f8")
        body_shell.pack(fill="both", expand=True)
        self.body_canvas = tk.Canvas(
            body_shell,
            bg="#eef3f8",
            highlightthickness=0,
            bd=0,
        )
        scrollbar = tk.Scrollbar(body_shell, orient="vertical", command=self.body_canvas.yview)
        self.body_canvas.configure(yscrollcommand=scrollbar.set)
        self.body_canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        body = tk.Frame(self.body_canvas, bg="#eef3f8", padx=12, pady=4)
        self.body_window_id = self.body_canvas.create_window((0, 0), window=body, anchor="nw")
        body.bind("<Configure>", self._on_body_configure)
        self.body_canvas.bind("<Configure>", self._on_canvas_configure)
        self.body_canvas.bind_all("<MouseWheel>", self._on_mousewheel)
        self.body_canvas.bind_all("<Button-4>", self._on_mousewheel_linux_up)
        self.body_canvas.bind_all("<Button-5>", self._on_mousewheel_linux_down)

        if self.position_states:
            self._build_section_title(body, "位置链路")
            self.position_grid = tk.Frame(body, bg="#eef3f8")
            self.position_grid.pack(fill="x", expand=True)
            self.position_empty_label = tk.Label(
                self.position_grid,
                text="当前没有启用的位置卡片。",
                font=("Sans", 11),
                fg="#64748b",
                bg="#eef3f8",
                anchor="w",
                justify="left",
                padx=8,
                pady=8,
            )
            self._build_position_grid(self.position_grid)

        if self.status_states:
            self._build_section_title(body, "状态链路")
            self.status_grid = tk.Frame(body, bg="#eef3f8")
            self.status_grid.pack(fill="x", expand=True)
            self.status_empty_label = tk.Label(
                self.status_grid,
                text="当前没有启用的状态卡片。",
                font=("Sans", 11),
                fg="#64748b",
                bg="#eef3f8",
                anchor="w",
                justify="left",
                padx=8,
                pady=8,
            )
            self._build_status_grid(self.status_grid)

        self._relayout_cards()
        self.root.after_idle(self._relayout_cards)

    def _build_section_title(self, parent: tk.Widget, title: str):
        tk.Label(
            parent,
            text=title,
            font=("Sans", 16, "bold"),
            fg="#0f172a",
            bg="#eef3f8",
            pady=8,
        ).pack(anchor="w")

    def _build_position_grid(self, parent: tk.Widget):
        self.position_frames = []
        self.position_frame_by_key = {}
        for state in self.position_states:
            frame = self._create_position_frame(parent, state)
            self.position_frames.append(frame)
            self.position_frame_by_key[state.spec.key] = frame

    def _build_status_grid(self, parent: tk.Widget):
        self.status_frames = []
        self.status_frame_by_key = {}
        for state in self.status_states:
            frame = self._create_status_frame(parent, state)
            self.status_frames.append(frame)
            self.status_frame_by_key[state.spec.key] = frame

    def _create_position_frame(
        self,
        parent: tk.Widget,
        state: PositionSourceState,
    ) -> tk.LabelFrame:
        frame = tk.LabelFrame(
            parent,
            text=state.spec.source_label,
            font=("Sans", 13, "bold"),
            bg="#ffffff",
            fg="#111827",
            bd=2,
            padx=14,
            pady=12,
        )
        frame.grid_columnconfigure(0, weight=1)

        details = tk.Frame(frame, bg="#ffffff")
        details.pack(fill="x", expand=True)
        details.grid_columnconfigure(1, weight=1)

        metrics = tk.Frame(frame, bg="#ffffff")
        metrics.pack(fill="x", expand=True, pady=(8, 0))

        status_var = tk.StringVar(value="等待数据")
        topic_var = tk.StringVar(value=state.spec.topic_name)
        frame_var = tk.StringVar(value="--")
        receive_count_var = tk.StringVar(value="0")
        sample_count_var = tk.StringVar(value="0")
        rate_var = tk.StringVar(value="--")
        freshness_var = tk.StringVar(value="等待数据")
        stamp_var = tk.StringVar(value="--")
        current_x_var = tk.StringVar(value="-- m")
        current_y_var = tk.StringVar(value="-- m")
        current_z_var = tk.StringVar(value="-- m")
        delta_x_var = tk.StringVar(value="-- m")
        delta_y_var = tk.StringVar(value="-- m")
        delta_z_var = tk.StringVar(value="-- m")
        distance_var = tk.StringVar(value="-- m")
        orientation_frame_var = tk.StringVar(value="--")
        roll_var = tk.StringVar(value="--")
        pitch_var = tk.StringVar(value="--")
        yaw_var = tk.StringVar(value="--")
        quaternion_var = tk.StringVar(value="--")
        velocity_frame_var = tk.StringVar(value="--")
        velocity_x_var = tk.StringVar(value="-- m/s")
        velocity_y_var = tk.StringVar(value="-- m/s")
        velocity_z_var = tk.StringVar(value="-- m/s")

        badge_label = tk.Label(
            details,
            textvariable=status_var,
            font=("Sans", 11, "bold"),
            padx=10,
            pady=5,
            bd=0,
        )
        badge_label.grid(row=0, column=1, sticky="w", pady=5, padx=(8, 0))
        self._set_badge_style(badge_label, "wait")

        tk.Label(
            details,
            text="状态",
            anchor="w",
            font=("Sans", 11, "bold"),
            bg="#ffffff",
            fg="#374151",
            width=10,
        ).grid(row=0, column=0, sticky="nw", pady=5)

        self.position_widgets[state.spec.key] = PositionWidgets(
            status_var=status_var,
            topic_var=topic_var,
            frame_var=frame_var,
            receive_count_var=receive_count_var,
            sample_count_var=sample_count_var,
            rate_var=rate_var,
            freshness_var=freshness_var,
            stamp_var=stamp_var,
            current_x_var=current_x_var,
            current_y_var=current_y_var,
            current_z_var=current_z_var,
            delta_x_var=delta_x_var,
            delta_y_var=delta_y_var,
            delta_z_var=delta_z_var,
            distance_var=distance_var,
            orientation_frame_var=orientation_frame_var,
            roll_var=roll_var,
            pitch_var=pitch_var,
            yaw_var=yaw_var,
            quaternion_var=quaternion_var,
            velocity_frame_var=velocity_frame_var,
            velocity_x_var=velocity_x_var,
            velocity_y_var=velocity_y_var,
            velocity_z_var=velocity_z_var,
            badge_label=badge_label,
        )

        self._add_info_row(details, 1, "话题", topic_var, bg="#ffffff")
        self._add_info_row(details, 2, "坐标系", frame_var, bg="#ffffff")
        self._add_info_row(details, 3, "收到消息", receive_count_var, bg="#ffffff")
        self._add_info_row(details, 4, "有效样本", sample_count_var, bg="#ffffff")
        self._add_info_row(details, 5, "估计接收率", rate_var, bg="#ffffff")
        self._add_info_row(details, 6, "接收距今", freshness_var, bg="#ffffff")
        self._add_info_row(details, 7, "源时间戳", stamp_var, bg="#ffffff")

        current_box = tk.LabelFrame(
            metrics,
            text="当前位置",
            font=("Sans", 12, "bold"),
            bg="#f8fafc",
            fg="#111827",
            padx=12,
            pady=10,
        )
        current_box.pack(fill="x", pady=(0, 8))
        self._add_big_value_row(current_box, 0, "X", current_x_var)
        self._add_big_value_row(current_box, 1, "Y", current_y_var)
        self._add_big_value_row(current_box, 2, "Z", current_z_var)

        relative_box = tk.LabelFrame(
            metrics,
            text="相对起点",
            font=("Sans", 12, "bold"),
            bg="#f8fafc",
            fg="#111827",
            padx=12,
            pady=10,
        )
        relative_box.pack(fill="x", pady=(0, 8))
        self._add_big_value_row(relative_box, 0, "ΔX", delta_x_var)
        self._add_big_value_row(relative_box, 1, "ΔY", delta_y_var)
        self._add_big_value_row(relative_box, 2, "ΔZ", delta_z_var)
        self._add_big_value_row(relative_box, 3, "总位移", distance_var)

        attitude_box = tk.LabelFrame(
            metrics,
            text="姿态",
            font=("Sans", 12, "bold"),
            bg="#f8fafc",
            fg="#111827",
            padx=12,
            pady=10,
        )
        attitude_box.pack(fill="x", pady=(0, 8))
        self._add_info_row(
            attitude_box,
            0,
            "姿态参考",
            orientation_frame_var,
            bg="#f8fafc",
            wraplength=720,
        )
        self._add_big_value_row(attitude_box, 1, "Roll", roll_var)
        self._add_big_value_row(attitude_box, 2, "Pitch", pitch_var)
        self._add_big_value_row(attitude_box, 3, "Yaw", yaw_var)
        self._add_info_row(
            attitude_box,
            4,
            "Quaternion",
            quaternion_var,
            bg="#f8fafc",
            wraplength=720,
            font=("Consolas", 11),
        )

        velocity_box = tk.LabelFrame(
            metrics,
            text="速度",
            font=("Sans", 12, "bold"),
            bg="#f8fafc",
            fg="#111827",
            padx=12,
            pady=10,
        )
        velocity_box.pack(fill="x", pady=(0, 8))
        self._add_info_row(
            velocity_box,
            0,
            "速度参考",
            velocity_frame_var,
            bg="#f8fafc",
            wraplength=720,
        )
        self._add_big_value_row(velocity_box, 1, "Vx", velocity_x_var)
        self._add_big_value_row(velocity_box, 2, "Vy", velocity_y_var)
        self._add_big_value_row(velocity_box, 3, "Vz", velocity_z_var)

        tk.Button(
            metrics,
            text="将当前点设为起点",
            font=("Sans", 11, "bold"),
            padx=12,
            pady=8,
            command=state.reset_origin,
        ).pack(fill="x")

        return frame

    def _create_status_frame(
        self,
        parent: tk.Widget,
        state: StatusSourceState,
    ) -> tk.LabelFrame:
        frame = tk.LabelFrame(
            parent,
            text=state.spec.source_label,
            font=("Sans", 13, "bold"),
            bg="#ffffff",
            fg="#111827",
            bd=2,
            padx=14,
            pady=12,
        )
        frame.grid_columnconfigure(1, weight=1)

        summary_var = tk.StringVar(value="等待数据")
        topic_var = tk.StringVar(
            value="\n".join(channel.topic_name for channel in state.spec.channels)
        )
        freshness_var = tk.StringVar(value="等待数据")
        receive_count_var = tk.StringVar(value="0")
        sample_count_var = tk.StringVar(value="0")
        details_var = tk.StringVar(value="等待数据")

        badge_label = tk.Label(
            frame,
            textvariable=summary_var,
            font=("Sans", 11, "bold"),
            padx=10,
            pady=6,
            bd=0,
        )
        badge_label.grid(row=0, column=1, sticky="w", pady=5, padx=(8, 0))
        self._set_badge_style(badge_label, "wait")

        tk.Label(
            frame,
            text="状态",
            anchor="w",
            font=("Sans", 11, "bold"),
            bg="#ffffff",
            fg="#374151",
            width=10,
        ).grid(row=0, column=0, sticky="nw", pady=5)

        self._add_info_row(frame, 1, "话题", topic_var, bg="#ffffff", wraplength=520)
        self._add_info_row(frame, 2, "接收距今", freshness_var, bg="#ffffff", wraplength=520)
        self._add_info_row(frame, 3, "收到消息", receive_count_var, bg="#ffffff", wraplength=520)
        self._add_info_row(frame, 4, "有效解析", sample_count_var, bg="#ffffff", wraplength=520)
        self._add_info_row(
            frame,
            5,
            "详情",
            details_var,
            bg="#ffffff",
            wraplength=520,
            font=("Consolas", 10),
        )

        self.status_widgets[state.spec.key] = StatusWidgets(
            summary_var=summary_var,
            topic_var=topic_var,
            freshness_var=freshness_var,
            receive_count_var=receive_count_var,
            sample_count_var=sample_count_var,
            details_var=details_var,
            badge_label=badge_label,
        )

        return frame

    def _add_info_row(
        self,
        parent: tk.Widget,
        row: int,
        label: str,
        variable: tk.StringVar,
        bg: str,
        wraplength: int = 480,
        font: Tuple[str, int] = ("Sans", 11),
    ):
        parent.grid_columnconfigure(1, weight=1)
        tk.Label(
            parent,
            text=label,
            anchor="w",
            font=("Sans", 11, "bold"),
            bg=bg,
            fg="#374151",
            width=10,
        ).grid(row=row, column=0, sticky="nw", pady=5)
        value_label = tk.Label(
            parent,
            textvariable=variable,
            justify="left",
            anchor="w",
            wraplength=wraplength,
            font=font,
            bg=bg,
            fg="#111827",
        )
        value_label.grid(row=row, column=1, sticky="nsew", pady=5, padx=(8, 0))
        value_label.bind(
            "<Configure>",
            lambda event, widget=value_label: self._update_wraplength(widget, event.width),
        )

    def _add_big_value_row(
        self,
        parent: tk.Widget,
        row: int,
        label: str,
        variable: tk.StringVar,
    ):
        parent.grid_columnconfigure(1, weight=1)
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
            justify="right",
            font=("Consolas", 16, "bold"),
            bg="#f8fafc",
            fg="#0f172a",
        ).grid(row=row, column=1, sticky="ew", pady=4, padx=(10, 0))

    def _set_badge_style(self, widget: tk.Label, level: str):
        style = LEVEL_STYLES.get(level, LEVEL_STYLES["wait"])
        widget.configure(bg=style["bg"], fg=style["fg"])

    def _toggle_source_subscription(self, source_key: str):
        enabled = self.source_enabled_vars[source_key].get()
        self.monitor_node.set_source_enabled(source_key, enabled)
        self._relayout_cards()

    @staticmethod
    def _update_wraplength(widget: tk.Label, width: int):
        target = max(int(width), 160)
        if int(float(widget.cget("wraplength"))) != target:
            widget.configure(wraplength=target)

    def _on_body_configure(self, _event: tk.Event):
        if self.body_canvas is not None:
            self.body_canvas.configure(scrollregion=self.body_canvas.bbox("all"))

    def _on_canvas_configure(self, event: tk.Event):
        if self.body_canvas is not None and self.body_window_id is not None:
            self.body_canvas.itemconfigure(self.body_window_id, width=event.width)
        self._schedule_relayout()

    def _schedule_relayout(self):
        if self._layout_after_id is not None:
            self.root.after_cancel(self._layout_after_id)
        self._layout_after_id = self.root.after(40, self._relayout_cards)

    def _relayout_cards(self):
        self._layout_after_id = None
        available_width = 0
        if self.body_canvas is not None:
            available_width = max(self.body_canvas.winfo_width() - 32, 0)

        if self.position_grid is not None and self.position_frames:
            position_columns = 2 if available_width >= 1380 else 1
            visible_position_frames = [
                frame
                for state in self.position_states
                if self.source_enabled_vars[state.spec.key].get()
                for frame in [self.position_frame_by_key[state.spec.key]]
            ]
            self._relayout_frame_grid(
                container=self.position_grid,
                frames=visible_position_frames,
                empty_label=self.position_empty_label,
                column_count=position_columns,
            )

        if self.status_grid is not None and self.status_frames:
            if available_width >= 1680:
                status_columns = 3
            elif available_width >= 1120:
                status_columns = 2
            else:
                status_columns = 1
            visible_status_frames = [
                frame
                for state in self.status_states
                if self.source_enabled_vars[state.spec.key].get()
                for frame in [self.status_frame_by_key[state.spec.key]]
            ]
            self._relayout_frame_grid(
                container=self.status_grid,
                frames=visible_status_frames,
                empty_label=self.status_empty_label,
                column_count=status_columns,
            )

    def _relayout_frame_grid(
        self,
        container: tk.Frame,
        frames: List[tk.LabelFrame],
        empty_label: Optional[tk.Label],
        column_count: int,
    ):
        column_count = max(1, column_count)
        managed_children = list(container.grid_slaves())
        for child in managed_children:
            child.grid_forget()

        if not frames:
            if empty_label is not None:
                empty_label.grid(row=0, column=0, sticky="w", padx=8, pady=8)
            return

        if empty_label is not None:
            empty_label.grid_forget()

        for column_index in range(max(column_count, 1)):
            container.grid_columnconfigure(column_index, weight=1)

        for frame_index, frame in enumerate(frames):
            row_index = frame_index // column_count
            column_index = frame_index % column_count
            frame.grid(
                row=row_index,
                column=column_index,
                sticky="nsew",
                padx=8,
                pady=8,
            )

    def _on_mousewheel(self, event: tk.Event):
        if self.body_canvas is None:
            return
        delta = 0
        if hasattr(event, "delta") and event.delta:
            delta = -1 * int(event.delta / 120)
        if delta != 0:
            self.body_canvas.yview_scroll(delta, "units")

    def _on_mousewheel_linux_up(self, _event: tk.Event):
        if self.body_canvas is not None:
            self.body_canvas.yview_scroll(-1, "units")

    def _on_mousewheel_linux_down(self, _event: tk.Event):
        if self.body_canvas is not None:
            self.body_canvas.yview_scroll(1, "units")

    def _reset_all_origins(self):
        for state in self.position_states:
            state.reset_origin()

    def _schedule_refresh(self):
        self._refresh_widgets()
        self.root.after(self.refresh_period_ms, self._schedule_refresh)

    def _refresh_widgets(self):
        now = time.monotonic()
        for state in self.position_states:
            self._refresh_position_card(state.snapshot(), now)
        for state in self.status_states:
            self._refresh_status_card(state.snapshot(now), now)

    def _refresh_position_card(self, snapshot: PositionSourceView, now: float):
        widgets = self.position_widgets[snapshot.spec.key]
        widgets.topic_var.set(snapshot.spec.topic_name)
        widgets.receive_count_var.set(str(snapshot.receive_count))
        widgets.sample_count_var.set(str(snapshot.sample_count))
        widgets.rate_var.set(format_rate(snapshot.estimated_rate_hz))

        age_s = age_since(snapshot.last_receive_wall_s, now)
        level, status_text = classify_stream_state(
            latest_available=snapshot.latest_sample is not None,
            age_s=age_s,
            slow_after_s=snapshot.spec.slow_after_s,
            stale_after_s=snapshot.spec.stale_after_s,
            last_error=snapshot.last_error,
        )

        widgets.status_var.set(status_text)
        self._set_badge_style(widgets.badge_label, level)

        if snapshot.latest_sample is None:
            if snapshot.receive_count > 0:
                widgets.frame_var.set("最近没有有效位置样本")
                widgets.freshness_var.set(
                    format_freshness(
                        age_s,
                        snapshot.spec.slow_after_s,
                        snapshot.spec.stale_after_s,
                    )
                )
            else:
                waited_s = max(now - snapshot.started_at_wall_s, 0.0)
                widgets.frame_var.set("--")
                widgets.freshness_var.set(f"等待 {format_age(waited_s)}")
            widgets.stamp_var.set("--")
            self._set_position_vars(widgets, None, None)
            self._set_orientation_vars(widgets, None)
            self._set_velocity_vars(widgets, None)
            return

        widgets.frame_var.set(snapshot.latest_sample.frame_label)
        widgets.freshness_var.set(
            format_freshness(age_s, snapshot.spec.slow_after_s, snapshot.spec.stale_after_s)
        )
        widgets.stamp_var.set(format_timestamp(snapshot.latest_sample.stamp_s))
        self._set_position_vars(widgets, snapshot.latest_sample, snapshot.first_sample)
        self._set_orientation_vars(widgets, snapshot.latest_sample)
        self._set_velocity_vars(widgets, snapshot.latest_sample)

    def _refresh_status_card(self, view: StatusSourceView, now: float):
        widgets = self.status_widgets[view.spec.key]
        widgets.summary_var.set(view.render.summary)
        self._set_badge_style(widgets.badge_label, view.render.level)
        widgets.topic_var.set("\n".join(channel.spec.topic_name for channel in view.channels.values()))
        widgets.freshness_var.set(format_status_freshness(view.channels, now))
        widgets.receive_count_var.set(format_status_receive_counts(view.channels))
        widgets.sample_count_var.set(format_status_sample_counts(view.channels))
        widgets.details_var.set(format_detail_rows(view.render.detail_rows))

    def _set_position_vars(
        self,
        widgets: PositionWidgets,
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

        widgets.current_x_var.set(format_distance(latest_sample.x))
        widgets.current_y_var.set(format_distance(latest_sample.y))
        widgets.current_z_var.set(format_distance(latest_sample.z))

        if not self.show_relative:
            widgets.delta_x_var.set("已关闭")
            widgets.delta_y_var.set("已关闭")
            widgets.delta_z_var.set("已关闭")
            widgets.distance_var.set("已关闭")
            return

        if first_sample is None:
            widgets.delta_x_var.set("-- m")
            widgets.delta_y_var.set("-- m")
            widgets.delta_z_var.set("-- m")
            widgets.distance_var.set("-- m")
            return

        dx = latest_sample.x - first_sample.x
        dy = latest_sample.y - first_sample.y
        dz = latest_sample.z - first_sample.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        widgets.delta_x_var.set(format_distance(dx))
        widgets.delta_y_var.set(format_distance(dy))
        widgets.delta_z_var.set(format_distance(dz))
        widgets.distance_var.set(format_distance(distance))

    def _set_orientation_vars(
        self,
        widgets: PositionWidgets,
        latest_sample: Optional[PositionSample],
    ):
        if latest_sample is None or latest_sample.rpy_rad is None:
            widgets.orientation_frame_var.set(
                latest_sample.orientation_frame_label if latest_sample is not None else "--"
            )
            widgets.roll_var.set("N/A")
            widgets.pitch_var.set("N/A")
            widgets.yaw_var.set("N/A")
            widgets.quaternion_var.set(
                format_quaternion(latest_sample.orientation_xyzw)
                if latest_sample is not None
                else "N/A"
            )
            return

        roll, pitch, yaw = latest_sample.rpy_rad
        widgets.orientation_frame_var.set(latest_sample.orientation_frame_label)
        widgets.roll_var.set(format_angle_deg(roll))
        widgets.pitch_var.set(format_angle_deg(pitch))
        widgets.yaw_var.set(format_angle_deg(yaw))
        widgets.quaternion_var.set(format_quaternion(latest_sample.orientation_xyzw))

    def _set_velocity_vars(
        self,
        widgets: PositionWidgets,
        latest_sample: Optional[PositionSample],
    ):
        if latest_sample is None or latest_sample.velocity_xyz is None:
            widgets.velocity_frame_var.set(
                latest_sample.velocity_frame_label if latest_sample is not None else "--"
            )
            widgets.velocity_x_var.set("N/A")
            widgets.velocity_y_var.set("N/A")
            widgets.velocity_z_var.set("N/A")
            return

        vx, vy, vz = latest_sample.velocity_xyz
        widgets.velocity_frame_var.set(latest_sample.velocity_frame_label)
        widgets.velocity_x_var.set(format_plain_float(vx, unit="m/s", signed=True))
        widgets.velocity_y_var.set(format_plain_float(vy, unit="m/s", signed=True))
        widgets.velocity_z_var.set(format_plain_float(vz, unit="m/s", signed=True))


def age_since(last_time_s: Optional[float], now_s: float) -> Optional[float]:
    if last_time_s is None:
        return None
    return max(now_s - last_time_s, 0.0)


def safe_float(value: Any) -> Optional[float]:
    try:
        converted = float(value)
    except (TypeError, ValueError):
        return None
    if math.isfinite(converted):
        return converted
    return None


def safe_int(value: Any) -> Optional[int]:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def safe_bool(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if value in (0, 1):
        return bool(value)
    return None


def safe_text(value: Any) -> str:
    if value is None:
        return ""
    return str(value)


def safe_sequence_float(sequence: Any, index: int) -> Optional[float]:
    try:
        return safe_float(sequence[index])
    except (IndexError, TypeError):
        return None


def extract_ros_stamp_seconds(stamp: Any) -> Optional[float]:
    sec = safe_int(getattr(stamp, "sec", None))
    nanosec = safe_int(getattr(stamp, "nanosec", None))
    if sec is None or nanosec is None:
        return None
    return float(sec) + float(nanosec) * 1e-9


def extract_px4_stamp_seconds(msg: Any) -> Optional[float]:
    timestamp_sample = safe_int(getattr(msg, "timestamp_sample", None))
    if timestamp_sample is not None and timestamp_sample > 0:
        return float(timestamp_sample) * 1e-6
    timestamp = safe_int(getattr(msg, "timestamp", None))
    if timestamp is not None and timestamp > 0:
        return float(timestamp) * 1e-6
    return None


def clamp_unit(value: float) -> float:
    return max(-1.0, min(1.0, value))


def normalized_xyzw_quaternion(
    x: Optional[float],
    y: Optional[float],
    z: Optional[float],
    w: Optional[float],
) -> Optional[Tuple[float, float, float, float]]:
    components = (x, y, z, w)
    if any(component is None or not math.isfinite(component) for component in components):
        return None
    norm = math.sqrt(sum(component * component for component in components))
    if norm <= 1.0e-12:
        return None
    return tuple(component / norm for component in components)


def ros_quaternion_from_msg(msg: Any) -> Optional[Tuple[float, float, float, float]]:
    return normalized_xyzw_quaternion(
        safe_float(getattr(msg, "x", None)),
        safe_float(getattr(msg, "y", None)),
        safe_float(getattr(msg, "z", None)),
        safe_float(getattr(msg, "w", None)),
    )


def px4_quaternion_array_to_xyzw(
    q: Any,
) -> Optional[Tuple[float, float, float, float]]:
    return normalized_xyzw_quaternion(
        safe_sequence_float(q, 1),
        safe_sequence_float(q, 2),
        safe_sequence_float(q, 3),
        safe_sequence_float(q, 0),
    )


def quaternion_xyzw_to_rpy(
    quaternion_xyzw: Optional[Tuple[float, float, float, float]],
) -> Optional[Tuple[float, float, float]]:
    if quaternion_xyzw is None:
        return None
    x, y, z, w = quaternion_xyzw
    sinr_cosp = 2.0 * ((w * x) + (y * z))
    cosr_cosp = 1.0 - (2.0 * ((x * x) + (y * y)))
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * ((w * y) - (z * x))
    pitch = math.asin(clamp_unit(sinp))

    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quaternion_xyzw_to_matrix(
    quaternion_xyzw: Optional[Tuple[float, float, float, float]],
) -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]]:
    if quaternion_xyzw is None:
        return None
    x, y, z, w = quaternion_xyzw
    return (
        (
            1.0 - (2.0 * ((y * y) + (z * z))),
            2.0 * ((x * y) - (z * w)),
            2.0 * ((x * z) + (y * w)),
        ),
        (
            2.0 * ((x * y) + (z * w)),
            1.0 - (2.0 * ((x * x) + (z * z))),
            2.0 * ((y * z) - (x * w)),
        ),
        (
            2.0 * ((x * z) - (y * w)),
            2.0 * ((y * z) + (x * w)),
            1.0 - (2.0 * ((x * x) + (y * y))),
        ),
    )


def matrix_multiply(
    lhs: Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]],
    rhs: Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]],
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]:
    return tuple(
        tuple(sum(lhs[row][k] * rhs[k][col] for k in range(3)) for col in range(3))
        for row in range(3)
    )


def quaternion_xyzw_from_matrix(
    matrix: Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]],
) -> Optional[Tuple[float, float, float, float]]:
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2][1] - matrix[1][2]) / scale
        y = (matrix[0][2] - matrix[2][0]) / scale
        z = (matrix[1][0] - matrix[0][1]) / scale
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        w = (matrix[2][1] - matrix[1][2]) / scale
        x = 0.25 * scale
        y = (matrix[0][1] + matrix[1][0]) / scale
        z = (matrix[0][2] + matrix[2][0]) / scale
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        w = (matrix[0][2] - matrix[2][0]) / scale
        x = (matrix[0][1] + matrix[1][0]) / scale
        y = 0.25 * scale
        z = (matrix[1][2] + matrix[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        w = (matrix[1][0] - matrix[0][1]) / scale
        x = (matrix[0][2] + matrix[2][0]) / scale
        y = (matrix[1][2] + matrix[2][1]) / scale
        z = 0.25 * scale
    return normalized_xyzw_quaternion(x, y, z, w)


def convert_px4_quaternion_to_ros_xyzw(
    q: Any,
    pose_frame: Optional[int],
) -> Tuple[Optional[Tuple[float, float, float, float]], str]:
    quaternion_xyzw = px4_quaternion_array_to_xyzw(q)
    if quaternion_xyzw is None:
        return None, "PX4 姿态无效"

    rotation = quaternion_xyzw_to_matrix(quaternion_xyzw)
    if rotation is None:
        return None, "PX4 姿态无效"

    flu_to_frd = (
        (1.0, 0.0, 0.0),
        (0.0, -1.0, 0.0),
        (0.0, 0.0, -1.0),
    )

    if pose_frame == 1:
        ned_to_enu = (
            (0.0, 1.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 0.0, -1.0),
        )
        converted = matrix_multiply(matrix_multiply(ned_to_enu, rotation), flu_to_frd)
        return quaternion_xyzw_from_matrix(converted), "ROS ENU/FLU（由 PX4 NED/FRD 转换）"

    if pose_frame == 2:
        frd_to_flu = flu_to_frd
        converted = matrix_multiply(matrix_multiply(frd_to_flu, rotation), flu_to_frd)
        return quaternion_xyzw_from_matrix(converted), "ROS FLU/FLU（由 PX4 FRD/FRD 转换）"

    return quaternion_xyzw, f"PX4 原始姿态（{describe_px4_pose_frame(pose_frame or 0)}）"


def describe_px4_velocity_frame(velocity_frame: int) -> str:
    mapping = {
        0: "PX4 未知速度坐标系",
        1: "PX4 NED 速度",
        2: "PX4 FRD 速度",
        3: "PX4 BODY_FRD 速度",
    }
    return mapping.get(velocity_frame, f"PX4 未知 velocity_frame={velocity_frame}")


def convert_px4_velocity_to_ros_xyz(
    x: Optional[float],
    y: Optional[float],
    z: Optional[float],
    velocity_frame: Optional[int],
) -> Tuple[Optional[Tuple[float, float, float]], str]:
    if x is None or y is None or z is None:
        return None, "PX4 速度无效"
    if velocity_frame == 1:
        return (y, x, -z), "ROS ENU 世界速度（由 PX4 NED 转换）"
    if velocity_frame == 2:
        return (x, -y, -z), "ROS FLU 世界速度（由 PX4 FRD 转换）"
    if velocity_frame == 3:
        return (x, -y, -z), "ROS body FLU 速度（由 PX4 BODY_FRD 转换）"
    return (x, y, z), f"PX4 原始速度（{describe_px4_velocity_frame(velocity_frame or 0)}）"


def format_quaternion(
    quaternion_xyzw: Optional[Tuple[float, float, float, float]],
) -> str:
    if quaternion_xyzw is None:
        return "N/A"
    x, y, z, w = quaternion_xyzw
    return f"[{x:+.3f}, {y:+.3f}, {z:+.3f}, {w:+.3f}]"


def format_bool(value: Optional[bool]) -> str:
    if value is None:
        return "N/A"
    return "true" if value else "false"


def format_rate(rate_hz: Optional[float]) -> str:
    if rate_hz is None or not math.isfinite(rate_hz):
        return "--"
    if rate_hz >= 20.0:
        return f"{rate_hz:.1f} Hz"
    if rate_hz >= 1.0:
        return f"{rate_hz:.2f} Hz"
    return f"{rate_hz:.3f} Hz"


def format_age(age_s: Optional[float]) -> str:
    if age_s is None:
        return "--"
    if age_s < 1.0:
        return f"{age_s * 1000.0:.0f} ms"
    if age_s < 60.0:
        return f"{age_s:.2f} s"
    return f"{age_s / 60.0:.1f} min"


def format_distance(value: Optional[float]) -> str:
    if value is None or not math.isfinite(value):
        return "N/A"
    return f"{value:+.3f} m"


def format_plain_float(
    value: Optional[float],
    unit: str = "",
    precision: int = 3,
    signed: bool = False,
) -> str:
    if value is None or not math.isfinite(value):
        return "N/A"
    pattern = f"{{:{'+' if signed else ''}.{precision}f}}"
    rendered = pattern.format(value)
    return f"{rendered} {unit}".rstrip()


def format_angle_deg(value_rad: Optional[float]) -> str:
    if value_rad is None:
        return "N/A"
    return format_plain_float(math.degrees(value_rad), unit="deg", precision=2, signed=True)


def format_timestamp(stamp_s: Optional[float]) -> str:
    if stamp_s is None:
        return "--"
    return f"{stamp_s:.3f} s"


def classify_stream_state(
    latest_available: bool,
    age_s: Optional[float],
    slow_after_s: float,
    stale_after_s: float,
    last_error: Optional[str],
) -> Tuple[str, str]:
    if not latest_available:
        if last_error:
            return "error", "等待数据 / 最近解析失败"
        return "wait", "等待数据"
    if last_error:
        return "warn", "接收中 / 最近解析失败"
    if age_s is None:
        return "wait", "等待数据"
    if age_s < slow_after_s:
        return "ok", "正常接收"
    if age_s < stale_after_s:
        return "warn", "数据变慢"
    return "error", "数据陈旧"


def format_freshness(age_s: Optional[float], slow_after_s: float, stale_after_s: float) -> str:
    if age_s is None:
        return "等待数据"
    if age_s < slow_after_s:
        prefix = "fresh"
    elif age_s < stale_after_s:
        prefix = "slow"
    else:
        prefix = "stale"
    return f"{prefix} / {format_age(age_s)}"


def format_detail_rows(rows: List[Tuple[str, str]]) -> str:
    if not rows:
        return "--"
    width = max(len(label) for label, _ in rows)
    return "\n".join(f"{label:<{width}} : {value}" for label, value in rows)


def format_status_freshness(
    channels: Dict[str, StatusChannelSnapshot],
    now_wall_s: float,
) -> str:
    parts = []
    for channel in channels.values():
        parts.append(f"{channel.spec.label}={format_channel_freshness(channel, now_wall_s)}")
    return " | ".join(parts)


def format_status_sample_counts(channels: Dict[str, StatusChannelSnapshot]) -> str:
    parts = []
    for channel in channels.values():
        rate_part = format_rate(channel.estimated_rate_hz)
        parts.append(f"{channel.spec.label}={channel.sample_count} ({rate_part})")
    return " | ".join(parts)


def format_status_receive_counts(channels: Dict[str, StatusChannelSnapshot]) -> str:
    parts = []
    for channel in channels.values():
        parts.append(f"{channel.spec.label}={channel.receive_count}")
    return " | ".join(parts)


def format_channel_freshness(
    channel: StatusChannelSnapshot,
    now_wall_s: float,
) -> str:
    age_s = age_since(channel.last_receive_wall_s, now_wall_s)
    if age_s is None:
        return "等待数据"
    if not channel.spec.expect_continuous_updates:
        return f"latched / {format_age(age_s)}"
    return format_freshness(age_s, channel.spec.slow_after_s, channel.spec.stale_after_s)


def describe_ros_odometry_frame(frame_id: str, child_frame_id: str) -> str:
    if frame_id and child_frame_id:
        return f"ROS Odom（{frame_id} -> {child_frame_id}）"
    if frame_id:
        return f"ROS frame_id={frame_id}"
    if child_frame_id:
        return f"ROS child_frame_id={child_frame_id}"
    return "ROS Odom（未设置 frame_id / child_frame_id）"


def describe_px4_pose_frame(pose_frame: int) -> str:
    mapping = {
        0: "PX4 未知坐标系",
        1: "PX4 NED 坐标系",
        2: "PX4 FRD 坐标系",
    }
    return mapping.get(pose_frame, f"PX4 未知 pose_frame={pose_frame}")


def convert_px4_position_to_ros_xyz(
    x: float,
    y: float,
    z: float,
    pose_frame: int,
) -> Tuple[float, float, float, str]:
    if pose_frame == 1:
        return y, x, -z, "ROS ENU（由 PX4 NED 转换）"
    if pose_frame == 2:
        return x, -y, -z, "ROS FLU（由 PX4 FRD 转换）"
    return x, y, z, f"ROS 默认方向显示（原始: {describe_px4_pose_frame(pose_frame)})"


def make_ros_odometry_spec(key: str, source_label: str, topic_name: str) -> PositionSourceSpec:
    try:
        from nav_msgs.msg import Odometry
    except ImportError as exc:
        raise RuntimeError("当前终端没有 nav_msgs。请先 source 含有 nav_msgs 的工作区。") from exc

    def extract(msg: Any) -> PositionSample:
        x = safe_float(getattr(msg.pose.pose.position, "x", None))
        y = safe_float(getattr(msg.pose.pose.position, "y", None))
        z = safe_float(getattr(msg.pose.pose.position, "z", None))
        if x is None or y is None or z is None:
            raise ValueError("Odometry 位姿里存在非有限值。")
        orientation_xyzw = ros_quaternion_from_msg(getattr(msg.pose.pose, "orientation", None))
        rpy_rad = quaternion_xyzw_to_rpy(orientation_xyzw)
        velocity_xyz = (
            safe_float(getattr(msg.twist.twist.linear, "x", None)),
            safe_float(getattr(msg.twist.twist.linear, "y", None)),
            safe_float(getattr(msg.twist.twist.linear, "z", None)),
        )
        if any(component is None for component in velocity_xyz):
            velocity_xyz = None
        return PositionSample(
            x=x,
            y=y,
            z=z,
            stamp_s=extract_ros_stamp_seconds(msg.header.stamp),
            frame_label=describe_ros_odometry_frame(msg.header.frame_id, msg.child_frame_id),
            orientation_xyzw=orientation_xyzw,
            rpy_rad=rpy_rad,
            orientation_frame_label=(
                f"ROS pose（{msg.child_frame_id or '?'} -> {msg.header.frame_id or '?'}）"
            ),
            velocity_xyz=velocity_xyz,
            velocity_frame_label=(
                f"ROS twist（child={msg.child_frame_id or '未设置'}）"
            ),
        )

    return PositionSourceSpec(
        key=key,
        source_label=source_label,
        topic_name=topic_name,
        message_type=Odometry,
        extractor=extract,
        qos_profile=FAST_TOPIC_QOS,
    )


def make_px4_position_spec(key: str, source_label: str, topic_name: str) -> PositionSourceSpec:
    try:
        from px4_msgs.msg import VehicleOdometry
    except ImportError as exc:
        raise RuntimeError("当前终端没有 px4_msgs。请先 source 含有 px4_msgs 的工作区。") from exc

    def extract(msg: Any) -> PositionSample:
        x = safe_sequence_float(getattr(msg, "position", None), 0)
        y = safe_sequence_float(getattr(msg, "position", None), 1)
        z = safe_sequence_float(getattr(msg, "position", None), 2)
        pose_frame = safe_int(getattr(msg, "pose_frame", None))
        velocity_frame = safe_int(getattr(msg, "velocity_frame", None))
        if x is None or y is None or z is None:
            raise ValueError("VehicleOdometry position 存在非有限值。")
        if pose_frame is None:
            pose_frame = 0
        ros_x, ros_y, ros_z, frame_label = convert_px4_position_to_ros_xyz(
            x,
            y,
            z,
            pose_frame,
        )
        orientation_xyzw, orientation_frame_label = convert_px4_quaternion_to_ros_xyzw(
            getattr(msg, "q", None),
            pose_frame,
        )
        rpy_rad = quaternion_xyzw_to_rpy(orientation_xyzw)
        velocity_xyz, velocity_frame_label = convert_px4_velocity_to_ros_xyz(
            safe_sequence_float(getattr(msg, "velocity", None), 0),
            safe_sequence_float(getattr(msg, "velocity", None), 1),
            safe_sequence_float(getattr(msg, "velocity", None), 2),
            velocity_frame,
        )
        return PositionSample(
            x=ros_x,
            y=ros_y,
            z=ros_z,
            stamp_s=extract_px4_stamp_seconds(msg),
            frame_label=frame_label,
            orientation_xyzw=orientation_xyzw,
            rpy_rad=rpy_rad,
            orientation_frame_label=orientation_frame_label,
            velocity_xyz=velocity_xyz,
            velocity_frame_label=velocity_frame_label,
        )

    return PositionSourceSpec(
        key=key,
        source_label=source_label,
        topic_name=topic_name,
        message_type=VehicleOdometry,
        extractor=extract,
        qos_profile=FAST_TOPIC_QOS,
    )


def make_ov_guard_status_spec(health_topic: str, fault_topic: str) -> StatusSourceSpec:
    try:
        from std_msgs.msg import Bool, String
    except ImportError as exc:
        raise RuntimeError("当前终端没有 std_msgs。请先 source ROS 环境。") from exc

    def render(channels: Dict[str, StatusChannelSnapshot], now_wall_s: float) -> StatusRender:
        health_channel = channels["health_ok"]
        fault_channel = channels["fault_reason"]

        health_value = safe_bool(health_channel.payload)
        fault_reason = safe_text(fault_channel.payload).strip()
        health_age_s = age_since(health_channel.last_receive_wall_s, now_wall_s)
        fault_age_s = age_since(fault_channel.last_receive_wall_s, now_wall_s)
        change_candidates = [
            age_since(health_channel.last_change_wall_s, now_wall_s),
            age_since(fault_channel.last_change_wall_s, now_wall_s),
        ]
        change_candidates = [value for value in change_candidates if value is not None]
        latest_change_age = min(change_candidates) if change_candidates else None

        if health_value is None:
            level = "wait"
            summary = "等待 health_ok"
        elif not health_value:
            level = "error"
            summary = "OV GUARD UNHEALTHY"
        else:
            level = "ok"
            summary = "OV GUARD HEALTHY"

        detail_rows = [
            ("health_ok", format_bool(health_value)),
            ("fault_reason", fault_reason if fault_reason else ("unknown" if health_value is False else "--")),
            (
                "recent_change",
                format_age(latest_change_age) if latest_change_age is not None else "--",
            ),
            (
                "health_freshness",
                format_channel_freshness(health_channel, now_wall_s),
            ),
            (
                "fault_freshness",
                format_channel_freshness(fault_channel, now_wall_s),
            ),
        ]

        if health_channel.last_error:
            detail_rows.append(("health_error", health_channel.last_error))
        if fault_channel.last_error:
            detail_rows.append(("fault_error", fault_channel.last_error))

        return StatusRender(level=level, summary=summary, detail_rows=detail_rows)

    return StatusSourceSpec(
        key="ov_guard",
        source_label="OV guard",
        channels=[
            StatusChannelSpec(
                key="health_ok",
                label="health_ok",
                topic_name=health_topic,
                message_type=Bool,
                extractor=lambda msg: safe_bool(getattr(msg, "data", None)),
                qos_profile=STATUS_LATCHED_QOS,
                slow_after_s=1.0,
                stale_after_s=15.0,
                expect_continuous_updates=False,
            ),
            StatusChannelSpec(
                key="fault_reason",
                label="fault_reason",
                topic_name=fault_topic,
                message_type=String,
                extractor=lambda msg: safe_text(getattr(msg, "data", "")),
                qos_profile=STATUS_LATCHED_QOS,
                slow_after_s=1.0,
                stale_after_s=15.0,
                expect_continuous_updates=False,
            ),
        ],
        renderer=render,
    )


def make_px4_local_status_spec(topic_name: str) -> StatusSourceSpec:
    try:
        from px4_msgs.msg import VehicleLocalPosition
    except ImportError as exc:
        raise RuntimeError("当前终端没有 px4_msgs。请先 source 含有 px4_msgs 的工作区。") from exc

    def extract(msg: Any) -> Dict[str, Any]:
        return {
            "stamp_s": extract_px4_stamp_seconds(msg),
            "xy_valid": safe_bool(getattr(msg, "xy_valid", None)),
            "z_valid": safe_bool(getattr(msg, "z_valid", None)),
            "v_xy_valid": safe_bool(getattr(msg, "v_xy_valid", None)),
            "v_z_valid": safe_bool(getattr(msg, "v_z_valid", None)),
            "dist_bottom_valid": safe_bool(getattr(msg, "dist_bottom_valid", None)),
            "heading_good_for_control": safe_bool(
                getattr(msg, "heading_good_for_control", None)
            ),
            "dead_reckoning": safe_bool(getattr(msg, "dead_reckoning", None)),
            "eph": safe_float(getattr(msg, "eph", None)),
            "epv": safe_float(getattr(msg, "epv", None)),
            "evh": safe_float(getattr(msg, "evh", None)),
            "xy_reset_counter": safe_int(getattr(msg, "xy_reset_counter", None)),
            "z_reset_counter": safe_int(getattr(msg, "z_reset_counter", None)),
            "heading_reset_counter": safe_int(getattr(msg, "heading_reset_counter", None)),
            "delta_xy_x": safe_sequence_float(getattr(msg, "delta_xy", None), 0),
            "delta_xy_y": safe_sequence_float(getattr(msg, "delta_xy", None), 1),
            "delta_z": safe_float(getattr(msg, "delta_z", None)),
            "delta_heading": safe_float(getattr(msg, "delta_heading", None)),
        }

    def render(channels: Dict[str, StatusChannelSnapshot], now_wall_s: float) -> StatusRender:
        local_channel = channels["px4_local"]
        data = local_channel.payload if isinstance(local_channel.payload, dict) else {}
        age_s = age_since(local_channel.last_receive_wall_s, now_wall_s)

        xy_valid = data.get("xy_valid")
        z_valid = data.get("z_valid")
        v_xy_valid = data.get("v_xy_valid")
        v_z_valid = data.get("v_z_valid")
        heading_good = data.get("heading_good_for_control")
        dead_reckoning = data.get("dead_reckoning")
        core_values = [xy_valid, z_valid, v_xy_valid, v_z_valid, heading_good]

        if not data:
            level = "wait"
            summary = "等待 VehicleLocalPosition"
        elif age_s is not None and age_s >= local_channel.spec.stale_after_s:
            level = "error"
            summary = "PX4 LOCAL STALE"
        elif any(value is None for value in core_values):
            level = "warn"
            summary = "PX4 LOCAL PARTIAL"
        elif dead_reckoning is True:
            level = "warn"
            summary = "PX4 LOCAL DEAD_RECKONING"
        elif False in (xy_valid, z_valid, v_xy_valid, v_z_valid) or heading_good is False:
            level = "warn"
            summary = "PX4 LOCAL DEGRADED"
        else:
            level = "ok"
            summary = "PX4 LOCAL READY"

        delta_xy_x = data.get("delta_xy_x")
        delta_xy_y = data.get("delta_xy_y")
        delta_xy_norm = None
        if delta_xy_x is not None and delta_xy_y is not None:
            delta_xy_norm = math.hypot(delta_xy_x, delta_xy_y)

        detail_rows = [
            ("xy_valid / z_valid", f"{format_bool(xy_valid)} / {format_bool(z_valid)}"),
            (
                "v_xy_valid / v_z_valid",
                f"{format_bool(v_xy_valid)} / {format_bool(v_z_valid)}",
            ),
            ("dist_bottom_valid", format_bool(data.get("dist_bottom_valid"))),
            ("heading_good", format_bool(heading_good)),
            ("dead_reckoning", format_bool(dead_reckoning)),
            (
                "eph / epv / evh",
                " / ".join(
                    [
                        format_plain_float(data.get("eph"), unit="m"),
                        format_plain_float(data.get("epv"), unit="m"),
                        format_plain_float(data.get("evh"), unit="m/s"),
                    ]
                ),
            ),
            (
                "xy_reset",
                f"{value_or_na(data.get('xy_reset_counter'))} | "
                f"dx={format_plain_float(delta_xy_x, unit='m', signed=True)} "
                f"dy={format_plain_float(delta_xy_y, unit='m', signed=True)} "
                f"| {format_plain_float(delta_xy_norm, unit='m')}",
            ),
            (
                "z_reset",
                f"{value_or_na(data.get('z_reset_counter'))} | "
                f"dz={format_plain_float(data.get('delta_z'), unit='m', signed=True)}",
            ),
            (
                "heading_reset",
                f"{value_or_na(data.get('heading_reset_counter'))} | "
                f"dpsi={format_angle_deg(data.get('delta_heading'))}",
            ),
            ("timestamp", format_timestamp(data.get("stamp_s"))),
        ]

        if local_channel.last_error:
            detail_rows.append(("extract_error", local_channel.last_error))

        return StatusRender(level=level, summary=summary, detail_rows=detail_rows)

    return StatusSourceSpec(
        key="px4_local",
        source_label="PX4 local validity",
        channels=[
            StatusChannelSpec(
                key="px4_local",
                label="local",
                topic_name=topic_name,
                message_type=VehicleLocalPosition,
                extractor=extract,
                qos_profile=STATUS_STREAM_QOS,
                slow_after_s=0.5,
                stale_after_s=2.0,
            )
        ],
        renderer=render,
    )


def make_px4_status_spec(topic_name: str) -> StatusSourceSpec:
    try:
        from px4_msgs.msg import VehicleStatus
    except ImportError as exc:
        raise RuntimeError("当前终端没有 px4_msgs。请先 source 含有 px4_msgs 的工作区。") from exc

    def extract(msg: Any) -> Dict[str, Any]:
        arming_state = safe_int(getattr(msg, "arming_state", None))
        nav_state = safe_int(getattr(msg, "nav_state", None))
        return {
            "stamp_s": extract_px4_stamp_seconds(msg),
            "arming_state": arming_state,
            "arming_label": vehicle_arming_label(arming_state),
            "nav_state": nav_state,
            "nav_label": vehicle_nav_label(nav_state),
            "failsafe": safe_bool(getattr(msg, "failsafe", None)),
            "pre_flight_checks_pass": safe_bool(
                getattr(msg, "pre_flight_checks_pass", None)
            ),
        }

    def render(channels: Dict[str, StatusChannelSnapshot], now_wall_s: float) -> StatusRender:
        status_channel = channels["vehicle_status"]
        data = status_channel.payload if isinstance(status_channel.payload, dict) else {}
        age_s = age_since(status_channel.last_receive_wall_s, now_wall_s)
        arming_state = data.get("arming_state")
        nav_state = data.get("nav_state")

        if not data:
            level = "wait"
            summary = "等待 VehicleStatus"
        elif arming_state is None or nav_state is None:
            level = "warn"
            summary = "PX4 STATUS PARTIAL"
        else:
            mode_summary = f"{data.get('arming_label', 'N/A')} | {data.get('nav_label', 'N/A')}"
            if data.get("failsafe") is True:
                level = "error"
                summary = f"FAILSAFE | {mode_summary}"
            elif age_s is not None and age_s >= status_channel.spec.stale_after_s:
                level = "warn"
                summary = f"STALE | {mode_summary}"
            else:
                level = "ok"
                summary = mode_summary

        detail_rows = [
            (
                "arming_state",
                f"{data.get('arming_label', 'N/A')} ({value_or_na(data.get('arming_state'))})",
            ),
            (
                "nav_state",
                f"{data.get('nav_label', 'N/A')} ({value_or_na(data.get('nav_state'))})",
            ),
            ("failsafe", format_bool(data.get("failsafe"))),
            ("preflight_checks", format_bool(data.get("pre_flight_checks_pass"))),
            ("timestamp", format_timestamp(data.get("stamp_s"))),
        ]

        if status_channel.last_error:
            detail_rows.append(("extract_error", status_channel.last_error))

        return StatusRender(level=level, summary=summary, detail_rows=detail_rows)

    return StatusSourceSpec(
        key="px4_status",
        source_label="PX4 flight status",
        channels=[
            StatusChannelSpec(
                key="vehicle_status",
                label="status",
                topic_name=topic_name,
                message_type=VehicleStatus,
                extractor=extract,
                qos_profile=STATUS_STREAM_QOS,
                slow_after_s=0.5,
                stale_after_s=2.0,
            )
        ],
        renderer=render,
    )


def value_or_na(value: Any) -> str:
    return "N/A" if value is None else str(value)


def vehicle_arming_label(arming_state: Optional[int]) -> str:
    mapping = {
        0: "INIT",
        1: "DISARMED",
        2: "ARMED",
        3: "STANDBY_ERROR",
        4: "SHUTDOWN",
        5: "IN_AIR_RESTORE",
        6: "MAX",
    }
    if arming_state is None:
        return "N/A"
    return mapping.get(arming_state, f"ARMING_{arming_state}")


def vehicle_nav_label(nav_state: Optional[int]) -> str:
    mapping = {
        0: "MANUAL",
        1: "ALTCTL",
        2: "POSCTL",
        3: "AUTO_MISSION",
        4: "AUTO_LOITER",
        5: "AUTO_RTL",
        6: "AUTO_RC_RECOVER",
        7: "AUTO_RTGS",
        8: "AUTO_LANDENGFAIL",
        9: "AUTO_LANDGPSFAIL",
        10: "ACRO",
        11: "DESCEND",
        12: "TERMINATION",
        13: "OFFBOARD_OLD",
        14: "OFFBOARD",
        15: "STAB",
        16: "RATTITUDE",
        17: "AUTO_TAKEOFF",
        18: "AUTO_LAND",
        19: "AUTO_FOLLOW_TARGET",
        20: "AUTO_PRECLAND",
        21: "ORBIT",
        22: "AUTO_VTOL_TAKEOFF",
        23: "EXTERNAL1",
        24: "EXTERNAL2",
        25: "EXTERNAL3",
        26: "EXTERNAL4",
        27: "EXTERNAL5",
        28: "EXTERNAL6",
        29: "EXTERNAL7",
        30: "EXTERNAL8",
    }
    if nav_state is None:
        return "N/A"
    return mapping.get(nav_state, f"NAV_{nav_state}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="GUI 方式低开销监视位置/姿态/速度全链路、OV guard 与 PX4 状态。"
    )
    parser.add_argument(
        "--sources",
        default="openvins,uav_state,px4_in,px4_out,ov_guard,px4_local,px4_status",
        help=(
            "要显示的数据源。默认: "
            "openvins,uav_state,px4_in,px4_out,ov_guard,px4_local,px4_status"
        ),
    )
    parser.add_argument(
        "--source",
        default=None,
        help=(
            "兼容旧用法。若设置，则只显示单一路数据源。"
            "可选: openvins / uav_state / state / px4_in / px4_out / px4 / "
            "ov_guard / px4_local / px4_status"
        ),
    )
    parser.add_argument(
        "--topic",
        default=None,
        help="只在单一路位置类模式下使用：覆盖该数据源的话题名。",
    )
    parser.add_argument(
        "--openvins-topic",
        default="/ov_msckf/odomimu",
        help="OpenVINS 原始里程计话题。",
    )
    parser.add_argument(
        "--uav-state-topic",
        default="/uav/state/odometry_px4",
        help="当前 UAV 融合状态里程计话题。",
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
        "--ov-health-topic",
        default="/uav/ov/bridge/health_ok",
        help="OV guard health topic。",
    )
    parser.add_argument(
        "--ov-fault-topic",
        default="/uav/ov/bridge/fault_reason",
        help="OV guard fault reason topic。",
    )
    parser.add_argument(
        "--px4-local-position-topic",
        default="/fmu/out/vehicle_local_position",
        help="PX4 VehicleLocalPosition topic。",
    )
    parser.add_argument(
        "--vehicle-status-topic",
        default="/fmu/out/vehicle_status",
        help="PX4 VehicleStatus topic。",
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
        help="不显示位置卡的相对起点位移。",
    )
    return parser.parse_args()


def normalize_source_keys(args: argparse.Namespace) -> List[str]:
    alias_map = {
        "px4": "px4_out",
        "state": "uav_state",
    }

    raw_items = [args.source] if args.source else args.sources.split(",")
    normalized: List[str] = []
    seen = set()
    for item in raw_items:
        key = item.strip()
        if not key:
            continue
        key = alias_map.get(key, key)
        if key not in seen:
            normalized.append(key)
            seen.add(key)
    if not normalized:
        raise RuntimeError("没有可显示的数据源。")
    return normalized


def build_sources(
    args: argparse.Namespace,
) -> Tuple[List[PositionSourceState], List[StatusSourceState]]:
    source_keys = normalize_source_keys(args)

    position_builders = {
        "openvins": lambda topic: make_ros_odometry_spec("openvins", "OpenVINS 原始", topic),
        "uav_state": lambda topic: make_ros_odometry_spec("uav_state", "当前 UAV 融合状态", topic),
        "px4_in": lambda topic: make_px4_position_spec("px4_in", "发给 PX4 的 VehicleOdometry", topic),
        "px4_out": lambda topic: make_px4_position_spec("px4_out", "PX4 输出 VehicleOdometry", topic),
    }
    position_default_topics = {
        "openvins": args.openvins_topic,
        "uav_state": args.uav_state_topic,
        "px4_in": args.px4_input_topic,
        "px4_out": args.px4_output_topic,
    }

    status_builders = {
        "ov_guard": lambda: make_ov_guard_status_spec(args.ov_health_topic, args.ov_fault_topic),
        "px4_local": lambda: make_px4_local_status_spec(args.px4_local_position_topic),
        "px4_status": lambda: make_px4_status_spec(args.vehicle_status_topic),
    }

    position_states: List[PositionSourceState] = []
    status_states: List[StatusSourceState] = []

    for key in source_keys:
        if key in position_builders:
            topic_name = (
                args.topic
                if args.topic and len(source_keys) == 1
                else position_default_topics[key]
            )
            position_states.append(PositionSourceState(position_builders[key](topic_name)))
            continue

        if key in status_builders:
            if args.topic and len(source_keys) == 1:
                raise RuntimeError(
                    "--topic 仅支持位置类 source；状态类 source 请使用专用 topic 参数。"
                )
            status_states.append(StatusSourceState(status_builders[key]()))
            continue

        raise RuntimeError(f"不支持的数据源: {key}")

    return position_states, status_states


def main():
    args = parse_args()
    position_states, status_states = build_sources(args)

    rclpy.init()
    node = TopicMonitorNode(position_states, status_states)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    root = tk.Tk()
    gui = PositionMonitorGui(
        root=root,
        monitor_node=node,
        position_states=position_states,
        status_states=status_states,
        refresh_hz=args.refresh_hz,
        show_relative=not args.no_relative,
    )

    def close_app():
        root.quit()

    root.protocol("WM_DELETE_WINDOW", close_app)

    try:
        root.mainloop()
    finally:
        del gui
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
