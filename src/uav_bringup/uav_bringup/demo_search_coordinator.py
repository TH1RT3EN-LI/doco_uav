import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from uav_bridge.srv import BodyPositionDelta
except ModuleNotFoundError:  # pragma: no cover - local unit tests can import helpers without sourced ROS env
    BodyPositionDelta = None

try:
    from uav_visual_landing.msg import AprilTagDetection
except ModuleNotFoundError:  # pragma: no cover - local unit tests can import helpers without sourced ROS env
    AprilTagDetection = object

try:
    from ugv_bringup_interfaces.srv import GoRelativeXY
except ModuleNotFoundError:  # pragma: no cover - local unit tests can import helpers without sourced ROS env
    GoRelativeXY = None


@dataclass(frozen=True)
class WaypointDelta:
    x: float
    y: float
    z: float


@dataclass
class ActiveWaypoint:
    target_x: float
    target_y: float
    target_z: float
    deadline: Time
    index: int
    label: str


def rotate_planar(x: float, y: float, yaw: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        (cos_yaw * x) - (sin_yaw * y),
        (sin_yaw * x) + (cos_yaw * y),
    )


def yaw_from_quaternion_xyzw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - 2.0 * ((y * y) + (z * z))
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_multiply(
    lhs: tuple[float, float, float, float],
    rhs: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        (lw * rx) + (lx * rw) + (ly * rz) - (lz * ry),
        (lw * ry) - (lx * rz) + (ly * rw) + (lz * rx),
        (lw * rz) + (lx * ry) - (ly * rx) + (lz * rw),
        (lw * rw) - (lx * rx) - (ly * ry) - (lz * rz),
    )


def rotate_vector_by_quaternion(
    vector_xyz: tuple[float, float, float],
    quaternion_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    vx, vy, vz = vector_xyz
    qx, qy, qz, qw = quaternion_xyzw
    q_vec = (vx, vy, vz, 0.0)
    q_conj = (-qx, -qy, -qz, qw)
    rotated = quaternion_multiply(quaternion_multiply(quaternion_xyzw, q_vec), q_conj)
    return (rotated[0], rotated[1], rotated[2])


def parse_waypoint_deltas(value: str) -> list[WaypointDelta]:
    text = value.strip()
    if not text:
        return []

    waypoints: list[WaypointDelta] = []
    for token in text.replace("|", ";").split(";"):
        parts = [part.strip() for part in token.replace(",", " ").split() if part.strip()]
        if not parts:
            continue
        if len(parts) not in (2, 3):
            raise ValueError(
                "each waypoint must contain 2 or 3 numeric values: x y [z]"
            )
        try:
            values = [float(part) for part in parts]
        except ValueError as exc:
            raise ValueError("waypoints must contain numeric values") from exc
        if len(values) == 2:
            values.append(0.0)
        waypoints.append(WaypointDelta(*values))

    return waypoints


def compute_relative_goal_from_global_delta(
    *,
    tag_global_xy: tuple[float, float],
    uav_global_xy: tuple[float, float],
    ugv_relative_from_uav_global_xy: tuple[float, float],
    ugv_yaw_global_rad: float,
    goal_offset_ugv_body_xy: tuple[float, float] = (0.0, 0.0),
) -> tuple[float, float]:
    delta_global_x = (
        tag_global_xy[0] - uav_global_xy[0] - ugv_relative_from_uav_global_xy[0]
    )
    delta_global_y = (
        tag_global_xy[1] - uav_global_xy[1] - ugv_relative_from_uav_global_xy[1]
    )
    goal_x, goal_y = rotate_planar(
        delta_global_x,
        delta_global_y,
        -ugv_yaw_global_rad,
    )
    return (
        goal_x + goal_offset_ugv_body_xy[0],
        goal_y + goal_offset_ugv_body_xy[1],
    )


def compute_body_delta_target(
    *,
    current_position_xyz: tuple[float, float, float],
    current_yaw_rad: float,
    delta_body_xyz: tuple[float, float, float],
) -> tuple[float, float, float]:
    delta_x, delta_y = rotate_planar(
        delta_body_xyz[0],
        delta_body_xyz[1],
        current_yaw_rad,
    )
    return (
        current_position_xyz[0] + delta_x,
        current_position_xyz[1] + delta_y,
        current_position_xyz[2] + delta_body_xyz[2],
    )


class DemoSearchCoordinatorNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_demo_search_coordinator")

        self.global_frame = str(
            self.declare_parameter("global_frame", "global").value
        )
        self.uav_local_frame = str(
            self.declare_parameter("uav_local_frame", "uav_odom").value
        )
        self.uav_state_topic = str(
            self.declare_parameter("uav_state_topic", "/uav/state/odometry_px4").value
        )
        self.ugv_state_topic = str(
            self.declare_parameter("ugv_state_topic", "/ugv/odometry/filtered").value
        )
        self.relative_pose_global_topic = str(
            self.declare_parameter(
                "relative_pose_global_topic", "/relative_position/estimate/global"
            ).value
        )
        self.tag_detection_topic = str(
            self.declare_parameter(
                "tag_detection_topic", "/uav/visual_landing/apriltag_detection"
            ).value
        )
        self.position_delta_service_name = str(
            self.declare_parameter(
                "position_delta_service", "/uav/control/command/position_delta"
            ).value
        )
        self.position_keep_yaw_topic = str(
            self.declare_parameter(
                "position_keep_yaw_topic", "/uav/control/position_keep_yaw"
            ).value
        )
        self.hold_service_name = str(
            self.declare_parameter("hold_service", "/uav/control/command/hold").value
        )
        self.ugv_go_relative_service_name = str(
            self.declare_parameter(
                "ugv_go_relative_service", "/ugv/navigation/go_relative_xy"
            ).value
        )
        self.start_service_name = str(
            self.declare_parameter(
                "start_service", "/uav/demo_search/command/start"
            ).value
        )
        self.stop_service_name = str(
            self.declare_parameter(
                "stop_service", "/uav/demo_search/command/stop"
            ).value
        )
        self.manual_waypoint_service_name = str(
            self.declare_parameter(
                "manual_waypoint_service",
                "/uav/demo_search/command/waypoint_delta",
            ).value
        )
        self.auto_start = bool(self.declare_parameter("auto_start", False).value)
        self.auto_start_delay_s = max(
            0.0, float(self.declare_parameter("auto_start_delay_s", 2.0).value)
        )
        self.hold_on_finish = bool(
            self.declare_parameter("hold_on_finish", True).value
        )
        self.hold_on_stop = bool(self.declare_parameter("hold_on_stop", True).value)
        self.waypoint_reach_tolerance_m = max(
            0.01,
            float(self.declare_parameter("waypoint_reach_tolerance_m", 0.12).value),
        )
        self.waypoint_timeout_s = max(
            0.5, float(self.declare_parameter("waypoint_timeout_s", 30.0).value)
        )
        self.transform_timeout_s = max(
            0.0, float(self.declare_parameter("transform_timeout_s", 0.20).value)
        )
        self.uav_state_timeout_s = max(
            0.0, float(self.declare_parameter("uav_state_timeout_s", 0.50).value)
        )
        self.ugv_state_timeout_s = max(
            0.0, float(self.declare_parameter("ugv_state_timeout_s", 0.50).value)
        )
        self.relative_pose_timeout_s = max(
            0.0, float(self.declare_parameter("relative_pose_timeout_s", 0.50).value)
        )
        self.tag_confidence_threshold = float(
            self.declare_parameter("tag_confidence_threshold", 0.0).value
        )
        self.goal_offset_x_m = float(
            self.declare_parameter("ugv_goal_offset_x_m", 0.0).value
        )
        self.goal_offset_y_m = float(
            self.declare_parameter("ugv_goal_offset_y_m", 0.0).value
        )
        self.service_wait_timeout_s = max(
            0.0, float(self.declare_parameter("service_wait_timeout_s", 1.0).value)
        )

        waypoint_text = str(self.declare_parameter("waypoints_body", "").value)
        self.waypoints = parse_waypoint_deltas(waypoint_text)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        if BodyPositionDelta is None or GoRelativeXY is None:
            raise RuntimeError(
                "demo_search_coordinator requires sourced ROS interfaces: uav_bridge and ugv_bringup_interfaces"
            )

        self.position_delta_client = self.create_client(
            BodyPositionDelta, self.position_delta_service_name
        )
        self.hold_client = self.create_client(Trigger, self.hold_service_name)
        self.ugv_go_relative_client = self.create_client(
            GoRelativeXY, self.ugv_go_relative_service_name
        )
        self.position_keep_yaw_pub = self.create_publisher(
            PointStamped, self.position_keep_yaw_topic, 10
        )

        self.create_subscription(
            Odometry,
            self.uav_state_topic,
            self._on_uav_state,
            10,
        )
        self.create_subscription(
            Odometry,
            self.ugv_state_topic,
            self._on_ugv_state,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.relative_pose_global_topic,
            self._on_relative_pose_global,
            10,
        )
        self.create_subscription(
            AprilTagDetection,
            self.tag_detection_topic,
            self._on_tag_detection,
            10,
        )

        if self.start_service_name:
            self.create_service(Trigger, self.start_service_name, self._handle_start)
        if self.stop_service_name:
            self.create_service(Trigger, self.stop_service_name, self._handle_stop)
        if self.manual_waypoint_service_name:
            self.create_service(
                BodyPositionDelta,
                self.manual_waypoint_service_name,
                self._handle_manual_waypoint_delta,
            )

        self.latest_uav_state: Optional[Odometry] = None
        self.latest_uav_state_receive_time: Optional[Time] = None
        self.latest_ugv_state: Optional[Odometry] = None
        self.latest_ugv_state_receive_time: Optional[Time] = None
        self.latest_relative_pose_global: Optional[PoseWithCovarianceStamped] = None
        self.latest_relative_pose_global_receive_time: Optional[Time] = None

        self.search_active = False
        self.next_waypoint_index = 0
        self.active_waypoint: Optional[ActiveWaypoint] = None
        self.command_sequence = 0
        self.tag_handled = False

        self.auto_start_timer = None
        if self.auto_start and self.waypoints:
            self.auto_start_timer = self.create_timer(
                self.auto_start_delay_s,
                self._auto_start_once,
            )

        self.watchdog_timer = self.create_timer(0.1, self._on_watchdog)

        self.get_logger().info(
            "demo search coordinator ready: waypoints=%d auto_start=%s tag_topic=%s relative_pose=%s position_delta=%s ugv_go_relative=%s"
            % (
                len(self.waypoints),
                "true" if self.auto_start else "false",
                self.tag_detection_topic,
                self.relative_pose_global_topic,
                self.position_delta_service_name,
                self.ugv_go_relative_service_name,
            )
        )

    def _auto_start_once(self) -> None:
        if self.auto_start_timer is not None:
            self.auto_start_timer.cancel()
            self.auto_start_timer = None
        self._start_search(reason="auto_start")

    def _handle_start(self, request, response):
        del request
        ok, message = self._start_search(reason="service")
        response.success = ok
        response.message = message
        return response

    def _handle_stop(self, request, response):
        del request
        self._stop_search(reason="stop_service", request_hold=self.hold_on_stop)
        response.success = True
        response.message = "search stopped"
        return response

    def _handle_manual_waypoint_delta(self, request, response):
        self.search_active = False
        self.active_waypoint = None
        self.command_sequence += 1

        ok, message, target_xyz = self._publish_manual_body_delta(
            delta_body_xyz=(float(request.x), float(request.y), float(request.z)),
            label="manual waypoint",
        )
        response.success = ok
        response.message = message
        if target_xyz is not None:
            response.target_x = float(target_xyz[0])
            response.target_y = float(target_xyz[1])
            response.target_z = float(target_xyz[2])
        else:
            response.target_x = 0.0
            response.target_y = 0.0
            response.target_z = 0.0
        return response

    def _start_search(self, *, reason: str) -> tuple[bool, str]:
        if not self.waypoints:
            message = "no waypoints configured"
            self.get_logger().warning(message)
            return False, message
        if self.search_active or self.active_waypoint is not None:
            return False, "search already active"

        self.search_active = True
        self.next_waypoint_index = 0
        self.active_waypoint = None
        self.tag_handled = False
        self.command_sequence += 1
        self.get_logger().info(
            "starting search (%s): %d waypoint(s)" % (reason, len(self.waypoints))
        )
        self._dispatch_next_waypoint()
        return True, "search started"

    def _stop_search(self, *, reason: str, request_hold: bool) -> None:
        was_active = self.search_active or self.active_waypoint is not None
        self.search_active = False
        self.active_waypoint = None
        self.command_sequence += 1
        if was_active:
            self.get_logger().info("search stopped: %s" % reason)
        if request_hold:
            self._request_hold(reason=reason)

    def _dispatch_next_waypoint(self) -> None:
        if not self.search_active:
            return
        if self.next_waypoint_index >= len(self.waypoints):
            self.search_active = False
            self.active_waypoint = None
            self.get_logger().info("search waypoints completed")
            if self.hold_on_finish:
                self._request_hold(reason="search_finished")
            return

        if not self._ensure_service_ready(
            self.position_delta_client, self.position_delta_service_name
        ):
            self._stop_search(reason="position_delta_unavailable", request_hold=True)
            return

        waypoint = self.waypoints[self.next_waypoint_index]
        sequence = self.command_sequence

        request = BodyPositionDelta.Request()
        request.x = float(waypoint.x)
        request.y = float(waypoint.y)
        request.z = float(waypoint.z)

        future = self.position_delta_client.call_async(request)
        future.add_done_callback(
            lambda done, seq=sequence, index=self.next_waypoint_index: self._on_position_delta_done(
                done,
                seq,
                index,
            )
        )
        self.get_logger().info(
            "dispatching waypoint %d/%d body_delta=(%.3f, %.3f, %.3f)"
            % (
                self.next_waypoint_index + 1,
                len(self.waypoints),
                waypoint.x,
                waypoint.y,
                waypoint.z,
            )
        )
        self.next_waypoint_index += 1

    def _on_position_delta_done(self, future, sequence: int, index: int) -> None:
        if sequence != self.command_sequence or not self.search_active:
            return

        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - defensive runtime logging
            self.get_logger().error("position delta call failed: %s" % exc)
            self._stop_search(reason="position_delta_failed", request_hold=True)
            return

        if response is None or not response.success:
            message = (
                "<no response>" if response is None else response.message.strip() or "rejected"
            )
            self.get_logger().warning(
                "waypoint %d rejected by position_delta: %s" % (index + 1, message)
            )
            self._stop_search(reason="position_delta_rejected", request_hold=True)
            return

        self.active_waypoint = ActiveWaypoint(
            target_x=float(response.target_x),
            target_y=float(response.target_y),
            target_z=float(response.target_z),
            deadline=self.get_clock().now() + Duration(seconds=self.waypoint_timeout_s),
            index=index,
            label=f"search waypoint {index + 1}",
        )
        self.get_logger().info(
            "%s accepted -> local target=(%.3f, %.3f, %.3f)"
            % (
                self.active_waypoint.label,
                self.active_waypoint.target_x,
                self.active_waypoint.target_y,
                self.active_waypoint.target_z,
            )
        )

    def _on_uav_state(self, msg: Odometry) -> None:
        self.latest_uav_state = msg
        self.latest_uav_state_receive_time = self.get_clock().now()

        if self.active_waypoint is None:
            return

        current_position = self._resolve_odometry_position_in_frame(
            msg, self.uav_local_frame
        )
        if current_position is None:
            return

        distance = math.sqrt(
            ((current_position[0] - self.active_waypoint.target_x) ** 2.0)
            + ((current_position[1] - self.active_waypoint.target_y) ** 2.0)
            + ((current_position[2] - self.active_waypoint.target_z) ** 2.0)
        )
        if distance <= self.waypoint_reach_tolerance_m:
            reached_waypoint = self.active_waypoint
            self.active_waypoint = None
            self.get_logger().info(
                "%s reached within %.3f m"
                % (reached_waypoint.label, self.waypoint_reach_tolerance_m)
            )
            self._dispatch_next_waypoint()

    def _on_ugv_state(self, msg: Odometry) -> None:
        self.latest_ugv_state = msg
        self.latest_ugv_state_receive_time = self.get_clock().now()

    def _on_relative_pose_global(self, msg: PoseWithCovarianceStamped) -> None:
        self.latest_relative_pose_global = msg
        self.latest_relative_pose_global_receive_time = self.get_clock().now()

    def _on_tag_detection(self, msg: AprilTagDetection) -> None:
        if self.tag_handled:
            return
        if not self.search_active and self.active_waypoint is None:
            return
        if not msg.detected or not msg.pose_valid:
            return
        if float(msg.confidence) < self.tag_confidence_threshold:
            return

        tag_global_xyz = self._resolve_tag_global_position(msg)
        if tag_global_xyz is None:
            return

        uav_global = self._resolve_global_pose_from_odometry(
            self.latest_uav_state,
            self.latest_uav_state_receive_time,
            self.uav_state_timeout_s,
            self.uav_state_topic,
        )
        if uav_global is None:
            return

        relative_pose = self._require_recent_relative_pose_global()
        if relative_pose is None:
            return

        ugv_yaw_global = self._resolve_global_yaw_from_odometry(
            self.latest_ugv_state,
            self.latest_ugv_state_receive_time,
            self.ugv_state_timeout_s,
            self.ugv_state_topic,
        )
        if ugv_yaw_global is None:
            return

        if relative_pose.header.frame_id and relative_pose.header.frame_id != self.global_frame:
            self.get_logger().warning(
                "relative pose frame mismatch: expected %s, got %s"
                % (self.global_frame, relative_pose.header.frame_id)
            )
            return

        goal_x, goal_y = compute_relative_goal_from_global_delta(
            tag_global_xy=(tag_global_xyz[0], tag_global_xyz[1]),
            uav_global_xy=(uav_global[0], uav_global[1]),
            ugv_relative_from_uav_global_xy=(
                float(relative_pose.pose.pose.position.x),
                float(relative_pose.pose.pose.position.y),
            ),
            ugv_yaw_global_rad=ugv_yaw_global,
            goal_offset_ugv_body_xy=(self.goal_offset_x_m, self.goal_offset_y_m),
        )

        self.tag_handled = True
        self.search_active = False
        self.active_waypoint = None
        self.command_sequence += 1

        self.get_logger().info(
            "tag detected id=%d -> stopping UAV and dispatching UGV body goal=(%.3f, %.3f)"
            % (msg.tag_id, goal_x, goal_y)
        )

        self._request_hold(reason="tag_detected")
        self._dispatch_ugv_relative_goal(goal_x, goal_y)

    def _dispatch_ugv_relative_goal(self, x: float, y: float) -> None:
        if not self._ensure_service_ready(
            self.ugv_go_relative_client, self.ugv_go_relative_service_name
        ):
            self.get_logger().warning(
                "UGV relative-goal service unavailable: %s"
                % self.ugv_go_relative_service_name
            )
            return

        request = GoRelativeXY.Request()
        request.x = float(x)
        request.y = float(y)
        future = self.ugv_go_relative_client.call_async(request)
        future.add_done_callback(self._on_ugv_go_relative_done)

    def _on_ugv_go_relative_done(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - defensive runtime logging
            self.get_logger().error("UGV go_relative call failed: %s" % exc)
            return

        if response is None or not response.success:
            message = (
                "<no response>" if response is None else response.message.strip() or "rejected"
            )
            self.get_logger().warning("UGV go_relative rejected: %s" % message)
            return

        self.get_logger().info(
            "UGV goal published in %s: (%.3f, %.3f, yaw=%.3f)"
            % (
                response.goal_frame_id,
                response.goal_x,
                response.goal_y,
                response.goal_yaw,
            )
        )

    def _request_hold(self, *, reason: str) -> None:
        if not self._ensure_service_ready(self.hold_client, self.hold_service_name):
            self.get_logger().warning(
                "hold service unavailable during %s: %s"
                % (reason, self.hold_service_name)
            )
            return

        future = self.hold_client.call_async(Trigger.Request())
        future.add_done_callback(lambda done, why=reason: self._on_hold_done(done, why))

    def _on_hold_done(self, future, reason: str) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - defensive runtime logging
            self.get_logger().warning("hold call failed during %s: %s" % (reason, exc))
            return

        if response is None:
            self.get_logger().warning("hold call returned no response during %s" % reason)
            return

        level = self.get_logger().info if response.success else self.get_logger().warning
        level(
            "hold result during %s: %s"
            % (reason, response.message.strip() or ("ok" if response.success else "failed"))
        )

    def _on_watchdog(self) -> None:
        if self.active_waypoint is None:
            return
        if self.get_clock().now() <= self.active_waypoint.deadline:
            return
        self.get_logger().warning(
            "%s timed out after %.1f s"
            % (self.active_waypoint.label, self.waypoint_timeout_s)
        )
        self._stop_search(reason="waypoint_timeout", request_hold=True)

    def _publish_manual_body_delta(
        self,
        *,
        delta_body_xyz: tuple[float, float, float],
        label: str,
    ) -> tuple[bool, str, Optional[tuple[float, float, float]]]:
        local_pose = self._resolve_local_pose_from_uav_state()
        if local_pose is None:
            return False, "fresh UAV local pose unavailable", None

        target_xyz = compute_body_delta_target(
            current_position_xyz=(local_pose[0], local_pose[1], local_pose[2]),
            current_yaw_rad=local_pose[3],
            delta_body_xyz=delta_body_xyz,
        )

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.uav_local_frame
        msg.point.x = float(target_xyz[0])
        msg.point.y = float(target_xyz[1])
        msg.point.z = float(target_xyz[2])
        self.position_keep_yaw_pub.publish(msg)

        self.active_waypoint = ActiveWaypoint(
            target_x=float(target_xyz[0]),
            target_y=float(target_xyz[1]),
            target_z=float(target_xyz[2]),
            deadline=self.get_clock().now() + Duration(seconds=self.waypoint_timeout_s),
            index=-1,
            label=label,
        )
        self.get_logger().info(
            "%s published body_delta=(%.3f, %.3f, %.3f) -> local target=(%.3f, %.3f, %.3f)"
            % (
                label,
                delta_body_xyz[0],
                delta_body_xyz[1],
                delta_body_xyz[2],
                target_xyz[0],
                target_xyz[1],
                target_xyz[2],
            )
        )
        return True, "manual waypoint accepted", target_xyz

    def _resolve_tag_global_position(
        self, detection: AprilTagDetection
    ) -> Optional[tuple[float, float, float]]:
        stamp = self._resolve_time(detection.header.stamp)

        if detection.position_odom_valid and detection.odom_frame_id:
            return self._transform_point_to_frame(
                point_xyz=(
                    float(detection.position_odom_m.x),
                    float(detection.position_odom_m.y),
                    float(detection.position_odom_m.z),
                ),
                source_frame=detection.odom_frame_id,
                target_frame=self.global_frame,
                stamp=stamp,
            )

        if detection.position_base_valid and detection.base_frame_id:
            return self._transform_point_to_frame(
                point_xyz=(
                    float(detection.position_base_m.x),
                    float(detection.position_base_m.y),
                    float(detection.position_base_m.z),
                ),
                source_frame=detection.base_frame_id,
                target_frame=self.global_frame,
                stamp=stamp,
            )

        self.get_logger().warning("tag detection missing usable odom/base position")
        return None

    def _resolve_global_pose_from_odometry(
        self,
        msg: Optional[Odometry],
        receive_time: Optional[Time],
        timeout_s: float,
        source_name: str,
    ) -> Optional[tuple[float, float, float, float]]:
        return self._resolve_pose_from_odometry(
            msg=msg,
            receive_time=receive_time,
            timeout_s=timeout_s,
            source_name=source_name,
            target_frame=self.global_frame,
            default_frame=self.uav_local_frame,
        )

    def _resolve_pose_from_odometry(
        self,
        *,
        msg: Optional[Odometry],
        receive_time: Optional[Time],
        timeout_s: float,
        source_name: str,
        target_frame: str,
        default_frame: str,
    ) -> Optional[tuple[float, float, float, float]]:
        if msg is None or receive_time is None:
            self.get_logger().warning("missing odometry from %s" % source_name)
            return None
        if timeout_s > 0.0 and (self.get_clock().now() - receive_time).nanoseconds > int(
            timeout_s * 1.0e9
        ):
            self.get_logger().warning("stale odometry from %s" % source_name)
            return None

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.stamp = self._resolve_time(msg.header.stamp).to_msg()
        pose.header.frame_id = msg.header.frame_id or default_frame
        pose.pose = msg.pose.pose

        if pose.header.frame_id == target_frame:
            yaw = yaw_from_quaternion_xyzw(
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            )
            return (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
                yaw,
            )

        transform = self._lookup_transform(
            target_frame,
            pose.header.frame_id,
            self._resolve_time(msg.header.stamp),
        )
        if transform is None:
            return None

        tf_quat = (
            float(transform.transform.rotation.x),
            float(transform.transform.rotation.y),
            float(transform.transform.rotation.z),
            float(transform.transform.rotation.w),
        )
        tf_translation = (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            float(transform.transform.translation.z),
        )
        rotated_position = rotate_vector_by_quaternion(
            (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            ),
            tf_quat,
        )
        orientation_global = quaternion_multiply(
            tf_quat,
            (
                float(pose.pose.orientation.x),
                float(pose.pose.orientation.y),
                float(pose.pose.orientation.z),
                float(pose.pose.orientation.w),
            ),
        )
        return (
            rotated_position[0] + tf_translation[0],
            rotated_position[1] + tf_translation[1],
            rotated_position[2] + tf_translation[2],
            yaw_from_quaternion_xyzw(*orientation_global),
        )

    def _resolve_local_pose_from_uav_state(
        self,
    ) -> Optional[tuple[float, float, float, float]]:
        return self._resolve_pose_from_odometry(
            msg=self.latest_uav_state,
            receive_time=self.latest_uav_state_receive_time,
            timeout_s=self.uav_state_timeout_s,
            source_name=self.uav_state_topic,
            target_frame=self.uav_local_frame,
            default_frame=self.uav_local_frame,
        )

    def _resolve_global_yaw_from_odometry(
        self,
        msg: Optional[Odometry],
        receive_time: Optional[Time],
        timeout_s: float,
        source_name: str,
    ) -> Optional[float]:
        pose = self._resolve_global_pose_from_odometry(
            msg,
            receive_time,
            timeout_s,
            source_name,
        )
        if pose is None:
            return None
        return pose[3]

    def _resolve_odometry_position_in_frame(
        self, msg: Odometry, target_frame: str
    ) -> Optional[tuple[float, float, float]]:
        source_frame = msg.header.frame_id or target_frame
        if source_frame == target_frame:
            return (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                float(msg.pose.pose.position.z),
            )
        return self._transform_point_to_frame(
            point_xyz=(
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                float(msg.pose.pose.position.z),
            ),
            source_frame=source_frame,
            target_frame=target_frame,
            stamp=self._resolve_time(msg.header.stamp),
        )

    def _require_recent_relative_pose_global(
        self,
    ) -> Optional[PoseWithCovarianceStamped]:
        if (
            self.latest_relative_pose_global is None
            or self.latest_relative_pose_global_receive_time is None
        ):
            self.get_logger().warning("missing relative pose estimate")
            return None

        if self.relative_pose_timeout_s > 0.0 and (
            self.get_clock().now() - self.latest_relative_pose_global_receive_time
        ).nanoseconds > int(self.relative_pose_timeout_s * 1.0e9):
            self.get_logger().warning("stale relative pose estimate")
            return None

        return self.latest_relative_pose_global

    def _transform_point_to_frame(
        self,
        *,
        point_xyz: tuple[float, float, float],
        source_frame: str,
        target_frame: str,
        stamp: Time,
    ) -> Optional[tuple[float, float, float]]:
        if source_frame == target_frame:
            return point_xyz

        transform = self._lookup_transform(target_frame, source_frame, stamp)
        if transform is None:
            return None

        rotation = (
            float(transform.transform.rotation.x),
            float(transform.transform.rotation.y),
            float(transform.transform.rotation.z),
            float(transform.transform.rotation.w),
        )
        translation = (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            float(transform.transform.translation.z),
        )
        rotated = rotate_vector_by_quaternion(point_xyz, rotation)
        return (
            rotated[0] + translation[0],
            rotated[1] + translation[1],
            rotated[2] + translation[2],
        )

    def _lookup_transform(
        self, target_frame: str, source_frame: str, stamp: Time
    ):
        timeout = Duration(seconds=self.transform_timeout_s)
        try:
            return self.tf_buffer.lookup_transform(
                target_frame, source_frame, stamp, timeout=timeout
            )
        except TransformException:
            try:
                return self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    timeout=timeout,
                )
            except TransformException as exc:
                self.get_logger().warning(
                    "failed to resolve transform %s <- %s: %s"
                    % (target_frame, source_frame, exc)
                )
                return None

    def _ensure_service_ready(self, client, service_name: str) -> bool:
        if client.service_is_ready():
            return True
        return client.wait_for_service(timeout_sec=self.service_wait_timeout_s)

    def _resolve_time(self, stamp_msg) -> Time:
        if stamp_msg.sec == 0 and stamp_msg.nanosec == 0:
            return Time()
        return Time.from_msg(stamp_msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DemoSearchCoordinatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
