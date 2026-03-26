#!/usr/bin/env python3

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from std_srvs.srv import Trigger
from uav_visual_landing.msg import LandingControllerState


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = 0.5 * yaw
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


@dataclass
class StateSnapshot:
    x: float
    y: float
    z: float
    yaw: float
    frame_id: str
    stamp: Time


@dataclass
class HomePose:
    x: float
    y: float
    z: float
    yaw: float
    frame_id: str


class HomeVisualLandingTestNode(Node):
    def __init__(self) -> None:
        super().__init__("home_visual_landing_test_node")

        self.declare_parameter("state_topic", "/uav/state/odometry")
        self.declare_parameter("position_topic", "/uav/control/pose")
        self.declare_parameter("takeoff_service", "/uav/control/command/takeoff")
        self.declare_parameter("hold_service", "/uav/control/command/hold")
        self.declare_parameter(
            "visual_landing_start_service", "/uav/visual_landing/command/start"
        )
        self.declare_parameter(
            "visual_landing_state_topic", "/uav/visual_landing/controller_state"
        )
        self.declare_parameter("home_takeoff_service", "/uav/test/home_takeoff")
        self.declare_parameter("home_visual_land_service", "/uav/test/home_visual_land")
        self.declare_parameter("arrival_xy_tolerance_m", 0.15)
        self.declare_parameter("state_timeout_s", 0.20)

        self._state_topic = self.get_parameter("state_topic").value
        self._position_topic = self.get_parameter("position_topic").value
        self._takeoff_service = self.get_parameter("takeoff_service").value
        self._hold_service = self.get_parameter("hold_service").value
        self._visual_landing_start_service = (
            self.get_parameter("visual_landing_start_service").value
        )
        self._visual_landing_state_topic = (
            self.get_parameter("visual_landing_state_topic").value
        )
        self._home_takeoff_service = self.get_parameter("home_takeoff_service").value
        self._home_visual_land_service = self.get_parameter("home_visual_land_service").value
        self._arrival_xy_tolerance_m = float(
            self.get_parameter("arrival_xy_tolerance_m").value
        )
        self._state_timeout_s = float(self.get_parameter("state_timeout_s").value)

        controller_state_qos = QoSProfile(depth=1)
        controller_state_qos.reliability = ReliabilityPolicy.RELIABLE
        controller_state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._position_pub = self.create_publisher(PoseStamped, self._position_topic, 10)
        self._state_sub = self.create_subscription(
            Odometry,
            self._state_topic,
            self._handle_state,
            qos_profile_sensor_data,
        )
        self._visual_landing_state_sub = self.create_subscription(
            LandingControllerState,
            self._visual_landing_state_topic,
            self._handle_visual_landing_state,
            controller_state_qos,
        )

        self._takeoff_client = self.create_client(Trigger, self._takeoff_service)
        self._hold_client = self.create_client(Trigger, self._hold_service)
        self._visual_landing_start_client = self.create_client(
            Trigger, self._visual_landing_start_service
        )

        self._home_takeoff_srv = self.create_service(
            Trigger, self._home_takeoff_service, self._handle_home_takeoff
        )
        self._home_visual_land_srv = self.create_service(
            Trigger, self._home_visual_land_service, self._handle_home_visual_land
        )

        self._latest_state: StateSnapshot | None = None
        self._visual_landing_active = False
        self._home_pose: HomePose | None = None
        self._pending_home_pose: HomePose | None = None

        self._takeoff_future = None
        self._visual_landing_start_future = None
        self._hold_future = None

        self._return_active = False

        self._timer = self.create_wall_timer(0.05, self._timer_callback)

        self.get_logger().info(
            "home_visual_landing_test_node: "
            f"state={self._state_topic} position={self._position_topic} "
            f"home_takeoff={self._home_takeoff_service} "
            f"home_visual_land={self._home_visual_land_service} "
            f"takeoff={self._takeoff_service} hold={self._hold_service} "
            f"visual_start={self._visual_landing_start_service} "
            f"visual_state={self._visual_landing_state_topic}"
        )

    def _handle_state(self, msg: Odometry) -> None:
        has_stamp = (msg.header.stamp.sec != 0) or (msg.header.stamp.nanosec != 0)
        stamp = Time.from_msg(msg.header.stamp) if has_stamp else self.get_clock().now()
        orientation = msg.pose.pose.orientation
        self._latest_state = StateSnapshot(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            z=float(msg.pose.pose.position.z),
            yaw=quaternion_to_yaw(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ),
            frame_id=msg.header.frame_id,
            stamp=stamp,
        )

    def _handle_visual_landing_state(self, msg: LandingControllerState) -> None:
        self._visual_landing_active = bool(msg.active)

    def _state_fresh(self) -> bool:
        if self._latest_state is None:
            return False
        age_s = (self.get_clock().now() - self._latest_state.stamp).nanoseconds / 1.0e9
        return math.isfinite(age_s) and 0.0 <= age_s <= self._state_timeout_s

    def _current_state_or_error(self) -> tuple[StateSnapshot | None, str | None]:
        if self._latest_state is None:
            return None, "state odometry is not available yet"
        if not self._state_fresh():
            return None, "state odometry is stale"
        return self._latest_state, None

    def _handle_home_takeoff(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        if self._takeoff_future is not None:
            response.success = False
            response.message = "home takeoff is already pending"
            return response

        state, error = self._current_state_or_error()
        if error is not None:
            response.success = False
            response.message = error
            return response

        if not self._takeoff_client.service_is_ready():
            response.success = False
            response.message = f"takeoff service not ready: {self._takeoff_service}"
            return response

        self._pending_home_pose = HomePose(
            x=state.x,
            y=state.y,
            z=state.z,
            yaw=state.yaw,
            frame_id=state.frame_id,
        )
        self._takeoff_future = self._takeoff_client.call_async(Trigger.Request())
        response.success = True
        response.message = "home takeoff queued; home will commit after takeoff acceptance"
        self.get_logger().info(
            f"queued home takeoff from [{state.x:.3f}, {state.y:.3f}, {state.z:.3f}] "
            f"yaw={state.yaw:.3f}"
        )
        return response

    def _handle_home_visual_land(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        if self._return_active or self._visual_landing_start_future is not None:
            response.success = False
            response.message = "home visual landing is already in progress"
            return response
        if self._visual_landing_active:
            response.success = False
            response.message = "visual landing is already active"
            return response
        if self._home_pose is None:
            response.success = False
            response.message = "home pose is not locked yet"
            return response

        state, error = self._current_state_or_error()
        if error is not None:
            response.success = False
            response.message = error
            return response

        self._publish_home_return_pose(state)
        self._return_active = True
        response.success = True
        response.message = "home visual landing started: returning to home xy"
        self.get_logger().info(
            "started home visual landing return towards "
            f"[{self._home_pose.x:.3f}, {self._home_pose.y:.3f}] "
            f"at current z={state.z:.3f} yaw={state.yaw:.3f}"
        )
        return response

    def _publish_home_return_pose(self, state: StateSnapshot) -> None:
        if self._home_pose is None:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = state.frame_id or self._home_pose.frame_id
        msg.pose.position.x = self._home_pose.x
        msg.pose.position.y = self._home_pose.y
        msg.pose.position.z = state.z
        quat_x, quat_y, quat_z, quat_w = yaw_to_quaternion(state.yaw)
        msg.pose.orientation.x = quat_x
        msg.pose.orientation.y = quat_y
        msg.pose.orientation.z = quat_z
        msg.pose.orientation.w = quat_w
        self._position_pub.publish(msg)

    def _arrival_reached(self, state: StateSnapshot) -> bool:
        if self._home_pose is None:
            return False
        xy_error = math.hypot(state.x - self._home_pose.x, state.y - self._home_pose.y)
        return xy_error <= self._arrival_xy_tolerance_m

    def _request_hold(self, reason: str) -> None:
        if not self._hold_client.service_is_ready():
            self.get_logger().error(
                f"failed to request hold after {reason}: service not ready: "
                f"{self._hold_service}"
            )
            return

        self._hold_future = self._hold_client.call_async(Trigger.Request())
        self.get_logger().warn(f"requested hold after {reason}")

    def _fail_home_visual_land(self, reason: str) -> None:
        self._return_active = False
        self._visual_landing_start_future = None
        self._request_hold(reason)

    def _process_takeoff_future(self) -> None:
        if self._takeoff_future is None or not self._takeoff_future.done():
            return

        try:
            result = self._takeoff_future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"home takeoff request failed: {exc}")
            result = None

        if result is not None and result.success and self._pending_home_pose is not None:
            self._home_pose = self._pending_home_pose
            self.get_logger().info(
                f"committed home pose at [{self._home_pose.x:.3f}, {self._home_pose.y:.3f}, "
                f"{self._home_pose.z:.3f}] yaw={self._home_pose.yaw:.3f}"
            )
        else:
            message = result.message if result is not None else "exception before response"
            self.get_logger().warn(f"takeoff was not accepted; home pose not updated: {message}")

        self._pending_home_pose = None
        self._takeoff_future = None

    def _process_hold_future(self) -> None:
        if self._hold_future is None or not self._hold_future.done():
            return
        try:
            result = self._hold_future.result()
            if result is not None and not result.success:
                self.get_logger().error(f"hold request failed: {result.message}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"hold request raised an exception: {exc}")
        self._hold_future = None

    def _process_visual_landing_start_future(self) -> None:
        if (
            self._visual_landing_start_future is None
            or not self._visual_landing_start_future.done()
        ):
            return

        try:
            result = self._visual_landing_start_future.result()
        except Exception as exc:  # noqa: BLE001
            self._fail_home_visual_land(f"visual landing start raised an exception: {exc}")
            return

        self._visual_landing_start_future = None
        if result is None or not result.success:
            message = result.message if result is not None else "empty response"
            self._fail_home_visual_land(f"visual landing start failed: {message}")
            return

        self._return_active = False
        self.get_logger().info("home return reached target; handed off to visual landing")

    def _process_home_visual_land(self) -> None:
        self._process_visual_landing_start_future()

        if not self._return_active:
            return
        if self._visual_landing_active and self._visual_landing_start_future is None:
            self._return_active = False
            self.get_logger().info(
                "visual landing became active during home return; stopping test node handoff logic"
            )
            return

        state, error = self._current_state_or_error()
        if error is not None:
            self._fail_home_visual_land(f"home return aborted: {error}")
            return
        if not self._arrival_reached(state):
            return
        if self._visual_landing_start_future is not None:
            return
        if not self._visual_landing_start_client.service_is_ready():
            self._fail_home_visual_land(
                f"visual landing start service not ready: {self._visual_landing_start_service}"
            )
            return

        self._visual_landing_start_future = self._visual_landing_start_client.call_async(
            Trigger.Request()
        )
        self.get_logger().info(
            f"home xy reached within {self._arrival_xy_tolerance_m:.3f} m; requesting visual landing start"
        )

    def _timer_callback(self) -> None:
        self._process_takeoff_future()
        self._process_hold_future()
        self._process_home_visual_land()


def main() -> None:
    rclpy.init()
    node = HomeVisualLandingTestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
