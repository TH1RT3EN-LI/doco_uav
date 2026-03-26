#!/usr/bin/env python3

import os
import shutil
import signal
import subprocess
import threading
from pathlib import Path

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from rclpy.node import Node
from std_msgs.msg import Empty
from std_srvs.srv import Trigger


def bool_to_ros(value: bool) -> str:
    return "true" if value else "false"


def normalize_namespace(value: str) -> str:
    if not value:
        return "/"
    if not value.startswith("/"):
        value = f"/{value}"
    while len(value) > 1 and value.endswith("/"):
        value = value[:-1]
    return value


class OpenVinsSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("openvins_supervisor")

        self.declare_parameter("child_namespace", self.get_namespace())
        self.declare_parameter("verbosity", "INFO")
        self.declare_parameter("save_total_state", False)
        self.declare_parameter("publish_calibration_tf", True)
        self.declare_parameter("filepath_est", "/tmp/ov_estimate.txt")
        self.declare_parameter("filepath_std", "/tmp/ov_estimate_std.txt")
        self.declare_parameter("filepath_gt", "/tmp/ov_groundtruth.txt")
        self.declare_parameter("config_path", "")
        self.declare_parameter("reset_service_name", "reset")
        self.declare_parameter("reset_counter_bump_topic", "reset_counter_bump")

        self._child_namespace = normalize_namespace(
            self.get_parameter("child_namespace").get_parameter_value().string_value
        )
        self._verbosity = self.get_parameter("verbosity").get_parameter_value().string_value
        self._save_total_state = (
            self.get_parameter("save_total_state").get_parameter_value().bool_value
        )
        self._publish_calibration_tf = (
            self.get_parameter("publish_calibration_tf").get_parameter_value().bool_value
        )
        self._filepath_est = self.get_parameter("filepath_est").get_parameter_value().string_value
        self._filepath_std = self.get_parameter("filepath_std").get_parameter_value().string_value
        self._filepath_gt = self.get_parameter("filepath_gt").get_parameter_value().string_value
        self._config_path = self.get_parameter("config_path").get_parameter_value().string_value
        self._reset_service_name = (
            self.get_parameter("reset_service_name").get_parameter_value().string_value
        )
        self._reset_counter_bump_topic = (
            self.get_parameter("reset_counter_bump_topic").get_parameter_value().string_value
        )

        self._process_lock = threading.Lock()
        self._process: subprocess.Popen | None = None
        self._openvins_executable = self._resolve_openvins_executable()

        self._reset_counter_bump_pub = self.create_publisher(
            Empty, self._reset_counter_bump_topic, 10
        )
        self._reset_srv = self.create_service(
            Trigger, self._reset_service_name, self._handle_reset
        )

        self._start_process("initial launch")

    def destroy_node(self) -> bool:
        with self._process_lock:
            self._stop_process_locked("node shutdown")
        return super().destroy_node()

    def _resolve_openvins_executable(self):
        try:
            prefix = Path(get_package_prefix("ov_msckf"))
            executable = prefix / "lib" / "ov_msckf" / "run_subscribe_msckf"
            if executable.is_file() and os.access(executable, os.X_OK):
                return [str(executable)]
        except PackageNotFoundError:
            pass

        ros2_path = shutil.which("ros2")
        if ros2_path:
            return [ros2_path, "run", "ov_msckf", "run_subscribe_msckf"]

        raise RuntimeError("failed to locate ov_msckf/run_subscribe_msckf executable")

    def _build_command(self):
        if not self._config_path:
            raise RuntimeError("openvins config_path is empty")

        return [
            *self._openvins_executable,
            "--ros-args",
            "-r",
            f"__ns:={self._child_namespace}",
            "-p",
            f"verbosity:={self._verbosity}",
            "-p",
            "use_stereo:=true",
            "-p",
            "max_cameras:=2",
            "-p",
            f"save_total_state:={bool_to_ros(self._save_total_state)}",
            "-p",
            f"publish_calibration_tf:={bool_to_ros(self._publish_calibration_tf)}",
            "-p",
            f"filepath_est:={self._filepath_est}",
            "-p",
            f"filepath_std:={self._filepath_std}",
            "-p",
            f"filepath_gt:={self._filepath_gt}",
            "-p",
            f"config_path:={self._config_path}",
        ]

    def _start_process(self, reason: str) -> None:
        with self._process_lock:
            self._start_process_locked(reason)

    def _start_process_locked(self, reason: str) -> None:
        if self._process is not None and self._process.poll() is None:
            return

        command = self._build_command()
        preexec_fn = os.setsid if os.name == "posix" else None
        self._process = subprocess.Popen(command, preexec_fn=preexec_fn)
        self.get_logger().info(
            f"started OpenVINS child process for {reason} with pid={self._process.pid}"
        )

    def _stop_process_locked(self, reason: str) -> None:
        if self._process is None:
            return

        if self._process.poll() is not None:
            self.get_logger().info(
                f"OpenVINS child process already exited before {reason}"
            )
            self._process = None
            return

        self.get_logger().info(
            f"stopping OpenVINS child process for {reason} pid={self._process.pid}"
        )

        try:
            if os.name == "posix":
                os.killpg(os.getpgid(self._process.pid), signal.SIGINT)
            else:
                self._process.send_signal(signal.SIGINT)
            self._process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warning("OpenVINS child process did not exit after SIGINT, escalating")
            try:
                if os.name == "posix":
                    os.killpg(os.getpgid(self._process.pid), signal.SIGTERM)
                else:
                    self._process.terminate()
                self._process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.get_logger().error("OpenVINS child process did not exit after SIGTERM, killing")
                if os.name == "posix":
                    os.killpg(os.getpgid(self._process.pid), signal.SIGKILL)
                else:
                    self._process.kill()
                self._process.wait(timeout=5.0)

        self._process = None

    def _publish_reset_counter_bump(self) -> None:
        self._reset_counter_bump_pub.publish(Empty())
        self.get_logger().info(
            f"published OpenVINS reset counter bump on {self._reset_counter_bump_topic}"
        )

    def _handle_reset(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request

        try:
            with self._process_lock:
                self._stop_process_locked("reset request")
                self._publish_reset_counter_bump()
                self._start_process_locked("reset request")
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"failed to reset OpenVINS: {exc}"
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = f"OpenVINS reset completed in namespace {self._child_namespace}"
        return response


def main() -> None:
    rclpy.init()
    node = OpenVinsSupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
