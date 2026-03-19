#!/usr/bin/env python3

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

import rclpy
from orbbec_camera_msgs.msg import Extrinsics, IMUInfo
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo

DEFAULT_ACCEL_NOISE_DENSITY = 0.00207649074
DEFAULT_ACCEL_RANDOM_WALK = 0.00041327852
DEFAULT_GYRO_NOISE_DENSITY = 0.00020544166
DEFAULT_GYRO_RANDOM_WALK = 0.00001110622
DEFAULT_MASK_CORNER_WIDTH_RATIO = 0.10
DEFAULT_MASK_TOP_Y_RATIO = 0.85

Transform = Tuple[List[List[float]], List[float]]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate OpenVINS bootstrap configs from live Orbbec topics.")
    parser.add_argument("--camera-name", default="uav_depth_camera")
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    parser.add_argument("--camera-rate-hz", type=float, default=30.0)
    parser.add_argument("--imu-rate-hz", type=float, default=200.0)
    parser.add_argument("--imu-reference", choices=["gyro", "accel"], default="gyro")
    parser.add_argument("--left-camera-info-topic", default="")
    parser.add_argument("--right-camera-info-topic", default="")
    parser.add_argument("--depth-to-left-ir-topic", default="")
    parser.add_argument("--depth-to-right-ir-topic", default="")
    parser.add_argument("--depth-to-gyro-topic", default="")
    parser.add_argument("--depth-to-accel-topic", default="")
    parser.add_argument("--gyro-info-topic", default="")
    parser.add_argument("--accel-info-topic", default="")
    return parser.parse_args()


class BootstrapCollector(Node):
    def __init__(self, topics: Dict[str, Tuple[str, object, bool]]) -> None:
        super().__init__("orbbec_openvins_bootstrap_generator")
        self.required_keys = {key for key, (_, _, required) in topics.items() if required}
        self.messages: Dict[str, object] = {}
        self._bootstrap_subscriptions = []
        extrinsics_qos = QoSProfile(depth=1)
        extrinsics_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        extrinsics_qos.reliability = ReliabilityPolicy.RELIABLE
        default_qos = 10

        for key, (topic_name, msg_type, _) in topics.items():
            qos = extrinsics_qos if msg_type is Extrinsics else default_qos
            subscription = self.create_subscription(
                msg_type,
                topic_name,
                lambda message, topic_key=key: self._on_message(topic_key, message),
                qos,
            )
            self._bootstrap_subscriptions.append(subscription)
            self.get_logger().info(f"waiting for {key}: {topic_name}")

    def _on_message(self, key: str, message: object) -> None:
        if key in self.messages:
            return
        self.messages[key] = message
        self.get_logger().info(f"received {key}")

    def has_required_messages(self) -> bool:
        return self.required_keys.issubset(self.messages.keys())


class BootstrapError(RuntimeError):
    pass


class BootstrapWarning:
    def __init__(self, text: str) -> None:
        self.text = text


def fill_topic_defaults(args: argparse.Namespace) -> None:
    base_topic = f"/{args.camera_name}"
    if not args.left_camera_info_topic:
        args.left_camera_info_topic = f"{base_topic}/left_ir/camera_info"
    if not args.right_camera_info_topic:
        args.right_camera_info_topic = f"{base_topic}/right_ir/camera_info"
    if not args.depth_to_left_ir_topic:
        args.depth_to_left_ir_topic = f"{base_topic}/depth_to_left_ir"
    if not args.depth_to_right_ir_topic:
        args.depth_to_right_ir_topic = f"{base_topic}/depth_to_right_ir"
    if not args.depth_to_gyro_topic:
        args.depth_to_gyro_topic = f"{base_topic}/depth_to_gyro"
    if not args.depth_to_accel_topic:
        args.depth_to_accel_topic = f"{base_topic}/depth_to_accel"
    if not args.gyro_info_topic:
        args.gyro_info_topic = f"{base_topic}/gyro/imu_info"
    if not args.accel_info_topic:
        args.accel_info_topic = f"{base_topic}/accel/imu_info"


def matrix_from_flat(values: List[float]) -> List[List[float]]:
    return [
        [float(values[0]), float(values[1]), float(values[2])],
        [float(values[3]), float(values[4]), float(values[5])],
        [float(values[6]), float(values[7]), float(values[8])],
    ]


def transpose_matrix(rotation: List[List[float]]) -> List[List[float]]:
    return [[rotation[column][row] for column in range(3)] for row in range(3)]


def multiply_matrices(left: List[List[float]], right: List[List[float]]) -> List[List[float]]:
    result = []
    for row in range(3):
        result_row = []
        for column in range(3):
            result_row.append(
                left[row][0] * right[0][column]
                + left[row][1] * right[1][column]
                + left[row][2] * right[2][column]
            )
        result.append(result_row)
    return result


def multiply_matrix_vector(rotation: List[List[float]], vector: List[float]) -> List[float]:
    return [
        rotation[0][0] * vector[0] + rotation[0][1] * vector[1] + rotation[0][2] * vector[2],
        rotation[1][0] * vector[0] + rotation[1][1] * vector[1] + rotation[1][2] * vector[2],
        rotation[2][0] * vector[0] + rotation[2][1] * vector[1] + rotation[2][2] * vector[2],
    ]


def invert_transform(transform: Transform) -> Transform:
    rotation, translation = transform
    rotation_inverse = transpose_matrix(rotation)
    rotated_translation = multiply_matrix_vector(rotation_inverse, translation)
    return rotation_inverse, [-value for value in rotated_translation]


def compose_transforms(first: Transform, second: Transform) -> Transform:
    first_rotation, first_translation = first
    second_rotation, second_translation = second
    rotation = multiply_matrices(first_rotation, second_rotation)
    translated = multiply_matrix_vector(first_rotation, second_translation)
    translation = [translated[index] + first_translation[index] for index in range(3)]
    return rotation, translation


def transform_from_extrinsics(message: Extrinsics) -> Transform:
    return matrix_from_flat(list(message.rotation)), [float(value) for value in message.translation]


def transform_from_orbbec_optical_extrinsics(message: Extrinsics) -> Transform:
    """Interpret Orbbec stream extrinsics with the same optical-frame semantics as ROS topics.

    The Orbbec ROS driver applies the same fixed stream->optical rotation to every camera and IMU
    stream frame before publishing image and IMU topics. For transforms between those optical
    frames, the raw stream-to-stream calibration published in `Extrinsics` is numerically
    equivalent to the optical-frame transform used by OpenVINS.
    """
    return transform_from_extrinsics(message)


def transform_delta(first: Transform, second: Transform) -> Tuple[float, float]:
    first_inverse = invert_transform(first)
    delta_rotation, delta_translation = compose_transforms(first_inverse, second)
    trace_value = delta_rotation[0][0] + delta_rotation[1][1] + delta_rotation[2][2]
    cosine = max(-1.0, min(1.0, (trace_value - 1.0) * 0.5))
    angle_deg = abs(math.degrees(math.acos(cosine)))
    translation_norm = sum(value * value for value in delta_translation) ** 0.5
    return angle_deg, translation_norm


def rotation_deviation_deg(rotation: List[List[float]]) -> float:
    trace_value = rotation[0][0] + rotation[1][1] + rotation[2][2]
    cosine = max(-1.0, min(1.0, (trace_value - 1.0) * 0.5))
    return abs(math.degrees(math.acos(cosine)))


def map_distortion_model(model_name: str) -> str:
    normalized = model_name.strip().lower()
    if normalized in {"plumb_bob", "radtan"}:
        return "radtan"
    if normalized in {"equidistant", "fisheye"}:
        return "equidistant"
    return "radtan"


def extract_camera_parameters(message: CameraInfo) -> Dict[str, object]:
    distortion_coeffs = [float(value) for value in list(message.d)[:4]]
    while len(distortion_coeffs) < 4:
        distortion_coeffs.append(0.0)
    return {
        "intrinsics": [float(message.k[0]), float(message.k[4]), float(message.k[2]), float(message.k[5])],
        "resolution": [int(message.width), int(message.height)],
        "distortion_model": map_distortion_model(message.distortion_model),
        "distortion_coeffs": distortion_coeffs,
    }


def format_scalar(value: float) -> str:
    return f"{value:.12g}"


def format_list(values: List[float]) -> str:
    return "[" + ", ".join(format_scalar(value) for value in values) + "]"


def format_transform(transform: Transform) -> List[str]:
    rotation, translation = transform
    return [
        f"    - [{format_scalar(rotation[0][0])}, {format_scalar(rotation[0][1])}, {format_scalar(rotation[0][2])}, {format_scalar(translation[0])}]",
        f"    - [{format_scalar(rotation[1][0])}, {format_scalar(rotation[1][1])}, {format_scalar(rotation[1][2])}, {format_scalar(translation[1])}]",
        f"    - [{format_scalar(rotation[2][0])}, {format_scalar(rotation[2][1])}, {format_scalar(rotation[2][2])}, {format_scalar(translation[2])}]",
        "    - [0.0, 0.0, 0.0, 1.0]",
    ]


def estimator_config_text(enable_online_calibration: bool, camera_rate_hz: float) -> str:
    extrinsics_value = "true" if enable_online_calibration else "false"
    timeoffset_value = "true" if enable_online_calibration else "false"
    return f'''%YAML:1.0

verbosity: "INFO"

use_fej: true
integration: "rk4"
use_stereo: true
max_cameras: 2

calib_cam_extrinsics: {extrinsics_value}
calib_cam_intrinsics: false
calib_cam_timeoffset: {timeoffset_value}
calib_imu_intrinsics: false
calib_imu_g_sensitivity: false

max_clones: 11
max_slam: 50
max_slam_in_update: 25
max_msckf_in_update: 40
dt_slam_delay: 1

gravity_mag: 9.81

feat_rep_msckf: "GLOBAL_3D"
feat_rep_slam: "ANCHORED_MSCKF_INVERSE_DEPTH"
feat_rep_aruco: "ANCHORED_MSCKF_INVERSE_DEPTH"

try_zupt: false
zupt_chi2_multipler: 0
zupt_max_velocity: 0.1
zupt_noise_multiplier: 10
zupt_max_disparity: 0.5
zupt_only_at_beginning: false

init_window_time: 2.0
init_imu_thresh: 1.5
init_max_disparity: 15.0
init_max_features: 80

init_dyn_use: false
init_dyn_mle_opt_calib: false
init_dyn_mle_max_iter: 50
init_dyn_mle_max_time: 0.05
init_dyn_mle_max_threads: 4
init_dyn_num_pose: 6
init_dyn_min_deg: 10.0

init_dyn_inflation_ori: 10
init_dyn_inflation_vel: 100
init_dyn_inflation_bg: 10
init_dyn_inflation_ba: 100
init_dyn_min_rec_cond: 1e-12

init_dyn_bias_g: [0.0, 0.0, 0.0]
init_dyn_bias_a: [0.0, 0.0, 0.0]

record_timing_information: false
record_timing_filepath: "/tmp/traj_timing.txt"

save_total_state: false
filepath_est: "/tmp/ov_estimate.txt"
filepath_std: "/tmp/ov_estimate_std.txt"
filepath_gt: "/tmp/ov_groundtruth.txt"

use_klt: true
num_pts: 220
fast_threshold: 20
grid_x: 5
grid_y: 5
min_px_dist: 15
knn_ratio: 0.70
track_frequency: {int(round(camera_rate_hz))}
downsample_cameras: false
num_opencv_threads: 2
histogram_method: "CLAHE"

use_aruco: false
num_aruco: 1024
downsize_aruco: true

up_msckf_sigma_px: 1
up_msckf_chi2_multipler: 1
up_slam_sigma_px: 1
up_slam_chi2_multipler: 1
up_aruco_sigma_px: 1
up_aruco_chi2_multipler: 1

use_mask: true
mask0: "mask0.pgm"
mask1: "mask1.pgm"

relative_config_imu: "kalibr_imu_chain.yaml"
relative_config_imucam: "kalibr_imucam_chain.yaml"
'''


def imucam_chain_text(
    left_camera: Dict[str, object],
    right_camera: Dict[str, object],
    left_transform: Transform,
    right_transform: Transform,
    stereo_transform: Transform,
    left_topic: str,
    right_topic: str,
) -> str:
    left_lines = format_transform(left_transform)
    right_lines = format_transform(right_transform)
    stereo_lines = format_transform(stereo_transform)
    return "\n".join(
        [
            "%YAML:1.0",
            "",
            "cam0:",
            "  T_cam_imu:",
            *left_lines,
            "  cam_overlaps: [1]",
            "  camera_model: pinhole",
            f"  distortion_coeffs: {format_list(left_camera['distortion_coeffs'])}",
            f"  distortion_model: {left_camera['distortion_model']}",
            f"  intrinsics: {format_list(left_camera['intrinsics'])}",
            f"  resolution: [{left_camera['resolution'][0]}, {left_camera['resolution'][1]}]",
            f"  rostopic: {left_topic}",
            "  timeshift_cam_imu: 0.0",
            "",
            "cam1:",
            "  T_cam_imu:",
            *right_lines,
            "  T_cn_cnm1:",
            *stereo_lines,
            "  cam_overlaps: [0]",
            "  camera_model: pinhole",
            f"  distortion_coeffs: {format_list(right_camera['distortion_coeffs'])}",
            f"  distortion_model: {right_camera['distortion_model']}",
            f"  intrinsics: {format_list(right_camera['intrinsics'])}",
            f"  resolution: [{right_camera['resolution'][0]}, {right_camera['resolution'][1]}]",
            f"  rostopic: {right_topic}",
            "  timeshift_cam_imu: 0.0",
            "",
        ]
    )


def imu_chain_text(
    imu_topic: str,
    accel_noise_density: float,
    accel_random_walk: float,
    gyro_noise_density: float,
    gyro_random_walk: float,
    imu_rate_hz: float,
) -> str:
    return f'''%YAML:1.0

imu0:
  T_i_b:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  accelerometer_noise_density: {format_scalar(accel_noise_density)}
  accelerometer_random_walk: {format_scalar(accel_random_walk)}
  gyroscope_noise_density: {format_scalar(gyro_noise_density)}
  gyroscope_random_walk: {format_scalar(gyro_random_walk)}
  rostopic: {imu_topic}
  time_offset: 0.0
  update_rate: {int(round(imu_rate_hz))}
  model: "kalibr"
  Tw:
    - [1.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0]
    - [0.0, 0.0, 1.0]
  R_IMUtoGYRO:
    - [1.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0]
    - [0.0, 0.0, 1.0]
  Ta:
    - [1.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0]
    - [0.0, 0.0, 1.0]
  R_IMUtoACC:
    - [1.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0]
    - [0.0, 0.0, 1.0]
  Tg:
    - [0.0, 0.0, 0.0]
    - [0.0, 0.0, 0.0]
    - [0.0, 0.0, 0.0]
'''


def build_default_mask(width: int, height: int) -> bytes:
    if width <= 0 or height <= 0:
        raise BootstrapError(f"invalid mask resolution: {width}x{height}")

    corner_width = max(1, int(round(width * DEFAULT_MASK_CORNER_WIDTH_RATIO)))
    top_y = min(height - 1, max(0, int(round(height * DEFAULT_MASK_TOP_Y_RATIO))))
    slope = 0.0 if corner_width <= 0 else (height - 1 - top_y) / float(corner_width)

    pixels = bytearray(width * height)
    for y in range(height):
        row_offset = y * width
        for x in range(width):
            mask_pixel = False
            if x <= corner_width:
                edge_y = top_y + slope * x
                mask_pixel = y >= edge_y
            elif x >= width - 1 - corner_width:
                edge_y = top_y + slope * (width - 1 - x)
                mask_pixel = y >= edge_y
            if mask_pixel:
                pixels[row_offset + x] = 255

    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    return header + bytes(pixels)


def write_mask(path: Path, width: int, height: int) -> None:
    path.write_bytes(build_default_mask(width, height))
    print(f"wrote {path}")


def add_sanity_warnings(
    warnings: List[BootstrapWarning],
    left_camera: Dict[str, object],
    right_camera: Dict[str, object],
    stereo_transform: Transform,
) -> None:
    left_resolution = tuple(left_camera["resolution"])
    right_resolution = tuple(right_camera["resolution"])
    if left_resolution != right_resolution:
        warnings.append(
            BootstrapWarning(
                f"left/right resolution mismatch: {left_resolution} vs {right_resolution}; verify the stream configuration before flight."
            )
        )

    stereo_baseline = sum(value * value for value in stereo_transform[1]) ** 0.5
    if stereo_baseline < 0.045 or stereo_baseline > 0.055:
        warnings.append(
            BootstrapWarning(
                "stereo baseline is outside the expected Gemini 336 range "
                f"({format_scalar(stereo_baseline)} m); verify factory extrinsics and stream selection."
            )
        )

    stereo_rotation_deg = rotation_deviation_deg(stereo_transform[0])
    if stereo_rotation_deg > 2.0:
        warnings.append(
            BootstrapWarning(
                "stereo relative rotation is larger than expected "
                f"({format_scalar(stereo_rotation_deg)} deg); verify the generated camera chain before using it for flight."
            )
        )


def summary_text(
    args: argparse.Namespace,
    output_dir: Path,
    warnings: List[BootstrapWarning],
    left_camera: Dict[str, object],
    right_camera: Dict[str, object],
    stereo_transform: Transform,
) -> str:
    stereo_baseline = sum(value * value for value in stereo_transform[1]) ** 0.5
    stereo_rotation_deg = rotation_deviation_deg(stereo_transform[0])
    lines = [
        "# OpenVINS Orbbec Bootstrap Summary",
        "",
        f"- Camera namespace: `{args.camera_name}`",
        f"- IMU reference: `{args.imu_reference}`",
        f"- Left camera topic: `{args.left_camera_info_topic}`",
        f"- Right camera topic: `{args.right_camera_info_topic}`",
        f"- Output directory: `{output_dir}`",
        f"- Left resolution: `{left_camera['resolution'][0]}x{left_camera['resolution'][1]}`",
        f"- Right resolution: `{right_camera['resolution'][0]}x{right_camera['resolution'][1]}`",
        f"- Stereo baseline estimate: `{format_scalar(stereo_baseline)}` m",
        f"- Stereo rotation deviation: `{format_scalar(stereo_rotation_deg)}` deg",
        "",
        "## Files",
        "",
        "- `estimator_config.calibration.yaml`: online calibration profile",
        "- `estimator_config.flight.yaml`: frozen flight profile",
        "- `kalibr_imucam_chain.yaml`: camera/IMU bootstrap chain",
        "- `kalibr_imu_chain.yaml`: IMU noise bootstrap chain",
        "- `mask0.pgm`, `mask1.pgm`: conservative lower-corner masks generated from current resolution",
        "",
        "## Notes",
        "",
        "- Camera and IMU transforms are interpreted with the same optical-frame semantics as the published ROS topics.",
        "- Intrinsics stay frozen during online calibration; only camera extrinsics and camera/IMU time offset are enabled.",
        "- Generated masks only suppress the lower-corner airframe intrusion and should be refined later if the mounted airframe changes.",
        "",
        "## Warnings",
        "",
    ]
    if warnings:
        lines.extend(f"- {warning.text}" for warning in warnings)
    else:
        lines.append("- No warnings.")
    lines.append("")
    return "\n".join(lines)


def resolve_output_dir(argument_value: str) -> Path:
    if argument_value:
        return Path(argument_value).expanduser().resolve()
    return (Path.cwd() / "openvins_orbbec_bootstrap").resolve()


def write_text(path: Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")
    print(f"wrote {path}")


def main() -> int:
    args = parse_args()
    fill_topic_defaults(args)

    rclpy.init(args=None)
    topics = {
        "left_camera_info": (args.left_camera_info_topic, CameraInfo, True),
        "right_camera_info": (args.right_camera_info_topic, CameraInfo, True),
        "depth_to_left_ir": (args.depth_to_left_ir_topic, Extrinsics, True),
        "depth_to_right_ir": (args.depth_to_right_ir_topic, Extrinsics, True),
        "depth_to_gyro": (args.depth_to_gyro_topic, Extrinsics, args.imu_reference == "gyro"),
        "depth_to_accel": (args.depth_to_accel_topic, Extrinsics, args.imu_reference == "accel"),
        "gyro_info": (args.gyro_info_topic, IMUInfo, False),
        "accel_info": (args.accel_info_topic, IMUInfo, False),
    }

    node = BootstrapCollector(topics)
    deadline = time.monotonic() + args.timeout_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if node.has_required_messages():
                break
            rclpy.spin_once(node, timeout_sec=0.1)

        if not node.has_required_messages():
            missing = sorted(node.required_keys.difference(node.messages.keys()))
            raise BootstrapError(f"timed out waiting for required topics: {', '.join(missing)}")

        warnings: List[BootstrapWarning] = []
        left_camera = extract_camera_parameters(node.messages["left_camera_info"])
        right_camera = extract_camera_parameters(node.messages["right_camera_info"])
        left_depth_transform = transform_from_orbbec_optical_extrinsics(node.messages["depth_to_left_ir"])
        right_depth_transform = transform_from_orbbec_optical_extrinsics(node.messages["depth_to_right_ir"])

        if args.imu_reference == "gyro":
            imu_depth_transform = transform_from_orbbec_optical_extrinsics(node.messages["depth_to_gyro"])
            imu_topic = f"/{args.camera_name}/gyro_accel/sample"
        else:
            imu_depth_transform = transform_from_orbbec_optical_extrinsics(node.messages["depth_to_accel"])
            imu_topic = f"/{args.camera_name}/gyro_accel/sample"

        if "depth_to_gyro" in node.messages and "depth_to_accel" in node.messages:
            rotation_delta_deg, translation_delta_m = transform_delta(
                transform_from_orbbec_optical_extrinsics(node.messages["depth_to_gyro"]),
                transform_from_orbbec_optical_extrinsics(node.messages["depth_to_accel"]),
            )
            if rotation_delta_deg > 1.0 or translation_delta_m > 0.005:
                warnings.append(
                    BootstrapWarning(
                        "gyro and accel factory extrinsics differ noticeably "
                        f"({format_scalar(rotation_delta_deg)} deg, {format_scalar(translation_delta_m)} m); "
                        f"using `{args.imu_reference}` as the IMU reference frame."
                    )
                )

        imu_depth_inverse = invert_transform(imu_depth_transform)
        left_cam_imu = compose_transforms(left_depth_transform, imu_depth_inverse)
        right_cam_imu = compose_transforms(right_depth_transform, imu_depth_inverse)
        stereo_transform = compose_transforms(right_cam_imu, invert_transform(left_cam_imu))

        add_sanity_warnings(warnings, left_camera, right_camera, stereo_transform)

        accel_noise_density = DEFAULT_ACCEL_NOISE_DENSITY
        accel_random_walk = DEFAULT_ACCEL_RANDOM_WALK
        gyro_noise_density = DEFAULT_GYRO_NOISE_DENSITY
        gyro_random_walk = DEFAULT_GYRO_RANDOM_WALK

        accel_info = node.messages.get("accel_info")
        if isinstance(accel_info, IMUInfo):
            accel_noise_density = float(accel_info.noise_density)
            accel_random_walk = float(accel_info.random_walk)
        else:
            warnings.append(BootstrapWarning("accel/imu_info was not received; using fallback accelerometer noise values."))

        gyro_info = node.messages.get("gyro_info")
        if isinstance(gyro_info, IMUInfo):
            gyro_noise_density = float(gyro_info.noise_density)
            gyro_random_walk = float(gyro_info.random_walk)
        else:
            warnings.append(BootstrapWarning("gyro/imu_info was not received; using fallback gyroscope noise values."))

        output_dir = resolve_output_dir(args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        write_text(output_dir / "estimator_config.calibration.yaml", estimator_config_text(True, args.camera_rate_hz))
        write_text(output_dir / "estimator_config.flight.yaml", estimator_config_text(False, args.camera_rate_hz))
        write_text(
            output_dir / "kalibr_imucam_chain.yaml",
            imucam_chain_text(
                left_camera,
                right_camera,
                left_cam_imu,
                right_cam_imu,
                stereo_transform,
                f"/{args.camera_name}/left_ir/image_raw",
                f"/{args.camera_name}/right_ir/image_raw",
            ),
        )
        write_text(
            output_dir / "kalibr_imu_chain.yaml",
            imu_chain_text(
                imu_topic,
                accel_noise_density,
                accel_random_walk,
                gyro_noise_density,
                gyro_random_walk,
                args.imu_rate_hz,
            ),
        )
        write_mask(output_dir / "mask0.pgm", left_camera["resolution"][0], left_camera["resolution"][1])
        write_mask(output_dir / "mask1.pgm", right_camera["resolution"][0], right_camera["resolution"][1])
        write_text(
            output_dir / "bootstrap_summary.md",
            summary_text(args, output_dir, warnings, left_camera, right_camera, stereo_transform),
        )
    except BootstrapError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
