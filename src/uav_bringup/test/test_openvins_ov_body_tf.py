import importlib.util
import math
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
OPENVINS_LAUNCH = REPO_ROOT / "launch" / "openvins_orbbec.launch.py"
OV_MONO_TAG_LAUNCH = REPO_ROOT / "launch" / "openvins_orbbec_mono_apriltag.launch.py"

DEFAULT_SENSOR_ROLL_IN_BODY_RAD = 0.0
DEFAULT_SENSOR_PITCH_IN_BODY_RAD = -math.pi / 2.0
DEFAULT_SENSOR_YAW_IN_BODY_RAD = math.pi / 2.0


def _load_module(module_path: Path, module_name: str):
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _transpose(matrix):
    return tuple(tuple(matrix[col][row] for col in range(3)) for row in range(3))


def _matmul(lhs, rhs):
    return tuple(
        tuple(sum(lhs[row][k] * rhs[k][col] for k in range(3)) for col in range(3))
        for row in range(3)
    )


def _quat_to_matrix(quaternion):
    x, y, z, w = quaternion
    norm = math.sqrt((x * x) + (y * y) + (z * z) + (w * w))
    x /= norm
    y /= norm
    z /= norm
    w /= norm
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


def _assert_matrix_close(lhs, rhs, tol=1.0e-9):
    for row in range(3):
        for col in range(3):
            assert math.isclose(lhs[row][col], rhs[row][col], abs_tol=tol), (
                row,
                col,
                lhs[row][col],
                rhs[row][col],
            )


def test_default_ov_body_transform_is_exact_inverse_of_body_to_sensor():
    module = _load_module(OPENVINS_LAUNCH, "openvins_orbbec_launch")

    body_to_sensor = module._rotation_matrix_from_rpy(
        DEFAULT_SENSOR_ROLL_IN_BODY_RAD,
        DEFAULT_SENSOR_PITCH_IN_BODY_RAD,
        DEFAULT_SENSOR_YAW_IN_BODY_RAD,
    )
    sensor_to_body = _transpose(body_to_sensor)

    identity = _matmul(body_to_sensor, sensor_to_body)
    _assert_matrix_close(
        identity,
        (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
    )

    body_to_sensor_quat = module._quat_from_rpy(
        DEFAULT_SENSOR_ROLL_IN_BODY_RAD,
        DEFAULT_SENSOR_PITCH_IN_BODY_RAD,
        DEFAULT_SENSOR_YAW_IN_BODY_RAD,
    )
    sensor_to_body_quat = (
        -body_to_sensor_quat[0],
        -body_to_sensor_quat[1],
        -body_to_sensor_quat[2],
        body_to_sensor_quat[3],
    )
    sensor_to_body_from_quat = _quat_to_matrix(sensor_to_body_quat)
    _assert_matrix_close(sensor_to_body, sensor_to_body_from_quat)

    _assert_matrix_close(
        body_to_sensor,
        (
            (0.0, -1.0, 0.0),
            (0.0, 0.0, -1.0),
            (1.0, 0.0, 0.0),
        ),
    )


def test_launch_files_expose_and_route_ov_body_frame_defaults():
    openvins_text = OPENVINS_LAUNCH.read_text()
    assert 'DeclareLaunchArgument("ov_body_frame_id", default_value="uav_ov_body")' in openvins_text
    assert 'DeclareLaunchArgument("publish_ov_body_tf", default_value="true")' in openvins_text
    assert 'name="uav_openvins_ov_body_static_tf"' in openvins_text

    ov_mono_tag_text = OV_MONO_TAG_LAUNCH.read_text()
    assert '"base_frame_id": ov_body_frame_id' in ov_mono_tag_text
    assert (
        'DeclareLaunchArgument("camera_frame_id", default_value="uav_ov_mono_camera_optical_frame")'
        in ov_mono_tag_text
    )
