import math
import importlib.util
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO_ROOT / "uav_bringup" / "demo_search_coordinator.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "demo_search_coordinator", MODULE_PATH
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_parse_waypoint_deltas_accepts_xy_and_xyz_segments():
    module = _load_module()
    waypoints = module.parse_waypoint_deltas("1.0,0.0; 0.5 -0.2 0.3")

    assert len(waypoints) == 2
    assert waypoints[0].x == 1.0
    assert waypoints[0].y == 0.0
    assert waypoints[0].z == 0.0
    assert waypoints[1].x == 0.5
    assert waypoints[1].y == -0.2
    assert waypoints[1].z == 0.3


def test_parse_waypoint_deltas_rejects_bad_segment_lengths():
    module = _load_module()
    try:
        module.parse_waypoint_deltas("1.0")
    except ValueError as exc:
        assert "2 or 3" in str(exc)
    else:  # pragma: no cover - defensive branch
        raise AssertionError("expected ValueError for malformed waypoint")


def test_compute_relative_goal_from_global_delta_rotates_into_ugv_body():
    module = _load_module()
    goal_x, goal_y = module.compute_relative_goal_from_global_delta(
        tag_global_xy=(4.0, 5.0),
        uav_global_xy=(1.0, 2.0),
        ugv_relative_from_uav_global_xy=(2.0, 1.0),
        ugv_yaw_global_rad=math.pi / 2.0,
    )

    assert math.isclose(goal_x, 2.0, abs_tol=1.0e-6)
    assert math.isclose(goal_y, -1.0, abs_tol=1.0e-6)


def test_compute_relative_goal_from_global_delta_applies_body_offsets():
    module = _load_module()
    goal_x, goal_y = module.compute_relative_goal_from_global_delta(
        tag_global_xy=(5.0, 1.0),
        uav_global_xy=(1.0, 1.0),
        ugv_relative_from_uav_global_xy=(1.0, 0.0),
        ugv_yaw_global_rad=0.0,
        goal_offset_ugv_body_xy=(0.2, -0.1),
    )

    assert math.isclose(goal_x, 3.2, abs_tol=1.0e-6)
    assert math.isclose(goal_y, -0.1, abs_tol=1.0e-6)


def test_compute_body_delta_target_rotates_body_step_into_local_frame():
    module = _load_module()
    target_x, target_y, target_z = module.compute_body_delta_target(
        current_position_xyz=(2.0, 3.0, 0.5),
        current_yaw_rad=math.pi / 2.0,
        delta_body_xyz=(1.0, 0.0, -0.2),
    )

    assert math.isclose(target_x, 2.0, abs_tol=1.0e-6)
    assert math.isclose(target_y, 4.0, abs_tol=1.0e-6)
    assert math.isclose(target_z, 0.3, abs_tol=1.0e-6)
