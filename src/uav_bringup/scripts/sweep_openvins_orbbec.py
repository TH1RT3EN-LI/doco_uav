#!/usr/bin/env python3

from __future__ import annotations

import argparse
import ast
import csv
import json
import math
import os
import re
import shutil
import signal
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


DEFAULT_PHASE1_BASE_RELATIVE = Path("config/openvins/orbbec_gemini336/estimator_config.diag_timeoffset.yaml")
DEFAULT_PHASE2_BASE_RELATIVE = Path("config/openvins/orbbec_gemini336/estimator_config.phase2_seed.yaml")
DEFAULT_LEFT_TOPIC = "/uav_depth_camera/left_ir/image_raw"
DEFAULT_RIGHT_TOPIC = "/uav_depth_camera/right_ir/image_raw"
DEFAULT_IMU_TOPIC = "/uav_depth_camera/gyro_accel/sample"
DEFAULT_NAMESPACE = "/ov_msckf_eval"

VALUE_RE = re.compile(r"^(?P<key>[A-Za-z0-9_]+)\s*:\s*(?P<value>.*?)(?:\s+#.*)?$")


@dataclass(frozen=True)
class SemanticOption:
    name: str
    description: str
    overrides: Dict[str, Any]


@dataclass(frozen=True)
class Preset:
    index: int
    name: str
    family: str
    description: str
    overrides: Dict[str, Any]


def debug(msg: str) -> None:
    print(msg, flush=True)


def find_package_share() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("uav_bringup")).resolve()
    except Exception:
        script_path = Path(__file__).resolve()
        source_root = script_path.parents[1]
        if (source_root / "config").exists():
            return source_root
        raise RuntimeError("Unable to locate uav_bringup package share; source the workspace or pass --base-config explicitly")


def find_source_root_from_script() -> Optional[Path]:
    script_path = Path(__file__).resolve()
    source_root = script_path.parents[1]
    if (source_root / "config").exists():
        return source_root
    return None


def ensure_ros2_available() -> None:
    if shutil.which("ros2") is None:
        raise RuntimeError("Unable to find `ros2` in PATH. Source the ROS workspace inside the container before running.")


def load_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def parse_scalar(raw: str) -> Any:
    value = raw.strip()
    if not value:
        return ""
    if value.lower() == "true":
        return True
    if value.lower() == "false":
        return False
    try:
        return ast.literal_eval(value)
    except Exception:
        return value


def parse_top_level_config(text: str) -> Dict[str, Any]:
    values: Dict[str, Any] = {}
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or stripped.startswith("%"):
            continue
        if line[:1].isspace():
            continue
        match = VALUE_RE.match(line)
        if match:
            values[match.group("key")] = parse_scalar(match.group("value"))
    return values


def format_value(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int) and not isinstance(value, bool):
        return str(value)
    if isinstance(value, float):
        if math.isnan(value) or math.isinf(value):
            raise ValueError(f"Invalid float value {value}")
        text = f"{value:.15g}"
        if "." not in text and "e" not in text and "E" not in text:
            text += ".0"
        return text
    if isinstance(value, str):
        return json.dumps(value)
    if isinstance(value, (list, tuple)):
        return "[ " + ", ".join(format_value(v) for v in value) + " ]"
    raise TypeError(f"Unsupported YAML value type: {type(value)!r}")


def replace_value(text: str, key: str, value: Any) -> str:
    pattern = re.compile(rf"^(?P<prefix>\s*{re.escape(key)}\s*:\s*)(?P<value>.*?)(?P<suffix>\s*(#.*)?)$", re.MULTILINE)

    def repl(match: re.Match[str]) -> str:
        return f"{match.group('prefix')}{format_value(value)}{match.group('suffix')}"

    new_text, count = pattern.subn(repl, text, count=1)
    if count != 1:
        raise KeyError(f"Failed to find top-level key `{key}` in template config")
    return new_text


def percentile(values: Sequence[float], p: float) -> Optional[float]:
    if not values:
        return None
    if len(values) == 1:
        return values[0]
    ordered = sorted(values)
    pos = (len(ordered) - 1) * (p / 100.0)
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return ordered[lo]
    blend = pos - lo
    return ordered[lo] * (1.0 - blend) + ordered[hi] * blend


def mean(values: Sequence[float]) -> Optional[float]:
    if not values:
        return None
    return statistics.fmean(values)


def read_log_tail(path: Path, max_lines: int = 30) -> Optional[str]:
    if not path.exists():
        return None
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except Exception:
        return None
    if not lines:
        return None
    return "\n".join(lines[-max_lines:])


def stddev(values: Sequence[float]) -> Optional[float]:
    if len(values) < 2:
        return 0.0 if values else None
    return statistics.pstdev(values)


def sanitize_name(text: str) -> str:
    slug = re.sub(r"[^a-zA-Z0-9]+", "_", text.strip()).strip("_").lower()
    return slug or "preset"


def clamp_int(value: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(value)))


def clamp_float(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(value)))


def canonical_overrides(overrides: Dict[str, Any]) -> str:
    return json.dumps(overrides, sort_keys=True, separators=(",", ":"))


def build_option_groups(base: Dict[str, Any]) -> Dict[str, List[SemanticOption]]:
    histogram_method = str(base.get("histogram_method", "HISTOGRAM"))
    use_mask = bool(base.get("use_mask", True))
    groups = {
        "calibration": [
            SemanticOption(
                "timeoffset_only",
                "keep online time offset estimation only",
                {
                    "calib_cam_timeoffset": bool(base.get("calib_cam_timeoffset", True)),
                    "calib_cam_extrinsics": bool(base.get("calib_cam_extrinsics", False)),
                },
            ),
            SemanticOption(
                "frozen_calibration",
                "freeze camera-IMU temporal and spatial calibration",
                {"calib_cam_timeoffset": False, "calib_cam_extrinsics": False},
            ),
            SemanticOption(
                "timeoffset_extrinsics",
                "estimate both time offset and camera-IMU extrinsics online",
                {"calib_cam_timeoffset": True, "calib_cam_extrinsics": True},
            ),
            SemanticOption(
                "extrinsics_probe",
                "keep time offset fixed while probing online extrinsics only",
                {"calib_cam_timeoffset": False, "calib_cam_extrinsics": True},
            ),
        ],
        "init": [
            SemanticOption(
                "strict_static_gate",
                "current strict standstill gate for vibration-heavy starts",
                {
                    "init_imu_thresh": float(base.get("init_imu_thresh", 0.3)),
                    "init_max_disparity": float(base.get("init_max_disparity", 2.0)),
                    "init_dyn_use": bool(base.get("init_dyn_use", False)),
                },
            ),
            SemanticOption(
                "balanced_static_gate",
                "less strict static gate to avoid rejecting mild motion before takeoff",
                {"init_imu_thresh": 0.6, "init_max_disparity": 4.0, "init_dyn_use": False},
            ),
            SemanticOption(
                "relaxed_static_gate",
                "relaxed standstill gate closer to flight defaults",
                {"init_imu_thresh": 1.0, "init_max_disparity": 6.0, "init_dyn_use": False},
            ),
            SemanticOption(
                "legacy_static_gate",
                "fully relaxed standstill gate matching the older, easier-to-init style",
                {"init_imu_thresh": 1.5, "init_max_disparity": 10.0, "init_dyn_use": False},
            ),
            SemanticOption(
                "dynamic_start_probe",
                "probe dynamic initialization for launches with unavoidable pre-takeoff motion",
                {"init_imu_thresh": 0.6, "init_max_disparity": 4.0, "init_dyn_use": True},
            ),
        ],
        "frontend": [
            SemanticOption(
                "sparse_strong_corners",
                "current sparse but strong-corner front-end",
                {
                    "num_pts": int(base.get("num_pts", 150)),
                    "fast_threshold": int(base.get("fast_threshold", 50)),
                    "min_px_dist": int(base.get("min_px_dist", 15)),
                },
            ),
            SemanticOption(
                "balanced_tracking",
                "balanced point density with moderate FAST threshold",
                {"num_pts": 200, "fast_threshold": 35, "min_px_dist": 15},
            ),
            SemanticOption(
                "reacquire_tracking",
                "favor faster feature reacquisition after vibration-induced dropouts",
                {"num_pts": 250, "fast_threshold": 25, "min_px_dist": 15},
            ),
            SemanticOption(
                "dense_recovery",
                "aggressive recovery with dense features and permissive corners",
                {"num_pts": 300, "fast_threshold": 20, "min_px_dist": 12},
            ),
            SemanticOption(
                "sparse_robust",
                "prefer spatially separated, stronger features over raw count",
                {"num_pts": 120, "fast_threshold": 40, "min_px_dist": 18},
            ),
        ],
        "visual_weight": [
            SemanticOption(
                "robust_updates",
                "current down-weighted visual residuals for vibration and blur",
                {
                    "up_msckf_sigma_px": float(base.get("up_msckf_sigma_px", 1.5)),
                    "up_slam_sigma_px": float(base.get("up_slam_sigma_px", 1.5)),
                },
            ),
            SemanticOption(
                "tight_updates",
                "trust pixel residuals more aggressively",
                {"up_msckf_sigma_px": 1.0, "up_slam_sigma_px": 1.0},
            ),
            SemanticOption(
                "loose_updates",
                "down-weight visual residuals further under blur or noisy features",
                {"up_msckf_sigma_px": 2.0, "up_slam_sigma_px": 2.0},
            ),
            SemanticOption(
                "very_loose_updates",
                "strongly distrust pixel residuals to favor inertial propagation continuity",
                {"up_msckf_sigma_px": 2.5, "up_slam_sigma_px": 2.5},
            ),
        ],
        "zupt": [
            SemanticOption(
                "strict_begin_only",
                "current begin-only ZUPT with moderate jitter tolerance",
                {
                    "try_zupt": bool(base.get("try_zupt", True)),
                    "zupt_max_velocity": float(base.get("zupt_max_velocity", 0.05)),
                    "zupt_noise_multiplier": float(base.get("zupt_noise_multiplier", 80.0)),
                    "zupt_max_disparity": float(base.get("zupt_max_disparity", 0.75)),
                    "zupt_only_at_beginning": bool(base.get("zupt_only_at_beginning", True)),
                },
            ),
            SemanticOption(
                "very_strict_begin_only",
                "favor only very clean standstill detections before takeoff",
                {
                    "try_zupt": True,
                    "zupt_max_velocity": 0.03,
                    "zupt_noise_multiplier": 60.0,
                    "zupt_max_disparity": 0.5,
                    "zupt_only_at_beginning": True,
                },
            ),
            SemanticOption(
                "tolerant_begin_only",
                "accept more rotor-induced jitter during early standstill detection",
                {
                    "try_zupt": True,
                    "zupt_max_velocity": 0.08,
                    "zupt_noise_multiplier": 100.0,
                    "zupt_max_disparity": 1.0,
                    "zupt_only_at_beginning": True,
                },
            ),
            SemanticOption(
                "disabled_zupt",
                "disable ZUPT entirely to see if false positives are hurting motion onset",
                {"try_zupt": False},
            ),
        ],
        "state": [
            SemanticOption(
                "baseline_window",
                "current state size and delayed SLAM activation",
                {
                    "max_clones": int(base.get("max_clones", 11)),
                    "max_slam": int(base.get("max_slam", 50)),
                    "max_slam_in_update": int(base.get("max_slam_in_update", 25)),
                    "max_msckf_in_update": int(base.get("max_msckf_in_update", 40)),
                    "dt_slam_delay": float(base.get("dt_slam_delay", 1.0)),
                },
            ),
            SemanticOption(
                "compact_window",
                "lighter state for lower compute and less delayed updates",
                {
                    "max_clones": 9,
                    "max_slam": 40,
                    "max_slam_in_update": 20,
                    "max_msckf_in_update": 30,
                    "dt_slam_delay": 0.5,
                },
            ),
            SemanticOption(
                "expanded_window",
                "more temporal context and more update capacity",
                {
                    "max_clones": 13,
                    "max_slam": 70,
                    "max_slam_in_update": 35,
                    "max_msckf_in_update": 50,
                    "dt_slam_delay": 1.5,
                },
            ),
            SemanticOption(
                "wide_context",
                "maximum context and SLAM budget for difficult segments",
                {
                    "max_clones": 15,
                    "max_slam": 90,
                    "max_slam_in_update": 45,
                    "max_msckf_in_update": 60,
                    "dt_slam_delay": 2.0,
                },
            ),
        ],
        "contrast": [
            SemanticOption(
                "current_contrast",
                "current image equalization and mask usage",
                {"histogram_method": histogram_method, "use_mask": use_mask},
            ),
            SemanticOption(
                "clahe_masked",
                "local contrast enhancement while keeping the rotor mask",
                {"histogram_method": "CLAHE", "use_mask": True},
            ),
            SemanticOption(
                "plain_masked",
                "disable histogram equalization while keeping the mask",
                {"histogram_method": "NONE", "use_mask": True},
            ),
            SemanticOption(
                "histogram_unmasked",
                "keep histogram equalization but remove the image mask",
                {"histogram_method": "HISTOGRAM", "use_mask": False},
            ),
        ],
    }
    return groups


def generate_phase1_presets(base: Dict[str, Any], count: int) -> List[Preset]:
    groups = build_option_groups(base)
    presets: List[Preset] = []
    seen: set[str] = set()

    def add_candidate(name: str, family: str, description: str, overrides: Dict[str, Any]) -> None:
        key = canonical_overrides(overrides)
        if key in seen:
            return
        seen.add(key)
        presets.append(
            Preset(
                index=len(presets),
                name=sanitize_name(name),
                family=family,
                description=description,
                overrides=dict(overrides),
            )
        )

    baseline = {
        "calibration": groups["calibration"][0],
        "init": groups["init"][0],
        "frontend": groups["frontend"][0],
        "visual_weight": groups["visual_weight"][0],
        "zupt": groups["zupt"][0],
        "state": groups["state"][0],
        "contrast": groups["contrast"][0],
    }

    add_candidate(
        "baseline_current_diag_timeoffset",
        "baseline",
        "exact current diag_timeoffset baseline with evaluation outputs enabled",
        {},
    )

    single_order = ["calibration", "init", "frontend", "visual_weight", "zupt", "state", "contrast"]
    for family in single_order:
        for option in groups[family][1:]:
            add_candidate(
                f"single_{family}_{option.name}",
                f"single/{family}",
                option.description,
                option.overrides,
            )

    def pairwise(
        family_a: str,
        options_a: Iterable[SemanticOption],
        family_b: str,
        options_b: Iterable[SemanticOption],
        family_label: str,
    ) -> None:
        for option_a in options_a:
            for option_b in options_b:
                merged = dict(option_a.overrides)
                merged.update(option_b.overrides)
                add_candidate(
                    f"{family_a}_{option_a.name}__{family_b}_{option_b.name}",
                    family_label,
                    f"{option_a.description}; {option_b.description}",
                    merged,
                )

    pairwise("calibration", groups["calibration"][1:], "frontend", groups["frontend"][1:], "pair/calibration_frontend")
    pairwise("init", groups["init"][1:], "frontend", groups["frontend"][1:], "pair/init_frontend")
    pairwise("frontend", groups["frontend"][1:], "visual", groups["visual_weight"][1:], "pair/frontend_visual")
    pairwise("frontend", groups["frontend"][1:], "zupt", groups["zupt"][1:], "pair/frontend_zupt")
    pairwise("calibration", groups["calibration"][1:], "visual", groups["visual_weight"][1:], "pair/calibration_visual")
    pairwise("state", groups["state"][1:], "frontend", groups["frontend"][1:], "pair/state_frontend")
    pairwise("contrast", groups["contrast"][1:], "frontend", [groups["frontend"][1]], "pair/contrast_frontend")

    reserve_builders: List[Tuple[str, List[SemanticOption], str, List[SemanticOption], str]] = [
        ("calibration", groups["calibration"][1:], "init", groups["init"][1:], "reserve/calibration_init"),
        ("visual", groups["visual_weight"][1:], "init", groups["init"][1:], "reserve/visual_init"),
        ("state", groups["state"][1:], "visual", groups["visual_weight"][1:], "reserve/state_visual"),
        ("contrast", groups["contrast"][1:], "zupt", groups["zupt"][1:], "reserve/contrast_zupt"),
        ("calibration", groups["calibration"][1:], "state", groups["state"][1:], "reserve/calibration_state"),
    ]
    for family_a, opts_a, family_b, opts_b, label in reserve_builders:
        if len(presets) >= count:
            break
        pairwise(family_a, opts_a, family_b, opts_b, label)

    if len(presets) < count:
        raise RuntimeError(f"Only generated {len(presets)} presets, fewer than requested {count}")

    return [Preset(index=i, name=p.name, family=p.family, description=p.description, overrides=p.overrides) for i, p in enumerate(presets[:count])]


def build_phase2_option_groups(base: Dict[str, Any]) -> Dict[str, List[SemanticOption]]:
    clones = int(base.get("max_clones", 13))
    max_slam = int(base.get("max_slam", 70))
    max_slam_in_update = int(base.get("max_slam_in_update", 35))
    max_msckf_in_update = int(base.get("max_msckf_in_update", 50))
    dt_slam_delay = float(base.get("dt_slam_delay", 1.5))

    num_pts = int(base.get("num_pts", 300))
    fast_threshold = int(base.get("fast_threshold", 20))
    min_px_dist = int(base.get("min_px_dist", 12))

    sigma_px = float(base.get("up_msckf_sigma_px", 1.5))
    use_mask = bool(base.get("use_mask", True))
    histogram_method = str(base.get("histogram_method", "HISTOGRAM"))

    init_imu_thresh = float(base.get("init_imu_thresh", 0.3))
    init_max_disparity = float(base.get("init_max_disparity", 2.0))
    init_dyn_use = bool(base.get("init_dyn_use", False))

    try_zupt = bool(base.get("try_zupt", True))
    zupt_max_velocity = float(base.get("zupt_max_velocity", 0.05))
    zupt_noise_multiplier = float(base.get("zupt_noise_multiplier", 80.0))
    zupt_max_disparity = float(base.get("zupt_max_disparity", 0.75))

    groups = {
        "state": [
            SemanticOption(
                "phase2_seed_state",
                "current phase-2 seed state window",
                {
                    "max_clones": clones,
                    "max_slam": max_slam,
                    "max_slam_in_update": max_slam_in_update,
                    "max_msckf_in_update": max_msckf_in_update,
                    "dt_slam_delay": dt_slam_delay,
                },
            ),
            SemanticOption(
                "expanded_trimmed",
                "slightly trim state size to reduce update noise while keeping extra context",
                {
                    "max_clones": clamp_int(clones - 1, 11, 20),
                    "max_slam": clamp_int(max_slam - 10, 50, 120),
                    "max_slam_in_update": clamp_int(max_slam_in_update - 5, 25, 60),
                    "max_msckf_in_update": clamp_int(max_msckf_in_update - 5, 40, 80),
                    "dt_slam_delay": clamp_float(dt_slam_delay - 0.25, 1.0, 3.0),
                },
            ),
            SemanticOption(
                "hybrid_wide",
                "push temporal context wider without jumping all the way to the widest phase-1 state",
                {
                    "max_clones": clamp_int(clones + 1, 11, 20),
                    "max_slam": clamp_int(max_slam + 10, 50, 120),
                    "max_slam_in_update": clamp_int(max_slam_in_update + 5, 25, 60),
                    "max_msckf_in_update": clamp_int(max_msckf_in_update + 5, 40, 80),
                    "dt_slam_delay": clamp_float(dt_slam_delay + 0.25, 1.0, 3.0),
                },
            ),
            SemanticOption(
                "phase1_wide_context",
                "reuse the strongest phase-1 wide-context state family",
                {
                    "max_clones": 15,
                    "max_slam": 90,
                    "max_slam_in_update": 45,
                    "max_msckf_in_update": 60,
                    "dt_slam_delay": 2.0,
                },
            ),
        ],
        "frontend": [
            SemanticOption(
                "phase2_seed_frontend",
                "current dense-recovery phase-2 seed front-end",
                {
                    "num_pts": num_pts,
                    "fast_threshold": fast_threshold,
                    "min_px_dist": min_px_dist,
                },
            ),
            SemanticOption(
                "dense_tightened",
                "slightly tighten dense recovery to reduce weak corners",
                {
                    "num_pts": clamp_int(num_pts - 20, 200, 400),
                    "fast_threshold": clamp_int(fast_threshold + 2, 10, 60),
                    "min_px_dist": clamp_int(min_px_dist + 1, 10, 18),
                },
            ),
            SemanticOption(
                "reacquire_bias",
                "shift toward the best phase-1 reacquire style while staying near the seed",
                {
                    "num_pts": clamp_int(num_pts - 50, 200, 400),
                    "fast_threshold": clamp_int(fast_threshold + 5, 10, 60),
                    "min_px_dist": clamp_int(min_px_dist + 3, 10, 18),
                },
            ),
            SemanticOption(
                "dense_more_aggressive",
                "push the dense recovery further for vibration-heavy feature loss",
                {
                    "num_pts": clamp_int(num_pts + 20, 200, 400),
                    "fast_threshold": clamp_int(fast_threshold - 2, 10, 60),
                    "min_px_dist": clamp_int(min_px_dist - 1, 10, 18),
                },
            ),
            SemanticOption(
                "dense_max_aggressive",
                "maximum local recovery push before compute cost becomes the main risk",
                {
                    "num_pts": clamp_int(num_pts + 40, 200, 400),
                    "fast_threshold": clamp_int(fast_threshold - 4, 10, 60),
                    "min_px_dist": clamp_int(min_px_dist - 2, 10, 18),
                },
            ),
        ],
        "visual": [
            SemanticOption(
                "seed_visual_weight",
                "current visual residual weighting",
                {"up_msckf_sigma_px": sigma_px, "up_slam_sigma_px": sigma_px},
            ),
            SemanticOption(
                "slightly_tighter_visual",
                "trust pixels slightly more if dense recovery produces clean enough tracks",
                {
                    "up_msckf_sigma_px": clamp_float(sigma_px - 0.25, 1.0, 3.0),
                    "up_slam_sigma_px": clamp_float(sigma_px - 0.25, 1.0, 3.0),
                },
            ),
            SemanticOption(
                "slightly_looser_visual",
                "down-weight pixels slightly more under blur and vibration",
                {
                    "up_msckf_sigma_px": clamp_float(sigma_px + 0.25, 1.0, 3.0),
                    "up_slam_sigma_px": clamp_float(sigma_px + 0.25, 1.0, 3.0),
                },
            ),
            SemanticOption(
                "looser_visual",
                "further reduce visual trust to favor inertial continuity",
                {
                    "up_msckf_sigma_px": clamp_float(sigma_px + 0.5, 1.0, 3.0),
                    "up_slam_sigma_px": clamp_float(sigma_px + 0.5, 1.0, 3.0),
                },
            ),
        ],
        "init": [
            SemanticOption(
                "seed_static_gate",
                "keep the current strict static initialization gate",
                {
                    "init_imu_thresh": init_imu_thresh,
                    "init_max_disparity": init_max_disparity,
                    "init_dyn_use": init_dyn_use,
                },
            ),
            SemanticOption(
                "balanced_static_gate",
                "relax static gating slightly in case the current gate is rejecting mild startup motion",
                {"init_imu_thresh": 0.6, "init_max_disparity": 4.0, "init_dyn_use": False},
            ),
            SemanticOption(
                "legacy_static_gate",
                "probe the older relaxed gate while keeping static initialization",
                {"init_imu_thresh": 1.0, "init_max_disparity": 6.0, "init_dyn_use": False},
            ),
            SemanticOption(
                "dynamic_probe",
                "probe dynamic initialization near the current seed family",
                {"init_imu_thresh": 0.6, "init_max_disparity": 4.0, "init_dyn_use": True},
            ),
        ],
        "zupt": [
            SemanticOption(
                "seed_zupt",
                "current begin-only ZUPT settings",
                {
                    "try_zupt": try_zupt,
                    "zupt_max_velocity": zupt_max_velocity,
                    "zupt_noise_multiplier": zupt_noise_multiplier,
                    "zupt_max_disparity": zupt_max_disparity,
                    "zupt_only_at_beginning": bool(base.get("zupt_only_at_beginning", True)),
                },
            ),
            SemanticOption(
                "very_strict_zupt",
                "tighten early ZUPT if startup false positives remain",
                {
                    "try_zupt": True,
                    "zupt_max_velocity": 0.03,
                    "zupt_noise_multiplier": 60.0,
                    "zupt_max_disparity": 0.5,
                    "zupt_only_at_beginning": True,
                },
            ),
            SemanticOption(
                "tolerant_zupt",
                "accept more rotor jitter while still keeping begin-only ZUPT",
                {
                    "try_zupt": True,
                    "zupt_max_velocity": 0.08,
                    "zupt_noise_multiplier": 100.0,
                    "zupt_max_disparity": 1.0,
                    "zupt_only_at_beginning": True,
                },
            ),
        ],
        "contrast": [
            SemanticOption(
                "seed_contrast",
                "current histogram equalization and mask usage",
                {"histogram_method": histogram_method, "use_mask": use_mask},
            ),
            SemanticOption(
                "histogram_unmasked",
                "remove the mask while keeping histogram equalization",
                {"histogram_method": "HISTOGRAM", "use_mask": False},
            ),
        ],
    }
    return groups


def generate_phase2_presets(base: Dict[str, Any], count: int) -> List[Preset]:
    groups = build_phase2_option_groups(base)
    presets: List[Preset] = []
    seen: set[str] = set()

    def add_candidate(name: str, family: str, description: str, overrides: Dict[str, Any]) -> None:
        key = canonical_overrides(overrides)
        if key in seen:
            return
        seen.add(key)
        presets.append(
            Preset(
                index=len(presets),
                name=sanitize_name(name),
                family=family,
                description=description,
                overrides=dict(overrides),
            )
        )

    add_candidate(
        "phase2_seed_baseline",
        "baseline",
        "exact phase-2 seed baseline with evaluation outputs enabled",
        {},
    )

    single_order = ["state", "frontend", "visual", "init", "zupt", "contrast"]
    for family in single_order:
        for option in groups[family][1:]:
            add_candidate(
                f"single_{family}_{option.name}",
                f"single/{family}",
                option.description,
                option.overrides,
            )

    def pairwise(
        family_a: str,
        options_a: Iterable[SemanticOption],
        family_b: str,
        options_b: Iterable[SemanticOption],
        family_label: str,
    ) -> None:
        for option_a in options_a:
            for option_b in options_b:
                merged = dict(option_a.overrides)
                merged.update(option_b.overrides)
                add_candidate(
                    f"{family_a}_{option_a.name}__{family_b}_{option_b.name}",
                    family_label,
                    f"{option_a.description}; {option_b.description}",
                    merged,
                )

    pairwise("state", groups["state"][1:], "frontend", groups["frontend"][1:], "pair/state_frontend")
    pairwise("frontend", groups["frontend"][1:], "visual", groups["visual"][1:], "pair/frontend_visual")
    pairwise("frontend", groups["frontend"][1:], "init", groups["init"][1:], "pair/frontend_init")
    pairwise("state", groups["state"][1:], "zupt", groups["zupt"][1:], "pair/state_zupt")
    pairwise("state", groups["state"][1:], "contrast", groups["contrast"][1:], "pair/state_contrast")

    if len(presets) < count:
        for state_option in groups["state"][1:]:
            for frontend_option in groups["frontend"][1:]:
                for visual_option in groups["visual"][1:]:
                    merged = dict(state_option.overrides)
                    merged.update(frontend_option.overrides)
                    merged.update(visual_option.overrides)
                    add_candidate(
                        f"triple_{state_option.name}__{frontend_option.name}__{visual_option.name}",
                        "triple/state_frontend_visual",
                        f"{state_option.description}; {frontend_option.description}; {visual_option.description}",
                        merged,
                    )
                    if len(presets) >= count:
                        break
                if len(presets) >= count:
                    break
            if len(presets) >= count:
                break

    if len(presets) < count:
        raise RuntimeError(f"Only generated {len(presets)} phase-2 presets, fewer than requested {count}")

    return [Preset(index=i, name=p.name, family=p.family, description=p.description, overrides=p.overrides) for i, p in enumerate(presets[:count])]


def generate_presets(base: Dict[str, Any], count: int, phase: str) -> List[Preset]:
    if phase == "phase1":
        return generate_phase1_presets(base, count)
    if phase == "phase2":
        return generate_phase2_presets(base, count)
    raise ValueError(f"Unsupported phase: {phase}")


def make_output_paths(output_root: Path, preset: Preset) -> Dict[str, Path]:
    slug = f"{preset.index:03d}_{preset.name}"
    case_root = output_root / slug
    return {
        "root": case_root,
        "config": case_root / "config.yaml",
        "timing": case_root / "timing.csv",
        "state_est": case_root / "state_estimate.txt",
        "state_std": case_root / "state_deviation.txt",
        "state_gt": case_root / "state_groundtruth.txt",
        "launch_log": case_root / "run_subscribe_msckf.log",
        "bag_log": case_root / "bag_play.log",
        "metrics": case_root / "metrics.json",
    }


def resolve_support_file(path_value: Any, baseline_dir: Path) -> Optional[Path]:
    if not isinstance(path_value, str) or not path_value:
        return None
    path = Path(path_value)
    if not path.is_absolute():
        path = (baseline_dir / path).resolve()
    return path


def stage_support_file(source: Optional[Path], case_root: Path) -> Optional[str]:
    if source is None:
        return None
    if not source.exists():
        raise FileNotFoundError(f"Support file not found: {source}")
    target = case_root / source.name
    if target.exists() or target.is_symlink():
        target.unlink()
    try:
        target.symlink_to(source)
    except OSError:
        shutil.copy2(source, target)
    return source.name


def stage_support_files(config: Dict[str, Any], baseline_dir: Path, case_root: Path) -> None:
    for key in ("relative_config_imu", "relative_config_imucam", "mask0", "mask1"):
        staged_name = stage_support_file(resolve_support_file(config.get(key), baseline_dir), case_root)
        if staged_name is not None:
            config[key] = staged_name


def build_config_text(
    template_text: str,
    template_values: Dict[str, Any],
    baseline_dir: Path,
    preset: Preset,
    paths: Dict[str, Path],
) -> Tuple[str, Dict[str, Any]]:
    effective = dict(template_values)
    effective.update(preset.overrides)
    effective["record_timing_information"] = True
    effective["record_timing_filepath"] = str(paths["timing"])
    effective["filepath_est"] = str(paths["state_est"])
    effective["filepath_std"] = str(paths["state_std"])
    effective["filepath_gt"] = str(paths["state_gt"])
    stage_support_files(effective, baseline_dir, paths["root"])

    text = template_text
    for key in (
        list(preset.overrides.keys())
        + [
            "record_timing_information",
            "record_timing_filepath",
            "filepath_est",
            "filepath_std",
            "filepath_gt",
            "mask0",
            "mask1",
            "relative_config_imu",
            "relative_config_imucam",
        ]
    ):
        text = replace_value(text, key, effective[key])
    return text, effective


def load_timing_summary(path: Path) -> Dict[str, Optional[float]]:
    empty = {
        "timing_rows": 0,
        "timing_track_mean_s": None,
        "timing_track_p95_s": None,
        "timing_prop_mean_s": None,
        "timing_prop_p95_s": None,
        "timing_msckf_mean_s": None,
        "timing_msckf_p95_s": None,
        "timing_slam_update_mean_s": None,
        "timing_slam_update_p95_s": None,
        "timing_slam_delay_mean_s": None,
        "timing_slam_delay_p95_s": None,
        "timing_marg_mean_s": None,
        "timing_marg_p95_s": None,
        "timing_total_mean_s": None,
        "timing_total_p95_s": None,
    }
    if not path.exists():
        return empty
    rows: List[List[float]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            try:
                rows.append([float(item) for item in stripped.split(",")])
            except ValueError:
                continue
    if not rows:
        return empty

    tracks = [row[1] for row in rows if len(row) > 1]
    props = [row[2] for row in rows if len(row) > 2]
    msckfs = [row[3] for row in rows if len(row) > 3]
    slam_updates = [row[4] for row in rows if len(row) >= 8]
    slam_delays = [row[5] for row in rows if len(row) >= 8]
    if rows and len(rows[0]) >= 8:
        margs = [row[6] for row in rows if len(row) > 6]
        totals = [row[7] for row in rows if len(row) > 7]
    else:
        margs = [row[4] for row in rows if len(row) > 4]
        totals = [row[5] for row in rows if len(row) > 5]

    return {
        "timing_rows": len(rows),
        "timing_track_mean_s": mean(tracks),
        "timing_track_p95_s": percentile(tracks, 95.0),
        "timing_prop_mean_s": mean(props),
        "timing_prop_p95_s": percentile(props, 95.0),
        "timing_msckf_mean_s": mean(msckfs),
        "timing_msckf_p95_s": percentile(msckfs, 95.0),
        "timing_slam_update_mean_s": mean(slam_updates),
        "timing_slam_update_p95_s": percentile(slam_updates, 95.0),
        "timing_slam_delay_mean_s": mean(slam_delays),
        "timing_slam_delay_p95_s": percentile(slam_delays, 95.0),
        "timing_marg_mean_s": mean(margs),
        "timing_marg_p95_s": percentile(margs, 95.0),
        "timing_total_mean_s": mean(totals),
        "timing_total_p95_s": percentile(totals, 95.0),
    }


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def unwrap_angles(values: Sequence[float]) -> List[float]:
    if not values:
        return []
    unwrapped = [values[0]]
    for value in values[1:]:
        prev = unwrapped[-1]
        delta = value - prev
        while delta > math.pi:
            value -= 2.0 * math.pi
            delta = value - prev
        while delta < -math.pi:
            value += 2.0 * math.pi
            delta = value - prev
        unwrapped.append(value)
    return unwrapped


def process_group_alive(proc: Optional[subprocess.Popen[Any]]) -> bool:
    return proc is not None and proc.poll() is None


def stop_process_group(proc: Optional[subprocess.Popen[Any]], sig: signal.Signals, wait_s: float) -> Optional[int]:
    if proc is None or proc.poll() is not None:
        return proc.returncode if proc is not None else None
    try:
        os.killpg(proc.pid, sig)
    except ProcessLookupError:
        return proc.poll()
    deadline = time.monotonic() + wait_s
    while time.monotonic() < deadline:
        code = proc.poll()
        if code is not None:
            return code
        time.sleep(0.1)
    return proc.poll()


def inspect_bag_topics(path: Path) -> Tuple[Optional[set[str]], Optional[str]]:
    if shutil.which("ros2") is None:
        return None, "ros2 not in PATH"
    try:
        output = subprocess.run(
            ["ros2", "bag", "info", "--yaml", str(path)],
            check=True,
            capture_output=True,
            text=True,
        )
    except Exception:
        try:
            output = subprocess.run(
                ["ros2", "bag", "info", str(path)],
                check=True,
                capture_output=True,
                text=True,
            )
        except Exception as exc:
            return None, str(exc)
        topics: set[str] = set()
        topic_re = re.compile(r"Topic:\s+(?P<name>\S+)\s+\|")
        for line in output.stdout.splitlines():
            match = topic_re.search(line)
            if match:
                topics.add(match.group("name"))
        return (topics or None), None
    try:
        import yaml

        info = yaml.safe_load(output.stdout)
    except Exception as exc:
        return None, f"failed to parse ros2 bag info --yaml: {exc}"
    topics: set[str] = set()
    for entry in info.get("topics_with_message_count", []) or []:
        meta = entry.get("topic_metadata", {})
        name = meta.get("name")
        if name:
            topics.add(name)
    return topics, None


def required_bag_topics(args: argparse.Namespace) -> set[str]:
    return {
        args.left_topic,
        args.right_topic,
        args.imu_topic,
    }


def create_monitor(namespace: str):
    import rclpy
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2

    topic_qos = QoSProfile(
        depth=50,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

    class SweepMonitor(Node):
        def __init__(self) -> None:
            super().__init__("openvins_sweep_monitor")
            odom_topic = f"{namespace.rstrip('/')}/odomimu"
            msckf_topic = f"{namespace.rstrip('/')}/points_msckf"
            slam_topic = f"{namespace.rstrip('/')}/points_slam"
            self.clock_start: Optional[float] = None
            self.clock_end: Optional[float] = None
            self.odom_times: List[float] = []
            self.positions: List[Tuple[float, float, float]] = []
            self.yaws: List[float] = []
            self.msckf_counts: List[int] = []
            self.slam_counts: List[int] = []
            self.last_odom_wall: Optional[float] = None
            self.create_subscription(Clock, "/clock", self.on_clock, topic_qos)
            self.create_subscription(Odometry, odom_topic, self.on_odom, topic_qos)
            self.create_subscription(PointCloud2, msckf_topic, self.on_msckf, topic_qos)
            self.create_subscription(PointCloud2, slam_topic, self.on_slam, topic_qos)

        @staticmethod
        def stamp_to_sec(stamp: Any) -> float:
            return float(stamp.sec) + float(stamp.nanosec) * 1e-9

        def on_clock(self, msg: Clock) -> None:
            stamp_s = self.stamp_to_sec(msg.clock)
            if self.clock_start is None:
                self.clock_start = stamp_s
            self.clock_end = stamp_s

        def on_odom(self, msg: Odometry) -> None:
            stamp_s = self.stamp_to_sec(msg.header.stamp)
            self.odom_times.append(stamp_s)
            pos = msg.pose.pose.position
            self.positions.append((float(pos.x), float(pos.y), float(pos.z)))
            ori = msg.pose.pose.orientation
            self.yaws.append(quat_to_yaw(float(ori.x), float(ori.y), float(ori.z), float(ori.w)))
            self.last_odom_wall = time.monotonic()

        def on_msckf(self, msg: PointCloud2) -> None:
            self.msckf_counts.append(int(msg.width) * int(msg.height))

        def on_slam(self, msg: PointCloud2) -> None:
            self.slam_counts.append(int(msg.width) * int(msg.height))

        def metrics(self) -> Dict[str, Any]:
            data: Dict[str, Any] = {
                "clock_start_s": self.clock_start,
                "clock_end_s": self.clock_end,
                "clock_span_s": None if self.clock_start is None or self.clock_end is None else self.clock_end - self.clock_start,
                "odom_count": len(self.odom_times),
                "points_msckf_count_msgs": len(self.msckf_counts),
                "points_slam_count_msgs": len(self.slam_counts),
                "points_msckf_mean": mean(self.msckf_counts),
                "points_msckf_p10": percentile(self.msckf_counts, 10.0),
                "points_msckf_p90": percentile(self.msckf_counts, 90.0),
                "points_slam_mean": mean(self.slam_counts),
                "points_slam_p10": percentile(self.slam_counts, 10.0),
                "points_slam_p90": percentile(self.slam_counts, 90.0),
                "last_odom_wall_monotonic": self.last_odom_wall,
            }
            if len(self.odom_times) < 2:
                data.update(
                    {
                        "has_odom": len(self.odom_times) > 0,
                        "init_delay_s": None if not self.odom_times or self.clock_start is None else self.odom_times[0] - self.clock_start,
                        "coverage_ratio": 0.0,
                        "odom_rate_hz": 0.0,
                        "max_gap_s": None,
                        "gap_count_over_0p2s": 0,
                        "max_step_m": None,
                        "step_p95_m": None,
                        "jump_count_over_0p4m": 0,
                        "speed_mean_mps": None,
                        "speed_p95_mps": None,
                        "speed_max_mps": None,
                        "speed_count_over_12mps": 0,
                        "accel_p95_mps2": None,
                        "accel_max_mps2": None,
                        "accel_count_over_35mps2": 0,
                        "yaw_rate_p95_radps": None,
                        "yaw_rate_max_radps": None,
                        "yaw_rate_count_over_6radps": 0,
                        "path_length_m": 0.0,
                        "start_end_distance_m": None,
                    }
                )
                return data

            dts: List[float] = []
            steps: List[float] = []
            speeds: List[float] = []
            yaws = unwrap_angles(self.yaws)
            yaw_rates: List[float] = []
            for idx in range(1, len(self.odom_times)):
                dt = self.odom_times[idx] - self.odom_times[idx - 1]
                if dt <= 1e-6:
                    continue
                dts.append(dt)
                prev = self.positions[idx - 1]
                cur = self.positions[idx]
                step = math.dist(prev, cur)
                steps.append(step)
                speeds.append(step / dt)
                yaw_rates.append(abs((yaws[idx] - yaws[idx - 1]) / dt))
            accels: List[float] = []
            for idx in range(1, len(speeds)):
                dt = self.odom_times[idx + 1] - self.odom_times[idx]
                if dt <= 1e-6:
                    continue
                accels.append(abs((speeds[idx] - speeds[idx - 1]) / dt))

            path_length = sum(steps)
            start_end_distance = math.dist(self.positions[0], self.positions[-1]) if self.positions else None
            clock_span = data["clock_span_s"] or 0.0
            traj_span = self.odom_times[-1] - self.odom_times[0]
            init_delay = None if self.clock_start is None else self.odom_times[0] - self.clock_start
            data.update(
                {
                    "has_odom": True,
                    "init_delay_s": init_delay,
                    "coverage_ratio": 0.0 if clock_span <= 1e-6 else max(0.0, min(1.0, traj_span / clock_span)),
                    "odom_rate_hz": 0.0 if traj_span <= 1e-6 else len(self.odom_times) / traj_span,
                    "max_gap_s": max(dts) if dts else None,
                    "gap_count_over_0p2s": sum(1 for dt in dts if dt > 0.2),
                    "max_step_m": max(steps) if steps else None,
                    "step_p95_m": percentile(steps, 95.0),
                    "jump_count_over_0p4m": sum(1 for step in steps if step > 0.4),
                    "speed_mean_mps": mean(speeds),
                    "speed_p95_mps": percentile(speeds, 95.0),
                    "speed_max_mps": max(speeds) if speeds else None,
                    "speed_count_over_12mps": sum(1 for speed in speeds if speed > 12.0),
                    "accel_p95_mps2": percentile(accels, 95.0),
                    "accel_max_mps2": max(accels) if accels else None,
                    "accel_count_over_35mps2": sum(1 for accel in accels if accel > 35.0),
                    "yaw_rate_p95_radps": percentile(yaw_rates, 95.0),
                    "yaw_rate_max_radps": max(yaw_rates) if yaw_rates else None,
                    "yaw_rate_count_over_6radps": sum(1 for rate in yaw_rates if rate > 6.0),
                    "path_length_m": path_length,
                    "start_end_distance_m": start_end_distance,
                }
            )
            return data

    return rclpy, SweepMonitor


def compute_score(metrics: Dict[str, Any], frame_budget_s: Optional[float], run_status: Dict[str, Any]) -> float:
    if not metrics.get("has_odom"):
        return 1_000_000.0

    score = 0.0
    init_delay = metrics.get("init_delay_s")
    if init_delay is not None:
        score += 800.0 * max(init_delay - 3.0, 0.0)

    coverage_ratio = metrics.get("coverage_ratio") or 0.0
    score += 2000.0 * max(0.0, 0.95 - coverage_ratio)
    score += 150.0 * float(metrics.get("gap_count_over_0p2s") or 0)
    max_gap = metrics.get("max_gap_s")
    if max_gap is not None:
        score += 300.0 * max(max_gap - 0.2, 0.0)

    score += 120.0 * float(metrics.get("jump_count_over_0p4m") or 0)
    score += 35.0 * float(metrics.get("speed_count_over_12mps") or 0)
    score += 25.0 * float(metrics.get("accel_count_over_35mps2") or 0)
    score += 25.0 * float(metrics.get("yaw_rate_count_over_6radps") or 0)

    msckf_mean = metrics.get("points_msckf_mean")
    msckf_p10 = metrics.get("points_msckf_p10")
    slam_mean = metrics.get("points_slam_mean")
    score += 2.0 * max(0.0, 40.0 - (msckf_mean or 0.0))
    score += 4.0 * max(0.0, 20.0 - (msckf_p10 or 0.0))
    score += 1.0 * max(0.0, 10.0 - (slam_mean or 0.0))

    if frame_budget_s is not None:
        total_mean = metrics.get("timing_total_mean_s")
        total_p95 = metrics.get("timing_total_p95_s")
        if total_mean is not None:
            score += 300.0 * max(total_mean - 0.75 * frame_budget_s, 0.0) / frame_budget_s
        if total_p95 is not None:
            score += 800.0 * max(total_p95 - frame_budget_s, 0.0) / frame_budget_s

    if run_status.get("launch_failed_before_bag_end"):
        score += 50_000.0
    if run_status.get("timed_out"):
        score += 50_000.0
    if run_status.get("bag_exit_code") not in (None, 0):
        score += 10_000.0
    return score


def run_case(
    args: argparse.Namespace,
    preset: Preset,
    paths: Dict[str, Path],
    effective_config: Dict[str, Any],
) -> Dict[str, Any]:
    ensure_ros2_available()
    for key in ("root",):
        paths[key].mkdir(parents=True, exist_ok=True)

    rclpy, SweepMonitor = create_monitor(args.namespace)
    rclpy.init(args=None)
    monitor = SweepMonitor()

    launch_log_handle = paths["launch_log"].open("w", encoding="utf-8")
    bag_log_handle = paths["bag_log"].open("w", encoding="utf-8")
    launch_proc: Optional[subprocess.Popen[Any]] = None
    bag_proc: Optional[subprocess.Popen[Any]] = None
    bag_done_wall: Optional[float] = None
    run_status = {
        "launch_exit_code": None,
        "bag_exit_code": None,
        "shutdown_exit_code": None,
        "launch_failed_before_bag_end": False,
        "timed_out": False,
    }

    try:
        launch_cmd = [
            "ros2",
            "run",
            "ov_msckf",
            "run_subscribe_msckf",
            "--ros-args",
            "-r",
            f"__ns:={args.namespace}",
            "-p",
            f"use_sim_time:=true",
            "-p",
            f"config_path:={paths['config']}",
            "-p",
            f"save_total_state:=true",
            "-p",
            f"filepath_est:={paths['state_est']}",
            "-p",
            f"filepath_std:={paths['state_std']}",
            "-p",
            f"filepath_gt:={paths['state_gt']}",
            "-p",
            f"cam0_rostopic:={args.left_topic}",
            "-p",
            f"cam1_rostopic:={args.right_topic}",
            "-p",
            f"imu0_rostopic:={args.imu_topic}",
        ]
        launch_proc = subprocess.Popen(
            launch_cmd,
            stdout=launch_log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )

        settle_deadline = time.monotonic() + args.launch_settle_sec
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(monitor, timeout_sec=0.1)
            if launch_proc.poll() is not None:
                run_status["launch_failed_before_bag_end"] = True
                run_status["launch_exit_code"] = launch_proc.returncode
                break
        if run_status["launch_failed_before_bag_end"]:
            metrics = monitor.metrics()
            metrics.update(load_timing_summary(paths["timing"]))
            frame_budget = None
            if float(effective_config.get("track_frequency", 0.0)) > 0.0:
                frame_budget = 1.0 / float(effective_config["track_frequency"])
            score = compute_score(metrics, frame_budget, run_status)
            result = {
                "preset_index": preset.index,
                "preset_name": preset.name,
                "status": "launch_failed",
                "score": score,
                "launch_log_tail": read_log_tail(paths["launch_log"]),
                **run_status,
                **metrics,
            }
            return result

        bag_cmd = [
            "ros2",
            "bag",
            "play",
            str(args.bag),
            "--clock",
            "--rate",
            str(args.play_rate),
        ]
        bag_proc = subprocess.Popen(
            bag_cmd,
            stdout=bag_log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )
        start_wall = time.monotonic()
        timeout_deadline = start_wall + args.timeout_sec

        while True:
            rclpy.spin_once(monitor, timeout_sec=0.1)
            if launch_proc.poll() is not None and bag_proc.poll() is None:
                run_status["launch_failed_before_bag_end"] = True
                run_status["launch_exit_code"] = launch_proc.returncode
                stop_process_group(bag_proc, signal.SIGINT, 5.0)
                break
            if bag_proc.poll() is not None:
                if bag_done_wall is None:
                    bag_done_wall = time.monotonic()
                    run_status["bag_exit_code"] = bag_proc.returncode
                if time.monotonic() - bag_done_wall >= args.post_bag_grace_sec:
                    break
            if time.monotonic() >= timeout_deadline:
                run_status["timed_out"] = True
                break

        run_status["launch_exit_code"] = launch_proc.poll()
        run_status["bag_exit_code"] = bag_proc.poll() if bag_proc is not None else None

    finally:
        if bag_proc is not None and bag_proc.poll() is None:
            stop_process_group(bag_proc, signal.SIGINT, 5.0)
            if bag_proc.poll() is None:
                stop_process_group(bag_proc, signal.SIGTERM, 5.0)
            run_status["bag_exit_code"] = bag_proc.poll()

        if launch_proc is not None:
            stop_process_group(launch_proc, signal.SIGINT, 5.0)
            if launch_proc.poll() is None:
                stop_process_group(launch_proc, signal.SIGTERM, 5.0)
            run_status["shutdown_exit_code"] = launch_proc.poll()
            run_status["launch_exit_code"] = launch_proc.returncode

        try:
            metrics = monitor.metrics()
        finally:
            monitor.destroy_node()
            rclpy.shutdown()
            launch_log_handle.close()
            bag_log_handle.close()

    metrics.update(load_timing_summary(paths["timing"]))
    frame_budget = None
    if float(effective_config.get("track_frequency", 0.0)) > 0.0:
        frame_budget = 1.0 / float(effective_config["track_frequency"])
    score = compute_score(metrics, frame_budget, run_status)
    status = "ok"
    if run_status["launch_failed_before_bag_end"]:
        status = "launch_failed"
    elif run_status["timed_out"]:
        status = "timed_out"
    elif not metrics.get("has_odom"):
        status = "no_odom"

    return {
        "preset_index": preset.index,
        "preset_name": preset.name,
        "family": preset.family,
        "description": preset.description,
        "status": status,
        "score": score,
        "launch_log_tail": read_log_tail(paths["launch_log"]) if status == "launch_failed" else None,
        **run_status,
        **metrics,
    }


def write_summary(results: List[Dict[str, Any]], summary_csv: Path, summary_json: Path) -> None:
    ordered = sorted(results, key=lambda item: float(item["score"]))
    with summary_json.open("w", encoding="utf-8") as handle:
        json.dump(ordered, handle, indent=2, sort_keys=False)
    fieldnames: List[str] = []
    for result in ordered:
        for key in result.keys():
            if key not in fieldnames:
                fieldnames.append(key)
    with summary_csv.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for result in ordered:
            writer.writerow(result)


def print_top_results(results: List[Dict[str, Any]], top_n: int) -> None:
    ordered = sorted(results, key=lambda item: float(item["score"]))[:top_n]
    debug("")
    debug("Top results:")
    for result in ordered:
        debug(
            "  "
            + f"{int(result['preset_index']):03d} "
            + f"{result['preset_name']} "
            + f"score={result['score']:.2f} "
            + f"status={result['status']} "
            + f"coverage={result.get('coverage_ratio', 0.0):.3f} "
            + f"init_delay={result.get('init_delay_s')} "
            + f"msckf_mean={result.get('points_msckf_mean')} "
            + f"timing_total_p95={result.get('timing_total_p95_s')} "
            + f"track_p95={result.get('timing_track_p95_s')} "
            + f"msckf_p95={result.get('timing_msckf_p95_s')} "
            + f"marg_p95={result.get('timing_marg_p95_s')}"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Semantic, no-GT OpenVINS sweep for Orbbec raw bags. Intended to run headless, including inside Docker.",
    )
    parser.add_argument(
        "--phase",
        choices=("phase1", "phase2"),
        default="phase1",
        help="Search phase: phase1 does broad semantic exploration; phase2 does local refinement around a stronger seed",
    )
    parser.add_argument("--bag", type=Path, help="Path to a rosbag2 directory containing raw left/right IR and IMU topics")
    parser.add_argument("--base-config", type=Path, help="Baseline estimator_config YAML to mutate")
    parser.add_argument("--output-root", type=Path, help="Where generated configs, logs, and summary files are written")
    parser.add_argument("--count", type=int, default=100, help="How many semantic presets to generate and evaluate")
    parser.add_argument("--dry-run", action="store_true", help="Generate presets and files only; do not run ROS processes")
    parser.add_argument("--limit", type=int, help="Optional cap on how many generated presets to execute")
    parser.add_argument("--launch-settle-sec", type=float, default=3.0, help="Wall-clock settle time before bag playback starts")
    parser.add_argument("--post-bag-grace-sec", type=float, default=2.0, help="Extra wall-clock time to keep OV alive after bag play exits")
    parser.add_argument("--timeout-sec", type=float, default=900.0, help="Wall-clock timeout for each preset")
    parser.add_argument("--play-rate", type=float, default=1.0, help="ros2 bag play rate")
    parser.add_argument("--left-topic", default=DEFAULT_LEFT_TOPIC, help="Left IR image topic inside the bag")
    parser.add_argument("--right-topic", default=DEFAULT_RIGHT_TOPIC, help="Right IR image topic inside the bag")
    parser.add_argument("--imu-topic", default=DEFAULT_IMU_TOPIC, help="IMU topic inside the bag")
    parser.add_argument("--namespace", default=DEFAULT_NAMESPACE, help="Namespace for the evaluated OpenVINS node")
    parser.add_argument("--top", type=int, default=10, help="How many top-scoring presets to print at the end")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    share_dir = find_package_share()
    default_base = DEFAULT_PHASE1_BASE_RELATIVE if args.phase == "phase1" else DEFAULT_PHASE2_BASE_RELATIVE
    if args.base_config:
        base_config = args.base_config.resolve()
    else:
        base_config = (share_dir / default_base).resolve()
        if not base_config.exists():
            source_root = find_source_root_from_script()
            if source_root is not None:
                fallback = (source_root / default_base).resolve()
                if fallback.exists():
                    base_config = fallback
    if not base_config.exists():
        raise FileNotFoundError(f"Base config not found: {base_config}")

    if args.count <= 0:
        raise ValueError("--count must be positive")

    if args.output_root:
        output_root = args.output_root.resolve()
    else:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        output_root = (Path.cwd() / f"openvins_orbbec_sweep_{timestamp}").resolve()
    output_root.mkdir(parents=True, exist_ok=True)

    template_text = load_text(base_config)
    template_values = parse_top_level_config(template_text)
    baseline_dir = base_config.parent
    presets = generate_presets(template_values, args.count, args.phase)
    if args.limit is not None:
        presets = presets[: args.limit]

    manifest_path = output_root / "preset_manifest.json"
    manifest: List[Dict[str, Any]] = []

    bag_topics, bag_info_error = (None, None)
    if args.bag is not None and not args.dry_run:
        args.bag = args.bag.resolve()
        if not args.bag.exists():
            raise FileNotFoundError(f"Bag path not found: {args.bag}")
        bag_topics, bag_info_error = inspect_bag_topics(args.bag)
        if bag_topics is not None:
            missing = sorted(required_bag_topics(args).difference(bag_topics))
            if missing:
                raise RuntimeError(
                    "Bag is missing required raw topics for replay: " + ", ".join(missing)
                )
        elif bag_info_error:
            debug(f"Warning: unable to inspect bag topics ahead of time: {bag_info_error}")
    elif not args.dry_run and args.bag is None:
        raise ValueError("--bag is required unless --dry-run is used")

    for preset in presets:
        paths = make_output_paths(output_root, preset)
        paths["root"].mkdir(parents=True, exist_ok=True)
        config_text, effective_config = build_config_text(template_text, template_values, baseline_dir, preset, paths)
        paths["config"].write_text(config_text, encoding="utf-8")
        entry = {
            "index": preset.index,
            "name": preset.name,
            "family": preset.family,
            "description": preset.description,
            "overrides": preset.overrides,
            "config_path": str(paths["config"]),
            "case_root": str(paths["root"]),
        }
        manifest.append(entry)

    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    debug(f"Wrote {len(manifest)} generated {args.phase} configs into {output_root}")

    if args.dry_run:
        debug("Dry run complete. No ROS processes were started.")
        return 0

    results: List[Dict[str, Any]] = []
    for preset in presets:
        paths = make_output_paths(output_root, preset)
        effective_config = parse_top_level_config(paths["config"].read_text(encoding="utf-8"))
        debug(f"[{preset.index + 1}/{len(presets)}] Running {preset.name}")
        result = run_case(args, preset, paths, effective_config)
        result["config_path"] = str(paths["config"])
        result["case_root"] = str(paths["root"])
        results.append(result)
        paths["metrics"].write_text(json.dumps(result, indent=2), encoding="utf-8")
        debug(
            f"  -> status={result['status']} score={result['score']:.2f} "
            f"coverage={result.get('coverage_ratio')} msckf_mean={result.get('points_msckf_mean')} "
            f"timing_total_p95={result.get('timing_total_p95_s')} "
            f"track_p95={result.get('timing_track_p95_s')} "
            f"msckf_p95={result.get('timing_msckf_p95_s')} "
            f"marg_p95={result.get('timing_marg_p95_s')}"
        )
        if result["status"] == "launch_failed" and result.get("launch_log_tail"):
            debug("     launch tail:")
            for line in str(result["launch_log_tail"]).splitlines():
                debug(f"       {line}")

    summary_csv = output_root / "summary.csv"
    summary_json = output_root / "summary.json"
    write_summary(results, summary_csv, summary_json)
    print_top_results(results, args.top)
    debug("")
    debug(f"Summary CSV:  {summary_csv}")
    debug(f"Summary JSON: {summary_json}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
