#!/usr/bin/env python3

import argparse
import shutil
import sys
from pathlib import Path
from typing import Dict, Tuple


REQUIRED_BASE_FILES = (
    "kalibr_imucam_chain.yaml",
    "kalibr_imu_chain.yaml",
    "mask0.pgm",
    "mask1.pgm",
)

REQUIRED_RUNTIME_CONFIG = "estimator_config.flight.yaml"
RUNTIME_CONFIG_GLOB = "estimator_config.flight*.yaml"

REQUIRED_RUNTIME_BOOL_VALUES = {
    "calib_cam_extrinsics": False,
    "calib_cam_timeoffset": False,
}

REQUIRED_RUNTIME_PATH_VALUES = {
    "relative_config_imu": "kalibr_imu_chain.yaml",
    "relative_config_imucam": "kalibr_imucam_chain.yaml",
    "mask0": "mask0.pgm",
    "mask1": "mask1.pgm",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Freeze the converged OpenVINS bootstrap workspace into frozen_final."
    )
    parser.add_argument(
        "--ov-root",
        default="",
        help="Optional OpenVINS config root. Defaults to the project orbbec_stereo_imu directory.",
    )
    parser.add_argument(
        "--bootstrap-dir",
        default="",
        help="Optional bootstrap directory override. Defaults to <ov-root>/bootstrap.",
    )
    parser.add_argument(
        "--frozen-dir",
        default="",
        help="Optional frozen_final directory override. Defaults to <ov-root>/frozen_final.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Copy files even if the runtime config validation fails.",
    )
    return parser.parse_args()


def default_ov_root() -> Path:
    return (
        Path(__file__).resolve().parent.parent
        / "config"
        / "openvins"
        / "orbbec_stereo_imu"
    ).resolve()


def resolve_directories(args: argparse.Namespace) -> Tuple[Path, Path, Path]:
    ov_root = Path(args.ov_root).expanduser().resolve() if args.ov_root else default_ov_root()
    bootstrap_dir = (
        Path(args.bootstrap_dir).expanduser().resolve()
        if args.bootstrap_dir
        else (ov_root / "bootstrap").resolve()
    )
    frozen_dir = (
        Path(args.frozen_dir).expanduser().resolve()
        if args.frozen_dir
        else (ov_root / "frozen_final").resolve()
    )
    return ov_root, bootstrap_dir, frozen_dir


def parse_top_level_scalars(path: Path) -> Dict[str, str]:
    values: Dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        if not raw_line or raw_line[0].isspace():
            continue
        stripped = raw_line.strip()
        if not stripped or stripped.startswith("#") or stripped.startswith("%"):
            continue
        if ":" not in stripped:
            continue
        key, value = stripped.split(":", 1)
        values[key.strip()] = value.strip()
    return values


def normalize_scalar(value: str) -> str:
    normalized = value.strip()
    if len(normalized) >= 2 and normalized[0] == normalized[-1] and normalized[0] in {'"', "'"}:
        normalized = normalized[1:-1]
    return normalized


def list_runtime_configs(config_dir: Path) -> list[str]:
    return sorted(
        path.name for path in config_dir.glob(RUNTIME_CONFIG_GLOB) if path.is_file()
    )


def validate_runtime_config(path: Path) -> Tuple[bool, list[str]]:
    values = parse_top_level_scalars(path)
    problems: list[str] = []

    for key, expected in REQUIRED_RUNTIME_BOOL_VALUES.items():
        actual = normalize_scalar(values.get(key, ""))
        if actual.lower() != str(expected).lower():
            problems.append(
                f"{key} must be {str(expected).lower()} in runtime config, got `{actual or '<missing>'}`"
            )

    for key, expected in REQUIRED_RUNTIME_PATH_VALUES.items():
        actual = normalize_scalar(values.get(key, ""))
        if actual != expected:
            problems.append(
                f"{key} must point to `{expected}` in runtime config, got `{actual or '<missing>'}`"
            )

    return not problems, problems


def ensure_required_bootstrap_files(bootstrap_dir: Path) -> Tuple[bool, list[str]]:
    missing = [name for name in REQUIRED_BASE_FILES if not (bootstrap_dir / name).exists()]
    if not (bootstrap_dir / REQUIRED_RUNTIME_CONFIG).exists():
        missing.append(REQUIRED_RUNTIME_CONFIG)
    return not missing, missing


def validate_runtime_configs(bootstrap_dir: Path) -> Tuple[bool, list[str], list[str]]:
    runtime_configs = list_runtime_configs(bootstrap_dir)
    problems: list[str] = []

    if REQUIRED_RUNTIME_CONFIG not in runtime_configs:
        problems.append(
            f"required runtime config `{REQUIRED_RUNTIME_CONFIG}` is missing from bootstrap"
        )
        return False, runtime_configs, problems

    for name in runtime_configs:
        valid, config_problems = validate_runtime_config(bootstrap_dir / name)
        for problem in config_problems:
            problems.append(f"{name}: {problem}")

    return not problems, runtime_configs, problems


def copy_required_files(bootstrap_dir: Path, frozen_dir: Path, runtime_configs: list[str]) -> None:
    frozen_dir.mkdir(parents=True, exist_ok=True)

    for name in list_runtime_configs(frozen_dir):
        if name not in runtime_configs:
            stale_path = frozen_dir / name
            stale_path.unlink()
            print(f"removed stale {stale_path}")

    for name in (*REQUIRED_BASE_FILES, *runtime_configs):
        src = bootstrap_dir / name
        dst = frozen_dir / name
        shutil.copy2(src, dst)
        print(f"copied {src} -> {dst}")


def main() -> int:
    args = parse_args()
    ov_root, bootstrap_dir, frozen_dir = resolve_directories(args)

    if bootstrap_dir == frozen_dir:
        print("error: bootstrap_dir and frozen_dir must be different", file=sys.stderr)
        return 1

    ok, missing = ensure_required_bootstrap_files(bootstrap_dir)
    if not ok:
        print(
            "error: bootstrap workspace is incomplete; missing: " + ", ".join(missing),
            file=sys.stderr,
        )
        return 1

    valid_runtime_config, runtime_configs, problems = validate_runtime_configs(bootstrap_dir)
    if problems and not args.force:
        for problem in problems:
            print(f"error: {problem}", file=sys.stderr)
        print(
            "error: refusing to freeze because bootstrap runtime config is not safe for flight; "
            "pass --force to override",
            file=sys.stderr,
        )
        return 1

    print(f"ov_root: {ov_root}")
    print(f"bootstrap_dir: {bootstrap_dir}")
    print(f"frozen_dir: {frozen_dir}")

    copy_required_files(bootstrap_dir, frozen_dir, runtime_configs)

    if problems:
        for problem in problems:
            print(f"warning: {problem}", file=sys.stderr)
        print("warning: freeze completed because --force was set", file=sys.stderr)

    print("freeze complete: runtime now reads frozen_final/")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
