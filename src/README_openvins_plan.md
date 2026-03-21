# OpenVINS 标定链路当前结构

## 1. 目标

当前本地项目只保留两套活动配置语义：

- `bootstrap/`：标定工作目录
- `frozen_final/`：运行最终目录

这次清理是破坏性的，不保留旧根目录活动副本的兼容入口。

## 2. 目录职责

```bash
uav_bringup/config/openvins/orbbec_stereo_imu/
├── bootstrap/
│   ├── estimator_config.calibration.yaml
│   ├── estimator_config.flight.yaml
│   ├── kalibr_imucam_chain.yaml
│   ├── kalibr_imu_chain.yaml
│   ├── mask0.pgm
│   └── mask1.pgm
└── frozen_final/
    ├── estimator_config.flight.yaml
    ├── kalibr_imucam_chain.yaml
    ├── kalibr_imu_chain.yaml
    ├── mask0.pgm
    └── mask1.pgm
```

- `bootstrap/` 可以被脚本反复覆盖。
- `frozen_final/` 只在确认收敛后从 `bootstrap/` 一次性复制。

## 3. 活动入口

### 标定源入口

`uav_bringup/launch/openvins_orbbec_bootstrap_source.launch.py`

- 只起 Orbbec
- 固定打开 depth / extrinsics / synced IMU / 双 IR
- 固定关闭 color / point cloud

### bootstrap 生成脚本

`uav_bringup/scripts/generate_orbbec_openvins_bootstrap.py`

- 只接受 gyro reference 语义
- 只写 `bootstrap/`
- 缺 `CameraInfo`、`depth_to_left/right_ir`、`depth_to_gyro` 直接失败
- `gyro_info` / `accel_info` 缺失时给出显式 warning

### 标定运行入口

`uav_bringup/launch/openvins_orbbec_calibration.launch.py`

- 强制 `openvins_save_total_state=true`
- 强制 `openvins_publish_calibration_tf=true`
- 强制 `publish_px4_external_vision=false`
- 强制 depth extrinsics 可用

### 结果回写脚本

`uav_bringup/scripts/apply_openvins_calibration_result.py`

- 默认原地覆盖输入 `kalibr_imucam_chain.yaml`
- 不再更新内参
- 遇到高 final speed / 几何突变时默认失败退出
- 只有 `--force` 可以绕过

### 运行入口

- `uav_bringup/launch/openvins_orbbec.launch.py`
- `uav_bringup/launch/minimal_control.launch.py`

默认都只读 `frozen_final/estimator_config.flight.yaml`。

## 4. 已删除的旧入口

以下路径不再允许作为活动配置来源：

- 根目录旧的运行配置副本
- 根目录旧的相机-IMU 标定副本
- 根目录旧的 IMU 噪声副本
- 根目录旧的遮罩副本

## 5. 工作流摘要

1. 起 `openvins_orbbec_bootstrap_source.launch.py`
2. 运行 `generate_orbbec_openvins_bootstrap.py`
3. 起 `openvins_orbbec_calibration.launch.py`
4. 用 `apply_openvins_calibration_result.py` 原地写回 `bootstrap/kalibr_imucam_chain.yaml`
5. 多轮重复 3-4 直到收敛
6. 一次性复制到 `frozen_final/`
7. 真机运行只读 `frozen_final/`

## 6. 运行态不可违反的约束

- `calib_cam_extrinsics=false`
- `calib_cam_timeoffset=false`
- 运行态不写 `bootstrap/`
- 不恢复根目录旧活动配置
