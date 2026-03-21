# OpenVINS 标定与运行唯一流程

本文档定义当前项目唯一允许的 OpenVINS 工作流。

## 1. 两套唯一真相

```bash
cd ~/uav_hw/src/workspace/doco_uav/src

OV_ROOT=uav_bringup/config/openvins/orbbec_stereo_imu
BOOT=$OV_ROOT/bootstrap
FROZEN=$OV_ROOT/frozen_final
```

- `bootstrap/`：标定工作区，可以反复覆盖。
- `frozen_final/`：运行配置区，所有真机运行入口默认只读这里。
- 根目录 `orbbec_stereo_imu/*.yaml` / `*.pgm` 已不再是合法活动入口。

## 2. 生成 bootstrap

先起仅提供标定源数据的相机入口。

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch uav_bringup openvins_orbbec_bootstrap_source.launch.py
```

这个 launch 固定开启：

- `enable_depth=true`
- `enable_publish_extrinsic=true`
- `enable_sync_output_accel_gyro=true`
- `enable_left_ir=true`
- `enable_right_ir=true`
- `enable_color=false`
- `enable_point_cloud=false`

然后在另一个终端生成 `bootstrap/` 工作区：

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run uav_bringup generate_orbbec_openvins_bootstrap.py
```

脚本固定只消费：

- 左右 `CameraInfo`
- `depth_to_left_ir`
- `depth_to_right_ir`
- `depth_to_gyro`
- `/gyro_accel/sample` 这一套 gyro reference 语义

缺这些输入会直接失败。`gyro/imu_info` 和 `accel/imu_info` 可以缺失，但会明确警告并回退到项目默认噪声值。

生成完成后，`bootstrap/` 里应至少有：

- `estimator_config.calibration.yaml`
- `estimator_config.flight.yaml`
- `kalibr_imucam_chain.yaml`
- `kalibr_imu_chain.yaml`
- `mask0.pgm`
- `mask1.pgm`
- `bootstrap_summary.md`

## 3. 运行在线标定

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

STATE=/tmp/openvins_orbbec_calibration

ros2 launch uav_bringup openvins_orbbec_calibration.launch.py \
  state_output_dir:=$STATE
```

这个标定 launch 会强制：

- `openvins_save_total_state=true`
- `openvins_publish_calibration_tf=true`
- `publish_px4_external_vision=false`
- `enable_depth=true`
- `enable_publish_extrinsic=true`
- `enable_sync_output_accel_gyro=true`

它不会再暴露会破坏标定语义的旧默认开关。

## 4. 把结果原地 apply 回 bootstrap

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

STATE=/tmp/openvins_orbbec_calibration
OV_ROOT=src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu
BOOT=$OV_ROOT/bootstrap

ros2 run uav_bringup apply_openvins_calibration_result.py \
  --state-estimate-path $STATE/ov_estimate.txt \
  --input-imucam-yaml $BOOT/kalibr_imucam_chain.yaml
```

默认行为就是**原地覆盖** `bootstrap/kalibr_imucam_chain.yaml`。

脚本默认会在以下情况直接非零退出，并拒绝写回：

- 最终速度过高
- stereo baseline 变化过大
- stereo relative rotation 变化过大

只有显式传 `--force` 才允许绕过：

```bash
ros2 run uav_bringup apply_openvins_calibration_result.py \
  --state-estimate-path $STATE/ov_estimate.txt \
  --input-imucam-yaml $BOOT/kalibr_imucam_chain.yaml \
  --force
```

## 5. 多轮收敛

多轮标定时，重复下面两步：

1. 重新运行 `openvins_orbbec_calibration.launch.py`
2. 再次把最新结果原地 apply 回 `bootstrap/kalibr_imucam_chain.yaml`

不再使用 `round_01_applied.yaml` 这类旁路文件作为默认工作流。

## 6. freeze 到运行目录

确认标定收敛后，再一次性冻结到运行目录：

```bash
cd ~/uav_hw

OV_ROOT=src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu
BOOT=$OV_ROOT/bootstrap
FROZEN=$OV_ROOT/frozen_final

mkdir -p $FROZEN
cp $BOOT/estimator_config.flight.yaml $FROZEN/estimator_config.flight.yaml
cp $BOOT/kalibr_imucam_chain.yaml $FROZEN/kalibr_imucam_chain.yaml
cp $BOOT/kalibr_imu_chain.yaml $FROZEN/kalibr_imu_chain.yaml
cp $BOOT/mask0.pgm $FROZEN/mask0.pgm
cp $BOOT/mask1.pgm $FROZEN/mask1.pgm
```

冻结完成后，运行态只认 `frozen_final/`。

## 7. 真机运行

运行阶段默认入口：

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch uav_bringup openvins_orbbec.launch.py
```

或者由最小控制入口带起：

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch uav_bringup minimal_control.launch.py \
  start_openvins_orbbec:=true
```

这两个入口默认都读取：

- `uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final/estimator_config.flight.yaml`

## 8. 运行阶段硬规则

- 运行阶段不允许开启 `calib_cam_extrinsics`
- 运行阶段不允许开启 `calib_cam_timeoffset`
- 不允许再把 `bootstrap/` 当运行目录
- 不允许再恢复根目录 `orbbec_stereo_imu/*.yaml` / `*.pgm` 活动副本

## 9. 快速自检

```bash
cd ~/uav_hw/src/workspace/doco_uav/src

find uav_bringup/config/openvins/orbbec_stereo_imu -maxdepth 2 -type f | sort
```

你应当看到：

- `bootstrap/` 下的工作文件
- `frozen_final/` 下的运行文件

但不应再看到根目录活动版：

- 根目录旧的运行配置 YAML
- 根目录旧的相机-IMU 链 YAML
- 根目录旧的 IMU 链 YAML
- 根目录旧的遮罩文件
