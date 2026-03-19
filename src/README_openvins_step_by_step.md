# OpenVINS 实机步骤

内容：

- 真机起 Orbbec 双 IR + IMU
- 生成 OpenVINS bootstrap
- 在线标定并回写结果
- 冻结飞行参数
- 起飞行链路并给 PX4 提供外部视觉

## 1. 目录约定

以下命令默认在真机工作区根目录执行：

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

REPO=src/workspace/doco_uav/src
OV_ROOT=$REPO/uav_bringup/config/openvins/orbbec_stereo_imu
BOOT=$OV_ROOT/bootstrap/current
FROZEN=$OV_ROOT/frozen_final

mkdir -p $BOOT
```

## 2. 先只起相机

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch uav_bringup orbbec_depth_camera.launch.py \
  camera_name:=uav_depth_camera \
  enable_depth:=false \
  enable_color:=false \
  enable_point_cloud:=false \
  enable_left_ir:=true \
  enable_right_ir:=true \
  enable_accel:=true \
  enable_gyro:=true \
  enable_sync_output_accel_gyro:=true \
  enable_publish_extrinsic:=true \
  left_ir_width:=848 \
  left_ir_height:=480 \
  left_ir_fps:=30 \
  left_ir_format:=Y8 \
  right_ir_width:=848 \
  right_ir_height:=480 \
  right_ir_fps:=30 \
  right_ir_format:=Y80 \
  enable_laser:=false \
  enable_ldp:=false
```

另开一个终端检查：

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic list | grep uav_depth_camera
ros2 topic hz /uav_depth_camera/left_ir/image_raw
ros2 topic hz /uav_depth_camera/right_ir/image_raw
ros2 topic hz /uav_depth_camera/gyro_accel/sample
```

正常时至少应看到：

- `/uav_depth_camera/left_ir/image_raw`
- `/uav_depth_camera/right_ir/image_raw`
- `/uav_depth_camera/gyro_accel/sample`

## 3. 生成 bootstrap 配置

相机在发布时执行：

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

REPO=src/workspace/doco_uav/src
OV_ROOT=$REPO/uav_bringup/config/openvins/orbbec_stereo_imu
BOOT=$OV_ROOT/bootstrap/current

mkdir -p $BOOT

python3 $REPO/uav_bringup/scripts/generate_orbbec_openvins_bootstrap.py \
  --camera-name uav_depth_camera \
  --output-dir $BOOT
```

脚本会写出：

- `$BOOT/estimator_config.calibration.yaml`
- `$BOOT/estimator_config.flight.yaml`
- `$BOOT/kalibr_imucam_chain.yaml`
- `$BOOT/kalibr_imu_chain.yaml`

固定相机内参，只在线估计外参和时间偏移：

```bash
cd ~/uav_hw

REPO=src/workspace/doco_uav/src
BOOT=$REPO/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/current

sed -i 's/^calib_cam_intrinsics: true$/calib_cam_intrinsics: false/' \
  $BOOT/estimator_config.calibration.yaml

grep -nE 'calib_cam_extrinsics|calib_cam_intrinsics|calib_cam_timeoffset' \
  $BOOT/estimator_config.calibration.yaml
```

## 4. 跑一轮在线标定

定义 round 目录：

```bash
cd ~/uav_hw

REPO=src/workspace/doco_uav/src
OV_ROOT=$REPO/uav_bringup/config/openvins/orbbec_stereo_imu
BOOT=$OV_ROOT/bootstrap/current
ROUND=$OV_ROOT/bootstrap/round_01

mkdir -p $ROUND
```

启动标定：

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

REPO=src/workspace/doco_uav/src
OV_ROOT=$REPO/uav_bringup/config/openvins/orbbec_stereo_imu
BOOT=$OV_ROOT/bootstrap/current
ROUND=$OV_ROOT/bootstrap/round_01

ros2 launch uav_bringup openvins_orbbec_calibration.launch.py \
  camera_name:=uav_depth_camera \
  openvins_config_path:=$BOOT/estimator_config.calibration.yaml \
  state_output_dir:=$ROUND \
  enable_depth:=false \
  enable_color:=false \
  enable_point_cloud:=false \
  enable_left_ir:=true \
  enable_right_ir:=true \
  enable_accel:=true \
  enable_gyro:=true \
  enable_sync_output_accel_gyro:=true \
  left_ir_width:=848 \
  left_ir_height:=480 \
  left_ir_fps:=30 \
  left_ir_format:=Y8 \
  right_ir_width:=848 \
  right_ir_height:=480 \
  right_ir_fps:=30 \
  right_ir_format:=Y80 \
  enable_laser:=false \
  enable_ldp:=false
```

手持动作只做下面这套：

- 开始静止 `3 ~ 5 s`
- 平滑前后、左右、上下移动
- 加少量俯仰、横滚、偏航
- 不要猛甩，不要撞击
- 结束静止 `3 ~ 5 s`

说明：OpenVINS 可能不会在完全静止时初始化，通常要先移动几秒。

保存方式：

- 直接按 `Ctrl + C`
- `state_output_dir` 里的 `ov_estimate.txt` 会保留下来

## 5. 回写本轮结果

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

REPO=src/workspace/doco_uav/src
OV_ROOT=$REPO/uav_bringup/config/openvins/orbbec_stereo_imu
BOOT=$OV_ROOT/bootstrap/current
ROUND=$OV_ROOT/bootstrap/round_01

cp $BOOT/kalibr_imucam_chain.yaml $ROUND/kalibr_imucam_chain.before.yaml

python3 $REPO/uav_bringup/scripts/apply_openvins_calibration_result.py \
  --state-estimate-path $ROUND/ov_estimate.txt \
  --input-imucam-yaml $BOOT/kalibr_imucam_chain.yaml

cp $BOOT/kalibr_imucam_chain.yaml $ROUND/kalibr_imucam_chain.applied.yaml
```

检查结果：

```bash
cd ~/uav_hw

REPO=src/workspace/doco_uav/src
OV_ROOT=$REPO/uav_bringup/config/openvins/orbbec_stereo_imu
BOOT=$OV_ROOT/bootstrap/current
ROUND=$OV_ROOT/bootstrap/round_01

grep -nE 'T_cam_imu|T_cn_cnm1|timeshift_cam_imu' $BOOT/kalibr_imucam_chain.yaml

diff -u \
  $ROUND/kalibr_imucam_chain.before.yaml \
  $ROUND/kalibr_imucam_chain.applied.yaml
```

## 6. 继续下一轮

下一轮只需要把 `round_01` 改成 `round_02`、`round_03`。

停止迭代的简单标准：

- `timeshift_cam_imu` 连续两轮变化很小
- 双目基线始终接近 `0.05 m`
- `T_cam_imu` 不再大幅跳变
- 某一轮如果突然跳很大，直接丢弃那一轮

## 7. 冻结飞行配置

结果稳定后，将 `bootstrap/current` 复制到 `frozen_final`：

```bash
cd ~/uav_hw

REPO=src/workspace/doco_uav/src
OV_ROOT=$REPO/uav_bringup/config/openvins/orbbec_stereo_imu
BOOT=$OV_ROOT/bootstrap/current
FROZEN=$OV_ROOT/frozen_final

mkdir -p $FROZEN

cp $BOOT/kalibr_imucam_chain.yaml $FROZEN/kalibr_imucam_chain.yaml
cp $BOOT/kalibr_imu_chain.yaml $FROZEN/kalibr_imu_chain.yaml
cp $BOOT/estimator_config.flight.yaml $FROZEN/estimator_config.flight.yaml
```

## 8. 起飞行链路

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

CFG=src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final/estimator_config.flight.yaml

ros2 launch uav_bringup openvins_orbbec.launch.py \
  camera_name:=uav_depth_camera \
  openvins_config_path:=$CFG \
  publish_px4_external_vision:=true \
  fmu_namespace:=/fmu \
  sensor_roll_in_body_rad:=0.0 \
  sensor_pitch_in_body_rad:=0.0 \
  sensor_yaw_in_body_rad:=0.0 \
  enable_depth:=false \
  enable_color:=false \
  enable_point_cloud:=false \
  enable_left_ir:=true \
  enable_right_ir:=true \
  enable_accel:=true \
  enable_gyro:=true \
  enable_sync_output_accel_gyro:=true \
  left_ir_width:=848 \
  left_ir_height:=480 \
  left_ir_fps:=30 \
  left_ir_format:=Y8 \
  right_ir_width:=848 \
  right_ir_height:=480 \
  right_ir_fps:=30 \
  right_ir_format:=Y80 \
  enable_laser:=false \
  enable_ldp:=false
```

快速检查：

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo --once /ov_msckf/odomimu
ros2 topic echo --once /fmu/in/vehicle_visual_odometry
```

## 9. 关键文件

- `uav_bringup/launch/orbbec_depth_camera.launch.py`
- `uav_bringup/launch/openvins_orbbec.launch.py`
- `uav_bringup/launch/openvins_orbbec_calibration.launch.py`
- `uav_bringup/scripts/generate_orbbec_openvins_bootstrap.py`
- `uav_bringup/scripts/apply_openvins_calibration_result.py`
- `uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/current`
- `uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final`
