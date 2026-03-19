# OpenVINS 项目约定

## 1. 当前链路

```text
Gemini 336
  -> left_ir / right_ir / gyro_accel
  -> OpenVINS
  -> /ov_msckf/odomimu
  -> openvins_px4_vision_bridge
  -> /fmu/in/vehicle_visual_odometry
  -> PX4 EKF2
```

主链路只保留：

- 双 IR
- OpenVINS
- PX4 external vision

不把 depth 和 RGB 接到 OpenVINS 主链路。

## 2. 当前默认值

- 双 IR `848x480 @ 30 Hz`
- IMU `200 Hz`
- `track_frequency: 30`
- `use_mask: true`
- `mask0.pgm` / `mask1.pgm` 默认屏蔽图像左右下角机体侵入区
- `enable_laser: false`
- `enable_ldp: false`
- `left_ir_format: Y8`
- `right_ir_format: Y8`

## 3. 配置目录

只关注：

- `uav_bringup/launch`
- `uav_bringup/scripts`
- `uav_bringup/config/openvins/orbbec_stereo_imu`
- `uav_bridge/src`

飞行默认使用：

- `uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final/estimator_config.flight.yaml`

在线标定使用：

- `uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/estimator_config.calibration.yaml`

## 4. PX4 侧约束

- 桥接节点当前只补旋转，不补平移
- 相机/IMU 相对机体的平移仍然要在 PX4 侧建模
- 如果桥接姿态与真实挂载不一致，启动时必须显式传 `sensor_roll_in_body_rad`、`sensor_pitch_in_body_rad`、`sensor_yaw_in_body_rad`
- 不要把仓库里的旧参数文件默认当成真机当前参数

## 5. 起飞前最少检查

```bash
ros2 topic echo --once /ov_msckf/odomimu
ros2 topic echo --once /fmu/in/vehicle_visual_odometry
```

满足下面三条再进入飞行测试：

- OpenVINS 已初始化
- `/ov_msckf/odomimu` 数值量级正常，不发散
- `/fmu/in/vehicle_visual_odometry` 持续发布

## 6. Git 固化

当前冻结参数至少提交：

```bash
cd ~/uav_hw/src/workspace/doco_uav/src

git add \
  uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final/estimator_config.flight.yaml \
  uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final/kalibr_imucam_chain.yaml \
  uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final/kalibr_imu_chain.yaml \
  uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final/mask0.pgm \
  uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final/mask1.pgm
```
