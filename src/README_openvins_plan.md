# OpenVINS 项目约定

记录链路、目录、配置规则和 PX4 参数。

## 1. 链路

链路：

```text
Gemini 336
  -> left_ir/right_ir/gyro_accel
  -> OpenVINS
  -> /ov_msckf/odomimu
  -> openvins_px4_vision_bridge
  -> /fmu/in/vehicle_visual_odometry
  -> PX4 EKF2
```

不使用：

- 不把 depth 接进 OpenVINS 主链路
- 不把 RGB 接进 OpenVINS 主链路
- 不保留旧的光流主定位方案

## 2. 有效目录

仓库根目录下只关注：

- `uav_bringup/launch`
- `uav_bringup/scripts`
- `uav_bringup/config/openvins/orbbec_stereo_imu`
- `uav_bridge/src`

其中最重要的是：

- `uav_bringup/launch/openvins_orbbec.launch.py`
- `uav_bringup/launch/openvins_orbbec_calibration.launch.py`
- `uav_bringup/scripts/generate_orbbec_openvins_bootstrap.py`
- `uav_bringup/scripts/apply_openvins_calibration_result.py`
- `uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final`

## 3. 配置使用规则

- 在线标定时使用 `bootstrap/current`
- 飞行时使用 `frozen_final`
- `frozen_final/estimator_config.flight.yaml` 里保持：
  - `calib_cam_extrinsics: false`
  - `calib_cam_intrinsics: false`
  - `calib_cam_timeoffset: false`
- `kalibr_imucam_chain.yaml` 里保存相机到 IMU 外参和时间偏移
- `kalibr_imu_chain.yaml` 里保存 IMU 噪声参数

## 4. PX4 参数

融合方式：

- 水平位置、速度、航向：使用 OpenVINS
- 高度：优先使用测距，气压计兜底
- 关闭 GPS 融合
- 关闭光流融合
- 关闭磁罗盘融合

参数：

```text
EKF2_EV_CTRL   13
EKF2_EV_DELAY  0
EKF2_GPS_CTRL  0
EKF2_OF_CTRL   0
SYS_HAS_MAG    0
EKF2_HGT_REF   2
EKF2_RNG_CTRL  2
EKF2_BARO_CTRL 1
```

说明：

- `EKF2_EV_POS_X/Y/Z` 用来补外部视觉传感器相对机体系原点的平移
- 桥接节点只处理旋转，不处理平移
- 所以平移补偿仍然交给 PX4

## 5. 起飞前最少检查

```bash
ros2 topic echo --once /ov_msckf/odomimu
ros2 topic echo --once /fmu/in/vehicle_visual_odometry
```

满足下面三条再进入飞行测试：

- OpenVINS 已初始化
- `/ov_msckf/odomimu` 数值量级正常，不发散
- `/fmu/in/vehicle_visual_odometry` 持续发布

## 6. 参数如何固化到 Git

如果你在真机上已经确认 `frozen_final` 可用，提交这三个文件即可：

```bash
cd ~/uav_hw/src/workspace/doco_uav/src

git add \
  uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final/estimator_config.flight.yaml \
  uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final/kalibr_imucam_chain.yaml \
  uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final/kalibr_imu_chain.yaml

git commit -m "freeze openvins gemini336 calibration"
git push origin main
```
