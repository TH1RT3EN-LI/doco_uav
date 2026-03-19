# OpenVINS 真机步骤

## 1. 生成 bootstrap

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

BOOT=src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap
mkdir -p $BOOT

python3 src/workspace/doco_uav/src/uav_bringup/scripts/generate_orbbec_openvins_bootstrap.py \
  --camera-name uav_depth_camera \
  --output-dir $BOOT
```

生成结果包含：

- `estimator_config.calibration.yaml`
- `estimator_config.flight.yaml`
- `kalibr_imucam_chain.yaml`
- `kalibr_imu_chain.yaml`
- `mask0.pgm`
- `mask1.pgm`

默认工作点：

- 双 IR
- `848x480 @ 30 Hz`
- IMU `200 Hz`
- `laser=false`
- `ldp=false`
- 在线标定只开外参与相机-IMU 时间偏移，不开内参

## 2. 在线标定

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

CFG=src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/estimator_config.calibration.yaml

ros2 launch uav_bringup openvins_orbbec_calibration.launch.py \
  openvins_config_path:=$CFG
```

动作要求：

- 开头静止 `2 ~ 3 s`；这套 calibration 配置允许静止初始化
- 中间做平滑平移、俯仰、横滚、偏航
- 近处和远处都要看到纹理
- 若未起，再给一次短促平移或小角度转动；不要长时间连续乱晃

## 3. 保存一轮结果

```bash
cd ~/uav_hw

BOOT=src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap
STATE=/tmp/openvins_orbbec_calibration/ov_estimate.txt
OUT=$BOOT/round_01_applied.yaml

python3 src/workspace/doco_uav/src/uav_bringup/scripts/apply_openvins_calibration_result.py \
  --state-estimate-path $STATE \
  --input-imucam-yaml $BOOT/kalibr_imucam_chain.yaml \
  --output-imucam-yaml $OUT
```

对比：

```bash
diff -u $BOOT/kalibr_imucam_chain.yaml $OUT
```

建议只做 `2 ~ 3` 轮。停止条件：

- 相邻两轮旋转变化 `< 0.5 deg`
- 相邻两轮平移变化 `< 0.005 m`
- `timeshift_cam_imu` 变化 `< 0.0005 s`

## 4. 冻结到飞行目录

确认收敛后覆盖：

```bash
cd ~/uav_hw

BOOT=src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap
FROZEN=src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final

cp $BOOT/estimator_config.flight.yaml $FROZEN/estimator_config.flight.yaml
cp $BOOT/kalibr_imucam_chain.yaml $FROZEN/kalibr_imucam_chain.yaml
cp $BOOT/kalibr_imu_chain.yaml $FROZEN/kalibr_imu_chain.yaml
cp $BOOT/mask0.pgm $FROZEN/mask0.pgm
cp $BOOT/mask1.pgm $FROZEN/mask1.pgm
```

## 5. 真机启动 OpenVINS + PX4 融合

当前默认入口直接使用 `frozen_final/estimator_config.flight.yaml`，并默认发布到 PX4：

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch uav_bringup openvins_orbbec.launch.py
```

如果桥接姿态和真实挂载不一致，再显式传：

```bash
sensor_roll_in_body_rad:=... \
sensor_pitch_in_body_rad:=... \
sensor_yaw_in_body_rad:=...
```

当前默认相机参数已经收口为：

- `left_ir_format:=Y8`
- `right_ir_format:=Y8`
- `enable_ir_auto_exposure:=true`
- `enable_laser:=false`
- `enable_ldp:=false`

如果快速运动时拖影明显，再手动压曝光：

```bash
ros2 launch uav_bringup openvins_orbbec.launch.py \
  enable_ir_auto_exposure:=false \
  ir_exposure:=4000 \
  ir_gain:=16
```

## 6. 手持验证

先只看 OpenVINS，不先看 PX4：

```bash
ros2 topic echo --once /ov_msckf/odomimu
```

目标：

- 静止时不持续发散
- 手持走 `1 m`，估计位移同量级
- 做 `2 m x 2 m` 小闭环，回环误差明显小于之前

图像下方两侧的浅灰机体侵入区域已经默认用 `mask0.pgm` 和 `mask1.pgm` 屏蔽。

## 7. 固化到 Git

```bash
cd ~/uav_hw/src/workspace/doco_uav/src

git add \
  uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final/estimator_config.flight.yaml \
  uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final/kalibr_imucam_chain.yaml \
  uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final/kalibr_imu_chain.yaml \
  uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final/mask0.pgm \
  uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final/mask1.pgm
```

如果当前工作区不是 `--symlink-install`，改完 launch 和 share 目录文件后需要重新 build 再 `source install/setup.bash`。
