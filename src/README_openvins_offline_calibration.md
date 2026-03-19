# OpenVINS 离线标定

内容：

- 真机录制原始数据
- 准备 Aprilgrid
- 本机运行 Kalibr
- 把离线结果回写到项目配置


## 1. 目录约定

```bash
cd ~/uav_hw

REPO=src/workspace/doco_uav/src
OV_ROOT=$REPO/uav_bringup/config/openvins/orbbec_stereo_imu
CALIB=calib/gemini336_batch_01

mkdir -p $CALIB/{bags_ros2,bags_ros1,targets,results_cam,results_imucam,results_allan}
```

## 2. 标定板

使用 `Aprilgrid 6x6`。

参考链接：

- OpenVINS calibration guide: <https://docs.openvins.com/gs-calibration.html>
- Kalibr VI calibration: <https://github.com/ethz-asl/kalibr/wiki/calibrating-the-vi-sensor>
- Kalibr targets: <https://github.com/ethz-asl/kalibr/wiki/calibration-targets>
- OpenVINS 文档中给出的 Aprilgrid PDF: <https://drive.google.com/file/d/14dY7z8pDb2iEBdveTviDXsoi5H9AaQP1/view?usp=drive_link>
- OpenVINS 文档中给出的 Aprilgrid YAML: <https://drive.google.com/file/d/1zXfr48_OY0RafwJalBLjqkqgnme-r7Gd/view?usp=drive_link>

如果你本地自己建 target 文件，可以直接写：

```bash
cd ~/uav_hw

CALIB=calib/gemini336_batch_01

cat > $CALIB/targets/aprilgrid_6x6_088.yaml <<'YAML'
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.088
tagSpacing: 0.3
YAML
```

## 3. 录制前只起原始相机

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch uav_bringup orbbec_depth_camera.launch.py \
  enable_publish_extrinsic:=false
```

## 4. 录 `stereo_static`

用途：双目标定和双目结果检查。

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

CALIB=calib/gemini336_batch_01

ros2 bag record --storage sqlite3 \
  -o $CALIB/bags_ros2/stereo_static \
  /uav_depth_camera/left_ir/image_raw \
  /uav_depth_camera/right_ir/image_raw
```

动作要求：

- 相机固定
- 板子移动
- 覆盖近、中、远和四角
- 录 `60 ~ 90 s`

## 5. 录 `vi_dynamic`

用途：离线求 `T_cam_imu` 和 `timeshift_cam_imu`。

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

CALIB=calib/gemini336_batch_01

ros2 bag record --storage sqlite3 \
  -o $CALIB/bags_ros2/vi_dynamic \
  /uav_depth_camera/left_ir/image_raw \
  /uav_depth_camera/right_ir/image_raw \
  /uav_depth_camera/gyro_accel/sample
```

动作要求：

- 板子固定
- 手持整机平滑运动
- 开头静止 `5 s`
- 中间运动 `30 ~ 60 s`
- 结尾静止 `5 s`

## 6. 录 `imu_allan`

用途：离线估 IMU 噪声。

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

CALIB=calib/gemini336_batch_01

ros2 bag record --storage sqlite3 \
  -o $CALIB/bags_ros2/imu_allan \
  /uav_depth_camera/gyro_accel/sample
```

动作要求：

- 整机完全静止
- 至少 `3 h`

## 7. 转成 rosbag1

如果你本机已经安装 `rosbags`：

```bash
python3 -m pip install --user -U rosbags
```

转换：

```bash
cd ~/uav_hw

CALIB=calib/gemini336_batch_01

~/.local/bin/rosbags-convert \
  $CALIB/bags_ros2/stereo_static \
  --dst $CALIB/bags_ros1/stereo_static.bag

~/.local/bin/rosbags-convert \
  $CALIB/bags_ros2/vi_dynamic \
  --dst $CALIB/bags_ros1/vi_dynamic.bag

~/.local/bin/rosbags-convert \
  $CALIB/bags_ros2/imu_allan \
  --dst $CALIB/bags_ros1/imu_allan.bag
```

## 8. 本机跑 Kalibr

以下命令假设你已经在本机装好了 Kalibr，并且能正常使用 `kalibr_calibrate_cameras` 和 `kalibr_calibrate_imu_camera`。

双目标定：

```bash
cd ~/uav_hw

CALIB=calib/gemini336_batch_01

kalibr_calibrate_cameras \
  --bag $CALIB/bags_ros1/stereo_static.bag \
  --target $CALIB/targets/aprilgrid_6x6_088.yaml \
  --models pinhole-radtan pinhole-radtan \
  --topics /uav_depth_camera/left_ir/image_raw /uav_depth_camera/right_ir/image_raw \
  --output-folder $CALIB/results_cam
```

相机-IMU 标定：

```bash
cd ~/uav_hw

REPO=src/workspace/doco_uav/src
OV_ROOT=$REPO/uav_bringup/config/openvins/orbbec_stereo_imu
CALIB=calib/gemini336_batch_01

cp $OV_ROOT/kalibr_imu_chain.yaml $CALIB/results_imucam/imu_initial.yaml

kalibr_calibrate_imu_camera \
  --bag $CALIB/bags_ros1/vi_dynamic.bag \
  --cam $CALIB/results_cam/camchain-stereo_static.yaml \
  --imu $CALIB/results_imucam/imu_initial.yaml \
  --target $CALIB/targets/aprilgrid_6x6_088.yaml \
  --output-folder $CALIB/results_imucam
```

## 9. 把离线结果回写到项目

先备份：

```bash
cd ~/uav_hw

REPO=src/workspace/doco_uav/src
OV_ROOT=$REPO/uav_bringup/config/openvins/orbbec_stereo_imu

cp $OV_ROOT/kalibr_imucam_chain.yaml $OV_ROOT/kalibr_imucam_chain.pre_offline.yaml
cp $OV_ROOT/kalibr_imu_chain.yaml $OV_ROOT/kalibr_imu_chain.pre_offline.yaml
```

再覆盖：

```bash
cd ~/uav_hw

REPO=src/workspace/doco_uav/src
OV_ROOT=$REPO/uav_bringup/config/openvins/orbbec_stereo_imu
CALIB=calib/gemini336_batch_01

cp $CALIB/results_imucam/camchain-imucam-vi_dynamic.yaml $OV_ROOT/kalibr_imucam_chain.yaml
```

如果你还做了 Allan 或重新估计了 IMU 噪声，再把更新后的 IMU YAML 一起覆盖到：

```bash
cd ~/uav_hw

REPO=src/workspace/doco_uav/src
OV_ROOT=$REPO/uav_bringup/config/openvins/orbbec_stereo_imu
CALIB=calib/gemini336_batch_01

cp $CALIB/results_imucam/imu_initial.yaml $OV_ROOT/kalibr_imu_chain.yaml
```

## 10. 再冻结到飞行目录

```bash
cd ~/uav_hw

REPO=src/workspace/doco_uav/src
OV_ROOT=$REPO/uav_bringup/config/openvins/orbbec_stereo_imu
FROZEN=$OV_ROOT/frozen_final

mkdir -p $FROZEN
cp $OV_ROOT/kalibr_imucam_chain.yaml $FROZEN/kalibr_imucam_chain.yaml
cp $OV_ROOT/kalibr_imu_chain.yaml $FROZEN/kalibr_imu_chain.yaml
```
