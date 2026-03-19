# OpenVINS 离线标定

## 1. 目标板

当前使用 `Aprilgrid 6x6`。

参考：

- Kalibr VI calibration: <https://github.com/ethz-asl/kalibr/wiki/calibrating-the-vi-sensor>
- Kalibr calibration targets: <https://github.com/ethz-asl/kalibr/wiki/calibration-targets>
- OpenVINS 文档中的 Aprilgrid PDF: <https://drive.google.com/file/d/14dY7z8pDb2iEBdveTviDXsoi5H9AaQP1/view?usp=drive_link>
- OpenVINS 文档中的 Aprilgrid YAML: <https://drive.google.com/file/d/1zXfr48_OY0RafwJalBLjqkqgnme-r7Gd/view?usp=drive_link>

本地保存一个目标参数文件：

```bash
cd ~/uav_hw
mkdir -p calib/gemini336_batch_01/targets

cat > calib/gemini336_batch_01/targets/aprilgrid_6x6_088.yaml <<'YAML'
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.088
tagSpacing: 0.3
YAML
```

## 2. 录数据

### 双目静态

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

### VI 动态

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

### IMU Allan

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

CALIB=calib/gemini336_batch_01

ros2 bag record --storage sqlite3 \
  -o $CALIB/bags_ros2/imu_allan \
  /uav_depth_camera/gyro_accel/sample
```

## 3. 转 rosbag1

```bash
cd ~/uav_hw
python3 -m pip install --user -U rosbags

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

## 4. 跑 Kalibr

### 双目标定

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

### 相机-IMU 标定

```bash
cd ~/uav_hw

OV_ROOT=src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu
CALIB=calib/gemini336_batch_01

cp $OV_ROOT/kalibr_imu_chain.yaml $CALIB/results_imucam/imu_initial.yaml

kalibr_calibrate_imu_camera \
  --bag $CALIB/bags_ros1/vi_dynamic.bag \
  --cam $CALIB/results_cam/camchain-stereo_static.yaml \
  --imu $CALIB/results_imucam/imu_initial.yaml \
  --target $CALIB/targets/aprilgrid_6x6_088.yaml \
  --output-folder $CALIB/results_imucam
```

## 5. 回写项目并冻结

```bash
cd ~/uav_hw

OV_ROOT=src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu
FROZEN=$OV_ROOT/frozen_final
CALIB=calib/gemini336_batch_01

cp $CALIB/results_imucam/camchain-imucam-vi_dynamic.yaml $OV_ROOT/kalibr_imucam_chain.yaml
cp $CALIB/results_imucam/imu_initial.yaml $OV_ROOT/kalibr_imu_chain.yaml

mkdir -p $FROZEN
cp $OV_ROOT/kalibr_imucam_chain.yaml $FROZEN/kalibr_imucam_chain.yaml
cp $OV_ROOT/kalibr_imu_chain.yaml $FROZEN/kalibr_imu_chain.yaml
```
