# OpenVINS + Gemini 336 真机录制与离线批量标定

本文档把 `Gemini 336 双目 IR + 机内 IMU + OpenVINS` 的**真机录制**、**离线批量标定**、**结果回写**流程单独整理出来。

适用场景：

- 你已经能在真机上稳定拉起 `orbbec_depth_camera.launch.py`
- 你已经做过一轮 OpenVINS 在线热启动/在线细化
- 你希望继续压 `T_cam_imu`、`timeshift_cam_imu`、双目外参一致性
- 你希望把“在线迭代”升级成“录袋 + 离线 batch”流程

本文默认：

- 真机运行目录是 `~/uav_hw`
- 代码仓源目录是 `uav_hw/src/workspace/doco_uav/src`
- 离线批量标定使用单独的 `Kalibr` Docker，而不是当前项目里的 ROS2/Humble 开发容器


## 1. 总体思路

推荐采用三袋法：

- `stereo_static`：只录双目图像，用于离线双目标定/双目结果验证
- `vi_dynamic`：录双目图像 + IMU，用于离线 `camera-imu` 时空标定
- `imu_allan`：只录 IMU 长静止数据，用于 Allan 方差估计 IMU 噪声

建议顺序：

1. 先做 `stereo_static`
2. 再做 `vi_dynamic`
3. 最后有时间再做 `imu_allan`

如果你现在只想尽快把主链路压稳，至少先完成前两项。


## 2. 推荐标定板：标准 Aprilgrid

### 2.1 推荐版本

优先使用 `Aprilgrid 6x6, 0.8 x 0.8 m, A0` 版本。

这是 OpenVINS 文档里直接推荐的版本，也是 Kalibr 常见的标准板之一。

### 2.2 标定板下载链接

- OpenVINS 标定说明页：
  - `https://docs.openvins.com/gs-calibration.html`
- Kalibr 标定总说明页：
  - `https://github.com/ethz-asl/kalibr/wiki/calibrating-the-vi-sensor`
- Kalibr 标定板说明页：
  - `https://github.com/ethz-asl/kalibr/wiki/calibration-targets`

OpenVINS 文档中给出的推荐标准板下载地址：

- 标准板 PDF：
  - `https://drive.google.com/file/d/14dY7z8pDb2iEBdveTviDXsoi5H9AaQP1/view?usp=drive_link`
- 标准板 YAML：
  - `https://drive.google.com/file/d/1zXfr48_OY0RafwJalBLjqkqgnme-r7Gd/view?usp=drive_link`

### 2.3 标定板 YAML 关键参数

官方推荐 Aprilgrid 参数等价于：

```yaml
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.088
tagSpacing: 0.3
```

### 2.4 打印和使用注意事项

- 必须 **100% 实际尺寸打印**，不要让打印店自动缩放
- 最好贴到**平整硬板**上，例如亚克力板、铝板、KT 板硬底
- 打印后请**实际测量** `tagSize`，不要盲信打印输出
- 如果打印店缩放过，必须同步修改 target YAML，不然离线结果会系统性偏掉
- 建议保留较宽白边，Kalibr 官方说明也建议保留白边


## 3. 真机录制前的准备

### 3.1 真机只起原始相机驱动

录袋阶段建议：

- **只起 Orbbec 原始驱动**
- **不要同时跑 OpenVINS**

这样做的好处：

- 录下来的数据最干净
- 后续袋子转换更简单
- 避免在线算法本身影响调试判断

### 3.2 采集参数要和实际运行一致

Gemini 336 当前建议保持和你在线 VIO 一致的参数：

- 左右 IR：`848x480`
- 帧率：`30 Hz`
- 格式：`Y8`
- IMU：`200 Hz`

### 3.3 建议创建离线标定工作目录

```bash
cd ~/uav_hw
mkdir -p ~/uav_hw/calib/gemini336_batch_01/{bags_ros2,bags_ros1,targets,results_cam,results_imucam,results_allan}
```


## 4. 真机启动 Gemini 336 原始数据流

在第一个终端执行：

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch uav_bringup orbbec_depth_camera.launch.py \
  camera_name:=uav_depth_camera \
  enable_left_ir:=true \
  enable_right_ir:=true \
  enable_accel:=true \
  enable_gyro:=true \
  enable_sync_output_accel_gyro:=true \
  enable_publish_extrinsic:=false \
  enable_depth:=false \
  enable_color:=false \
  enable_point_cloud:=false \
  left_ir_width:=848 \
  left_ir_height:=480 \
  left_ir_fps:=30 \
  left_ir_format:=Y8 \
  right_ir_width:=848 \
  right_ir_height:=480 \
  right_ir_fps:=30 \
  right_ir_format:=Y8
```

建议先确认：

- `/uav_depth_camera/left_ir/image_raw`
- `/uav_depth_camera/right_ir/image_raw`
- `/uav_depth_camera/gyro_accel/sample`

都在正常发布。


## 5. 录制 `stereo_static`

### 5.1 用途

用于：

- 双目标定
- 双目外参与重投影误差检查
- 生成/验证 `camchain.yaml`

### 5.2 动作要求

- 相机固定不动
- Aprilgrid 板在相机前移动
- 覆盖中心、四角、近距离、远距离、倾斜角度
- 时长建议 `60 ~ 90 s`
- 尽量避免运动模糊

### 5.3 录制命令

在第二个终端执行：

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 bag record --storage sqlite3 \
  -o ~/uav_hw/calib/gemini336_batch_01/bags_ros2/stereo_static \
  /uav_depth_camera/left_ir/image_raw \
  /uav_depth_camera/right_ir/image_raw
```

录完后按 `Ctrl + C` 结束。


## 6. 录制 `vi_dynamic`

### 6.1 用途

用于离线求解：

- `cam0.T_cam_imu`
- `cam1.T_cam_imu`
- `timeshift_cam_imu`

### 6.2 动作要求

- Aprilgrid 固定
- 手持整机在板前做平滑运动
- 做前后、左右、上下、小角度俯仰/横滚/偏航组合运动
- 起始静止约 `5 s`
- 结束静止约 `5 s`
- 中间主体运动约 `30 ~ 60 s`
- **不要猛甩，不要冲击，不要剧烈震动**

### 6.3 录制命令

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 bag record --storage sqlite3 \
  -o ~/uav_hw/calib/gemini336_batch_01/bags_ros2/vi_dynamic \
  /uav_depth_camera/left_ir/image_raw \
  /uav_depth_camera/right_ir/image_raw \
  /uav_depth_camera/gyro_accel/sample
```

录完后按 `Ctrl + C` 结束。


## 7. 录制 `imu_allan`

### 7.1 用途

用于离线估计 IMU 噪声：

- `gyroscope_noise_density`
- `gyroscope_random_walk`
- `accelerometer_noise_density`
- `accelerometer_random_walk`

### 7.2 动作要求

- 整机完全静止
- 放在稳定支撑面上
- 时长至少 `3 h`
- 更理想是 `8 ~ 20 h`

### 7.3 录制命令

```bash
cd ~/uav_hw
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 bag record --storage sqlite3 \
  -o ~/uav_hw/calib/gemini336_batch_01/bags_ros2/imu_allan \
  /uav_depth_camera/gyro_accel/sample
```


## 8. 录制完成后的最小检查

```bash
ros2 bag info ~/uav_hw/calib/gemini336_batch_01/bags_ros2/stereo_static
ros2 bag info ~/uav_hw/calib/gemini336_batch_01/bags_ros2/vi_dynamic
ros2 bag info ~/uav_hw/calib/gemini336_batch_01/bags_ros2/imu_allan
```

检查点：

- `stereo_static` 只包含左右图像
- `vi_dynamic` 包含左右图像 + IMU
- `imu_allan` 只包含 IMU
- 不要使用 split bag；后续 `rosbags-convert` 对 split bag 支持有限


## 9. ROS2 袋转 ROS1 袋

Kalibr 主要工作流仍以 ROS1 bag 为主，因此建议先转换。

参考：

- Rosbags 转换文档：
  - `https://ternaris.gitlab.io/rosbags/topics/convert.html`

注意：

- `rosbag2 -> rosbag1` 时，优先只录标准消息类型
- 当前转换工具对 split bag 支持有限

### 9.1 安装转换工具

```bash
python3 -m pip install --user -U rosbags
```

### 9.2 转换命令

```bash
~/.local/bin/rosbags-convert \
  ~/uav_hw/calib/gemini336_batch_01/bags_ros2/stereo_static \
  --dst ~/uav_hw/calib/gemini336_batch_01/bags_ros1/stereo_static.bag

~/.local/bin/rosbags-convert \
  ~/uav_hw/calib/gemini336_batch_01/bags_ros2/vi_dynamic \
  --dst ~/uav_hw/calib/gemini336_batch_01/bags_ros1/vi_dynamic.bag

~/.local/bin/rosbags-convert \
  ~/uav_hw/calib/gemini336_batch_01/bags_ros2/imu_allan \
  --dst ~/uav_hw/calib/gemini336_batch_01/bags_ros1/imu_allan.bag
```


## 10. Kalibr Docker 准备

当前项目仓库没有内置可直接拿来做 `Kalibr` 的容器环境，建议单独使用 Kalibr 官方仓库自带的 ROS1 Dockerfile。

Kalibr 仓库：

- `https://github.com/ethz-asl/kalibr`

### 10.1 克隆 Kalibr

```bash
mkdir -p ~/tooling
cd ~/tooling
git clone https://github.com/ethz-asl/kalibr.git
cd kalibr
```

### 10.2 构建 Kalibr Docker

```bash
docker build -t kalibr -f Dockerfile_ros1_20_04 .
```


## 11. 进入 Kalibr Docker

```bash
FOLDER=~/uav_hw/calib/gemini336_batch_01

xhost +local:root

docker run --rm -it \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$FOLDER:/data" \
  kalibr
```

进入容器后建议先：

```bash
source devel/setup.bash
```


## 12. 准备 target YAML

如果你使用的是上面推荐的标准板，并且确认打印尺寸没有变化，可以将官方 YAML 下载后放到：

- `/data/targets/april_6x6_80x80cm.yaml`

如果你想直接在本地写一个同等配置，也可以：

```bash
cat > /data/targets/april_6x6_80x80cm.yaml <<'EOF'
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.088
tagSpacing: 0.3
EOF
```

只有在**打印实际尺寸与官方尺寸完全一致**时，才允许直接用这份参数。


## 13. 离线双目标定

### 13.1 目的

- 估计/验证双目外参
- 输出 `camchain` 文件
- 检查重投影误差

### 13.2 命令

在 Kalibr 容器里执行：

```bash
source devel/setup.bash
cd /data/results_cam

rosrun kalibr kalibr_calibrate_cameras \
  --bag /data/bags_ros1/stereo_static.bag \
  --target /data/targets/april_6x6_80x80cm.yaml \
  --models pinhole-radtan pinhole-radtan \
  --topics /uav_depth_camera/left_ir/image_raw /uav_depth_camera/right_ir/image_raw \
  --bag-freq 10.0
```

### 13.3 输出文件

通常会得到：

- `camchain-*.yaml`
- `results-cam-*.txt`
- `report-cam-*.pdf`


## 14. 离线 Camera-IMU 标定

### 14.1 目的

- 求 `T_cam_imu`
- 求 `timeshift_cam_imu`

### 14.2 准备 IMU 配置

第一轮可以直接使用当前 OpenVINS 的 `kalibr_imu_chain.yaml` 作为初始 IMU 配置。

建议在宿主机先复制一份：

```bash
cp ~/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/kalibr_imu_chain.yaml \
   ~/uav_hw/calib/gemini336_batch_01/targets/imu_seed.yaml
```

如果后面完成了 Allan 方差分析，再把新的噪声参数写回 `imu_seed.yaml` 后重跑这一节。

### 14.3 命令

Kalibr 官方 `camera-imu` 文档说明：**时间标定默认关闭**，如果要估计时间偏移，需要显式加 `--time-calibration`。

在 Kalibr 容器里执行：

```bash
source devel/setup.bash
cd /data/results_imucam

rosrun kalibr kalibr_calibrate_imu_camera \
  --bag /data/bags_ros1/vi_dynamic.bag \
  --cam /data/results_cam/camchain-stereo_static.yaml \
  --imu /data/targets/imu_seed.yaml \
  --target /data/targets/april_6x6_80x80cm.yaml \
  --time-calibration
```

如果你的 `results_cam` 输出文件名不是 `camchain-stereo_static.yaml`，请按实际文件名替换。

### 14.4 输出文件

通常会得到：

- `camchain-imucam-*.yaml`
- `results-imucam-*.txt`
- `report-imucam-*.pdf`


## 15. Allan 方差（可选但推荐）

如果你要把 IMU 噪声也压实，建议再单独跑 `allan_variance_ros`。

参考：

- `https://github.com/ori-drs/allan_variance_ros`

这一部分的输出最终只需要回写到 OpenVINS 的：

- `gyroscope_noise_density`
- `gyroscope_random_walk`
- `accelerometer_noise_density`
- `accelerometer_random_walk`

注意：

- 不要盲目整文件覆盖
- 只更新这四个核心噪声字段和必要的 `update_rate` / `rostopic`


## 16. 如何判断离线结果是否靠谱

建议按以下顺序看：

### 16.1 双目几何

Gemini 336 的官方基线约为 `50 mm`。

因此：

- `T_cn_cnm1.x` 应接近 `-0.05 m`
- `y/z` 不应离谱偏大
- 左右相机相对姿态不应出现明显大角度异常

### 16.2 时间偏移

你在线阶段较好的结果大约在 `7 ms` 量级。

离线 batch 的 `timeshift_cam_imu`：

- 如果仍在相近量级，通常是正常的
- 如果突然跳到很离谱的量级，需要优先检查袋子质量和时间戳一致性

### 16.3 重投影误差

经验上：

- `< 0.5 px`：比较好
- `0.5 ~ 1.0 px`：可接受但还可继续优化
- `> 1.0 px`：通常说明数据质量、板尺寸、焦点、模糊或时间同步仍有问题

### 16.4 IMU 拟合情况

重点检查 PDF 报告中：

- 加计残差是否明显炸开
- 陀螺残差是否明显炸开
- 估计偏置是否长期冲出合理范围


## 17. 回写到 OpenVINS

### 17.1 先备份现有工作配置

```bash
cp ~/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/kalibr_imucam_chain.yaml \
   ~/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/kalibr_imucam_chain.pre_offline_batch.yaml

cp ~/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/kalibr_imu_chain.yaml \
   ~/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/kalibr_imu_chain.pre_offline_batch.yaml
```

### 17.2 用离线结果覆盖 `kalibr_imucam_chain.yaml`

```bash
cp ~/uav_hw/calib/gemini336_batch_01/results_imucam/camchain-imucam-vi_dynamic.yaml \
   ~/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/kalibr_imucam_chain.yaml
```

如果文件名不同，请按实际输出文件名替换。

### 17.3 更新 `kalibr_imu_chain.yaml`

如果完成了 Allan 方差分析，只更新噪声相关字段：

- `gyroscope_noise_density`
- `gyroscope_random_walk`
- `accelerometer_noise_density`
- `accelerometer_random_walk`


## 18. 离线结果通过后怎么冻结

建议额外做一个独立冻结目录，避免后续 `git pull`、在线 round、临时改动把它冲掉：

```bash
mkdir -p ~/uav_hw/calib/frozen_gemini336_final

cp ~/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/kalibr_imucam_chain.yaml \
   ~/uav_hw/calib/frozen_gemini336_final/

cp ~/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/kalibr_imu_chain.yaml \
   ~/uav_hw/calib/frozen_gemini336_final/
```

推荐同时保存：

- 原始 `ros2 bag`
- 转换后的 `ros1 bag`
- Kalibr 输出 PDF
- Kalibr 输出 YAML
- 当前最终使用的 OpenVINS YAML


## 19. 最终建议

对 Gemini 336 这条链路，推荐采用下面的收敛策略：

1. 保留工厂内参，不再在线继续放开 `calib_cam_intrinsics`
2. 离线重点压 `T_cam_imu` 与 `timeshift_cam_imu`
3. 用双目离线结果验证基线是否稳定贴近 `50 mm`
4. 如果离线结果比当前在线最佳结果更稳，再覆盖飞行配置
5. 最终飞行配置建议 freeze，不再在线继续改外参/时间偏移


## 20. 参考链接

- OpenVINS 标定总览：
  - `https://docs.openvins.com/gs-calibration.html`
- Kalibr 仓库：
  - `https://github.com/ethz-asl/kalibr`
- Kalibr 标定总说明：
  - `https://github.com/ethz-asl/kalibr/wiki/calibrating-the-vi-sensor`
- Kalibr 标定板说明：
  - `https://github.com/ethz-asl/kalibr/wiki/calibration-targets`
- Kalibr 多相机标定：
  - `https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration`
- Kalibr Camera-IMU 标定：
  - `https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration`
- Rosbags 转换文档：
  - `https://ternaris.gitlab.io/rosbags/topics/convert.html`
- Allan 方差工具：
  - `https://github.com/ori-drs/allan_variance_ros`

