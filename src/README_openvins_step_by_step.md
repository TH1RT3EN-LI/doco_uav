# OpenVINS + Orbbec 双 IR + 相机 IMU 从零执行手册

这是一份**独立执行版 README**。

它不再讨论大规划，只回答三件事：

1. **现在要做什么**
2. **具体执行什么命令**
3. **怎么判断这一步做对了**

当前这份手册默认你使用的是：

- Orbbec 深度相机
- `left_ir`
- `right_ir`
- 相机自带 IMU：`gyro_accel/sample`
- OpenVINS
- ROS 2 Humble
- PX4 通过外部视觉接口接收 OpenVINS 结果

当前 `uav_bringup` 这一层已经把 Orbbec 的 `enable_laser` / `enable_ldp` 透传出来，并且默认都是 `false`。

也就是说：

- 默认关闭红外点阵 / 主动投射
- 默认按“更适合 VIO / 标定”的方式起相机
- 如果你后面想做 A/B 对比，再手动覆盖打开

这份手册还遵守你的环境限制：

- **宿主机不编译**
- **所有 build / run 都放在 Docker 内完成**

---

## 0. 这套流程最终要达到什么结果

你要完成的不是“把程序跑起来”这么简单，而是下面这条闭环：

1. 相机正常出 **双 IR + IMU**
2. 自动生成一份 OpenVINS **粗初值配置**
3. 用手持慢走方式做 **在线标定**
4. 把本轮结果 **回写到 YAML**
5. 重复 2~3 轮直到参数稳定
6. 冻结一份 **飞行配置**
7. 再让 OpenVINS 输出通过桥接节点送给 PX4

注意：这一版 OpenVINS **不使用 depth / RGB**。

也就是说：

- OpenVINS 只吃 `left_ir`
- OpenVINS 只吃 `right_ir`
- OpenVINS 只吃 `gyro_accel/sample`
- 深度图和 RGB 可以以后给避障 / 建图用，但不参与当前 VIO 标定

---

## 1. 你现在要用到哪些文件

当前仓库里和这套流程直接相关的文件是：

- `uav_hw/src/workspace/doco_uav/src/uav_bringup/launch/orbbec_depth_camera.launch.py`
- `uav_hw/src/workspace/doco_uav/src/uav_bringup/launch/openvins_orbbec.launch.py`
- `uav_hw/src/workspace/doco_uav/src/uav_bringup/launch/openvins_orbbec_calibration.launch.py`
- `uav_hw/src/workspace/doco_uav/src/uav_bringup/scripts/generate_orbbec_openvins_bootstrap.py`
- `uav_hw/src/workspace/doco_uav/src/uav_bringup/scripts/apply_openvins_calibration_result.py`
- `uav_hw/src/workspace/doco_uav/src/uav_bridge/src/openvins_px4_vision_bridge_node.cpp`

后面每一步其实就是围绕这几个入口在操作。

---

## 2. 开始前先约定工作目录

以下命令默认你在宿主机终端里操作，仓库根目录是：

```bash
cd /home/th1rt3en/doco
```

后面所有 Docker 命令都从这个目录执行。

---

## 3. Step 1：先在 Docker 内编译一次

### 3.1 这一步干什么

这一步是为了让下面这些 launch / script 都能在容器里直接使用：

- `uav_bringup`
- `uav_bridge`

你只要代码有改动，或者第一次跑这套流程，就先做一次。

### 3.2 执行什么

```bash
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  cd /ws &&
  source /opt/ros/humble/setup.bash &&
  colcon build \
    --packages-select uav_bringup uav_bridge \
    --base-paths /ws/src /ws/doco_common/src /ws/doco_uav/src \
    --build-base /ws/build \
    --install-base /ws/install \
    --symlink-install \
    --event-handlers console_direct+'
```

### 3.3 做完以后看什么

成功标准：

- `colcon build` 正常结束
- 没有 `Failed` 包
- 容器里存在 `/ws/install/setup.bash`

如果失败：

- 先不要继续标定流程
- 先把 build 错误修掉

---

## 4. Step 2：先单独拉起 Orbbec，相机数据必须先通

### 4.1 这一步干什么

这一步只验证硬件输入是不是正常：

- 左 IR 有没有图
- 右 IR 有没有图
- IMU 有没有数据
- Orbbec 工厂外参话题有没有发布

如果这一步都不稳定，后面的 OpenVINS、标定、PX4 都不用看。

### 4.2 执行什么

在第一个终端里执行：

```bash
cd /home/th1rt3en/doco
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  cd /ws &&
  source /opt/ros/humble/setup.bash &&
  source /ws/install/setup.bash &&
  ros2 launch uav_bringup orbbec_depth_camera.launch.py \
    camera_name:=uav_depth_camera \
    enable_left_ir:=true \
    enable_right_ir:=true \
    enable_accel:=true \
    enable_gyro:=true \
    enable_sync_output_accel_gyro:=true \
    enable_publish_extrinsic:=true \
    enable_depth:=false \
    enable_color:=false'
```

### 4.3 这一步成功以后怎么检查

在第二个终端里执行：

```bash
cd /home/th1rt3en/doco
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  source /opt/ros/humble/setup.bash &&
  source /ws/install/setup.bash &&
  timeout 3 ros2 topic hz /uav_depth_camera/left_ir/image_raw &&
  timeout 3 ros2 topic hz /uav_depth_camera/right_ir/image_raw &&
  timeout 3 ros2 topic hz /uav_depth_camera/gyro_accel/sample'
```

你还可以额外检查 topic 名字是否真的存在：

```bash
cd /home/th1rt3en/doco
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  source /opt/ros/humble/setup.bash &&
  source /ws/install/setup.bash &&
  ros2 topic list | grep uav_depth_camera'
```

### 4.4 这一步什么现象才算正常

正常现象：

- `/uav_depth_camera/left_ir/image_raw` 有频率
- `/uav_depth_camera/right_ir/image_raw` 有频率
- `/uav_depth_camera/gyro_accel/sample` 有频率
- 频率不要忽高忽低到离谱

如果不正常：

- 先检查 USB 和供电
- 再检查 `camera_name` 是否写对
- 再检查 Orbbec launch 里各个 `enable_*` 选项
- 如果连 topic 都没有，先别继续后面的步骤

---

## 5. Step 3：生成 OpenVINS 的粗初值配置（Bootstrap）

### 5.1 这一步干什么

这一步是把 Orbbec 当前能提供的工厂信息自动整理成 OpenVINS 可吃的初始配置。

生成的不是最终标定结果，而是：

- 一个**足够靠谱的起点**
- 用于后续在线细化的配置基线

### 5.2 执行什么

确保 Step 2 的相机 launch 还在运行，然后在新终端执行：

```bash
cd /home/th1rt3en/doco
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  cd /ws &&
  source /opt/ros/humble/setup.bash &&
  source /ws/install/setup.bash &&
  ros2 run uav_bringup generate_orbbec_openvins_bootstrap.py \
    --camera-name uav_depth_camera \
    --output-dir /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap'
```

### 5.3 做完以后看什么

生成目录应该存在：

```bash
ls /home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap
```

应该至少能看到：

- `estimator_config.calibration.yaml`
- `estimator_config.flight.yaml`
- `kalibr_imucam_chain.yaml`
- `kalibr_imu_chain.yaml`
- `bootstrap_summary.md`

### 5.4 成功标准

成功标准：

- `bootstrap` 目录生成出来
- 上面 5 个文件齐全
- `bootstrap_summary.md` 里没有明显离谱的警告

如果失败：

- 通常是相机话题没起来
- 或者 `enable_publish_extrinsic` 没打开
- 或者 `camera_name` 写错了

---

## 6. Step 4：启动 OpenVINS 标定模式（第 1 轮）

### 6.1 这一步干什么

这一步启动的是**专用标定模式**，不是正常飞行模式。

它会自动做这些事：

- 开 OpenVINS
- 订阅左右 IR 和相机 IMU
- 开启 `save_total_state`
- 保存在线标定状态到文件
- 发布 calibration TF
- 关闭发送到 PX4 的外部视觉输出

也就是说：

- 这一步是为了收敛标定参数
- 不是为了让 PX4 融合

### 6.2 执行什么

先把 Step 2 启动的纯相机 launch 停掉，避免重复启动相机。

然后执行：

```bash
cd /home/th1rt3en/doco
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  cd /ws &&
  source /opt/ros/humble/setup.bash &&
  source /ws/install/setup.bash &&
  ros2 launch uav_bringup openvins_orbbec_calibration.launch.py \
    camera_name:=uav_depth_camera \
    openvins_config_path:=/repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/estimator_config.calibration.yaml \
    state_output_dir:=/repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/round_01'
```

### 6.3 起起来之后先不要动，先检查

新开一个终端执行：

```bash
cd /home/th1rt3en/doco
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  source /opt/ros/humble/setup.bash &&
  source /ws/install/setup.bash &&
  timeout 3 ros2 topic hz /uav_depth_camera/left_ir/image_raw &&
  timeout 3 ros2 topic hz /uav_depth_camera/right_ir/image_raw &&
  timeout 3 ros2 topic hz /uav_depth_camera/gyro_accel/sample &&
  ros2 topic echo --once /ov_msckf/odomimu'
```

### 6.4 这一步正常现象

正常现象：

- 左右 IR 继续有图
- IMU 继续有数据
- `/ov_msckf/odomimu` 能出消息
- 不应该往 PX4 发视觉里程计

如果 `/ov_msckf/odomimu` 根本没有：

- 先检查 `openvins_config_path` 是否存在
- 再检查 bootstrap 文件是否完整
- 再检查左右 IR 和 IMU 话题是否一致

---

## 7. Step 5：开始手持慢走标定（第 1 轮）

### 7.1 这一步干什么

这一步是给 OpenVINS 足够的三维激励，让它在线细化：

- 相机-IMU 外参
- 相机时间偏移
- 必要时也包括部分内参

### 7.2 你手上应该怎么动

建议总时长：**2 到 4 分钟**。

按这个顺序做：

1. 静止 10~15 秒
2. 缓慢前后平移
3. 缓慢左右平移
4. 缓慢上下移动
5. 缓慢 yaw 旋转
6. 补一点小幅 pitch / roll 摆动
7. 最后再静止 5~10 秒

### 7.3 做这一步时要注意什么

必须注意：

- 不要暴力甩动
- 不要一直只做单轴运动
- 不要让一只 IR 经常被手挡住
- 尽量选纹理丰富场景
- 避免强反光和频闪灯

### 7.4 什么时候应该立即停下来

如果出现下面现象，就不要继续“硬跑”：

- `/ov_msckf/odomimu` 明显发散
- 姿态狂跳
- 位置瞬间飞走
- 左右图像频繁断流

这说明当前这一轮数据质量差，继续跑意义不大。

---

## 8. Step 6：停止第 1 轮，并把结果回写到 YAML

### 8.1 这一步干什么

这一步是把 OpenVINS 在线跑出来的最后一轮标定结果保存下来，并写回 `kalibr_imucam_chain.yaml`。

也就是把“这一轮学到的东西”变成下一轮的初值。

### 8.2 执行什么

先回到跑 `openvins_orbbec_calibration.launch.py` 的终端，按 `Ctrl-C` 停掉。

然后执行：

```bash
cd /home/th1rt3en/doco
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  python3 /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/scripts/apply_openvins_calibration_result.py \
    --state-estimate-path /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/round_01/ov_estimate.txt \
    --input-imucam-yaml /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/kalibr_imucam_chain.yaml'
```

### 8.3 这一步会更新哪些内容

回写脚本会更新：

- `cam0.T_cam_imu`
- `cam1.T_cam_imu`
- `cam1.T_cn_cnm1`
- `timeshift_cam_imu`

默认情况下：

- 它会直接覆盖输入 YAML

### 8.4 做完以后看什么

执行：

```bash
sed -n '1,120p' /home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/kalibr_imucam_chain.yaml
```

### 8.5 什么才算正常

正常现象：

- `T_cam_imu` 看起来不是特别离谱
- 左右相机的相对位姿符合你的硬件安装直觉
- `timeshift_cam_imu` 不是一个夸张的大值

如果明显离谱：

- 这一轮结果不要继续用
- 回头重做 Step 4 和 Step 5

---

## 9. Step 7：做第 2 轮和第 3 轮，让参数收敛

### 9.1 这一步干什么

第 1 轮通常只是“从能跑到不太离谱”。

真正稳定，通常要再做 1~2 轮。

### 9.2 第 2 轮怎么执行

命令和第 1 轮一样，只是输出目录换成 `round_02`：

```bash
cd /home/th1rt3en/doco
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  cd /ws &&
  source /opt/ros/humble/setup.bash &&
  source /ws/install/setup.bash &&
  ros2 launch uav_bringup openvins_orbbec_calibration.launch.py \
    camera_name:=uav_depth_camera \
    openvins_config_path:=/repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/estimator_config.calibration.yaml \
    state_output_dir:=/repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/round_02'
```

跑完以后，再执行一次回写：

```bash
cd /home/th1rt3en/doco
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  python3 /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/scripts/apply_openvins_calibration_result.py \
    --state-estimate-path /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/round_02/ov_estimate.txt \
    --input-imucam-yaml /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/kalibr_imucam_chain.yaml'
```

如果还想再稳一轮，再做 `round_03`，方法完全一样。

### 9.3 什么时候可以认为已经收敛得差不多

工程上看这几件事：

- 相邻两轮 `T_cam_imu` 变化已经不大
- `timeshift_cam_imu` 基本稳定
- 手持走一圈回来时漂移明显减小
- 静止时姿态比较稳

如果轮次越多越乱：

- 通常不是“再多跑一轮就好”
- 而是数据质量、光照、同步、安装刚度出了问题

---

## 10. Step 8：冻结一份飞行配置

### 10.1 这一步干什么

这一步是把在线标定收敛后的结果整理成一份**准备上机联调**的配置。

注意：

- `bootstrap` 目录是工作区
- `frozen_final` 目录是候选飞行区

### 10.2 执行什么

```bash
mkdir -p /home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final

cp /home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/estimator_config.flight.yaml \
   /home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final/estimator_config.flight.yaml

cp /home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/kalibr_imucam_chain.yaml \
   /home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final/kalibr_imucam_chain.yaml

cp /home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/kalibr_imu_chain.yaml \
   /home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final/kalibr_imu_chain.yaml
```

### 10.3 做完以后看什么

执行：

```bash
ls /home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final
```

应该有：

- `estimator_config.flight.yaml`
- `kalibr_imucam_chain.yaml`
- `kalibr_imu_chain.yaml`

---

## 11. Step 9：启动正常 OpenVINS 链路，并准备给 PX4

### 11.1 这一步干什么

这一步已经不是标定模式，而是正常运行模式：

- OpenVINS 用冻结后的 flight 配置运行
- OpenVINS 输出由桥接节点送给 PX4

### 11.2 执行什么

```bash
cd /home/th1rt3en/doco
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  cd /ws &&
  source /opt/ros/humble/setup.bash &&
  source /ws/install/setup.bash &&
  ros2 launch uav_bringup openvins_orbbec.launch.py \
    camera_name:=uav_depth_camera \
    openvins_config_path:=/repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final/estimator_config.flight.yaml'
```

### 11.3 这一步做完以后看什么

新开终端执行：

```bash
cd /home/th1rt3en/doco
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  source /opt/ros/humble/setup.bash &&
  source /ws/install/setup.bash &&
  timeout 3 ros2 topic hz /ov_msckf/odomimu &&
  timeout 3 ros2 topic hz /fmu/in/vehicle_visual_odometry'
```

### 11.4 正常现象

正常现象：

- `/ov_msckf/odomimu` 持续发布
- `/fmu/in/vehicle_visual_odometry` 持续发布

如果前者有、后者没有：

- 看桥接节点是否起来
- 看 `publish_px4_external_vision` 是否被错误关掉

---

## 12. Step 10：确认 PX4 真的在融合外部视觉

### 12.1 这一步干什么

这一步不是看“桥有没有发”，而是看“PX4 有没有真的接收并融合”。

### 12.2 执行什么

```bash
cd /home/th1rt3en/doco
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  source /opt/ros/humble/setup.bash &&
  source /ws/install/setup.bash &&
  timeout 3 ros2 topic hz /fmu/out/vehicle_odometry &&
  timeout 3 ros2 topic hz /uav/state/odometry'
```

如果想看一次消息内容：

```bash
cd /home/th1rt3en/doco
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  source /opt/ros/humble/setup.bash &&
  source /ws/install/setup.bash &&
  ros2 topic echo --once /fmu/in/vehicle_visual_odometry &&
  ros2 topic echo --once /fmu/out/vehicle_odometry'
```

### 12.3 这一阶段你还要做什么

PX4 侧要确认参数已经配对，例如：

- `EKF2_EV_CTRL`
- `EKF2_EV_DELAY`
- `EKF2_EV_POS_X`
- `EKF2_EV_POS_Y`
- `EKF2_EV_POS_Z`

如果这些没配，桥发得再好，PX4 也可能不融合。

---

## 13. Step 11：最后做无桨地面联调，不直接起飞

### 13.1 这一步干什么

这一步是飞行前的安全验证。

重点是看：

- 坐标轴有没有反
- yaw 有没有反号
- 静止时漂不漂
- 小范围搬动时位置跟不跟

### 13.2 你要怎么做

建议流程：

1. 先拆桨
2. 上电但不直接起飞
3. 观察 `/uav/state/odometry`
4. 手动小范围搬动机体
5. 看姿态和位置是否连续
6. 看 QGC / PX4 estimator 状态是否正常

### 13.3 什么现象是危险信号

如果出现下面现象，不要进下一步：

- 机体静止但位置快速飘
- 前后移动却显示左右移动
- yaw 转动方向反了
- roll / pitch 动一下就发散

这说明你的外参或机体系对齐还没处理好。

---

## 14. 最常见的失败点

### 14.1 OpenVINS 根本没输出

优先检查：

- 左右 IR topic 名是否对
- IMU topic 名是否对
- bootstrap 配置是否生成成功
- `openvins_config_path` 是否指向对的文件

### 14.2 第 1 轮能跑，第 2 轮反而更差

优先检查：

- 手持动作是不是太猛
- 场景纹理是否太少
- IR 自动曝光是否不稳
- 有无遮挡一只 IR

### 14.3 OpenVINS 很稳，但 PX4 不融合

优先检查：

- `/fmu/in/vehicle_visual_odometry` 有没有消息
- PX4 EV 参数是否正确
- 机体系安装偏移是否和 PX4 参数一致

### 14.4 一切都在发，但姿态方向不对

优先检查：

- `sensor_roll_in_body_rad`
- `sensor_pitch_in_body_rad`
- `sensor_yaw_in_body_rad`

这几个参数如果设错，坐标系方向会错得很明显。

---

## 15. 一页命令速查

如果你已经理解整套流程，以后基本只会重复这几条。

### 15.1 build

```bash
cd /home/th1rt3en/doco
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  cd /ws &&
  source /opt/ros/humble/setup.bash &&
  colcon build \
    --packages-select uav_bringup uav_bridge \
    --base-paths /ws/src /ws/doco_common/src /ws/doco_uav/src \
    --build-base /ws/build \
    --install-base /ws/install \
    --symlink-install \
    --event-handlers console_direct+'
```

### 15.2 起相机

```bash
cd /home/th1rt3en/doco
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  cd /ws &&
  source /opt/ros/humble/setup.bash &&
  source /ws/install/setup.bash &&
  ros2 launch uav_bringup orbbec_depth_camera.launch.py \
    camera_name:=uav_depth_camera \
    enable_left_ir:=true \
    enable_right_ir:=true \
    enable_accel:=true \
    enable_gyro:=true \
    enable_sync_output_accel_gyro:=true \
    enable_publish_extrinsic:=true \
    enable_depth:=false \
    enable_color:=false'
```

### 15.3 生成 bootstrap

```bash
cd /home/th1rt3en/doco
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  cd /ws &&
  source /opt/ros/humble/setup.bash &&
  source /ws/install/setup.bash &&
  ros2 run uav_bringup generate_orbbec_openvins_bootstrap.py \
    --camera-name uav_depth_camera \
    --output-dir /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap'
```

### 15.4 起第 1 轮标定

```bash
cd /home/th1rt3en/doco
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  cd /ws &&
  source /opt/ros/humble/setup.bash &&
  source /ws/install/setup.bash &&
  ros2 launch uav_bringup openvins_orbbec_calibration.launch.py \
    camera_name:=uav_depth_camera \
    openvins_config_path:=/repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/estimator_config.calibration.yaml \
    state_output_dir:=/repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/round_01'
```

### 15.5 回写第 1 轮

```bash
cd /home/th1rt3en/doco
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  python3 /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/scripts/apply_openvins_calibration_result.py \
    --state-estimate-path /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/round_01/ov_estimate.txt \
    --input-imucam-yaml /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/kalibr_imucam_chain.yaml'
```

### 15.6 起冻结后的 flight 配置

```bash
cd /home/th1rt3en/doco
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  cd /ws &&
  source /opt/ros/humble/setup.bash &&
  source /ws/install/setup.bash &&
  ros2 launch uav_bringup openvins_orbbec.launch.py \
    camera_name:=uav_depth_camera \
    openvins_config_path:=/repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/frozen_final/estimator_config.flight.yaml'
```

---

## 16. 推荐你真实执行时的顺序

如果你现在就要开干，建议你按这个顺序走：

1. 先做 Step 1：Docker 内 build
2. 再做 Step 2：只起相机，看双 IR + IMU
3. 再做 Step 3：生成 bootstrap
4. 再做 Step 4：起第 1 轮标定
5. 再做 Step 5：手持慢走 2~4 分钟
6. 再做 Step 6：停掉并回写 YAML
7. 再做 Step 7：至少再来一轮
8. 再做 Step 8：冻结一份 flight 配置
9. 再做 Step 9：起正常 OpenVINS 链路
10. 再做 Step 10：确认 PX4 融合
11. 最后做 Step 11：无桨地面联调

如果你想省时间，**不要跳过第 2 轮标定**。

第 1 轮通常只是从“能跑”变成“不太离谱”，第 2 轮才更像能上机联调的版本。
