# OpenVINS 接入规划 README

> 适用目录：`/home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/src`
>
> 编写时间：2026-03-17
>
> 目标：在当前 `uav_bringup + uav_bridge + PX4` 硬件链路上，分阶段接入 OpenVINS，并最终把 VIO 结果稳定送入 PX4 外部视觉融合链路。

---

## 0. 重要修正：本规划现在以 Orbbec 深度相机 为准

> 如果本节和下文其它章节有冲突，以本节为准。

你刚补充的约束非常关键：**OpenVINS 这次不是接普通 USB 单目，而是接当前的 Orbbec 深度相机（有 depth / RGB，且当前驱动还支持 left IR / right IR / accel / gyro）**。

这会直接改变推荐方案：

### 0.1 先说结论

- **OpenVINS 不直接吃 depth image 做主输入**
- OpenVINS 主输入仍然是：
  - `sensor_msgs/msg/Image`
  - `sensor_msgs/msg/Imu`
- 所以对于 Orbbec，真正适合 OpenVINS 的输入组合是：
  1. **首选：双目 IR + 相机自带 IMU**
  2. 备选：RGB 单目 + 相机自带 IMU
  3. 最后兜底：RGB 或 IR + PX4 IMU
- **深度图保留给其它模块**，例如：
  - 避障
  - 降落
  - 点云
  - VIO 结果交叉验证
  - 后续做 RGB-D / TSDF / mapping

### 0.2 为什么不推荐“depth + RGB 直接喂 OpenVINS”

因为当前工作区里的 OpenVINS ROS2 订阅入口只直接订阅：

- `topic_imu`
- `topic_camera0`
- `topic_camera1`

也就是 **IMU + 单目/同步双目图像**，而不是 RGB-D 专用接口。

### 0.3 结合当前 Orbbec 驱动，推荐优先级

当前 Orbbec 330 系列 launch 已经支持：

- `enable_left_ir`
- `enable_right_ir`
- `enable_sync_output_accel_gyro`
- `enable_accel`
- `enable_gyro`

因此推荐优先级改成：

#### 方案 A：`left_ir + right_ir + gyro_accel/sample`（最推荐）

这是最适合 OpenVINS 的 Orbbec 接法，因为：

- 左右 IR 天然更接近“视觉前端稳定输入”
- 与深度模组几何关系更直接
- 使用相机自带 IMU，可以减少“相机-PX4 IMU 外参 + 时间同步”的工程负担

建议 OpenVINS 订阅：

- `/uav_depth_camera/left_ir/image_raw`
- `/uav_depth_camera/right_ir/image_raw`
- `/uav_depth_camera/gyro_accel/sample`

#### 方案 B：`color/image_raw + gyro_accel/sample`

如果当前机型的 IR 画质、曝光、同步性不理想，或者你更想先低成本跑通，可以先用：

- `/uav_depth_camera/color/image_raw`
- `/uav_depth_camera/gyro_accel/sample`

这对应 **单目 RGB + 相机 IMU**。

#### 方案 C：`Orbbec 图像 + PX4 IMU`

只有在以下情况才建议：

- 相机 IMU 不可用
- 相机 IMU 噪声太差
- 或你明确希望状态估计与飞控使用同一套 IMU

但这条路要额外解决：

- 相机到 PX4 IMU 外参
- 相机到 PX4 IMU 时间偏移
- ROS 时间与 PX4 时间对齐

工程成本比方案 A / B 高不少。

### 0.4 对当前仓库意味着什么

这次接入重点不再是“补一个 PX4 IMU 桥就够了”，而是变成下面 4 件事：

1. **把 `orbbec_depth_camera.launch.py` 扩展成真正能给 OpenVINS 供图/供 IMU 的入口**
   - 现在它默认：`depth=true, color=false, ir=false`
   - 还没有把 `left_ir/right_ir/gyro/accel` 这些关键开关透传出来
2. **新增一个 OpenVINS 专用的 Orbbec launch**
   - 例如：`openvins_orbbec.launch.py`
3. **为 Orbbec 建一套 OpenVINS 标定配置**
   - `kalibr_imucam_chain.yaml`
   - `kalibr_imu_chain.yaml`
4. **把 OpenVINS odom 桥接到 PX4 external vision**
   - `/fmu/in/vehicle_visual_odometry`

### 0.5 当前推荐主链路（Orbbec版）

```text
Orbbec left_ir/right_ir 或 color
  -> OpenVINS
Orbbec gyro_accel/sample
  -> OpenVINS
OpenVINS odomimu
  -> openvins_px4_vision_bridge_node
  -> /fmu/in/vehicle_visual_odometry
PX4 EKF2 融合 external vision
  -> /fmu/out/vehicle_odometry
  -> uav_state_bridge_node
  -> /uav/state/odometry
uav_control_node
  <- /uav/state/odometry
```

### 0.6 因此，后文中这些旧假设现在作废

下面章节里如果出现这些默认前提，请按 Orbbec 版本理解：

- “先接普通单目相机” -> 改为 **优先接 Orbbec IR stereo 或 RGB**
- “先补 PX4 IMU bridge” -> 改为 **优先用 Orbbec `gyro_accel/sample`**
- “主要图像话题是 `/uav/camera/image_raw`” -> 改为 **优先用 `/uav_depth_camera/...` 话题**

## 1. 当前仓库基线

先把当前系统状态说清楚，这样 OpenVINS 不会“接错位置”。

### 1.1 当前硬件最小控制链路

当前主要入口是：

- `uav_bringup/launch/minimal_control.launch.py`

这个 launch 里已经做了几件关键事情：

1. 启动单目相机：
   - 图像话题：`/uav/camera/image_raw`
   - 相机信息：`/uav/camera/camera_info`
   - 相机 TF：`uav_base_link -> uav_camera_optical_frame`
2. 读取 PX4 的状态：
   - `/fmu/out/vehicle_local_position`
   - `/fmu/out/vehicle_odometry`
   - `/fmu/out/vehicle_status`
3. 通过 `uav_state_bridge_node` 生成统一状态：
   - 输出：`/uav/state/odometry`
4. 通过 `uav_control_node` 执行控制：
   - 读取：`/uav/state/odometry`
   - 写入：`/fmu/in/offboard_control_mode`
   - 写入：`/fmu/in/trajectory_setpoint`
   - 写入：`/fmu/in/vehicle_command`

### 1.2 当前系统里与 OpenVINS 最相关的“已具备条件”

已经具备：

- 单目图像 ROS 话题已经有了：`/uav/camera/image_raw`
- 相机静态 TF 已经有了：`uav_base_link -> uav_camera_optical_frame`
- PX4 ROS 2 话题链路已经通了：`/fmu/in/*`、`/fmu/out/*`
- 控制链已经统一依赖 `/uav/state/odometry`

### 1.3 当前系统里与 OpenVINS 最相关的“缺口”

还缺：

1. **OpenVINS 需要的 IMU ROS 话题**
   - OpenVINS 订阅的是 `sensor_msgs/msg/Imu`
   - 当前仓库里没有现成的 `/uav/sensors/imu`
   - 当前 `fmu_topic_namespace_bridge_node` 也没有转发 `/fmu/out/sensor_combined`

2. **OpenVINS 到 PX4 的外部视觉桥接**
   - PX4 需要的是：`/fmu/in/vehicle_visual_odometry`
   - 消息类型：`px4_msgs/msg/VehicleOdometry`
   - 当前仓库里没有“OpenVINS odom -> PX4 vehicle_visual_odometry”桥接节点

3. **OpenVINS 的相机-IMU 联合标定配置**
   - OpenVINS 不直接依赖 `/uav/camera/camera_info`
   - 它主要依赖自己的 YAML 配置：
     - `estimator_config.yaml`
     - `kalibr_imucam_chain.yaml`
     - `kalibr_imu_chain.yaml`

4. **基于 OpenVINS 的集成 launch**
   - 当前只有相机、状态桥、控制链 launch
   - 还没有 `hw_openvins.launch.py` 这样的组合入口

---

## 2. 推荐接法：不要直接绕过 PX4，优先走“OpenVINS -> PX4 外部视觉 -> PX4 融合状态”

这是我推荐的主方案。

### 2.1 推荐架构

```text
单目相机
  -> /uav/camera/image_raw

PX4 IMU (/fmu/out/sensor_combined)
  -> px4_sensor_combined_to_imu_node
  -> /uav/sensors/imu

/uav/camera/image_raw + /uav/sensors/imu
  -> OpenVINS
  -> /ov_msckf/odomimu

/ov_msckf/odomimu
  -> openvins_px4_vision_bridge_node
  -> /fmu/in/vehicle_visual_odometry

PX4 EKF2 融合 external vision
  -> /fmu/out/vehicle_odometry
  -> uav_state_bridge_node
  -> /uav/state/odometry

uav_control_node
  <- /uav/state/odometry
  -> /fmu/in/trajectory_setpoint 等控制话题
```

### 2.2 为什么推荐这条链路

原因很简单：

- 当前控制器 `uav_control_node` 的主输入已经是 `/uav/state/odometry`
- 当前 `/uav/state/odometry` 的来源是 `uav_state_bridge_node`
- `uav_state_bridge_node` 的上游就是 PX4 输出状态

所以如果你直接让控制器吃 OpenVINS 原始输出，而 PX4 仍然用它自己的状态估计，就会出现：

- **控制器的世界观** 和 **飞控的世界观** 不一致
- PX4 内部位置估计与外部控制目标存在偏移
- 起飞、悬停、降落时更容易出现坐标系不一致问题

因此推荐做法是：

- 先把 OpenVINS 作为 **PX4 的外部视觉来源**
- 再让 PX4 融合后输出统一状态
- 最后由现有控制链继续使用 `/uav/state/odometry`

这能最大程度复用当前仓库已经稳定的控制结构。

### 2.3 不推荐但可用于调试的旁路

仅调试时可以临时走：

```text
OpenVINS -> 适配器 -> /uav/state/odometry
```

但这条路只适合做算法联调，不适合最终飞行闭环，因为 PX4 和上层控制可能会“看见不同的状态”。

---

## 3. OpenVINS 接入前必须先做的决策

在开工前，建议先把这 4 件事拍板。

### 3.1 决策一：IMU 来源

推荐优先级：

1. **优先：直接使用 PX4 导出的 `/fmu/out/sensor_combined`**
   - 优点：和飞控内部使用的是同一套 IMU 数据
   - 优点：后续做 PX4 external vision 融合时更一致
   - 缺点：需要一个 `SensorCombined -> sensor_msgs/Imu` 桥接节点

2. 外部独立 IMU 驱动
   - 只有在 PX4 IMU 无法稳定导出或时序不理想时再考虑

结论：**本仓库优先做 PX4 IMU 桥接。**

### 3.2 决策二：先单目还是直接双目

当前 `minimal_control.launch.py` 已经稳定提供的是 **单目相机**：

- `/uav/camera/image_raw`

所以建议第一阶段先做：

- **Mono + IMU OpenVINS**

不要一开始就把 Orbbec 深度/双目链路也卷进来，否则问题会混在一起。

### 3.3 决策三：OpenVINS 的状态是“调试状态”还是“主闭环状态”

建议分两步：

- 第一步：OpenVINS 仅用于输出调试 odometry
- 第二步：OpenVINS 输出送入 `/fmu/in/vehicle_visual_odometry`
- 第三步：验证 PX4 融合结果后，再让控制系统继续使用 `/uav/state/odometry`

### 3.4 决策四：相机帧率/分辨率先保守还是一步拉满

当前默认相机参数偏激进：

- `1280x720 @ 120 Hz`

对 OpenVINS 初次接入来说，不建议一开始就这么跑。更推荐先降低到：

- `camera_fps:=30.0`
- 先关闭录像：`record_mono_video:=false`

等整个 VIO 链路稳定之后，再逐步拉高分辨率和帧率。

---

## 4. 实施总规划（推荐分 6 个阶段）

下面是我建议的落地顺序。不要跳阶段，不然很容易在“时序、标定、坐标系、飞控参数”几个问题里互相缠住。

### 阶段 0：确认当前硬件基线稳定

**目标**：确保不是现有相机/PX4 链路本身有问题。

#### 要做什么

1. 先用当前 launch 单独跑通：

```bash
ros2 launch uav_bringup minimal_control.launch.py \
  camera_fps:=30.0 \
  record_mono_video:=false
```

2. 验证这些话题是稳定的：

- `/uav/camera/image_raw`
- `/uav/camera/camera_info`
- `/fmu/out/vehicle_local_position`
- `/fmu/out/vehicle_odometry`
- `/uav/state/odometry`

3. 记录基线数据：

- 相机实际帧率
- `/uav/state/odometry` 是否持续输出
- 控制服务是否还能正常起飞/悬停/降落

#### 验收标准

- 图像持续无掉流
- `/uav/state/odometry` 稳定
- 当前最小控制链可以保持原有功能

---

### 阶段 1：补齐 IMU ROS 话题

**目标**：给 OpenVINS 提供标准 ROS IMU 输入。

#### 计划新增内容

建议新增一个节点，例如：

- `uav_bridge/src/px4_sensor_combined_to_imu_node.cpp`

它负责：

- 订阅：`/fmu/out/sensor_combined`
- 发布：`/uav/sensors/imu`
- 消息类型：`sensor_msgs/msg/Imu`

#### 这个桥接节点至少要做的事情

1. 时间戳填充
2. 角速度/线加速度字段填充
3. 坐标系统一
4. `frame_id` 固定，例如：`uav_imu_link`
5. 若当前系统统一使用 FLU，则推荐把 PX4 的 FRD 数据转换成 FLU 后再发布

#### 为什么推荐统一到 FLU

因为当前仓库的上层语义大多偏 ROS 常见风格：

- 机体系：`uav_base_link`
- 世界系：上层一般按 ENU 语义理解

如果 IMU 还保持 PX4 FRD，而相机/控制/TF 又按 FLU 讲，会很容易把标定和调试搞乱。

#### 同阶段要补的 bridge 转发

如果后续 `fmu_namespace` 不是默认 `/fmu`，那么还要扩展：

- `uav_bridge/src/fmu_topic_namespace_bridge_node.cpp`

至少补这两个 topic：

- `/out/sensor_combined`
- `/in/vehicle_visual_odometry`

#### 验收标准

- `ros2 topic hz /uav/sensors/imu` 有稳定输出
- IMU 频率足够高，建议至少 `>100 Hz`
- IMU 方向约定写清楚，后续标定不再变

---

### 阶段 2：准备 OpenVINS 配置与标定文件

**目标**：把 OpenVINS 需要的配置放进当前仓库，避免“配置漂移”。

#### 计划新增目录

建议新增：

- `uav_bringup/config/openvins/mono/estimator_config.yaml`
- `uav_bringup/config/openvins/mono/kalibr_imucam_chain.yaml`
- `uav_bringup/config/openvins/mono/kalibr_imu_chain.yaml`

#### 这里要特别注意

OpenVINS **不会直接吃** 当前系统的：

- `/uav/camera/camera_info`

所以：

- 当前 `camera_info_url` 对 OpenVINS 不是主配置入口
- OpenVINS 的主入口是它自己的 YAML

也就是说，你要么：

1. 用同一次标定同时生成：
   - ROS 相机标定文件
   - OpenVINS/Kalibr 文件

要么：

2. 以 Kalibr 文件为准，再把当前相机 `camera_info_url` 同步更新

#### 第一版配置建议

第一版先按“单目 + IMU”保守配置：

- `use_stereo: false`
- `max_cameras: 1`
- 追踪频率先按 `20~30 Hz`
- 初期先不开太多在线标定项

#### 标定建议

最关键的是：

- **相机到 IMU 的外参**
- **相机到 IMU 的时间偏移**

如果这两个没有校准好，OpenVINS 很容易“能跑，但不稳”。

#### 验收标准

- 配置文件已固定在仓库中
- 明确哪一套标定文件对应哪台硬件
- 相机/IMU 外参与时间偏移有可追溯来源

---

### 阶段 3：先让 OpenVINS 独立跑起来

**目标**：先验证 OpenVINS 本身能在你这套话题上出稳定 odometry。

#### 建议新增 launch

建议新增：

- `uav_bringup/launch/hw_openvins.launch.py`

它的职责是：

1. 启动 IMU bridge
2. 启动 OpenVINS
3. 统一传入：
   - `topic_imu`
   - `topic_camera0`
   - `config_path`
   - `namespace`

#### 建议的 OpenVINS 命名空间

建议固定成：

- `/ov_msckf`

这样第一版输出默认就是：

- `/ov_msckf/odomimu`

#### 这一阶段只看 3 件事

1. 能否成功初始化
2. `odomimu` 是否连续输出
3. 轨迹是否大体合理

#### 这一阶段不要急着做的事

不要一上来就：

- 接 PX4 external vision
- 开自动起飞
- 做闭环飞行

先单独验证 OpenVINS 自己是否健康。

#### 验收标准

- `/ov_msckf/odomimu` 有稳定输出
- 初始化不需要极端手动干预
- 静止时漂移可接受
- 小范围移动时轨迹方向正确

---

### 阶段 4：把 OpenVINS 输出桥接到 PX4 external vision

**目标**：让 PX4 开始接收 OpenVINS 视觉里程计。

#### 计划新增节点

建议新增：

- `uav_bridge/src/openvins_px4_vision_bridge_node.cpp`

建议它做这些事情：

1. 订阅：`/ov_msckf/odomimu`
2. 发布：`/fmu/in/vehicle_visual_odometry`
3. 类型转换：
   - `nav_msgs/msg/Odometry`
   - -> `px4_msgs/msg/VehicleOdometry`
4. 坐标系转换：
   - OpenVINS/ROS 侧 -> ENU / FLU 语义
   - PX4 侧 -> NED / FRD
5. 协方差映射：
   - 从 ROS covariance 中取对角线
   - 填入 `position_variance`
   - 填入 `orientation_variance`
   - 填入 `velocity_variance`
6. 时间戳策略：
   - 第一版先和现有 PX4 控制节点保持一致的时间戳策略
   - 若 PX4 侧判定外部视觉数据过期，再补 `timesync_status` 校时逻辑

#### 这里最关键的 3 个工程点

##### A. 坐标系

PX4 的 `VehicleOdometry` 语义是：

- 位置姿态：通常按 `POSE_FRAME_NED`
- 速度：通常按 `VELOCITY_FRAME_NED`
- 机体系：FRD

而 OpenVINS 输出 `odomimu` 时：

- `header.frame_id = global`
- `child_frame_id = imu`

所以桥接节点不能只“改 topic 名”，必须做明确数值转换。

##### B. body / imu 的区别

OpenVINS 输出的是 **IMU 位姿**。

但 PX4 真正关心的是飞行器机体估计。若 `imu` 和 `uav_base_link` 不完全重合，就要处理：

- `T_body_imu`

第一版如果 IMU 与机体原点/朝向差异很小，可以先近似忽略平移；但**朝向约定必须明确**。

##### C. 协方差

PX4 外部视觉融合对协方差非常敏感。第一版不要全填 0，也不要乱填一个超大数。

建议做法：

- 先使用 OpenVINS odom 的 pose/twist covariance 对角线
- 如果融合效果差，再单独调尺度

#### 验收标准

- `/fmu/in/vehicle_visual_odometry` 有持续输出
- 话题频率稳定，建议至少 `20~30 Hz`
- PX4 没有明显拒收/超时现象

---

### 阶段 5：PX4 参数与融合验证

**目标**：确认 PX4 真的在融合 OpenVINS，而不是“你发了但它没用”。

#### 这一阶段的核心工作

1. 配置 PX4 EKF2 external vision 相关参数
2. 在地面静止和小范围手持移动下验证
3. 看 PX4 输出状态是否明显受 OpenVINS 影响

#### 这阶段要重点看哪些现象

- `vehicle_odometry` 是否更稳定
- 起飞前静止时估计是否收敛
- 小范围平移/转动时输出是否方向正确
- 估计是否有明显跳变、延迟、镜像、轴交换

#### 建议的观测对象

- `/ov_msckf/odomimu`
- `/fmu/in/vehicle_visual_odometry`
- `/fmu/out/vehicle_odometry`
- `/uav/state/odometry`

#### 验收标准

- PX4 输出状态与 OpenVINS 的运动趋势一致
- 没有明显轴反、符号反、yaw 反向的问题
- `/uav/state/odometry` 仍然正常供控制链使用

---

### 阶段 6：把 OpenVINS 纳入正式 bringup

**目标**：让它成为 `minimal_control.launch.py` 的一个可开关能力，而不是散装调试脚本。

#### 建议修改入口

修改：

- `uav_bringup/launch/minimal_control.launch.py`

建议新增 launch 参数，例如：

- `start_openvins`
- `openvins_namespace`
- `openvins_config_path`
- `openvins_imu_topic`
- `openvins_camera_topic`
- `publish_px4_external_vision`

#### 推荐行为

- 默认先 `start_openvins:=false`
- 等地面验证稳定后，再默认打开

#### 这一步完成后，最终使用方式应该像这样

```bash
ros2 launch uav_bringup minimal_control.launch.py \
  camera_fps:=30.0 \
  record_mono_video:=false \
  start_openvins:=true \
  openvins_config_path:=/absolute/path/to/estimator_config.yaml
```

> 上面这条命令是**目标形态**，不是当前仓库已经具备的命令。

#### 验收标准

- 一个 launch 就能拉起“相机 + IMU bridge + OpenVINS + PX4 EV bridge + 状态桥 + 控制链”
- OpenVINS 可开关，不影响原有最小控制链回退

---

## 5. 这次接入最关键的 4 个技术点

### 5.1 OpenVINS 不直接使用 `camera_info`

这是最容易误判的一点。

当前仓库里有：

- `/uav/camera/camera_info`
- `camera_info_url`

但 OpenVINS 的 ROS 2 订阅核心是：

- `topic_imu`
- `topic_camera0`
- `topic_camera1`（如果是双目）

它真正的相机/IMU内外参主要来自 YAML 配置，而不是当前这个 `CameraInfo` 话题。

### 5.2 标定优先级高于“代码先跑起来”

如果：

- 外参错一点
- 时间偏移错一点
- 坐标方向理解错一点

你会看到的现象常常不是“完全跑不起来”，而是：

- 能出轨迹，但漂移很大
- 能初始化，但一动就炸
- yaw 或某个轴方向不对

所以不要把“有输出”当成“已经接好了”。

### 5.3 OpenVINS 输出的是 IMU 状态，不一定等于机体状态

如果以后你需要：

- 让 RViz 里 `uav_base_link` 和 `ov_msckf` 输出完全一致
- 或者想直接把 OpenVINS 结果用于其他基于 `base_link` 的模块

那就必须明确：

- `uav_base_link`
- `uav_imu_link`
- `uav_camera_optical_frame`

三者的几何关系。

### 5.4 时间戳要从一开始就当成一级问题

OpenVINS 对相机/IMU 时序敏感；PX4 对 external vision 数据时效也敏感。

所以要把下面这些都当成正式工程项，而不是“之后再看”：

- 图像时间戳是否稳定
- IMU 时间戳是否稳定
- 相机与 IMU 是否有固定偏移
- 发给 PX4 的时间戳是否被接受

---

## 6. 建议改哪些文件

下面是我建议的代码落点。

### 6.1 建议新增

- `uav_bringup/launch/hw_openvins.launch.py`
- `uav_bringup/config/openvins/mono/estimator_config.yaml`
- `uav_bringup/config/openvins/mono/kalibr_imucam_chain.yaml`
- `uav_bringup/config/openvins/mono/kalibr_imu_chain.yaml`
- `uav_bridge/src/px4_sensor_combined_to_imu_node.cpp`
- `uav_bridge/src/openvins_px4_vision_bridge_node.cpp`

### 6.2 建议修改

- `uav_bringup/launch/minimal_control.launch.py`
  - 增加 `start_openvins` 等参数
  - 条件包含 `hw_openvins.launch.py`

- `uav_bridge/src/fmu_topic_namespace_bridge_node.cpp`
  - 增加 `/out/sensor_combined` relay
  - 增加 `/in/vehicle_visual_odometry` relay

- `uav_bridge/CMakeLists.txt`
  - 注册新增 bridge 节点

- `uav_bridge/package.xml`
  - 若新增依赖，保持声明同步

### 6.3 可选增强

- 增加一个调试适配器：
  - `OpenVINS odom -> /uav/state/odometry`
  - 仅用于算法联调
- 增加测试：
  - 坐标系转换单测
  - covariance 映射单测

---

## 7. 推荐的话题与坐标系约定

### 7.1 推荐话题表

| 角色 | 推荐话题 | 类型 | 说明 |
|---|---|---|---|
| Orbbec 左 IR 输入 | `/uav_depth_camera/left_ir/image_raw` | `sensor_msgs/msg/Image` | OpenVINS stereo 首选 cam0 |
| Orbbec 右 IR 输入 | `/uav_depth_camera/right_ir/image_raw` | `sensor_msgs/msg/Image` | OpenVINS stereo 首选 cam1 |
| Orbbec RGB 输入 | `/uav_depth_camera/color/image_raw` | `sensor_msgs/msg/Image` | 若先跑 mono，可作为备选 cam0 |
| Orbbec 联合 IMU 输入 | `/uav_depth_camera/gyro_accel/sample` | `sensor_msgs/msg/Imu` | 推荐优先直接给 OpenVINS |
| Orbbec 深度图 | `/uav_depth_camera/depth/image_raw` | `sensor_msgs/msg/Image` | 不直接进 OpenVINS，给避障/验证/建图 |
| OpenVINS 输出 | `/ov_msckf/odomimu` | `nav_msgs/msg/Odometry` | OpenVINS 默认 odom 输出 |
| PX4 外部视觉输入 | `/fmu/in/vehicle_visual_odometry` | `px4_msgs/msg/VehicleOdometry` | 需要新增桥接 |
| 控制主状态 | `/uav/state/odometry` | `nav_msgs/msg/Odometry` | 当前控制链已使用 |

### 7.2 推荐帧语义

| 帧名 | 语义 | 说明 |
|---|---|---|
| `uav_base_link` | 机体系，推荐 FLU | 当前控制语义核心帧 |
| `uav_camera_optical_frame` | 相机光学帧 | 当前已有 |
| `uav_imu_link` | OpenVINS 使用的 IMU 帧 | 建议新增并固定定义 |
| `uav_odom` | 控制使用的局部里程计帧 | 当前 `uav_state_bridge_node` 使用 |
| `uav_map` | 更高层 map/world 语义 | 当前状态桥可发布 map->odom |
| `global` / `imu` | OpenVINS 默认输出帧 | 作为桥接内部参考，不建议长期直接暴露给全系统做主语义 |

### 7.3 关于 PX4 坐标

PX4 内部与 `VehicleOdometry` 相关的常见语义是：

- 世界系：NED
- 机体系：FRD

当前仓库控制语义更偏：

- 世界系：ENU 风格理解
- 机体系：FLU 风格理解

因此 **OpenVINS -> PX4** 这一段一定要做显式变换，不能默认“ROS 的 odom 发过去就能用”。

---

## 8. 推荐的联调顺序

建议按下面顺序，每完成一步再往下走。

### Step A

只跑当前最小链路：

- `minimal_control.launch.py`

确认相机和控制都没问题。

### Step B

只加 IMU bridge：

- 看 `/uav/sensors/imu`

### Step C

只加 OpenVINS：

- 看 `/ov_msckf/odomimu`

### Step D

只加 `vehicle_visual_odometry` bridge：

- 看 `/fmu/in/vehicle_visual_odometry`

### Step E

检查 PX4 融合输出：

- 看 `/fmu/out/vehicle_odometry`
- 看 `/uav/state/odometry`

### Step F

最后再测试控制与飞行。

---

## 9. 验收清单（建议逐项打勾）

### 9.1 数据层

- [ ] `/uav/camera/image_raw` 频率稳定
- [ ] `/uav/sensors/imu` 频率稳定
- [ ] 图像与 IMU 时间戳没有明显跳变

### 9.2 OpenVINS 层

- [ ] `/ov_msckf/odomimu` 稳定输出
- [ ] 静止漂移在可接受范围内
- [ ] 手持平移/转动方向正确

### 9.3 PX4 融合层

- [ ] `/fmu/in/vehicle_visual_odometry` 稳定输出
- [ ] `/fmu/out/vehicle_odometry` 明显受 OpenVINS 影响
- [ ] 没有明显轴交换、镜像或 yaw 符号错误

### 9.4 系统闭环层

- [ ] `/uav/state/odometry` 仍稳定
- [ ] `uav_control_node` 不需要大改就能继续工作
- [ ] 可以随时关闭 OpenVINS，回退到原始最小链路

---

## 10. 最终建议：先做“最小可验证版本”

如果让我给一个最稳的执行顺序，我建议是：

1. **先不改控制链**
2. **先做 IMU bridge**
3. **先让 OpenVINS 单独出 `/ov_msckf/odomimu`**
4. **再做 OpenVINS -> PX4 的视觉桥接**
5. **最后才把它接入正式 bringup**

这样你每一步都能单独判断问题在哪一层：

- 是图像问题
- 是 IMU 问题
- 是标定问题
- 是 OpenVINS 参数问题
- 还是 PX4 external vision 融合问题

这是把复杂系统接起来时，成本最低、最不容易绕远路的方式。

---

## 11. 参考资料

下面这些资料是这份规划的主要依据，后续实施时建议直接对照：

- OpenVINS 仓库：<https://github.com/rpng/open_vins>
- OpenVINS ROS 2 启动入口 `subscribe.launch.py`：<https://github.com/rpng/open_vins/blob/master/ov_msckf/launch/subscribe.launch.py>
- OpenVINS ROS 2 订阅/输出实现 `ROS2Visualizer.cpp`：<https://github.com/rpng/open_vins/blob/master/ov_msckf/src/ros/ROS2Visualizer.cpp>
- OpenVINS 示例配置：<https://github.com/rpng/open_vins/tree/master/config>
- PX4 外部位置/视觉融合文档：<https://docs.px4.io/main/en/ros/external_position_estimation>
- PX4 uXRCE-DDS 主题清单（本地对应 `dds_topics.yaml`）：<https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml>

---

## 12. 下一步建议

如果按这个 README 往下做，建议下一个具体实现任务就是：

1. 先扩展 `uav_bringup/launch/orbbec_depth_camera.launch.py`
   - 透传 `enable_left_ir`
   - 透传 `enable_right_ir`
   - 透传 `enable_sync_output_accel_gyro`
   - 透传 `enable_accel`
   - 透传 `enable_gyro`
2. 再新增 `uav_bringup/launch/openvins_orbbec.launch.py`
   - 先跑 `left_ir + right_ir + gyro_accel/sample`
3. 然后做 `uav_bridge/src/openvins_px4_vision_bridge_node.cpp`

也就是说，**第一步不再是补 PX4 IMU bridge，而是先把 Orbbec 自己的图像和 IMU 正确接给 OpenVINS。**

---

## 13. 当前已落地状态（Orbbec 双 IR + 相机 IMU 版本）

截至当前版本，这条链路已经不是“规划中”，而是已经在仓库里落了主干：

### 13.1 已接通的数据链

```text
/uav_depth_camera/left_ir/image_raw
/uav_depth_camera/right_ir/image_raw
/uav_depth_camera/gyro_accel/sample
  -> /ov_msckf/odomimu
  -> openvins_px4_vision_bridge_node
  -> /fmu/in/vehicle_visual_odometry
  -> PX4 EKF2
  -> /fmu/out/vehicle_odometry
  -> /uav/state/odometry
```

### 13.2 已落地的代码入口

- `uav_bringup/launch/orbbec_depth_camera.launch.py`
  - 已透传 `enable_left_ir`、`enable_right_ir`、`enable_sync_output_accel_gyro`、`enable_accel`、`enable_gyro`
- `uav_bringup/launch/openvins_orbbec.launch.py`
  - 已能一键拉起 Orbbec + OpenVINS + `openvins_px4_vision_bridge_node`
- `uav_bringup/launch/minimal_control.launch.py`
  - 已可通过 `start_openvins_orbbec:=true` 接入现有最小控制链
- `uav_bridge/src/openvins_px4_vision_bridge_node.cpp`
  - 已把 OpenVINS `nav_msgs/msg/Odometry` 转成 PX4 `px4_msgs/msg/VehicleOdometry`
- `uav_bridge/src/fmu_topic_namespace_bridge_node.cpp`
  - 已补 `/in/vehicle_visual_odometry` relay

### 13.3 当前实现边界

当前版本有两个非常重要的边界，请按这个理解后续调参：

1. **OpenVINS 只吃双 IR + 相机 IMU，不直接吃 depth**
   - depth 先不要接进 OpenVINS 主链路
   - depth 可以留给避障、降落、建图或对比验证
2. **桥接节点当前只处理旋转外参，不处理平移外参**
   - 旋转外参：在 ROS launch 里通过 `sensor_roll_in_body_rad` / `sensor_pitch_in_body_rad` / `sensor_yaw_in_body_rad` 处理
   - 平移外参：交给 PX4 `EKF2_EV_POS_X/Y/Z`

如果以后你想把 EV 位置直接改写成机体原点位置，那时再考虑把平移补进桥接节点；当前先不要混两套补偿。

---

## 14. 怎么启动

### 14.1 只测 Orbbec + OpenVINS + PX4 EV bridge

先只拉这条链，不带控制：

```bash
ros2 launch uav_bringup openvins_orbbec.launch.py
```

如果你的相机名字或挂载姿态不同，可以直接覆盖参数，例如：

```bash
ros2 launch uav_bringup openvins_orbbec.launch.py \
  camera_name:=uav_depth_camera \
  base_frame_id:=uav_base_link \
  camera_frame_id:=uav_stereo_camera_optical_frame \
  sensor_roll_in_body_rad:=0.0 \
  sensor_pitch_in_body_rad:=0.0 \
  sensor_yaw_in_body_rad:=0.0
```

### 14.2 接到当前最小控制链

如果要把 OpenVINS 直接接到当前硬件最小控制链：

```bash
ros2 launch uav_bringup minimal_control.launch.py \
  start_orbbec_depth_camera:=true \
  start_openvins_orbbec:=true \
  publish_px4_external_vision:=true
```

如果 `fmu_namespace` 不是默认的 `/fmu`，记得一起覆盖：

```bash
ros2 launch uav_bringup minimal_control.launch.py \
  fmu_namespace:=/my_fmu \
  start_orbbec_depth_camera:=true \
  start_openvins_orbbec:=true \
  publish_px4_external_vision:=true
```

---

## 15. 怎么实测（建议按顺序）

下面这套顺序是“最少绕路版”。不要一上来就上桨飞，先做桌面静态、手持、小范围无桨测试。

### 15.1 第一步：确认 Orbbec 原始话题正常

先确认双 IR 和相机 IMU 都稳定：

```bash
ros2 topic hz /uav_depth_camera/left_ir/image_raw
ros2 topic hz /uav_depth_camera/right_ir/image_raw
ros2 topic hz /uav_depth_camera/gyro_accel/sample
```

你要重点看：

- 左右 IR 都在持续发布
- 左右 IR 频率与 launch 里的 `left_ir_fps` / `right_ir_fps` 基本一致
- `gyro_accel/sample` 频率稳定，没有明显断流
- 图像编码适合 OpenVINS，当前建议保持 `Y8`

如果你有图像查看工具，再开一个：

```bash
rqt_image_view
```

重点观察：

- 左右 IR 曝光是否稳定
- 画面是否过暗/过曝
- 有无大面积拖影
- 两路画面视场是否基本对应

### 15.2 第二步：确认 OpenVINS 已经真的在跑

检查 OpenVINS 输出：

```bash
ros2 topic hz /ov_msckf/odomimu
ros2 topic echo --once /ov_msckf/odomimu
```

此时做几个最简单动作：

1. **静止放置 10~20 秒**
   - 看位置是否只缓慢漂移
   - 看姿态是否没有突跳
2. **机体前后平移**
   - 看 odom 位置是否连续变化
3. **机体左右平移**
   - 看横向量是否跟着变
4. **缓慢 yaw 旋转**
   - 看姿态是否连续、方向是否正确

如果 OpenVINS 连 `/ov_msckf/odomimu` 都没有，就先不要看 PX4；问题一定还在图像、IMU、时间或 YAML 这一层。

### 15.3 第三步：确认桥接节点在发 PX4 EV

```bash
ros2 topic hz /fmu/in/vehicle_visual_odometry
ros2 topic echo --once /fmu/in/vehicle_visual_odometry
```

这里重点看：

- `pose_frame` 应该是 `POSE_FRAME_NED`
- `velocity_frame` 应该至少在有速度时变成 `VELOCITY_FRAME_NED`
- `q` 不应该长期是 NaN
- `position` / `velocity` 数值应该随手持运动变化

### 15.4 第四步：确认 PX4 真的在融合

桥发了，不代表 PX4 一定收了、用了。你还要看 PX4 输出侧：

```bash
ros2 topic hz /fmu/out/vehicle_odometry
ros2 topic echo --once /fmu/out/vehicle_odometry
ros2 topic hz /uav/state/odometry
```

建议同时在 QGroundControl 里看 EKF / estimator 状态，重点确认 external vision 相关 fusion flag 是否置位。

如果 PX4 参数已经配好，正常现象应该是：

- `/fmu/out/vehicle_odometry` 的趋势和 OpenVINS 基本一致
- `/uav/state/odometry` 也跟着变化
- 不会出现明显轴交换、镜像或 yaw 反号

### 15.5 第五步：无桨地面联调

在**拆桨**或确保绝对安全的前提下：

1. 保持机体静止，观察 `uav_control_node` 是否还正常工作
2. 小范围手动搬动机体，确认 `/uav/state/odometry` 跟随稳定
3. 如果要做 offboard 前的系统检查，只做上电和模式切换，不做实际起飞

### 15.6 实测时最常见的 6 个问题

1. **OpenVINS 没输出**
   - 通常是 IR/IMU 话题名不对，或 YAML 不匹配
2. **静止漂移很大**
   - 通常是相机-IMU 外参、时间偏移、噪声参数不对
3. **平移方向对，但 yaw 反了**
   - 通常是 `sensor_yaw_in_body_rad` 设错
4. **roll/pitch 一动就乱**
   - 通常是 IMU 相对机体安装姿态没配对
5. **PX4 没融合**
   - 通常是 `EKF2_EV_CTRL`、`EKF2_EV_DELAY` 或 EV offset 没配好
6. **PX4 融合后位置反而更差**
   - 通常是给 PX4 的外参、噪声或 yaw 融合配置过激进

---

## 16. Fast-Drone-250 风格的新标定工作流

这一版从这里开始，**直接替换旧版的“怎么配置 / Docker 内编译 / 自动 Bootstrap”那套尾部说明**。

目标不是照搬 `Fast-Drone-250` 的具体算法，而是保留它最有工程价值的那种节奏：

1. **先拿到粗初值**
2. **手持慢走 / 跑动做在线收敛**
3. **把本轮结果导出并回写**
4. **重复 2~3 轮直到收敛**
5. **冻结成飞行配置，再接 PX4**

但在你当前仓库里，具体实现已经换成更适合这套硬件的版本：

- 传感器输入用的是 **Orbbec 左 IR + 右 IR + 相机自带 IMU**
- VIO 主体用的是 **OpenVINS**
- 粗初值不是纯手填，而是优先用 **Orbbec 工厂信息自动生成 bootstrap**
- 在线标定结果不是靠肉眼抄 log，而是直接读 **OpenVINS 保存的状态文件** 回写

这套方案比“完全手填 + 一次性拍脑袋固定参数”更稳，也比“完全照抄别人的 VINS-Fusion 配方”更贴合你现在的 Orbbec + OpenVINS 组合。

### 16.1 这一版到底用哪些传感器

当前这套标定 / 飞行链路，**OpenVINS 只吃下面三路**：

- `left_ir`
- `right_ir`
- `gyro_accel/sample`

也就是说：

- **深度图不送 OpenVINS**
- **RGB 不送 OpenVINS**
- 深度 / RGB 可以保留给建图、避障、感知，但不参与这一版 VIO 标定闭环

这和你前面提出的目标一致：**先把两个 IR + 相机 IMU 这套 VIO 做稳**。

### 16.2 这次真正需要关注的关键配置项

另外，当前 `uav_bringup` 已经把 Orbbec 的 `enable_laser` / `enable_ldp` 透传出来，并且默认都关闭。

这样做的目的就是：

- 减少 IR 点阵对 OpenVINS 特征跟踪的干扰
- 让双 IR + IMU 标定优先基于自然纹理收敛
- 如果以后你要做 A/B 测试，再手动打开

以后你主要只需要盯这几个入口参数：

- `camera_name`
  - 默认是 `uav_depth_camera`
  - 如果相机命名空间变了，优先改这个
- `openvins_config_path`
  - 标定阶段指向 `estimator_config.calibration.yaml`
  - 飞行阶段指向 `estimator_config.flight.yaml`
- `state_output_dir`
  - 每一轮在线标定结果保存目录
  - 推荐放到 repo 挂载路径里，不要只放容器 `/tmp`
- `publish_px4_external_vision`
  - **标定阶段关掉**
  - **飞行阶段打开**
- `enable_publish_extrinsic`
  - 建议保持 `true`
  - 这样 Orbbec 工厂外参话题会发布出来，bootstrap 才能自动取数

如果你的话题名不是默认规则，再额外覆盖：

- `imu_topic`
- `left_ir_topic`
- `right_ir_topic`

默认情况下，这三个话题会跟着 `camera_name` 自动生成，所以通常不用手改。

---

## 17. Step A：先生成粗初值（Bootstrap）

这一步的目的很明确：

- 先把 **左右 IR 内参**、**左右 IR 相对 IMU 的粗外参**、**IMU 噪声参数** 自动整理出来
- 生成一套可直接进入在线细化的 **OpenVINS 初版配置**

### 17.1 先在 Docker 内 build 一次

你已经明确说过：**不要在宿主机编译**。

所以这一步开始统一按 Docker 里的工作流来：

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

如果后面你只改 README，不改代码，这一步不用反复做。

### 17.2 把 Orbbec 相机先拉起来

如果你只是做 bootstrap，先单独起相机就够了：

```bash
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

这时候先确认三类数据都在：

```bash
ros2 topic hz /uav_depth_camera/left_ir/image_raw
ros2 topic hz /uav_depth_camera/right_ir/image_raw
ros2 topic hz /uav_depth_camera/gyro_accel/sample
```

如果这三路都不稳定，就先不要做下一步。

### 17.3 生成 bootstrap 配置

然后在另一个 Docker 终端里跑自动生成器：

```bash
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  cd /ws &&
  source /opt/ros/humble/setup.bash &&
  source /ws/install/setup.bash &&
  ros2 run uav_bringup generate_orbbec_openvins_bootstrap.py \
    --camera-name uav_depth_camera \
    --output-dir /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap'
```

生成后，重点看这个目录：

- `/repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap`

里面应该至少有：

- `estimator_config.calibration.yaml`
- `estimator_config.flight.yaml`
- `kalibr_imucam_chain.yaml`
- `kalibr_imu_chain.yaml`
- `bootstrap_summary.md`

这里建议把它理解成 **“本轮工作目录”**，不是最终定稿目录。

---

## 18. Step B：按 Fast-Drone-250 的手持慢走风格做在线标定

这一步就是你要的那个风格：

- 不是架在桌上静态点一下结束
- 而是 **拿着整机 / 传感器模组去慢走、转动、上下摆动**
- 用真实运动把外参、时间偏移、必要时的内参慢慢逼近

### 18.1 启动专用标定模式

现在仓库里已经有专门的标定 launch：

- `uav_bringup/launch/openvins_orbbec_calibration.launch.py`

它和普通飞行入口的区别是：

- 自动开启 `save_total_state`
- 自动开启 `publish_calibration_tf`
- 自动关闭 `publish_px4_external_vision`
- 会把每轮标定结果写到你指定的 `state_output_dir`
- 启动前会自动建目录
- 如果 bootstrap 配置还没生成，会直接报错，不会让你黑盒失败

推荐第一轮这样起：

```bash
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  cd /ws &&
  source /opt/ros/humble/setup.bash &&
  source /ws/install/setup.bash &&
  ros2 launch uav_bringup openvins_orbbec_calibration.launch.py \
    camera_name:=uav_depth_camera \
    openvins_config_path:=/repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/estimator_config.calibration.yaml \
    state_output_dir:=/repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/round_01'
```

### 18.2 起起来以后先做 30 秒检查

不要一上来就抱着机器乱跑，先确认下面几件事：

```bash
ros2 topic hz /uav_depth_camera/left_ir/image_raw
ros2 topic hz /uav_depth_camera/right_ir/image_raw
ros2 topic hz /uav_depth_camera/gyro_accel/sample
ros2 topic echo --once /ov_msckf/odomimu
```

正常现象应该是：

- 左右 IR 都连续出图
- IMU 频率稳定
- `/ov_msckf/odomimu` 能出消息
- `publish_px4_external_vision` 已经关闭，不会在标定时往 PX4 灌数据

### 18.3 手持慢走的推荐动作 SOP

这一段就是参考 `Fast-Drone-250` 的“人工拿着跑动”思路，但我给你改成更适合 OpenVINS 的动作模板。

建议单轮时长：**2~4 分钟**，不要只有十几秒。

推荐动作顺序：

1. **静止 10~15 秒**
   - 给 IMU 偏置和初始化一个稳定起点
2. **慢速前后平移**
   - 让系统看到明显视差
3. **慢速左右平移**
   - 避免所有运动都退化到一条轴
4. **上下抬升 / 下放**
   - 给相机-IMU 外参更多激励
5. **缓慢 yaw 转动**
   - 让时间偏移和旋转外参更容易收敛
6. **补一点 pitch / roll 小摆动**
   - 不要太猛，但要有三维激励
7. **最后再静止 5~10 秒**
   - 方便你看末端漂移和稳定性

动作上有 4 个硬要求：

- **慢，不要暴力甩**
- **场景要有纹理，不要纯白墙**
- **光照稳定，尽量别有频闪**
- **两只 IR 都要能持续看到有效场景，别总遮住一只**

如果你发现 `/ov_msckf/odomimu` 中途已经发散、翻轴、狂跳，就直接停，先不要把这一轮结果拿去回写。

### 18.4 这一轮结束后会得到什么

只要这轮跑过，`state_output_dir` 里就会有：

- `ov_estimate.txt`
- `ov_estimate_std.txt`
- `ov_groundtruth.txt`（如果配置里有对应输出）

我们后面回写时，主要用的是：

- `ov_estimate.txt`

---

## 19. Step C：导出并回写本轮标定结果

这一版已经不建议你再去终端里手抄最终 log 了。

现在直接读 OpenVINS 保存下来的最后一帧状态，把它写回 `kalibr_imucam_chain.yaml`。

### 19.1 先停掉本轮标定

本轮动作做完以后，先 `Ctrl-C` 停掉 `openvins_orbbec_calibration.launch.py`。

原因很简单：

- 你要确保 `ov_estimate.txt` 已经完整 flush 到磁盘
- 然后再拿最后一行做回写

### 19.2 把结果应用回 YAML

```bash
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  python3 /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/scripts/apply_openvins_calibration_result.py \
    --state-estimate-path /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/round_01/ov_estimate.txt \
    --input-imucam-yaml /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/kalibr_imucam_chain.yaml'
```

默认行为是：

- **直接覆盖输入 YAML**

也就是把这一轮结果直接灌回：

- `bootstrap/kalibr_imucam_chain.yaml`

如果你想保留每轮快照，就额外加：

```bash
--output-imucam-yaml /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/kalibr_imucam_chain.round_01.yaml
```

### 19.3 这个脚本会回写哪些内容

当前脚本会自动更新：

- `cam0.T_cam_imu`
- `cam1.T_cam_imu`
- `cam1.T_cn_cnm1`
- `timeshift_cam_imu`

如果你确认 OpenVINS 在线内参也已经比较可信，还可以加：

```bash
--update-intrinsics
```

但我的建议是：

- **前两轮先只回写外参 + 时间偏移**
- **最后一轮再考虑要不要把内参一起冻住**

### 19.4 回写后建议马上人工看一眼

至少检查两件事：

1. `T_cam_imu` 没有出现肉眼明显离谱的旋转 / 平移
2. `timeshift_cam_imu` 没有大到明显不合理

你可以直接看文件：

```bash
sed -n '1,120p' /home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/kalibr_imucam_chain.yaml
```

如果这一步就已经明显离谱，不要进入下一轮，先回头查：

- 场景纹理够不够
- 光照是否频闪
- 是否有一只 IR 经常被挡住
- 动作是不是太快太猛
- bootstrap 初值是不是就错了

---

## 20. Step D：重复 2~3 轮，直到结果收敛

这一步非常关键。

**不要跑一轮就当最终值。**

更稳的做法是：

1. `round_01` 跑完并回写
2. 用更新后的 `kalibr_imucam_chain.yaml` 再跑 `round_02`
3. 再回写
4. 必要时再跑 `round_03`

第二轮命令和第一轮一样，只是把输出目录换掉：

```bash
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  cd /ws &&
  source /opt/ros/humble/setup.bash &&
  source /ws/install/setup.bash &&
  ros2 launch uav_bringup openvins_orbbec_calibration.launch.py \
    camera_name:=uav_depth_camera \
    openvins_config_path:=/repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/estimator_config.calibration.yaml \
    state_output_dir:=/repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/round_02'
```

### 20.1 什么时候算“收敛得差不多了”

工程上可以用下面这几个现象来判断：

- 相邻两轮 `T_cam_imu` 的平移变化已经很小
- 相邻两轮 `T_cam_imu` 的姿态变化已经不再大跳
- `timeshift_cam_imu` 基本稳定
- 手持走一圈回来时，`/ov_msckf/odomimu` 的整体漂移明显收敛
- 静止段姿态不乱跳，平移不会疯狂滑走

如果第二轮和第三轮还差得很大，说明不是“还需要多跑一轮”这么简单，通常是下面的问题之一：

- 图像质量不好
- 红外曝光不稳定
- 时间同步本身有问题
- 机体 / 相机安装刚度不够
- 运动激励还是不够三维

### 20.2 我建议的实操节奏

一个比较稳的版本是：

- **第 1 轮**：只求先跑通，确认方向不反
- **第 2 轮**：认真做完整动作，把外参和时间偏移压稳
- **第 3 轮**：如果前两轮已经很接近，就做最后冻结前确认

通常做到这里，已经比“从 placeholder 直接上机飞”稳很多了。

---

## 21. Step E：冻结飞行配置，然后再接 PX4

到了这一步，你才应该把这套配置当成“飞行候选版本”。

### 21.1 推荐先做一个冻结快照目录

我不建议你一上来就把 `bootstrap` 目录里的文件直接覆盖仓库默认值。

更稳的做法是先做一个冻结目录，例如：

```bash
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  mkdir -p /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/frozen_20260318 &&
  cp /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/estimator_config.flight.yaml \
     /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/frozen_20260318/estimator_config.flight.yaml &&
  cp /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/kalibr_imucam_chain.yaml \
     /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/frozen_20260318/kalibr_imucam_chain.yaml &&
  cp /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/kalibr_imu_chain.yaml \
     /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/frozen_20260318/kalibr_imu_chain.yaml'
```

这样你可以同时保留：

- 一个可继续迭代的 `bootstrap`
- 一个准备上机的 `frozen_20260318`

### 21.2 用冻结后的 flight 配置起正常链路

确认标定版本稳定以后，再起正常入口：

```bash
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  cd /ws &&
  source /opt/ros/humble/setup.bash &&
  source /ws/install/setup.bash &&
  ros2 launch uav_bringup openvins_orbbec.launch.py \
    camera_name:=uav_depth_camera \
    openvins_config_path:=/repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/frozen_20260318/estimator_config.flight.yaml'
```

这时候和标定模式相比，最大的区别是：

- `publish_px4_external_vision` 回到正常工作流
- 你开始把 OpenVINS 的输出桥接给 PX4
- 进入“飞行前联调”而不是“在线求标定”

### 21.3 接 PX4 之前怎么实测

建议按下面顺序走，**不要直接起飞**：

1. **先只看 OpenVINS**
   - `ros2 topic hz /ov_msckf/odomimu`
   - 手持移动时位置 / 姿态连续变化
2. **再看桥是否进 PX4**
   - `ros2 topic hz /fmu/in/vehicle_visual_odometry`
   - `ros2 topic echo --once /fmu/in/vehicle_visual_odometry`
3. **再看 PX4 是否真的在融合**
   - `ros2 topic hz /fmu/out/vehicle_odometry`
   - `ros2 topic hz /uav/state/odometry`
4. **最后做无桨地面联调**
   - 只做上电、模式切换、位置观测
   - 不做直接起飞验证

PX4 侧仍然要配好你前面那几个关键参数：

- `EKF2_EV_CTRL`
- `EKF2_EV_DELAY`
- `EKF2_EV_POS_X`
- `EKF2_EV_POS_Y`
- `EKF2_EV_POS_Z`

如果 OpenVINS 本体还没稳，就先不要急着调 PX4 的融合参数。

### 21.4 什么时候再去覆盖仓库默认配置

只有在下面 3 个条件同时满足时，我才建议你把默认配置真正替换掉：

- 连续几轮手持标定结果已经稳定
- `frozen` 配置跑正常链路时表现稳定
- 无桨地面联调没有出现轴翻转、yaw 反号、严重漂移

到那时候，如果你想把仓库里的旧默认值彻底替掉，再手动执行：

```bash
cp /home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/frozen_20260318/estimator_config.flight.yaml \
   /home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/estimator_config.yaml

cp /home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/frozen_20260318/kalibr_imucam_chain.yaml \
   /home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/kalibr_imucam_chain.yaml
```

这样才叫“旧版本删除，新版本正式接管”。

---

## 22. Docker 内常用命令索引

以后你大概率反复只会用下面这几条。

### 22.1 build

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

### 22.2 生成 bootstrap

```bash
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  cd /ws &&
  source /opt/ros/humble/setup.bash &&
  source /ws/install/setup.bash &&
  ros2 run uav_bringup generate_orbbec_openvins_bootstrap.py \
    --camera-name uav_depth_camera \
    --output-dir /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap'
```

### 22.3 起标定模式

```bash
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  cd /ws &&
  source /opt/ros/humble/setup.bash &&
  source /ws/install/setup.bash &&
  ros2 launch uav_bringup openvins_orbbec_calibration.launch.py \
    camera_name:=uav_depth_camera \
    openvins_config_path:=/repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/estimator_config.calibration.yaml \
    state_output_dir:=/repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/round_01'
```

### 22.4 回写本轮结果

```bash
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  python3 /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/scripts/apply_openvins_calibration_result.py \
    --state-estimate-path /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/round_01/ov_estimate.txt \
    --input-imucam-yaml /repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/bootstrap/kalibr_imucam_chain.yaml'
```

### 22.5 起冻结后的飞行链路

```bash
WORKSPACE_SUBDIR=uav_hw ./scripts/dev.sh exec bash -lc '
  cd /ws &&
  source /opt/ros/humble/setup.bash &&
  source /ws/install/setup.bash &&
  ros2 launch uav_bringup openvins_orbbec.launch.py \
    camera_name:=uav_depth_camera \
    openvins_config_path:=/repo/uav_hw/src/workspace/doco_uav/src/uav_bringup/config/openvins/orbbec_stereo_imu/frozen_20260318/estimator_config.flight.yaml'
```

---

如果你接下来愿意，我下一步最值得继续补的是两件事：

- 再给你补一个 **“一键 round_01 / round_02 / round_03 管理脚本”**
- 或者直接继续帮你把 **`frozen_20260318` 这一版目录结构和默认 flight 配置也落下来**
