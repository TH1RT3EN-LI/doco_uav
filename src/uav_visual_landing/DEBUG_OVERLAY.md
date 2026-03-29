# `uav_visual_landing` Debug Overlay 说明

当前 debug 图像由 `ws/src/uav_visual_landing/src/aruco_detector_node.cpp` 生成，默认发布到 `/uav/visual_landing/debug_image`。

## 当前叠字行的含义

1. `VISUAL LANDING DEBUG`
2. `PHASE: ...`
3. `TAG: DETECTED/MISSING`
4. `TAG_Z=... src=... conf=...`
5. `H_CTRL=... src=...`
6. `RANGE=... src=... valid=...`
7. `HFRESH samp=... recv=... basis=...`
8. `TERM: ...`
9. `ERR n=(..., ...)`
10. `ERR_F=(..., ...)`
11. `DERR=(..., ...)`
12. `LERR=... xy=(..., ...)`
13. `TH=0.08/0.05 VXY_MAX=...`
14. `ZTG=... ZE=...`
15. `YAW=... rep=... span=...`

## 字段解释

- `TAG_Z`
  - 当前视觉侧估计的 tag 轴向深度，而不是相机到 tag 中心的斜距
  - `src=PNP_Z` 表示使用 `tvec.z`
  - `src=SPAN_Z` 表示使用 `tag_size * focal_mean / marker_span_px`

- `H_CTRL`
  - 控制器当前真正用于 `z` 闭环的高度
  - 该值优先来自测距高度，测距失效时会回退到 odom

- `RANGE`
  - 最近一次收到的原始高度测量值
  - `src=DISTANCE_SENSOR_RAW` 表示直接读取原始 `DistanceSensor`
  - legacy 原始链路通常来自 `/uav/fmu/out/distance_sensor`
  - 默认推荐链路会由 `uav_bridge/height_measurement_bridge_node` 发布 `sensor_msgs/msg/Range`
  - 默认 stamped topic 为 `/uav/sensors/downward_range`
  - `src=PX4_DIST_BOTTOM[...]` 表示读取 PX4 `vehicle_local_position.dist_bottom`
  - `valid=Y` 表示该测量当前通过控制器校验并可用于 `H_CTRL`

- `HFRESH`
  - `samp=Y/N` 表示基于样本时间 `header.stamp` 的 freshness
  - `recv=Y/N` 表示基于节点接收时间的 freshness
  - `basis=SAMPLE_TIME` 表示当前控制器按样本时间解释高度输入
  - `basis=LEGACY_RECEIVE_TIME` 表示当前仍在 legacy 路径上，overlay 会额外显示 `legacy_time_basis=receive_time`
  - `sample_fresh` 与 `receive_fresh` 只描述“原始高度测量是否新鲜”，不等同于 `H_CTRL` 最终是否采用该高度

- `TERM`
  - 当前终端对齐触发源
  - `RANGE_MEASUREMENT` 表示 terminal 由连续满足阈值的 sample-time 高度测量触发
  - `NONE` 表示当前还未满足 terminal 触发条件

- `src`
  - 在 `H_CTRL` 行里:
    - `RANGE_MEASUREMENT` 表示当前 `z` 控制使用测距高度
    - `ODOM` 表示当前 `z` 控制退回到 odom 高度
  - 在 `RANGE` 行里:
    - 表示原始高度测量来自哪里

- `ERR n`
  - 当前帧原始归一化像面误差

- `ERR_F`
  - 控制器滤波后的归一化像面误差

- `DERR`
  - 控制器估计的归一化误差变化率

- `LERR`
  - 基于当前 `H_CTRL` 投影得到的真实横向偏差，单位米
  - `xy=(x, y)` 分别对应图像 `u / v` 误差换算后的米制偏差

- `TH=0.08/0.05`
  - 对齐窗迟滞阈值，分别表示“进入对齐窗 8 cm”和“保持对齐窗 5 cm”

- `ZTG`
  - 当前 `z` 目标高度

- `ZE`
  - `z` 目标高度与 `H_CTRL` 之间的误差

- `YAW`
  - 当前 yaw 误差

- `rep`
  - PnP 重投影误差，单位像素

- `span`
  - 标签平均边长对应的像素尺度，单位像素

## 如何解读当前链路

- `TAG_Z` 只用于视觉诊断，不直接作为主控高输入
- `H_CTRL` 才是控制器实际拿去做 `z` 闭环的高度
- `RANGE` 表示原始高度输入，`HFRESH` 说明它当前采用的是 sample-time 还是 receive-time 语义
- `LERR` 才是当前对齐判据对应的真实米制偏差
- 启动搜索后，控制器会先拉到 `search_height_m` 并在该高度完成对齐与稳定保持
- 控制器允许短时漏检，只有连续 miss 达到阈值或有效观测超时，才会退出当前视觉跟踪阶段
- 默认配置下，下降阶段只有 sample-time 高度测量连续进入 `terminal_entry_height_m` 以下，控制器才会进入终端对齐并最终触发 `LAND`
- 若当前是 legacy 高度路径，overlay 会明确显示 `legacy_time_basis`；此时 legacy freshness 仅用于诊断，不承担严格 sample-time terminal 判据
