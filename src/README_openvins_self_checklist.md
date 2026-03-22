# OpenVINS 标定链路自检清单

## 1. 目录结构

- [ ] 我知道 `bootstrap/` 是可覆盖的标定工作区。
- [ ] 我知道 `frozen_final/` 是唯一运行态配置区。
- [ ] 我确认根目录 `orbbec_stereo_imu/*.yaml` / `*.pgm` 不再存在活动副本。

## 2. bootstrap 输入

- [ ] 我会先启动 `openvins_orbbec_bootstrap_source.launch.py`，而不是直接盲跑 bootstrap 脚本。
- [ ] 我知道 bootstrap 脚本必须拿到左右 `CameraInfo`。
- [ ] 我知道 bootstrap 脚本必须拿到 `depth_to_left_ir` / `depth_to_right_ir`。
- [ ] 我知道 bootstrap 脚本必须拿到 `depth_to_gyro`。
- [ ] 我知道当前项目只认 gyro reference 语义，不再保留 accel-reference 分支。
- [ ] 我知道 `gyro_info` / `accel_info` 缺失时只是回退到项目默认噪声值，不代表真实设备值。

## 3. 标定 launch 语义

- [ ] 我知道 `openvins_orbbec_calibration.launch.py` 会强制打开 depth extrinsics。
- [ ] 我知道它会强制 `openvins_save_total_state=true`。
- [ ] 我知道它会强制 `openvins_publish_calibration_tf=true`。
- [ ] 我知道它会强制 `publish_px4_external_vision=false`。
- [ ] 我知道它不是运行态 launch，不能拿来当正常飞行默认入口。

## 4. apply 行为

- [ ] 我知道 `apply_openvins_calibration_result.py` 默认原地覆盖输入 `kalibr_imucam_chain.yaml`。
- [ ] 我知道它不再支持旧的“顺手更新内参”开关。
- [ ] 我知道它默认会拒绝写回不稳定的一轮结果。
- [ ] 我知道不稳定判据至少包括：高 final speed、baseline 变化过大、stereo relative rotation 变化过大。
- [ ] 我知道只有显式传 `--force` 才能绕过这些保护。

## 5. freeze 与运行

- [ ] 我会先在 `bootstrap/` 收敛，再用 `freeze_openvins_orbbec_config.py` 冻结到 `frozen_final/`。
- [ ] 我知道 `openvins_orbbec.launch.py` 默认只读 `frozen_final/estimator_config.flight.yaml`。
- [ ] 我知道 `minimal_control.launch.py` 默认也只读 `frozen_final/estimator_config.flight.yaml`。
- [ ] 我知道运行阶段必须保持 `calib_cam_extrinsics=false`。
- [ ] 我知道运行阶段必须保持 `calib_cam_timeoffset=false`。

## 6. 常见误区

- [ ] 我不会再把根目录旧配置文件当成正式入口。
- [ ] 我不会再用 `round_01_applied.yaml` 这种旁路文件作为默认多轮工作流。
- [ ] 我不会在运行态直接读 `bootstrap/`。

## 7. 过线标准

如果下面 5 条都能勾上，说明当前链路理解和使用方式基本达标：

- [ ] 我能独立生成一套新的 `bootstrap/`。
- [ ] 我能解释 bootstrap 脚本依赖的 topic 为何缺一不可。
- [ ] 我能完成一轮在线标定并把结果安全地 apply 回 `bootstrap/`。
- [ ] 我能在收敛后正确 freeze 到 `frozen_final/`。
- [ ] 我能确认运行入口没有任何路径再退回旧根目录配置。
