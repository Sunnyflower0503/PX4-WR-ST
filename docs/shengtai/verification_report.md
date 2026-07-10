# PX4-WR 迁移验证报告

日期：2026-07-10

## Git 基线

- 主仓库：`D:\2025\program\shengtai\work\px4\PX4-WR`
- 起始分支：`port-firmware-control-law`
- 迁移分支：`shengtai/flight-control-hitl-align-px4wr`
- 基线保护文件：`D:\2025\program\shengtai\work\px4\PX4_WR_baseline_20260710.txt`

## 迁移提交

- `0036234 feat: add tandem hitl airframes and mixers`
- `742ecaa43c feat: align tandem vtol actuator controls`
- 文档记录提交：`docs: record tandem px4wr migration notes`

## Target 发现结果

已通过 WSL 执行 `make list_config_targets`，确认存在：

- `cuav_x7pro_default`
- `cuav_x7pro_test`
- `px4_fmu-v5_default`
- `cubepilot_cubeorange_default`
- `px4_sitl_default`

## 构建状态

迁移前后均已尝试 `make cuav_x7pro_default`。CMake 已进入 `cuav_x7pro_default` 配置阶段，但 WSL 环境缺少 `arm-none-eabi-gcc` / `arm-none-eabi-g++`，因此硬件 target 构建被工具链阻塞。

该阻塞不是迁移代码造成的。恢复 ARM 工具链后，应优先重新构建 `cuav_x7pro_default`。如果实际控制器是 V5/V5nano，再补充构建 `px4_fmu-v5_default`。

已执行 `make px4_sitl_default`，配置、参数生成、uORB 生成和源码编译均通过，最终完成 `811/811` 个构建步骤并生成 `bin/px4`。

## 静态接口验证

静态检查确认以下接口仍存在：

- `actuator_controls_0`
- `actuator_controls_6`
- `actuator_controls_dsc`
- `airdata_hil`
- `13020_tandem_x_tailsitter`
- `TANDEM_x_vtol.main.mix`
- `VT_FW_DIF_R_SC`
- `diff_thrust_roll_scale`
- `TANDEM_x_vtol.main.mix` 中消费 `S: 6 ...`
