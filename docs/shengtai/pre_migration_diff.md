# PX4-WR 迁移前差异报告

日期：2026-07-10

## 源码与资料位置

- 主工作仓库：`D:\2025\program\shengtai\work\px4\PX4-WR`
- 旧参考仓库：`D:\2025\program\shengtai\work\px4\PX4-WR-ST-main\PX4-WR-ST-main`
- 实飞固件：`D:\2025\program\shengtai\work\px4\Firmware`
- 参数文档：`D:\2025\program\shengtai\work\px4\串列翼参数.docx`
- 最终主验证机型：`13020_tandem_x_tailsitter`

## 已确认一致或保留的内容

| 范围 | 状态 | 说明 |
| --- | --- | --- |
| `airdata_hil` | 保留 | `PX4-WR` 已有 MAVLink receiver 发布和固定翼位置控制消费链路。 |
| `actuator_controls_dsc` | 保留 | `PX4-WR` 已有直接侧力 / 差动油门半物理链路。 |
| 固定翼 `actuator_controls_6` | 保留 | `PX4-WR` 已有固定翼侧发布逻辑和相关参数。 |
| `pwm_mix_out` 半物理输出 | 保留 | 已有 `actuator_controls_6`、`actuator_controls_dsc` 订阅。 |

## 需要迁移的内容

| 范围 | 迁移动作 |
| --- | --- |
| 串列翼 airframe | 引入 `13020_tandem_x_tailsitter`，同时保留 `2102_tandem_fw`、`2103_tandem_2p` 作为参考/备选。 |
| 串列翼 mixer | 引入 `TANDEM.main.mix`、`TANDEM_2P.main.mix`、`TANDEM_elevator.main.mix`、`TANDEM_x_vtol.main.mix`。 |
| VTOL 控制律 | 补齐 `VT_FW_DIF_R_SC`、`diff_thrust_roll_scale`、VTOL 下 `actuator_controls_6` 发布。 |
| Tailsitter 映射 | 在固定翼模式下补齐实飞固件已有的 pitch 到多旋翼输出映射。 |

## 风险说明

- `MAIN1` 到 `MAIN4` 的实机四组主动力顺序仍需结合接线或 Simulink 输入确认，本次迁移不改输出顺序。
- `PX4-WR` 当前存在多个子模块 `m` 状态，属于接收源码后的子模块状态，不纳入迁移提交。
- `git submodule status --recursive` 会因 `Tools/simulation-ignition` 缺少 `.gitmodules` 映射报错，该问题单独记录，不和控制律迁移混在一起。
