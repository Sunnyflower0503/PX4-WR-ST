# 串列翼通道表与半物理检查项

日期：2026-07-10

## 主验证机型

本阶段最终使用 `13020_tandem_x_tailsitter`，对应 mixer 为 `TANDEM_x_vtol.main.mix`。

## `13020_tandem_x_tailsitter` 输出链路

- 多旋翼主动力：`R: 4x`，用于四组主动力输出。
- `MAIN5`：左升降副翼，来自固定翼虚拟控制组。
- `MAIN6`：右升降副翼，来自固定翼虚拟控制组。
- `MAIN7`：左翼尖小桨，来自 `actuator_controls_6`。
- `MAIN8`：右翼尖小桨，来自 `actuator_controls_6`。

## 需要实物或 Simulink 确认的映射

当前计划验证的四组主动力映射是：

- `MAIN1`：前翼右侧两个桨。
- `MAIN2`：后翼左侧两个桨。
- `MAIN3`：前翼左侧两个桨。
- `MAIN4`：后翼右侧两个桨。

在接线或 Simulink 输入顺序确认前，不凭假设改固件输出顺序。

## 半物理检查项

- 固定翼手控：观察 pitch、roll、yaw、throttle 对四组主动力、舵面和 `actuator_controls_6` 的影响。
- VTOL / 旋翼模式：观察 pitch 映射、roll 差动、yaw、throttle，以及 `MAIN7/MAIN8` 翼尖小桨输出。
- 模式切换：观察 `actuator_controls_0`、`actuator_controls_1`、`actuator_controls_6` 输出连续性。
- 通信链路：确认 `airdata_hil`、`actuator_controls_dsc`、Simulink 输入顺序仍正常。

## 本阶段不做

- 不实现 `45°` 静止跃升起飞自动状态机。
- 不把 8 个主桨重新设计为 8 个独立控制输出。
