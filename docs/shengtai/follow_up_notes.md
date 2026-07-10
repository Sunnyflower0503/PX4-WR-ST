# 后续说明

日期：2026-07-10

## 当前阶段目标

当前阶段只把实飞固件已有的串列翼输出链路和 VTOL/tailsitter 控制律迁移到可编译的 `PX4-WR` 半物理固件中，用于 `13020_tandem_x_tailsitter` 半物理验证。

## 翼尖小桨判断

`13020_tandem_x_tailsitter` 使用 `TANDEM_x_vtol.main.mix`，其中 `MAIN7`、`MAIN8` 消费 `actuator_controls_6`。因此固件层面已经具备翼尖小桨输出路径。

联调时仍需确认：

- 所选 airframe 确实是 `13020_tandem_x_tailsitter`。
- `TANDEM_x_vtol.main.mix` 被加载。
- `MAIN7`、`MAIN8` 在 Simulink 中对应翼尖小桨。
- `actuator_controls_6` 在 VTOL / 固定翼相关模式下有有效输出。

## `45°` 跃升起飞建议

`45°` 静止跃升起飞不在本阶段实现。建议在半物理输出链路验证稳定后，单独设计：

- 起飞前姿态保持和油门约束。
- 从静止 `45°` 姿态到跃升爬升的切换条件。
- 与现有 VTOL/tailsitter 模式切换的关系。
- 先在半物理中验证，再考虑合入实飞固件。
