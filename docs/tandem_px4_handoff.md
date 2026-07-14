# TandemTailSitter PX4 HITL 开发交接文档

本文档记录 TandemTailSitter (SYS_AUTOSTART=13020, MAV_TYPE=20, VT_TYPE=0, COM_VEHICLE_ID=10)
在 PX4-WR-ST 仓库中的 HITL 固件修改历史。每次对话一个条目，供其他智能体快速了解上下文。

---

## 对话1 — 2026-07-12: ATTITUDE_QUATERNION.hpp 修复 + pwm_mix_out 混控改造

### 修改1: 去除 tailsitter FW 模式 repr_offset_q 旋转

**文件**: `src/modules/mavlink/streams/ATTITUDE_QUATERNION.hpp`

**问题**: PX4 对 tailsitter 发送 ROTATION_PITCH_90 的 repr_offset_q，QGC 显示 Rot=180°, Pitch=52.4°。

**修复**: 替换 if/else 块为平零四元数 `[0,0,0,0]`，GCS 直接渲染 vehicle_attitude 原始姿态。

```cpp
// 修改后 (行86-101)
msg.repr_offset_q[0] = 0.0f;
msg.repr_offset_q[1] = 0.0f;
msg.repr_offset_q[2] = 0.0f;
msg.repr_offset_q[3] = 0.0f;
```

### 修改2: TandemTailSitter MC 模式混控从 quad-X 改为 Tandem 矩形布局

**文件**: `src/drivers/pwm_mix_out/pwm_mix_out.cpp`

MC 模式 MAIN1-4: `thr0 ± pitch0 ± roll0`（矩形 Tandem 布局），不再使用 0.707107 系数。 yaw0 不用于 MAIN1-4。

### 修改3: TandemTailSitter FW 模式偏航差动

**文件**: `src/drivers/pwm_mix_out/pwm_mix_out.cpp`

FW 模式下增加左组(MAIN2+3) vs 右组(MAIN1+4) 偏航推力差动，参考旧代码的 dt_left/dt_right 模式。

### 修改4: fw_att_control 90° 坐标旋转分析（仅分析，未改代码）

**文件**: `src/modules/fw_att_control/FixedwingAttitudeControl.cpp` (行374-416)

完整追踪姿态误差路径，确认 tailsitter 旋转矩阵变换：列0↔列2交换，新列0取反，roll/yaw rate 交换。

---

## 对话2 — 2026-07-12: 撤销大面积混控修改 + 恢复原始符号

**背景**: 对话1中对 elevon 符号和 pitch_motor_diff 符号也做了修改，用户要求撤销，只保留偏航差动。

**操作**: 一系列精确 Edit 恢复:
- pitch_motor_diff = +pitch1 × FW_PMD_GAIN (恢复正号)
- elevon: left = pitch1 + roll1, right = pitch1 - roll1 (恢复原符号)
- 保留 yaw 差动的 dt_right/dt_left 分组逻辑

---

## 对话3 — 2026-07-13: Pitch电机差动手动/自动方向不一致 — 根因分析

**问题**: 手动STABILIZED和自稳/自动模式对 pitch_motor_diff 方向要求相反。

**分析结果** (详见 `docs/pitch_direction_root_cause.md`):

1. **坐标系混用**: `pitch_error = pitch_setpoint(NED) - pitch_feedback(FW适配帧)` 在不同坐标系做减法。

2. **FW俯仰歧义性**: θ_fw = asin(cos(θ_ned)) 是偶函数。抬头37.6°和低头37.6°映射到同一个 θ_fw=52.4°，只靠 φ_fw (0° vs 180°) 区分。

3. **Jacobian翻转**: `bodyrate_sp = cos(φ_fw) × rate_sp`。φ_fw=180°时符号翻转，导致相同 pitch_error 在不同姿态角下产生相反的 pitch_u。

**结论**: 符号需求取决于飞行器的物理俯仰方向(抬头 vs 低头)，不是手动 vs 自动的差异。

**推荐修复**: 添加 FW_PMD_SIGN 参数(默认-1)，现场标定确定符号。

---

## 对话4 — 2026-07-13: yaw 控制信号追踪 — 找到 yaw1/yaw0 永远为0的根因

**问题**: FW 模式下 MAIN1-4 偏航推力差动无效。

**根因**: `vtol_att_control/tailsitter.cpp` 的 `fill_actuator_outputs()` 遗漏了 yaw 路由。

**具体**: 函数中有 `fw_out[ROLL] = fw_in[ROLL]` 和 `fw_out[PITCH] = fw_in[PITCH]`，但没有 `fw_out[YAW] = fw_in[YAW]`。导致 actuator_controls_1 的 yaw 始终为0。

### 修改1: tailsitter.cpp — 新增 yaw 路由

**文件**: `src/modules/vtol_att_control/tailsitter.cpp` (行320新增)

```cpp
// else 分支中原有:
fw_out[actuator_controls_s::INDEX_ROLL]  = fw_in[actuator_controls_s::INDEX_ROLL];
fw_out[actuator_controls_s::INDEX_PITCH] = fw_in[actuator_controls_s::INDEX_PITCH];
// 新增:
fw_out[actuator_controls_s::INDEX_YAW]   = fw_in[actuator_controls_s::INDEX_YAW];
```

### 修改2: pwm_mix_out.cpp — yaw0 改为 yaw1

**文件**: `src/drivers/pwm_mix_out/pwm_mix_out.cpp` (行630)

```cpp
// 旧: thr_diff = math::constrain(yaw0, -thr_diff_limit, thr_diff_limit);
// 新: thr_diff = math::constrain(yaw1, -thr_diff_limit, thr_diff_limit);
```

### 链路确认

```
fw_att_control yaw controller
  → actuator_controls_virtual_fw.control[YAW]
  → vtol_att_control fw_in[YAW]
  → fill_actuator_outputs: fw_out[YAW] = fw_in[YAW]  ← 新增
  → actuator_controls_1.control[YAW]
  → pwm_mix_out yaw1
  → thr_diff = constrain(yaw1, ...)                  ← 改读 yaw1
  → MAIN1/4 vs MAIN2/3 偏航推力差动
```

### 验证方法

固件编译烧录后，在 FW 模式下:
```
listener actuator_controls_1
```
左右打方向舵，`control[2]` (INDEX_YAW) 应有非零值变化。

---

## 当前代码关键参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| VT_FW_DIFTHR_EN | 0 | 差动推力使能(当前关闭) |
| VT_FW_DIFTHR_SC | 0.1 | 差动推力缩放(fw_in[PITCH]→mc_out[PITCH]) |
| FW_PMD_GAIN | 0.2 | FW俯仰电机差动增益 |
| FW_PMD_ASPD_ST | 15.0 m/s | 差动开始减小的空速 |
| FW_PMD_ASPD_FULL | 8.0 m/s | 差动全效的空速下限 |
| MIXER_D_THR_LIM | 0.3 | 推力差动限幅 |
| COM_VEHICLE_ID | 10 | TandemTailSitter标识 |

## 当前混控矩阵 (pwm_mix_out.cpp TandemTailSitter case)

### FW 模式 (第612-670行)

```
pitch_motor_diff = pitch1 × FW_PMD_GAIN × low_aspd_weight
thr_diff = constrain(yaw1, -thr_diff_limit, +thr_diff_limit)

dt_right = constrain(thr0 + 0.5 × thr_diff, idle, max)
dt_left  = constrain(thr0 - 0.5 × thr_diff, idle, max)

MAIN1 = right_front = dt_right - pitch_motor_diff   (右前)
MAIN2 = left_rear   = dt_left  + pitch_motor_diff   (左后)
MAIN3 = left_front  = dt_left  - pitch_motor_diff   (左前)
MAIN4 = right_rear  = dt_right + pitch_motor_diff   (右后)

MAIN5 = left_elevon  = pitch1 + roll1
MAIN6 = right_elevon = pitch1 - roll1
MAIN7 = min (翼尖桨关闭)
MAIN8 = min
```

### MC 模式 (第672-689行)

```
motor1 = thr0 + pitch0 + roll0  → MAIN1 (右前)
motor2 = thr0 - pitch0 - roll0  → MAIN2 (左后)
motor3 = thr0 + pitch0 - roll0  → MAIN3 (左前)
motor4 = thr0 - pitch0 + roll0  → MAIN4 (右后)

MAIN5/6 = trim (锁定)
MAIN7 = tip_idle + yaw_gain × yaw6
MAIN8 = tip_idle - yaw_gain × yaw6
```

## 相关文档

- `docs/tandem_control_group_mapping.md` — 控制组映射详情
- `docs/tandem_mixer_design.md` — 混控设计文档
- `docs/pitch_direction_root_cause.md` — 俯仰方向根因分析

## 待解决问题

1. **Pitch 差动符号**: 需要在 HITL 中通过 FW_PMD_SIGN 参数确定正确符号，取决于飞行器实际抬头/低头姿态。
2. **VT_FW_DIFTHR_EN = 0**: 当前差动推力开关关闭，如果需要 fw_in[YAW]→mc_out[ROLL] 的差动推力路由，需设为1。
3. **FW_PMD_GAIN 调参**: 飞行测试后可能需要调整俯仰差动增益。
