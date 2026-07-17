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
4. **MC mode yaw feedforward coupling**: 已修复，见对话8。

---

## 对话8 — 2026-07-13 ~ 2026-07-14: MC 模式帧适配验证 + 混控重写 + 单轴调试

### 帧适配代码审查

用户对 `mc_att_control_main.cpp` 的三处改动经审查确认正确：
- `q_control = q * Euler(0, -π/2, 0)` — 鼻头朝上映射为水平零姿态 ✓
- `q_sp = AxisAnglef(v)` — 适配帧悬停目标 = I ✓
- rate swap (适配X→真实Z, Y→Y, Z→-真实X) ✓

### MC 混控从 Quad-X 改为 Tandem 矩形布局

**文件**: `pwm_mix_out.cpp`, `pwm_mix_out.hpp`, `mixer_params.c`

帧适配后 actuator_controls_0 真实轴语义：
```
roll0  = 绕真实X轴(推力轴/垂直朝上) → 自旋偏航
pitch0 = 绕真实Y轴(水平) → 前后倾斜
yaw0   = 绕真实Z轴(水平) → 左右倾斜 (注意:变量名叫yaw但物理是横滚!)
```

MC 混控改为 Tandem 矩形布局：
```
motor1 = thr0 + pitch0 - yaw0 + main_yaw  // MAIN1 右前: front+right
motor2 = thr0 - pitch0 + yaw0 + main_yaw  // MAIN2 左后: rear+left
motor3 = thr0 + pitch0 + yaw0 - main_yaw  // MAIN3 左前: front+left
motor4 = thr0 - pitch0 - yaw0 - main_yaw  // MAIN4 右后: rear+right
```
- pitch0 做前后差动 (MAIN1+3 vs MAIN2+4)
- yaw0 做左右差动 (MAIN1+4 vs MAIN2+3)
- main_yaw (源自 roll0) 走对角反扭 + 翼尖桨 MAIN7-8 差动
- 去掉 Quad-X 的 0.707107 系数（不等力臂才需要）

### 新增 MC 单轴调试参数

**文件**: `mixer_params.c` + `pwm_mix_out.hpp`

| 参数 | 默认 | 说明 |
|------|------|------|
| TD_MC_DBG_PITCH | 0 | 前后倾斜 (pitch0 → 前列vs后列). -1=反, 0=关, 1=正 |
| TD_MC_DBG_ROLL  | 0 | 左右倾斜 (yaw0 → 右侧vs左侧). -1=反, 0=关, 1=正 |
| TD_MC_DBG_SPIN  | 0 | 自旋偏航 (roll0 → 翼尖+反扭). -1=反, 0=关, 1=正 |

油门始终开启。全关=四桨等推力纯悬停。逐轴确认方向后全开。

### 单轴测试结论

- 俯仰轴和滚转轴方向正确，单开可稳住。
- 偏航(自旋)轴控不住 — 根因是 `AttitudeControl::update()` 的 yawspeed feedforward 通过 `q_control.inversed().dcm_z()` 投影到轻微倾斜的机体，耦合出 pitch/roll 分量导致发散。

---

## 对话9 — 2026-07-17: 修复 yawspeed feedforward 轴间耦合

### 问题

打偏航杆 → `yaw_sp_move_rate ≠ 0` → `AttitudeControl::update()` 执行：
```cpp
rate_setpoint += q_control.inversed().dcm_z() * _yawspeed_setpoint;
```

`dcm_z()` 在 q_control ≠ I 时（飞机有微小倾斜）会投影出三轴分量，yaw feedforward 漏入 pitch/roll 轴，mixer 真的产生俯仰/滚转力矩，飞机发散。

### 修复

不改 AttitudeControl 本体，在 `mc_att_control_main.cpp` 的调用侧处理：

1. 调用 `update(q_control)` 前 — 保存并清零 yawspeed
2. rate swap 后 — 把 yaw rate 直接加到 `rates_sp(0)`（真实X轴=推力轴=自旋轴），不经过投影

### 改动文件

**`AttitudeControl.hpp`** (行92后新增):
```cpp
float getYawspeedSetpoint() const { return _yawspeed_setpoint; }
void clearYawspeedSetpoint() { _yawspeed_setpoint = 0.0f; }
```

**`mc_att_control_main.cpp`** (行340-347替换):
```cpp
// 保存并清零 yawspeed，避免 update() 内 dcm_z() 投影耦合
float yaw_ff = 0.0f;
if (tailsitter_control_frame) {
    yaw_ff = _attitude_control.getYawspeedSetpoint();
    _attitude_control.clearYawspeedSetpoint();
}

Vector3f rates_sp = _attitude_control.update(q_control);

if (tailsitter_control_frame) {
    // ... rate swap (不变) ...
    // 直接加到推力轴自旋通道，零耦合
    if (fabsf(yaw_ff) > 1e-6f) {
        rates_sp(0) -= yaw_ff;
    }
}
```

非 tailsitter 路径完全不受影响。

---

## 对话10 — 2026-07-17: 旋翼悬停角与翼尖桨俯仰前馈

新增两个参数：

- `TD_MC_HOV_P`：tailsitter 旋翼增稳模式的中立悬停俯仰角。全局默认 `90 deg`，13020 机型默认 `87 deg`。仅修改手动增稳姿态目标，不作用于固定翼、过渡、定高或定点控制。
- `TD_TIP_P_FF`：根据翼尖桨实际限幅 PWM 的总推力平方估计，为四个主桨加入俯仰差动前馈。扣除翼尖怠速基线，默认 `0` 表示关闭。

前馈计算使用：
```text
tip_load = (tip_left_norm^2 + tip_right_norm^2) / 2 - tip_idle_norm^2
pitch_ff = TD_TIP_P_FF * tip_load
```

支架验证时先保持 `TD_TIP_P_FF=0`，确认 `TD_MC_HOV_P=87` 的方向正确；随后以很小绝对值逐步测试 `TD_TIP_P_FF` 的正负方向。若翼尖桨增大后俯仰偏差变大，立即归零并反向。固定翼和过渡控制链未改。

构建验证：`make cuav_nora_default` 通过，固件位于 `build/cuav_nora_default/cuav_nora_default.px4`。

### 偏航触发发散的后续修复

实时 MAVLink 监测显示：目标自旋角速度约 `+/-0.0349 rad/s`，实际 `xgyro` 却发散到约 `+6.8 rad/s`；速率控制器已输出约 `roll=-1.3` 刹车，但旋转仍加速。

确认并修复两处执行链问题：

1. `TD_MC_DIRECT_EN` 原先在自稳模式也会让混控采用摇杆值而忽略闭环 `roll0`。现在直通只允许在姿态控制关闭时生效，自稳模式始终保留速率控制器刹车权限。
2. STaircraft 定义翼尖力矩 `Mx = addprop_y * (T_left - T_right)`，而原 `TD_TIP_YAW_REV=0` 的输出方向相反。13020 默认改为 `TD_TIP_YAW_REV=1`。

首次验证建议将 `TD_MC_YAW_MAIN=0`，只验证翼尖闭环方向；确认后再逐步恢复主桨反扭份额。
