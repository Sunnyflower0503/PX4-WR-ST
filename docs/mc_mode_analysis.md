# TandemTailSitter MC 模式自稳紊乱分析

日期: 2026-07-13

## 现象

机头垂直朝上（旋翼模式），STABILIZED 解锁后乱飘。没有偏航和滚转输入，但飞机无法稳定悬停，姿态不受控。

## 发现的问题

### 问题1: MC 模式用 Quad-X 系数 (0.707107), 不是 Tandem 矩形布局

**文件**: `src/drivers/pwm_mix_out/pwm_mix_out.cpp` (行689-692)

```cpp
// 当前 MC 模式混控 — 用 Quad-X 0.707107 系数
const float motor1 = thr0 + 0.707107f * pitch0 - 0.707107f * roll0 + main_yaw;  // MAIN1
const float motor2 = thr0 - 0.707107f * pitch0 + 0.707107f * roll0 + main_yaw;  // MAIN2
const float motor3 = thr0 + 0.707107f * pitch0 + 0.707107f * roll0 - main_yaw;  // MAIN3
const float motor4 = thr0 - 0.707107f * pitch0 - 0.707107f * roll0 - main_yaw;  // MAIN4
```

这是标准 Quad-X 多旋翼的混控矩阵。但 TandemTailSitter 的四个电机是**矩形串联布局**（前后两排，左右两组），不是对角线分布的 X 四轴。

Quad-X 的 pitch0/roll0 映射到 Tandem: MAIN1(右前), MAIN2(左后), MAIN3(左前), MAIN4(右后)。

Quad-X pitch0 加力方向: **MAIN1+MAIN3(前), MAIN2-MAIN4(后)** — 但 Tandem 的前后是 MAIN1/MAIN3 vs MAIN2/MAIN4，这是对的。

Quad-X roll0 加力方向: **MAIN3(左前)增, MAIN4(右后)增, MAIN1(右前)减, MAIN2(左后)减** — 这会同时产生滚转和耦合俯仰！

**结论**: MC 混控系数仍然是 Quad-X 布局(含 0.707107)，不是之前记录的 Tandem 矩形布局。之前对话中声称的"改为 Tandem 矩形布局"实际没有改——当前代码仍然是 Quad-X。

### 问题2: MC 姿态控制器的 dcm_z() 推力向量方向

**文件**: `src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp` (行55-81)

MC 姿态控制器的 `update()` 使用 reduced attitude quaternion:
```cpp
const Vector3f e_z = q.dcm_z();    // 机体 Z 轴在 NED 中的方向 = 当前推力方向
const Vector3f e_z_d = qd.dcm_z(); // 目标姿态的机体 Z 轴方向 = 目标推力方向
Quatf qd_red(e_z, e_z_d);         // 从当前推力对齐到目标推力的最短旋转
```

**对于机头垂直朝上的 tailsitter:**
- 物理: Z 轴向下(推力向下), X 轴指天(机头方向)
- `q.dcm_z()` = (0, 0, -1) 在 NED 中 → 推力指向地

**目标姿态** (tailsitter STABILIZED, 行165-173):
```cpp
const Quatf q_sp = Quatf(AxisAnglef(v(0), v(1), 0.f)) * Quatf(Eulerf(0.0f, M_PI_2_F, 0.0f));
```

- `Eulerf(0, M_PI_2_F, 0)` = 绕 Y 轴旋转 +90° = 机头从水平抬起 90° → 机头朝天
- `qd.dcm_z()` 在这种情况下 = (?, ?, ?)

当 sticks centered (v=0,0):
- q_sp = I × Euler(0, π/2, 0) = nose-up 姿态
- 推力向量 = Z 轴在 NED → (X_body 方向, 在 NED 中... )

**核心问题**: 机头垂直朝上时:
- 机体 Z 轴水平指向某个方向
- 机体 X 轴垂直朝上
- 四个螺旋桨的推力方向 = 机体 X 轴方向 = 垂直向上（反重力）

但 MC 姿态控制器**假设推力和机体 Z 轴对齐**。当 Z 轴水平时，`q.dcm_z()` 给出的是水平方向，不是推力方向。

这导致:
- 姿态误差 `eq = 2 * qe.imag()` 的 pitch/roll/yaw 分量与实际需要的控制方向可能不同
- 姿态控制器产生的 rates_sp 可能指向错误的方向

### 问题3: generate_attitude_setpoint 中 tailsitter 的 q_sp 构造

**文件**: `src/modules/mc_att_control/mc_att_control_main.cpp` (行165-179)

```cpp
const Quatf q_sp = Quatf(AxisAnglef(v(0), v(1), 0.f)) * Quatf(Eulerf(0.0f, M_PI_2_F, 0.0f));
```

当摇杆居中 (v = 0,0):
- `AxisAnglef(0,0,0) = I` (恒等旋转)
- `q_sp = I * Euler(0, π/2, 0)` = 纯俯仰+90°
- 目标姿态 = 机头朝天，0 roll

这个 q_sp 的 `dcm_z()`:
```
Euler(0, π/2, 0) 的旋转矩阵:
[cos(π/2)  0  sin(π/2)]   [0  0  1]
[0         1  0       ] = [0  1  0]
[-sin(π/2) 0  cos(π/2)]   [-1 0  0]

Z 轴 (第2列, index 2): [0, 0, 1] → 旋转后 = [0, 1, 0]
```

所以 q_sp 的 dcm_z() = **β = (0, 1, 0)**，目标推力方向 = NED 北方向(水平)！

但实际车辆的推力方向是 **垂直向上** (机体 X 轴 = 推力/升力方向)。

### 根本问题: Z 轴和推力方向不重合

MC 姿态控制器 built on the assumption that **机体质心推力沿着机体 Z 轴负方向**。对于 tailsitter 垂直悬停状态:
- 机体 Z 轴指向水平方向
- 推力(升力)沿着机体 X 轴垂直向上
- 控制器用 dcm_z() 对齐，但实际需要的是 dcm_x() 对齐

**这导致姿态控制器彻底错误** — 它在尝试把水平方向的 Z 轴对齐到某个方向，而实际需要控制的是垂直方向的 X 轴。

## 总结

MC 模式自稳紊乱的三个原因:

| 序号 | 问题 | 影响 |
|---|---|---|
| 1 | MC 混控是 Quad-X 系数(0.707107)，不是 Tandem 矩形 | pitch/roll 力臂不正确，俯仰和滚转耦合 |
| 2 | AttitudeControl 假设推力沿 Z 轴，但 tailsitter 垂停时推力沿 X 轴 | 姿态控制器的推力向量对齐完全错误 |
| 3 | yaw0 在 `fill_actuator_outputs()` MC 模式是活值 (`mc_in[YAW] × 1.0`)，但 yaw 来源是 mc_att_control 的 rates_sp，可能产生零或不正确的偏航控制量 | 偏航控制失效 |
