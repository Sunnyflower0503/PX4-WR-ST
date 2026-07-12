# vtol_att_control 启动失败和 actuator_controls_0/1/6 不发布 — 根因分析报告

## 1. 问题现象

- `vtol_att_control start` 报错：`ERROR [vtol_att_control] can't open /dev/pwm_output0`
- `listener actuator_controls_virtual_mc` 有数据（来自 `mc_rate_control`）
- `listener actuator_controls_0/1/6` 从未发布

## 2. 根因

### 完整因果链

```
pwm_mix_out 替代 pwm_out (rc.interface line 13)
  → pwm_mix_out 使用 NuttX 原生 up_pwm_servo_init() API
  → 没有继承 cdev::CDev，不创建 /dev/pwm_output0 设备节点
    → vtol_att_control 启动时 VtolType::init() 调用 px4_open("/dev/pwm_output0") 失败
      → vtol_att_control 启动失败，exit_and_cleanup()，从未到达 Run()
        → actuator_controls_0/1/6 从未发布
          → pwm_mix_out 虽然有读取 actuator_controls_0/1/6 的逻辑，但收不到数据
            → 无 PWM 输出
```

### 关键文件与代码位置

| 文件 | 作用 | 关键行 |
|---|---|---|
| `vtol_type.cpp:82-91` | `VtolType::init()` 打开 `/dev/pwm_output0` 查询 PWM 限制值 | **根因位置** — `px4_open(dev, 0)` 失败即退出 |
| `vtol_type.cpp:345-373` | `apply_pwm_limits()` 运行时也通过 ioctl 设置 PWM 最大/最小值 | **次要问题** — init 过了这里也会失败 |
| `pwm_mix_out.cpp:59` | 使用 `up_pwm_servo_init(_pwm_mask)` 直接初始化 PWM | 不创建 `/dev/pwm_output0` 设备节点 |
| `rc.interface:13` | `set OUTPUT_CMD pwm_mix_out` — 用自定义模块替代 `pwm_out` | 引入问题的入口 |
| `13020_tandem_x_tailsitter:28` | `set MIXER skip` — 跳过 mixer 加载 | 即使不 skip，设备节点也不存在 |

### 为什么 HITL 和实飞代码对比没有差异

因为两个仓库的 `vtol_att_control` 源码完全一致。问题不在 `vtol_att_control` 本身的逻辑，而在运行时环境：实飞固件有 `pwm_out` 驱动创建 `/dev/pwm_output0`，HITL 固件的 `pwm_mix_out` 不创建。

## 3. 最小修改方案

**策略：让 `VtolType::init()` 和 `apply_pwm_limits()` 在 `/dev/pwm_output0` 不存在时使用默认值优雅降级，而不是直接失败退出。**

只需修改一个文件：`src/modules/vtol_att_control/vtol_type.cpp`，两处改动。

### 改动 1：`VtolType::init()` — 允许设备打开失败，使用默认值

**文件：** `src/modules/vtol_att_control/vtol_type.cpp`  
**位置：** 第 82-132 行

构造函数（第 73-78 行）已经预设了默认值：
- `_max_mc_pwm_values` 各通道 = `PWM_DEFAULT_MAX`（2000）
- `_disarmed_pwm_values` 各通道 = `PWM_MOTOR_OFF`（900）

所以当设备打不开时，保留这些默认值即可。

### 改动 2：`apply_pwm_limits()` — 设备打不开时直接跳过

**文件：** `src/modules/vtol_att_control/vtol_type.cpp`  
**位置：** 第 345-373 行

当前代码：`fd < 0` 时 `return false`。  
改为：`fd < 0` 时 `return true`（跳过 ioctl，继续运行）。

## 4. 具体修改

### 修改 A：vtol_type.cpp `init()`

将 `init()` 中的硬错误改为警告 + 降级使用默认值：

```cpp
bool VtolType::init()
{
    const char *dev = _params->vt_mc_on_fmu ? PWM_OUTPUT1_DEVICE_PATH : PWM_OUTPUT0_DEVICE_PATH;

    int fd = px4_open(dev, 0);

    if (fd < 0) {
        // 改: 不返回 false，降级使用构造函数中预设的默认 PWM 值继续运行
        PX4_WARN("can't open %s, using default PWM values", dev);
    } else {
        int ret = px4_ioctl(fd, PWM_SERVO_GET_MAX_PWM, (long unsigned int)&_max_mc_pwm_values);
        _current_max_pwm_values = _max_mc_pwm_values;

        if (ret != PX4_OK) {
            PX4_ERR("failed getting max values");
            px4_close(fd);
            return false;
        }

        ret = px4_ioctl(fd, PWM_SERVO_GET_MIN_PWM, (long unsigned int)&_min_mc_pwm_values);

        if (ret != PX4_OK) {
            PX4_ERR("failed getting min values");
            px4_close(fd);
            return false;
        }

        ret = px4_ioctl(fd, PWM_SERVO_GET_DISARMED_PWM, (long unsigned int)&_disarmed_pwm_values);

        if (ret != PX4_OK) {
            PX4_ERR("failed getting disarmed values");
            px4_close(fd);
            return false;
        }

        px4_close(fd);
    }

    // ... 后面的 bitmap 生成逻辑保持不变 ...
```

### 修改 B：vtol_type.cpp `apply_pwm_limits()`

```cpp
bool VtolType::apply_pwm_limits(struct pwm_output_values &pwm_values, pwm_limit_type type)
{
    const char *dev = _params->vt_mc_on_fmu ? PWM_OUTPUT1_DEVICE_PATH : PWM_OUTPUT0_DEVICE_PATH;

    int fd = px4_open(dev, 0);

    if (fd < 0) {
        // 改: 不返回 false，因为 pwm_mix_out 不创建设备节点，跳过 ioctl 继续运行
        PX4_DEBUG("can't open %s, skip pwm limit ioctl", dev);
        return true;  // 原来是 return false
    }

    // ... 后面的 ioctl 逻辑保持不变 ...
```

## 5. 为什么这是最小且安全的修改

1. **只改一个文件**：`src/modules/vtol_att_control/vtol_type.cpp`
2. **只改两处**：`init()` 的设备打开失败路径、`apply_pwm_limits()` 的设备打开失败路径
3. **不改变任何正常路径的逻辑**：如果 `/dev/pwm_output0` 存在（标准 `pwm_out` 场景），行为完全不变
4. **默认值合理**：构造函数预设的 `PWM_DEFAULT_MAX`(2000) / `PWM_MOTOR_OFF`(900) 是 PX4 标准默认值，对 tailsitter 机型足够使用
5. **不需要修改 pwm_mix_out**：`pwm_mix_out` 本身不需要 `/dev/pwm_output0`，它通过 `up_pwm_servo_set()` 直接写硬件
6. **不需要重构架构**：不需要给 `pwm_mix_out` 添加 `cdev::CDev` 设备节点（那会引入大量代码）

## 6. 数据流验证

修改后的完整数据流：

```
mc_rate_control (vtol 模式)
  → 发布 actuator_controls_virtual_mc  ← 用户已确认有数据
    → vtol_att_control::Run() 的 should_run 条件满足
      → 发布 actuator_controls_0   ← 修复后正常
      → 发布 actuator_controls_1   ← 修复后正常
      → 发布 actuator_controls_6   ← 修复后正常
        → pwm_mix_out::mix_and_update_outputs() 读到数据
          → up_pwm_servo_set() 输出 PWM  ← 修复后正常
```

## 7. 为什么不改 pwm_mix_out

让 `pwm_mix_out` 也创建 `/dev/pwm_output0` 设备节点需要：
- 继承 `cdev::CDev` 类
- 实现完整的 `ioctl` 接口（至少 `PWM_SERVO_GET_MAX_PWM`、`PWM_SERVO_GET_MIN_PWM`、`PWM_SERVO_GET_DISARMED_PWM`、`PWM_SERVO_SET_MIN_PWM`、`PWM_SERVO_SET_MAX_PWM`）
- 管理多通道的 PWM 值数组
- 这等于把 `pwm_out` 的大部分逻辑复制到 `pwm_mix_out`

而 `pwm_mix_out` 的设计初衷就是绕过 PX4 的 mixer 和 pwm_out 框架，直接控制硬件。在 `vtol_att_control` 侧做降级处理更合理，改动更小。
