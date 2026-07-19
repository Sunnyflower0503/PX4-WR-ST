# PX4-WR-ST：Tandem Tailsitter HITL 固件

> 本仓库是 PX4 的项目定制分支，不是未修改的上游副本。上游 PX4 介绍保留在下文；本项目操作先阅读本节和工作区 [`../README.md`](../README.md)。

当前机型使用 `SYS_AUTOSTART=13020`、`COM_VEHICLE_ID=10` 和 `cuav_nora_default`。已实现并验证 Tandem 专用执行器分配、旋翼姿态坐标解耦、固定翼支架跃升门控、Mission 外环/TECS 调整，以及低速固定翼到旋翼转换。最近验证基线为提交 `d1bbed94be`。

## 项目接口

- MATLAB 飞机模型：`../STaircraft`
- HITL 运行说明：`../STaircraft/HITL/README.md`
- 调试报告：`../report/README.md`
- 最近链路：TELEM2 COM9@115200 负责 HIL，飞控 USB COM5 负责命令，bootloader 曾枚举为 COM3；端口号可能改变。
- QGC 路径：`D:\Program Files\QGroundControl\bin\QGroundControl.exe`，自动测试时应关闭，人工验证时可作为显示与操作界面。

## 编译和刷写

推荐保留 `build/cuav_nora_default` 做增量编译：

```bash
export PATH=/root/gcc-arm-none-eabi-7-2017-q4-major/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
cd /mnt/d/D_zx/26WORK/ShengTai/0710HITL_ST/PX4-WR-ST
make -j8 cuav_nora_default
```

固件产物：`build/cuav_nora_default/cuav_nora_default.px4`。刷写时应自动发现 CUAV Nora bootloader，不要硬编码 COM3。固件源码修改前必须先取得用户批准；获批后可直接修改、增量编译、刷写并运行回归测试。

## 关键安全状态

Mission 支架跃升使用 `TD_FW_TKO_EN` 作为独立开关，0→1 后固件还要求连续等待约 2 s。测试结束必须上锁并恢复 `TD_FW_TKO_EN=0`、`COM_RC_IN_MODE=0`，同时冻结 MATLAB 模型 `force_enable=0`。

与当前功能最相关的代码位于：

```text
ROMFS/px4fmu_common/init.d/airframes/13020_tandem_x_tailsitter
src/drivers/pwm_mix_out/
src/modules/mc_att_control/
src/modules/fw_att_control/
src/modules/fw_pos_control_l1/
src/modules/vtol_att_control/
src/modules/navigator/
```

不要修改或提交当前无关的嵌套依赖工作树（例如 `src/drivers/uavcan/libuavcan`、`src/lib/crypto/monocypher`），除非任务明确涉及它们。

---

# PX4 Drone Autopilot

[![Releases](https://img.shields.io/github/release/PX4/PX4-Autopilot.svg)](https://github.com/PX4/PX4-Autopilot/releases) [![DOI](https://zenodo.org/badge/22634/PX4/PX4-Autopilot.svg)](https://zenodo.org/badge/latestdoi/22634/PX4/PX4-Autopilot)

[![Nuttx Targets](https://github.com/PX4/PX4-Autopilot/workflows/Nuttx%20Targets/badge.svg)](https://github.com/PX4/PX4-Autopilot/actions?query=workflow%3A%22Nuttx+Targets%22?branch=master) [![SITL Tests](https://github.com/PX4/PX4-Autopilot/workflows/SITL%20Tests/badge.svg?branch=master)](https://github.com/PX4/PX4-Autopilot/actions?query=workflow%3A%22SITL+Tests%22)

[![Slack](https://px4-slack.herokuapp.com/badge.svg)](http://slack.px4.io)

This repository holds the [PX4](http://px4.io) flight control solution for drones, with the main applications located in the [src/modules](https://github.com/PX4/PX4-Autopilot/tree/master/src/modules) directory. It also contains the PX4 Drone Middleware Platform, which provides drivers and middleware to run drones.

PX4 is highly portable, OS-independent and supports Linux, NuttX and QuRT out of the box.

* Official Website: http://px4.io (License: BSD 3-clause, [LICENSE](https://github.com/PX4/PX4-Autopilot/blob/master/LICENSE))
* [Supported airframes](https://docs.px4.io/master/en/airframes/airframe_reference.html) ([portfolio](http://px4.io/#airframes)):
  * [Multicopters](https://docs.px4.io/master/en/frames_multicopter/)
  * [Fixed wing](https://docs.px4.io/master/en/frames_plane/)
  * [VTOL](https://docs.px4.io/master/en/frames_vtol/)
  * [Autogyro](https://docs.px4.io/master/en/frames_autogyro/)
  * [Rover](https://docs.px4.io/master/en/frames_rover/)
  * many more experimental types (Blimps, Boats, Submarines, High altitude balloons, etc)
* Releases: [Downloads](https://github.com/PX4/PX4-Autopilot/releases)


## Building a PX4 based drone, rover, boat or robot

The [PX4 User Guide](https://docs.px4.io/master/en/) explains how to assemble [supported vehicles](https://docs.px4.io/master/en/airframes/airframe_reference.html) and fly drones with PX4.
See the [forum and chat](https://docs.px4.io/master/en/#support) if you need help!


## Changing code and contributing

This [Developer Guide](https://docs.px4.io/master/en/development/development.html) is for software developers who want to modify the flight stack and middleware (e.g. to add new flight modes), hardware integrators who want to support new flight controller boards and peripherals, and anyone who wants to get PX4 working on a new (unsupported) airframe/vehicle.

Developers should read the [Guide for Contributions](https://docs.px4.io/master/en/contribute/).
See the [forum and chat](https://dev.px4.io/master/en/#support) if you need help!


### Weekly Dev Call

The PX4 Dev Team syncs up on a [weekly dev call](https://dev.px4.io/master/en/contribute/#dev_call).

> **Note** The dev call is open to all interested developers (not just the core dev team). This is a great opportunity to meet the team and contribute to the ongoing development of the platform. It includes a QA session for newcomers. All regular calls are listed in the [Dronecode calendar](https://www.dronecode.org/calendar/).


## Maintenance Team

  * Project: Founder
    * [Lorenz Meier](https://github.com/LorenzMeier)
  * Architecture
    * [Daniel Agar](https://github.com/dagar)
  * [Dev Call](https://github.com/PX4/PX4-Autopilot/labels/devcall)
    * [Ramon Roche](https://github.com/mrpollo)
  * Communication Architecture
    * [Beat Kueng](https://github.com/bkueng)
    * [Julian Oes](https://github.com/JulianOes)
  * UI in QGroundControl
    * [Gus Grubba](https://github.com/dogmaphobic)
  * [Multicopter Flight Control](https://github.com/PX4/PX4-Autopilot/labels/multicopter)
    * [Mathieu Bresciani](https://github.com/bresch)
  * [Multicopter Software Architecture](https://github.com/PX4/PX4-Autopilot/labels/multicopter)
    * [Matthias Grob](https://github.com/MaEtUgR)
  * [VTOL Flight Control](https://github.com/PX4/PX4-Autopilot/labels/vtol)
    * [Roman Bapst](https://github.com/RomanBapst)
  * [Fixed Wing Flight Control](https://github.com/PX4/PX4-Autopilot/labels/fixedwing)
    * [Roman Bapst](https://github.com/RomanBapst)
  * OS / NuttX
    * [David Sidrane](https://github.com/davids5)
  * Driver Architecture
    * [Daniel Agar](https://github.com/dagar)
  * Commander Architecture
    * [Julian Oes](https://github.com/julianoes)
  * [UAVCAN](https://github.com/PX4/PX4-Autopilot/labels/uavcan)
    * [Daniel Agar](https://github.com/dagar)
  * [State Estimation](https://github.com/PX4/PX4-Autopilot/issues?q=is%3Aopen+is%3Aissue+label%3A%22state+estimation%22)
    * [Paul Riseborough](https://github.com/priseborough)
  * Vision based navigation and Obstacle Avoidance
    * [Markus Achtelik](https://github.com/markusachtelik)
  * RTPS/ROS2 Interface
    * [Nuno Marques](https://github.com/TSC21)

See also [maintainers list](https://px4.io/community/maintainers/) (px4.io) and the [contributors list](https://github.com/PX4/PX4-Autopilot/graphs/contributors) (Github).

## Supported Hardware

This repository contains code supporting Pixhawk standard boards (best supported, best tested, recommended choice) and proprietary boards.

### Pixhawk Standard Boards
  * FMUv6X and FMUv6U (STM32H7, 2021)
    * Various vendors will provide FMUv6X and FMUv6U based designs Q3/2021
  * FMUv5 and FMUv5X (STM32F7, 2019/20)
    * [Pixhawk 4 (FMUv5)](https://docs.px4.io/master/en/flight_controller/pixhawk4.html)
    * [Pixhawk 4 mini (FMUv5)](https://docs.px4.io/master/en/flight_controller/pixhawk4_mini.html)
    * [CUAV V5+ (FMUv5)](https://docs.px4.io/master/en/flight_controller/cuav_v5_plus.html)
    * [CUAV V5 nano (FMUv5)](https://docs.px4.io/master/en/flight_controller/cuav_v5_nano.html)
    * [Auterion Skynode (FMUv5X)](https://docs.px4.io/master/en/flight_controller/auterion_skynode.html)
  * FMUv4 (STM32F4, 2015)
    * [Pixracer](https://docs.px4.io/master/en/flight_controller/pixracer.html)
    * [Pixhawk 3 Pro](https://docs.px4.io/master/en/flight_controller/pixhawk3_pro.html)
  * FMUv3 (STM32F4, 2014)
    * [Pixhawk 2](https://docs.px4.io/master/en/flight_controller/pixhawk-2.html)
    * [Pixhawk Mini](https://docs.px4.io/master/en/flight_controller/pixhawk_mini.html)
    * [CUAV Pixhack v3](https://docs.px4.io/master/en/flight_controller/pixhack_v3.html)
  * FMUv2 (STM32F4, 2013)
    * [Pixhawk](https://docs.px4.io/master/en/flight_controller/pixhawk.html)
    * [Pixfalcon](https://docs.px4.io/master/en/flight_controller/pixfalcon.html)

### Manufacturer and Community supported
  * [Holybro Durandal](https://docs.px4.io/master/en/flight_controller/durandal.html)
  * [Hex Cube Orange](https://docs.px4.io/master/en/flight_controller/cubepilot_cube_orange.html)
  * [Hex Cube Yellow](https://docs.px4.io/master/en/flight_controller/cubepilot_cube_yellow.html)
  * [Airmind MindPX V2.8](http://www.mindpx.net/assets/accessories/UserGuide_MindPX.pdf)
  * [Airmind MindRacer V1.2](http://mindpx.net/assets/accessories/mindracer_user_guide_v1.2.pdf)
  * [Bitcraze Crazyflie 2.0](https://docs.px4.io/master/en/flight_controller/crazyflie2.html)
  * [Omnibus F4 SD](https://docs.px4.io/master/en/flight_controller/omnibus_f4_sd.html)
  * [Holybro Kakute F7](https://docs.px4.io/master/en/flight_controller/kakutef7.html)
  * [Raspberry PI with Navio 2](https://docs.px4.io/master/en/flight_controller/raspberry_pi_navio2.html)

Additional information about supported hardware can be found in [PX4 user Guide > Autopilot Hardware](https://docs.px4.io/master/en/flight_controller/).

## Project Roadmap

A high level project roadmap is available [here](https://github.com/orgs/PX4/projects/25).
