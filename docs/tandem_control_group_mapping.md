# TandemTailSitter Control Group Mapping

Date: 2026-07-12

This document records the HITL `pwm_mix_out` control-group wiring and the TandemTailSitter output mapping after aligning it with the original flight firmware architecture. The original flight reference is `D:\D_zx\251201CLY\Firmware\Firmware`; the HITL code is `D:\D_zx\26WORK\ShengTai\0710HITL_ST\PX4-WR-ST`.

## Current HITL Variable Sources

Before this change, `mix_and_update_outputs()` named the group 0 values as `roll1/pitch1/yaw1/thr1`, so the variable names did not match their uORB control group. The code now reads each group explicitly:

| Variable | uORB topic | Source struct |
| --- | --- | --- |
| `roll0/pitch0/yaw0/thr0` | `actuator_controls_0` | `_actuator_controls_0.control[...]` |
| `roll1/pitch1/yaw1/thr1` | `actuator_controls_1` | `_actuator_controls_1.control[...]` |
| `roll6/pitch6/yaw6/thr6` | `actuator_controls_6` | `_actuator_controls_6.control[...]` |

The driver subscribes to all three required topics in `src/drivers/pwm_mix_out/pwm_mix_out.hpp`:

| Topic | Subscription member | Status |
| --- | --- | --- |
| `actuator_controls_0` | `_actuator_controls_0_sub` | Subscribed |
| `actuator_controls_1` | `_actuator_controls_1_sub` | Subscribed |
| `actuator_controls_6` | `_actuator_controls_6_sub` | Subscribed |

The last assignment before `mix_and_update_outputs()` is in `Run()`:

```cpp
_actuator_controls_0_sub.update(&_actuator_controls_0);
_actuator_controls_1_sub.update(&_actuator_controls_1);
_actuator_controls_6_sub.update(&_actuator_controls_6);
_vtol_vehicle_status_sub.update(&_vtol_vehicle_status);
```

The variables are then copied from those structs at the start of `mix_and_update_outputs()`.

## Original Flight Architecture

The original flight airframe `13020_tandem_x_tailsitter` uses:

```sh
set MIXER TANDEM_x_vtol
set PWM_OUT 12345678
param set-default VT_TYPE 0
param set-default VT_ELEV_MC_LOCK 1
```

The original `TANDEM_x_vtol.main.mix` architecture is:

| Output | Control group | Mixer role |
| --- | --- | --- |
| MAIN1-MAIN4 | `actuator_controls_0` | `R: 4x` multirotor mixer; each output drives a pair of main motors |
| MAIN5-MAIN6 | `actuator_controls_1` | elevon mixer |
| MAIN7-MAIN8 | `actuator_controls_6` | wingtip/additional prop mixer |

The confirmed main-motor grouping is:

| Output | Physical group |
| --- | --- |
| MAIN1 | Right-front main motor pair |
| MAIN2 | Left-rear main motor pair |
| MAIN3 | Left-front main motor pair |
| MAIN4 | Right-rear main motor pair |

The original elevon signs are:

| Output | Roll | Pitch |
| --- | ---: | ---: |
| MAIN5 left elevon | -1 | +1 |
| MAIN6 right elevon | -1 | -1 |

The HITL code now follows this group ownership for TandemTailSitter instead of using group 1 values to drive main motors or using `roll6` to drive wingtip props.

## Before And After

| Output | Before HITL Tandem mapping | After Tandem mapping |
| --- | --- | --- |
| MAIN1-MAIN4 | Used variables named `thr1/yaw1`, but those were actually from `actuator_controls_0`; custom left/right differential formula | Uses `actuator_controls_0` only; fixed-wing uses `roll0/pitch0/yaw0/thr0` with `R: 4x` coefficients; rotary/transition uses `roll0/pitch0/thr0` with yaw forced to zero; outputs drive right-front, left-rear, left-front, right-rear motor pairs |
| MAIN5-MAIN6 | Used variables named `roll1/pitch1`, but those were actually from `actuator_controls_0`; signs did not match original right elevon | Uses `actuator_controls_1`; fixed-wing signs match original elevon mixer: left `-roll1 + pitch1`, right `-roll1 - pitch1` |
| MAIN7-MAIN8 | Fixed-wing set to min, but rotary/other old branches could use `roll6`; no clear yaw-only contract | Fixed-wing set to min; rotary/transition uses only `yaw6`: `left_tip = tip_idle + yaw_gain * yaw6`, `right_tip = tip_idle - yaw_gain * yaw6` |

## Fixed-Wing Mode

Fixed-wing mode is detected as:

```cpp
!_vtol_vehicle_status.vtol_in_rw_mode && !_vtol_vehicle_status.vtol_in_trans_mode
```

| Output | Control group used | Axes used | Notes |
| --- | --- | --- | --- |
| MAIN1-MAIN4 | group 0 | roll, pitch, yaw, throttle | `R: 4x` coefficient order from PX4 multirotor mixer |
| MAIN5-MAIN6 | group 1 | roll, pitch | Original elevon signs |
| MAIN7-MAIN8 | none | none | Forced to minimum output |

## Rotary-Wing / Transition Mode

Transition is treated with the rotary-wing output policy so it does not accidentally enable fixed-wing-only behavior.

| Output | Control group used | Axes used | Notes |
| --- | --- | --- | --- |
| MAIN1-MAIN4 | group 0 | roll, pitch, throttle | group 0 yaw is forced to zero for these outputs |
| MAIN5-MAIN6 | group 1 or trim | roll/pitch only if `VT_ELEV_MC_LOCK=0`; otherwise trim | Mirrors original `VT_ELEV_MC_LOCK` intent |
| MAIN7-MAIN8 | group 6 | yaw only | `roll6`, `pitch6`, and `thr6` are not used |

Wingtip prop output formula:

```cpp
left_tip = tip_idle + yaw_gain * yaw6;
right_tip = tip_idle - yaw_gain * yaw6;
```

In code, `tip_idle = 1300 us` and `yaw_gain = 1000 * MIXER_YAW_SC`, with each output constrained by the corresponding MAIN7/MAIN8 min/max parameters.

## Disabled Error Mappings

For TandemTailSitter, this change removes or disables:

- `thr1/yaw1` driving MAIN1-MAIN4.
- group 1 directly driving main props.
- `roll6` driving wingtip props.
- fixed-wing mode producing non-minimum MAIN7/MAIN8 output.
- use of group 6 roll, pitch, or throttle in the output driver.

The old upstream `vtol_att_control` behavior that forced `actuator_controls_6[THROTTLE] = 0.6` is not modified in this task, per the scope restriction. The output driver now ignores `thr6`, so that old value no longer affects MAIN7-MAIN8 through `pwm_mix_out`.

## Still To Confirm

- Exact ESC wiring inside each MAIN1-MAIN4 motor pair.
- The physical left/right assignment of MAIN7 and MAIN8 wingtip props.
- Whether `tip_idle = 1300 us` is correct for the real wingtip ESC idle point.
- Whether the servo throw sign after MAIN5/MAIN6 matches real linkage direction; the software signs now match the original flight mixer.
