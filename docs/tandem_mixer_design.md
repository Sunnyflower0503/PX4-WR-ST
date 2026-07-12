# TandemTailSitter Mixer Design

Date: 2026-07-12

This document records the actuator allocation implemented in `src/drivers/pwm_mix_out/pwm_mix_out.cpp` for the TandemTailSitter. It only defines fixed-wing and rotary-wing actuator mapping. It does not implement automatic jump takeoff, waypoint logic, transition state machines, or landing assistance.

## Channel Map

| Output | Function |
| --- | --- |
| MAIN1 | Right-front main prop pair |
| MAIN2 | Left-rear main prop pair |
| MAIN3 | Left-front main prop pair |
| MAIN4 | Right-rear main prop pair |
| MAIN5 | Left elevon |
| MAIN6 | Right elevon |
| MAIN7 | Left wingtip yaw prop |
| MAIN8 | Right wingtip yaw prop |

MAIN1-MAIN4 represent four PWM groups for eight main propellers, with two motors per group.

## Fixed-Wing Mixer

Fixed-wing mode is detected when `vtol_vehicle_status.vtol_in_rw_mode == false` and `vtol_vehicle_status.vtol_in_trans_mode == false`.

The main props do not use normal 4x multirotor roll or pitch mixing in fixed-wing mode. Fixed-wing roll is intentionally ignored by MAIN1-MAIN4 while the aircraft is supported on the ground. The main props provide total thrust, low-airspeed front/rear pitch assist, and left/right yaw differential:

```text
low_aspd_weight = ramp(airspeed, FW_PMD_ASPD_ST, FW_PMD_ASPD_FULL)
pitch_motor_diff = constrain(pitch0 * FW_PMD_GAIN * low_aspd_weight,
                             -MIXER_D_THR_LIM,
                             +MIXER_D_THR_LIM)

front = throttle0 + pitch_motor_diff
rear  = throttle0 - pitch_motor_diff

yaw_motor_diff = constrain(yaw0 * MIXER_THR_RUD_S,
                           -MIXER_D_THR_LIM,
                           +MIXER_D_THR_LIM)

MAIN1 = right_front = front - 0.5 * yaw_motor_diff
MAIN2 = left_rear   = rear  + 0.5 * yaw_motor_diff
MAIN3 = left_front  = front + 0.5 * yaw_motor_diff
MAIN4 = right_rear  = rear  - 0.5 * yaw_motor_diff
```

MAIN5 and MAIN6 remain the original elevon allocation and receive normal fixed-wing roll and pitch commands:

```text
MAIN5 = -roll1 + pitch1
MAIN6 = -roll1 - pitch1
```

Wingtip props are off in fixed-wing mode:

```text
MAIN7 = minimum
MAIN8 = minimum
```

## Rotary-Wing Mixer

Rotary-wing and transition states use the rotary allocation in this driver. MAIN1-MAIN4 use only group 0 throttle, roll, and pitch. Group 0 yaw is intentionally ignored:

```text
MAIN1 = throttle0 - 0.707107 * roll0 + 0.707107 * pitch0
MAIN2 = throttle0 + 0.707107 * roll0 - 0.707107 * pitch0
MAIN3 = throttle0 + 0.707107 * roll0 + 0.707107 * pitch0
MAIN4 = throttle0 - 0.707107 * roll0 - 0.707107 * pitch0
```

MAIN5 and MAIN6 are locked to trim in rotary-wing mode.

MAIN7 and MAIN8 use only group 6 yaw:

```text
left_tip  = tip_idle + yaw6 * yaw_gain
right_tip = tip_idle - yaw6 * yaw_gain
```

`roll6`, `pitch6`, and `thr6` are not used by the TandemTailSitter output allocation.

## Parameters

PX4 limits parameter names to 16 characters, so the implemented names are shortened versions of the requested names.

| Implemented parameter | Requested meaning | Default | Meaning |
| --- | --- | --- | --- |
| `FW_PMD_GAIN` | `FW_PITCH_MOTOR_DIFF_GAIN` | `0.2` | Gain from fixed-wing pitch control to front/rear main-prop differential |
| `FW_PMD_ASPD_ST` | `FW_PITCH_MOTOR_DIFF_ASPD_START` | `15.0 m/s` | Above this airspeed, fixed-wing pitch motor differential is zero |
| `FW_PMD_ASPD_FULL` | `FW_PITCH_MOTOR_DIFF_ASPD_FULL` | `8.0 m/s` | At and below this airspeed, fixed-wing pitch motor differential uses full gain |
| `MIXER_D_THR_LIM` | existing limit parameter | existing value | Symmetric limit for `pitch_motor_diff` |
| `MIXER_YAW_SC` | existing yaw scale parameter | existing value | Wingtip yaw gain scale in rotary-wing mode |

## Mode Behavior

| Mode | MAIN1-MAIN4 | MAIN5-MAIN6 | MAIN7-MAIN8 |
| --- | --- | --- | --- |
| Fixed-wing | Total thrust, low-airspeed front/rear pitch differential, and left/right yaw differential; roll ignored by MAIN1-MAIN4 | Elevon roll/pitch | Minimum |
| Rotary-wing | Multirotor roll/pitch/throttle, yaw disabled | Trim locked | Group 6 yaw only |
| Transition flag set | Uses rotary-wing allocation in this driver | Trim locked | Group 6 yaw only |

## Not Implemented

- Automatic jump takeoff.
- Waypoint behavior.
- Fixed-wing to rotary-wing transition logic.
- Rotary-wing to fixed-wing transition logic.
- Landing assistance.
- Changes to `fw_att_control`, `mc_att_control`, PID gains, or tailsitter control laws.
