/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "pwm_mix_out.hpp"

using namespace matrix;

pwm_mix_out::pwm_mix_out() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
    _cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
    _interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": interval"))
{
//        _mixing_output.setAllMinValues(PWM_DEFAULT_MIN);
//        _mixing_output.setAllMaxValues(PWM_DEFAULT_MAX);

    update_params();

    _VEHICLE_ID = _vehicle_id.get();
    _vtol_vehicle_status.vtol_in_rw_mode = true;

    _pwm_default_rate = 50;
    _pwm_alt_rate = 100;
    _pwm_alt_rate_channels = 0;
    _pwm_mask = 0b0011'1111'1111'1111;
    _pwm_initialized = true;
    _num_outputs = 14;

    up_pwm_servo_init(_pwm_mask);
}

pwm_mix_out::~pwm_mix_out()
{
    /* make sure servos are off */
    up_pwm_servo_deinit(_pwm_mask);

    perf_free(_cycle_perf);
    perf_free(_interval_perf);
}


bool pwm_mix_out::init()
{
    _control_subs[0].registerCallback();
    ScheduleNow();
    return true;
}

void pwm_mix_out::update_params()
{
        updateParams();

//        pwm_main1_max = (uint16_t) _pwm_main1_max.get();
//        pwm_main2_max = (uint16_t) _pwm_main2_max.get();
//        pwm_main3_max = (uint16_t) _pwm_main3_max.get();
//        pwm_main4_max = (uint16_t) _pwm_main4_max.get();
//        pwm_main5_max = (uint16_t) _pwm_main5_max.get();
//        pwm_main6_max = (uint16_t) _pwm_main6_max.get();
//        pwm_main7_max = (uint16_t) _pwm_main7_max.get();
//        pwm_main8_max = (uint16_t) _pwm_main8_max.get();
//        pwm_aux1_max  = (uint16_t) _pwm_aux1_max.get();
//        pwm_aux2_max  = (uint16_t) _pwm_aux2_max.get();
//        pwm_aux3_max  = (uint16_t) _pwm_aux3_max.get();
//        pwm_aux4_max  = (uint16_t) _pwm_aux4_max.get();
//        pwm_aux5_max  = (uint16_t) _pwm_aux5_max.get();
//        pwm_aux6_max  = (uint16_t) _pwm_aux6_max.get();
//        pwm_aux7_max  = (uint16_t) _pwm_aux7_max.get();
//        pwm_aux8_max  = (uint16_t) _pwm_aux8_max.get();

//        pwm_main1_min = (uint16_t) _pwm_main1_min.get();
//        pwm_main2_min = (uint16_t) _pwm_main2_min.get();
//        pwm_main3_min = (uint16_t) _pwm_main3_min.get();
//        pwm_main4_min = (uint16_t) _pwm_main4_min.get();
//        pwm_main5_min = (uint16_t) _pwm_main5_min.get();
//        pwm_main6_min = (uint16_t) _pwm_main6_min.get();
//        pwm_main7_min = (uint16_t) _pwm_main7_min.get();
//        pwm_main8_min = (uint16_t) _pwm_main8_min.get();
//        pwm_aux1_min  = (uint16_t) _pwm_aux1_min.get();
//        pwm_aux2_min  = (uint16_t) _pwm_aux2_min.get();
//        pwm_aux3_min  = (uint16_t) _pwm_aux3_min.get();
//        pwm_aux4_min  = (uint16_t) _pwm_aux4_min.get();
//        pwm_aux5_min  = (uint16_t) _pwm_aux5_min.get();
//        pwm_aux6_min  = (uint16_t) _pwm_aux6_min.get();
//        pwm_aux7_min  = (uint16_t) _pwm_aux7_min.get();
//        pwm_aux8_min  = (uint16_t) _pwm_aux8_min.get();

//        pwm_main1_trim = (uint16_t) _pwm_main1_trim.get();
//        pwm_main2_trim = (uint16_t) _pwm_main2_trim.get();
//        pwm_main3_trim = (uint16_t) _pwm_main3_trim.get();
//        pwm_main4_trim = (uint16_t) _pwm_main4_trim.get();
//        pwm_main5_trim = (uint16_t) _pwm_main5_trim.get();
//        pwm_main6_trim = (uint16_t) _pwm_main6_trim.get();
//        pwm_main7_trim = (uint16_t) _pwm_main7_trim.get();
//        pwm_main8_trim = (uint16_t) _pwm_main8_trim.get();
//        pwm_aux1_trim  = (uint16_t) _pwm_aux1_trim.get();
//        pwm_aux2_trim  = (uint16_t) _pwm_aux2_trim.get();
//        pwm_aux3_trim  = (uint16_t) _pwm_aux3_trim.get();
//        pwm_aux4_trim  = (uint16_t) _pwm_aux4_trim.get();
//        pwm_aux5_trim  = (uint16_t) _pwm_aux5_trim.get();
//        pwm_aux6_trim  = (uint16_t) _pwm_aux6_trim.get();
//        pwm_aux7_trim  = (uint16_t) _pwm_aux7_trim.get();
//        pwm_aux8_trim  = (uint16_t) _pwm_aux8_trim.get();

//        pwm_main1_max_x = _pwm_main1_max_x.get();
//        pwm_main2_max_x = _pwm_main2_max_x.get();
//        pwm_main3_max_x = _pwm_main3_max_x.get();
//        pwm_main4_max_x = _pwm_main4_max_x.get();
//        pwm_main5_max_x = _pwm_main5_max_x.get();
//        pwm_main6_max_x = _pwm_main6_max_x.get();
//        pwm_main7_max_x = _pwm_main7_max_x.get();
//        pwm_main8_max_x = _pwm_main8_max_x.get();
//        pwm_aux1_max_x  = _pwm_aux1_max_x.get();
//        pwm_aux2_max_x  = _pwm_aux2_max_x.get();
//        pwm_aux3_max_x  = _pwm_aux3_max_x.get();
//        pwm_aux4_max_x  = _pwm_aux4_max_x.get();
//        pwm_aux5_max_x  = _pwm_aux5_max_x.get();
//        pwm_aux6_max_x  = _pwm_aux6_max_x.get();
//        pwm_aux7_max_x  = _pwm_aux7_max_x.get();
//        pwm_aux8_max_x  = _pwm_aux8_max_x.get();

//        pwm_main1_min_x = _pwm_main1_min_x.get();
//        pwm_main2_min_x = _pwm_main2_min_x.get();
//        pwm_main3_min_x = _pwm_main3_min_x.get();
//        pwm_main4_min_x = _pwm_main4_min_x.get();
//        pwm_main5_min_x = _pwm_main5_min_x.get();
//        pwm_main6_min_x = _pwm_main6_min_x.get();
//        pwm_main7_min_x = _pwm_main7_min_x.get();
//        pwm_main8_min_x = _pwm_main8_min_x.get();
//        pwm_aux1_min_x  = _pwm_aux1_min_x.get();
//        pwm_aux2_min_x  = _pwm_aux2_min_x.get();
//        pwm_aux3_min_x  = _pwm_aux3_min_x.get();
//        pwm_aux4_min_x  = _pwm_aux4_min_x.get();
//        pwm_aux5_min_x  = _pwm_aux5_min_x.get();
//        pwm_aux6_min_x  = _pwm_aux6_min_x.get();
//        pwm_aux7_min_x  = _pwm_aux7_min_x.get();
//        pwm_aux8_min_x  = _pwm_aux8_min_x.get();

        thr_alloc_threshold = _thr_alloc_threshold.get();
        thr_rud_sc = _thr_rud_sc.get();
        thr_ele_sc = _thr_ele_sc.get();

        thr_diff_limit = _thr_diff_limit.get();
        param_fw_thr_idle = _param_fw_thr_idle.get();
        param_fw_thr_max = _param_fw_thr_max.get();
}

int pwm_mix_out::set_pwm_rate(unsigned current_rate, unsigned default_rate, unsigned alt_rate)
{
    /* We should note that group is iterated over from 0 to FMU_MAX_ACTUATORS.
     * This allows for the ideal worlds situation: 1 channel per group
     * configuration.
     *
     * This is typically not what HW supports. A group represents a timer
     * and channels belongs to a timer.
     * Therefore all channels in a group are dependent on the timer's
     * common settings and can not be independent in terms of count frequency
     * (granularity of pulse width) and rate (period of repetition).
     *
     * To say it another way, all channels in a group must have the same
     * rate and mode. (See rates above.)
     */


//    uint32_t alt = rate_map & 0;
    uint8_t group = 0;

    if (current_rate > alt_rate) {
        if (up_pwm_servo_set_rate_group_update(group, current_rate) == OK) {
            return OK;
        }
        else if (up_pwm_servo_set_rate_group_update(group, alt_rate) == OK) {
            _current_update_rate = alt_rate;
            return OK;
        }
        else {
            if (up_pwm_servo_set_rate_group_update(group, default_rate) != OK) {
                PX4_WARN("rate group set default_rate");
                return -EINVAL;
            }
            else {
                _current_update_rate = default_rate;
                return OK;
            }
        }
    }
    else {
        if (up_pwm_servo_set_rate_group_update(group, alt_rate) == OK) {
            _current_update_rate = alt_rate;
            return OK;
        }
        else if (up_pwm_servo_set_rate_group_update(group, current_rate) == OK) {
            return OK;
        }
        else {
            if (up_pwm_servo_set_rate_group_update(group, default_rate) != OK) {
                PX4_WARN("rate group set default_rate");
                return -EINVAL;
            }
            else {
                _current_update_rate = default_rate;
                return OK;
            }
        }
    }


//    else if(current_rate <= alt_rate) {
//        if (up_pwm_servo_set_rate_group_update(group, alt_rate) != OK) {
//                    PX4_WARN("rate group set alt failed");
//                    return -EINVAL;
//        }

//    }
//    else {
//        if (up_pwm_servo_set_rate_group_update(group, default_rate) != OK) {
//                PX4_WARN("rate group set default failed");
//                return -EINVAL;
//        }
//    }

//    return OK;
}

bool pwm_mix_out::update_pwm_out_state(bool on)
{
    if(on==true) {
        up_pwm_servo_init(_pwm_mask);

//                PX4_INFO("update_pwm_out_state: on = %d, _pwm_mask = %d, pwm_mask_new = %d", on, _pwm_mask, pwm_mask_new);

        // Set rate is not affecting non-masked channels, so can be called
        // individually

//                set_pwm_rate(get_alt_rate_channels(), get_default_rate(), get_alt_rate());

        set_pwm_rate((unsigned)get_current_rate(), get_default_rate(), get_alt_rate());




        up_pwm_servo_arm(on, _pwm_mask);

//        PX4_INFO("pwm arm: on = %d, _pwm_mask = %d, PWM_OUT_MAX_INSTANCES = %d", on, _pwm_mask, PWM_OUT_MAX_INSTANCES);
    }
    else {
//        up_pwm_servo_deinit(_pwm_mask);   // = up_pwm_servo_arm(false, channel_mask);
//        up_pwm_servo_set_rate(0);
        up_pwm_servo_arm(false, _pwm_mask);
    }

    return OK;
}

void pwm_mix_out::update_current_rate()
{
        /*
        * Adjust actuator topic update rate to keep up with
        * the highest servo update rate configured.
        *
        * We always mix at max rate; some channels may update slower.
        */
        int max_rate = (_pwm_default_rate > _pwm_alt_rate) ? _pwm_default_rate : _pwm_alt_rate;

        // oneshot
        if ((_pwm_default_rate == 0) || (_pwm_alt_rate == 0)) {
                max_rate = 2000;

        } else {
                // run up to twice PWM rate to reduce end-to-end latency
                //  actual pulse width only updated for next period regardless of output module
                max_rate *= 2;
        }

        // max interval 0.5 - 100 ms (10 - 2000Hz)
//        const int update_interval_in_us = math::constrain(1000000 / max_rate, 500, 100000);

        if (_current_update_rate != max_rate) {
//                PX4_INFO("instance: %d, max rate: %d, default: %d, alt: %d", _instance, max_rate, _pwm_default_rate, _pwm_alt_rate);
        }

        _current_update_rate = max_rate;
//        _mixing_output.setMaxTopicUpdateRate(update_interval_in_us);
}

void pwm_mix_out::mix_and_update_outputs()
{
    // 修改为直接在这里写混控器

    /* 0 - MC, 1 - FW */
//  固定翼的控制参数为第0组
//    uint8_t num_outputs = 14;
// TODO： 根据飞机类型作判断，到底应该用第0还是1组

    const float roll0 = _actuator_controls_0.control[actuator_controls_s::INDEX_ROLL];
    const float pitch0 = _actuator_controls_0.control[actuator_controls_s::INDEX_PITCH];
    const float yaw0 = _actuator_controls_0.control[actuator_controls_s::INDEX_YAW];
    const float thr0 = _actuator_controls_0.control[actuator_controls_s::INDEX_THROTTLE];
    const float flap0 = _actuator_controls_0.control[actuator_controls_s::INDEX_FLAPS];

    const float roll1 = _actuator_controls_1.control[actuator_controls_s::INDEX_ROLL];
    const float pitch1 = _actuator_controls_1.control[actuator_controls_s::INDEX_PITCH];
    const float yaw1 = _actuator_controls_1.control[actuator_controls_s::INDEX_YAW];
    const float thr1 = _actuator_controls_1.control[actuator_controls_s::INDEX_THROTTLE];
    const float flap1 = _actuator_controls_1.control[actuator_controls_s::INDEX_FLAPS];

    const float roll6 = _actuator_controls_6.control[actuator_controls_s::INDEX_ROLL];
    const float pitch6 = _actuator_controls_6.control[actuator_controls_s::INDEX_PITCH];
    const float yaw6 = _actuator_controls_6.control[actuator_controls_s::INDEX_YAW];
    const float thr6 = _actuator_controls_6.control[actuator_controls_s::INDEX_THROTTLE];

    const float legacy_roll = roll0;
    const float legacy_pitch = pitch0;
    const float legacy_yaw = yaw0;
    const float legacy_thr = thr0;
    const float legacy_flap = flap0;

    (void)yaw1;
    (void)thr1;
    (void)flap1;
    (void)pitch6;
    (void)thr6;

    // float gear1 = _actuator_controls_0.control[actuator_controls_s::INDEX_LANDING_GEAR];

    /* 根据无人机名称定义对应的混控器并输出 */
    switch (_VEHICLE_ID)
    {
    case vehicle_id_e::MC650 :
    {
        break;
    }

    case vehicle_id_e::FeiLong_tiltrotor :
    {
        break;
    }

    case vehicle_id_e::GuangHuan :
    {
        // // # @output MAIN1 aileron
        // // # @output MAIN2 elevator
        // // # @output MAIN3 throttle
        // // # @output MAIN4 rudder
        // // # @output MAIN5 flaps
        // // # @output MAIN6 gear
        // _actuator_outputs.output[0] = math::gradual3(roll1*57.3f, _pwm_main1_min_x.get(), 0.f, _pwm_main1_max_x.get(), _pwm_main1_min.get(), _pwm_main1_trim.get(), _pwm_main1_max.get());
        // _actuator_outputs.output[1] = math::gradual3(pitch1*57.3f, _pwm_main2_min_x.get(), 0.f, _pwm_main2_max_x.get(), _pwm_main2_min.get(), _pwm_main2_trim.get(), _pwm_main2_max.get());
        // _actuator_outputs.output[2] = math::constrain(1000.f+thr1*1000.f, _pwm_main3_min.get(), _pwm_main3_max.get());
        // _actuator_outputs.output[3] = math::gradual3(yaw1*57.3f, _pwm_main4_min_x.get(), 0.f, _pwm_main4_max_x.get(), _pwm_main4_min.get(), _pwm_main4_trim.get(), _pwm_main4_max.get());
        // _actuator_outputs.output[4] = math::gradual3(flap1*57.3f, _pwm_main5_min_x.get(), 0.f, _pwm_main5_max_x.get(), _pwm_main5_min.get(), _pwm_main5_trim.get(), _pwm_main5_max.get());
        // _actuator_outputs.output[5] = math::gradual3(gear1*57.3f, _pwm_main6_min_x.get(), 0.f, _pwm_main6_max_x.get(), _pwm_main6_min.get(), _pwm_main6_trim.get(), _pwm_main6_max.get());

        // # @output MAIN1 right front dt
        // # @output MAIN2 left behind dt
        // # @output MAIN3 left front dt
        // # @output MAIN4 right behind dt
        // # @output MAIN5 elevon left
        // # @output MAIN6 elevon right
        // # @output MAIN7 add prop left
        // # @output MAIN8 add prop right
        float k_dd = 1.f; //俯仰差动缩放
        float thr_diffp = math::constrain(legacy_pitch, -thr_diff_limit, thr_diff_limit) * k_dd;
        float thr_diffyaw = math::constrain(legacy_yaw, -thr_diff_limit, thr_diff_limit) * k_dd;
        float dt_frontleft  = math::constrain(legacy_thr + 0.5f*thr_diffp+0.5f*thr_diffyaw, 0.01f, param_fw_thr_max);
        float dt_frontright  = math::constrain(legacy_thr + 0.5f*thr_diffp-0.5f*thr_diffyaw, 0.01f, param_fw_thr_max);
        float dt_backleft = math::constrain(legacy_thr - 0.5f*thr_diffp+0.2f*thr_diffyaw, 0.01f, param_fw_thr_max);
        float dt_backright = math::constrain(legacy_thr - 0.5f*thr_diffp-0.2f*thr_diffyaw, 0.01f, param_fw_thr_max);

        _actuator_outputs.output[0] = math::constrain(1000.f+dt_frontright*1000.f, _pwm_main3_min.get(), _pwm_main3_max.get());
        _actuator_outputs.output[1] = math::constrain(1000.f+dt_backleft*1000.f, _pwm_main3_min.get(), _pwm_main3_max.get());
        _actuator_outputs.output[2] = math::constrain(1000.f+dt_frontleft*1000.f, _pwm_main3_min.get(), _pwm_main3_max.get());
        _actuator_outputs.output[3] = math::constrain(1000.f+dt_backright*1000.f, _pwm_main3_min.get(), _pwm_main3_max.get());
        _actuator_outputs.output[4] = math::gradual3(-legacy_roll*57.3f + legacy_pitch*57.3f, _pwm_main1_min_x.get(), 0.f, _pwm_main1_max_x.get(), _pwm_main1_min.get(), _pwm_main1_trim.get(), _pwm_main1_max.get());
        _actuator_outputs.output[5] = math::gradual3(legacy_roll*57.3f + legacy_pitch*57.3f, _pwm_main1_min_x.get(), 0.f, _pwm_main1_max_x.get(), _pwm_main1_min.get(), _pwm_main1_trim.get(), _pwm_main1_max.get());
        // _actuator_outputs.output[4] = math::gradual3(pitch1*57.3f, _pwm_main1_min_x.get(), 0.f, _pwm_main1_max_x.get(), _pwm_main1_min.get(), _pwm_main1_trim.get(), _pwm_main1_max.get());
        // _actuator_outputs.output[5] = math::gradual3(pitch1*57.3f, _pwm_main1_min_x.get(), 0.f, _pwm_main1_max_x.get(), _pwm_main1_min.get(), _pwm_main1_trim.get(), _pwm_main1_max.get());
        _actuator_outputs.output[6] = math::constrain(1300.f+roll6*1000.f,_pwm_main3_min.get(), _pwm_main3_max.get());
        _actuator_outputs.output[7] = math::constrain(1300.f-roll6*1000.f,_pwm_main3_min.get(), _pwm_main3_max.get());
        break;
    }

    case vehicle_id_e::VTOL_X1 :
    {
        break;
    }

    case vehicle_id_e::MYSolar_7m :
    {
        /*
        flap>0对应方向舵张开

        1：左油门
        2：右油门
        3：升降
        4：左方向
        5：右方向
        6：左副翼
        7：右副翼
        */
        //    actuator_outputs_s actuator_outputs{};

        if( _armed_state == true ) {
            // 小油门情况下自动放大油门差动的比例和限幅
            float k_dd = 1.f;
            k_dd = math::gradual(legacy_thr, 0.1f, 0.3f, 1.5f, 1.0f);

            float thr_diff = math::constrain(legacy_pitch, -thr_diff_limit, thr_diff_limit) * k_dd;
            float dt_left  = math::constrain(legacy_thr + 0.5f*thr_diff, param_fw_thr_idle, param_fw_thr_max);
            float dt_right = math::constrain(dt_left - 1.0f*thr_diff, param_fw_thr_idle, param_fw_thr_max);
            // 差动的权限比油门高
            // 如果右油门饱和了，重新分配左油门以确保差动量足够
            if (dt_right <= param_fw_thr_idle || dt_right >= param_fw_thr_max) {
                dt_left = dt_right + 1.0f*thr_diff;
            }

            // TODO: 加入直接侧力控制，继写 _actuators.control[7] = (_att_sp.roll_body * 9.8f) * -0.5f; 	// dr = ayc * Kayc; ayc = phic*9.8; dd = -0.36*dr
    //        float dirct_f = math::constrain(actuator_controls_1_t.control[7],-1.0f,1.0f);
            _actuator_outputs.output[0] = math::constrain(1000.f+dt_left *1000.f, _pwm_main1_min.get(), _pwm_main1_max.get());
            _actuator_outputs.output[1] = math::constrain(1000.f+dt_right*1000.f, _pwm_main2_min.get(), _pwm_main2_max.get());
            _actuator_outputs.output[2] = math::gradual3(legacy_pitch*57.3f*_pitch_scale.get(), _pwm_main3_min_x.get(), 0.f, _pwm_main3_max_x.get(), _pwm_main3_min.get(), _pwm_main3_trim.get(), _pwm_main3_max.get());

            _actuator_outputs.output[3] = math::gradual3(legacy_yaw*57.3f*_yaw_scale.get() -legacy_flap*57.3f, _pwm_main4_min_x.get(), 0.f, _pwm_main4_max_x.get(), _pwm_main4_min.get(), _pwm_main4_trim.get(), _pwm_main4_max.get());
            _actuator_outputs.output[4] = math::gradual3(legacy_yaw*57.3f*_yaw_scale.get() +legacy_flap*57.3f, _pwm_main5_min_x.get(), 0.f, _pwm_main5_max_x.get(), _pwm_main5_min.get(), _pwm_main5_trim.get(), _pwm_main5_max.get());
            _actuator_outputs.output[5] = math::gradual3(legacy_roll*57.3f*_roll_scale.get(), _pwm_main6_min_x.get(), 0.f, _pwm_main6_max_x.get(), _pwm_main6_min.get(), _pwm_main6_trim.get(), _pwm_main6_max.get());
            _actuator_outputs.output[6] = math::gradual3(legacy_roll*57.3f*_roll_scale.get(), _pwm_main7_min_x.get(), 0.f, _pwm_main7_max_x.get(), _pwm_main7_min.get(), _pwm_main7_trim.get(), _pwm_main7_max.get());
        }
        else {
            _actuator_outputs.output[0] = 900.0f;
            _actuator_outputs.output[1] = 900.0f;
            _actuator_outputs.output[2] = _pwm_main3_trim.get();
            _actuator_outputs.output[3] = _pwm_main4_trim.get();
            _actuator_outputs.output[4] = _pwm_main5_trim.get();
            _actuator_outputs.output[5] = _pwm_main6_trim.get();
            _actuator_outputs.output[6] = _pwm_main7_trim.get();
        }

        break;
    }

    case vehicle_id_e::hiland270 :
    {
        /*
        1-8：从左到右的8个油门，分为四组：12、34、56、78，12和78可以差动控制航向
        9： 副翼
        10：升降
        11：方向
        */
        //    actuator_outputs_s actuator_outputs{};

        if( _armed_state == true ) {
            // 小油门情况下自动放大油门差动的比例和限幅
            float k_dd = 1.f;
//            k_dd = math::gradual(thr1, 0.1f, 0.3f, 1.5f, 1.0f);

            _actuator_dsc_sub.update(&_actuator_controls_dsc);

            //                actuators_dsc.control[0] = 0.059f * ds; // da
//                            actuators_dsc.control[2] = ds;      // dr
            //                actuators_dsc.control[3] = -1.07f * ds;      // diff throttle, +left, -right

            // 允许使用差动油门才能使用直接侧力控制
            float ds_dr = (fabsf(thr_diff_limit)>=0.01f) ? _actuator_controls_dsc.control[2] : 0.f;
            float ds_dd = ds_dr * -1.07f;
            float ds_da = ds_dr * 0.059f;

            float thr_diff = math::constrain(ds_dd, -thr_diff_limit, thr_diff_limit) * k_dd;
            float dt_left  = math::constrain(legacy_thr + 0.5f*thr_diff, param_fw_thr_idle, param_fw_thr_max);
            float dt_right = math::constrain(dt_left - 1.0f*thr_diff, param_fw_thr_idle, param_fw_thr_max);
            // 差动的权限比油门高
            // 如果右油门饱和了，重新分配左油门以确保差动量足够
            if (dt_right <= param_fw_thr_idle || dt_right >= param_fw_thr_max) {
                dt_left = dt_right + 1.0f*thr_diff;
            }

            // TODO: 加入直接侧力控制，继写 _actuators.control[7] = (_att_sp.roll_body * 9.8f) * -0.5f; 	// dr = ayc * Kayc; ayc = phic*9.8; dd = -0.36*dr
    //        float dirct_f = math::constrain(actuator_controls_1_t.control[7],-1.0f,1.0f);
            _actuator_outputs.output[0] = math::constrain(1000.f+dt_left *1000.f, _pwm_main1_min.get(), _pwm_main1_max.get());
            _actuator_outputs.output[1] = math::constrain(1000.f+dt_left*1000.f, _pwm_main2_min.get(), _pwm_main2_max.get());
            _actuator_outputs.output[2] = math::constrain(1000.f+legacy_thr*1000.f, _pwm_main3_min.get(), _pwm_main3_max.get());
            _actuator_outputs.output[3] = math::constrain(1000.f+legacy_thr*1000.f, _pwm_main4_min.get(), _pwm_main4_max.get());
            _actuator_outputs.output[4] = math::constrain(1000.f+legacy_thr*1000.f, _pwm_main5_min.get(), _pwm_main5_max.get());
            _actuator_outputs.output[5] = math::constrain(1000.f+legacy_thr*1000.f, _pwm_main6_min.get(), _pwm_main6_max.get());
            _actuator_outputs.output[6] = math::constrain(1000.f+dt_right*1000.f, _pwm_main7_min.get(), _pwm_main7_max.get());
            _actuator_outputs.output[7] = math::constrain(1000.f+dt_right*1000.f, _pwm_main8_min.get(), _pwm_main8_max.get());

            _actuator_outputs.output[8] = math::gradual3((legacy_roll+ds_da)*57.3f*_roll_scale.get(), _pwm_aux1_min_x.get(), 0.f, _pwm_aux1_max_x.get(), _pwm_aux1_min.get(), _pwm_aux1_trim.get(), _pwm_aux1_max.get());
            _actuator_outputs.output[9] = math::gradual3(legacy_pitch*57.3f*_pitch_scale.get(), _pwm_aux2_min_x.get(), 0.f, _pwm_aux2_max_x.get(), _pwm_aux2_min.get(), _pwm_aux2_trim.get(), _pwm_aux2_max.get());
            _actuator_outputs.output[10] = math::gradual3((legacy_yaw+ds_dr)*57.3f*_yaw_scale.get(), _pwm_aux3_min_x.get(), 0.f, _pwm_aux3_max_x.get(), _pwm_aux3_min.get(), _pwm_aux3_trim.get(), _pwm_aux3_max.get());

//            if( _count % 100 == 0 ) {
//                PX4_INFO("dsc: da dr dd = %4.2f, %4.2f, %4.2f", (double)ds_da*57.3, (double)ds_dr*57.3, (double)ds_dd);
//            }
        }
        else {
            _actuator_outputs.output[0] = 900.0f;
            _actuator_outputs.output[1] = 900.0f;
            _actuator_outputs.output[2] = 900.0f;
            _actuator_outputs.output[3] = 900.0f;
            _actuator_outputs.output[4] = 900.0f;
            _actuator_outputs.output[5] = 900.0f;
            _actuator_outputs.output[6] = 900.0f;
            _actuator_outputs.output[7] = 900.0f;

            _actuator_outputs.output[8] = 1500.0f;
            _actuator_outputs.output[9] = 1500.0f;
            _actuator_outputs.output[10] = 1500.0f;
        }

        break;
    }


    case vehicle_id_e::VTOL_X2 :
    case vehicle_id_e::XWing :
    case vehicle_id_e::x1mini :
    {
    /*
    Legacy tailsitter mapping for non-Tandem variants.
    */

    if (_armed_state == true) {
        // -- 偏航油门差动 --
        float thr_diff = math::constrain(legacy_yaw, -thr_diff_limit, thr_diff_limit);
        float dt_left  = math::constrain(legacy_thr + 0.5f * thr_diff, param_fw_thr_idle, param_fw_thr_max);
        float dt_right = math::constrain(legacy_thr - 0.5f * thr_diff, param_fw_thr_idle, param_fw_thr_max);

        if (dt_right <= param_fw_thr_idle || dt_right >= param_fw_thr_max) {
            dt_left = dt_right + 1.0f * thr_diff;
        }

        _actuator_outputs.output[0] = math::constrain(1000.f + dt_right * 1000.f,
            _pwm_main1_min.get(), _pwm_main1_max.get());
        _actuator_outputs.output[1] = math::constrain(1000.f + dt_left  * 1000.f,
            _pwm_main2_min.get(), _pwm_main2_max.get());
        _actuator_outputs.output[2] = math::constrain(1000.f + dt_left  * 1000.f,
            _pwm_main3_min.get(), _pwm_main3_max.get());
        _actuator_outputs.output[3] = math::constrain(1000.f + dt_right * 1000.f,
            _pwm_main4_min.get(), _pwm_main4_max.get());

        _actuator_outputs.output[4] = math::gradual3(
            (-legacy_roll + legacy_pitch) * 57.3f * _pitch_scale.get(),
            _pwm_main5_min_x.get(), 0.f, _pwm_main5_max_x.get(),
            _pwm_main5_min.get(), _pwm_main5_trim.get(), _pwm_main5_max.get());
        _actuator_outputs.output[5] = math::gradual3(
            ( legacy_roll + legacy_pitch) * 57.3f * _pitch_scale.get(),
            _pwm_main6_min_x.get(), 0.f, _pwm_main6_max_x.get(),
            _pwm_main6_min.get(), _pwm_main6_trim.get(), _pwm_main6_max.get());

        _actuator_outputs.output[6] = _pwm_main7_min.get();
        _actuator_outputs.output[7] = _pwm_main8_min.get();

    } else {
        _actuator_outputs.output[0] = 900.0f;
        _actuator_outputs.output[1] = 900.0f;
        _actuator_outputs.output[2] = 900.0f;
        _actuator_outputs.output[3] = 900.0f;
        _actuator_outputs.output[4] = _pwm_main5_trim.get();
        _actuator_outputs.output[5] = _pwm_main6_trim.get();
        _actuator_outputs.output[6] = 900.0f;
        _actuator_outputs.output[7] = 900.0f;
    }

    break;
    }

    case vehicle_id_e::TandemTailSitter :
    {
    /*
    Tandem tailsitter actuator allocation:
    Fixed-wing: MAIN1/3 front and MAIN2/4 rear thrust with low-airspeed pitch differential,
    plus left/right yaw differential. Fixed-wing roll is intentionally ignored by MAIN1-4.
    Rotary-wing: MAIN1-4 use roll0/pitch0/thr0 only; MAIN7-8 use yaw6 only.
    */

    if (_armed_state == true) {
        const bool fixed_wing_mode = !_vtol_vehicle_status.vtol_in_rw_mode && !_vtol_vehicle_status.vtol_in_trans_mode;
        const float airspeed = PX4_ISFINITE(_airspeed_validated.true_airspeed_m_s) ?
            _airspeed_validated.true_airspeed_m_s : 0.0f;
        float pitch_motor_diff = 0.0f;
        float thr_diff = 0.0f;

        if (fixed_wing_mode) {
                const float aspd_start = _fw_pitch_motor_diff_aspd_start.get();
                const float aspd_full = _fw_pitch_motor_diff_aspd_full.get();

                const float aspd_span = math::max(fabsf(aspd_start - aspd_full), 0.1f);

                const float low_aspd_weight = (aspd_start >= aspd_full) ?
                    math::constrain((aspd_start - airspeed) / aspd_span, 0.0f, 1.0f) :
                    math::constrain((aspd_full - airspeed) / aspd_span, 0.0f, 1.0f);


                pitch_motor_diff = math::constrain(
                    pitch1 * _fw_pitch_motor_diff_gain.get() * low_aspd_weight,
                    -thr_diff_limit,
                    thr_diff_limit);


                // yaw thrust differential: left group (MAIN2+3) vs right group (MAIN1+4)
                thr_diff = math::constrain(yaw1, -thr_diff_limit, thr_diff_limit);
                float dt_right = math::constrain(thr0 + 0.5f * thr_diff, param_fw_thr_idle, param_fw_thr_max);
                float dt_left  = math::constrain(thr0 - 0.5f * thr_diff, param_fw_thr_idle, param_fw_thr_max);

                // prevent differential loss when one side saturates
                if (dt_right <= param_fw_thr_idle || dt_right >= param_fw_thr_max) {
                    dt_left = dt_right + 1.0f * thr_diff;
                }

                // MAIN1 right-front: front + right group
                const float right_front = math::constrain(dt_right - pitch_motor_diff, param_fw_thr_idle, param_fw_thr_max);
                // MAIN3 left-front: front + left group
                const float left_front  = math::constrain(dt_left  - pitch_motor_diff, param_fw_thr_idle, param_fw_thr_max);
                // MAIN2 left-rear: rear + left group
                const float left_rear   = math::constrain(dt_left  + pitch_motor_diff, param_fw_thr_idle, param_fw_thr_max);
                // MAIN4 right-rear: rear + right group
                const float right_rear  = math::constrain(dt_right + pitch_motor_diff, param_fw_thr_idle, param_fw_thr_max);

                _actuator_outputs.output[0] = math::constrain(1000.f + right_front * 1000.f,
                    _pwm_main1_min.get(), _pwm_main1_max.get());
                _actuator_outputs.output[2] = math::constrain(1000.f + left_front  * 1000.f,
                    _pwm_main3_min.get(), _pwm_main3_max.get());
                _actuator_outputs.output[1] = math::constrain(1000.f + left_rear   * 1000.f,
                    _pwm_main2_min.get(), _pwm_main2_max.get());
                _actuator_outputs.output[3] = math::constrain(1000.f + right_rear  * 1000.f,
                    _pwm_main4_min.get(), _pwm_main4_max.get());

                const float left_elevon = pitch1 + roll1;
                const float right_elevon = pitch1 - roll1;

                _actuator_outputs.output[4] =
                    math::gradual3(left_elevon * 57.3f * _pitch_scale.get(),
                        _pwm_main5_min_x.get(), 0.f, _pwm_main5_max_x.get(),
                        _pwm_main5_min.get(), _pwm_main5_trim.get(), _pwm_main5_max.get());
                _actuator_outputs.output[5] =
                    math::gradual3(right_elevon * 57.3f * _pitch_scale.get(),
                        _pwm_main6_min_x.get(), 0.f, _pwm_main6_max_x.get(),
                        _pwm_main6_min.get(), _pwm_main6_trim.get(), _pwm_main6_max.get());

                _actuator_outputs.output[6] = _pwm_main7_min.get();
                _actuator_outputs.output[7] = _pwm_main8_min.get();

        } else {
            /*
             * 原13020 mixer: R: 4x 使用 quad-X 布局 (0.707107 系数).
             * 修改点: Tandem 不是 X quad, 而是矩形 Tandem 布局.
             * pitch0 控制前后差动, roll0 控制左右差动, yaw0 不用于 MAIN1-4 (由 MAIN7-8 的 yaw6 处理).
             */
            const float motor1 = math::constrain(thr0 + pitch0 + roll0, 0.0f, 1.0f);  // MAIN1 right-front
            const float motor2 = math::constrain(thr0 - pitch0 - roll0, 0.0f, 1.0f);  // MAIN2 left-rear
            const float motor3 = math::constrain(thr0 + pitch0 - roll0, 0.0f, 1.0f);  // MAIN3 left-front
            const float motor4 = math::constrain(thr0 - pitch0 + roll0, 0.0f, 1.0f);  // MAIN4 right-rear

            _actuator_outputs.output[0] = math::constrain(1000.f + motor1 * 1000.f,
                _pwm_main1_min.get(), _pwm_main1_max.get());
            _actuator_outputs.output[1] = math::constrain(1000.f + motor2 * 1000.f,
                _pwm_main2_min.get(), _pwm_main2_max.get());
            _actuator_outputs.output[2] = math::constrain(1000.f + motor3 * 1000.f,
                _pwm_main3_min.get(), _pwm_main3_max.get());
            _actuator_outputs.output[3] = math::constrain(1000.f + motor4 * 1000.f,
                _pwm_main4_min.get(), _pwm_main4_max.get());

            _actuator_outputs.output[4] = _pwm_main5_trim.get();
            _actuator_outputs.output[5] = _pwm_main6_trim.get();

            const float tip_idle = 1300.0f;
            const float yaw_gain = 1000.0f * _yaw_scale.get();

            _actuator_outputs.output[6] = math::constrain(tip_idle + yaw_gain * yaw6,
                _pwm_main7_min.get(), _pwm_main7_max.get());
            _actuator_outputs.output[7] = math::constrain(tip_idle - yaw_gain * yaw6,
                _pwm_main8_min.get(), _pwm_main8_max.get());
        }

        if (_count % 100 == 0) {
            PX4_INFO("Tandem mix mode=%s aspd=%4.1f thr0=%4.2f r1=%4.2f p1=%4.2f y1=%4.2f y6=%4.2f pmd=%4.2f td=%4.2f "
                "out=%4.0f,%4.0f,%4.0f,%4.0f,%4.0f,%4.0f,%4.0f,%4.0f",
                fixed_wing_mode ? "FW" : "MC",
                (double)airspeed, (double)thr0, (double)roll0, (double)pitch0, (double)yaw0, (double)yaw6,
                (double)pitch_motor_diff, (double)thr_diff,
                (double)_actuator_outputs.output[0], (double)_actuator_outputs.output[1],
                (double)_actuator_outputs.output[2], (double)_actuator_outputs.output[3],
                (double)_actuator_outputs.output[4], (double)_actuator_outputs.output[5],
                (double)_actuator_outputs.output[6], (double)_actuator_outputs.output[7]);
        }

    } else {
        // 未解锁 → 电机停转，舵面回中
        _actuator_outputs.output[0] = 900.0f;
        _actuator_outputs.output[1] = 900.0f;
        _actuator_outputs.output[2] = 900.0f;
        _actuator_outputs.output[3] = 900.0f;
        _actuator_outputs.output[4] = _pwm_main5_trim.get();
        _actuator_outputs.output[5] = _pwm_main6_trim.get();
        _actuator_outputs.output[6] = 900.0f;
        _actuator_outputs.output[7] = 900.0f;
    }

    break;
    }
    case vehicle_id_e::TEST :
    default:
        break;
    }
    // =================================================================
    // Throttle Kill: force motor outputs to disarmed value (900 us)
    // when armed but throttle is killed. Control surface channels
    // are NOT affected, preserving attitude control.
    // =================================================================
    if (_throttle_killed && _armed_state) {
        switch (_VEHICLE_ID) {
        case vehicle_id_e::GuangHuan:
            // Motors: MAIN1-4 (0-3), add props MAIN7-8 (6-7)
            // Servos: elevons MAIN5-6 (4-5) -- NOT killed
            _actuator_outputs.output[0] = 900.0f;
            _actuator_outputs.output[1] = 900.0f;
            _actuator_outputs.output[2] = 900.0f;
            _actuator_outputs.output[3] = 900.0f;
            _actuator_outputs.output[6] = 900.0f;
            _actuator_outputs.output[7] = 900.0f;
            break;

        case vehicle_id_e::MYSolar_7m:
            // Motors: left/right throttle MAIN1-2 (0-1)
            // Servos: elevator/rudder/ailerons MAIN3-7 (2-6) -- NOT killed
            _actuator_outputs.output[0] = 900.0f;
            _actuator_outputs.output[1] = 900.0f;
            break;

        case vehicle_id_e::hiland270:
            // Motors: MAIN1-8 (0-7)
            // Servos: aileron/elevator/rudder AUX1-3 (8-10) -- NOT killed
            for (int i = 0; i < 8; i++) {
                _actuator_outputs.output[i] = 900.0f;
            }
            break;

        case vehicle_id_e::VTOL_X2:
        case vehicle_id_e::XWing:
        case vehicle_id_e::x1mini:
        case vehicle_id_e::TandemTailSitter:
            // Motors: MAIN1-4 (0-3), wingtip MAIN7-8 (6-7)
            // Servos: elevons MAIN5-6 (4-5) -- NOT killed
            _actuator_outputs.output[0] = 900.0f;
            _actuator_outputs.output[1] = 900.0f;
            _actuator_outputs.output[2] = 900.0f;
            _actuator_outputs.output[3] = 900.0f;
            _actuator_outputs.output[6] = 900.0f;
            _actuator_outputs.output[7] = 900.0f;
            break;

        case vehicle_id_e::FeiLong_tiltrotor:
        case vehicle_id_e::MC650:
        case vehicle_id_e::VTOL_X1:
        case vehicle_id_e::TEST:
        default:
            // Placeholder/test vehicles: safe default, kill all MAIN outputs
            for (int i = 0; i < 8; i++) {
                _actuator_outputs.output[i] = 900.0f;
            }
            break;
        }
    }




//    _actuator_outputs.noutputs = 7;
    _actuator_outputs.noutputs = _num_outputs;


    // publish to mavlink
    _actuator_outputs.timestamp = hrt_absolute_time();
    _actuator_outputs_pub.publish(_actuator_outputs);

    // write to registor for pwm outputs
    for (size_t i = 0; i < _num_outputs; i++) {
            up_pwm_servo_set(_output_base + i, (uint16_t)_actuator_outputs.output[i]);
    }
    up_pwm_update();


//        /* output to the servos */
////        if (_pwm_initialized)
//        {
//                for (size_t i = 0; i < _num_outputs; i++) {
//                        up_pwm_servo_set(_output_base + i, outputs[i]);
//                }
//        }

//        /* Trigger all timer's channels in Oneshot mode to fire
//         * the oneshots with updated values.
//         */
//        if (num_control_groups_updated > 0) {
//                up_pwm_update(); // TODO: review for multi
//        }

//        if( _count %400 == 0 ) {
//            PX4_INFO("_output_base = %d, outputs[0] = %d", _output_base, outputs[0]);
//        }
//        _count++;

}

void pwm_mix_out::Run()
{
    if (should_exit()) {
            ScheduleClear();
//            _control_subs.unregister();

            //exit_and_cleanup();
            return;
    }

    perf_begin(_cycle_perf);
    perf_count(_interval_perf);

    // push backup schedule
    ScheduleDelayed(_backup_schedule_interval_us);

//    if( _count %200 == 0 ) {
////        PX4_INFO("time elapsed (usec) = %d", (int)(hrt_absolute_time() - _time_last_update));
//        PX4_INFO("time elapsed (usec) = %d, pwm_up_rate (hz) = %d", (int)(hrt_absolute_time() - _time_last_update), get_current_rate());
//    }
//    _time_last_update = hrt_absolute_time();


    // check for parameter updates
    if (_parameter_update_sub.updated()) {
            // clear update
            parameter_update_s pupdate;
            _parameter_update_sub.copy(&pupdate);

            // update parameters from storage
             update_params(); // do not update PWM params for now (was interfering with VTOL PWM settings)
    }


    _actuator_armed_sub.update(&_actuator_armed);
    if( _actuator_armed.armed != _armed_state_old )
    {
        _armed_state = _actuator_armed.armed;
        _armed_state_old = _armed_state;

        if( _armed_state == true ) {
            up_pwm_servo_arm(_armed_state, _pwm_mask);
        }

        if( _armed_state == false ) {
            up_pwm_servo_arm(true, _pwm_mask);  // 经验证，arm先true再false, disarm时才不会打死
//            up_pwm_servo_arm(_armed_state, 0);  // 不是 0 而是 _pwm_mask 的话disarm时会打死？
//            up_pwm_update();    // 不update就不会下电？
//            return;
        }
    }

 /* 现象记录： 不同的else的设置
    1. 以下情况在disarm时舵会打死，或者刚上电时舵会乱打
        up_pwm_servo_arm(false,0)
    2. 以下情况在disarm时舵会打死，但是刚上电时舵稳定不动。arm久了会偶尔抽风
        up_pwm_servo_arm(false, 0);
        up_pwm_update();
        return;
    3.以下情况在刚上电时舵会乱打，但是disarm时稳定，arm久了也不抽风
        up_pwm_servo_arm(true, 0);
        up_pwm_servo_arm(false, 0);
        up_pwm_update();
        return;
    4.以下情况在刚上电时舵偶尔会乱打，disarm时打死，arm久了不抽风
        up_pwm_servo_arm(false, _pwm_mask);
        up_pwm_update();
        return;
    5.以下情况在刚上电时舵会乱打，但是disarm时稳定，arm久了也不抽风
        up_pwm_servo_arm(true, _pwm_mask);
        up_pwm_servo_arm(false, 0);
        up_pwm_update();
        return;
    6.以下情况在刚上电时稳定，但是disarm时打死，arm久了也不抽风
        if( _actuator_armed.armed != _armed_state_old )
        {
            _armed_state = _actuator_armed.armed;
            _armed_state_old = _armed_state;
            up_pwm_servo_arm(_armed_state, _pwm_mask);

            if( _armed_state == false ) {
                up_pwm_servo_arm(_armed_state, 0);  // 不是 0 而是 _pwm_mask 的话disarm时会打死？
                up_pwm_update();    // 不update就不会下电？
                return;
            }
        }
    7.很偶尔在上电时舵乱打，其余一切正常
        if( _armed_state == true ) {
            up_pwm_servo_arm(_armed_state, _pwm_mask);
        }
        if( _armed_state == false ) {
            up_pwm_servo_arm(true, 0);
            up_pwm_servo_arm(_armed_state, 0);  // 不是 0 而是 _pwm_mask 的话disarm时会打死？
            up_pwm_update();    // 不update就不会下电？
            return;
        }
    8.一切正常，暂时还没发现上电时舵乱打
        if( _armed_state == true ) {
            up_pwm_servo_arm(_armed_state, _pwm_mask);
        }
        if( _armed_state == false ) {
            up_pwm_servo_arm(true, _pwm_mask);
            up_pwm_servo_arm(_armed_state, 0);  // 不是 0 而是 _pwm_mask 的话disarm时会打死？
            up_pwm_update();    // 不update就不会下电？
            return;
        }

*/

//    if( _armed_state == true ) {
//        up_pwm_servo_arm(_armed_state, _pwm_mask);
//    }
//    else {
//        up_pwm_servo_arm(true, _pwm_mask);
//        up_pwm_servo_arm(false, 0);
//        up_pwm_update();
//        return;
//    }

    _actuator_controls_0_sub.update(&_actuator_controls_0);
    _actuator_controls_1_sub.update(&_actuator_controls_1);
    _actuator_controls_6_sub.update(&_actuator_controls_6);
    _airspeed_validated_sub.update(&_airspeed_validated);
    _vtol_vehicle_status_sub.update(&_vtol_vehicle_status);
    // Check throttle kill from uORB topic and parameter
    _throttle_kill_sub.update(&_throttle_kill);
    bool kill_requested = _throttle_kill.kill || (_thr_kill.get() == 1);

    // If auto-restored, ignore persistent kill signal until explicitly cleared
    if (_throttle_kill_auto_restored) {
        if (!kill_requested) {
            // Signal cleared: ready for next trigger
            _throttle_kill_auto_restored = false;
        }

    } else if (kill_requested && !_throttle_killed) {
        // Rising edge: start timer
        _throttle_kill_start_time = hrt_absolute_time();
        _throttle_killed = true;

    } else if (kill_requested && _throttle_killed) {
        // Already killed: check auto-restore timeout
        float timeout_s = _thr_kill_t.get();

        if (timeout_s > 0.0f) {
            hrt_abstime elapsed = hrt_absolute_time() - _throttle_kill_start_time;

            if (elapsed > (hrt_abstime)(timeout_s * 1_s)) {
                _throttle_killed = false;
                _throttle_kill_auto_restored = true;
            }
        }

    } else {
        // Kill released externally
        _throttle_killed = false;
    }


    /*
     * ======================混控器工作流程=========================================
    -》src/drivers/pwm_out/PWMOut.cpp里的main是入口。它的Run()中有_mixing_output.update();
    -》_mixing_output是MixingOutput的实例，MixingOutput是src/lib/mixer/mixer_module.cpp中定义的一个类
    -》bool MixingOutput::update()这个函数里调用了混控器 _mixers->mix(outputs, _max_num_outputs);
    -》_mixers是MixerGroup的私有变量，是src/lib/mixer/MixerBase/Mixer.cpp的实例
    -》MixerGroup是src/lib/mixer/MixerGroup.cpp里的类，用于根据文本构造混控器
    -》MixingOutput::update()里还调用了_interface.updateOutputs()，又回到这个类的updateOutputs()

    */

//        _mixing_output.update();

//        /* update PWM status if armed or if disarmed PWM values are set */
//        bool pwm_on = _mixing_output.armed().armed || (_num_disarmed_set > 0) || _mixing_output.armed().in_esc_calibration_mode;

//        if (_pwm_on != pwm_on) {

//                if (update_pwm_out_state(pwm_on)) {
//                        _pwm_on = pwm_on;
//                }
//        }

//    if( _count % 100 == 0 ) {
//        PX4_INFO("VEHICLE_ID = %d", _VEHICLE_ID);
//    }

    mix_and_update_outputs();

    if (_current_update_rate == 0) {
            update_current_rate();
    }

    // check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
//        _mixing_output.updateSubscriptions(true, true);

    perf_end(_cycle_perf);

    _count++;


//    PX4_INFO("pwm_mix_out::Run()");
}

int pwm_mix_out::custom_command(int argc, char *argv[])
{
        return print_usage("unknown command");
}

int pwm_mix_out::print_usage(const char *reason)
{
    return OK;
}

int pwm_mix_out::print_status()
{
    return 0;
}

int pwm_mix_out::task_spawn(int argc, char *argv[])
{
        pwm_mix_out *instance = new pwm_mix_out();

        if (instance) {
                _object.store(instance);
                _task_id = task_id_is_work_queue;

                if (instance->init()) {
                        return PX4_OK;
                }

        } else {
                PX4_ERR("alloc failed");
        }

        delete instance;
        _object.store(nullptr);
        _task_id = -1;

        return PX4_ERROR;
}

extern "C" __EXPORT int pwm_mix_out_main(int argc, char *argv[])
{
    return pwm_mix_out::main(argc, argv);
}
