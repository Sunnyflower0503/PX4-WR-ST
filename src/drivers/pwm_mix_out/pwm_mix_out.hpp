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

#pragma once

#include <float.h>
#include <math.h>

#include <board_config.h>
#include <drivers/device/device.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_input_capture.h>
#include <drivers/drv_mixer.h>
#include <drivers/drv_pwm_output.h>
#include <lib/cdev/CDev.hpp>
#include <lib/mathlib/mathlib.h>
//#include <lib/mixer_module/mixer_module.hpp>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/throttle_kill.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
//#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>


typedef enum
{
    MC650 				= 0,	// 4 rotors copter, diameter 650mm, 3kg
    FeiLong_tiltrotor	= 1,	// VTOL, tilt rotor, v-tail, 6kg
    GuangHuan			= 2,	// standard fixed-wing, 2kg
    VTOL_X1				= 3,	// VTOL, 2kg
    MYSolar_7m			= 4,		// MeiYing solar-powered uav, span = 7m, 16kg
    VTOL_X2				= 5,		// 18 ducts, 14kg
    XWing				= 6,		// 18 ducts, 14kg
    x1mini				= 7,		// 3kg, 8 propellers, vtol
    hiland270			= 8,		// 270kg, 8 propellers, standard fw
    TEST				= 9,		// test
    TandemTailSitter    = 10        // 生态无人机
} vehicle_id_e;


using namespace time_literals;



//class pwm_mix_out : public cdev::CDev, public OutputModuleInterface
class pwm_mix_out final : public ModuleBase<pwm_mix_out>, public ModuleParams, public px4::ScheduledWorkItem
{
public:

    explicit pwm_mix_out();
    ~pwm_mix_out() override;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase::run() */
    void Run() override;

    /** @see ModuleBase::print_status() */
    int print_status() override;

    bool init();

    uint32_t	get_alt_rate_channels() { return _pwm_alt_rate_channels; }
    unsigned	get_alt_rate() { return _pwm_alt_rate; }
    unsigned	get_default_rate() { return _pwm_default_rate; }
    int         get_current_rate() { return _current_update_rate; }

    bool should_exit() const { return _task_should_exit.load(); }
    void request_stop() { _task_should_exit.store(true); }

    void mix_and_update_outputs();

private:
    px4::atomic_bool _task_should_exit{false};

    const uint32_t _output_base{0};
    uint32_t _output_mask{0};

//        static const int MAX_PER_INSTANCE{8};
    static const int MAX_PER_INSTANCE{14};
    unsigned	_pwm_default_rate{50};
    unsigned	_pwm_alt_rate{100};
    int         _current_update_rate{0};

    uint32_t	_pwm_alt_rate_channels{0};

    uint32_t	_backup_schedule_interval_us{1_s};



    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
    uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
    uORB::Subscription _actuator_controls_0_sub{ORB_ID(actuator_controls_0)};
    uORB::Subscription _actuator_controls_1_sub{ORB_ID(actuator_controls_1)};
    uORB::Subscription _actuator_controls_6_sub{ORB_ID(actuator_controls_6)};
    uORB::Subscription _vtol_vehicle_status_sub{ORB_ID(vtol_vehicle_status)};
    uORB::SubscriptionCallbackWorkItem _control_subs[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS] {
                {this, ORB_ID(actuator_controls_0), 0},
                {this, ORB_ID(actuator_controls_1), 1},
                {this, ORB_ID(actuator_controls_2), 2},
                {this, ORB_ID(actuator_controls_3), 3},
                {this, ORB_ID(actuator_controls_4), 4},
                {this, ORB_ID(actuator_controls_5), 5},
                {this, ORB_ID(actuator_controls_6), 6}
    };

    uORB::Subscription _actuator_dsc_sub{ORB_ID(actuator_controls_dsc)};
    uORB::Subscription _throttle_kill_sub{ORB_ID(throttle_kill)};


    uORB::PublicationMulti<actuator_outputs_s> _actuator_outputs_pub{ORB_ID(actuator_outputs)};

    unsigned	_num_outputs{14};
    int		_class_instance{-1};

    bool		_pwm_on{false};
    uint32_t	_pwm_mask{0};
    bool		_pwm_initialized{false};
    bool		_test_mode{false};

    unsigned	_num_disarmed_set{0};


//        void		capture_callback(uint32_t chan_index,
//                                         hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow);
    void		update_current_rate();
    int			set_pwm_rate(unsigned rate_map, unsigned default_rate, unsigned alt_rate);
//        int			pwm_ioctl(file *filp, int cmd, unsigned long arg);

    bool		update_pwm_out_state(bool on);

    void		update_params();

//        static void		sensor_reset(int ms);
//        static void		peripheral_reset(int ms);

//        int		capture_ioctl(file *filp, int cmd, unsigned long arg);

//        pwm_mix_out(const pwm_mix_out &) = delete;
//        pwm_mix_out operator=(const pwm_mix_out &) = delete;

    actuator_armed_s _actuator_armed{0};
    bool _armed_state{0};
    bool _armed_state_old{0};

    actuator_outputs_s _actuator_outputs{};
    actuator_controls_s _actuator_controls_0{};
    actuator_controls_s _actuator_controls_1{};
    actuator_controls_s _actuator_controls_6{};
    vtol_vehicle_status_s _vtol_vehicle_status{};

    actuator_controls_s _actuator_controls_dsc{};

    throttle_kill_s _throttle_kill{};
    bool _throttle_killed{false};
    bool _throttle_kill_auto_restored{false};
    hrt_abstime _throttle_kill_start_time{0};



    perf_counter_t	_cycle_perf;
    perf_counter_t	_interval_perf;

    hrt_abstime _time_last_update{0};


    uint16_t _count{0};


    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::MIXER_ROLL_SC>) _roll_scale,
        (ParamFloat<px4::params::MIXER_PITCH_SC>) _pitch_scale,
        (ParamFloat<px4::params::MIXER_YAW_SC>) _yaw_scale,
        (ParamFloat<px4::params::MIXER_WH_SC>) _wh_scale,

        (ParamFloat<px4::params::PWM_MAIN1_MAX>) _pwm_main1_max,
        (ParamFloat<px4::params::PWM_MAIN2_MAX>) _pwm_main2_max,
        (ParamFloat<px4::params::PWM_MAIN3_MAX>) _pwm_main3_max,
        (ParamFloat<px4::params::PWM_MAIN4_MAX>) _pwm_main4_max,
        (ParamFloat<px4::params::PWM_MAIN5_MAX>) _pwm_main5_max,
        (ParamFloat<px4::params::PWM_MAIN6_MAX>) _pwm_main6_max,
        (ParamFloat<px4::params::PWM_MAIN7_MAX>) _pwm_main7_max,
        (ParamFloat<px4::params::PWM_MAIN8_MAX>) _pwm_main8_max,
        (ParamFloat<px4::params::PWM_AUX1_MAX>)  _pwm_aux1_max,
        (ParamFloat<px4::params::PWM_AUX2_MAX>)  _pwm_aux2_max,
        (ParamFloat<px4::params::PWM_AUX3_MAX>)  _pwm_aux3_max,
        (ParamFloat<px4::params::PWM_AUX4_MAX>)  _pwm_aux4_max,
        (ParamFloat<px4::params::PWM_AUX5_MAX>)  _pwm_aux5_max,
        (ParamFloat<px4::params::PWM_AUX6_MAX>)  _pwm_aux6_max,
        (ParamFloat<px4::params::PWM_AUX7_MAX>)  _pwm_aux7_max,
        (ParamFloat<px4::params::PWM_AUX8_MAX>)  _pwm_aux8_max,

        (ParamFloat<px4::params::PWM_MAIN1_MIN>) _pwm_main1_min,
        (ParamFloat<px4::params::PWM_MAIN2_MIN>) _pwm_main2_min,
        (ParamFloat<px4::params::PWM_MAIN3_MIN>) _pwm_main3_min,
        (ParamFloat<px4::params::PWM_MAIN4_MIN>) _pwm_main4_min,
        (ParamFloat<px4::params::PWM_MAIN5_MIN>) _pwm_main5_min,
        (ParamFloat<px4::params::PWM_MAIN6_MIN>) _pwm_main6_min,
        (ParamFloat<px4::params::PWM_MAIN7_MIN>) _pwm_main7_min,
        (ParamFloat<px4::params::PWM_MAIN8_MIN>) _pwm_main8_min,
        (ParamFloat<px4::params::PWM_AUX1_MIN>)  _pwm_aux1_min,
        (ParamFloat<px4::params::PWM_AUX2_MIN>)  _pwm_aux2_min,
        (ParamFloat<px4::params::PWM_AUX3_MIN>)  _pwm_aux3_min,
        (ParamFloat<px4::params::PWM_AUX4_MIN>)  _pwm_aux4_min,
        (ParamFloat<px4::params::PWM_AUX5_MIN>)  _pwm_aux5_min,
        (ParamFloat<px4::params::PWM_AUX6_MIN>)  _pwm_aux6_min,
        (ParamFloat<px4::params::PWM_AUX7_MIN>)  _pwm_aux7_min,
        (ParamFloat<px4::params::PWM_AUX8_MIN>)  _pwm_aux8_min,

        (ParamFloat<px4::params::PWM_MAIN1_TRIM>) _pwm_main1_trim,
        (ParamFloat<px4::params::PWM_MAIN2_TRIM>) _pwm_main2_trim,
        (ParamFloat<px4::params::PWM_MAIN3_TRIM>) _pwm_main3_trim,
        (ParamFloat<px4::params::PWM_MAIN4_TRIM>) _pwm_main4_trim,
        (ParamFloat<px4::params::PWM_MAIN5_TRIM>) _pwm_main5_trim,
        (ParamFloat<px4::params::PWM_MAIN6_TRIM>) _pwm_main6_trim,
        (ParamFloat<px4::params::PWM_MAIN7_TRIM>) _pwm_main7_trim,
        (ParamFloat<px4::params::PWM_MAIN8_TRIM>) _pwm_main8_trim,
        (ParamFloat<px4::params::PWM_AUX1_TRIM>) _pwm_aux1_trim,
        (ParamFloat<px4::params::PWM_AUX2_TRIM>) _pwm_aux2_trim,
        (ParamFloat<px4::params::PWM_AUX3_TRIM>) _pwm_aux3_trim,
        (ParamFloat<px4::params::PWM_AUX4_TRIM>) _pwm_aux4_trim,
        (ParamFloat<px4::params::PWM_AUX5_TRIM>) _pwm_aux5_trim,
        (ParamFloat<px4::params::PWM_AUX6_TRIM>) _pwm_aux6_trim,
        (ParamFloat<px4::params::PWM_AUX7_TRIM>) _pwm_aux7_trim,
        (ParamFloat<px4::params::PWM_AUX8_TRIM>) _pwm_aux8_trim,

        (ParamFloat<px4::params::PWM_MAIN1_MAX_X>) _pwm_main1_max_x,
        (ParamFloat<px4::params::PWM_MAIN2_MAX_X>) _pwm_main2_max_x,
        (ParamFloat<px4::params::PWM_MAIN3_MAX_X>) _pwm_main3_max_x,
        (ParamFloat<px4::params::PWM_MAIN4_MAX_X>) _pwm_main4_max_x,
        (ParamFloat<px4::params::PWM_MAIN5_MAX_X>) _pwm_main5_max_x,
        (ParamFloat<px4::params::PWM_MAIN6_MAX_X>) _pwm_main6_max_x,
        (ParamFloat<px4::params::PWM_MAIN7_MAX_X>) _pwm_main7_max_x,
        (ParamFloat<px4::params::PWM_MAIN8_MAX_X>) _pwm_main8_max_x,
        (ParamFloat<px4::params::PWM_AUX1_MAX_X>)  _pwm_aux1_max_x,
        (ParamFloat<px4::params::PWM_AUX2_MAX_X>)  _pwm_aux2_max_x,
        (ParamFloat<px4::params::PWM_AUX3_MAX_X>)  _pwm_aux3_max_x,
        (ParamFloat<px4::params::PWM_AUX4_MAX_X>)  _pwm_aux4_max_x,
        (ParamFloat<px4::params::PWM_AUX5_MAX_X>)  _pwm_aux5_max_x,
        (ParamFloat<px4::params::PWM_AUX6_MAX_X>)  _pwm_aux6_max_x,
        (ParamFloat<px4::params::PWM_AUX7_MAX_X>)  _pwm_aux7_max_x,
        (ParamFloat<px4::params::PWM_AUX8_MAX_X>)  _pwm_aux8_max_x,

        (ParamFloat<px4::params::PWM_MAIN1_MIN_X>) _pwm_main1_min_x,
        (ParamFloat<px4::params::PWM_MAIN2_MIN_X>) _pwm_main2_min_x,
        (ParamFloat<px4::params::PWM_MAIN3_MIN_X>) _pwm_main3_min_x,
        (ParamFloat<px4::params::PWM_MAIN4_MIN_X>) _pwm_main4_min_x,
        (ParamFloat<px4::params::PWM_MAIN5_MIN_X>) _pwm_main5_min_x,
        (ParamFloat<px4::params::PWM_MAIN6_MIN_X>) _pwm_main6_min_x,
        (ParamFloat<px4::params::PWM_MAIN7_MIN_X>) _pwm_main7_min_x,
        (ParamFloat<px4::params::PWM_MAIN8_MIN_X>) _pwm_main8_min_x,
        (ParamFloat<px4::params::PWM_AUX1_MIN_X>)  _pwm_aux1_min_x,
        (ParamFloat<px4::params::PWM_AUX2_MIN_X>)  _pwm_aux2_min_x,
        (ParamFloat<px4::params::PWM_AUX3_MIN_X>)  _pwm_aux3_min_x,
        (ParamFloat<px4::params::PWM_AUX4_MIN_X>)  _pwm_aux4_min_x,
        (ParamFloat<px4::params::PWM_AUX5_MIN_X>)  _pwm_aux5_min_x,
        (ParamFloat<px4::params::PWM_AUX6_MIN_X>)  _pwm_aux6_min_x,
        (ParamFloat<px4::params::PWM_AUX7_MIN_X>)  _pwm_aux7_min_x,
        (ParamFloat<px4::params::PWM_AUX8_MIN_X>)  _pwm_aux8_min_x,

        (ParamFloat<px4::params::MIXER_THR_THRES>) _thr_alloc_threshold,
        (ParamFloat<px4::params::MIXER_THR_RUD_S>) _thr_rud_sc,
        (ParamFloat<px4::params::MIXER_THR_ELE_S>) _thr_ele_sc,

        (ParamFloat<px4::params::MIXER_D_THR_LIM>) _thr_diff_limit,
        (ParamFloat<px4::params::FW_THR_IDLE>) _param_fw_thr_idle,
        (ParamFloat<px4::params::FW_THR_MAX>) _param_fw_thr_max,

        (ParamInt<px4::params::MIXER_THR_KILL>) _thr_kill,
        (ParamFloat<px4::params::MIXER_THR_KILL_T>) _thr_kill_t,

        (ParamInt<px4::params::VT_ELEV_MC_LOCK>) _vt_elev_mc_lock,
        (ParamInt<px4::params::COM_VEHICLE_ID>) _vehicle_id

    );

//     uint16_t pwm_main1_max{0};
//     uint16_t pwm_main2_max{0};
//     uint16_t pwm_main3_max{0};
//     uint16_t pwm_main4_max{0};
//     uint16_t pwm_main5_max{0};
//     uint16_t pwm_main6_max{0};
//     uint16_t pwm_main7_max{0};
//     uint16_t pwm_main8_max{0};
//     uint16_t pwm_aux1_max{0};
//     uint16_t pwm_aux2_max{0};
//     uint16_t pwm_aux3_max{0};
//     uint16_t pwm_aux4_max{0};
//     uint16_t pwm_aux5_max{0};
//     uint16_t pwm_aux6_max{0};
//     uint16_t pwm_aux7_max{0};
//     uint16_t pwm_aux8_max{0};

//     uint16_t pwm_main1_min{0};
//     uint16_t pwm_main2_min{0};
//     uint16_t pwm_main3_min{0};
//     uint16_t pwm_main4_min{0};
//     uint16_t pwm_main5_min{0};
//     uint16_t pwm_main6_min{0};
//     uint16_t pwm_main7_min{0};
//     uint16_t pwm_main8_min{0};
//     uint16_t pwm_aux1_min{0};
//     uint16_t pwm_aux2_min{0};
//     uint16_t pwm_aux3_min{0};
//     uint16_t pwm_aux4_min{0};
//     uint16_t pwm_aux5_min{0};
//     uint16_t pwm_aux6_min{0};
//     uint16_t pwm_aux7_min{0};
//     uint16_t pwm_aux8_min{0};

//     uint16_t pwm_main1_trim{0};
//     uint16_t pwm_main2_trim{0};
//     uint16_t pwm_main3_trim{0};
//     uint16_t pwm_main4_trim{0};
//     uint16_t pwm_main5_trim{0};
//     uint16_t pwm_main6_trim{0};
//     uint16_t pwm_main7_trim{0};
//     uint16_t pwm_main8_trim{0};
//     uint16_t pwm_aux1_trim{0};
//     uint16_t pwm_aux2_trim{0};
//     uint16_t pwm_aux3_trim{0};
//     uint16_t pwm_aux4_trim{0};
//     uint16_t pwm_aux5_trim{0};
//     uint16_t pwm_aux6_trim{0};
//     uint16_t pwm_aux7_trim{0};
//     uint16_t pwm_aux8_trim{0};

//     float pwm_main1_max_x{0};
//     float pwm_main2_max_x{0};
//     float pwm_main3_max_x{0};
//     float pwm_main4_max_x{0};
//     float pwm_main5_max_x{0};
//     float pwm_main6_max_x{0};
//     float pwm_main7_max_x{0};
//     float pwm_main8_max_x{0};
//     float pwm_aux1_max_x{0};
//     float pwm_aux2_max_x{0};
//     float pwm_aux3_max_x{0};
//     float pwm_aux4_max_x{0};
//     float pwm_aux5_max_x{0};
//     float pwm_aux6_max_x{0};
//     float pwm_aux7_max_x{0};
//     float pwm_aux8_max_x{0};

//     float pwm_main1_min_x{0};
//     float pwm_main2_min_x{0};
//     float pwm_main3_min_x{0};
//     float pwm_main4_min_x{0};
//     float pwm_main5_min_x{0};
//     float pwm_main6_min_x{0};
//     float pwm_main7_min_x{0};
//     float pwm_main8_min_x{0};
//     float pwm_aux1_min_x{0};
//     float pwm_aux2_min_x{0};
//     float pwm_aux3_min_x{0};
//     float pwm_aux4_min_x{0};
//     float pwm_aux5_min_x{0};
//     float pwm_aux6_min_x{0};
//     float pwm_aux7_min_x{0};
//     float pwm_aux8_min_x{0};

     float thr_alloc_threshold = 0.0f;  // 当油门输入大于这个值时其它组才输出
     float thr_rud_sc = 0.2f;  // 油门差动辅助控制航向的比例, dd = thr_rud_sc * dr
     float thr_ele_sc = 0.2f;  // 油门差动辅助控制俯仰的比例, dd = thr_ele_sc * de

     float thr_diff_limit = 0.3f;  // 油门差动限幅
     float param_fw_thr_idle = 0.15f;
     float param_fw_thr_max = 1.f;

    uint32_t _VEHICLE_ID{0};

//    DEFINE_PARAMETERS(
//            (ParamFloat<px4::params::FW_THR_MAX>) _param_fw_thr_max,
//        (ParamInt<px4::params::MC_AIRMODE>) _param_mc_airmode,   ///< multicopter air-mode
//        (ParamFloat<px4::params::MOT_SLEW_MAX>) _param_mot_slew_max,
//        (ParamFloat<px4::params::THR_MDL_FAC>) _param_thr_mdl_fac, ///< thrust to motor control signal modelling factor
//        (ParamInt<px4::params::MOT_ORDERING>) _param_mot_ordering

//    );

};
