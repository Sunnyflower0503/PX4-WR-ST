/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

/**
* @file tailsitter.cpp
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author David Vorsin     <davidvorsin@gmail.com>
*
*/

#include "tailsitter.h"
#include "vtol_att_control_main.h"

#define PITCH_TRANSITION_BACK -0.25f	// pitch angle to switch to MC

using namespace matrix;

Tailsitter::Tailsitter(VtolAttitudeControl *attc) :
	VtolType(attc)
{
	_vtol_schedule.flight_mode = vtol_mode::FW_MODE;
	_vtol_schedule.transition_start = 0;
	_vtol_mode = mode::FIXED_WING;

	_mc_roll_weight = 0.0f;
	_mc_pitch_weight = 0.0f;
	_mc_yaw_weight = 0.0f;

	_flag_was_in_trans_mode = false;
	_params_handles_tailsitter.fw_pitch_sp_offset = param_find("FW_PSP_OFF");
	_params_handles_tailsitter.front_trans_pitch = param_find("VT_TS_TRANS_P");
	_params_handles_tailsitter.back_trans_airspeed_max = param_find("TD_BTR_ARSP");
	_params_handles_tailsitter.back_trans_roll_max = param_find("TD_BTR_ROLL");
	_params_handles_tailsitter.back_trans_pitch_max = param_find("TD_BTR_PITCH");
	_params_handles_tailsitter.back_trans_gate_time = param_find("TD_BTR_GATE_T");
	_params_handles_tailsitter.back_trans_throttle_max = param_find("TD_BTR_THR");
}

void
Tailsitter::parameters_update()
{
	float v;

	param_get(_params_handles_tailsitter.fw_pitch_sp_offset, &v);
	_params_tailsitter.fw_pitch_sp_offset = math::radians(v);

	param_get(_params_handles_tailsitter.front_trans_pitch, &v);
	_params_tailsitter.front_trans_pitch = v;

	param_get(_params_handles_tailsitter.back_trans_airspeed_max, &_params_tailsitter.back_trans_airspeed_max);
	param_get(_params_handles_tailsitter.back_trans_roll_max, &v);
	_params_tailsitter.back_trans_roll_max = math::radians(v);
	param_get(_params_handles_tailsitter.back_trans_pitch_max, &v);
	_params_tailsitter.back_trans_pitch_max = math::radians(v);
	param_get(_params_handles_tailsitter.back_trans_gate_time, &_params_tailsitter.back_trans_gate_time);
	param_get(_params_handles_tailsitter.back_trans_throttle_max, &_params_tailsitter.back_trans_throttle_max);
}

void Tailsitter::update_vtol_state()
{
	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting in MC control mode, picking up
	 * forward speed. After the vehicle has picked up enough and sufficient pitch angle the uav will go into FW mode.
	 * For the backtransition the pitch is controlled in MC mode again and switches to full MC control reaching the sufficient pitch angle.
	*/

	float pitch = Eulerf(Quatf(_v_att->q)).theta();

	if (_vtol_vehicle_status->vtol_transition_failsafe) {
		// Failsafe event, switch to MC mode immediately
		_vtol_schedule.flight_mode = vtol_mode::MC_MODE;

		//reset failsafe when FW is no longer requested
		if (!_attc->is_fixed_wing_requested()) {
			_vtol_vehicle_status->vtol_transition_failsafe = false;
		}

	} else if (!_attc->is_fixed_wing_requested()) {

		switch (_vtol_schedule.flight_mode) { // user switchig to MC mode
		case vtol_mode::MC_MODE:
			break;

		case vtol_mode::FW_MODE:
		{
			if (can_transition_on_ground()) {
				_vtol_schedule.flight_mode = vtol_mode::MC_MODE;
				_back_trans_gate_since = 0;
				_back_trans_wait_reported = false;
				_back_trans_requested_waiting = false;
				PX4_INFO("Back transition gate: ground/disarmed MC selection");
				break;
			}

			if (_params->tandem_backtrans_debug_fast) {
				_vtol_schedule.flight_mode = vtol_mode::TRANSITION_BACK;
				_vtol_schedule.transition_start = hrt_absolute_time();
				_back_trans_gate_since = 0;
				_back_trans_wait_reported = false;
				_back_trans_requested_waiting = false;
				PX4_WARN("Back transition gate: HITL debug fast FW->MC");
				break;
			}

			_back_trans_requested_waiting = true;
			const Eulerf attitude{Quatf(_v_att->q)};
			const float airspeed = _airspeed_validated->calibrated_airspeed_m_s;
			const bool airspeed_ok = PX4_ISFINITE(airspeed)
				&& (_params_tailsitter.back_trans_airspeed_max <= FLT_EPSILON
				    || airspeed <= _params_tailsitter.back_trans_airspeed_max);
			const bool attitude_ok = fabsf(attitude.phi()) <= _params_tailsitter.back_trans_roll_max
				&& fabsf(attitude.theta()) <= _params_tailsitter.back_trans_pitch_max;

			if (airspeed_ok && attitude_ok) {
				if (_back_trans_gate_since == 0) {
					_back_trans_gate_since = hrt_absolute_time();
					PX4_INFO("Back transition gate: stable, waiting %.1f s",
						 (double)_params_tailsitter.back_trans_gate_time);
				}

				const float gate_elapsed = (hrt_absolute_time() - _back_trans_gate_since) * 1e-6f;

				if (gate_elapsed >= _params_tailsitter.back_trans_gate_time) {
					_vtol_schedule.flight_mode = vtol_mode::TRANSITION_BACK;
					_vtol_schedule.transition_start = hrt_absolute_time();
					_back_trans_gate_since = 0;
					_back_trans_wait_reported = false;
					_back_trans_requested_waiting = false;
					PX4_INFO("Back transition gate: enabled at %.1f m/s", (double)airspeed);
				}

			} else {
				_back_trans_gate_since = 0;

				if (!_back_trans_wait_reported) {
					PX4_INFO("Back transition gate: waiting (V %.1f, roll %.1f, pitch %.1f)",
						 (double)airspeed, (double)math::degrees(attitude.phi()),
						 (double)math::degrees(attitude.theta()));
					_back_trans_wait_reported = true;
				}
			}
		}
			break;

		case vtol_mode::TRANSITION_FRONT_P1:
			// failsafe into multicopter mode
			_vtol_schedule.flight_mode = vtol_mode::MC_MODE;
			break;

		case vtol_mode::TRANSITION_BACK:
			float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;

			// check if we have reached pitch angle to switch to MC mode
			if (pitch >= PITCH_TRANSITION_BACK || time_since_trans_start > _params->back_trans_duration) {
				_vtol_schedule.flight_mode = vtol_mode::MC_MODE;
			}

			break;
		}

	} else {  // user switchig to FW mode
		_back_trans_gate_since = 0;
		_back_trans_wait_reported = false;
		_back_trans_requested_waiting = false;

		switch (_vtol_schedule.flight_mode) {
		case vtol_mode::MC_MODE:
			// initialise a front transition
			_vtol_schedule.flight_mode = vtol_mode::TRANSITION_FRONT_P1;
			_vtol_schedule.transition_start = hrt_absolute_time();
			break;

		case vtol_mode::FW_MODE:
			break;

		case vtol_mode::TRANSITION_FRONT_P1: {


				const bool airspeed_triggers_transition = PX4_ISFINITE(_airspeed_validated->calibrated_airspeed_m_s)
						&& !_params->airspeed_disabled;

				bool transition_to_fw = false;

				if (pitch <= _params_tailsitter.front_trans_pitch) {
					if (airspeed_triggers_transition) {
						transition_to_fw = _airspeed_validated->calibrated_airspeed_m_s >= _params->transition_airspeed;

					} else {
						transition_to_fw = true;
					}
				}

				transition_to_fw |= can_transition_on_ground();

				if (transition_to_fw) {
					_vtol_schedule.flight_mode = vtol_mode::FW_MODE;
				}

				break;
			}

		case vtol_mode::TRANSITION_BACK:
			// failsafe into fixed wing mode
			_vtol_schedule.flight_mode = vtol_mode::FW_MODE;
			break;
		}
	}

	// map tailsitter specific control phases to simple control modes
	switch (_vtol_schedule.flight_mode) {
	case vtol_mode::MC_MODE:
		_vtol_mode = mode::ROTARY_WING;
		_vtol_vehicle_status->vtol_in_trans_mode = false;
		_flag_was_in_trans_mode = false;
		break;

	case vtol_mode::FW_MODE:
		_vtol_mode = mode::FIXED_WING;
		_vtol_vehicle_status->vtol_in_trans_mode = false;
		_flag_was_in_trans_mode = false;
		break;

	case vtol_mode::TRANSITION_FRONT_P1:
		_vtol_mode = mode::TRANSITION_TO_FW;
		_vtol_vehicle_status->vtol_in_trans_mode = true;
		break;

	case vtol_mode::TRANSITION_BACK:
		_vtol_mode = mode::TRANSITION_TO_MC;
		_vtol_vehicle_status->vtol_in_trans_mode = true;
		break;
	}
}

void Tailsitter::update_transition_state()
{
	const float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;

	if (!_flag_was_in_trans_mode) {
		_flag_was_in_trans_mode = true;

		if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_BACK) {
			// calculate rotation axis for transition.
			_q_trans_start = Quatf(_v_att->q);
			Vector3f z = -_q_trans_start.dcm_z();
			_trans_rot_axis = z.cross(Vector3f(0, 0, -1));

			// as heading setpoint we choose the heading given by the direction the vehicle points
			float yaw_sp = atan2f(z(1), z(0));

			// the intial attitude setpoint for a backtransition is a combination of the current fw pitch setpoint,
			// the yaw setpoint and zero roll since we want wings level transition
			_q_trans_start = Eulerf(0.0f, _fw_virtual_att_sp->pitch_body, yaw_sp);

			// attitude during transitions are controlled by mc attitude control so rotate the desired attitude to the
			// multirotor frame
			_q_trans_start = _q_trans_start * Quatf(Eulerf(0, -M_PI_2_F, 0));

		} else if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_FRONT_P1) {
			// initial attitude setpoint for the transition should be with wings level
			_q_trans_start = Eulerf(0.0f, _mc_virtual_att_sp->pitch_body, _mc_virtual_att_sp->yaw_body);
			Vector3f x = Dcmf(Quatf(_v_att->q)) * Vector3f(1, 0, 0);
			_trans_rot_axis = -x.cross(Vector3f(0, 0, -1));
		}

		_q_trans_sp = _q_trans_start;
	}

	// ensure input quaternions are exactly normalized because acosf(1.00001) == NaN
	_q_trans_sp.normalize();

	// tilt angle (zero if vehicle nose points up (hover))
	float cos_tilt = _q_trans_sp(0) * _q_trans_sp(0) - _q_trans_sp(1) * _q_trans_sp(1) - _q_trans_sp(2) *
			 _q_trans_sp(2) + _q_trans_sp(3) * _q_trans_sp(3);
	cos_tilt = cos_tilt >  1.0f ?  1.0f : cos_tilt;
	cos_tilt = cos_tilt < -1.0f ? -1.0f : cos_tilt;
	const float tilt = acosf(cos_tilt);

	if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_FRONT_P1) {

		const float trans_pitch_rate = M_PI_2_F / _params->front_trans_duration;

		if (tilt < M_PI_2_F - _params_tailsitter.fw_pitch_sp_offset) {
			_q_trans_sp = Quatf(AxisAnglef(_trans_rot_axis,
						       time_since_trans_start * trans_pitch_rate)) * _q_trans_start;
		}

	} else if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_BACK) {

		const float trans_pitch_rate = M_PI_2_F / _params->back_trans_duration;

		if (!_flag_idle_mc) {
			_flag_idle_mc = set_idle_mc();
		}

		if (tilt > 0.01f) {
			_q_trans_sp = Quatf(AxisAnglef(_trans_rot_axis,
						       time_since_trans_start * trans_pitch_rate)) * _q_trans_start;
		}
	}

	_v_att_sp->thrust_body[2] = _mc_virtual_att_sp->thrust_body[2];

	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;

	_v_att_sp->timestamp = hrt_absolute_time();

	const Eulerf euler_sp(_q_trans_sp);
	_v_att_sp->roll_body = euler_sp.phi();
	_v_att_sp->pitch_body = euler_sp.theta();
	_v_att_sp->yaw_body = euler_sp.psi();

	_q_trans_sp.copyTo(_v_att_sp->q_d);
}

void Tailsitter::waiting_on_tecs()
{
	// copy the last trust value from the front transition
	_v_att_sp->thrust_body[0] = _thrust_transition;
}

void Tailsitter::update_fw_state()
{
	VtolType::update_fw_state();

}

/**
* Write data to actuator output topic.
*/
void Tailsitter::fill_actuator_outputs()
{
	auto &mc_in = _actuators_mc_in->control;
	auto &fw_in = _actuators_fw_in->control;

	auto &mc_out = _actuators_out_0->control;
	auto &fw_out = _actuators_out_1->control;

	mc_out[actuator_controls_s::INDEX_ROLL]  = mc_in[actuator_controls_s::INDEX_ROLL]  * _mc_roll_weight;
	mc_out[actuator_controls_s::INDEX_PITCH] = mc_in[actuator_controls_s::INDEX_PITCH] * _mc_pitch_weight;
	mc_out[actuator_controls_s::INDEX_YAW]   = mc_in[actuator_controls_s::INDEX_YAW]   * _mc_yaw_weight;

	if (_vtol_schedule.flight_mode == vtol_mode::FW_MODE) {
		const float fw_throttle = _back_trans_requested_waiting ?
			math::min(fw_in[actuator_controls_s::INDEX_THROTTLE], _params_tailsitter.back_trans_throttle_max) :
			fw_in[actuator_controls_s::INDEX_THROTTLE];
		mc_out[actuator_controls_s::INDEX_THROTTLE] = fw_throttle;
		mc_out[actuator_controls_s::INDEX_PITCH] = fw_in[actuator_controls_s::INDEX_PITCH] * _params->diff_thrust_scale;

		/* allow differential thrust if enabled */
		if (_params->diff_thrust == 1) {
			mc_out[actuator_controls_s::INDEX_ROLL] = fw_in[actuator_controls_s::INDEX_YAW] * _params->diff_thrust_scale;
		}

	} else {
		mc_out[actuator_controls_s::INDEX_THROTTLE] = mc_in[actuator_controls_s::INDEX_THROTTLE];
	}

	if (_params->elevons_mc_lock && _vtol_schedule.flight_mode == vtol_mode::MC_MODE) {
		fw_out[actuator_controls_s::INDEX_ROLL]  = 0;
		fw_out[actuator_controls_s::INDEX_PITCH] = 0;

	} else {
		fw_out[actuator_controls_s::INDEX_ROLL]  = fw_in[actuator_controls_s::INDEX_ROLL];
		fw_out[actuator_controls_s::INDEX_PITCH] = fw_in[actuator_controls_s::INDEX_PITCH];
		fw_out[actuator_controls_s::INDEX_YAW]   = fw_in[actuator_controls_s::INDEX_YAW];
	}

	_actuators_out_0->timestamp_sample = _actuators_mc_in->timestamp_sample;
	_actuators_out_1->timestamp_sample = _actuators_fw_in->timestamp_sample;

	_actuators_out_0->timestamp = _actuators_out_1->timestamp = hrt_absolute_time();
}
