/****************************************************************************
 *
 *   Copyright (c) 2013-2020 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file ecl_pitch_controller.cpp
 * Implementation of a simple orthogonal pitch PID controller.
 *
 * Authors and acknowledgements in header.
 */

#include "ecl_pitch_controller.h"
#include <float.h>
#include <lib/ecl/geo/geo.h>
#include <mathlib/mathlib.h>

using namespace matrix;

float ECL_PitchController::control_attitude(const float dt, const ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.pitch_setpoint) &&
	      PX4_ISFINITE(ctl_data.roll) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.airspeed))) {

		return _rate_setpoint;
	}

	/* Calculate the error */
	float pitch_error = ctl_data.pitch_setpoint - ctl_data.pitch;

	/*  Apply P controller: rate setpoint from current error and time constant */
	_rate_setpoint =  pitch_error / _tc;

	return _rate_setpoint;
}

float ECL_PitchController::control_bodyrate(const float dt, const ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.roll) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.body_y_rate) &&
	      PX4_ISFINITE(ctl_data.body_z_rate) &&
	      PX4_ISFINITE(ctl_data.yaw_rate_setpoint) &&
	      PX4_ISFINITE(ctl_data.airspeed_min) &&
	      PX4_ISFINITE(ctl_data.airspeed_max) &&
	      PX4_ISFINITE(ctl_data.scaler))) {

		return math::constrain(_last_output, -1.0f, 1.0f);
	}

	/* Calculate body angular rate error */
	_rate_error = _bodyrate_setpoint - ctl_data.body_y_rate;

	if (!ctl_data.lock_integrator && _k_i > 0.0f) {

		/* Integral term scales with 1/IAS^2 */
		float id = _rate_error * dt * ctl_data.scaler * ctl_data.scaler;

		/*
		 * anti-windup: do not allow integrator to increase if actuator is at limit
		 */
		if (_last_output < -1.0f) {
			/* only allow motion to center: increase value */
			id = math::max(id, 0.0f);

		} else if (_last_output > 1.0f) {
			/* only allow motion to center: decrease value */
			id = math::min(id, 0.0f);
		}

		/* add and constrain */
		_integrator = math::constrain(_integrator + id * _k_i, -_integrator_max, _integrator_max);
	}

	/* Apply PI rate controller and store non-limited output */
	/* FF terms scales with 1/TAS and P,I with 1/IAS^2 */
	_last_output = _bodyrate_setpoint * _k_ff * ctl_data.scaler +
		       _rate_error * _k_p * ctl_data.scaler * ctl_data.scaler
		       + _integrator;

	return math::constrain(_last_output, -1.0f, 1.0f);
}

float ECL_PitchController::control_euler_rate(const float dt, const ECL_ControlData &ctl_data)
{
	/* Transform setpoint to body angular rates (jacobian) */
	_bodyrate_setpoint = cosf(ctl_data.roll) * _rate_setpoint +
			     cosf(ctl_data.pitch) * sinf(ctl_data.roll) * ctl_data.yaw_rate_setpoint;

	set_bodyrate_setpoint(_bodyrate_setpoint);

	return control_bodyrate(dt, ctl_data);
}


// 采用INDI控制
float ECL_PitchController::control_euler_rate_INDI(const float dt, const ECL_ControlData &ctl_data)
{
	/* Transform setpoint to body angular rates (jacobian) */
	_bodyrate_setpoint = cosf(ctl_data.roll) * _rate_setpoint +
			     cosf(ctl_data.pitch) * sinf(ctl_data.roll) * ctl_data.yaw_rate_setpoint;

	set_bodyrate_setpoint(_bodyrate_setpoint);

	return control_bodyrate_INDI(dt, ctl_data);
}

float ECL_PitchController::control_bodyrate_INDI(const float dt, const ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.roll) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.body_y_rate) &&
	      PX4_ISFINITE(ctl_data.body_z_rate) &&
	      PX4_ISFINITE(ctl_data.yaw_rate_setpoint) &&
	      PX4_ISFINITE(ctl_data.airspeed_min) &&
	      PX4_ISFINITE(ctl_data.airspeed_max) &&
	      PX4_ISFINITE(ctl_data.scaler))) {

		return math::constrain(_last_output, -1.0f, 1.0f);
	}

	/* Calculate body angular rate error */
	_rate_error = _bodyrate_setpoint - ctl_data.body_y_rate;

	// Euler法求解ESO
	// // float control_matrix = _INDI_B/(ctl_data.scaler*ctl_data.scaler);	// MbarDe = ( CmDe*Q0*Sw*CA ) / Iy; scaler=V0/V
	// float control_matrix = _INDI_B/(ctl_data.scaler);	// MbarDe = ( CmDe*Q0*Sw*CA ) / Iy; scaler=V0/V
	// float e = _INDI_z1-ctl_data.body_y_rate;
	// float z1_dot = _INDI_z2+control_matrix*_last_output-2.f*_INDI_omega*e;
	// float z2_dot = -_INDI_omega*_INDI_omega*e;
	// _INDI_z1 = _INDI_z1 + z1_dot*dt;
	// _INDI_z2 = _INDI_z2 + z2_dot*dt;

	// if( _count%300 == 0 ) {
	// 	log_i("dt = %.3f, B = %.1f, omega = %.1f", dt, _INDI_B, _INDI_omega);
	// 	log_i("z1_dot = %.2f, z2_dot = %.2f, z1 = %.2f, z2 = %.2f", z1_dot, z2_dot, _INDI_z1, _INDI_z2);
	// 	log_i("err = %.2f, q_sp = %.2f, q = %.2f, du = %.2f, u = %.2f",_rate_error, _bodyrate_setpoint, ctl_data.body_y_rate, du, _last_output);

	// }
	// _count++;

	// RK4法求解ESO
	// float x0_dot = z1_dot;

	// float B_scale = math::gradual3(_last_output, -0.2f, 0.f, 0.2f,    0.5f, 1.f, 1.5f);	// (x, x_low,x_hi, y_low,y_hi), 根据X-Wing的MDt曲线来给
	float control_matrix = _INDI_B/(ctl_data.scaler);	// MbarDe = ( CmDe*Q0*Sw*CA ) / Iy; scaler=V0/V
	// float control_matrix = _INDI_B;	// MbarDe = ( CmDe*Q0*Sw*CA ) / Iy; scaler=V0/V
	// float control_matrix = _INDI_B*B_scale;	// MbarDe = ( CmDe*Q0*Sw*CA ) / Iy; scaler=V0/V


	// Vector2f u = Vector2f(ctl_data.body_y_rate, _last_output);
	Vector2f u = Vector2f(ctl_data.body_y_rate, _last_output/ctl_data.scaler);	// _last_output除以ctl_data.scaler是为了把ESO方程中的B(或者说u(1))也转换为control_matrix
	// Vector2f u = Vector2f(ctl_data.body_y_rate, _last_output*B_scale);	// _last_output除以ctl_data.scaler是为了把ESO方程中的B(或者说u(1))也转换为control_matrix
	float x0_dot = get_x_dot(u, dt);

	// float x_err = x_sp - x;
	float du = (_INDI_kp_rate*_rate_error - x0_dot)/control_matrix;
	du = PX4_ISFINITE(du) ? du : 0.0f;
	_last_output = (_last_output+du);

	// if( _count%50 == 0 ) {
	// 	// log_i("dt = %.3f, B = %.1f, omega = %.1f", dt, _INDI_B, _INDI_omega);
	// 	// log_i("fw_att, dt = %.3f", dt);	// dt = 0.005
	// 	// log_i("fw_att, q = %.2f, q_hat = %.2f", ctl_data.body_y_rate*57.3f, _INDI_state(0)*57.3f);
	// 	// log_i("err = %.2f, q_sp = %.2f, q = %.2f, du = %.2f, u = %.2f",_rate_error, _bodyrate_setpoint, ctl_data.body_y_rate, du, _last_output);

	// 	log_i("%.2f, %.2f", _last_output, B_scale);

	// }
	// _count++;

	return math::constrain(-_last_output, -1.0f, 1.0f);
}
