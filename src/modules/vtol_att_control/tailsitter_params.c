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
 * @file tailsitter_params.c
 * Parameters for vtol attitude controller.
 *
 * @author Roman Bapst <bapstroman@gmail.com>
 * @author David Vorsin     <davidvorsin@gmail.com>
 */

/**
 * Tailsitter front transition pitch threshold
 *
 * Pitch angle at which the tailsitter front transition can complete and switch
 * to fixed-wing mode. This matches the previous hard-coded threshold by default.
 *
 * @unit rad
 * @min -1.57
 * @max 1.57
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TS_TRANS_P, -1.1f);

/** Maximum airspeed for tailsitter back-transition entry.
 * @unit m/s
 * @min 0
 * @max 30
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(TD_BTR_ARSP, 12.0f);

/** Maximum absolute fixed-wing roll angle for back-transition entry.
 * @unit deg
 * @min 1
 * @max 45
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(TD_BTR_ROLL, 10.0f);

/** Maximum absolute fixed-wing pitch angle for back-transition entry.
 * @unit deg
 * @min 1
 * @max 45
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(TD_BTR_PITCH, 15.0f);

/** Continuous stable time required before entering a back transition.
 * @unit s
 * @min 0
 * @max 5
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(TD_BTR_GATE_T, 1.0f);

/**
 * Back-transition waiting throttle limit
 *
 * The fixed-wing attitude loop remains active while throttle is limited so
 * the aircraft can decelerate to TD_BTR_ARSP before pitching into hover.
 *
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(TD_BTR_THR, 0.35f);

/**
 * Duration of front transition phase 2
 *
 * Time in seconds it should take for the rotors to rotate forward completely from the point
 * when the plane has picked up enough airspeed and is ready to go into fixed wind mode.
 *
 * @unit s
 * @min 0.1
 * @max 5.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control

PARAM_DEFINE_FLOAT(VT_TRANS_P2_DUR, 0.5f);*/
