/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef ATTITUDE_QUATERNION_HPP
#define ATTITUDE_QUATERNION_HPP

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vtol_vehicle_status.h>

class MavlinkStreamAttitudeQuaternion : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamAttitudeQuaternion(mavlink); }

	static constexpr const char *get_name_static() { return "ATTITUDE_QUATERNION"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ATTITUDE_QUATERNION; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _att_sub.advertised() ? MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamAttitudeQuaternion(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vtol_status_sub{ORB_ID(vtol_vehicle_status)};

	bool send() override
	{
		vehicle_attitude_s att;

		if (_att_sub.update(&att)) {
			vehicle_angular_velocity_s angular_velocity{};
			_angular_velocity_sub.copy(&angular_velocity);

			vehicle_status_s status{};
			_status_sub.copy(&status);
			vtol_vehicle_status_s vtol_status{};
			_vtol_status_sub.copy(&vtol_status);

			mavlink_attitude_quaternion_t msg{};
			matrix::Quatf q{att.q};

			// QGC needs the MC frame for a tailsitter in rotary-wing mode.
			// Keep vehicle_attitude itself unchanged for EKF and controllers.
			if (status.is_vtol_tailsitter && vtol_status.vtol_in_rw_mode) {
				q = q * matrix::Quatf(matrix::Eulerf(0.0f, -M_PI_2_F, 0.0f));
			}

			msg.time_boot_ms = att.timestamp / 1000;
			msg.q1 = q(0);
			msg.q2 = q(1);
			msg.q3 = q(2);
			msg.q4 = q(3);
			msg.rollspeed = angular_velocity.xyz[0];
			msg.pitchspeed = angular_velocity.xyz[1];
			msg.yawspeed = angular_velocity.xyz[2];

			// The quaternion above is a display-only representation. Keep the
			// representation offset empty so the GCS does not apply another rotation.
			msg.repr_offset_q[0] = 0.0f;
			msg.repr_offset_q[1] = 0.0f;
			msg.repr_offset_q[2] = 0.0f;
			msg.repr_offset_q[3] = 0.0f;

			mavlink_msg_attitude_quaternion_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // ATTITUDE_QUATERNION_HPP
